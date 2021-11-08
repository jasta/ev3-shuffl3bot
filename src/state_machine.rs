use std::any::TypeId;
use std::cmp::min;
use std::collections::{HashMap, HashSet, VecDeque};
use std::convert::{TryFrom, TryInto};
use std::fmt::{Debug, Formatter, Pointer};
use std::marker::PhantomData;
use std::sync::Arc;
use thiserror::Error;
use tokio::sync::mpsc::{Receiver, Sender};
use tokio::sync::Mutex;
use tokio::task::{JoinError, JoinHandle, spawn_blocking};
use tokio::time::{Duration, Instant};

pub trait StateMachineDescriptor {
  type Context: Default;
  type Event: Debug;

  fn debug_name() -> &'static str;
  fn states() -> StateGraph<Self::Context, Self::Event>;
  fn new_context() -> Self::Context { Self::Context::default() }
}

pub trait State {
  type Context;
  type Event: Debug;

  fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {}
  fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {}

  #[must_use]
  fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event>;
}

pub struct Context<C, E> {
  pub user: C,
  tx: Sender<InternalEvent<E>>,
}

impl<C, E> Context<C, E> {
  pub fn dispatcher(&self) -> Dispatcher<E> {
    Dispatcher { tx: self.tx.clone() }
  }
}

pub struct Dispatcher<E> {
  tx: Sender<InternalEvent<E>>,
}

impl<E: Send + 'static> Dispatcher<E> {
  pub async fn dispatch(&self, event: E) {
    self.tx.send(InternalEvent::UserEvent(event, HANDLE_BY_SUBSTATE)).await;
  }

  pub fn dispatch_sync(&self, event: E) {
    self.dispatch_delay(event, Duration::from_millis(0));
  }

  pub fn dispatch_delay_ms(&self, event: E, delay_ms: u64) {
    self.dispatch_delay(event, Duration::from_millis(delay_ms));
  }

  pub fn dispatch_delay(&self, event: E, delay: Duration) {
    let clone_self = Dispatcher { tx: self.tx.clone() };
    tokio::spawn(async move {
      tokio::time::sleep(delay).await;
      clone_self.dispatch(event).await;
    });
  }

  pub fn dispatch_at_time(&self, event: E, deadline: Instant) {
    let clone_self = Dispatcher { tx: self.tx.clone() };
    tokio::spawn(async move {
      tokio::time::sleep_until(deadline).await;
      clone_self.dispatch(event).await;
    });
  }
}

pub type HandleResult<E> = Result<Transition, NotHandled<E>>;

pub enum Transition {
  MoveTo(TypeId),
  None,
}

#[derive(Error, Debug)]
pub enum NotHandled<E> {
  #[error("unknown event")]
  UnknownEvent(E),

  #[error("internal error: {0}")]
  InternalError(anyhow::Error),

  /// Defer an event until the next state transition occurs.  This can be useful when you
  /// aren't ready to handle a particular message yet but know that you're actively working
  /// on getting to a state where you can (which it will then be re-introduced to the queue and
  /// handled in that state normally).
  #[error("intentionally deferred")]
  DeferEvent(E),
}

pub struct StateMachine<C, E> {
  tx: Sender<InternalEvent<E>>,
  join_handle: JoinHandle<C>,
}

impl<C: Send + 'static, E: Debug + Send + 'static> StateMachine<C, E> {
  pub fn start<D: StateMachineDescriptor<Context = C, Event = E>>() -> Self {
    let (tx, rx) = tokio::sync::mpsc::channel(32);
    let tx_inner = tx.clone();
    let join_handle = tokio::spawn(async move {
      let context = Context {
        user: D::new_context(),
        tx: tx_inner,
      };
      let dispatcher = context.dispatcher();
      let states = D::states();
      let initial_state_id = states.initial_state.unwrap();
      let mut internal = StateMachineInternal {
        debug_name: D::debug_name(),
        context,
        dispatcher,
        states,
        current_states: vec![],
        rx,
        front_of_queue_events: VecDeque::new(),
        deferred_events: VecDeque::new(),
      };
      internal.enter_initial_states(initial_state_id);
      internal.handle_events().await;

      // Give the context back to the caller on shutdown so they can inspect the results.  Mostly
      // this is useful for testing though.
      internal.context.user
    });
    Self {
      tx,
      join_handle,
    }
  }

  pub async fn shutdown(self) -> Result<C, JoinError> {
    self.tx.send(InternalEvent::Shutdown).await;
    self.join_handle.await
  }

  pub fn dispatcher(&self) -> Dispatcher<E> {
    Dispatcher { tx: self.tx.clone() }
  }
}

struct StateMachineInternal<C, E> {
  debug_name: &'static str,
  context: Context<C, E>,
  dispatcher: Dispatcher<E>,
  states: StateGraph<C, E>,

  /// Stack of states with the last pushed state being the substate with no children.
  current_states: Vec<StateInstance<C, E>>,

  rx: Receiver<InternalEvent<E>>,

  /// Special mechanism to deliver events that will always be processed before dispatched
  /// user events.  This lets us use the message queue design to handle even our own internal
  /// actions like changing states in a way that's no longer re-entrant.
  front_of_queue_events: VecDeque<InternalEvent<E>>,

  /// Events that were marked by the user that they should be deferred until after the next
  /// state transition.
  deferred_events: VecDeque<E>,
}

impl<C, E: Debug> StateMachineInternal<C, E> {
  async fn handle_events(&mut self) {
    while let Some(event) = self.take_next_event().await {
      self.handle_event(event).await;
    }
    println!("{}: No longer handling events", self.debug_name);
  }

  async fn take_next_event(&mut self) -> Option<InternalEvent<E>> {
    let front = self.front_of_queue_events.pop_front();
    if front.is_some() {
      return front;
    }
    return self.rx.recv().await;
  }

  async fn handle_event(&mut self, event: InternalEvent<E>) {
    match event {
      InternalEvent::UserEvent(event, state_index) => self.handle_user_event(event, state_index),
      InternalEvent::Shutdown => self.handle_shutdown(),
      InternalEvent::MoveTo(state_type_id) => self.handle_move_to(state_type_id),
    };
  }

  fn handle_user_event(&mut self, event: E, state_index: isize) {
    let event_str = format!("{:?}", event);

    let (real_state_index, receive_prefix, receive_label) = match state_index < 0 {
      true => (self.current_states.len() - 1, "", "Received"),
      false => (usize::try_from(state_index).unwrap(), "...", "Trying to handle"),
    };
    let handling_state = self.current_states.get_mut(real_state_index).unwrap();
    let handling_state_str = handling_state.debug_name;
    println!(
      "{}: {}[{}] {} [{:?}]",
      self.debug_name,
      receive_prefix,
      handling_state_str,
      receive_label,
      event_str);
    let result = handling_state.instance.handle(&mut self.context, event);
    match result {
      Ok(action) => {
        match action {
          Transition::None => (),
          Transition::MoveTo(next_type_id) => {
            self.front_of_queue_events.push_front(InternalEvent::MoveTo(next_type_id));
          },
        }
      },
      Err(reason) => {
        match reason {
          NotHandled::UnknownEvent(event) => {
            let next_handling_index = match state_index {
              HANDLE_BY_SUBSTATE => isize::try_from(self.current_states.len()).unwrap() - 2,
              _ => state_index - 1,
            };
            if next_handling_index >= 0 {
              self.front_of_queue_events.push_front(InternalEvent::UserEvent(event, next_handling_index));
            } else {
              println!("{}: Unhandled [{}]!!!", self.debug_name, event_str);
            }
          },
          NotHandled::InternalError(err) => {
            println!("{}: Internal error handling [{}]: {}", self.debug_name, event_str, err);
          },
          NotHandled::DeferEvent(event) => {
            self.deferred_events.push_back(event);
          }
        }
      }
    }
  }

  fn handle_shutdown(&mut self) {
    println!("{}: Received shutdown signal...", self.debug_name);
    self.rx.close();
  }

  fn handle_move_to(&mut self, next_type_id: TypeId) {
    let state_changes = self.compute_state_changes(next_type_id);
    self.ensure_valid_transitions(&state_changes);

    println!(
      "{}: [{}] => [{}]",
      self.debug_name,
      state_changes.exit_child_name.unwrap(),
      state_changes.enter_child_name);

    self.apply_state_changes(state_changes);

    self.enqueue_deferred_events();
  }

  fn enter_initial_states(&mut self, initial_type_id: TypeId) {
    let state_changes = self.compute_state_changes(initial_type_id);
    self.apply_state_changes(state_changes);
  }

  fn compute_state_changes(&self, child_target_type_id: TypeId) -> EntersAndExits {
    let target_state_ids = self.collect_target_state_ids(child_target_type_id);
    let current_state_ids: Vec<TypeId> = self.current_states.iter().map(
      |s| s.type_id).collect();

    let index_of_differences = find_index_of_difference(&current_state_ids, &target_state_ids);

    let all_enters: Vec<TypeId> = target_state_ids[index_of_differences..].iter().copied().collect();
    let all_exits: Vec<TypeId> = current_state_ids[index_of_differences..].iter().copied().collect();

    let enter_child_id = all_enters.last().copied().expect("Must have at least one enter");
    let exit_child_id = all_exits.last().copied();

    let enter_child_name = self.states.lookup(enter_child_id).debug_name;
    let exit_child_name = exit_child_id.map(|t| self.states.lookup(t).debug_name);

    EntersAndExits {
      index_of_differences,
      exit_child_name,
      enter_child_name,
      all_enters,
      all_exits,
    }
  }

  fn ensure_valid_transitions(&self, state_changes: &EntersAndExits) {
    let all_valid_targets: Vec<TypeId> = state_changes.all_exits
        .iter()
        // Find just the exit transitions
        .flat_map(|&t| self.states.lookup(t).transitions_to.iter())
        // Then expand them with all parent states
        .flat_map(|&t| self.collect_target_state_ids(t))
        .collect();

    let invalid_target_ids: Vec<TypeId> = state_changes.all_enters
        .iter()
        .filter(|t| !all_valid_targets.contains(t))
        .copied()
        .collect();
    if !invalid_target_ids.is_empty() {
      let invalid_target_names: Vec<&str> = invalid_target_ids
          .iter()
          .map(|&t| self.states.lookup(t).debug_name)
          .collect();
      panic!(
        "{}: [{}] => [{}]: Included invalid transitions to: {:?}!",
        self.debug_name,
        state_changes.exit_child_name.unwrap(),
        state_changes.enter_child_name,
        invalid_target_names);
    }
  }

  fn apply_state_changes(&mut self, state_changes: EntersAndExits) {
    let index_of_diff = state_changes.index_of_differences;

    // Exit in reverse order so that child-most states are exited first.  The reason this is
    // backwards from exit is to keep things feeling symmetrically balanced so we enter as
    // P0, P1, C0 and exit as C0, P1, P0.
    for state in self.current_states[index_of_diff..].iter_mut().rev() {
      println!("{}: Exiting [{}]...", self.debug_name, state.debug_name);
      state.instance.on_exit(&mut self.context);
    }
    self.current_states.truncate(index_of_diff);

    // Push the states first.
    for &target_id in &state_changes.all_enters {
      let state = self.states.execute_factory(target_id);
      self.current_states.push(state);
    }

    // Then enter them, parents first.
    for state in &mut self.current_states[index_of_diff..] {
      println!("{}: Entering [{}]...", self.debug_name, state.debug_name);
      state.instance.on_enter(&mut self.context);
    }
  }

  fn collect_target_state_ids(&self, target_state_id: TypeId) -> Vec<TypeId> {
    let mut target_state_ids = Vec::<TypeId>::new();
    let mut next_possible = Some(target_state_id);
    while let Some(next) = next_possible {
      let node = self.states.lookup(next);
      target_state_ids.push(node.type_id);
      next_possible = node.parent_type_id;
    }
    target_state_ids.reverse();
    return target_state_ids;
  }

  fn enqueue_deferred_events(&mut self) {
    for deferred_event in self.deferred_events.drain(..) {
      self.front_of_queue_events.push_back(
        InternalEvent::UserEvent(deferred_event, HANDLE_BY_SUBSTATE));
    }
  }
}

struct EntersAndExits {
  index_of_differences: usize,
  enter_child_name: &'static str,
  all_enters: Vec<TypeId>,
  exit_child_name: Option<&'static str>,
  all_exits: Vec<TypeId>,
}

/// Compute the index at which two vecs begin to differ.  For example, given
/// a=[ 1, 2, 3, 4, 5 ] and b=[ 1, 2, 6, 7 ], we would return 2 because it is the lowest
/// value for which a[i] != b[i].  If the two vecs are equal, the return value will be the
/// length of either vec.
fn find_index_of_difference<T: PartialEq>(a: &Vec<T>, b: &Vec<T>) -> usize {
  if a.len() == 0 {
    return 0;
  }
  let n = min(a.len(), b.len());
  for i in 0..n {
    if a.get(i) != b.get(i) {
      return i;
    }
  }
  return n;
}

const HANDLE_BY_SUBSTATE: isize = -1;

enum InternalEvent<E> {
  /// Move to the state denoted by the provided TypeId.
  MoveTo(TypeId),

  /// Handle a user-provided event at the index specified in current_states.  This value should
  /// always start at [HANDLE_BY_SUBSTATE], but logically decrements until it reaches the 0th
  /// item in the stack of states, i.e., the super-most state for which all other current states
  /// are children.
  UserEvent(E, isize),

  /// Shutdown the state machine; no longer handle messages after this one is received (which is
  /// still governed by FIFO ordering).
  Shutdown,
}

pub struct StateTreeBuilder<C, E> {
  tree: StateGraph<C, E>,
  parent_type_ids: Vec<TypeId>,
  last_type_id: Option<TypeId>,
}

impl<C, E> StateTreeBuilder<C, E> {
  pub fn new() -> Self {
    Self {
      tree: StateGraph::new(),
      parent_type_ids: vec![],
      last_type_id: None
    }
  }

  pub fn add_parent<S>(mut self, add_children: fn(Self) -> Self) -> Self
      where S: State<Context = C, Event = E> + 'static + Default + Send {
    let parent_type_id = self.add_state_internal::<S>();
    self.parent_type_ids.push(parent_type_id);
    let mut moved_self = add_children(self);
    moved_self.parent_type_ids.pop();
    moved_self.last_type_id = Some(parent_type_id);
    moved_self
  }

  pub fn add<S>(mut self) -> Self
      where S: State<Context = C, Event = E> + 'static + Default + Send {
    self.add_state_internal::<S>();
    self
  }

  fn add_state_internal<S>(&mut self) -> TypeId
      where S: State<Context = C, Event = E> + 'static + Default + Send {
    let state_id = TypeId::of::<S>();
    let full_name = std::any::type_name::<S>();
    let short_name = full_name.split("::").last().unwrap();
    self.tree.add_state(
      state_id,
      short_name,
      self.parent_type_ids.last().copied(),
      || Box::new(S::default())
    );
    self.last_type_id = Some(state_id);
    return state_id;
  }

  pub fn transitions_to<S: State<Context = C, Event = E>>(mut self) -> Self
      where S: 'static {
    let defining_state = self.last_type_id.unwrap();
    self.tree.add_transition(defining_state, TypeId::of::<S>());
    self
  }

  pub fn initial(mut self) -> Self {
    self.tree.set_initial(self.last_type_id.unwrap());
    self
  }

  pub fn build(self) -> StateGraph<C, E> {
    // TODO: validate transitions make sense!
    self.tree.initial_state.expect("Must invoke .initial_state()!");
    self.tree
  }
}

pub struct StateGraph<C, E> {
  nodes: HashMap<TypeId, StateNode<C, E>>,
  insertion_order: Vec<TypeId>,
  initial_state: Option<TypeId>,
}
impl<C, E> StateGraph<C, E> {
  fn new() -> Self {
    Self { nodes: HashMap::new(), insertion_order: vec![], initial_state: None }
  }

  pub fn builder() -> StateTreeBuilder<C, E> {
    StateTreeBuilder::new()
  }

  fn lookup(&self, type_id: TypeId) -> &StateNode<C, E> {
    self.nodes.get(&type_id).unwrap()
  }

  fn execute_factory(&self, type_id: TypeId) -> StateInstance<C, E> {
    let node = self.lookup(type_id);
    let instance = (node.factory)();
    StateInstance {
      type_id: node.type_id,
      debug_name: node.debug_name,
      instance,
    }
  }

  fn add_state(
    &mut self,
    type_id: TypeId,
    debug_name: &'static str,
    parent_type_id: Option<TypeId>,
    factory: fn() -> Box<dyn State<Context = C, Event = E> + Send>) {
    let node = StateNode {
      type_id,
      debug_name,
      parent_type_id,
      transitions_to: HashSet::new(),
      is_parent: false,
      factory
    };
    if let Some(real_parent_type_id) = parent_type_id {
      let parent = self.nodes.get_mut(&real_parent_type_id).unwrap();
      parent.is_parent = true;
    }
    self.nodes.insert(type_id, node);
    self.insertion_order.push(type_id);
  }

  fn add_transition(&mut self, src_type_id: TypeId, dest_type_id: TypeId) {
    let node = self.nodes.get_mut(&src_type_id).unwrap();
    node.transitions_to.insert(dest_type_id);
  }

  fn set_initial(&mut self, initial_state: TypeId) {
    self.initial_state = Some(initial_state);
  }
}

struct StateInstance<C, E> {
  type_id: TypeId,
  debug_name: &'static str,
  instance: Box<dyn State<Context = C, Event = E> + Send>,
}

struct StateNode<C, E> {
  type_id: TypeId,
  debug_name: &'static str,
  parent_type_id: Option<TypeId>,
  transitions_to: HashSet<TypeId>,
  is_parent: bool,
  factory: fn() -> Box<dyn State<Context = C, Event = E> + Send>,
}

pub struct StateGraphPrinter;
impl StateGraphPrinter {
  /// # Example output:
  ///
  /// ```text
  /// StateGraph {
  ///   Uninitialized => Initializing
  ///   Initializing => [Ready, Uninitialized]
  ///   Initialized { => Uninitialized
  ///     Ready => Working
  ///     Working => Ready
  ///   }
  /// }
  /// ```
  pub fn pretty_print<C, E>(states: &StateGraph<C, E>) {
    println!("StateGraph {{");
    StateGraphPrinter::print_internal(states, "StateGraph", None, 1);
    println!("}}");
  }

  fn print_internal<C, E>(
    states: &StateGraph<C, E>,
    parent_name: &'static str,
    parent_type_id: Option<TypeId>,
    indent: usize) {
    let children = states.nodes.values().filter(|s| {
      s.parent_type_id == parent_type_id
    });

    // Stupid inefficient iteration but since this is just for pretty printing who really cares.
    // I really don't know what the proper idiomatic way to do this in Rust would even be either...
    for &ordered_type_id in &states.insertion_order {
      let found = children.clone().find(|s| {
        s.type_id == ordered_type_id
      });
      if let Some(node) = found {
        for _ in 0..indent {
          print!("  ");
        }

        let node_name = states.nodes.get(&node.type_id).unwrap().debug_name;
        print!("{}", node_name);

        if node.is_parent {
          print!(" {{");
        }

        let num_transitions = node.transitions_to.len();
        if num_transitions > 0 {
          print!(" => ");
          let transition_suffix = if num_transitions > 1 {
            print!("[");
            "]"
          } else {
            ""
          };
          let names: Vec<&str> = node.transitions_to.iter().map(|t| {
            states.nodes.get(t).unwrap().debug_name
          }).collect();
          print!("{}", names.join(", "));
          print!("{}", transition_suffix);
        }
        println!();

        if node.is_parent {
          StateGraphPrinter::print_internal(states, node_name, Some(node.type_id), indent + 1);
          for _ in 0..indent {
            print!("  ");
          }
          println!("}}");
        }
      }
    }
  }
}

#[cfg(test)]
mod tests {
  use std::any::TypeId;
  use anyhow::anyhow;
  use tokio::time::Duration;
  use crate::state_machine::{HandleResult, NotHandled, State, StateMachineDescriptor, StateGraph, Transition, StateGraphPrinter, Dispatcher, Context, StateMachine};

  const INITIALIZE_DELAY_MS: u64 = 1000;

  #[derive(Default)]
  struct NotInitialized;
  impl State for NotInitialized {
    type Context = TestContext;
    type Event = TestEvent;

    fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {
      context.user.event("NotInitialized: enter");
    }

    fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {
      context.user.event("NotInitialized: exit");
    }

    fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
      match event {
        TestEvent::DoWork(_, _) => {
          context.user.event("NotInitialized: DoWork");
          context.dispatcher().dispatch_sync(TestEvent::DoInitialize);
          Err(NotHandled::DeferEvent(event))
        }
        _ => Err(NotHandled::UnknownEvent(event)),
      }
    }
  }

  #[derive(Default)]
  struct WaitingToInit;
  impl State for WaitingToInit {
    type Context = TestContext;
    type Event = TestEvent;

    fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {
      context.user.event("WaitingToInit: enter");
    }

    fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {
      context.user.event("WaitingToInit: exit");
    }

    fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
      match event {
        TestEvent::DoInitialize => {
          context.user.event("WaitingToInit: DoInitialize");
          Ok(Transition::MoveTo(TypeId::of::<Initializing>()))
        },
        _ => Err(NotHandled::UnknownEvent(event)),
      }
    }
  }

  #[derive(Default)]
  struct Initializing;
  impl State for Initializing {
    type Context = TestContext;
    type Event = TestEvent;

    fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {
      context.user.event("Initializing: enter");
      context.dispatcher().dispatch_delay_ms(TestEvent::OnInitialized, INITIALIZE_DELAY_MS);
    }

    fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {
      context.user.event("Initializing: exit");
    }

    fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
      match event {
        TestEvent::DoInitialize => {
          context.user.event("Initializing: DoInitialize");
          Ok(Transition::None)
        },
        TestEvent::OnInitialized => {
          context.user.event("Initializing: OnInitialized");
          Ok(Transition::MoveTo(TypeId::of::<Ready>()))
        },
        _ => Err(NotHandled::UnknownEvent(event)),
      }
    }
  }

  #[derive(Default)]
  struct Initialized {
    work_count: u32,
  }
  impl State for Initialized {
    type Context = TestContext;
    type Event = TestEvent;

    fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {
      context.user.event("Initialized: enter");
    }

    fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {
      context.user.event("Initialized: exit");
    }


    fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
      match event {
        _ => Err(NotHandled::UnknownEvent(event)),
      }
    }
  }

  #[derive(Default)]
  struct Ready;
  impl State for Ready {
    type Context = TestContext;
    type Event = TestEvent;

    fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {
      context.user.event("Ready: enter");
    }

    fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {
      context.user.event("Ready: exit");
    }


    fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
      match event {
        TestEvent::DoWork(token, will_be_success) => {
          context.user.event(format!("Ready: DoWork {} {}", token, will_be_success).as_str());
          let dispatcher = context.dispatcher();
          if will_be_success {
            dispatcher.dispatch_sync(TestEvent::OnWorkSuccess);
          } else {
            dispatcher.dispatch_sync(TestEvent::OnWorkFailed(anyhow!("Bummer")));
          }
          Ok(Transition::MoveTo(TypeId::of::<Busy>()))
        }
        _ => Err(NotHandled::UnknownEvent(event)),
      }
    }
  }

  #[derive(Default)]
  struct Busy;
  impl State for Busy {
    type Context = TestContext;
    type Event = TestEvent;

    fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
      match event {
        TestEvent::OnWorkSuccess => {
          context.user.event("Busy: OnWorkSuccess");
          Ok(Transition::MoveTo(TypeId::of::<Ready>()))
        },
        TestEvent::OnWorkFailed(err) => {
          context.user.event(format!("Busy: OnWorkFailed {:?}", err).as_str());
          Ok(Transition::MoveTo(TypeId::of::<WaitingToInit>()))
        },
        TestEvent::DoWork(_, _) => {
          context.user.event("Busy: DoWork");
          Err(NotHandled::DeferEvent(event))
        },
        _ => Err(NotHandled::UnknownEvent(event)),
      }
    }
  }

  #[derive(Default, Clone)]
  struct TestContext {
    events: Vec<String>,
  }
  impl TestContext {
    fn event(&mut self, label: &str) {
      println!("TestContext: {}", label);
      self.events.push(String::from(label));
    }
  }

  #[derive(Debug)]
  enum TestEvent {
    DoInitialize,
    OnInitialized,
    OnInitializationError(anyhow::Error),
    DoWork(i32, bool),
    OnWorkSuccess,
    OnWorkFailed(anyhow::Error),
  }

  struct TestStateMachine;
  impl StateMachineDescriptor for TestStateMachine {
    type Context = TestContext;
    type Event = TestEvent;

    fn debug_name() -> &'static str {
      return "TestStateMachine";
    }

    fn states() -> StateGraph<Self::Context, Self::Event> {
      StateGraph::builder()
          .add_parent::<NotInitialized>(|s| s
              .add::<WaitingToInit>().initial().transitions_to::<Initializing>()
              .add::<Initializing>()
                  .transitions_to::<Ready>()
                  .transitions_to::<WaitingToInit>()
          )
          .add_parent::<Initialized>(|s| s
              .add::<Ready>().transitions_to::<Busy>()
              .add::<Busy>().transitions_to::<Ready>())
          .transitions_to::<WaitingToInit>()
          .build()
    }
  }

  #[test]
  fn test_state_builder_smoke() {
    let graph = TestStateMachine::states();
    StateGraphPrinter::pretty_print(&graph);
  }

  #[tokio::test]
  async fn test_shutdown_causes_task_cleanup() {
    let machine = StateMachine::<TestContext, TestEvent>::start::<TestStateMachine>();
    machine.shutdown().await;
  }

  #[tokio::test(start_paused = true)]
  async fn test_deferred_and_delayed_messages() {
    let machine = StateMachine::<TestContext, TestEvent>::start::<TestStateMachine>();
    let dispatcher = machine.dispatcher();
    dispatcher.dispatch(TestEvent::DoWork(123, true)).await;
    dispatcher.dispatch(TestEvent::DoWork(456, false)).await;

    tokio::task::yield_now().await;
    tokio::time::advance(Duration::from_millis(INITIALIZE_DELAY_MS)).await;
    tokio::task::yield_now().await;

    let context = machine.shutdown().await.unwrap();
    assert_eq!(context.events, vec![
      "NotInitialized: enter",
      "WaitingToInit: enter",
      "NotInitialized: DoWork",
      "NotInitialized: DoWork",
      "WaitingToInit: DoInitialize",
      "WaitingToInit: exit",
      "Initializing: enter",
      "NotInitialized: DoWork",
      "NotInitialized: DoWork",
      "Initializing: DoInitialize",
      "Initializing: DoInitialize",
      "Initializing: DoInitialize",
      "Initializing: OnInitialized",
      "Initializing: exit",
      "NotInitialized: exit",
      "Initialized: enter",
      "Ready: enter",
      "Ready: DoWork 123 true",
      "Ready: exit",
      "Busy: DoWork",
      "Busy: OnWorkSuccess",
      "Ready: enter",
      "Ready: DoWork 456 false",
      "Ready: exit",
      "Busy: OnWorkFailed Bummer",
      "Initialized: exit",
      "NotInitialized: enter",
      "WaitingToInit: enter",
    ]);
  }

  #[tokio::test]
  #[should_panic]
  async fn test_invalid_transitions() {
    let machine = StateMachine::<LiteralContext, LiteralEvent>::start::<LiteralStateMachine>();
    let dispatcher = machine.dispatcher();
    dispatcher.dispatch(LiteralEvent::GoToZ).await;

    machine.shutdown().await.unwrap();
  }

  #[derive(Default)]
  struct StateX;
  impl State for StateX {
    type Context = LiteralContext;
    type Event = LiteralEvent;

    fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
      common_handling(event)
    }
  }
  #[derive(Default)]
  struct StateY;
  impl State for StateY {
    type Context = LiteralContext;
    type Event = LiteralEvent;

    fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
      common_handling(event)
    }
  }
  #[derive(Default)]
  struct StateZ;
  impl State for StateZ {
    type Context = LiteralContext;
    type Event = LiteralEvent;

    fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
      common_handling(event)
    }
  }

  fn common_handling(event: LiteralEvent) -> HandleResult<LiteralEvent> {
    match event {
      LiteralEvent::GoToX => {
        Ok(Transition::MoveTo(TypeId::of::<StateX>()))
      },
      LiteralEvent::GoToY => {
        Ok(Transition::MoveTo(TypeId::of::<StateY>()))
      },
      LiteralEvent::GoToZ => {
        Ok(Transition::MoveTo(TypeId::of::<StateZ>()))
      },
      _ => Err(NotHandled::UnknownEvent(event)),
    }
  }

  #[derive(Default)]
  struct LiteralContext;

  #[derive(Debug)]
  enum LiteralEvent {
    GoToX,
    GoToY,
    GoToZ,
  }

  /// A state machine that just does literally whatever the events say, which allows us to invoke
  /// bad runtime behaviour at will.  Eventually we'd like this to all be caught at compile-time
  /// but doing so significantly degrades our ability to implement things without macro hell AFAICT.
  struct LiteralStateMachine;
  impl StateMachineDescriptor for LiteralStateMachine {
    type Context = LiteralContext;
    type Event = LiteralEvent;

    fn debug_name() -> &'static str {
      return "LiteralStateMachine";
    }

    fn states() -> StateGraph<Self::Context, Self::Event> {
      StateGraph::builder()
          .add::<StateX>().initial().transitions_to::<StateY>()
          .add::<StateY>().transitions_to::<StateZ>()
          .add::<StateZ>().transitions_to::<StateX>()
          .build()
    }
  }
}