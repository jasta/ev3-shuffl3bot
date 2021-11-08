use std::any::TypeId;
use std::cmp::min;
use std::collections::HashSet;
use std::collections::VecDeque;
use std::convert::TryFrom;
use std::fmt::Debug;

use tokio::sync::mpsc::{Receiver, Sender};
use tokio::task::{JoinError, JoinHandle};
use tokio::time::{Duration, Instant};

use crate::state::*;
use crate::descriptor::*;
use crate::graph::*;

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

pub(crate) struct StateInstance<C, E> {
  pub(crate) type_id: TypeId,
  pub(crate) debug_name: &'static str,
  pub(crate) instance: Box<dyn State<Context = C, Event = E> + Send>,
}

pub(crate) struct StateNode<C, E> {
  pub(crate) type_id: TypeId,
  pub(crate) debug_name: &'static str,
  pub(crate) parent_type_id: Option<TypeId>,
  pub(crate) transitions_to: HashSet<TypeId>,
  pub(crate) is_parent: bool,
  pub(crate) factory: fn() -> Box<dyn State<Context = C, Event = E> + Send>,
}
