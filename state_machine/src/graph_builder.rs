use std::any::TypeId;

use crate::graph::StateGraph;
use crate::state::State;

pub struct StateGraphBuilder<C, E> {
  tree: StateGraph<C, E>,
  parent_type_ids: Vec<TypeId>,
  last_type_id: Option<TypeId>,
}

impl<C, E> StateGraphBuilder<C, E> {
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
