use std::any::TypeId;
use std::collections::{HashMap, HashSet};

use crate::machine::*;
use crate::state::*;
use crate::graph_builder::StateGraphBuilder;

pub struct StateGraph<C, E> {
  pub(crate) nodes: HashMap<TypeId, StateNode<C, E>>,
  pub(crate) insertion_order: Vec<TypeId>,
  pub(crate) initial_state: Option<TypeId>,
}

impl<C, E> StateGraph<C, E> {
  pub(crate) fn new() -> Self {
    Self { nodes: HashMap::new(), insertion_order: vec![], initial_state: None }
  }

  pub fn builder() -> StateGraphBuilder<C, E> {
    StateGraphBuilder::new()
  }

  pub(crate) fn lookup(&self, type_id: TypeId) -> &StateNode<C, E> {
    self.nodes.get(&type_id).unwrap()
  }

  pub(crate) fn execute_factory(&self, type_id: TypeId) -> StateInstance<C, E> {
    let node = self.lookup(type_id);
    let instance = (node.factory)();
    StateInstance {
      type_id: node.type_id,
      debug_name: node.debug_name,
      instance,
    }
  }

  pub(crate) fn add_state(
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

  pub(crate) fn add_transition(&mut self, src_type_id: TypeId, dest_type_id: TypeId) {
    let node = self.nodes.get_mut(&src_type_id).unwrap();
    node.transitions_to.insert(dest_type_id);
  }

  pub(crate) fn set_initial(&mut self, initial_state: TypeId) {
    self.initial_state = Some(initial_state);
  }
}