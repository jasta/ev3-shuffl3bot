use std::fmt::Debug;

use crate::graph::StateGraph;

pub trait StateMachineDescriptor {
  type Context: Default;
  type Event: Debug;

  fn debug_name() -> &'static str;
  fn states() -> StateGraph<Self::Context, Self::Event>;
  fn new_context() -> Self::Context { Self::Context::default() }
}
