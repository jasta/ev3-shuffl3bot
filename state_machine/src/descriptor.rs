use std::fmt::Debug;

use crate::graph::StateGraph;

pub trait StateMachineDescriptor {
  type Context: Default;
  type Event: Debug;

  fn debug_name(&self) -> &'static str;
  fn states(&self) -> StateGraph<Self::Context, Self::Event>;
  fn into_context(self) -> Self::Context where Self: Sized { Self::Context::default() }
}
