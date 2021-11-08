use std::any::TypeId;
use std::fmt::Debug;

use thiserror::Error;

use crate::machine::*;

pub trait State {
  type Context;
  type Event: Debug;

  fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {}
  fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {}

  #[must_use]
  fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event>;
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
