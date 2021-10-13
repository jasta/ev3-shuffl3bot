use std::fmt::Debug;
use std::sync::Arc;
use tokio::sync::mpsc::{Receiver, Sender};
use tokio::sync::{mpsc, Mutex};

use thiserror::Error;
use tokio::task::JoinHandle;

pub struct SimpleAsyncStateMachine<E: Debug, C: Send> {
  debug_name: &'static str,
  current_state: Box<dyn BaseState<Context = C, Event = E>>,
  context: Arc<Mutex<C>>,
}

impl<E: Debug + Send + 'static, C: Send + Default + 'static> SimpleAsyncStateMachine<E, C> {
  pub fn start(debug_name: &'static str, initial_state: Box<dyn BaseState<Context = C, Event = E> + Send>, context: Arc<Mutex<C>>) -> StartedStateMachineHandle<E> {
    let (tx, rx) = mpsc::channel::<InternalEvent<E>>(32);
    let join_handle = tokio::spawn(async move {
      let mut me = Self {
        debug_name,
        current_state: initial_state,
        context,
      };
      me.initialize().await;
      me.handle_events(rx).await;
    });

    return StartedStateMachineHandle { tx, join_handle };
  }

  async fn initialize(&mut self) {
    println!("{}: Initializing...", self.debug_name);
    let mut context = self.context.lock().await;
    self.current_state.on_enter(&mut context);
  }

  async fn handle_events(
    &mut self,
    mut rx: Receiver<InternalEvent<E>>) {
    'outer: while let Some(event) = rx.recv().await {
      match event {
        InternalEvent::UserEvent(event) => {
          println!("{}: [{:?}] Received [{:?}]", self.debug_name, self.current_state, event);
          let event_str = format!("{:?}", event);
          let mut context = self.context.lock().await;
          let handled = self.current_state.handle(&mut context, event);
          match handled {
            Ok(ok) => {
              match ok {
                Transition::Stay => (),
                Transition::MoveTo(mut next) => {
                  println!("{}: [{:?}] => [{:?}]", self.debug_name, self.current_state, next);
                  self.current_state.on_exit(&mut context);
                  next.on_enter(&mut context);
                  self.current_state = next;
                }
              }
            },
            Err(err) => {
              match err {
                NotHandled::Unknown => {
                  println!("{}: Unhandled: [{}]!!", self.debug_name, event_str);
                },
                NotHandled::InternalError(internal_err) => {
                  println!("{}: Internal error handling [{}]: {}", self.debug_name, event_str, internal_err);
                }
              }
            },
          }
        },
        InternalEvent::Shutdown => {
          println!("{}: Received shutdown signal...", self.debug_name);
          break 'outer
        },
      }
    }
    println!("{}: State machine shutting down", self.debug_name);
  }
}

pub struct StartedStateMachineHandle<E: Debug> {
  tx: Sender<InternalEvent<E>>,
  join_handle: JoinHandle<()>,
}

impl<E: Debug> StartedStateMachineHandle<E> {
  pub async fn shutdown(self) {
    self.tx.send(InternalEvent::Shutdown).await;
    self.join_handle.await;
  }

  pub async fn dispatch(&self, event: E) {
    self.tx.send(InternalEvent::UserEvent(event)).await;
  }
}

enum InternalEvent<E: Debug> {
  UserEvent(E),
  Shutdown,
}

pub trait BaseState: Debug + Send {
  type Context;
  type Event: Debug;

  fn on_enter(&mut self, context: &mut Self::Context) {}
  fn on_exit(&mut self, context: &mut Self::Context) {}

  #[must_use]
  fn handle(&mut self, context: &mut Self::Context, event: Self::Event) -> HandleResult<Self::Event, Self::Context>;
}

pub type HandleResult<E, C> = Result<Transition<E, C>, NotHandled>;

pub enum Transition<E, C> {
  MoveTo(Box<dyn BaseState<Context = C, Event = E> + Send>),
  Stay,
}

#[derive(Error, Debug)]
pub enum NotHandled {
  #[error("unknown event")]
  Unknown,
  #[error("internal error: {0}")]
  InternalError(anyhow::Error)
}

#[cfg(test)]
mod tests {
  use futures::TryFutureExt;
  use super::*;

  #[derive(Debug)]
  enum TestEvent {
    DoTransitionToNext(i32),
    DoPing,
  }

  #[derive(Debug, Default)]
  struct TestContext {
    recordings: Vec<String>,
  }

  impl TestContext {
    fn push(&mut self, value: &str) {
      self.recordings.push(String::from(value));
    }
  }

  #[derive(Debug, Default)]
  struct TestStateX {
    num_pings: usize,
  }
  impl BaseState for TestStateX {
    type Context = TestContext;
    type Event = TestEvent;

    fn on_enter(&mut self, context: &mut Self::Context) {
      context.push("X: enter");
    }

    fn on_exit(&mut self, context: &mut Self::Context) {
      context.push("X: exit");
    }

    fn handle(&mut self, context: &mut Self::Context, event: Self::Event) -> HandleResult<Self::Event, Self::Context> {
      match event {
        TestEvent::DoPing => {
          self.num_pings += 1;
          context.recordings.push(format!("X: ping {}", self.num_pings));
          Ok(Transition::Stay)
        },
        TestEvent::DoTransitionToNext(token) => {
          Ok(Transition::MoveTo(Box::new(TestStateY { token })))
        },
        _ => Err(NotHandled::Unknown),
      }
    }
  }
  #[derive(Debug)]
  struct TestStateY {
    token: i32,
  }
  impl BaseState for TestStateY {
    type Context = TestContext;
    type Event = TestEvent;

    fn on_enter(&mut self, context: &mut Self::Context) {
      context.push(format!("Y: enter w/ token {}", self.token).as_str());
    }

    fn on_exit(&mut self, context: &mut Self::Context) {
      context.push(format!("Y: exit w/ token {}", self.token).as_str());
    }

    fn handle(&mut self, context: &mut Self::Context, event: Self::Event) -> HandleResult<Self::Event, Self::Context> {
      match event {
        TestEvent::DoPing => {
          context.recordings.push(format!("Y: ping w/ token {}", self.token));
          Ok(Transition::Stay)
        },
        TestEvent::DoTransitionToNext(token) => {
          Ok(Transition::MoveTo(Box::new(TestStateZ {})))
        },
        _ => Err(NotHandled::Unknown),
      }
    }
  }

  #[derive(Debug)]
  struct TestStateZ;
  impl BaseState for TestStateZ {
    type Context = TestContext;
    type Event = TestEvent;

    fn on_enter(&mut self, context: &mut Self::Context) {
      context.push("Z: enter");
    }

    fn on_exit(&mut self, context: &mut Self::Context) {
      context.push("Z: exit");
    }

    fn handle(&mut self, context: &mut Self::Context, event: Self::Event) -> HandleResult<Self::Event, Self::Context> {
      match event {
        TestEvent::DoPing => {
          context.push("Z: ping");
          Ok(Transition::Stay)
        },
        _ => Err(NotHandled::Unknown),
      }
    }
  }

  #[tokio::test]
  async fn test_shutdown_causes_task_cleanup() {
    let context = Arc::new(Mutex::new(TestContext::default()));
    let sm = SimpleAsyncStateMachine::start(
      "test",
      Box::new(TestStateX::default()),
      context.clone());
    sm.shutdown().await;
  }

  #[tokio::test]
  async fn test_events_and_transitions_with_state() {
    let context = Arc::new(Mutex::new(TestContext::default()));
    let sm = SimpleAsyncStateMachine::start(
        "test",
        Box::new(TestStateX::default()),
        context.clone());
    sm.dispatch(TestEvent::DoPing).await;
    sm.dispatch(TestEvent::DoPing).await;
    sm.dispatch(TestEvent::DoPing).await;
    sm.dispatch(TestEvent::DoTransitionToNext(123)).await;
    sm.dispatch(TestEvent::DoPing).await;
    sm.dispatch(TestEvent::DoPing).await;
    sm.dispatch(TestEvent::DoTransitionToNext(456)).await;
    sm.dispatch(TestEvent::DoPing).await;

    tokio::task::yield_now().await;

    let locked_context = context.lock().await;
    assert_eq!(locked_context.recordings, vec![
      "X: enter",
      "X: ping 1",
      "X: ping 2",
      "X: ping 3",
      "X: exit",
      "Y: enter w/ token 123",
      "Y: ping w/ token 123",
      "Y: ping w/ token 123",
      "Y: exit w/ token 123",
      "Z: enter",
      "Z: ping",
    ]);
  }
}