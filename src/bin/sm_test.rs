use std::any::Any;
use std::ops::Range;
use std::sync::{Arc, Mutex};

use tokio::sync::mpsc;
use tokio::sync::mpsc::{Receiver, Sender};

#[derive(Clone, Debug)]
pub struct PumpAndHoldAtData {
  direction: PumpDirection,
  pressure_range_pa: Range<i32>,
}

#[derive(Clone, Debug)]
pub enum PumpDirection {
  Vacuum,
  Pressurize,
}

#[derive(Debug, Clone)]
enum Event {
  DoCalibrate,
  OnCalibrationResult,
  DoStop,
  DoPumpAndHoldAt(PumpAndHoldAtData),
  OnPressureChange(HalResult<i32>),
}

#[derive(Debug, Default)]
struct PumpSmContext {
  pump_and_hold_at_data: Option<PumpAndHoldAtData>,
}

mod Motor {
  use ev3_shuffl3bot::simple_async_state_machine::{BaseState, HandleResult, SimpleAsyncStateMachine};

  use crate::{Event, PumpAndHoldAtData};

  pub trait State: BaseState {
  }

  #[derive(Debug)]
  pub struct Stopped;
  impl BaseState for Stopped {
    fn handle(&mut self, sm: &mut SimpleAsyncStateMachine, event: Event) -> HandleResult {
      match event {
        Event::DoPumpAndHoldAt(data) => {
          sm.transition_to(Box::new(Pumping {
            pump_and_hold_at_data: data.clone(),
          }));
          HandleResult::Handled
        },
        _ => HandleResult::NotHandled,
      }
    }
  }

  #[derive(Debug)]
  struct Pumping {
    pump_and_hold_at_data: PumpAndHoldAtData,
  }
  impl BaseState for Pumping {
    fn handle(&mut self, sm: &mut SimpleAsyncStateMachine, event: Event) -> HandleResult {
      match event {
        Event::OnPressureChange(_) => {
          todo!("lol, obviously no");
          sm.transition_to(Box::new(Hold {
            pump_and_hold_at_data: self.pump_and_hold_at_data.clone(),
          }));
          HandleResult::Handled
        },
        _ => HandleResult::NotHandled,
      }
    }
  }

  #[derive(Debug)]
  struct Hold {
    pump_and_hold_at_data: PumpAndHoldAtData,
  }
  impl BaseState for Hold {
    fn handle(&mut self, sm: &mut SimpleAsyncStateMachine, event: Event) -> HandleResult {
      match event {
        Event::OnPressureChange(_) => {
          todo!("lol, obviously no");
          sm.transition_to(Box::new(Pumping {
            pump_and_hold_at_data: self.pump_and_hold_at_data.clone(),
          }));
          HandleResult::Handled
        }
        _ => HandleResult::NotHandled
      }
    }
  }
}

mod Pressure {
  use ev3_shuffl3bot::simple_async_state_machine::BaseState;

  pub trait State: BaseState {
  }

  #[derive(Debug, PartialEq)]
  enum State2 {
    NoReading,
    Initializing,
    Open,
    Closed,
  }
}

fn main() {

}

