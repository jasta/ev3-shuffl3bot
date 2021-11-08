use std::any::TypeId;
use std::borrow::BorrowMut;
use std::ops::{Deref, Range};
use std::rc::Rc;
use std::sync::Arc;

use futures::TryFutureExt;
use futures_signals::signal::{Mutable, MutableSignalCloned};
use rand::distributions::uniform::SampleBorrow;
use tokio::sync::{Mutex, oneshot};
use tokio::sync::watch::{Receiver, Sender};
use tokio::task::JoinHandle;

use finny::{finny_fsm, FsmEventQueueVec};
use finny::decl::{BuiltFsm, FsmBuilder};
use finny::FsmBackend;
use finny::FsmFactory;
use finny::FsmFrontend;
use state_machine::descriptor::StateMachineDescriptor;
use state_machine::graph::StateGraph;
use state_machine::machine::{Context, Dispatcher, StateMachine};
use state_machine::state::{HandleResult, NotHandled, State, Transition};

use crate::pressure_sampler::PressureSensorSampler;
use crate::state_machine::{State, StateGraph, StateMachine, StateMachineDescriptor};
use crate::suction_pump_hal::{HalError, HalResult, PumpDirection, SuctionPumpHal, SuctionPumpMotor, SuctionPumpPressureSensor};

const NORMAL_ATMOSPHERIC_PRESSURE: Range<i32> = 104000 .. 110000;

pub struct SuctionPumpMachine {
  pressure_sm_dispatcher: Dispatcher<SuctionPumpEvents>,
  motor_sm_dispatcher: Dispatcher<SuctionPumpEvents>,
  airtight_seal_sender: Sender<AirtightSealState>
}

impl SuctionPumpMachine {
  pub fn new(pump_motor: Box<dyn SuctionPumpMotor + Send>, pressure_sampler: PressureSensorSampler) -> Self {
    let (airtight_seal_sender, _) = tokio::sync::watch::channel(AirtightSealState(None));
    let pressure_sampler_arc = Arc::new(Mutex::new(pressure_sampler));
    let pressure_sm = StateMachine::<(), SuctionPumpEvents>::start(
        PressureSm {
          context: PressureSmContext {
            pressure_sampler: pressure_sampler_arc.clone(),
          }
        });
    let motor_sm = StateMachine::<MotorSmContext, SuctionPumpEvents>::start(
        MotorSm {
          context: MotorSmContext {
            motor: pump_motor,
            pressure_sampler: pressure_sampler_arc.clone(),
          }
        });
    Self {
      pressure_sm_dispatcher: pressure_sm.dispatcher(),
      motor_sm_dispatcher: motor_sm.dispatcher(),
      airtight_seal_sender,
    }
  }

  pub async fn subscribe_airtight_seal(&self) -> Receiver<AirtightSealState> {
    self.airtight_seal_sender.lock().await.subscribe()
  }

  pub async fn pump_and_hold_at(&self, request: PumpAndHoldAtData) {
    self.pressure_sm_dispatcher.dispatch(SuctionPumpEvents::DoStartSensingPressure);
    self.motor_sm_dispatcher.dispatch(SuctionPumpEvents::DoPumpAndHoldAt(request));
  }
}

#[derive(Copy, Clone, Debug)]
pub struct AirtightSealState(Option<bool>);

struct MotorSmContext {
  motor: Box<dyn SuctionPumpMotor>,
  pressure_sampler: Arc<Mutex<PressureSensorSampler>>,
}

struct MotorSm {
  context: MotorSmContext,
}
impl StateMachineDescriptor for MotorSm {
  type Context = MotorSmContext;
  type Event = SuctionPumpEvents;

  fn debug_name(&self) -> &'static str {
    return "SuctionPumpMotorSm";
  }

  fn states(&self) -> StateGraph<Self::Context, Self::Event> {
    StateGraph::builder()
        .add::<MotorStopped>().initial().transitions_to::<MotorPumping>()
        .add_parent::<MotorSensingPressure>(|s| s
            .add::<MotorPumping>()
                .transitions_to::<MotorStopped>()
                .transitions_to::<MotorStandby>()
            .add::<MotorStandby>()
                .transitions_to::<MotorStopped>()
                .transitions_to::<MotorPumping>()
        )
        .build()
  }

  fn into_context(self) -> Self::Context {
    self.context
  }
}

#[derive(Default)]
struct MotorStopped;
impl State for MotorStopped {
  type Context = MotorSmContext;
  type Event = SuctionPumpEvents;

  fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
    match event {
      SuctionPumpEvents::DoPumpAndHoldAt(data) => {
        Ok(Transition::MoveTo(TypeId::of::<MotorPumping>()))
      },
      _ => Err(NotHandled::UnknownEvent(event))
    }
  }
}

#[derive(Default)]
struct MotorSensingPressure {
  pressure_relay_handle: JoinHandle<()>,
}
impl State for MotorSensingPressure {
  type Context = MotorSmContext;
  type Event = SuctionPumpEvents;

  fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {
    let dispatcher = context.dispatcher();
    self.pressure_relay_handle = tokio::spawn(async move {
      let sampler = context.user.pressure_sampler.lock().await;
      let mut subscription = sampler.subscribe();
      drop(sampler);
      while subscription.changed().await.is_ok() {
        let result = subscription.borrow();
        match result {
          Ok(pressure) => dispatcher.dispatch(SuctionPumpEvents::OnPressureChange(pressure)).await,
          Err(err) => dispatcher.dispatch(SuctionPumpEvents::OnSensorError(err)).await,
        }
      }
    });
  }

  fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {
    self.pressure_relay_handle.abort();
  }

  fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
    Err(NotHandled::UnknownEvent(event))
  }
}

#[derive(Default)]
struct MotorPumping;
impl State for MotorPumping {
  type Context = MotorSmContext;
  type Event = SuctionPumpEvents;

  fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
    match event {
      SuctionPumpEvents::OnPressureChange(pressure) => {
        if (check_pressure(pressure, ))
        Ok(Transition::None)
      },
      _ => Err(NotHandled::UnknownEvent(event)),
    }
  }
}

#[derive(Default)]
struct MotorStandby;
impl State for MotorStandby {
}

struct PressureSmContext {
  pressure_sampler: Arc<Mutex<PressureSensorSampler>>,
}

struct PressureSm {
  context: PressureSmContext,
}

impl StateMachineDescriptor for PressureSm {
  type Context = PressureSmContext;
  type Event = SuctionPumpEvents;

  fn debug_name(&self) -> &'static str {
    return "SuctionPumpPressureSm";
  }

  fn states(&self) -> StateGraph<Self::Context, Self::Event> {
    StateGraph::builder()
        .add_parent::<PressureNoReading>(|s| s
            .add::<PressureIdle>().initial().transitions_to::<PressureWaitingForFirstReading>()
            .add::<PressureWaitingForFirstReading>()
                .transitions_to::<PressureOpen>()
                .transitions_to::<PressureClosed>())
        .add_parent::<PressureHasReading>(|s| s
            .add::<PressureOpen>().transitions_to::<PressureClosed>()
            .add::<PressureClosed>().transitions_to::<PressureOpen>())
            .transitions_to::<PressureIdle>()
        .build()
  }

  fn into_context(self) -> Self::Context {
    return self.context
  }
}

#[derive(Default)]
struct PressureNoReading;
impl State for PressureNoReading {
}

#[derive(Default)]
struct PressureIdle;
impl State for PressureIdle {
}

#[derive(Default)]
struct PressureWaitingForFirstReading;
impl State for PressureWaitingForFirstReading {
}

#[derive(Default)]
struct PressureHasReading;
impl State for PressureHasReading {
}

#[derive(Default)]
struct PressureClosed;
impl State for PressureClosed {
}

#[derive(Default)]
struct PressureOpen;
impl State for PressureOpen {
}

#[derive(Debug, Clone)]
enum SuctionPumpEvents {
  OnPressureChange(u32),
  OnSensorError(HalError),
  DoStartSensingPressure,
  DoPumpAndHoldAt(PumpAndHoldAtData),
  DoStopUngracefully,
}

#[derive(Clone, Debug)]
pub struct PumpAndHoldAtData {
  direction: PumpDirection,
  pressure_range_pa: Range<i32>,
}

fn check_pressure(current_pressure: i32, target: PumpAndHoldAtData, which_check: PressureCheck) -> bool {
  let target_min = target.pressure_range_pa.min().unwrap();
  let target_max = target.pressure_range_pa.max().unwrap();
  match which_check {
    PressureCheck::PumpedTooLittle => {
      match target.direction {
        PumpDirection::Vacuum => value > target_max,
        PumpDirection::Pressurize => value < target_min,
      }
    },
    PressureCheck::PumpedTooMuch => {
      match target.direction {
        PumpDirection::Vacuum => value < target_min,
        PumpDirection::Pressurize => value > target_max,
      }
    },
    PressureCheck::NormalAtmosphericPressure => {
      NORMAL_ATMOSPHERIC_PRESSURE.contains(&value)
    },
  }
}

enum PressureCheck {
  PumpedTooLittle,
  PumpedTooMuch,
  NormalAtmosphericPressure,
}
