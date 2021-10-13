use std::ops::Range;

use finny::{finny_fsm, FsmEventQueueVec};
use finny::decl::{BuiltFsm, FsmBuilder};
use finny::FsmBackend;
use finny::FsmFactory;
use finny::FsmFrontend;
use futures::TryFutureExt;
use futures_signals::signal::{Mutable, MutableSignalCloned};
use crate::pressure_sampler::PressureSensorSampler;

use crate::suction_pump_hal::{HalError, HalResult, SuctionPumpHal};

const NORMAL_ATMOSPHERIC_PRESSURE: Range<i32> = 104000 .. 110000;

// pub struct SuctionPumpMachine {
//   sm: FsmFrontend<PumpSm, FsmEventQueueVec<PumpSm>>,
//   state: Mutable<SuctionPumpState>,
// }
//
// impl SuctionPumpMachine {
//   fn new(hal: Box<dyn SuctionPumpHal>) -> Self {
//     let state = Mutable::new(SuctionPumpState::NotCalibrated);
//     let context = PumpSmContext { hal, emitted_state: state.clone(), pump_and_hold_at_data: None };
//     let sm = PumpSm::new(context).unwrap();
//     sm.start().unwrap();
//     Self { sm, state: state.clone() }
//   }
//
//   fn observe(&self) -> MutableSignalCloned<SuctionPumpState> {
//     self.state.signal_cloned()
//   }
//
//   fn calibrate(&mut self) {
//     self.sm.dispatch(EventDoCalibrate).unwrap();
//   }
//
//   fn pump_and_hold_at(&mut self, data: PumpAndHoldAtData) {
//     self.sm.dispatch(EventDoPumpAndHoldAt(data)).unwrap();
//   }
//
//   fn stop(&mut self) {
//     self.sm.dispatch(EventDoStop).unwrap();
//   }
// }
//
// #[derive(Clone, Debug)]
// pub struct PumpAndHoldAtData {
//   direction: PumpDirection,
//   pressure_range_pa: Range<i32>,
// }
//
#[derive(Clone, Debug)]
pub enum PumpDirection {
  Vacuum,
  Pressurize,
}
//
// // TODO: Replace finny, I'm really not in love with this syntax and it's super confusing
// // to structure more complex SM logic.
// #[finny_fsm]
// fn pump_state_machine(mut fsm: FsmBuilder<PumpSm, PumpSmContext>) -> BuiltFsm {
//   // fsm.state::<StateNotCalibrated>()
//   //     .on_entry(|state, ctx| {
//   //       let external_state = match state.0 {
//   //         Some(ref r) => {
//   //           SuctionPumpState::CalibrationError(r.as_ref().unwrap_err().clone())
//   //         },
//   //         None => SuctionPumpState::NotCalibrated,
//   //       };
//   //       (ctx.context as PumpSmContext).emitted_state.set(external_state);
//   //     })
//   //     .on_exit(|state, _ctx| {
//   //       state.0 = None;
//   //     })
//   //     .on_event::<EventDoCalibrate>()
//   //     .transition_to::<StateCalibrating>();
//   //
//   // fsm.state::<StateCalibrating>()
//   //     .on_entry(|_state, ctx| {
//   //       let smContext = &(ctx.context as PumpSmContext);
//   //       smContext.emitted_state.set(SuctionPumpState::Calibrating);
//   //       let pressure = smContext.hal.get_pressure_pa();
//   //       let result = match pressure {
//   //         Ok(value) => {
//   //           if NORMAL_ATMOSPHERIC_PRESSURE.contains(&value) {
//   //             Ok(())
//   //           } else {
//   //             // TODO: Handle actually pumping to calibrate...
//   //             Err(HalError::InternalError(format!("pressure={}", value)));
//   //           }
//   //         },
//   //         Err(e) => Err(e),
//   //       };
//   //       ctx.queue.enqueue(EventOnCalibrationResult(result));
//   //     });
//   // fsm.state::<StateCalibrating>()
//   //     .on_event::<EventOnCalibrationResult>()
//   //     .transition_to::<StateIdleOpen>()
//   //     .guard(|ev, _ctx| {
//   //       ev.0.is_ok()
//   //     });
//   // fsm.state::<StateCalibrating>()
//   //     .on_event::<EventOnCalibrationResult>()
//   //     .transition_to::<StateNotCalibrated>()
//   //     .guard(|ev, _ctx| {
//   //       ev.0.is_err()
//   //     })
//   //     .action(|ev, _ctx, _from, to| {
//   //       to.0 = Some(ev.0.clone());
//   //     });
//   //
//   // fsm.state::<StateIdleOpen>()
//   //     .on_entry(|_state, ctx| {
//   //       ctx.context.state.set(SuctionPumpState::Ready {
//   //         has_airtight_seal: false,
//   //         active_state: SuctionPumpActiveState::OnStopped,
//   //       });
//   //     });
//   // fsm.state::<StateIdleOpen>()
//   //     .on_event::<EventDoPumpAndHoldAt>()
//   //     .transition_to::<StateActiveOpen>()
//   //     .action(|ev, ctx| {
//   //       (ctx.context as PumpSmContext).pump_and_hold_at_data = Some(ev.0.clone());
//   //     });
//   //
//   // fsm.state::<StateActiveOpen>()
//   //     .on_entry(|_state, ctx| {
//   //       (ctx.context as PumpSmContext).emitted_state.set(SuctionPumpState::Ready {
//   //         has_airtight_seal: false,
//   //         active_state: SuctionPumpActiveState::OnPumping,
//   //       })
//   //     });
//   // fsm.state::<StateActiveOpen>()
//   //     .on_event::<EventOnPressureChange>()
//   //     .transition_to::<StateActiveClosed>()
//   //     .guard(|ev, ctx| {
//   //       let isOpen = (ctx.context as PumpSmContext).check_pressure(
//   //         ev, PressureCheck::NormalAtmosphericPressure);
//   //       !isOpen
//   //     });
//   //
//   // fsm.state::<StateActiveClosed>()
//   //     .on_entry(|_state, ctx| {
//   //       (ctx.context as PumpSmContext).emitted_state.set(SuctionPumpState::Ready {
//   //         has_airtight_seal: true,
//   //         active_state: SuctionPumpActiveState::OnPumping,
//   //       })
//   //     })
//   //     .on_event::<EventOnPressureChange>()
//   //     .transition_to::<StateHoldClosed>()
//   //     .guard(|ev, _ctx| {
//   //       (ctx.context as PumpSmContext).check_pressure(ev, PressureCheck::PumpedTooMuch)
//   //     });
//   //
//   // fsm.state::<StateHoldClosed>()
//   //     .on_entry(|_state, ctx| {
//   //       (ctx.context as PumpSmContext).emitted_state.set(SuctionPumpState::Ready {
//   //         has_airtight_seal: true,
//   //         active_state: SuctionPumpActiveState::OnHold,
//   //       })
//   //     });
//   // fsm.state::<StateHoldClosed>()
//   //     .on_event::<EventOnPressureChange>()
//   //     .transition_to::<StateIdleOpen>()
//   //     .guard(|ev, _ctx| {
//   //       (ctx.context as PumpSmContext).check_pressure(ev, PressureCheck::NormalAtmosphericPressure)
//   //     });
//   // fsm.state::<StateHoldClosed>()
//   //     .on_event::<EventOnPressureChange>()
//   //     .transition_to::<StateActiveClosed>()
//   //     .guard(|ev, _ctx| {
//   //       (ctx.context as PumpSmContext).check_pressure(ev, PressureCheck::PumpedTooLittle)
//   //     });
//
//   fsm.state::<StateMotorStopped>()
//       .on_entry(|state, ctx| {
//         (ctx.context as PumpSmContext).set_active_state(SuctionPumpActiveState::OnStopped);
//       });
//
//   fsm.state::<StateMotorPumping>()
//       .on_entry(|state, ctx| {
//         (ctx.context as PumpSmContext).set_active_state(SuctionPumpActiveState::OnPumping);
//       });
//
//   fsm.state::<StateMotorHold>()
//       .on_entry(|state, ctx| {
//         (ctx.context as PumpSmContext).set_active_state(SuctionPumpActiveState::OnHold);
//       });
//
//   fsm.state::<StatePressureNoReading>()
//       .on_entry(|state, ctx| {
//         let mut smContext = ctx.context as PumpSmContext;
//         smContext.set_is_closed(None);
//         smContext.try_stop_pressure_sensor_sampler();
//       });
//   fsm.state::<StatePressureNoReading>()
//       .on_event::<EventDoCalibrate>()
//       .transition_to::<StatePressureInitializing>();
//
//   fsm.state::<StatePressureInitializing>()
//       .on_entry(|state, ctx| {
//         let mut smContext = ctx.context as PumpSmContext;
//         smContext.set_actively_calibrating(true);
//         smContext.try_start_pressure_sensor_sampler(ctx.queue);
//       })
//       .on_exit(|state, ctx| {
//         (ctx.context as PumpSmContext).set_actively_calibrating(false);
//       });
//   fsm.state::<StatePressureInitializing>()
//       .on_event::<EventOnPressureChange>()
//       .transition_to::<StatePressureOpen>()
//       .guard(|ev, ctx| {
//         let isOpen = (ctx.context as PumpSmContext).check_pressure(ev, PressureCheck::NormalAtmosphericPressure);
//         isOpen
//       });
//   fsm.state::<StatePressureInitializing>()
//       .on_event::<EventOnPressureChange>()
//       .transition_to::<StatePressureClosed>()
//       .guard(|ev, ctx| {
//         let isOpen = (ctx.context as PumpSmContext).check_pressure(ev, PressureCheck::NormalAtmosphericPressure);
//         !isOpen
//       });
//
//   fsm.state::<StatePressureOpen>()
//       .on_entry(|state, ctx| {
//         (ctx.context as PumpSmContext).set_is_closed(Some(false));
//       });
//
//   fsm.state::<StatePressureClosed>()
//       .on_entry(|state, ctx| {
//         (ctx.context as PumpSmContext).set_is_closed(Some(true));
//       });
//
//   fsm.initial_states::<(StateMotorStopped, StatePressureNoReading)>();
//   fsm.build()
// }
//
// pub struct PumpSmContext {
//   hal: Arc<dyn SuctionPumpHal>,
//   emitted_state: Mutable<SuctionPumpState>,
//   internal_state: PumpSmInternalState,
//   pump_and_hold_at_data: Option<PumpAndHoldAtData>,
//   pressure_sensor_sampler: Option<PressureSensorSampler>,
// }
//
// #[derive(Clone, Debug)]
// pub struct PumpSmInternalState {
//   actively_calibrating: bool,
//   last_sensor_error: Option<HalError>,
//   is_closed: Option<bool>,
//   active_state: SuctionPumpActiveState,
// }
//
// impl PumpSmContext {
//   fn set_is_closed(&mut self, is_closed: Option<bool>) {
//     self.internal_state.is_closed = is_closed;
//     self.maybe_emit_state();
//   }
//
//   fn set_active_state(&mut self, active_state: SuctionPumpActiveState) {
//     self.internal_state.active_state = active_state;
//     self.maybe_emit_state();
//   }
//
//   fn set_last_sensor_error(&mut self, last_sensor_error: HalError) {
//     self.internal_state.last_sensor_error = Some(last_sensor_error.clone());
//     self.maybe_emit_state();
//   }
//
//   fn set_actively_calibrating(&mut self, actively_calibrating: bool) {
//     self.internal_state.actively_calibrating = actively_calibrating;
//     self.maybe_emit_state();
//   }
//
//   fn maybe_emit_state(&mut self) {
//     let computed = self.compute_external_state();
//     self.emitted_state.set(computed);
//   }
//
//   fn compute_external_state(&self) -> SuctionPumpState {
//     return match self.internal_state.is_closed {
//       Some(is_closed) => {
//         SuctionPumpState::Ready {
//           has_airtight_seal: is_closed,
//           active_state: self.internal_state.active_state.clone(),
//         }
//       }
//       None => {
//         match self.internal_state.actively_calibrating {
//           true => SuctionPumpState::Calibrating,
//           false => {
//             match self.internal_state.last_sensor_error {
//               Some(ref calibration_err) => {
//                 SuctionPumpState::SensorError(calibration_err.clone())
//               }
//               None => SuctionPumpState::NotCalibrated,
//             }
//           }
//         }
//       },
//     };
//   }
//
//   fn try_start_pressure_sensor_sampler(&mut self) -> anyhow::Result<()> {
//     if let Some(ref existing) = self.pressure_sensor_sampler {
//       return Err(anyhow!("Pressure sensor already active"));
//     }
//     let sampler_start = PressureSensorSampler::start(self.hal.clone().into());
//     self.pressure_sensor_sampler = Some(sampler_start.await.unwrap());
//     Ok(())
//   }
//
//   fn try_stop_pressure_sensor_sampler(&mut self) -> anyhow::Result<()> {
//     // Dropping the old reference should be enough.
//     let had_value = self.pressure_sensor_sampler.is_some();
//     self.pressure_sensor_sampler = None;
//     match had_value {
//       true => Ok(()),
//       false => Err(anyhow!("Pressure sensor not active")),
//     }
//   }
//
//   fn check_pressure(&self, change_event: &EventOnPressureChange, which_check: PressureCheck) -> bool {
//     match change_event.0 {
//       Ok(value) => {
//         let target = self.pump_and_hold_at_data.unwrap();
//         let target_min = target.pressure_range_pa.min().unwrap();
//         let target_max = target.pressure_range_pa.max().unwrap();
//         match which_check {
//           PressureCheck::PumpedTooLittle => {
//             match target.direction {
//               PumpDirection::Vacuum => value > target_max,
//               PumpDirection::Pressurize => value < target_min,
//             }
//           },
//           PressureCheck::PumpedTooMuch => {
//             match target.direction {
//               PumpDirection::Vacuum => value < target_min,
//               PumpDirection::Pressurize => value > target_max,
//             }
//           },
//           PressureCheck::NormalAtmosphericPressure => {
//             NORMAL_ATMOSPHERIC_PRESSURE.contains(&value)
//           },
//         }
//       },
//       _ => false
//     }
//   }
// }
//
// enum PressureCheck {
//   PumpedTooLittle,
//   PumpedTooMuch,
//   NormalAtmosphericPressure,
// }
//
// #[derive(Clone, Debug)]
// pub enum SuctionPumpState {
//   NotCalibrated,
//   Calibrating,
//   SensorError(HalError),
//   Ready {
//     has_airtight_seal: bool,
//     active_state: SuctionPumpActiveState,
//   }
// }
//
// #[derive(Clone, Debug)]
// pub enum SuctionPumpActiveState {
//   OnHold,
//   OnPumping,
//   OnStopped,
// }
//
// #[derive(Default)]
// pub struct StateMotorStopped;
//
// #[derive(Default)]
// pub struct StateMotorPumping;
//
// #[derive(Default)]
// pub struct StateMotorHold;
//
// #[derive(Default)]
// pub struct StatePressureNoReading;
//
// #[derive(Default)]
// pub struct StatePressureInitializing;
//
// #[derive(Default)]
// pub struct StatePressureOpen;
//
// #[derive(Default)]
// pub struct StatePressureClosed;
//
// #[derive(Clone)]
// pub struct EventDoCalibrate;
//
// #[derive(Clone)]
// pub struct EventDoPumpAndHoldAt(PumpAndHoldAtData);
//
// #[derive(Clone)]
// pub struct EventDoStop;
//
// #[derive(Clone)]
// pub struct EventOnPressureChange(HalResult<i32>);
//
// #[derive(Clone)]
// pub struct EventOnCalibrationResult(HalResult<()>);
