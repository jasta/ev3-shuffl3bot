use std::borrow::Borrow;
use std::ops::Deref;

use ev3dev_lang_rust::{Attribute, Device, Driver, Ev3Error, Ev3Result};
#[macro_use]
use ev3dev_lang_rust::findable;
use ev3dev_lang_rust::motors::{LargeMotor, MotorPort};
use ev3dev_lang_rust::sensors::{Sensor, SensorPort, TouchSensor};
use ev3dev_lang_rust_derive::{Device, Sensor};
use finny::*;
use finny::decl::{BuiltFsm, FsmBuilder};
use futures_signals::signal::{Mutable, MutableSignal, Signal};

use ev3_shuffl3bot::suction_pump_machine::MSPressureSensor;

fn main() {

}

// #[finny_fsm]
// fn build_state_machine(mut fsm: FsmBuilder<PumpSM, PumpSMContext>) -> BuiltFsm {
//   fsm.state::<StateIdleOpen>();
//   fsm.build();
// }