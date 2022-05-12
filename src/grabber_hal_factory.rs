use crate::ev3::grabber_hal_ev3::{Ev3PortSpec, GrabberHalEv3};
use crate::grabber_hal::GrabberHal;
use crate::grabber_hal_mock::GrabberHalMock;
use ev3dev_lang_rust::motors::MotorPort;
use ev3dev_lang_rust::sensors::SensorPort;
use std::path::Path;

#[derive(Default)]
pub struct GrabberHalFactory {
    force_mock: bool,
}

impl GrabberHalFactory {
    pub fn new() -> Self {
        Default::default()
    }

    pub fn new_maybe_mock(force_mock: bool) -> Self {
        Self { force_mock }
    }

    pub fn create_hal(&self) -> anyhow::Result<Box<dyn GrabberHal>> {
        if !self.force_mock && Path::new("/sys/class/tacho-motor").exists() {
            Ok(Box::new(GrabberHalEv3::new(Ev3PortSpec {
                pressure_sensor: SensorPort::In4,
                x_motor: MotorPort::OutB,
                y_motor: MotorPort::OutC,
                pump_motor: MotorPort::OutA,
                arm_motor: MotorPort::OutD,
            })?))
        } else {
            Ok(Box::new(GrabberHalMock::default()))
        }
    }
}
