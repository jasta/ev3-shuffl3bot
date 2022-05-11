use crate::ev3::grabber_hal_ev3::{Ev3PortSpec, GrabberHalEv3};
use crate::grabber_hal::GrabberHal;
use crate::grabber_hal_mock::GrabberHalMock;
use ev3dev_lang_rust::motors::MotorPort;
use ev3dev_lang_rust::sensors::SensorPort;
use std::path::Path;

pub struct GrabberHalFactory;

impl GrabberHalFactory {
    pub fn create_hal(&self) -> anyhow::Result<Box<dyn GrabberHal>> {
        if Path::new("/sys/class/tacho-motor").exists() {
            Ok(Box::new(GrabberHalEv3::new(Ev3PortSpec {
                pressure_sensor: SensorPort::In4,
                pump_motor: MotorPort::OutA,
                arm_motor: MotorPort::OutD,
            })?))
        } else {
            Ok(Box::new(GrabberHalMock::default()))
        }
    }
}
