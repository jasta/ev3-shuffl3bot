use core::result::Result::Ok;

use ev3dev_lang_rust::{Attribute, Device, Driver};
use ev3dev_lang_rust::{Ev3Error, Ev3Result};
#[macro_use]
use ev3dev_lang_rust::findable;
use ev3dev_lang_rust::motors::LargeMotor;
use ev3dev_lang_rust::motors::MotorPort;
use ev3dev_lang_rust::sensors::{Sensor, SensorPort};
use ev3dev_lang_rust_derive::{Device, Sensor};

use crate::suction_pump_hal::{HalError, HalResult, SuctionPumpHal};
use crate::suction_pump_machine::PumpDirection;

#[derive(Debug, Clone, Device, Sensor)]
struct MSPressureSensor {
  driver: Driver,
}

impl MSPressureSensor {
  fn new(driver: Driver) -> Self {
    Self { driver }
  }

  findable! {
    "lego-sensor",
    [ "ms-pps58-mx" ],
    SensorPort,
    "MSPressureSensor",
    "in"
  }

  pub fn current_pressure_pa(&self) -> Ev3Result<i32> {
    self.ensure_mode("RAW")?;
    self.get_value0()
  }

  fn ensure_mode(&self, requested_mode: &str) -> Ev3Result<()> {
    let current_mode = self.get_mode()?;
    if current_mode != requested_mode {
      self.set_mode(requested_mode)?;
    }
    Ok(())
  }
}

struct Ev3SuctionPumpHAL {
  pump_motor: LargeMotor,
  pressure_sensor: MSPressureSensor,
}

impl Ev3SuctionPumpHAL {
  fn init(pump_motor_port: MotorPort, pressure_sensor_port: SensorPort) -> Ev3Result<Self> {
    let pump_motor = LargeMotor::get(pump_motor_port)?;
    let pressure_sensor = MSPressureSensor::get(pressure_sensor_port)?;

    Ok(Self { pump_motor, pressure_sensor })
  }
}

impl SuctionPumpHal for Ev3SuctionPumpHAL {
  fn pressure_sensor_frequency_hz(&self) -> HalResult<u32> {
    Ok(10)
  }

  fn get_pressure_pa(&self) -> HalResult<i32> {
    let pressure = self.pressure_sensor.current_pressure_pa()?;
    Ok(pressure)
  }

  fn start_pump_motor(&mut self, direction: PumpDirection) -> HalResult<()> {
    let direction_mod = match direction {
      PumpDirection::Vacuum => 1,
      PumpDirection::Pressurize => -1,
    };
    self.pump_motor.set_duty_cycle_sp(direction_mod * 100)?;
    self.pump_motor.run_direct()?;
    Ok(())
  }

  fn stop_pump_motor(&mut self) -> HalResult<()> {
    self.pump_motor.stop()?;
    Ok(())
  }
}

impl From<Ev3Error> for HalError {
  fn from(e: Ev3Error) -> Self {
    match e {
      Ev3Error::NotConnected { device, port } => {
        HalError::DeviceNotConnected(format!("{} not connected at port: {}", device, port
            .unwrap_or(String::from("<unknown>"))))
      },
      Ev3Error::MultipleMatches { device, ports } => {
        HalError::DeviceNotConnected(format!("{} found at multiple ports: {}", device, ports.join
        (", ")))
      },
      Ev3Error::InternalError { msg } => {
        HalError::InternalError(msg)
      },
    }
  }
}