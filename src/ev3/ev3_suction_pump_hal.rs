use core::result::Result::Ok;

use ev3dev_lang_rust::{Attribute, Device, Driver};
use ev3dev_lang_rust::{Ev3Error, Ev3Result};
use ev3dev_lang_rust::findable;
use ev3dev_lang_rust::motors::LargeMotor;
use ev3dev_lang_rust::motors::MotorPort;
use ev3dev_lang_rust::sensors::{Sensor, SensorPort};
use ev3dev_lang_rust_derive::{Device, Sensor};

use crate::ev3::ms_pressure_sensor::MSPressureSensor;
use crate::suction_pump_hal::{HalError, HalResult, SuctionPumpHal, SuctionPumpMotor, SuctionPumpPressureSensor};
use crate::suction_pump_hal::PumpDirection;

struct Ev3SuctionPumpHAL;

impl Ev3SuctionPumpHAL {
  fn init(pump_motor_port: MotorPort, pressure_sensor_port: SensorPort) -> Ev3Result<SuctionPumpHal> {
    let pump_motor = LargeMotor::get(pump_motor_port)?;
    let pressure_sensor = MSPressureSensor::get(pressure_sensor_port)?;

    Ok(SuctionPumpHal {
      motor: Box::new(Ev3SuctionPumpMotor { pump_motor }),
      pressure_sensor: Box::new(Ev3SuctionPumpPressureSensor { pressure_sensor }),
    })
  }
}

struct Ev3SuctionPumpPressureSensor {
  pressure_sensor: MSPressureSensor,
}
impl SuctionPumpPressureSensor for Ev3SuctionPumpPressureSensor {
  fn pressure_sensor_frequency_hz(&self) -> HalResult<u32> {
    Ok(10)
  }

  fn get_pressure_pa(&self) -> HalResult<i32> {
    let pressure = self.pressure_sensor.current_pressure_pa()?;
    Ok(pressure)
  }
}

struct Ev3SuctionPumpMotor {
  pump_motor: LargeMotor,
}

impl SuctionPumpMotor for Ev3SuctionPumpMotor {
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