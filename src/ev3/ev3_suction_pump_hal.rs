use core::result::Result::Ok;
use std::sync::Arc;

use ev3dev_lang_rust::{Attribute, Device, Driver};
use ev3dev_lang_rust::{Ev3Error, Ev3Result};
#[macro_use]
use ev3dev_lang_rust::findable;
use ev3dev_lang_rust::motors::LargeMotor;
use ev3dev_lang_rust::motors::MotorPort;
use ev3dev_lang_rust::sensors::{Sensor, SensorPort};
use ev3dev_lang_rust_derive::{Device, Sensor};
use futures_signals::signal::{Mutable, MutableSignalCloned};
use tokio::time;
use tokio::time::{Duration, MissedTickBehavior};

use crate::suction_pump_hal::{HalError, HalResult, PressureSensorReading, SuctionPumpHal};
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

struct PressureSensorSampler {
  sensor: MSPressureSensor,
  reading: Mutable<Arc<PressureSensorReading>>,
}

impl PressureSensorSampler {
  pub fn spawn_forever(self) {
    tokio::spawn(async move {
      self.run_pressure_sensor_loop();
    });
  }

  async fn run_pressure_sensor_loop(&self) {
    let mut interval = time::interval(Duration::from_millis(10));
    interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
    loop {
      interval.tick().await;
      let reading = self.sensor.current_pressure_pa();
      self.reading.set(Arc::new(reading.map_err(|e| e.into())));
    }
  }
}

struct Ev3SuctionPumpHAL {
  pump_motor: LargeMotor,
  pressure_reading: Mutable<Arc<PressureSensorReading>>,
}

impl Ev3SuctionPumpHAL {
  fn init(pump_motor_port: MotorPort, pressure_sensor_port: SensorPort) -> Ev3Result<Self> {
    let pump_motor = LargeMotor::get(pump_motor_port)?;
    let pressure_sensor = MSPressureSensor::get(pressure_sensor_port)?;

    let initial_reading = pressure_sensor.current_pressure_pa()?;
    let pressure_reading =
        Mutable::new(Arc::new(Ok(initial_reading)));

    let pressure_sensor_sampler = PressureSensorSampler {
      sensor: pressure_sensor,
      reading: pressure_reading.clone(),
    };
    pressure_sensor_sampler.spawn_forever();
    Ok(Self { pump_motor, pressure_reading: pressure_reading.clone() })
  }

  fn start_pump_motor_internal(&self, direction: PumpDirection) -> Ev3Result<()> {
    let direction_mod = match direction {
      PumpDirection::Vacuum => 1,
      PumpDirection::Pressurize => -1,
    };
    self.pump_motor.set_duty_cycle_sp(direction_mod * 100)?;
    self.pump_motor.run_direct()
  }

  fn stop_pump_motor_internal(&self) -> Ev3Result<()> {
    self.pump_motor.stop()
  }
}

impl SuctionPumpHal for Ev3SuctionPumpHAL {
  fn pressure_sensor_signal(&self) -> MutableSignalCloned<Arc<PressureSensorReading>> {
    self.pressure_reading.signal_cloned()
  }

  fn start_pump_motor(&self, direction: PumpDirection) -> HalResult<()> {
    self.start_pump_motor_internal(direction)?;
    Ok(())
  }

  fn stop_pump_motor(&self) -> HalResult<()> {
    self.stop_pump_motor_internal()?;
    Ok(())
  }
}

impl From<Ev3Error> for HalError {
  fn from(e: Ev3Error) -> Self {
    match e {
      Ev3Error::NotConnected { device, port } =>
        HalError::DeviceNotConnected(format!("{} not connected at port: {}", device, port
            .unwrap_or(String::from("<unknown>")))),
      Ev3Error::MultipleMatches { device, ports } =>
        HalError::DeviceNotConnected(format!("{} found at multiple ports: {}", device, ports.join
        (", "))),
      Ev3Error::InternalError { msg } => HalError::InternalError(msg),
    }
  }
}