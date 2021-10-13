use std::sync::Arc;

use futures_signals::signal::MutableSignalCloned;
use thiserror::Error;

use crate::suction_pump_machine::PumpDirection;

pub type PressureSensorReading = Result<i32, HalError>;

#[derive(Error, Debug)]
pub enum HalError {
  #[error("{0}")]
  DeviceNotConnected(String),
  #[error("{0}")]
  InternalError(String),
}

pub type HalResult<T> = Result<T, HalError>;

pub trait SuctionPumpHal {
  fn pressure_sensor_signal(&self) -> MutableSignalCloned<Arc<PressureSensorReading>>;
  fn start_pump_motor(&self, direction: PumpDirection) -> HalResult<()>;
  fn stop_pump_motor(&self) -> HalResult<()>;
}
