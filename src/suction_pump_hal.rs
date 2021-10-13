use thiserror::Error;

use crate::suction_pump_machine::PumpDirection;

#[derive(Error, PartialEq, Clone, Debug)]
pub enum HalError {
  #[error("{0}")]
  DeviceNotConnected(String),
  #[error("{0}")]
  InternalError(String),
}

pub type HalResult<T> = Result<T, HalError>;

pub trait SuctionPumpHal {
  fn pressure_sensor_frequency_hz(&self) -> HalResult<u32>;
  fn get_pressure_pa(&self) -> HalResult<i32>;
  fn start_pump_motor(&mut self, direction: PumpDirection) -> HalResult<()>;
  fn stop_pump_motor(&mut self) -> HalResult<()>;
}
