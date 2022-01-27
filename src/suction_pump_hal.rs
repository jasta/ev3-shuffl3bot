use thiserror::Error;

#[derive(Error, PartialEq, Clone, Debug)]
pub enum HalError {
  #[error("{0}")]
  DeviceNotConnected(String),
  #[error("{0}")]
  InternalError(String),
}

pub type HalResult<T> = Result<T, HalError>;

pub trait SuctionPumpMotor {
  fn start_pump_motor(&mut self, direction: PumpDirection) -> HalResult<()>;
  fn stop_pump_motor(&mut self) -> HalResult<()>;
}

pub trait SuctionPumpPressureSensor {
  fn pressure_sensor_frequency_hz(&self) -> HalResult<u32>;
  fn get_pressure_pa(&self) -> HalResult<i32>;
}

pub struct SuctionPumpHal {
  pub motor: Box<dyn SuctionPumpMotor + Send>,
  pub pressure_sensor: Box<dyn SuctionPumpPressureSensor + Send>,
}

#[derive(Clone, Debug)]
pub enum PumpDirection {
  Vacuum,
  Pressurize,
}
