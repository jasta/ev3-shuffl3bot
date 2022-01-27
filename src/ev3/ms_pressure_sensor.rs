use ev3dev_lang_rust::{Driver, Ev3Result};
use ev3dev_lang_rust_derive::{device_macro_derive as Device, sensor_macro_derive as Sensor};

#[derive(Debug, Clone, Device, Sensor)]
pub(crate) struct MSPressureSensor {
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
