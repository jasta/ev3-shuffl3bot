use tokio::sync::watch;
use tokio::sync::mpsc::{UnboundedReceiver, UnboundedSender};
use tokio::sync::watch::{Receiver, Sender};
use tokio::task::JoinHandle;
use tokio::time::{Duration, MissedTickBehavior};
use tokio::time::interval;

use crate::suction_pump_hal::{HalResult, SuctionPumpHal, SuctionPumpPressureSensor};

static SAMPLER: Option<PressureSensorSampler> = None;

pub struct PressureSensorSampler {
  handle: JoinHandle<()>,
  reader: Receiver<HalResult<i32>>,
}

impl PressureSensorSampler {
  pub async fn start(sensor: Box<dyn SuctionPumpPressureSensor + Send>) -> HalResult<Self> {
    let current_pressure = sensor.get_pressure_pa()?;
    let (sender, reader) = watch::channel::<HalResult<i32>>(Ok(current_pressure));
    let handle = tokio::spawn(async move {
      let freq = sensor.pressure_sensor_frequency_hz().unwrap();
      let mut interval = interval(Duration::from_millis(freq.into()));
      interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
      loop {
        interval.tick().await;
        sender.send(sensor.get_pressure_pa());
      }
    });
    Ok(Self { handle, reader })
  }

  pub fn subscribe(&self) -> Receiver<HalResult<i32>> {
    self.reader.clone()
  }
}

impl Drop for PressureSensorSampler {
  fn drop(&mut self) {
    self.handle.abort();
  }
}

#[cfg(test)]
mod tests {
  use std::cell::RefCell;

  use futures::StreamExt;
  use futures_signals::signal::SignalExt;

  use crate::suction_pump_hal::PumpDirection;

  use super::*;

  const TEST_PRESSURE_SENSOR_FREQUENCY_HZ: u32 = 10;

  struct TestSuctionPumpHal {
    values: RefCell<Box<dyn Iterator<Item=HalResult<i32>> + Send>>
  }

  impl TestSuctionPumpHal {
    fn with_test_pressure_values<I>(values_iter: I) -> Self
    where
        I: Iterator<Item=HalResult<i32>> + Send + 'static {
      Self { values: RefCell::new(Box::new(values_iter)) }
    }
  }

  impl SuctionPumpPressureSensor for TestSuctionPumpHal {
    fn pressure_sensor_frequency_hz(&self) -> HalResult<u32> {
      Ok(TEST_PRESSURE_SENSOR_FREQUENCY_HZ)
    }

    fn get_pressure_pa(&self) -> HalResult<i32> {
      self.values.borrow_mut().next().unwrap()
    }
  }

  #[tokio::test(start_paused = true)]
  async fn test_happy_path_descending_values() {
    let test_values: Vec<HalResult<i32>> = vec![Ok(100000), Ok(90000), Ok(80000)];
    let sampler = PressureSensorSampler::start(Box::new(
      TestSuctionPumpHal::with_test_pressure_values(test_values.clone().into_iter())))
        .await.unwrap();

    let mut receiver = sampler.subscribe();
    for &expected in &test_values {
      receiver.changed().await;
      assert_eq!(expected, receiver.borrow());
    }
  }
}
