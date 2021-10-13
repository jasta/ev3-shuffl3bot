use futures_signals::signal::Mutable;
use tokio::sync;
use tokio::sync::mpsc::{UnboundedReceiver, UnboundedSender};
use tokio::task::JoinHandle;
use tokio::time::{Duration, MissedTickBehavior};
use tokio::time::interval;

use crate::suction_pump_hal::{HalResult, SuctionPumpHal};

pub struct PressureSensorSampler {
  handle: JoinHandle<()>,
  pub reading: Mutable<HalResult<i32>>,
  shutdown: UnboundedSender<()>,
}

impl PressureSensorSampler {
  pub async fn start(hal: Box<dyn SuctionPumpHal + Send>) -> HalResult<Self> {
    let current_pressure = hal.get_pressure_pa()?;
    let reading = Mutable::new(Ok(current_pressure));
    let reading_for_async = reading.clone();
    let (shutdown_tx, shutdown_rx) = sync::mpsc::unbounded_channel::<()>();
    let handle = tokio::spawn(async move {
      run_pressure_sensor_loop(hal, reading_for_async, shutdown_rx).await;
      println!("Shutting down...");
    });
    Ok(Self { handle, reading, shutdown: shutdown_tx })
  }
}

async fn run_pressure_sensor_loop(
    hal: Box<dyn SuctionPumpHal + Send>,
    reading: Mutable<HalResult<i32>>,
    mut shutdown: UnboundedReceiver<()>) {
  let freq = hal.pressure_sensor_frequency_hz().unwrap();
  let mut interval = interval(Duration::from_millis(freq.into()));
  interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
  loop {
    tokio::select! {
      _ = interval.tick() => {
        reading.set(hal.get_pressure_pa());
      },
      _ = shutdown.recv() => {
        return;
      },
    }
  }
}

impl Drop for PressureSensorSampler {
  fn drop(&mut self) {
    self.shutdown.send(()).unwrap();
  }
}

enum Message {
  Terminate,
}

#[cfg(test)]
mod tests {
  use std::cell::RefCell;

  use futures::StreamExt;
  use futures_signals::signal::SignalExt;

  use crate::suction_pump_machine::PumpDirection;

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

  impl SuctionPumpHal for TestSuctionPumpHal {
    fn pressure_sensor_frequency_hz(&self) -> HalResult<u32> {
      Ok(TEST_PRESSURE_SENSOR_FREQUENCY_HZ)
    }

    fn get_pressure_pa(&self) -> HalResult<i32> {
      self.values.borrow_mut().next().unwrap()
    }

    fn start_pump_motor(&mut self, direction: PumpDirection) -> HalResult<()> {
      panic!();
    }

    fn stop_pump_motor(&mut self) -> HalResult<()> {
      panic!();
    }
  }

  #[tokio::test(start_paused = true)]
  async fn test_happy_path_descending_values() {
    let test_values: Vec<HalResult<i32>> = vec![Ok(100000), Ok(90000), Ok(80000)];
    let sampler = PressureSensorSampler::start(Box::new(
      TestSuctionPumpHal::with_test_pressure_values(test_values.clone().into_iter())))
        .await.unwrap();

    let mut stream = sampler.reading.signal_cloned().to_stream();
    for expected in &test_values {
      let actual = stream.next().await.unwrap();
      assert_eq!(*expected, actual);
    }
  }
}
