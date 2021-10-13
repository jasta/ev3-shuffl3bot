use crate::suction_pump_hal::SuctionPumpHal;

pub struct SuctionPumpMachine {
  hal: Box<dyn SuctionPumpHal>,
}

impl SuctionPumpMachine {
  fn new(hal: Box<dyn SuctionPumpHal>) -> Self {
    Self { hal }
  }

  fn pump_and_hold_at(&mut self, data: PumpAndHoldAtData) {

  }

  fn stop(&mut self) {

  }
}

#[derive(Default)]
struct PumpSMContext { pump_and_hold_at_data: Option<PumpAndHoldAtData> }

/// Motor is idle; tube is open to normal atmospheric pressure
#[derive(Default)]
struct StateIdleOpen;

/// Motor is active; tube is open to normal atmospheric pressure
#[derive(Default)]
struct StateActiveOpen;

/// Motor is active; tube is closed and we are adjusting the pressure
#[derive(Default)]
struct StateActiveClosed;

/// Motor is inactive; tube is closed and we could change pressure by becoming active
#[derive(Default)]
struct StateHoldClosed;

pub enum PumpDirection {
  Vacuum,
  Pressurize,
}

pub struct PumpAndHoldAtData {
  direction: PumpDirection,
  pressure_pa: u32,
}

enum Event {
  OnPressureChange,
  DoPumpAndHoldAt { data: PumpAndHoldAtData },
  DoStop,
}
