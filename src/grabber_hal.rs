pub const MIN_PRESSURE_CONTACT: u32 = 99000;
pub const MIN_PRESSURE_GRAB: u32 = 93000;
pub const TARGET_PRESSURE_GRAB: u32 = 80000;

pub trait GrabberHal {
    fn calibrate(&mut self) -> anyhow::Result<()>;
    fn current_pressure_pa(&self) -> anyhow::Result<u32>;
    fn send_arm_command(&mut self, command: ArmCommand) -> anyhow::Result<()>;
    fn is_arm_idle(&self) -> anyhow::Result<bool>;
    fn send_pump_command(&mut self, command: PumpCommand) -> anyhow::Result<()>;
    fn on_tick_while_holding(&mut self) -> anyhow::Result<()>;
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum ArmCommand {
    LowerToGrab,
    LowerToDrop,
    Raise,
    Hold,
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum PumpCommand {
    StartVacuum,
    CreateAndHoldVacuum,
    ReverseVacuum,
    Stop,
}
