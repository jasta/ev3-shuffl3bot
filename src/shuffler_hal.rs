use std::fmt::Debug;

pub const MIN_PRESSURE_CONTACT: u32 = 99000;
pub const MIN_PRESSURE_GRAB: u32 = 75000;
pub const TARGET_PRESSURE_GRAB: u32 = 65000;

pub trait ShufflerHal {
    fn calibrate_gantry(&mut self) -> anyhow::Result<()>;
    fn calibrate_grabber(&mut self) -> anyhow::Result<()>;
    fn current_pressure_pa(&self) -> anyhow::Result<u32>;
    fn send_move_to_row_command(&mut self, row: usize) -> anyhow::Result<()>;
    fn send_move_to_col_command(&mut self, col: usize) -> anyhow::Result<()>;
    fn did_move_to_rowcol(&self) -> anyhow::Result<bool>;
    fn send_arm_command(&mut self, command: ArmCommand) -> anyhow::Result<()>;
    fn did_move_arm(&self) -> anyhow::Result<bool>;
    fn send_pump_command(&mut self, command: PumpCommand) -> anyhow::Result<()>;
    fn on_tick_while_holding(&mut self) -> anyhow::Result<()>;
    fn dump(&self) -> anyhow::Result<()>;
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum ArmCommand {
    LowerToGrab,
    LowerToDrop,
    RaiseToMove,
    JiggleHigh,
    JiggleLow,
    Hold,
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum PumpCommand {
    StartVacuum,
    CreateAndHoldVacuum,
    ReverseVacuum,
    Stop,
}
