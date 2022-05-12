use crate::shuffler_hal;
use crate::shuffler_hal::{ArmCommand, ShufflerHal, PumpCommand};

#[derive(Debug, Default)]
pub struct ShufflerHalMock {
    row_move_command: Option<usize>,
    col_move_command: Option<usize>,
    arm_command: Option<ArmCommand>,
    pump_command: Option<PumpCommand>,
}

impl ShufflerHal for ShufflerHalMock {
    fn calibrate_gantry(&mut self) -> anyhow::Result<()> {
        println!("Boop.");
        Ok(())
    }

    fn calibrate_grabber(&mut self) -> anyhow::Result<()> {
        println!("Beep.");
        Ok(())
    }

    fn current_pressure_pa(&self) -> anyhow::Result<u32> {
        let answer = match self.pump_command.unwrap_or(PumpCommand::Stop) {
            PumpCommand::StartVacuum => match self.arm_command.unwrap_or(ArmCommand::Hold) {
                ArmCommand::LowerToGrab => shuffler_hal::MIN_PRESSURE_CONTACT,
                _ => u32::MAX,
            },
            PumpCommand::CreateAndHoldVacuum => shuffler_hal::MIN_PRESSURE_GRAB,
            PumpCommand::ReverseVacuum => u32::MAX,
            PumpCommand::Stop => u32::MAX,
        };
        println!("current_pressure_pa: {answer}");
        Ok(answer)
    }

    fn send_move_to_row_command(&mut self, row: usize) -> anyhow::Result<()> {
        println!("send_move_to_row_command: {row}");
        self.row_move_command = Some(row);
        Ok(())
    }

    fn send_move_to_col_command(&mut self, col: usize) -> anyhow::Result<()> {
        println!("send_move_to_col_command: {col}");
        self.col_move_command = Some(col);
        Ok(())
    }

    fn did_move_to_rowcol(&self) -> anyhow::Result<bool> {
        let answer = self.row_move_command.or(self.col_move_command).is_some();
        println!("did_move_to_rowcol: {answer}");
        Ok(answer)
    }

    fn send_arm_command(&mut self, command: ArmCommand) -> anyhow::Result<()> {
        println!("send_arm_command: {command:?}");
        self.arm_command = Some(command);
        Ok(())
    }

    fn did_move_arm(&self) -> anyhow::Result<bool> {
        let answer = match self.arm_command.unwrap_or(ArmCommand::Hold) {
            ArmCommand::LowerToGrab => false,
            ArmCommand::LowerToDrop => true,
            ArmCommand::RaiseToMove => true,
            ArmCommand::JiggleHigh => true,
            ArmCommand::JiggleLow => true,
            ArmCommand::Hold => false,
        };
        println!("is_arm_idle: {answer}");
        Ok(answer)
    }

    fn send_pump_command(&mut self, command: PumpCommand) -> anyhow::Result<()> {
        println!("send_pump_command: {command:?}");
        self.pump_command = Some(command);
        Ok(())
    }

    fn on_tick_while_holding(&mut self) -> anyhow::Result<()> {
        println!("on_tick_while_grabbed");
        Ok(())
    }

    fn dump(&self) -> anyhow::Result<()> {
        println!("{self:?}");
        Ok(())
    }
}
