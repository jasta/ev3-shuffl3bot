use crate::grabber_hal;
use crate::grabber_hal::{ArmCommand, GrabberHal, PumpCommand};

#[derive(Default)]
pub struct GrabberHalMock {
    arm_command: Option<ArmCommand>,
    pump_command: Option<PumpCommand>,
}

impl GrabberHal for GrabberHalMock {
    fn calibrate(&mut self) -> anyhow::Result<()> {
        println!("Beep.");
        Ok(())
    }

    fn current_pressure_pa(&self) -> anyhow::Result<u32> {
        let answer = match self.pump_command.unwrap_or(PumpCommand::Stop) {
            PumpCommand::StartVacuum => match self.arm_command.unwrap_or(ArmCommand::Hold) {
                ArmCommand::LowerToGrab => grabber_hal::MIN_PRESSURE_CONTACT,
                _ => u32::MAX,
            },
            PumpCommand::CreateAndHoldVacuum => grabber_hal::MIN_PRESSURE_GRAB,
            PumpCommand::ReverseVacuum => u32::MAX,
            PumpCommand::Stop => u32::MAX,
        };
        println!("current_pressure_pa: {answer}");
        Ok(answer)
    }

    fn send_arm_command(&mut self, command: ArmCommand) -> anyhow::Result<()> {
        println!("send_arm_command: {command:?}");
        self.arm_command = Some(command);
        Ok(())
    }

    fn is_arm_idle(&self) -> anyhow::Result<bool> {
        let answer = match self.arm_command.unwrap_or(ArmCommand::Hold) {
            ArmCommand::LowerToGrab => false,
            ArmCommand::LowerToDrop => true,
            ArmCommand::Raise => true,
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
}
