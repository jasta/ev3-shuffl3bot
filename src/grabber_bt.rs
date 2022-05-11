use ai_behavior::{Action, Behavior, Fail, Failure, Running, Sequence, Success, Wait, WhenAny, While};
use std::time::Duration;
use crate::dynamic_action::DynamicAction;
use crate::grabber_hal;
use crate::grabber_hal::{ArmCommand, GrabberHal, PumpCommand};

pub struct GrabberBehaviourTreeFactory;

impl GrabberBehaviourTreeFactory {
    pub fn create_bt(&self) -> Behavior<GrabberAction> {
        let make_contact = Action(DynamicAction::new(|s: &mut GrabberState| {
            if s.set_pump_command(PumpCommand::StartVacuum).is_err() {
                return Failure;
            }
            if s.set_arm_command(ArmCommand::LowerToGrab).is_err() {
                return Failure;
            }
            match s.suction_state() {
                Ok(SuctionState::Contact) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let running_while_in_contact = Action(DynamicAction::new(|s: &mut GrabberState| {
            if s.hal.on_tick_while_holding().is_err() {
                return Failure;
            }
            match s.suction_state() {
                Ok(SuctionState::NoContact) => Failure,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let grab_card = Action(DynamicAction::new(|s: &mut GrabberState| {
            if s.set_arm_command(ArmCommand::Hold).is_err() {
                return Failure;
            }
            if s.set_pump_command(PumpCommand::CreateAndHoldVacuum).is_err() {
                return Failure;
            }
            match s.suction_state() {
                Ok(SuctionState::Grab) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let lift_arm = Action(DynamicAction::new(|s: &mut GrabberState| {
            if s.set_arm_command(ArmCommand::Raise).is_err() {
                return Failure;
            }
            match s.hal.is_arm_idle() {
                Ok(true) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let lower_card = Action(DynamicAction::new(|s: &mut GrabberState| {
            if s.set_arm_command(ArmCommand::LowerToDrop).is_err() {
                return Failure;
            }
            match s.hal.is_arm_idle() {
                Ok(true) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let release_card = Action(DynamicAction::new(|s: &mut GrabberState| {
            if s.set_pump_command(PumpCommand::ReverseVacuum).is_err() {
                return Failure;
            }
            match s.suction_state() {
                Ok(SuctionState::NoContact) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        Sequence(
            vec![
                self.WithTimeout(Duration::from_secs(5), make_contact),
                While(
                    Box::new(Sequence(
                        vec![
                            self.WithTimeout(Duration::from_secs(2), grab_card),
                            self.WithTimeout(Duration::from_secs(2), lift_arm.clone()),
                            Wait(Duration::from_secs(2).as_secs_f64()),
                            self.WithTimeout(Duration::from_secs(2), lower_card),
                        ])),
                    vec![running_while_in_contact]),
                self.WithTimeout(Duration::from_secs(5), release_card),
                self.WithTimeout(Duration::from_secs(2), lift_arm),
            ])
    }

    #[allow(non_snake_case)]
    fn WithTimeout(&self, period: Duration, action: Behavior<GrabberAction>) -> Behavior<GrabberAction> {
        WhenAny(vec![Fail(Box::new(Wait(period.as_secs_f64()))), action])
    }
}

pub type GrabberAction = DynamicAction<GrabberState>;

pub struct GrabberState {
    hal: Box<dyn GrabberHal>,
    arm_command: Option<ArmCommand>,
    pump_command: Option<PumpCommand>,
}

impl GrabberState {
    pub fn new(hal: Box<dyn GrabberHal>) -> Self {
        Self {
            hal,
            arm_command: None,
            pump_command: None,
        }
    }

    pub fn into_hal(self) -> Box<dyn GrabberHal> {
        self.hal
    }

    pub fn has_suction_state(&self, query: SuctionState) -> bool {
        match self.suction_state() {
            Ok(state) if state == query => true,
            _ => false,
        }
    }

    pub fn suction_state(&self) -> anyhow::Result<SuctionState> {
        match self.hal.current_pressure_pa()? {
            reading if reading <= grabber_hal::MIN_PRESSURE_GRAB => Ok(SuctionState::Grab),
            reading if reading <= grabber_hal::MIN_PRESSURE_CONTACT => Ok(SuctionState::Contact),
            _ => Ok(SuctionState::NoContact),
        }
    }

    pub fn set_arm_command(&mut self, command: ArmCommand) -> anyhow::Result<()> {
        if self.arm_command.replace(command) != Some(command) {
            self.hal.send_arm_command(command)
        } else {
            Ok(())
        }
    }

    pub fn set_pump_command(&mut self, command: PumpCommand) -> anyhow::Result<()> {
        if self.pump_command.replace(command) != Some(command) {
            self.hal.send_pump_command(command)
        } else {
            Ok(())
        }
    }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum SuctionState {
    NoContact,
    Contact,
    Grab,
}
