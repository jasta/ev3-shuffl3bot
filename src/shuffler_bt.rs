use std::collections::VecDeque;
use crate::dynamic_action::DynamicAction;
use crate::shuffler_hal;
use crate::shuffler_hal::{ArmCommand, ShufflerHal, PumpCommand};
use ai_behavior::{Action, Behavior, Fail, Failure, If, Running, Sequence, Success, Wait, WaitForever, WhenAny, While};
use std::time::Duration;
use futures_signals::signal::WaitFor;
use crate::shuffle_solver::CardMove;

#[derive(Default)]
pub struct ShufflerBehaviourTreeFactory {
    options: Options,
}

#[derive(Default)]
pub struct Options {
    pub skip_moves: bool,
    pub fake_hw: bool,
}

impl ShufflerBehaviourTreeFactory {
    pub fn new(options: Options) -> Self {
        Self { options }
    }

    pub fn create_bt(&self) -> Behavior<ShufflerAction> {
        let take_next_move = Action(DynamicAction::new(|s: &mut ShufflerState| {
            s.current_move = s.moves_queue.pop_front();
            println!("Next move is: {:?} (with {} after that)", s.current_move, s.moves_queue.len());
            if s.current_move.is_some() { Success } else { Failure }
        }));

        let move_to_src_stack = self.MoveToStack(WhichStack::Src);
        let move_to_dst_stack = self.MoveToStack(WhichStack::Dst);

        let make_contact = Action(DynamicAction::new(|s: &mut ShufflerState| {
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

        let running_while_in_contact = Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.hal.on_tick_while_holding().is_err() {
                return Failure;
            }
            match s.suction_state() {
                Ok(SuctionState::NoContact) => Failure,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let grab_card = Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_arm_command(ArmCommand::Hold).is_err() {
                return Failure;
            }
            if s.set_pump_command(PumpCommand::CreateAndHoldVacuum)
                .is_err()
            {
                return Failure;
            }
            match s.suction_state() {
                Ok(SuctionState::Grab) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let lift_arm_to_confirm = Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_arm_command(ArmCommand::RaiseToConfirm).is_err() {
                return Failure;
            }
            match s.hal.did_move_arm() {
                Ok(true) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let lift_arm_to_move = Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_arm_command(ArmCommand::RaiseToMove).is_err() {
                return Failure;
            }
            match s.hal.did_move_arm() {
                Ok(true) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let lower_card = Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_arm_command(ArmCommand::LowerToDrop).is_err() {
                return Failure;
            }
            match s.hal.did_move_arm() {
                Ok(true) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let release_card = Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_pump_command(PumpCommand::ReverseVacuum).is_err() {
                return Failure;
            }
            match s.suction_state() {
                Ok(SuctionState::NoContact) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let stop_pump = Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_pump_command(PumpCommand::Stop).is_err() {
                return Failure;
            }
            Success
        }));

        let lift_confirm_wait = if self.options.fake_hw {
            Duration::from_millis(0)
        } else {
            Duration::from_millis(if self.options.skip_moves { 2000 } else { 750 })
        };

        let skip_move_with_card_wait = Duration::from_millis(if self.options.fake_hw { 0 } else { 1000 });
        let skip_move_without_card_wait = Duration::from_millis(if self.options.fake_hw { 0 } else { 2000 });

        let do_move = Sequence(vec![
            take_next_move,
            self.IfSkipMoves(
                self.DoNothing(),
                self.WithTimeout(Duration::from_secs(5), move_to_src_stack)),
            self.WithTimeout(Duration::from_secs(3), make_contact),
            While(
                Box::new(Sequence(vec![
                    self.WithTimeout(Duration::from_secs(4), grab_card),
                    self.WithTimeout(Duration::from_secs(4), lift_arm_to_confirm),
                    Wait(lift_confirm_wait.as_secs_f64()),
                    self.WithTimeout(Duration::from_secs(4), lift_arm_to_move.clone()),
                    self.IfSkipMoves(
                        Wait(skip_move_with_card_wait.as_secs_f64()),
                        self.WithTimeout(Duration::from_secs(6), move_to_dst_stack)),
                    self.WithTimeout(Duration::from_secs(4), lower_card),
                ])),
                vec![running_while_in_contact],
            ),
            self.WithTimeout(Duration::from_secs(5), release_card),
            self.WithTimeout(Duration::from_secs(4), lift_arm_to_move),
            self.IfSkipMoves(
                Sequence(vec![stop_pump, Wait(skip_move_without_card_wait.as_secs_f64())]),
                self.DoNothing()),
        ]);

        While(Box::new(WaitForever), vec![do_move])
    }

    #[allow(non_snake_case)]
    fn MoveToStack(&self, which_stack: WhichStack) -> Behavior<ShufflerAction> {
        Action(DynamicAction::new(move |s: &mut ShufflerState| {
            let current_move = s.current_move.as_ref().unwrap();
            let stack_index = match which_stack {
                WhichStack::Src => current_move.src_stack,
                WhichStack::Dst => current_move.dst_stack,
            };
            if s.set_move_to_stack_command(stack_index).is_err() {
                return Failure;
            }
            match s.hal.did_move_to_rowcol() {
                Ok(true) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }))
    }

    #[allow(non_snake_case)]
    fn IfSkipMoves(
        &self,
        if_true: Behavior<ShufflerAction>,
        if_false: Behavior<ShufflerAction>,
    ) -> Behavior<ShufflerAction> {
        let skip_moves = self.options.skip_moves;
        If(
            Box::new(Action(DynamicAction::new(move |_s: &mut ShufflerState| {
                if skip_moves { Success } else { Failure }
            }))),
            Box::new(if_true),
            Box::new(if_false))
    }

    #[allow(non_snake_case)]
    fn DoNothing(&self) -> Behavior<ShufflerAction> {
        Action(DynamicAction::new(|_s: &mut ShufflerState| {
            Success
        }))
    }

    #[allow(non_snake_case)]
    fn WithTimeout(
        &self,
        period: Duration,
        action: Behavior<ShufflerAction>,
    ) -> Behavior<ShufflerAction> {
        While(Box::new(action), vec![Fail(Box::new(Wait(period.as_secs_f64())))])
    }
}

pub type ShufflerAction = DynamicAction<ShufflerState>;

pub enum WhichStack {
    Src,
    Dst,
}

pub struct ShufflerState {
    hal: Box<dyn ShufflerHal>,
    moves_queue: VecDeque<CardMove>,
    current_move: Option<CardMove>,
    num_rows: usize,
    row_move_command: Option<usize>,
    col_move_command: Option<usize>,
    arm_command: Option<ArmCommand>,
    pump_command: Option<PumpCommand>,
}

impl ShufflerState {
    pub fn new(hal: Box<dyn ShufflerHal>, moves_queue: VecDeque<CardMove>, num_rows: usize) -> Self {
        Self {
            hal,
            moves_queue,
            current_move: None,
            num_rows,
            row_move_command: None,
            col_move_command: None,
            arm_command: None,
            pump_command: None,
        }
    }

    pub fn is_really_success(&self) -> bool {
        self.moves_queue.is_empty() && self.current_move.is_none()
    }

    pub fn into_hal(self) -> Box<dyn ShufflerHal> {
        self.hal
    }

    pub fn set_move_to_stack_command(&mut self, stack_index: usize) -> anyhow::Result<()> {
        let row = stack_index / self.num_rows;
        let col = stack_index % self.num_rows;
        println!("set_move_to_stack_command: {stack_index}, row={row}, col={col}");

        if self.row_move_command.replace(row) != Some(row) {
            self.hal.send_move_to_row_command(row)?;
        }
        if self.col_move_command.replace(col) != Some(col) {
            self.hal.send_move_to_col_command(col)?;
        }

        Ok(())
    }

    pub fn has_suction_state(&self, query: SuctionState) -> bool {
        match self.suction_state() {
            Ok(state) if state == query => true,
            _ => false,
        }
    }

    pub fn suction_state(&self) -> anyhow::Result<SuctionState> {
        match self.hal.current_pressure_pa()? {
            reading if reading <= shuffler_hal::MIN_PRESSURE_GRAB => Ok(SuctionState::Grab),
            reading if reading <= shuffler_hal::MIN_PRESSURE_CONTACT => Ok(SuctionState::Contact),
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
