use std::collections::VecDeque;
use std::fmt::Debug;
use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::path::Path;
use std::time::Duration;

use ai_behavior::{Action, Behavior, Fail, Failure, If, Running, Select, Sequence, Success, Wait, WaitForever, While};
use log::{debug, info};
use serde::{Deserialize, Serialize};

use crate::dynamic_action::DynamicAction;
use crate::shuffle_solver::{CardMove};
use crate::shuffler_hal;
use crate::shuffler_hal::{ArmCommand, PumpCommand, ShufflerHal};

pub struct ShufflerBehaviourTreeLibrary;

#[allow(non_snake_case)]
impl ShufflerBehaviourTreeLibrary {
    /// Main BT to do the shuffling (and also for some reason the grabber test???)
    pub fn MoveCards(&self, skip_moves: bool, fake_hw: bool) -> Behavior<ShufflerAction> {
        let do_move = Sequence(vec![
            self.TakeNextMove(),
            self.IfBool(
                skip_moves,
                self.DoNothing(),
                self.MoveToStack(WhichStack::Src)),
            Select(vec![
                self.MakeContact(Duration::from_secs(3)),
                // Simple test to "try again", if this works we need to improve it a bit to
                // check for a stalled motor and try some tricks...
                Sequence(vec![
                    self.StopPump(),
                    self.LiftArmToMove(),
                    self.WaitUnless(fake_hw, Duration::from_secs(1)),
                    self.MakeContact(Duration::from_secs(3))])
                ]),
            self.GrabAndMoveCardToDst(skip_moves, fake_hw),
            self.IfBool(
                skip_moves,
                Sequence(vec![
                    self.StopPump(),
                    self.WaitUnless(fake_hw, Duration::from_secs(2))]),
                self.DoNothing()),
        ]);

        self.Repeat(do_move)
    }

    pub fn MoveAllCardsToStack(&self, dst_stack: usize, fake_hw: bool) -> Behavior<ShufflerAction> {
        let do_move = Sequence(vec![
            self.MaybeSetupNextStackToClear(dst_stack),
            self.MoveToStack(WhichStack::Src),
            If(
                Box::new(self.MakeContact(Duration::from_secs(2))),
                Box::new(self.GrabAndMoveCardToDst(false, fake_hw)),
                Box::new(
                    Sequence(vec![
                        self.LiftArmToMove(),
                        self.MarkCurrentStackCleared()]))),
        ]);

        self.Repeat(do_move)
    }

    fn GrabAndMoveCardToDst(&self, skip_moves: bool, fake_hw: bool) -> Behavior<ShufflerAction> {
        Sequence(
            vec![
                self.GrabAndLiftWithRetry(fake_hw),
                While(
                    Box::new(Sequence(vec![
                        self.IfBool(
                            skip_moves,
                            self.WaitUnless(fake_hw, Duration::from_secs(1)),
                            self.MoveToStack(WhichStack::Dst)),
                    ])),
                    vec![self.RunningWhileInContact()]),
                self.ReleaseCard(),
                self.WaitUnless(fake_hw, Duration::from_millis(100)),
                self.LiftArmToMove(),
                self.StopPump(),
            ])
    }

    fn GrabAndLiftWithRetry(&self, fake_hw: bool) -> Behavior<ShufflerAction> {
        let jiggle_and_lift = While(
            Box::new(Sequence(vec![
                self.GrabCard(fake_hw),
                self.JiggleAFewTimes(),
                self.WaitUnless(fake_hw, Duration::from_millis(2000)),
                self.LiftArmToMove(),
            ])),
            vec![self.RunningWhileInContact()]);

        // Try twice in case we lost contact while jiggling (which happens sometimes when the
        // suction pump gets a little bit dirty from grabbing so many cards...)
        Select(vec![
            jiggle_and_lift.clone(),
            Sequence(vec![
                self.StopPump(),
                self.WaitUnless(fake_hw, Duration::from_secs(1)),
                self.LiftArmToMove(),
                self.MakeContact(Duration::from_secs(3)),
                jiggle_and_lift,
            ]),
        ])
    }

    fn TakeNextMove(&self) -> Behavior<ShufflerAction> {
        Action(DynamicAction::new(|s: &mut ShufflerState| {
            let moves_queue = s.rt.moves_queue.as_mut().unwrap();
            s.rt.current_move = moves_queue.pop_front();
            info!("Next move is: {:?} (with {} after that)", s.rt.current_move, moves_queue.len());
            if s.rt.current_move.is_some() { Success } else { Failure }
        }))
    }

    fn MaybeSetupNextStackToClear(&self, dst_stack: usize) -> Behavior<ShufflerAction> {
        Action(DynamicAction::new(move |s: &mut ShufflerState| {
            match s.rt.current_move {
                None => {
                    let queue = s.rt.cleanup_stacks_queue.as_mut().unwrap();
                    match queue.pop_front() {
                        Some(next) => {
                            s.rt.current_move = Some(CardMove { src_stack: next, dst_stack });
                            info!("Next moves are: {:?} (with {} after that)", s.rt.current_move, queue.len());
                            Success
                        }
                        None => Failure,
                    }
                },
                Some(_) => Success,
            }
        }))
    }

    fn MarkCurrentStackCleared(&self) -> Behavior<ShufflerAction> {
        Action(DynamicAction::new(|s: &mut ShufflerState| {
            s.rt.current_move = None;
            Success
        }))
    }

    fn MakeContact(&self, timeout: Duration) -> Behavior<ShufflerAction> {
        self.WithTimeout(timeout, Action(DynamicAction::new(|s: &mut ShufflerState| {
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
        })))
    }

    fn RunningWhileInContact(&self) -> Behavior<ShufflerAction> {
        Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.hal.on_tick_while_holding().is_err() {
                return Failure;
            }
            match s.suction_state() {
                Ok(SuctionState::NoContact) => Failure,
                Ok(_) => Running,
                _ => Failure,
            }
        }))
    }

    fn GrabCard(&self, fake_hw: bool) -> Behavior<ShufflerAction> {
        let grab_card = self.WithTimeout(Duration::from_secs(4), Action(DynamicAction::new(|s: &mut ShufflerState| {
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
        })));

        Sequence(vec![grab_card, self.WaitUnless(fake_hw, Duration::from_millis(100))])
    }

    fn JiggleAFewTimes(&self) -> Behavior<ShufflerAction> {
        let num_jiggles = 4;

        let jiggle_high = Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_arm_command(ArmCommand::JiggleHigh).is_err() {
                return Failure;
            }
            match s.hal.did_move_arm() {
                Ok(true) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        let jiggle_low = Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_arm_command(ArmCommand::JiggleLow).is_err() {
                return Failure;
            }
            match s.hal.did_move_arm() {
                Ok(true) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        }));

        self.RepeatN(num_jiggles,
            Sequence(vec![
                self.WithTimeout(Duration::from_secs(2), jiggle_low),
                self.WithTimeout(Duration::from_secs(2), jiggle_high)]))
    }

    fn LiftArmToMove(&self) -> Behavior<ShufflerAction> {
        self.WithTimeout(Duration::from_secs(4), Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_arm_command(ArmCommand::RaiseToMove).is_err() {
                return Failure;
            }
            match s.hal.did_move_arm() {
                Ok(true) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        })))
    }

    fn LowerCard(&self) -> Behavior<ShufflerAction> {
        self.WithTimeout(Duration::from_secs(4), Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_arm_command(ArmCommand::LowerToDrop).is_err() {
                return Failure;
            }
            match s.hal.did_move_arm() {
                Ok(true) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        })))
    }

    fn ReleaseCard(&self) -> Behavior<ShufflerAction> {
        self.WithTimeout(Duration::from_secs(5), Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_pump_command(PumpCommand::ReverseVacuum).is_err() {
                return Failure;
            }
            match s.suction_state() {
                Ok(SuctionState::NoContact) => Success,
                Ok(_) => Running,
                _ => Failure,
            }
        })))
    }

    fn StopPump(&self) -> Behavior<ShufflerAction> {
        Action(DynamicAction::new(|s: &mut ShufflerState| {
            if s.set_pump_command(PumpCommand::Stop).is_err() {
                return Failure;
            }
            Success
        }))
    }

    fn RepeatN(&self, n: usize, behaviour: Behavior<ShufflerAction>) -> Behavior<ShufflerAction> {
        // Hehe, pretty lame way to do it, but whatever :P
        Sequence(vec![behaviour; n])
    }

    fn MoveToStack(&self, which_stack: WhichStack) -> Behavior<ShufflerAction> {
        self.WithTimeout(Duration::from_secs(6), self.WithTimeout(Duration::from_secs(5), Action(DynamicAction::new(move |s: &mut ShufflerState| {
            let current_move = s.rt.current_move.as_ref().unwrap();
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
        }))))
    }

    fn Repeat(&self, action: Behavior<ShufflerAction>) -> Behavior<ShufflerAction> {
        While(Box::new(WaitForever), vec![action])
    }

    fn IfBool(
        &self,
        cond: bool,
        if_true: Behavior<ShufflerAction>,
        if_false: Behavior<ShufflerAction>,
    ) -> Behavior<ShufflerAction> {
        If(
            Box::new(Action(DynamicAction::new(move |_s: &mut ShufflerState| {
                if cond { Success } else { Failure }
            }))),
            Box::new(if_true),
            Box::new(if_false))
    }

    fn WaitUnless(&self, cond: bool, duration: Duration) -> Behavior<ShufflerAction> {
        self.IfBool(
            !cond,
            Wait(duration.as_secs_f64()),
            self.DoNothing())
    }

    fn DoNothing(&self) -> Behavior<ShufflerAction> {
        Action(DynamicAction::new(|_s: &mut ShufflerState| {
            Success
        }))
    }

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
    rt: RuntimeState,
}

#[derive(Serialize, Deserialize)]
struct RuntimeState {
    moves_queue: Option<VecDeque<CardMove>>,
    cleanup_stacks_queue: Option<VecDeque<usize>>,
    current_move: Option<CardMove>,
    num_rows: usize,
    row_move_command: Option<usize>,
    col_move_command: Option<usize>,
    arm_command: Option<ArmCommand>,
    pump_command: Option<PumpCommand>,
}

impl ShufflerState {
    pub fn from_save_state(hal: Box<dyn ShufflerHal>, state_in: impl AsRef<Path>) -> anyhow::Result<Self> {
        let file = File::open(state_in)?;
        let reader = BufReader::new(file);
        let mut rt: RuntimeState = serde_json::from_reader(reader)?;
        let mut me = Self { hal, rt };
        me.scrub_save_state();
        Ok(me)
    }

    pub fn new(args: ShuffleStateArgs) -> Self {
        if !(args.moves_queue.is_some() ^ args.cleanup_stacks_queue.is_some()) {
            panic!("Either moves_queue or cleanup_stacks_queue, not both!");
        }
        Self {
            hal: args.hal,
            rt: RuntimeState {
                moves_queue: args.moves_queue,
                cleanup_stacks_queue: args.cleanup_stacks_queue,
                current_move: None,
                num_rows: args.num_rows,
                row_move_command: None,
                col_move_command: None,
                arm_command: None,
                pump_command: None,
            },
        }
    }

    fn scrub_save_state(&mut self) {
        if let Some(current_move) = self.rt.current_move.take() {
            if let Some(ref mut moves_queue) = self.rt.moves_queue {
                moves_queue.push_front(current_move);
            }
        }
        self.rt.row_move_command = None;
        self.rt.col_move_command = None;
        self.rt.arm_command = None;
        self.rt.pump_command = None;
    }

    pub fn is_really_success(&self) -> bool {
        let queue_is_empty =
            self.rt.moves_queue.as_ref().map(|q| q.is_empty())
                .or_else(|| self.rt.cleanup_stacks_queue.as_ref().map(|q| q.is_empty()))
                .unwrap();
        queue_is_empty && self.rt.current_move.is_none()
    }

    pub fn set_move_to_stack_command(&mut self, stack_index: usize) -> anyhow::Result<()> {
        let row = stack_index / self.rt.num_rows;
        let col = stack_index % self.rt.num_rows;

        if self.rt.row_move_command.replace(row) != Some(row) {
            self.hal.send_move_to_row_command(row)?;
        }
        if self.rt.col_move_command.replace(col) != Some(col) {
            self.hal.send_move_to_col_command(col)?;
        }

        Ok(())
    }

    pub fn has_suction_state(&self, query: SuctionState) -> bool {
        self.suction_state().ok() == Some(query)
    }

    pub fn suction_state(&self) -> anyhow::Result<SuctionState> {
        match self.hal.current_pressure_pa()? {
            reading if reading <= shuffler_hal::MIN_PRESSURE_GRAB => Ok(SuctionState::Grab),
            reading if reading <= shuffler_hal::MIN_PRESSURE_CONTACT => Ok(SuctionState::Contact),
            _ => Ok(SuctionState::NoContact),
        }
    }

    pub fn set_arm_command(&mut self, command: ArmCommand) -> anyhow::Result<()> {
        if self.rt.arm_command.replace(command) != Some(command) {
            self.hal.send_arm_command(command)
        } else {
            Ok(())
        }
    }

    pub fn set_pump_command(&mut self, command: PumpCommand) -> anyhow::Result<()> {
        if self.rt.pump_command.replace(command) != Some(command) {
            self.hal.send_pump_command(command)
        } else {
            Ok(())
        }
    }

    pub fn dump(&self) {
        debug!("Dumping all state:");

        debug!("Hal state:");
        if let Err(e) = self.hal.dump() {
            debug!("<error: {e:?}>");
        }

        debug!("Machine state:");
        debug!("moves_queue.len: {:?}", self.rt.moves_queue.as_ref().map(|q| q.len()));
        debug!("cleanup_stacks_queue.len: {:?}", self.rt.cleanup_stacks_queue.as_ref().map(|q| q.len()));
        debug!("current_move: {:?}", self.rt.current_move);
        debug!("num_rows: {}", self.rt.num_rows);
        debug!("row_move_command: {:?}", self.rt.row_move_command);
        debug!("col_move_command: {:?}", self.rt.col_move_command);
        debug!("arm_command: {:?}", self.rt.arm_command);
        debug!("pump_command: {:?}", self.rt.pump_command);
    }

    pub fn save(&self, state_out: impl AsRef<Path>) -> anyhow::Result<()> {
        let file = File::create(state_out)?;
        let writer = BufWriter::new(file);
        serde_json::to_writer(writer, &self.rt)?;
        Ok(())
    }
}

pub struct ShuffleStateArgs {
    pub hal: Box<dyn ShufflerHal>,
    pub moves_queue: Option<VecDeque<CardMove>>,
    pub cleanup_stacks_queue: Option<VecDeque<usize>>,
    pub num_rows: usize,
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum SuctionState {
    NoContact,
    Contact,
    Grab,
}
