//! Stress test the grabber mechanism by executing the following sequence in a loop:
//!
//! 1. Lower arm
//! 2. Start suction pump
//! 3. Contact card
//! 4. "Grab" card (using suction)
//! 5. Lift card
//! 6. Reverse suction pump, release card
//!
//! This allows us to monitor the machine and confirm that this mechanism is working as expected.

use std::path::Path;
use std::rc::Rc;
use std::{env, io, thread};
use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use ai_behavior::{Action, Behavior, Fail, Failure, Running, Sequence, State, Status, Success, Wait, WhenAny, While};
use anyhow::anyhow;
use clap::lazy_static;
use ev3dev_lang_rust::motors::{LargeMotor, MediumMotor, MotorPort};
use ev3dev_lang_rust::sensors::SensorPort;
use input::{Event, UpdateArgs};

use ev3::ms_pressure_sensor::MSPressureSensor;
use ev3_shuffl3bot::ev3;

const TICK_INTERVAL: Duration = Duration::from_millis(20);

fn main() -> anyhow::Result<()> {
    let args: Vec<_> = env::args().collect();
    let num_runs_str = args.get(1).cloned().unwrap_or_else(|| "50".to_owned());
    let num_runs: u32 = num_runs_str.parse()?;
    let mut hal = create_hal()?;
    hal.calibrate()?;
    let bt = BehaviourTreeFactory.create_bt();

    for i in 0..num_runs {
        println!("Starting run #{i}...");
        hal = run_machine(bt.clone(), hal)?;
    }
    println!("Successful stress test!");
    Ok(())
}

fn create_hal() -> anyhow::Result<Box<dyn GrabberHal>> {
    if Path::new("/sys/class/tacho-motor").exists() {
        Ok(Box::new(GrabberHalEv3 {
            pressure_sensor: MSPressureSensor::get(SensorPort::In4)?,
            pump_motor: LargeMotor::get(MotorPort::OutA)?,
            arm_motor: MediumMotor::get(MotorPort::OutD)?,
        }))
    } else {
        Ok(Box::new(GrabberHalMock::default()))
    }
}

fn run_machine(behaviour: Behavior<GrabberAction>, hal: Box<dyn GrabberHal>) -> anyhow::Result<Box<dyn GrabberHal>> {
    let mut machine: State<GrabberAction, ()> = State::new(behaviour);
    let mut state = GrabberState::new(hal);

    let mut dt = 0.0;
    let mut ticks = 0;
    let result = loop {
        let start = Instant::now();
        let e: Event = UpdateArgs { dt }.into();
        let (status, _) = machine.event(&e, &mut |args| {
            (args.action.action.handle(&mut state), args.dt)
        });

        // TODO: This of course should be a proper RT interval!
        thread::sleep(TICK_INTERVAL);
        dt = start.elapsed().as_secs_f64();
        ticks += 1;

        match status {
            Success => break Ok(()),
            Failure => break Err(anyhow!("Unknown state failure!")),
            Running => {
                // Print that a tick happened but keep going...
                if ticks % 60 != 0 {
                    print!(".");
                    io::stdout().flush()?;
                } else {
                    println!();
                }
            }
        }
    };
    println!();

    // Oh hell, the ownership of the HAL isn't great but it's not worth fixing right now for this
    // tool.
    result.map(|_| state.into_hal())
}

struct BehaviourTreeFactory;

trait GrabberHal {
    fn calibrate(&mut self) -> anyhow::Result<()>;
    fn current_pressure_pa(&self) -> anyhow::Result<u32>;
    fn send_arm_command(&mut self, command: ArmCommand) -> anyhow::Result<()>;
    fn is_arm_idle(&self) -> anyhow::Result<bool>;
    fn send_pump_command(&mut self, command: PumpCommand) -> anyhow::Result<()>;
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
enum SuctionState {
    NoContact,
    Contact,
    Grab,
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
enum ArmCommand {
    LowerToGrab,
    LowerToDrop,
    Raise,
    Hold,
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
enum PumpCommand {
    CreateVacuum,
    ReleaseVacuum,
    Stop,
}

#[derive(Default)]
struct GrabberHalMock {
    arm_command: Option<ArmCommand>,
    pump_command: Option<PumpCommand>,
}

const MIN_PRESSURE_CONTACT: u32 = 100000;
const MIN_PRESSURE_GRAB: u32 = 80000;

impl GrabberHal for GrabberHalMock {
    fn calibrate(&mut self) -> anyhow::Result<()> {
        println!("Beep.");
        Ok(())
    }

    fn current_pressure_pa(&self) -> anyhow::Result<u32> {
        let answer = match self.pump_command.unwrap_or(PumpCommand::Stop) {
            PumpCommand::CreateVacuum => {
                match self.arm_command.unwrap_or(ArmCommand::Hold) {
                    ArmCommand::LowerToGrab => MIN_PRESSURE_CONTACT,
                    ArmCommand::LowerToDrop => MIN_PRESSURE_GRAB,
                    ArmCommand::Raise => MIN_PRESSURE_GRAB,
                    ArmCommand::Hold => MIN_PRESSURE_GRAB,
                }
            }
            PumpCommand::ReleaseVacuum => u32::MAX,
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
}

struct GrabberHalEv3 {
    pressure_sensor: MSPressureSensor,
    arm_motor: MediumMotor,
    pump_motor: LargeMotor,
}

impl GrabberHalEv3 {
    const ARM_CALIBRATE_DUTY_CYCLE: i32 = 70;
    const ARM_RAISED_POS: i32 = -10;
    const ARM_RAISE_SPEED: i32 = 600;
    const ARM_LOWER_TO_DROP_SPEED: i32 = -600;
    const ARM_RELEASE_POS: i32 = -100;
    const ARM_LOWER_DUTY_CYCLE: i32 = -50;
    const PUMP_CREATE_VACUUM_CYCLE: i32 = -100;
    const PUMP_RELEASE_VACUUM_CYCLE: i32 = 100;
}

impl GrabberHal for GrabberHalEv3 {
    fn calibrate(&mut self) -> anyhow::Result<()> {
        self.arm_motor.set_duty_cycle_sp(GrabberHalEv3::ARM_CALIBRATE_DUTY_CYCLE)?;
        self.arm_motor.run_direct()?;
        if !self.arm_motor.wait_until(MediumMotor::STATE_STALLED, Some(Duration::from_secs(4))) {
            return Err(anyhow!("Arm calibration failed!"));
        }
        self.arm_motor.reset()?;
        self.pump_motor.reset()?;
        Ok(())
    }

    fn current_pressure_pa(&self) -> anyhow::Result<u32> {
        let reading = self.pressure_sensor.current_pressure_pa()?;
        println!("current_pressure_pa: {reading}");
        Ok(u32::try_from(reading)?)
    }

    fn send_arm_command(&mut self, command: ArmCommand) -> anyhow::Result<()> {
        println!("send_arm_command: {command:?}");
        match command {
            ArmCommand::LowerToGrab => {
                self.arm_motor.set_duty_cycle_sp(GrabberHalEv3::ARM_LOWER_DUTY_CYCLE)?;
                self.arm_motor.run_direct()?;
            }
            ArmCommand::LowerToDrop => {
                self.arm_motor.set_stop_action("coast")?;
                self.arm_motor.set_speed_sp(GrabberHalEv3::ARM_LOWER_TO_DROP_SPEED)?;
                self.arm_motor.run_to_abs_pos(Some(GrabberHalEv3::ARM_RELEASE_POS))?;
            }
            ArmCommand::Raise => {
                self.arm_motor.set_stop_action("brake")?;
                self.arm_motor.set_speed_sp(GrabberHalEv3::ARM_RAISE_SPEED)?;
                self.arm_motor.run_to_abs_pos(Some(GrabberHalEv3::ARM_RAISED_POS))?;
            }
            ArmCommand::Hold => {
                self.arm_motor.set_stop_action("brake")?;
                self.arm_motor.stop()?;
            }
        }
        Ok(())
    }

    fn is_arm_idle(&self) -> anyhow::Result<bool> {
        Ok(self.arm_motor.get_state()?.is_empty())
    }

    fn send_pump_command(&mut self, command: PumpCommand) -> anyhow::Result<()> {
        println!("send_pump_command: {command:?}");
        match command {
            PumpCommand::CreateVacuum => {
                self.pump_motor.set_duty_cycle_sp(GrabberHalEv3::PUMP_CREATE_VACUUM_CYCLE)?;
                self.pump_motor.run_direct()?;
            }
            PumpCommand::ReleaseVacuum => {
                self.pump_motor.set_duty_cycle_sp(GrabberHalEv3::PUMP_RELEASE_VACUUM_CYCLE)?;
                self.pump_motor.run_direct()?;
            }
            PumpCommand::Stop => self.pump_motor.stop()?,
        }
        Ok(())
    }
}

impl BehaviourTreeFactory {
    pub fn create_bt(&self) -> Behavior<GrabberAction> {
        let make_contact = Action(DynamicAction::new(|s: &mut GrabberState| {
            if s.set_pump_command(PumpCommand::CreateVacuum).is_err() {
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
            if s.set_pump_command(PumpCommand::CreateVacuum).is_err() {
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
            if s.set_pump_command(PumpCommand::ReleaseVacuum).is_err() {
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

struct GrabberState {
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
            reading if reading <= MIN_PRESSURE_GRAB => Ok(SuctionState::Grab),
            reading if reading <= MIN_PRESSURE_CONTACT => Ok(SuctionState::Contact),
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

type GrabberAction = DynamicAction<GrabberState>;

struct DynamicAction<S> {
    action: Rc<dyn ActionTrait<S>>,
}

impl<S> Clone for DynamicAction<S> {
    fn clone(&self) -> Self {
        Self {
            action: self.action.clone(),
        }
    }
}

trait ActionTrait<S> {
    fn handle(&self, state: &mut S) -> Status;
}

impl<F, S> ActionTrait<S> for F
where F: Fn(&mut S) -> Status {
    fn handle(&self, state: &mut S) -> Status {
        (self)(state)
    }
}

impl<S> DynamicAction<S> {
    pub fn new(action: impl ActionTrait<S> + 'static) -> Self {
        Self {
            action: Rc::new(action),
        }
    }
}
