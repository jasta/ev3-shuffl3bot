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
use std::sync::{Mutex, MutexGuard};
use std::{io, thread};
use std::io::Write;
use std::time::{Duration, Instant};

use ai_behavior::{Action, Behavior, Fail, Failure, Running, Select, Sequence, State, Status, Success, Wait, WhenAll, WhenAny, While};
use anyhow::anyhow;
use ev3dev_lang_rust::Ev3Result;
use ev3dev_lang_rust::motors::{LargeMotor, MediumMotor, MotorPort};
use ev3dev_lang_rust::sensors::SensorPort;
use input::{Event, UpdateArgs};
use serde::{Serialize, Serializer};

use ev3::ms_pressure_sensor::MSPressureSensor;
use ev3_shuffl3bot::ev3;

const TICK_INTERVAL: Duration = Duration::from_millis(20);

fn main() -> anyhow::Result<()> {
    let mut hal = create_hal()?;
    hal.calibrate()?;
    let bt = BehaviourTreeFactory { hal: Rc::from(hal) }.create();
    run_machine(State::new(bt))
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

fn run_machine(mut state: State<GrabberAction, ()>) -> anyhow::Result<()> {
    let mut my_state = GrabberState::default();

    let mut dt = 0.0;
    let mut ticks = 0;
    let result = loop {
        let start = Instant::now();
        let e: Event = UpdateArgs { dt }.into();
        let (status, _) = state.event(&e, &mut |args| {
            (args.action.action.handle(&mut my_state), args.dt)
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
    result
}

struct BehaviourTreeFactory {
    hal: Rc<dyn GrabberHal>,
}

trait GrabberHal {
    fn calibrate(&mut self) -> anyhow::Result<()>;
    fn current_pressure_pa(&self) -> anyhow::Result<u32>;
    fn send_arm_command(&self, command: ArmCommand) -> anyhow::Result<()>;
    fn is_arm_idle(&self) -> anyhow::Result<bool>;
    fn send_pump_command(&self, command: PumpCommand) -> anyhow::Result<()>;
}

#[derive(Debug, Clone)]
enum ArmCommand {
    Lower,
    Raise,
    Hold,
}

#[derive(Debug, Clone)]
enum PumpCommand {
    CreateVacuum,
    ReleaseVacuum,
    Stop,
}

#[derive(Default)]
struct GrabberHalMock {
    state: Mutex<MockState>,
}

#[derive(Default)]
struct MockState {
    last_arm_command: Option<ArmCommand>,
    last_pump_command: Option<PumpCommand>,
}

const MIN_PRESSURE_CONTACT: u32 = 100000;
const MIN_PRESSURE_GRAB: u32 = 80000;

impl GrabberHal for GrabberHalMock {
    fn calibrate(&mut self) -> anyhow::Result<()> {
        println!("Beep.");
        Ok(())
    }

    fn current_pressure_pa(&self) -> anyhow::Result<u32> {
        let state = self.state.lock().unwrap();
        let answer = match state.last_pump_command.as_ref().unwrap_or(&PumpCommand::Stop) {
            PumpCommand::CreateVacuum => {
                match state.last_arm_command.as_ref().unwrap_or(&ArmCommand::Hold) {
                    ArmCommand::Lower => MIN_PRESSURE_CONTACT,
                    ArmCommand::Raise => u32::MAX,
                    ArmCommand::Hold => MIN_PRESSURE_GRAB,
                }
            }
            PumpCommand::ReleaseVacuum => u32::MAX,
            PumpCommand::Stop => u32::MAX,
        };
        println!("current_pressure_pa: {answer}");
        Ok(answer)
    }

    fn send_arm_command(&self, command: ArmCommand) -> anyhow::Result<()> {
        println!("send_arm_command: {command:?}");
        self.state.lock().unwrap().last_arm_command = Some(command);
        Ok(())
    }

    fn is_arm_idle(&self) -> anyhow::Result<bool> {
        let answer = match self.state.lock().unwrap().last_arm_command.as_ref().unwrap_or(&ArmCommand::Hold) {
            ArmCommand::Lower => false,
            ArmCommand::Raise => true,
            ArmCommand::Hold => false,
        };
        println!("is_arm_idle: {answer}");
        Ok(answer)
    }

    fn send_pump_command(&self, command: PumpCommand) -> anyhow::Result<()> {
        println!("send_pump_command: {command:?}");
        self.state.lock().unwrap().last_pump_command = Some(command);
        Ok(())
    }
}

struct GrabberHalEv3 {
    pressure_sensor: MSPressureSensor,
    arm_motor: MediumMotor,
    pump_motor: LargeMotor,
}

impl GrabberHalEv3 {
    const ARM_RAISED_POS: i32 = 10;
    const ARM_LOWER_DUTY_CYCLE: i32 = -30;
    const PUMP_CREATE_VACUUM_CYCLE: i32 = -100;
    const PUMP_RELEASE_VACUUM_CYCLE: i32 = 100;
}

impl GrabberHal for GrabberHalEv3 {
    fn calibrate(&mut self) -> anyhow::Result<()> {
        self.arm_motor.reset()?;
        self.pump_motor.reset()?;
        Ok(())
    }

    fn current_pressure_pa(&self) -> anyhow::Result<u32> {
        let reading = self.pressure_sensor.current_pressure_pa()?;
        Ok(u32::try_from(reading)?)
    }

    fn send_arm_command(&self, command: ArmCommand) -> anyhow::Result<()> {
        match command {
            ArmCommand::Lower => {
                self.arm_motor.set_duty_cycle_sp(GrabberHalEv3::ARM_LOWER_DUTY_CYCLE)?;
                self.arm_motor.run_direct()?;
            }
            ArmCommand::Raise => {
                self.arm_motor.set_stop_action("brake")?;
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

    fn send_pump_command(&self, command: PumpCommand) -> anyhow::Result<()> {
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
    pub fn create(&self) -> Behavior<GrabberAction> {
        Sequence(vec![self.get_ready_to_move(), self.bt_release()])
    }

    fn get_ready_to_move(&self) -> Behavior<GrabberAction> {
        let is_ready = Action(DynamicAction::new(|s: &mut GrabberState| {
            println!("ready_to_move: {}", s.ready_to_move);
            if s.ready_to_move { Success } else { Failure }
        }));

        WhenAny(
            vec![
                is_ready,
                Sequence(vec![self.make_contact(), self.bt_secure_grab(), self.bt_lift()])])
    }

    fn make_contact(&self) -> Behavior<GrabberAction> {
        let contact_hal = self.hal.clone();
        let down_and_pump_hal = self.hal.clone();
        let hold_down_hal = self.hal.clone();
        let has_contact = Action(DynamicAction::new(move |_s: &mut GrabberState| {
            match contact_hal.current_pressure_pa() {
                Ok(reading) if reading <= MIN_PRESSURE_CONTACT => Success,
                _ => Failure,
            }
        }));
        let down_and_pump = Action(DynamicAction::new(move |_s: &mut GrabberState| {
            if down_and_pump_hal.send_pump_command(PumpCommand::CreateVacuum).is_err() {
                return Failure;
            }
            if down_and_pump_hal.send_arm_command(ArmCommand::Lower).is_err() {
                return Failure;
            }
            Running
        }));
        let send_hold_down = Action(DynamicAction::new(move |_s: &mut GrabberState| {
            if hold_down_hal.send_arm_command(ArmCommand::Hold).is_err() {
                return Failure;
            }
            Success
        }));
        Sequence(
            vec![
                WhenAny(vec![
                    has_contact,
                    While(Box::new(Fail(Box::new(Wait(5.0)))), vec![down_and_pump])]),
                send_hold_down])
    }

    fn bt_secure_grab(&self) -> Behavior<GrabberAction> {
        let grab_hal = self.hal.clone();
        let has_grab = Action(DynamicAction::new(move |_s: &mut GrabberState| {
            match grab_hal.current_pressure_pa() {
                Ok(reading) if reading <= MIN_PRESSURE_GRAB => Success,
                _ => Failure,
            }
        }));

        // TODO: I think we need to change how fast the pump operates here or adjust how hard the
        // arm grabs in order to overcome the suction problem from the cards underneath.
        has_grab
    }

    fn bt_lift(&self) -> Behavior<GrabberAction> {
        let is_lifted_hal = self.hal.clone();
        let lift_hal = self.hal.clone();
        let is_idle = Action(DynamicAction::new(move |s: &mut GrabberState| {
            match is_lifted_hal.is_arm_idle() {
                Ok(is_idle) if is_idle => {
                    s.ready_to_move = true;
                    Success
                },
                _ => Failure,
            }
        }));
        let send_lift = Action(DynamicAction::new(move |_s: &mut GrabberState| {
            if lift_hal.send_arm_command(ArmCommand::Raise).is_err() {
                return Failure
            }
            Success
        }));

        Sequence(
            vec![
                send_lift,
                While(Box::new(Fail(Box::new(Wait(5.0)))), vec![is_idle])])
    }

    fn bt_release(&self) -> Behavior<GrabberAction> {
        let contact_hal = self.hal.clone();
        let release_hal = self.hal.clone();
        let lost_contact = Action(DynamicAction::new(move |_s: &mut GrabberState| {
             match contact_hal.current_pressure_pa() {
                 Ok(reading) if reading > MIN_PRESSURE_CONTACT => Success,
                 _ => Failure,
             }
        }));
        let release_pressure = Action(DynamicAction::new(move |_s: &mut GrabberState| {
            if release_hal.send_pump_command(PumpCommand::ReleaseVacuum).is_err() {
                return Failure;
            }
            Running
        }));

        WhenAny(vec![lost_contact, release_pressure])
    }
}

#[derive(Default, Serialize)]
struct GrabberState {
    ready_to_move: bool,
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

impl<State> Serialize for DynamicAction<State> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
        serializer.serialize_str("{action}")
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
