use crate::ev3::ms_pressure_sensor::MSPressureSensor;
use crate::shuffler_hal;
use crate::shuffler_hal::{ArmCommand, ShufflerHal, PumpCommand};
use anyhow::{anyhow, Context};
use conv::{ConvUtil, RoundToNearest};
use ev3dev_lang_rust::motors::{LargeMotor, MediumMotor, MotorPort};
use ev3dev_lang_rust::sensors::SensorPort;
use ev3dev_lang_rust::Ev3Result;
use pid::Pid;
use std::time::Duration;

pub struct ShufflerHalEv3 {
    pressure_sensor: MSPressureSensor,
    x_motor: MediumMotor,
    y_motor: MediumMotor,
    arm_motor: MediumMotor,
    pump_motor: LargeMotor,
    pump_pid: Pid<f64>,
}

impl ShufflerHalEv3 {
    const X_MOTOR_CALIBRATE_DUTY_CYCLE: i32 = -60;
    const X_MOTOR_TRANSLATE_SPEED: i32 = 900;
    const Y_MOTOR_CALIBRATE_DUTY_CYCLE: i32 = -80;
    const Y_MOTOR_TRANSLATE_SPEED: i32 = 900;

    /// Very bizarre quirk of the Robot right now where the X-axis cannot translate smoothly
    /// if the arm is raised too high.  Make sure it is at least this low before attempting to.
    const X_TRANSLATION_MAX_SAFE_ARM_HEIGHT: i32 = -20;

    const ARM_CALIBRATE_DUTY_CYCLE: i32 = 50;
    const ARM_CONFIRM_GRAB_REL_POS: i32 = 100;
    const ARM_CONFIRM_GRAB_MOVE_SPEED: i32 = 300;
    const ARM_RAISE_TO_MOVE_POS: i32 = ShufflerHalEv3::X_TRANSLATION_MAX_SAFE_ARM_HEIGHT;
    const ARM_RAISE_TO_MOVE_SPEED: i32 = 600;
    const ARM_LOWER_TO_DROP_SPEED: i32 = -600;
    const ARM_RELEASE_POS: i32 = -100;
    const ARM_LOWER_DUTY_CYCLE: i32 = -60;
    const PUMP_CREATE_VACUUM_CYCLE: i32 = -100;
    const PUMP_RELEASE_VACUUM_CYCLE: i32 = 100;

    const COLUMN_X_POSITIONS: [i32; 3] = [
        82,
        282,
        528,
    ];

    const ROW_Y_POSITIONS: [i32; 3] = [
        175,
        592,
        1048,
    ];

    pub fn new(port_spec: Ev3PortSpec) -> Ev3Result<Self> {
        let target = f64::from(shuffler_hal::TARGET_PRESSURE_GRAB);
        let limit = 100.0;
        let pump_pid = Pid::new(2.0, 0.1, 1.0, limit, limit, limit, limit, target);

        Ok(Self {
            pressure_sensor: MSPressureSensor::get(port_spec.pressure_sensor)?,
            x_motor: MediumMotor::get(port_spec.x_motor)?,
            y_motor: MediumMotor::get(port_spec.y_motor)?,
            pump_motor: LargeMotor::get(port_spec.pump_motor)?,
            arm_motor: MediumMotor::get(port_spec.arm_motor)?,
            pump_pid,
        })
    }
}

impl ShufflerHal for ShufflerHalEv3 {
    fn calibrate_gantry(&mut self) -> anyhow::Result<()> {
        println!("Calibrating gantry...");
        if self.arm_motor.get_position()? > ShufflerHalEv3::X_TRANSLATION_MAX_SAFE_ARM_HEIGHT {
            return Err(anyhow!("Arm is too far raised, cannot translate on X-axis!"));
        }

        calibrate_motor("y", &self.y_motor, ShufflerHalEv3::Y_MOTOR_CALIBRATE_DUTY_CYCLE)?;
        calibrate_motor("x", &self.x_motor, ShufflerHalEv3::X_MOTOR_CALIBRATE_DUTY_CYCLE)?;
        Ok(())
    }

    fn calibrate_grabber(&mut self) -> anyhow::Result<()> {
        println!("Calibrating grabber...");
        calibrate_motor("arm", &self.arm_motor, ShufflerHalEv3::ARM_CALIBRATE_DUTY_CYCLE)?;
        self.pump_motor.reset()?;

        // Move to the safe raised position so calibrate_gantry() doesn't freak out...
        self.arm_motor.set_speed_sp(ShufflerHalEv3::ARM_RAISE_TO_MOVE_SPEED)?;
        self.arm_motor.run_to_abs_pos(Some(ShufflerHalEv3::X_TRANSLATION_MAX_SAFE_ARM_HEIGHT))?;
        if !self.arm_motor.wait_until_not_moving(Some(Duration::from_secs(2))) {
            return Err(anyhow!("Arm failed to stop moving, what?"));
        }

        Ok(())
    }

    fn current_pressure_pa(&self) -> anyhow::Result<u32> {
        let reading = self.pressure_sensor.current_pressure_pa()?;
        Ok(u32::try_from(reading)?)
    }

    fn send_move_to_row_command(&mut self, row: usize) -> anyhow::Result<()> {
        let pos = *ShufflerHalEv3::COLUMN_X_POSITIONS.get(row).unwrap();
        println!("send_move_to_row_command: {row}, pos={pos}");
        self.x_motor.set_stop_action(MediumMotor::STOP_ACTION_HOLD)?;
        self.x_motor.set_speed_sp(ShufflerHalEv3::X_MOTOR_TRANSLATE_SPEED)?;
        self.x_motor.run_to_abs_pos(Some(pos))?;
        Ok(())
    }

    fn send_move_to_col_command(&mut self, col: usize) -> anyhow::Result<()> {
        let pos = *ShufflerHalEv3::ROW_Y_POSITIONS.get(col).unwrap();
        println!("send_move_to_col_command: {col}, pos={pos}");
        self.y_motor.set_stop_action(MediumMotor::STOP_ACTION_HOLD)?;
        self.y_motor.set_speed_sp(ShufflerHalEv3::Y_MOTOR_TRANSLATE_SPEED)?;
        self.y_motor.run_to_abs_pos(Some(pos))?;
        Ok(())
    }

    fn did_move_to_rowcol(&self) -> anyhow::Result<bool> {
        for motor in [&self.x_motor, &self.y_motor] {
            if motor.is_running()? {
                return Ok(false)
            }
        }
        for motor in [&self.x_motor, &self.y_motor] {
            let state = motor.get_state()?;
            let pos = motor.get_position()?;
            println!("...motor: state={state:?}, pos={pos}");
        }
        Ok(true)
    }

    fn send_arm_command(&mut self, command: ArmCommand) -> anyhow::Result<()> {
        println!("send_arm_command: {command:?}");
        match command {
            ArmCommand::LowerToGrab => {
                self.arm_motor
                    .set_duty_cycle_sp(ShufflerHalEv3::ARM_LOWER_DUTY_CYCLE)?;
                self.arm_motor.run_direct()?;
            }
            ArmCommand::LowerToDrop => {
                self.arm_motor.set_stop_action("coast")?;
                self.arm_motor
                    .set_speed_sp(ShufflerHalEv3::ARM_LOWER_TO_DROP_SPEED)?;
                self.arm_motor
                    .run_to_abs_pos(Some(ShufflerHalEv3::ARM_RELEASE_POS))?;
            }
            ArmCommand::RaiseToMove => {
                self.arm_motor.set_stop_action("brake")?;
                self.arm_motor
                    .set_speed_sp(ShufflerHalEv3::ARM_RAISE_TO_MOVE_SPEED)?;
                self.arm_motor
                    .run_to_abs_pos(Some(ShufflerHalEv3::ARM_RAISE_TO_MOVE_POS))?;
            }
            ArmCommand::RaiseToConfirm => {
                self.arm_motor.set_stop_action("hold")?;
                self.arm_motor.set_speed_sp(ShufflerHalEv3::ARM_CONFIRM_GRAB_MOVE_SPEED)?;
                let current_pos = self.arm_motor.get_position()?;
                let default_rel_pos = ShufflerHalEv3::ARM_CONFIRM_GRAB_REL_POS;
                let max_rel_pos = (current_pos - ShufflerHalEv3::X_TRANSLATION_MAX_SAFE_ARM_HEIGHT).abs();

                let actual_rel_pos = default_rel_pos.min(max_rel_pos);
                println!("pos={current_pos}, default={default_rel_pos}, max={max_rel_pos}, actual={actual_rel_pos}");
                self.arm_motor.run_to_rel_pos(Some(actual_rel_pos))?;
            }
            ArmCommand::Hold => {
                self.arm_motor.set_stop_action("hold")?;
                self.arm_motor.stop()?;
            }
        }
        Ok(())
    }

    fn did_move_arm(&self) -> anyhow::Result<bool> {
        Ok(!self.arm_motor.is_running()?)
    }

    fn send_pump_command(&mut self, command: PumpCommand) -> anyhow::Result<()> {
        println!("send_pump_command: {command:?}");
        match command {
            PumpCommand::StartVacuum => {
                self.pump_motor
                    .set_duty_cycle_sp(ShufflerHalEv3::PUMP_CREATE_VACUUM_CYCLE)?;
                self.pump_motor.run_direct()?;
            }
            PumpCommand::CreateAndHoldVacuum => {
                self.on_tick_while_holding()?;
                self.pump_motor.run_direct()?;
            }
            PumpCommand::ReverseVacuum => {
                self.pump_motor
                    .set_duty_cycle_sp(ShufflerHalEv3::PUMP_RELEASE_VACUUM_CYCLE)?;
                self.pump_motor.run_direct()?;
            }
            PumpCommand::Stop => self.pump_motor.stop()?,
        }
        Ok(())
    }

    fn on_tick_while_holding(&mut self) -> anyhow::Result<()> {
        let current = f64::from(self.current_pressure_pa()?);
        let full_output = self.pump_pid.next_control_output(current);
        let pid_output = full_output
            .output
            .approx_as_by::<i32, RoundToNearest>()
            .unwrap();
        // Refuse to go pump in reverse...
        let duty_cycle = pid_output.min(0);
        println!("on_tick_while_grabbed: current={current}, pid gave us {pid_output}, using {duty_cycle}");
        self.pump_motor.set_duty_cycle_sp(duty_cycle)?;
        Ok(())
    }
}

fn calibrate_motor(name: &str, motor: &MediumMotor, duty_cycle: i32) -> anyhow::Result<()> {
    motor.set_stop_action(MediumMotor::STOP_ACTION_BRAKE)?;
    motor.set_duty_cycle_sp(duty_cycle)?;
    motor.run_direct()?;
    if !motor.wait_until(MediumMotor::STATE_STALLED, Some(Duration::from_secs(4))) {
        return Err(anyhow!("'{name}' failed to stall out motor!"));
    }
    motor.stop()?;
    if !motor.wait_until_not_moving(Some(Duration::from_secs(2))) {
        return Err(anyhow!("'{name}' failed trying to stop motor!"));
    }
    motor.reset()?;
    Ok(())
}

pub struct Ev3PortSpec {
    pub pressure_sensor: SensorPort,
    pub x_motor: MotorPort,
    pub y_motor: MotorPort,
    pub pump_motor: MotorPort,
    pub arm_motor: MotorPort,
}
