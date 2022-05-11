use ev3dev_lang_rust::Ev3Result;
use pid::Pid;
use ev3dev_lang_rust::motors::{LargeMotor, MediumMotor, MotorPort};
use std::time::Duration;
use anyhow::anyhow;
use conv::{ConvUtil, RoundToNearest};
use ev3dev_lang_rust::sensors::SensorPort;
use crate::ev3::ms_pressure_sensor::MSPressureSensor;
use crate::grabber_hal;
use crate::grabber_hal::{ArmCommand, GrabberHal, PumpCommand};

pub struct GrabberHalEv3 {
    pressure_sensor: MSPressureSensor,
    arm_motor: MediumMotor,
    pump_motor: LargeMotor,
    pump_pid: Pid<f64>,
}

impl GrabberHalEv3 {
    const ARM_CALIBRATE_DUTY_CYCLE: i32 = 50;
    const ARM_RAISED_POS: i32 = -10;
    const ARM_RAISE_SPEED: i32 = 600;
    const ARM_LOWER_TO_DROP_SPEED: i32 = -600;
    const ARM_RELEASE_POS: i32 = -100;
    const ARM_LOWER_DUTY_CYCLE: i32 = -40;
    const PUMP_CREATE_VACUUM_CYCLE: i32 = -100;
    const PUMP_RELEASE_VACUUM_CYCLE: i32 = 100;

    pub fn new(port_spec: Ev3PortSpec) -> Ev3Result<Self> {
        let target = f64::from(grabber_hal::TARGET_PRESSURE_GRAB);
        let limit = 100.0;
        let pump_pid = Pid::new(2.0, 0.1, 1.0, limit, limit, limit, limit, target);

        Ok(Self {
            pressure_sensor: MSPressureSensor::get(port_spec.pressure_sensor)?,
            pump_motor: LargeMotor::get(port_spec.pump_motor)?,
            arm_motor: MediumMotor::get(port_spec.arm_motor)?,
            pump_pid,
        })
    }
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
            PumpCommand::StartVacuum => {
                self.pump_motor.set_duty_cycle_sp(GrabberHalEv3::PUMP_CREATE_VACUUM_CYCLE)?;
                self.pump_motor.run_direct()?;
            }
            PumpCommand::CreateAndHoldVacuum => {
                self.on_tick_while_holding()?;
                self.pump_motor.run_direct()?;
            }
            PumpCommand::ReverseVacuum => {
                self.pump_motor.set_duty_cycle_sp(GrabberHalEv3::PUMP_RELEASE_VACUUM_CYCLE)?;
                self.pump_motor.run_direct()?;
            }
            PumpCommand::Stop => self.pump_motor.stop()?,
        }
        Ok(())
    }

    fn on_tick_while_holding(&mut self) -> anyhow::Result<()> {
        let current = f64::from(self.current_pressure_pa()?);
        let full_output = self.pump_pid.next_control_output(current);
        let pid_output = full_output.output.approx_as_by::<i32, RoundToNearest>().unwrap();
        // Refuse to go pump in reverse...
        let duty_cycle = pid_output.min(0);
        println!("on_tick_while_grabbed: current={current}, pid gave us {pid_output}, using {duty_cycle}");
        self.pump_motor.set_duty_cycle_sp(duty_cycle)?;
        Ok(())
    }
}

pub struct Ev3PortSpec {
    pub pressure_sensor: SensorPort,
    pub pump_motor: MotorPort,
    pub arm_motor: MotorPort,
}
