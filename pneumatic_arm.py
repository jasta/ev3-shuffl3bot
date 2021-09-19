#!/usr/bin/env python3

from time import sleep
import ev3dev2
from ev3dev2.motor import OUTPUT_A, OUTPUT_D, SpeedDPS, SpeedValue
from ev3dev2.motor import LargeMotor, MediumMotor, Motor, SpeedPercent
from ev3dev2.sensor import Sensor, INPUT_1, INPUT_4
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sound import Sound
from threading import Lock, Thread, Event
import math
import sys
import os

def debug_print(*args, **kwargs):
    print(*args, **kwargs, file=sys.stderr)

class MSPressureSensor(Sensor):
  SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
  SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

  MODE_RAW = 'RAW'
  MODE_ABS_PSI = 'ABS-PSI'
  MODE_ABS_MBAR = 'ABS-MBAR'
  MODE_ABS_KPA = 'ABS-KPA'
  MODE_REL_PSI = 'REL-PSI'
  MODE_REL_MBAR = 'REL-MBAR'
  MODE_REL_KPA = 'REL-KPA'
  MODES = (MODE_RAW, MODE_ABS_PSI, MODE_ABS_MBAR, MODE_ABS_KPA, MODE_REL_PSI, MODE_REL_MBAR, MODE_REL_KPA)

  def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
    super(MSPressureSensor, self).__init__(
        address,
        name_pattern,
        name_exact,
        driver_name=['ms-pps58-nx'],
        **kwargs)

  @property
  def absolute_pressure_pa(self):
    self._ensure_mode(self.MODE_RAW)
    return self.value(0)

class VacuumPumpMachine:
  EXPECTED_ROOM_PRESSURE_PA = range(104000, 110000)

  MODE_VACUUM = -1
  MODE_FREEFLOW = 1

  CALIBRATE_SWITCH_SPEED = SpeedDPS(200) * MODE_VACUUM

  CYLINDER_MOTOR_DUTY_CYCLE = 100
  SWITCH_MOTOR_SPEED = SpeedPercent(50)

  def __init__(self, cylinder_motor_port: str, switch_motor_port: str, pressure_sensor_port: str):
    self.cylinder_motor = LargeMotor(cylinder_motor_port)
    self.switch_motor = MediumMotor(switch_motor_port)
    self.pressure_sensor = MSPressureSensor(pressure_sensor_port)
    self.is_calibrated = False
    self.desired_pressure_range_pa = None
    self.intended_switch_pos = None
    self.hold_pressure_thread = None

  def calibrate_automatically(self):
    self.cylinder_motor.reset()

    self.switch_motor.on(speed = VacuumPumpMachine.CALIBRATE_SWITCH_SPEED, block = True)
    self.switch_motor.off()
    self.switch_motor.reset()

    current_pressure = self.get_current_pressure_pa()
    if current_pressure not in VacuumPumpMachine.EXPECTED_ROOM_PRESSURE_PA:
      raise Exception('Release line pressure first: {}'.format(current_pressure))

    self.is_calibrated = True

  def _ensure_calibrated(self):
    if not self.is_calibrated:
      raise Exception('Must calibrate machine first!')

  def on_and_hold_at(self, desired_pressure_range_pa: range):
    self._ensure_calibrated()

    self.switch_motor.on(speed = VacuumPumpMachine.SWITCH_MOTOR_SPEED * VacuumPumpMachine.MODE_VACUUM, block = True)
    self.switch_motor.off()
    self.desired_pressure_range_pa = desired_pressure_range_pa

    debug_print('Starting hold pressure thread')
    if self.hold_pressure_thread is None:
      self.hold_pressure_thread = HoldPressureThread(self)
      self.hold_pressure_thread.start()

  def cancel_hold(self):
    if self.hold_pressure_thread is not None:
      debug_print('Stopping hold pressure thread...')
      self.hold_pressure_thread.shutdown_event.set()
      self.hold_pressure_thread.join()
      self.hold_pressure_thread = None

  def release_pressure(self):
    self._ensure_calibrated()

    self.cancel_hold()

    self.cylinder_motor.off()
    self.switch_motor.on(speed = VacuumPumpMachine.SWITCH_MOTOR_SPEED * VacuumPumpMachine.MODE_FREEFLOW, block = True)
    self.switch_motor.off()

  def get_current_pressure_pa(self):
    return self.pressure_sensor.absolute_pressure_pa

class HoldPressureThread(Thread):
  def __init__(self, pump: VacuumPumpMachine):
    Thread.__init__(self, daemon = True)
    self.shutdown_event = Event()
    self.pump = pump

  def __str__(self):
    return "HoldPressureThread"

  def run(self):
    motor_is_on = False
    while not self.shutdown_event.is_set():
      current_pressure = self.pump.get_current_pressure_pa()
      if current_pressure in self.pump.desired_pressure_range_pa:
        motor_should_be_on = False
      else:
        motor_should_be_on = True
      debug_print('Pressure is {}, target: {}, motor_should_be_on: {}'.format(current_pressure, self.pump.desired_pressure_range_pa, motor_should_be_on))

      if motor_is_on != motor_should_be_on:
        debug_print('Setting motor to {}'.format(motor_should_be_on))
        motor_is_on = motor_should_be_on
        if motor_should_be_on:
          self.pump.cylinder_motor.duty_cycle_sp = VacuumPumpMachine.CYLINDER_MOTOR_DUTY_CYCLE
          self.pump.cylinder_motor.run_direct()
        else:
          self.pump.cylinder_motor.off(brake = False)

      sleep(0.1)

class ArmStateDesc:
  def __init__(self, name: str, pressure_range_pa: range = None):
    self.name = name
    self.pressure_range_pa = pressure_range_pa

class PneumaticArmMachine:
  STATE_GRABBED = ArmStateDesc('STATE_GRABBED', pressure_range_pa = range(60000, 20000, -1))
  STATE_RELEASED = ArmStateDesc('STATE_RELEASED')

  _STATES = (STATE_GRABBED, STATE_RELEASED)

  def __init__(self, pump: VacuumPumpMachine, on_off_sensor: TouchSensor):
    self.pump = pump
    self.on_off_sensor = on_off_sensor
    self.ready_to_actuate_event = Event()
    self.ready_to_actuate_event.set()
    self.automatic_thread = self.AutomaticGrabAndReleaseThread(self)
    self.arm_state = None

  def calibrate_manually_at_pressed_down(self):
    self.pump.calibrate_automatically()

    if self.on_off_sensor.is_pressed:
      raise Exception('Release contact touch sensor first')

    self.arm_state = PneumaticArmMachine.STATE_RELEASED

  def begin_automatic_grab_and_release(self):
    self.automatic_thread.start()
    pass

  def end_automatic_grab_and_release(self):
    self.automatic_thread.shutdown_event.set()
    self.automatic_thread.join()

  def is_successfully_grabbing(self):
    current_pressure = self.pump.get_current_pressure_pa()
    return current_pressure in PneumaticArmMachine.STATE_GRABBED.pressure_range_pa

  def set_arm_state(self, arm_state: ArmStateDesc):
    if arm_state not in PneumaticArmMachine._STATES:
      raise Exception('Unsupported arm_state={}'.format(arm_state))

    debug_print('Setting arm state to {}'.format(arm_state.name))
    if arm_state.pressure_range_pa is not None:
      self.pump.on_and_hold_at(arm_state.pressure_range_pa)
    else:
      self.pump.release_pressure()

  def shutdown(self):
    self.pump.release_pressure()

  class AutomaticGrabAndReleaseThread(Thread):
    def __init__(self, arm_machine):
      Thread.__init__(self, daemon = True)
      self.shutdown_event = Event()
      self.arm_machine = arm_machine

    def __str__(self):
      return "GrabAndRelease"

    def run(self):
      while not self.shutdown_event.is_set():
        if self.arm_machine.on_off_sensor.wait_for_pressed(timeout_ms = 1000):
          self.arm_machine._toggle_grab_or_release()

def main():
  sound = Sound()
  touch_sensor = TouchSensor(INPUT_1)
  pump = VacuumPumpMachine(cylinder_motor_port = OUTPUT_A, switch_motor_port = OUTPUT_D, pressure_sensor_port = INPUT_4)
  arm = PneumaticArmMachine(pump, on_off_sensor = touch_sensor)
  arm.calibrate_manually_at_pressed_down()

  while True:
    debug_print('Grabbing...')
    arm.set_arm_state(PneumaticArmMachine.STATE_GRABBED)
    sleep(25)
    debug_print('Releasing...')
    arm.set_arm_state(PneumaticArmMachine.STATE_RELEASED)
    sleep(15)
    
if __name__ == "__main__":
  if ev3dev2.get_current_platform() == "fake":
    fake_sys = os.path.join(os.path.dirname(__file__), 'fake-sys')
    ev3dev2.Device.DEVICE_ROOT_PATH = os.path.join(fake_sys, 'arena')

  main()
