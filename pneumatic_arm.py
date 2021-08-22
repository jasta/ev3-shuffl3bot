#!/usr/bin/env python3

from time import sleep
import ev3dev2
from ev3dev2.motor import OUTPUT_A, OUTPUT_D, SpeedValue
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

class LockableMotorDesc:
  def __init__(self, motor: Motor, name: str, movement_desc_lambda):
    self.motor = motor
    self.name = name
    self.movement_desc_lambda = movement_desc_lambda

class MovementDesc:
  def __init__(self, speed: SpeedValue, absolute_position: int = None, run_forever: bool = None):
    self.speed = speed
    self.absolute_position = absolute_position
    self.run_forever = run_forever

class SoftwareMechanicalLocking:
  """ Implement a crude mechanical 'lock' in software as if two components in the machine
  were mechanically locked to each other in some way.
  
  For example, if a single drive shaft had
  a linked axle with another shaft, they would be required to stay in sync always.  We can do something
  similar in software but with custom and much more sophisticated logic.  This saves significantly on space
  in the robot machinery and makes it much simpler to work with components like the new pneumatic switch albeit
  at the cost of an extra driving motor.
  """

  def __init__(self, user_data, motors: list):
    self.user_data = user_data
    self.motors = motors
    self.threads = []
    self.lock = Lock()

  def start_locking(self):
    if self.is_active():
      raise Exception('Already locking!')

    for motor in self.motors:
      other_motors = [ other_motor for other_motor in self.motors if other_motor != motor ]
      thread = self.MotorLockingThread(self.lock, self.user_data, motor, other_motors)
      thread.start()
      self.threads.append(thread)

  def stop_locking(self):
    for thread in self.threads:
      thread.stop_event.set()

    for thread in self.threads:
      thread.join()

    self.threads = []

  def is_active(self):
    return len(self.threads) > 0

  class MotorLockingThread(Thread):
    def __init__(self, state_lock: Lock, user_data, my_motor: LockableMotorDesc, other_motors: list):
      Thread.__init__(self, daemon = True)
      self.state_lock = state_lock
      self.user_data = user_data
      self.stop_event = Event()
      self.my_motor = my_motor
      self.other_motors = other_motors

    def __str__(self):
      return 'MotorLockingThread-'.format(self.my_motor.name)

    def run(self):
      try:
        self.do_run()
      finally:
        sys.exit(1)

    def do_run(self):
      sample_tick_ms = 50

      debug_print('Here we go for {}'.format(self.my_motor.name))
      while not self.stop_event.is_set():
        with self.state_lock:
          debug_print('got here, hmm: {}'.format(self.my_motor.movement_desc_lambda))
          run_desc = self.my_motor.movement_desc_lambda(self.user_data, self.my_motor, self.other_motors)

        debug_print('{} run to: {}'.format(self.my_motor.name, run_desc.absolute_position))
        if run_desc is not None:
          if run_desc.absolute_position is not None:
            self.my_motor.motor.on_to_position(run_desc.speed, run_desc.absolute_position, block = False)
          else:
            self.my_motor.motor.on(run_desc.speed)

        sleep(sample_tick_ms / 1000)

      debug_print('Stopped by caller!')
      self.my_motor.motor.off()

class ArmStateDesc:
  def __init__(self, name: str, pressure_range_pa: range):
    self.name = name
    self.pressure_range_pa = pressure_range_pa

class PneumaticArmMachine:
  STATE_GRABBED = ArmStateDesc('STATE_GRABBED', pressure_range_pa = range(70000, 20000, -1))
  STATE_RELEASED = ArmStateDesc('STATE_RELEASED', pressure_range_pa = range(104000, 120000, 1))

  _STATES = (STATE_GRABBED, STATE_RELEASED)

  def __init__(self, pump, on_off_sensor: TouchSensor):
    self.pump = pump
    self.on_off_sensor = on_off_sensor
    self.ready_to_actuate_event = Event()
    self.ready_to_actuate_event.set()
    self.current_state = None
    self.automatic_thread = self.AutomaticGrabAndReleaseThread(self)

  def calibrate_manually_at_pressed_down_and_outlet_open(self):
    self.pump.calibrate_manually_at_pressed_down_and_outlet_open()
    self.current_state = PneumaticArmMachine.STATE_RELEASED

  def begin_automatic_grab_and_release(self):
    self.automatic_thread.start()
    pass

  def end_automatic_grab_and_release(self):
    self.automatic_thread.shutdown_event.set()
    self.automatic_thread.join()

  def toggle_grab_or_release(self):
    if not self.ready_to_actuate_event.is_set():
      debug_print('Serious state error, button pressed while actuating!')
    else:
      if self.current_state == PneumaticArmMachine.STATE_GRABBED:
        desired_state = PneumaticArmMachine.STATE_RELEASED
      else:
        desired_state = PneumaticArmMachine.STATE_GRABBED
      self.set_arm_state(desired_state)

  def set_arm_state(self, arm_state: ArmStateDesc):
    if arm_state not in PneumaticArmMachine._STATES:
      raise Exception('Unsupported arm_state={}'.format(arm_state))
    
    if self.current_state == arm_state:
      return

    debug_print('Setting arm state to {}'.format(arm_state.name))
    self._pump_until(desired_pressure_range_pa = arm_state.pressure_range_pa)
    self.current_state = arm_state

  def get_arm_state(self):
    return self.current_state

  def _pump_until(self, desired_pressure_range_pa: range):
    debug_print('Waiting for pump to become idle...')
    self.ready_to_actuate_event.wait()
    self.ready_to_actuate_event.clear()
    current_pressure = self.pump.get_current_pressure_pa()
    if current_pressure < min(desired_pressure_range_pa):
      mode = VacuumPumpMachine.MODE_COMPRESSOR
    elif current_pressure > max(desired_pressure_range_pa):
      mode = VacuumPumpMachine.MODE_VACUUM
    else:
      mode = None

    if mode is None:
      debug_print('Pressure already in range at: {}'.format(current_pressure))
    else:
      debug_print('Pumping to reach pressure: {}'.format(desired_pressure_range_pa))
      self.pump.on(mode)
      self.pump.wait_until(desired_pressure_range_pa)
      self.pump.off()

    self.ready_to_actuate_event.set()
    pass

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

class VacuumPumpMachine:
  EXPECTED_ROOM_PRESSURE_PA = range(104000, 110000)

  CYLINDER_MOTOR_SPEED = SpeedPercent(100)
  SWITCH_MOTOR_SPEED = SpeedPercent(100)

  MODE_VACUUM = 1
  MODE_COMPRESSOR = 2

  def __init__(self, cylinder_motor_port: str, switch_motor_port: str, pressure_sensor_port: str):
    self.cylinder_motor = LargeMotor(cylinder_motor_port)
    self.switch_motor = MediumMotor(switch_motor_port)
    self.pressure_sensor = MSPressureSensor(pressure_sensor_port)
    self.is_calibrated = False
    motors = []
    motors.append(LockableMotorDesc(self.cylinder_motor, 'cylinder', self._cylinder_lambda))
    motors.append(LockableMotorDesc(self.switch_motor, 'switch', self._switch_lambda))
    self.mechanical_lock = SoftwareMechanicalLocking(self, motors)

  def calibrate_manually_at_pressed_down_and_outlet_open(self):
    self.cylinder_motor.reset()
    self.switch_motor.reset()

    current_pressure = self.get_current_pressure_pa()
    if current_pressure not in VacuumPumpMachine.EXPECTED_ROOM_PRESSURE_PA:
      raise Exception('Release line pressure first: {}'.format(current_pressure))

    self.is_calibrated = True

  def _ensure_calibrated(self):
    if not self.is_calibrated:
      raise Exception('Must calibrate machine first!')

  def _cylinder_lambda(dummy, self, my_motor: LockableMotorDesc, others):
    return None

  def _switch_lambda(dummy, self, my_motor: LockableMotorDesc, others):
    cylinder_pos = others[0].motor.position

    # Use a bit of math here to determine the oscillations at every 180 deg mark (switch left @
    # 0deg, right @ 180deg, left @ 360deg, etc)
    left_or_right = math.floor((cylinder_pos / 180) % 2)
    if self.mode == VacuumPumpMachine.MODE_COMPRESSOR:
      left_or_right = 1 if left_or_right == 0 else 0
    
    debug_print('Switch should be going to {}'.format(left_or_right * 75))

    # We're claiming to head to 75 deg offsets, but _actually_ mechanically we're on 45 degree pivots.  Apparently the 
    # medium motor has quite a bit of slop for such a high torque move...
    return MovementDesc(speed = VacuumPumpMachine.SWITCH_MOTOR_SPEED, absolute_position = left_or_right * 75)

  def on(self, mode: int):
    self._ensure_calibrated()

    self.off()

    self.mode = mode
    self.cylinder_motor.on(VacuumPumpMachine.CYLINDER_MOTOR_SPEED)

    debug_print('Starting mechanical lock...')
    self.mechanical_lock.start_locking()

  def off(self):
    self._ensure_calibrated()

    self.cylinder_motor.off()

    if self.mechanical_lock.is_active():
      debug_print('Stopping mechanical lock...')
      self.mechanical_lock.stop_locking()

  def reset(self):
    self._ensure_calibrated()
    self.switch_motor.on_to_position(VacuumPumpMachine.SWITCH_MOTOR_SPEED, 0)
    self.cylinder_motor.on_to_position(VacuumPumpMachine.CYLINDER_MOTOR_SPEED, 0)

  def wait_until(self, desired_pressure_range_pa: range, stop_on_motor_stall: bool = True):
    self._ensure_calibrated()
    sample_tick_ms = 100
    while True:
      if stop_on_motor_stall:
        if self.cylinder_motor.wait_until_not_moving(timeout = sample_tick_ms):
          debug_print('Motor stalled, bailing!')
          return False
      else:
        sleep(sample_tick_ms / 1000)

      pressure = self.get_current_pressure_pa()
      debug_print('Pressure reading: {}'.format(pressure))
      if pressure in desired_pressure_range_pa:
        return True

  def get_current_pressure_pa(self):
    return self.pressure_sensor.absolute_pressure_pa

def main():
  sound = Sound()
  touch_sensor = TouchSensor(INPUT_1)
  pump = VacuumPumpMachine(cylinder_motor_port = OUTPUT_A, switch_motor_port = OUTPUT_D, pressure_sensor_port = INPUT_4)
  arm = PneumaticArmMachine(pump, on_off_sensor = touch_sensor)
  arm.calibrate_manually_at_pressed_down_and_outlet_open()
  while True:
    sound.speak('Press button...')
    debug_print('Waiting for touch sensor...')
    if touch_sensor.wait_for_pressed():
      arm.toggle_grab_or_release()
    
if __name__ == "__main__":
  if ev3dev2.get_current_platform() == "fake":
    fake_sys = os.path.join(os.path.dirname(__file__), 'fake-sys')
    ev3dev2.Device.DEVICE_ROOT_PATH = os.path.join(fake_sys, 'arena')

  main()
