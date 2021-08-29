#!/usr/bin/env python3

from enum import Enum
import random
import sys
import os
from time import sleep
from ev3dev2._platform.fake import OUTPUT_B, OUTPUT_C

from ev3dev2.motor import MediumMotor, Motor, OUTPUT_A, OUTPUT_D, SpeedDPS, SpeedPercent, SpeedValue
from ev3dev2.button import Button
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
from ev3dev2.button import Button
from ev3dev2.console import Console
import ev3dev2
from moves_generator import ShuffleSolver

def debug_print(*args, **kwargs):
    print(*args, **kwargs, file=sys.stderr)

class MovementAxis:
    def __init__(self, name: str, motor: Motor, stable_range, zeroed_direction, calibrate_speed: SpeedDPS, nominal_speed: SpeedValue, ramp_up_sp: int, ramp_down_sp: int) -> None:
        self.name = name
        self.motor = motor
        self.stable_range = stable_range
        self.zeroed_direction = zeroed_direction
        self.calibrate_speed = calibrate_speed
        self.nominal_speed = nominal_speed
        self.ramp_up_sp = ramp_up_sp
        self.ramp_down_sp = ramp_down_sp

class ArmStateDesc:
    def __init__(self, name: str, direction: int = None, resistance_range = None, max_resistance_before_stop: float = None, absolute_position: int = None, relative_position: int = None) -> None:
        self.name = name
        self.direction = direction
        self.resistance_range = resistance_range
        self.max_resistance_before_stop = max_resistance_before_stop
        self.absolute_position = absolute_position
        self.relative_position = relative_position

    def __str__(self) -> str:
        return self.name

# Implements the specific geometry of the shuffle bot, with all its weird mechanical quirks :)
class ShuffleBotMachine:
    STACK_COUNT = 9
    INPUT_STACK_INDEX = 0
    OUTPUT_STACK_INDEX = 2

    # Arm is fully pressed down with some amount of "pressure" applied.
    PRESSED_DOWN = ArmStateDesc(name = "PRESSED_DOWN", direction = -1, resistance_range = range(-120, -9999, -1), max_resistance_before_stop = 0.50)

    # Arm is slightly raised after being pressed down to allow cards stuck to the bottom of the top card to fall before we add
    # any more force by accelerating upward.
    CONFIRMING_GRAB = ArmStateDesc(name = "CONFIRMING_GRAB", relative_position = 80)

    # Arm is fully raised in the "release" position (a physical obstruction exists along the grabber arm path such that
    # no object should be grabbed in this state).
    UP_RELEASED = ArmStateDesc(name = "UP_RELEASED", absolute_position = -2)

    # Special variation of UP_RELEASED that is slightly relaxed from the zeroed position as the machinery has slightly less
    # friction when translating in this configuration.
    UP_READY_TO_TRANSLATE = ArmStateDesc(name = 'UP_READY_TO_TRANSLATE', absolute_position = -15)

    # Arm is raised but in a position that would permit holding onto a grabbed object.
    UP_HOLDING = ArmStateDesc(name = "UP_HOLDING", absolute_position = -72)

    ARM_STATES = (PRESSED_DOWN, CONFIRMING_GRAB, UP_RELEASED, UP_READY_TO_TRANSLATE, UP_HOLDING)

    ROW_Y_POSITIONS = (
        84,
        519,
        957,
    )

    COLUMN_X_POSITIONS = (
        41,
        282,
        528,
    )

    Z_AXIS = MovementAxis(
        name = 'UpDownMotor',
        motor = MediumMotor(OUTPUT_D),
        stable_range = range(0, -320, -1),
        zeroed_direction = 1, # UP
        calibrate_speed = SpeedDPS(100),
        nominal_speed = SpeedPercent(50),
        ramp_up_sp = 0,
        ramp_down_sp = 0)
    X_AXIS = MovementAxis(
        name = 'LeftRightMotor',
        motor = MediumMotor(OUTPUT_B),
        stable_range = range(0, 545),
        zeroed_direction = -1, # RIGHT
        calibrate_speed = None, # must manually calibrate for now...
        nominal_speed = SpeedPercent(80),
        ramp_up_sp = 0,
        ramp_down_sp = 200)
    Y_AXIS = MovementAxis(
        name = 'FrontBackMotor',
        motor = MediumMotor(OUTPUT_C),
        stable_range = range(0, 1150), # TODO: didn't measure precisely
        zeroed_direction = -1, # BACK
        calibrate_speed = SpeedDPS(300),
        nominal_speed = SpeedPercent(80),
        ramp_up_sp = 0,
        ramp_down_sp = 200)

    def __init__(self) -> None:
        self.gantry = ThreeAxisGantryMachine(self.ARM_STATES, self.X_AXIS, self.Y_AXIS, self.Z_AXIS)

    def calibrate_manually_arm_is_furthest_from_motors(self):
        self.gantry.calibrate_manually_arm_is_furthest_from_motors()

    def reset_positions(self):
        self.gantry.reset_positions()

    def move_item(self, from_stack, to_stack):
        self._move_to_stack(from_stack, to_stack)

    def _move_to_stack(self, from_stack, to_stack):
        gantry = self.gantry

        if from_stack == to_stack and from_stack is not None:
            return
        if from_stack is not None:
            self._translate_to_stack(from_stack)
        gantry.set_arm_state(self.PRESSED_DOWN)
        gantry.set_arm_state(self.CONFIRMING_GRAB)
        sleep(1.5)
        gantry.set_arm_state(self.UP_HOLDING)
        if to_stack is not None:
            self._translate_to_stack(to_stack)
        gantry.set_arm_state(self.UP_RELEASED)
        gantry.set_arm_state(self.UP_READY_TO_TRANSLATE)

    def _translate_to_stack(self, stack_index):
        rowcol = self._stack_to_rowcol(stack_index)
        row_index = rowcol[0]
        column_index = rowcol[1]
        self.gantry.translate_arm_xy(self.COLUMN_X_POSITIONS[column_index], self.ROW_Y_POSITIONS[row_index])

    def _stack_to_rowcol(self, stack_index):
        rows = len(self.ROW_Y_POSITIONS)
        return (int(stack_index / rows), stack_index % rows)

    def grab_test(self):
        self.move_item(None, None)

class ThreeAxisGantryMachine:

    def __init__(self, arm_states, x_axis: MovementAxis, y_axis: MovementAxis, z_axis: MovementAxis) -> None:
        self.x_axis = x_axis
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.axes = (z_axis, x_axis, y_axis)
        self.arm_states = arm_states
        self.current_state = None
        self.current_xy = None
        self.is_calibrated = False

    def _calibrate_individual(self, axis: MovementAxis, direction: int, speed: SpeedValue, calibrationTarget: int):
        debug_print("Calibrating {} by stepping {} up to {}".format(
            axis.name, direction, axis.stable_range))
        motor = axis.motor

        distanceTraveled = self._on_until_resists(axis, direction, speed)

        if calibrationTarget is not None:
            motor.position = calibrationTarget

        debug_print("...distance traveled: {}, new position: {}".format(distanceTraveled, motor.position))

    def _on_until_resists(self, axis: MovementAxis, direction: int, resistance_range, max_resistance, speed: SpeedValue) -> int:
        motor = axis.motor
        startPos = motor.position
        simpleMethod = True
        if simpleMethod == True:
            motor.on(speed * direction, brake = True, block = True)
        else:
            motor.on(speed * direction)
            motor.wait_until('running', timeout = 100)
            lastSpeed = motor.speed
            topSpeed = -1
            while motor.wait_until_not_moving(timeout = 100) is False:
                currentPos = motor.position
                currentSpeed = motor.speed

                if currentPos not in axis.stable_range:
                    debug_print("No resistance detected but outside of stable range at {}!".format(currentPos))
                    break

                if currentSpeed > topSpeed:
                    topSpeed = currentSpeed

                if currentPos in resistance_range:
                    speedChange = abs(currentSpeed) - abs(lastSpeed)
                    speedChangeFromMax = abs(topSpeed - currentSpeed)
                    relativeResistance = -speedChange / abs(motor.speed_sp)
                    debug_print("speed={}, speed_sp={}, speedChange={}, speedChangeFromMax={}".format(currentSpeed, motor.speed_sp, speedChange, speedChangeFromMx))
                    debug_print("...relativeResistance={}".format(relativeResistance))

                    if relativeResistance > max_resistance:
                        debug_print("Encountered resistance of {}, we're satisfied...".format(relativeResistance))
                        break

                lastSpeed = currentSpeed

        motor.off(brake = False)
        motor.wait_until_not_moving()

        return abs(motor.position - startPos)

    def calibrate_manually_arm_is_furthest_from_motors(self):
        for axis in self.axes:
            axis.motor.off()
            axis.motor.wait_until_not_moving()
            axis.motor.reset()
            axis.motor.ramp_down_sp = axis.ramp_down_sp
            axis.motor.ramp_up_sp = axis.ramp_up_sp

        # Automatically calibrate the torque safe axes.  The order is important here, up-down must go first to make sure
        # the gantry can physically translate!
        torque_safe_axes = (self.z_axis, self.y_axis)
        for axis in torque_safe_axes:
            debug_print('Calibrating axis {} (at {})'.format(axis.name, axis.motor.address))

            self._on_with_stop(axis.motor, speed = axis.calibrate_speed * axis.zeroed_direction, stop_action = Motor.STOP_ACTION_BRAKE)
            axis.motor.wait_until('stalled')
            debug_print('...motor is: {}'.format(axis.motor.state))
            axis.motor.off()
            axis.motor.wait_until_not_moving()
            axis.motor.reset()

        self.current_xy = (0, 0)
        self.is_calibrated = True

    def reset_positions(self):
        if self.is_calibrated == False:
            raise Exception('Must be calibrated already!')
        
        for axis in (self.z_axis, self.x_axis, self.y_axis):
            motor = axis.motor
            self._on_to_position_with_stop(motor, speed = axis.nominal_speed, position = 0, stop_action = Motor.STOP_ACTION_BRAKE)

    def calibrate_automatically_wip(self):
        raise Exception("This probably will break your machine, not well tested or adjusted yet...")

    def _ensure_calibrated(self):
        if not self.is_calibrated:
            debug_print("Forcing calibration...")
            self.calibrate_automatically_wip()

    def set_arm_state(self, target: ArmStateDesc, override_speed: SpeedValue = None):
        if target not in self.arm_states:
            raise Exception("Unsupported target: {}".format(target))
        if self.current_state == target:
            return
        self._ensure_calibrated()

        debug_print("Setting arm state to {}...".format(target))

        axis = self.z_axis

        if override_speed:
            debug_print("...using override speed of {}".format(override_speed))
            speed = override_speed
        else:
            speed = axis.nominal_speed

        if target.max_resistance_before_stop is not None:
            self._on_until_resists(axis, target.direction, target.resistance_range, target.max_resistance_before_stop, speed)
        else:
            resolved_position = None
            if target.relative_position is not None:
                resolved_position = axis.motor.position + target.relative_position
            elif target.absolute_position is not None:
                resolved_position = target.absolute_position
            else:
                raise Exception("Unsupported target arm state: {}".format(target))

            if resolved_position not in axis.stable_range:
                raise Exception("Calibration or configuration error, illegal position: {}".format(resolved_position))

            self._on_to_position_with_stop(axis.motor, speed, resolved_position, Motor.STOP_ACTION_HOLD)

        debug_print("...done, up-down motor at {}".format(axis.motor.position))

    def translate_arm_xy(self, x, y):
        target_xy = (x, y)
        if self.current_xy == target_xy:
            return

        debug_print("Translating arm to {}...".format(target_xy))
        translate_axes = []

        if self.current_xy[0] != x:
            translate_axes.append((self.x_axis, x))
        if self.current_xy[1] != y:
            translate_axes.append((self.y_axis, y))

        # Get them both moving at the same time...
        for (axis, position_sp) in translate_axes:
            self._translate_arm_raw(axis, position_sp)

        # Then block waiting for both to stop moving.
        for (axis, position_sp) in translate_axes:
            axis.motor.wait_until('running', 100)
            axis.motor.wait_until_not_moving()

        debug_print("...done, motors that moved are now at:")
        for (axis, position_sp) in translate_axes:
            debug_print('    {}: {}'.format(axis.name, axis.motor.position))

        self.current_xy = target_xy

    def _translate_arm_raw(self, axis, position_sp: int):
        if position_sp not in axis.stable_range:
            raise Exception("Position out of range of {} {}: {}".format(axis.name, axis.stable_range, position_sp))

        self._on_to_position_with_stop(axis.motor, axis.nominal_speed, position_sp, Motor.STOP_ACTION_HOLD, block = False)

    # Mysteriously, on_* commands all require a stop_action of either hold or coast, not break.  We don't want hold as we might
    # slightly overshoot our position or encounter some mechanical resistance and we want to just "let it go" and
    # adjust back into a clean spot.  I experimented with this very briefly and found that without this behaviour it was quite
    # easy to rip the chain off the left/right axis especially.
    def _on_to_position_with_stop(self, motor: Motor, speed: SpeedValue, position: int, stop_action: str, block: bool = True):
        real_speed = speed.to_native_units(motor)
        motor.speed_sp = int(round(real_speed))
        motor.position_sp = position
        motor.stop_action = stop_action
        motor.run_to_abs_pos()

        if block:
            motor.wait_until('running', timeout=100)
            motor.wait_until_not_moving()

    def _on_with_stop(self, motor: Motor, speed: SpeedValue, stop_action: str):
        real_speed = speed.to_native_units(motor)
        motor.speed_sp = int(round(real_speed))
        motor.stop_action = stop_action
        motor.run_forever()

class SimpleGuiPrompt:
    def __init__(self, console: Console, button: Button, leds: Leds):
        self.console = console
        self.button = button
        self.leds = leds

    def prompt_number(self, prompt: str, starting_value: int) -> int:
        value = starting_value
        self.leds.all_off()
        self.leds.set_color('LEFT', 'ORANGE', pct = 0.2)
        self.leds.set_color('RIGHT', 'ORANGE', pct = 0.2)
        while True:
            self.console.reset_console()
            self.console.text_at('{}{}\n'.format(prompt, value), column = 1, row = 3)
            pressed = self._wait_for_button_press()
            if pressed == 'up' or pressed == 'right':
                value += 1
            elif pressed == 'down' or pressed == 'left':
                value -= 1
            elif pressed == 'enter':
                break
        self.leds.all_off()
        return value

    def _wait_for_button_press(self):
        button = self.button
        pressed = None
        while True:
            allpressed = button.buttons_pressed
            if bool(allpressed):
                pressed = allpressed[0]
                while not button.wait_for_released(pressed):
                    pass
                break
        return pressed

class ShuffleBotMain:
    def main(self):
        sound = Sound()
        machine = ShuffleBotMachine()
        leds = Leds()
        button = Button()
        console = Console()

        deck_size = SimpleGuiPrompt(console, button, leds).prompt_number('Deck size: ', starting_value = 20)
        debug_print('Running with deck size of {}'.format(deck_size))

        sound.speak('Manual calibration')
        machine.calibrate_manually_arm_is_furthest_from_motors()

        sound.speak('Shuffling')
        solver = ShuffleSolver(deck_size, machine.STACK_COUNT, machine.INPUT_STACK_INDEX, machine.OUTPUT_STACK_INDEX)
        moves = solver.solve()
        debug_print('Moves:')
        for move in moves:
            debug_print('  {} -> {}'.format(move[0], move[1]))

        sound.speak('Here we go, {} moves'.format(len(moves)))

        progress_interval = 25
        next_progress_notice = progress_interval
        for move_index in range(len(moves)):
            pct_complete = move_index / len(moves) * 100
            if pct_complete > next_progress_notice and next_progress_notice < 100:
                sound.speak('{} percent complete'.format(next_progress_notice), play_type = Sound.PLAY_NO_WAIT_FOR_COMPLETE)
                next_progress_notice += progress_interval

            move = moves[move_index]
            src = move[0]
            dst = move[1]
            sound.speak('Move {} to {}'.format(src, dst), play_type = Sound.PLAY_NO_WAIT_FOR_COMPLETE)
            machine.move_item(src, dst)

        sound.speak('Finished', play_type = Sound.PLAY_NO_WAIT_FOR_COMPLETE)
        machine.reset_positions()

class FiguringStuffOutBebot:
    def __init__(self):
        self.sound = Sound()
        self.machine = ShuffleBotMachine()

    def just_calibrate(self):
        self.machine.calibrate_manually_arm_is_furthest_from_motors()

    def translate_test(self):
        self.sound.speak('Calibrating')
        self.machine.calibrate_manually_arm_is_furthest_from_motors()
        for i in range(0, self.machine.STACK_COUNT):
            self.sound.speak('Move to {}'.format(i))
            self.machine._translate_to_stack(i)
    
    def input_test(self):
        leds = Leds()
        button = Button()
        console = Console()
    
        deck_size = SimpleGuiPrompt(console, button, leds).prompt_number('Deck size: ', starting_value = 40)
        debug_print('Got {}'.format(deck_size))

    def move_test(self):
        sound = self.sound
        machine = self.machine
        sound.speak('Calibrating...')
        machine.calibrate_manually_arm_is_furthest_from_motors()
        sound.speak('Ready!')
        moves = (
            (1, 2),
            (1, 3),
            (1, 2),
            (1, 3),
            (1, 2),
            (2, 3),
            (2, 1),
            (1, 2),
            (1, 2),
            (1, 2),
            (1, 2),
            (1, 2),
            (1, 2),
            (1, 2),
            (2, 1),
            (2, 1),
            (2, 1),
            (1, 2),
            (1, 2),
            (1, 2),
            (1, 2),
        )
        for move in moves:
            src = move[0]
            dst = move[1]
            sound.speak('Moving {} to {}'.format(src, dst), play_type = Sound.PLAY_NO_WAIT_FOR_COMPLETE)
            machine.move_item(src, dst)

    def up_down_test(self):
        sound = self.sound
        machine = self.machine
        sound.speak('Calibrating...')
        machine.calibrate_manually_arm_is_furthest_from_motors()
        sound.speak('Ready!')
        for i in range(5):
            sound.speak('Grab test')
            machine.grab_test()

    def all_around_test(self):
        sound = self.sound
        gantry = self.machine
        sound.speak('Calibrating...')
        gantry.calibrate_manually_arm_is_furthest_from_motors()
        gantry.set_arm_state(gantry.UP_HOLDING)
        sound.speak('Ready!')
        for i in range(10):
            columns = [0, 1, 2]
            random.shuffle(columns)
            for column in columns:
                sound.speak('Go to {}'.format(column))
                gantry.translate_arm_to_column(column)
                sleep(2)
            sound.speak('Holding')
            gantry.set_arm_state(gantry.UP_HOLDING)
            sleep(2)
            sound.speak('Releasing')
            gantry.set_arm_state(gantry.UP_RELEASED)
            sleep(2)
            sound.speak('Pressing down')
            gantry.set_arm_state(gantry.PRESSED_DOWN)
            sleep(2)
            sound.speak('Releasing')
            gantry.set_arm_state(gantry.UP_RELEASED)
            sound.beep()
            sleep(2)

def main():
    if ev3dev2.get_current_platform() == "fake":
        fake_sys = os.path.join(os.path.dirname(__file__), 'fake-sys')
        ev3dev2.Device.DEVICE_ROOT_PATH = os.path.join(fake_sys, 'arena')

    # FiguringStuffOutBebot().translate_test()
    ShuffleBotMain().main()

if __name__ == '__main__':
    main()
