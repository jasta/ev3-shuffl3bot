#!/usr/bin/env python3

from enum import Enum
import random
import sys
import os
from time import sleep

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
    def __init__(self, name: str, motor: Motor, stable_range, zeroed_direction, init_speed_removeme: SpeedDPS, nominal_speed: SpeedValue, ramp_up_sp: int, ramp_down_sp: int) -> None:
        self.name = name
        self.motor = motor
        self.stable_range = stable_range
        self.zeroed_direction = zeroed_direction
        self.init_speed_removeme = init_speed_removeme
        self.nominal_speed = nominal_speed
        self.ramp_up_sp = ramp_up_sp
        self.ramp_down_sp = ramp_down_sp

class ArmStateDesc:
    def __init__(self, name: str, direction: int, resistance_range = None, max_resistance_before_stop: float = None, absolute_position: int = None, relative_position: int = None) -> None:
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
    STACK_COUNT = 4
    INPUT_STACK_INDEX = 0
    OUTPUT_STACK_INDEX = 3

    # Arm is fully pressed down with some amount of "pressure" applied.
    PRESSED_DOWN = ArmStateDesc(name = "PRESSED_DOWN", direction = -1, resistance_range = range(-120, -9999, -1), max_resistance_before_stop = 0.50)

    # Arm is slightly raised after being pressed down to allow cards stuck to the bottom of the top card to fall before we add
    # any more force by accelerating upward.
    CONFIRMING_GRAB = ArmStateDesc(name = "CONFIRMING_GRAB", direction = 1, relative_position = 80)

    # Just for testing pretend to go down but don't push hard...
    FAKE_PRESSED_DOWN = ArmStateDesc(name = "FAKE_PRESSED_DOWN", direction = -1, absolute_position = -220)

    # Arm is fully raised in the "release" position (a physical obstruction exists along the grabber arm path such that
    # no object should be grabbed in this state).
    UP_RELEASED = ArmStateDesc(name = "UP_RELEASED", direction = 1, absolute_position = -2)

    # Arm is raised but in a position that would permit holding onto a grabbed object.
    UP_HOLDING = ArmStateDesc(name = "UP_HOLDING", direction = 1, absolute_position = -72)

    ARM_STATES = (PRESSED_DOWN, CONFIRMING_GRAB, FAKE_PRESSED_DOWN, UP_RELEASED, UP_HOLDING)

    # We actually define 6 separate columns for only 4 card stacks because the entire gantry machine is technically offset slightly
    # such that the left and right-most stacks don't pick up or drop off centered on the card.  To account for this we need the machine
    # to understand that some adjustments are required when going from the first stack or to the last stack.  For example,
    # to drop a card off from stack[0] to stack[1], we actually translate from column[0] to column[2] to account for the offset.  To
    # then move that card from stack[1] to stack[2] we'd go from column[2] to column[3] per normal.
    COLUMN_POSITIONS = (
        543,
        393,
        313, # Offset position for stack[1]
        172,
        94, # Offset position for stack[2]
        0,
    )

    STACK_CENTER_GRAB_COLUMNS = (
        -1,
        1,
        3,
        -1,
    )

    STACK_RIGHT_GRAB_COLUMNS = (
        0,
        2,
        4,
        -1,
    )

    STACK_LEFT_OR_CENTER_DROP_COLUMNS = (
        -1,
        1,
        3,
        5,
    )

    STACK_RIGHT_DROP_COLUMNS = STACK_RIGHT_GRAB_COLUMNS

    INPUT_COLUMN_INDEX = 0
    OUTPUT_COLUMN_INDEX = 3

    UP_DOWN_AXIS = MovementAxis(
        name = 'UpDownMotor',
        motor = MediumMotor(OUTPUT_D),
        stable_range = range(0, -260, -1),
        zeroed_direction = 1, # UP
        init_speed_removeme = SpeedDPS(50),
        nominal_speed = SpeedPercent(50),
        ramp_up_sp = 0,
        ramp_down_sp = 0)
    LEFT_RIGHT_AXIS = MovementAxis(
        name = 'LeftRightMotor',
        motor = MediumMotor(OUTPUT_A),
        stable_range = range(0, 545),
        zeroed_direction = -1, # RIGHT
        init_speed_removeme = SpeedDPS(100),
        nominal_speed = SpeedPercent(80),
        ramp_up_sp = 0,
        ramp_down_sp = 200)

    def __init__(self) -> None:
        self.gantry = TwoAxisGantryMachine(self.ARM_STATES, self.COLUMN_POSITIONS, self.UP_DOWN_AXIS, self.LEFT_RIGHT_AXIS)

    def calibrate_manually_as_top_right(self):
        self.gantry.calibrate_manually_as_top_right()

    def reset_positions(self):
        self.gantry.reset_positions()

    def move_item(self, from_stack, to_stack):
        if from_stack is None and to_stack is None:
            self._move_column_raw(None, None)
        else:
            column_moves = self._stack_translation_to_column_moves(from_stack, to_stack)
            for column_move in column_moves:
                self._move_column_raw(column_move[0], column_move[1])

    def _move_column_raw(self, from_column, to_column):
        gantry = self.gantry

        if from_column == to_column and from_column is not None:
            return
        if from_column is not None:
            gantry.translate_arm_to_column(from_column)
        gantry.set_arm_state(self.PRESSED_DOWN)
        gantry.set_arm_state(self.CONFIRMING_GRAB)
        sleep(1.5)
        gantry.set_arm_state(self.UP_HOLDING)
        if to_column is not None:
            gantry.translate_arm_to_column(to_column)
        gantry.set_arm_state(self.UP_RELEASED)
        gantry.set_arm_state(self.UP_HOLDING)

    def grab_test(self):
        self.move_item(None, None)

    def _stack_translation_to_column_moves(self, from_stack, to_stack) -> list:
        columns = None
        if from_stack == 0 or to_stack == 0:
            if to_stack == 3:
                return self._stack_translation_to_column_moves(0, 2) + self._stack_translation_to_column_moves(2, 3)
            else:
                columns = (self.STACK_RIGHT_GRAB_COLUMNS, self.STACK_RIGHT_DROP_COLUMNS)
        else:
            columns = (self.STACK_CENTER_GRAB_COLUMNS, self.STACK_LEFT_OR_CENTER_DROP_COLUMNS)

        from_column = columns[0][from_stack]
        to_column = columns[1][to_stack]
        if from_column == -1 or to_column == -1:
            raise Exception('Illegal move: {} to {}'.format(from_column, to_column))

        return [ (from_column, to_column) ]

class TwoAxisGantryMachine:

    def __init__(self, arm_states, column_positions, up_down_axis: MovementAxis, left_right_axis: MovementAxis) -> None:
        self.up_down_axis = up_down_axis
        self.left_right_axis = left_right_axis
        self.axes = (up_down_axis, left_right_axis, )
        self.arm_states = arm_states
        self.column_positions = column_positions
        self.current_state = None
        self.current_column = None
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

    def calibrate_manually_as_top_right(self):
        for axis in self.axes:
            motor = axis.motor

            motor.reset()

            motor.ramp_up_sp = axis.ramp_up_sp
            motor.ramp_down_sp = axis.ramp_down_sp

        self.is_calibrated = True

    def reset_positions(self):
        if self.is_calibrated == False:
            raise Exception('Must be calibrated already!')
        
        for axis in (self.up_down_axis, self.left_right_axis):
            motor = axis.motor
            self._to_position_with_brake(motor, speed = axis.init_speed_removeme, position = 0)

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

        axis = self.up_down_axis

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

            self._to_position_with_brake(axis.motor, speed, resolved_position)

        debug_print("...done")

    def translate_arm_to_column(self, column_index):
        if self.current_column == column_index:
            return

        debug_print("Translating arm to {}...".format(column_index))
        self._translate_arm_raw(self.column_positions[column_index])
        debug_print("...done")

        self.current_column = column_index

    def _translate_arm_raw(self, absolute_position_sp: int):
        axis = self.left_right_axis
        if absolute_position_sp not in axis.stable_range:
            raise Exception("Position out of range of {}: {}".format(axis.stable_range, absolute_position_sp))

        self._to_position_with_brake(axis.motor, axis.nominal_speed, absolute_position_sp)

    # Mysteriously, on_* commands all require a stop_action of either hold or coast, not break.  We don't want hold as we might
    # slightly overshoot our position or encounter some mechanical resistance and we want to just "let it go" and
    # adjust back into a clean spot.  I experimented with this very briefly and found that without this behaviour it was quite
    # easy to rip the chain off the left/right axis especially.
    def _to_position_with_brake(self, motor: Motor, speed: SpeedValue, position: int):
        real_speed = speed.to_native_units(motor)
        motor.speed_sp = int(round(real_speed))
        motor.position_sp = position
        motor.stop_action = Motor.STOP_ACTION_BRAKE
        motor.run_to_abs_pos()
        motor.wait_until('running', timeout=100)
        motor.wait_until_not_moving()

    def _on_with_brake(self, motor: Motor, speed: SpeedValue):
        real_speed = speed.to_native_units(motor)
        motor.speed_sp = int(round(real_speed))
        motor.stop_action = Motor.STOP_ACTION_BRAKE
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

        deck_size = SimpleGuiPrompt(console, button, leds).prompt_number('Deck size: ', starting_value = 10)
        debug_print('Running with deck size of {}'.format(deck_size))

        sound.speak('Manual calibration')
        machine.calibrate_manually_as_top_right()

        sound.speak('Shuffling')
        solver = ShuffleSolver(deck_size, machine.STACK_COUNT, machine.INPUT_COLUMN_INDEX, machine.OUTPUT_COLUMN_INDEX)
        moves = solver.solve()

        sound.speak('Here we go, {} moves'.format(len(moves)))

        progress_interval = 10
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
        machine.calibrate_manually_as_top_right()
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
        machine.calibrate_manually_as_top_right()
        sound.speak('Ready!')
        for i in range(5):
            sound.speak('Grab test')
            machine.grab_test()

    def all_around_test(self):
        sound = self.sound
        gantry = self.machine
        sound.speak('Calibrating...')
        gantry.calibrate_manually_as_top_right()
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

def fucking_around():
    b = Button()
    s = Sound()
    m = LargeMotor(OUTPUT_A)
    while True:
        debug_print("Waiting for ENTER...")
        b.wait_for_released('enter')
        s.beep()
        m.on(SpeedPercent(10))
        startPos = m.position
        while m.wait_until_not_moving(timeout = 100) is False:
            currentPos = m.position
            offsetDegrees = abs(currentPos - startPos) % 360
            debug_print("Current speed is: {} (of {})".format(m.speed, m.speed_sp))
            debug_print("Offset is {}".format(offsetDegrees))
            debug_print("State is: {}".format(m.state))
        debug_print("State is now: {}".format(m.state))
        m.stop()

def main():
    if ev3dev2.get_current_platform() == "fake":
        fake_sys = os.path.join(os.path.dirname(__file__), 'fake-sys')
        ev3dev2.Device.DEVICE_ROOT_PATH = os.path.join(fake_sys, 'arena')

    #fucking_around()
    # FiguringStuffOutBebot().input_test()
    ShuffleBotMain().main()

if __name__ == '__main__':
    main()
