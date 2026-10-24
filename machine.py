import math
import threading
from enum import Enum, auto

from relay import Arduino, ArduinoError


# settings
MOTOR_0 = 0               # spindle (winding); its direction is set by hand
MOTOR_1 = 1               # traverse

STEPS_PER_MM = 80.0       # TODO: set from lead screw pitch + microstepping
STEPS_PER_REV = 200       # TODO: set from motor type + microstepping
WIND_SPEED = 30           # steps/s, spindle speed during automatic winding
SEMI_WIND_SPEED = WIND_SPEED  # steps/s, spindle speed in semi-manual mode (traverse follows it)
MANUAL_SPEED_0 = 30       # steps/s, default spindle speed in manual mode
MANUAL_SPEED_1 = 300      # steps/s, default traverse speed in manual mode
ACCEL = 100               # steps/s^2 (Arduino accepts motor_min_accel..399)
MIN_SPEED = 1             # Arduino minimum speed
MAX_SPEED = 999           # Arduino maximum speed (SPD accepts 1..999)
MAX_STP = 32000           # Arduino limit for one STP command

DIR_LEFT = 0              # TODO: swap if the motor turns the wrong way
DIR_RIGHT = 1

# state machine
class Status(Enum):
    RESETTING = auto()
    INITIALIZING = auto()
    IDLE = auto()
    SEMI_MANUAL = auto()     # spindle + traverse together, active while the key/button is held
    MANUAL = auto()          # one motor only (spindle or traverse), active while the key/button is held
    RUNNING = auto()         # automatic winding
    STOPPING = auto()
    KILL = auto()


class Event(Enum):
    RESET_DONE = auto()
    INIT_DONE = auto()
    # automatic winding
    START = auto()
    STOP = auto()                  # gentle stop requested
    TOGGLE_AUTO = auto()           # Shift: start when idle, stop when running
    STOPPED = auto()               # motion actually finished
    # semi-manual mode (spindle + traverse synchronised)
    SEMI_WIND_FWD = auto()         # key PRESSED: wind (adds turns)
    SEMI_WIND_BACK = auto()        # key PRESSED: unwind (removes turns)
    SEMI_JOG_FWD = auto()
    SEMI_JOG_BACK = auto()
    # manual mode (one motor only)
    MANUAL_WIND_FWD = auto()       # key PRESSED: spindle only, adds turns
    MANUAL_WIND_BACK = auto()      # key PRESSED: spindle only, removes turns
    MANUAL_JOG_FWD = auto()
    MANUAL_JOG_BACK = auto()
    TRAVERSE_LEFT = auto()         # key PRESSED: traverse only
    TRAVERSE_RIGHT = auto()        # key PRESSED: traverse only
    HOLD_RELEASED = auto()         # key/button RELEASED (both modes)
    EMERGENCY_STOP = auto()


class Machine:
    def __init__(self, hw: Arduino):
        self.hw = hw
        self.status = Status.RESETTING
        self.coil_diameter = 0.0
        self.coil_length = 0.0
        self.wire_diameter = 0.0
        self.wire_length = 0.0
        self.turns_needed = 0.0
        self.jog_size = 0.0
        self._decel_timer = None

        # directions
        self.wind_dir = DIR_RIGHT        # spindle direction, set by hand (Up/Down in the GUI)
        self.traverse_dir = +1           # traverse direction while winding (+1 = DIR_RIGHT)

        # manual mode speeds, steps/s
        self.manual_speed_0 = MANUAL_SPEED_0
        self.manual_speed_1 = MANUAL_SPEED_1

        # winding progress
        self.turns_done = 0.0            # turns wound so far (whole coil)
        self._tracking = False           # True while spindle steps are counted into turns_done
        self._turns_at_start = 0.0       # turns_done when the current run started
        self._steps_at_start = 0         # spindle step counter when the current run started
        self._turns_sign = +1            # +1 = this run adds turns, -1 = removes turns
        self._jog_target = None          # spindle steps of a running jog (None = not a jog)

        # (state, event) -> (new_state, action)
        self.table = {
            (Status.RESETTING,    Event.RESET_DONE):       (Status.INITIALIZING, self.start_init),
            (Status.INITIALIZING, Event.INIT_DONE):        (Status.IDLE,         None),
            # automatic winding
            (Status.IDLE,         Event.START):            (Status.RUNNING,      self.start_winding),
            (Status.IDLE,         Event.TOGGLE_AUTO):      (Status.RUNNING,      self.start_winding),
            (Status.RUNNING,      Event.STOP):             (Status.STOPPING,     self.begin_decel),
            (Status.RUNNING,      Event.TOGGLE_AUTO):      (Status.STOPPING,     self.begin_decel),
            (Status.STOPPING,     Event.STOPPED):          (Status.IDLE,         self.finish_stop),
            # semi-manual mode
            (Status.IDLE,         Event.SEMI_WIND_FWD):    (Status.SEMI_MANUAL,  self.semi_wind_fwd),
            (Status.IDLE,         Event.SEMI_WIND_BACK):   (Status.SEMI_MANUAL,  self.semi_wind_back),
            (Status.SEMI_MANUAL,  Event.HOLD_RELEASED):    (Status.IDLE,         self.stop_hold),
            (Status.IDLE,         Event.SEMI_JOG_FWD):     (Status.IDLE,         self.semi_jog_fwd),
            (Status.IDLE,         Event.SEMI_JOG_BACK):    (Status.IDLE,         self.semi_jog_back),
            # manual mode
            (Status.IDLE,         Event.MANUAL_WIND_FWD):  (Status.MANUAL,       self.manual_wind_fwd),
            (Status.IDLE,         Event.MANUAL_WIND_BACK): (Status.MANUAL,       self.manual_wind_back),
            (Status.IDLE,         Event.TRAVERSE_LEFT):    (Status.MANUAL,       self.traverse_left),
            (Status.IDLE,         Event.TRAVERSE_RIGHT):   (Status.MANUAL,       self.traverse_right),
            (Status.MANUAL,       Event.HOLD_RELEASED):    (Status.IDLE,         self.stop_hold),
            (Status.IDLE,         Event.MANUAL_JOG_FWD):   (Status.IDLE,         self.manual_jog_fwd),
            (Status.IDLE,         Event.MANUAL_JOG_BACK):  (Status.IDLE,         self.manual_jog_back),
        }

    def set_parameters(self, coil_diameter, coil_length, wire_length, wire_diameter, jog_size):
        if self.status != Status.IDLE:
            raise RuntimeError("Cannot set parameters while machine is not idle.")
        if coil_diameter <= 0 or coil_length <= 0 or wire_length <= 0 or wire_diameter <= 0 or jog_size <= 0:
            raise ValueError("All parameters must be positive numbers.")
        self.coil_diameter = coil_diameter
        self.wire_diameter = wire_diameter
        self.coil_length = coil_length
        self.wire_length = wire_length
        self.jog_size = jog_size
        self.turns_needed = self._compute_turns_needed()
        self.turns_done = 0.0            # new parameters = new coil

    def set_manual_speeds(self, speed_0, speed_1):
        """Speeds for manual mode, steps/s: speed_0 = spindle, speed_1 = traverse.
        Used from the next start of a manual motion."""
        for speed in (speed_0, speed_1):
            if not MIN_SPEED <= speed <= MAX_SPEED:
                raise ValueError(f"Manual speed must be {MIN_SPEED}..{MAX_SPEED} steps/s.")
        self.manual_speed_0 = int(speed_0)
        self.manual_speed_1 = int(speed_1)

    def _compute_turns_needed(self):
        """Turns needed to use up wire_length. Wire is laid in layers: each layer
        holds coil_length / wire_diameter turns and raises the diameter by 2 * wire_diameter."""
        turns_per_layer = max(1, int(self.coil_length // self.wire_diameter))
        remaining = self.wire_length
        turns = 0.0
        layer = 0
        while remaining > 0:
            diameter = self.coil_diameter + self.wire_diameter * (2 * layer + 1)
            per_turn = math.pi * diameter
            layer_length = turns_per_layer * per_turn
            if remaining >= layer_length:
                turns += turns_per_layer
                remaining -= layer_length
                layer += 1
            else:
                turns += remaining / per_turn
                remaining = 0
        return turns

    # --- progress ---
    def poll(self):
        """Call periodically from the GUI: updates turns_done."""
        self.update_progress()

    def update_progress(self):
        """Read spindle step counter and update turns_done."""
        if not self._tracking:
            return
        try:
            steps = self.hw.total_steps(MOTOR_0)
        except (ArduinoError, ValueError):
            return                       # keep the last known value
        # abs(): the Arduino counter goes negative when the spindle turns in DIR_LEFT,
        # so the direction of the run is given by _turns_sign instead
        delta_steps = abs(steps - self._steps_at_start)
        if self._jog_target is not None and delta_steps >= self._jog_target:
            delta_steps = self._jog_target
            self._jog_target = None      # jog finished, stop tracking
            self._tracking = False
        delta = delta_steps / STEPS_PER_REV
        self.turns_done = max(0.0, self._turns_at_start + self._turns_sign * delta)

    def read_end_switch(self):
        """(closed_9, closed_10, hits) from the Arduino, or None when it could not be read.
        The Arduino handles the switches by itself (stops the spindle, backs the traverse off
        and stops it); this only reports it."""
        try:
            return self.hw.end_switch()
        except (ArduinoError, ValueError):
            return None

    @property
    def turns_remaining(self):
        return max(0.0, self.turns_needed - self.turns_done)

    @property
    def time_total(self):
        """Seconds to wind the whole coil at WIND_SPEED."""
        return self.turns_needed * STEPS_PER_REV / WIND_SPEED

    @property
    def time_remaining(self):
        return self.turns_remaining * STEPS_PER_REV / WIND_SPEED

    def handle_event(self, ev: Event):
        # safety: works from any state, never blocked by the table
        if ev == Event.EMERGENCY_STOP:
            self.kill()                  # cut power first
            self.status = Status.KILL
            return

        if self.status == Status.KILL:
            return                       # ignore all events in KILL state

        key = (self.status, ev)
        if key not in self.table:
            print(f"Ignored {ev.name} in {self.status.name}")
            return

        new_status, action = self.table[key]
        try:
            if action:                   # action can be None
                action()
        except ArduinoError as e:
            print(f"Hardware error: {e}")
            self.kill()
            self.status = Status.KILL
            return
        except ValueError:
            self.hw.stop_all()           # never leave a half-configured motion running
            raise                        # state is unchanged; the GUI shows the message
        self.status = new_status

    # --- helpers ---
    def _prepare(self, n, direction, speed):
        """Motor must be stopped before DIR; RUN 0 also resets speed to default,
        so speed and accel are always set again after it."""
        self.hw.run(n, False)
        self.hw.direction(n, direction)
        self.hw.accel(n, ACCEL)
        self.hw.speed(n, speed)

    def _spindle_dir(self, reverse):
        if reverse:
            return DIR_LEFT if self.wind_dir == DIR_RIGHT else DIR_RIGHT
        return self.wind_dir

    def _traverse_hw_dir(self, reverse):
        direction = -self.traverse_dir if reverse else self.traverse_dir
        return DIR_RIGHT if direction > 0 else DIR_LEFT

    def _traverse_speed(self, spindle_speed):
        """Traverse speed that advances one wire diameter per spindle turn."""
        speed = round(spindle_speed / STEPS_PER_REV * self.wire_diameter * STEPS_PER_MM)
        return max(MIN_SPEED, speed)

    def _bank_progress(self):
        """Motor 0 must already be stopped. Takes over the turns of the previous run/jog."""
        if self._tracking:
            self.update_progress()
            self._tracking = False
            self._jog_target = None

    def _begin_tracking(self, reverse, jog_steps=None):
        """Motor 0 must already be stopped. Starts counting spindle steps into turns_done."""
        self._bank_progress()
        self._steps_at_start = self.hw.total_steps(MOTOR_0)
        self._turns_at_start = self.turns_done
        self._turns_sign = -1 if reverse else +1
        self._jog_target = jog_steps
        self._tracking = True

    def _jog_spindle_steps(self):
        if self.jog_size <= 0 or self.wire_diameter <= 0:
            raise ValueError("Apply parameters first.")
        turns = self.jog_size / self.wire_diameter
        spindle_steps = round(turns * STEPS_PER_REV)
        if not 0 < spindle_steps <= MAX_STP:
            raise ValueError(f"Jog of {spindle_steps} spindle steps is outside 1..{MAX_STP}")
        return turns, spindle_steps

    # --- actions ---
    def start_init(self):
        print("Starting initialization...")
        self.hw.stop_all()
        # TODO: homing of the traverse 

    def _wind_motion(self, reverse, speed):
        """Start spindle + traverse together (traverse advances one wire diameter per turn).
        reverse=True turns both the opposite way (unwinding)."""
        traverse_hw = self._traverse_hw_dir(reverse)
        self._prepare(MOTOR_0, self._spindle_dir(reverse), speed)
        self._begin_tracking(reverse)
        self._prepare(MOTOR_1, traverse_hw, self._traverse_speed(speed))
        self.hw.run(MOTOR_0, True)
        self.hw.run(MOTOR_1, True)

    def start_winding(self):
        print("Starting winding process...")
        self._wind_motion(reverse=False, speed=WIND_SPEED)
        # TODO: traverse reversal at the coil ends (end switch)
        # TODO: stop when the wire is finished (turns_done >= turns_needed)
        print(f"Expected number of turns: {self.turns_needed:.2f}")

    # semi-manual: spindle + traverse
    def semi_wind_fwd(self):
        print("Semi-manual winding (adding turns)...")
        self._wind_motion(reverse=False, speed=SEMI_WIND_SPEED)

    def semi_wind_back(self):
        print("Semi-manual unwinding (removing turns)...")
        self._wind_motion(reverse=True, speed=SEMI_WIND_SPEED)

    def _semi_jog(self, reverse):
        """Wind (reverse=False) or unwind (reverse=True) jog_size mm along the coil:
        spindle turns jog_size / wire_diameter times, traverse moves jog_size."""
        turns, spindle_steps = self._jog_spindle_steps()
        traverse_steps = round(self.jog_size * STEPS_PER_MM)
        if not 0 < traverse_steps <= MAX_STP:
            raise ValueError(f"Jog of {traverse_steps} traverse steps is outside 1..{MAX_STP}")
        print(f"Semi-manual jog {'unwind' if reverse else 'wind'}: "
              f"{turns:.2f} turns ({self.jog_size} mm)...")

        traverse_hw = self._traverse_hw_dir(reverse)
        self._prepare(MOTOR_0, self._spindle_dir(reverse), SEMI_WIND_SPEED)
        self._begin_tracking(reverse, jog_steps=spindle_steps)
        self._prepare(MOTOR_1, traverse_hw, self._traverse_speed(SEMI_WIND_SPEED))
        self.hw.steps(MOTOR_0, spindle_steps)   # Arduino stops each motor itself
        self.hw.steps(MOTOR_1, traverse_steps)

    def semi_jog_fwd(self):  self._semi_jog(False)
    def semi_jog_back(self): self._semi_jog(True)

    # manual: one motor only
    def _spindle_only(self, reverse):
        print(f"Manual spindle {'back' if reverse else 'forward'} "
              f"at {self.manual_speed_0} steps/s (until released)...")
        self.hw.run(MOTOR_1, False)      # traverse stays where it is
        self._prepare(MOTOR_0, self._spindle_dir(reverse), self.manual_speed_0)
        self._begin_tracking(reverse)
        self.hw.run(MOTOR_0, True)

    def manual_wind_fwd(self):  self._spindle_only(False)
    def manual_wind_back(self): self._spindle_only(True)

    def _manual_jog(self, reverse):
        """Spindle only: the same number of turns as a semi-manual jog, traverse stays."""
        turns, spindle_steps = self._jog_spindle_steps()
        print(f"Manual spindle jog {'back' if reverse else 'forward'}: {turns:.2f} turns...")
        self.hw.run(MOTOR_1, False)
        self._prepare(MOTOR_0, self._spindle_dir(reverse), self.manual_speed_0)
        self._begin_tracking(reverse, jog_steps=spindle_steps)
        self.hw.steps(MOTOR_0, spindle_steps)

    def manual_jog_fwd(self):  self._manual_jog(False)
    def manual_jog_back(self): self._manual_jog(True)

    def _traverse_move(self, direction, name):
        print(f"Manual traverse {name} at {self.manual_speed_1} steps/s (until released)...")
        self.hw.run(MOTOR_0, False)
        self.hw.run(MOTOR_1, False)
        self._bank_progress()
        self._prepare(MOTOR_1, direction, self.manual_speed_1)
        self.hw.run(MOTOR_1, True)

    def traverse_left(self):  self._traverse_move(DIR_LEFT, "left")
    def traverse_right(self): self._traverse_move(DIR_RIGHT, "right")

    def stop_hold(self):
        print("Manual motion stopped.")
        self.hw.stop_all()
        self._bank_progress()            # final value

    # automatic stop
    def begin_decel(self):
        print("Beginning deceleration...")
        # Arduino has no 'decelerate and stop' command: lower the target speed,
        # wait until the speed ramp finishes, then RUN 0.
        for n in (MOTOR_0, MOTOR_1):
            self.hw.speed(n, MIN_SPEED)
        decel_time = WIND_SPEED / ACCEL + 0.5
        self._decel_timer = threading.Timer(decel_time, self._decel_done)
        self._decel_timer.start()

    def _decel_done(self):
        self.hw.stop_all()
        self.handle_event(Event.STOPPED)

    def finish_stop(self):
        self._bank_progress()            # final value
        print("Machine stopped.")

    def kill(self):
        print("Killing machine...")
        if self._decel_timer:
            self._decel_timer.cancel()
        self.hw.stop_all()
        try:
            self._bank_progress()        # keep the count right after an interrupted run/jog
        except Exception:
            pass
        self._tracking = False
        self._jog_target = None


if __name__ == "__main__":
    m = Machine(Arduino("/dev/ttyUSB0", 115200)) 
    m.handle_event(Event.RESET_DONE)
    m.handle_event(Event.INIT_DONE)
    m.set_parameters(coil_diameter=20, coil_length=50, wire_length=1000, wire_diameter=0.5, jog_size=2)
    m.handle_event(Event.SEMI_JOG_FWD)
