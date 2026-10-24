import serial
import threading
import time

class ArduinoError(Exception):
    pass

class Arduino:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # wait for Arduino to reset

        self.ser.reset_input_buffer()
        self.lock = threading.Lock()

    def _readline(self):
        return self.ser.readline().decode(errors="replace").strip()

    def _send(self, command, extra_lines=0):
        with self.lock:
            # Commands go strictly one at a time, so anything already waiting here is a late
            # answer to an earlier command (e.g. one that timed out). Throw it away, otherwise
            # every following command would read the answer meant for the previous one.
            self.ser.reset_input_buffer()
            self.ser.write((command + '\n').encode())
            # The whole answer must come within one timeout. Without this limit a board that keeps
            # printing numbers on its own (e.g. a test sketch printing pin 9) kept this loop
            # skipping them forever and froze the GUI.
            deadline = time.monotonic() + self.ser.timeout
            skipped = []
            while True:
                response = self._readline()
                if response in ("OK", "ERR"):
                    break
                if not response:
                    raise ArduinoError(f"Arduino did not answer {command!r} within "
                                       f"{self.ser.timeout} s")
                if not response.lstrip("+-").isdigit():
                    raise ArduinoError(f"Unexpected response to {command!r} from Arduino: {response!r}")
                skipped.append(response)          # late numeric payload of an earlier query
                if time.monotonic() > deadline:
                    raise ArduinoError(f"No OK/ERR to {command!r}, the Arduino only sends numbers "
                                       f"{skipped[:5]}... by itself. Wrong sketch on the board?")
            if response != "OK":
                raise ArduinoError(f"Arduino rejected {command!r}: {response}")

            lines = [self._readline() for _ in range(extra_lines)]
            if any(not line for line in lines):
                raise ArduinoError(f"Incomplete response to {command!r} from Arduino: {lines!r}")
            return lines  # read extra lines of response

    def ping(self, attempts=3):
        """Check that the sketch answers (call right after connecting).
        Several attempts: some boards need more than 2 s to start after the port is opened."""
        for attempt in range(attempts):
            try:
                self.total_steps(0)
                return
            except ArduinoError as error:
                last = error
        raise ArduinoError(f"No answer from the Arduino on {self.ser.port} ({last}). "
                           "Is the sketch uploaded, the port right and the Serial Monitor closed?")

# commadns to Arduino

    def run(self, n, on):      self._send(f"RUN {n},{1 if on else 0}")
    def direction(self, n, d): self._send(f"DIR {n},{d}")     # motor must be stopped
    def speed(self, n, v):     self._send(f"SPD {n},{int(v)}")
    def accel(self, n, a):     self._send(f"ACC {n},{int(a)}")
    def steps(self, n, s):     self._send(f"STP {n},{int(s)}")  # motor must be stopped; -1 = continuous

 # queries
    def total_steps(self, n):
        return int(self._send(f"TOT {n}", extra_lines=1)[0])
 
    def revolutions(self, n):
        return int(self._send(f"REV {n}", extra_lines=1)[0])

    def end_switch(self):
        """(closed_9, closed_10, hits): end switches on pins 9 and 10 pressed right now, and how
        many times they have stopped the machine since the Arduino was powered up."""
        state_9, state_10, hits = self._send("ESW 1", extra_lines=3)
        return int(state_9) == 1, int(state_10) == 1, int(hits)

 
    def stop_all(self, motors=(0, 1)):
        for n in motors:
            try:
                self.run(n, False)
            except ArduinoError:
                pass
 
    def close(self):
        self.ser.close()



# Test code
if __name__ == "__main__":
    arduino = Arduino(port='/dev/ttyUSB0', baudrate=115200)
    try:
        arduino.ping()
        print("Arduino answers, TOT 0 =", arduino.total_steps(0))
        arduino.run(0, True)
        time.sleep(5)
        arduino.run(0, False)
    except ArduinoError as e:
        print(f"Arduino error: {e}")
    finally:
        arduino.close()
