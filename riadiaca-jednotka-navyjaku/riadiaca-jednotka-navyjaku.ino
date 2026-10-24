/*
What follows is the basis for a code that runs the stepper motors based on timers. Explanations are provided.

How a timer works:

A timer needs a reference "clock frequency" and a circuit that can store an integer numerical value: "a counter". 
The clock frequency is usually referenced to the onboard clock controlling the processor.
A frequency scaling (division) factor is usually defined, it can usually be any power of 2 ranging from 1 to 1024. On Arduino Uno, choose from 1, 8, 32, 64, 128, 256, 512, 1024.
For the purposes of the timer, the onboard clock frequency (Arduino: 16 MHz, stabilized by a crystal oscillator) is divided by this factor.

Then, at every cycle of this scaled frequency (a "tick"), the counter of the timer increases by 1 (starting at zero), until it reaches its maximum value,
at which point it overflows and resets to zero.

The timer allows generation of special events called "Interrupts" that can be triggered in two usual ways. 
Either at the counter overflow at the maximum value: "overflow interrupt", or when the counter reaches a particular pre-set value: "compare interrupt".

There is also an option to reset the timer at any moment or preconfigure it to reset automatically when a "compare interrupt" is triggered.

This can be used for precision timing control with better than microsecond resolution.
(...and most importantly, unburdened by any operating system with complicated timing mechanisms, such as Windows or Linux)

The idea behind using timers and interrupts:

Arduino Uno has three timers available: TIMER0, TIMER1, TIMER2.
For us, TIMER2 is best, because it does not control other system functions and counts up to 256 only (which takes about 4 microseconds with no frequency division),
thus being reasonably matched to our speed requirements.

We will use TIMER2 to generate "compare interrupts" that will call an interrupt handler routine with timing defined by the compare value (0-255). 
Additionally, frequency scaling can be used as well for slower timing.

The behaviour of timers is controlled by values of so called timer control registers. These need to be configured, which is usually done using bitwise operations.
*/

// timing constants
const unsigned long interval_micros = 100;                                                    // trigger interval, microseconds (value estimated manually)
const unsigned long interrupts_per_second = 1000000/interval_micros;                          // conversion of the above to interrupts per second
const unsigned long interrupts_per_adjust = 1000;                                             // determines number of interrupts after which velocity is adjusted, set to 1000
const unsigned int adjust_freq = interrupts_per_second/interrupts_per_adjust;                 // frequency of velocity adjustments, should be reasonably low, around 10 Hz
const unsigned int clock_freq = 16;                                                           // Arduino clock frequency, MHz
const unsigned int freq_scaling_factor = 32;                                                  // frequency division factor for TIMER2, choose from 1, 8, 32, 64, 128, 256, 512, 1024
const unsigned int compareValue = interval_micros * clock_freq / freq_scaling_factor - 1;     // final comparison register value for use with TIMER2 (value must be < 256)                                    <-----

// timing variables
volatile unsigned long interrupt_count = 0;                                                   // counts interrupt occurrences
volatile unsigned long test_period;                                                           // auxilliary variable used to evaluate motor stepping period
volatile unsigned int adjust_counter = 0;                                                     // counts interrupts up to interrupts_per_adjust (replaces a slow 32-bit modulo in the ISR)

//motor constants
const unsigned int steps_per_revolution = 200;               // motor property: number of steps per full revolution
const unsigned int motor_speed_default = 100;                // default speed, set to 100 steps/sec ( 0.5 Hz on the motors with 200 steps/rev )
const unsigned int motor_min_speed = 1;                      // minimum speed setting, set to 1 stes/sec
const unsigned int motor_max_speed = 1000;                   // maximum speed setting, this was tested relatively safe up to 1000 steps/sec
const unsigned int motor_min_accel = adjust_freq;            // minimum useful acceleration, steps/sec squared, lower value might fail to change velocity
const unsigned int motor_max_accel = 400;                    // maximum allowed acceleration, steps/sec squared, estimated
const unsigned int motor_accel_default = motor_min_accel;    // default acceleration
const int N_motors = 2;                                      // total number of motors

// motor variables
volatile bool motor_direction[N_motors];                     // directions of stepper motors (volatile: the end switch interrupt changes it)
unsigned int motor_accel[N_motors];                          // representation of motor acceleration, steps/second squared 
volatile long motor_steps[N_motors];                         // motor steps counter
volatile bool motor_status[N_motors];                        // sets if motors should run
volatile bool motor_run_nonstop[N_motors];                   // sets continuous rotation (true) vs pre-determined number of steps, "jog mode" (false), acessed in interrupt handler
volatile unsigned long motor_steps_to_go[N_motors];          // pre-determined number of steps to move motor in "jog mode",m updated in interrupt handler
volatile unsigned int motor_period[N_motors];                // representation of current motor period: how many interrupts per single motor step
volatile unsigned int motor_speed_current[N_motors];         // representation of motor current velocity, steps/second, updated in interrupt handler 
volatile unsigned int motor_speed_target[N_motors];          // representation of motor target velocity, steps/second
volatile unsigned int motor_skips_left[N_motors];            // how many interrupts until motors move by 1 step
volatile unsigned int speed_diff;                            // used in interrupt handling to store the difference between current and target speed
volatile unsigned int speed_adjust;                          // used in interrupt handling to store speed adjustment value

// Arduino pin definitions
const int step_pin[N_motors] = {2, 3};
const int dir_pin[N_motors]  = {5, 6};
const int enable_pin[N_motors] = {8, 8};

/*
End switches (pins 9 and 10), handled by a hardware interrupt:
Pin 9 is PB1 and pin 10 is PB2 on the Arduino Uno. They have no "external" interrupt (those are
only on pins 2 and 3, used here for STEP), but they have "pin change interrupts" (PCINT1, PCINT2),
both in the group PCINT0_vect. That interrupt fires on EVERY change of either pin, so the handler
checks which pin is LOW (= switch closed).

Wiring: each switch between its pin and GND, internal pull-up -> open = HIGH, closed = LOW.
One switch sits at each end of the traverse path.

When a switch closes while end_motor (traverse) runs:
  1. the other motor (spindle) stops immediately,
  2. end_motor is set to the back-off direction of THAT switch (pin 9 and pin 10 back off
     in opposite directions, always away from their own switch),
  3. it waits end_backoff_pause_ms (the motor comes to a stop before reversing),
  4. it makes end_backoff_steps steps back at end_backoff_speed,
  5. it stops by itself (the existing jog mode in the timer ISR does that).
Contact bounce: after the first closing, further pin changes are ignored until
the PC starts end_motor again (RUN / STP command), see end_backoff_active.
*/
const int end_switch_pin_9 = 9;                     // end switch input (PB1 / PCINT1)
const int end_switch_pin_10 = 10;                   // end switch input (PB2 / PCINT2)
const bool end_backoff_dir_9 = 0;                   // DIR of the traverse when backing off from switch 9  TODO: swap if it backs INTO the switch
const bool end_backoff_dir_10 = !end_backoff_dir_9; // switch 10 is at the other end: always the opposite direction
const int end_motor = 1;                            // motor backed off by the end switches (traverse)
const int other_motor = 0;                          // motor only stopped by the end switches (spindle)
const unsigned int end_backoff_steps = 80;          // steps back after hitting the switch (80 steps = 1 mm at 80 steps/mm)  TODO: tune
const unsigned int end_backoff_speed = 100;         // steps/s while backing off
const unsigned int end_backoff_pause_ms = 50;       // pause before backing off, ms
volatile bool end_backoff_active = false;           // true from the switch hit until the PC starts end_motor again
volatile unsigned int end_hits = 0;                 // how many times the switch stopped end_motor (read by "ESW")

// command parsing variables
String data, cmd, num, value = "";            // Strings used to parse commands sent over Serial
int n, param;                                 // ints used to parse commands sent over Serial (motor number and parameter value)


// helper routine to initialize motor-related variables
void init_motors() {
  for (int i=0; i<N_motors; i++) {
    motor_steps[i] = 0;
    motor_period[i] = 1;
    motor_direction[i] = 0;
    motor_status[i] = 0;
    motor_skips_left[i] = 0;
    motor_steps_to_go[i] = -1;
    motor_run_nonstop[i] = true;

    motor_speed_current[i] = 0;
    motor_speed_target[i] = motor_speed_default;
    motor_accel[i] = motor_accel_default;

  }
}

/*
Note on bitwise operations:

The registers that control timers store settings values in individual bits, but here, we are using data types like byte (8-bits). 
Therefore, to change a single setting, a special operation is needed.
If we wish to start from all registers set to 0, the best way to flip individual bits is to use a bitwise OR operation, denoted by the symbol "|". 
This keeps all other bits in the same register exactly as they were.

Example: 
We start with an 8-bit register called REG in the state: "01010000".
Now we want to set the third least significant bit to 1 to change some setting.
To do this, we use "REG = REG | 00000100;", or alternatively just "REG |= 00000100;" which combines the assignment with the bitwise or into 1 operation. 
This produces the state of REG given as 01010100.

As programmers in C++, we cannot just directly write "00000000" or "00000100" in our code, we need to use byte values.
One could use powers of 2: "0" for 00000001, "1" for 00000010, "2" for 00000010, etc... but this is cumbersome.

It is easier to start from the value "1" (00000001) and use so-called bitwise shift operations, which simply shift the binary representation by a given number of positions 
to place the only 1 in the binary represantation correctly.

We use the left bitwise shift, denoted by "<<" like this: result = value << positions.
So then "1 << 4" gives 00010000 in binary, or 16 in decimal representation. This is how we prepare the settings for the registers that control the timers.

If more than one bit needs to be set at the same time, we combine the above expressions using bitwise or "|".

Arduino also has names for individual setting bits. For example, the bits labelled CS20, CS21, CS22 are used to control the frequency scaling.

These things are usually found in tutorials or in spec sheets of the microcontroller.
*/



// timer and interrupt configuration
void config_timers_interrupts() {
  
  noInterrupts();           // Disable all interrupts during setup
  
  TCCR2A = 0;               // Reset Timer2 Control Register A
  TCCR2B = 0;               // Reset Timer2 Control Register B
  TCNT2  = 0;               // Clear the timer counter register
  
  // check and enter the Compare value
  if (compareValue < 256) {
    if (compareValue > 1) {
      OCR2A = uint8_t(compareValue); // Load the target comparison value
    }
    else {
      OCR2A = 1; // use minimum if underrange
      Serial.println("Compare value underrange, using 1. Change scaling."); 
    }     
  }
  else {
    OCR2A = 255; // use maximum if overrange
    Serial.println("Compare value overrange, using 255. Change scaling.");
  }

  TCCR2A |= (1 << WGM21);   // Turn on CTC mode (Clear Timer on Compare, or simply timer reset when "compare interrupt" is generated)
  
  switch(freq_scaling_factor) {
    case 1: TCCR2B |= (1 << CS20); break;                                   // Set prescaler to 1 (No prescaling)
    case 8: TCCR2B |= (1 << CS21); break;                                   // Set prescaler to 8
    case 32: TCCR2B |= (1 << CS21) | (1 << CS20); break;                    // Set prescaler to 32
    case 64: TCCR2B |= (1 << CS22); break;                                  // Set prescaler to 64
    case 128: TCCR2B |= (1 << CS22) | (1 << CS20); break;                   // Set prescaler to 128
    case 256: TCCR2B |= (1 << CS22) | (1 << CS21); break;                   // Set prescaler to 256
    case 1024: TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); break;    // Set prescaler to 1024
  }

  TIMSK2 |= (1 << OCIE2A);  // Enable Timer2 Compare Match A Interrupt
  
  interrupts();             // Re-enable all interrupts
}


// mathematical sign function
int sign(int val) {
  if (val>0) {return 1;}
  else if (val<0) {return -1;}
  else return 0;
}


// Motor i is about to start from standstill: begin its speed ramp right away.
// Without this it stepped with the period left over from its previous run (after power-up:
// period 1 = one step every 2 interrupts = 5000 steps/s) until the next speed adjustment, up to 0.1 s.
// Call only while motor i is stopped and with interrupts off.
void begin_ramp(int i) {
  unsigned int v = motor_accel[i] / adjust_freq;          // first step of the ramp
  if (v > motor_speed_target[i]) v = motor_speed_target[i];
  if (v < motor_min_speed) v = motor_min_speed;
  motor_speed_current[i] = v;
  motor_period[i] = 1000000UL / (interval_micros * v);
  motor_skips_left[i] = 0;
}


// pin change interrupt configuration for the end switches on pins 9 and 10
void config_end_switch_interrupt() {
  pinMode(end_switch_pin_9, INPUT_PULLUP);
  pinMode(end_switch_pin_10, INPUT_PULLUP);

  noInterrupts();
  PCMSK0 |= (1 << PCINT1) | (1 << PCINT2);  // only pins 9 (PB1) and 10 (PB2) may trigger the PCINT0 group
  PCIFR  |= (1 << PCIF0);   // clear a change that may have been recorded before enabling
  PCICR  |= (1 << PCIE0);   // enable pin change interrupts for the PCINT0 group (pins 8-13)
  interrupts();
}


// End switch interrupt handler: runs on every change of pin 9 or pin 10.
// AVR interrupts do not interrupt each other, so the timer ISR cannot step the motors
// in the middle of this routine.
ISR(PCINT0_vect) {
  uint8_t pins = PINB;
  bool backoff_dir;
  if (!(pins & (1 << PINB1))) {
    backoff_dir = end_backoff_dir_9;     // pin 9 LOW = switch 9 closed
  }
  else if (!(pins & (1 << PINB2))) {
    backoff_dir = end_backoff_dir_10;    // pin 10 LOW = switch 10 closed
  }
  else {
    return;                              // both HIGH = a switch opened, nothing to do
  }
  if (!motor_status[end_motor]) {
    return;                              // traverse is not running (e.g. switch pressed by hand)
  }
  if (end_backoff_active) {
    return;                              // already backing off: contact bounce / switch released
  }
  end_backoff_active = true;
  end_hits++;

  // 1. stop the other motor (spindle) at once, the same way as "RUN 0,0"
  motor_status[other_motor] = false;
  motor_run_nonstop[other_motor] = true;
  motor_speed_current[other_motor] = 0;
  motor_speed_target[other_motor] = motor_speed_default;

  // 2. back-off direction of the switch that closed
  motor_direction[end_motor] = backoff_dir;
  digitalWrite(dir_pin[end_motor], backoff_dir);

  // 3. + 4. jog back by end_backoff_steps at a fixed slow speed (current = target: no ramp)
  motor_speed_current[end_motor] = end_backoff_speed;
  motor_speed_target[end_motor]  = end_backoff_speed;
  motor_period[end_motor] = 1000000UL / (interval_micros * end_backoff_speed);
  motor_skips_left[end_motor] = end_backoff_pause_ms * 1000UL / interval_micros;  // pause before the first step back
  motor_steps_to_go[end_motor] = end_backoff_steps;
  motor_run_nonstop[end_motor] = false;
  motor_status[end_motor] = true;
  // 5. the timer ISR stops the motor after the last step (jog mode)
}


//setup, to be run once at startup
void setup() {
  Serial.begin(115200); // initialize Serial for high speed communication
  while(!Serial) {
    delay(100);         // wait for Serial initialization
  }

  for (int i = 0; i < N_motors; i++) {
    pinMode(step_pin[i], OUTPUT);
    pinMode(dir_pin[i], OUTPUT);
    pinMode(enable_pin[i], OUTPUT);
    digitalWrite(enable_pin[i], LOW);
    
  }

  // some data printing over Serial for debugging purposes
  
  // debugging off
  /*
  Serial.print("Interval (microseconds): ");
  Serial.println(interval_micros);
  Serial.print("Frequency scaling: ");
  Serial.println(freq_scaling_factor);
  Serial.print("Compare value: ");
  Serial.println(compareValue);
  */
  
  config_timers_interrupts();

  init_motors();

  config_end_switch_interrupt();   // after init_motors(): the handler uses the motor variables
}


// Below is the Compare interrupt handler routine: "Interrupt service request"
// Triggers exactly when TCNT2 matches OCR2A. Hardware then resets TCNT2 to 0 automatically.
ISR(TIMER2_COMPA_vect) {
  /* No need to manually reset or reload registers here!
  Note that all variables used here should be declared using the keyword "volatile". 
  This is absolutely necessary if the ISR is REWRITING the variable values, a little bit less if it just reads them.
  */

interrupt_count++;

// Decide ONCE per interrupt whether speeds are adjusted now. "interrupt_count % interrupts_per_adjust"
// is a 32-bit division (~40 us on AVR) and was computed for every running motor in every interrupt:
// with both motors running the ISR used up almost the whole 100 us interval, loop() got almost
// no time and answers to serial commands came too late (timeouts in Python).
bool do_adjust = false;
if (++adjust_counter >= interrupts_per_adjust) {
  adjust_counter = 0;
  do_adjust = true;
}

for (int i = 0; i < N_motors; i++) {
  
  // ignore stopped motors
  if (!motor_status[i]) {
    continue;
  }
  
  // check if velocity should be adjusted (and do so)
  if (do_adjust) {
    speed_diff = motor_speed_target[i] - motor_speed_current[i];
    if (speed_diff != 0) {
      speed_adjust = motor_accel[i] / adjust_freq;
      if (abs((int)motor_speed_current[i] - (int)motor_speed_target[i]) > speed_adjust) {
        motor_speed_current[i] += speed_adjust * sign((int)motor_speed_target[i] - (int)motor_speed_current[i]);
      }
      else {
        motor_speed_current[i] = motor_speed_target[i];
      }
        
      // only period is adjusted, not the OCR value
      test_period = 1000000 / (interval_micros * motor_speed_current[i]);
      if (test_period > 32767) {
          test_period = 32767;
      }
      else if (test_period < 1) {
        test_period = 1;
      }
      motor_period[i] = int(test_period);
    }
  }

  // handle motor stepping if applicable now
  if (motor_skips_left[i] > 0) {
    motor_skips_left[i] -= 1;
  }
  else {
    digitalWrite(step_pin[i], HIGH);
    delayMicroseconds(10);               //ZPOMALENI PREPINANI SIGNALU NA PINU, KVULI KONECNE SLEW RATE NA PINECH ARDUINA A V OBVODU, NELZE PRILIS SNIZIT POD 10 us
    digitalWrite(step_pin[i], LOW);

    if (motor_direction[i]) {
      motor_steps[i] += 1;
    }

    else {
      motor_steps[i] -= 1;
    }

    // handle special case of predetermined number of steps to make (jog mode)
    if (!motor_run_nonstop[i]) {
      motor_steps_to_go[i] -= 1;
      if (motor_steps_to_go[i] <= 0) {
        motor_status[i] = false;
        motor_speed_current[i] = 0;
        motor_speed_target[i] = motor_speed_default;  // reset speed whenever motor stops
      }
    }

    motor_skips_left[i] = motor_period[i];
  }
}

  /*
  Note on ISRs:

  The ISR needs to run fast enough to finish before another interrupt is generated, otherwise the program can become blocked completely.
  Do not put anything too demanding here and simplify as much as possible. 
  With an interval of 200 microseconds, we have certainly less than 200*16 = + processor ticks available here.
  Thus it really pays of to use only boolean or 8-bit values here, and to perform only addition, subtraction, and multiplication.
  Hardware functions, like setting outputs high or low, take some time too. 
  Using digitalWrite() on a standard Arduino takes 34 to 52 ticks, but there are some tricks to make it much faster, if needed. Can be found online.
  */

}


/*
In the main loop below we want to parse serial input and perform motor settings. For this we define the following commands:
RUN n,value (toggle motor on/off)
DIR n,value (toggle motor direction)
SPD n,value (set motor speed, steps/sec)
ACC n,value (set motor acceleration, steps/sec squared)
STP n,value (set fixed number of steps to move, using the direction, speed and acceleration provided)

Here, n is the index of the motor to be configured, starting from 0. Value is the setting required and is an integer. 
The value supplied for SPD represents steps per second, ACC uses steps/sec squared.
The value supplied to STP denotes the number of steps to move before stopping the motor, the maximum is 32000 (i.e., plenty). 
STP n,-1 returns to continuous mode.
The values supplied for RUN and DIR can be only 0 or 1.

The given motor must be stopped (e.g., by "RUN n,0") before using DIR or STP!

End switches on pins 9 and 10: when one closes while motor 1 (end_motor) runs, motor 0 stops at once
and motor 1 backs off by end_backoff_steps away from that switch and stops by itself,
see ISR(PCINT0_vect). Nothing is sent over Serial when that happens.

Examples: 
To switch motor 0 on, send: RUN 0,1
To switch motor 0 off, send: RUN 0,0
To set motor 1 speed to 20 steps/second, send: SPD 1,20
etc...

To check parsing and syntax, either "OK" or "ERR" is returned via serial port.

Additionally, three queries are implemented: 
TOT n
REV n
ESW n
TOT/REV return the total number of steps (TOT n) or revolutions (REV n) made in the positive direction with motor n.
ESW returns three lines: the state of switch 9, the state of switch 10 (1 = closed/pressed, 0 = open)
and how many times the end switches have stopped the machine since power-up (n is ignored).
*/


// main loop, to be run continuously after setup
void loop() {
  if (Serial.available() > 0) {
    // Read the incoming string until a newline character is found
    data = Serial.readStringUntil('\n');

  // Trim accidental whitespaces or carriage returns (\r)
    data.trim(); 
    // the command abbreviation is 3 letters long, use this to parse the command and parameters
    cmd = data.substring(0,3);
    num = data.substring(4,5);
    num.trim();   
    n = int(num.toInt()); 

    
    // first handle queries, as they do not contain a param value
    if ((cmd == "TOT") || (cmd == "tot")) {
      long s;
      noInterrupts();                // 32-bit value changed by the ISR: copy it in one piece
      s = motor_steps[n];
      interrupts();
      Serial.println("OK");
      Serial.println(s);
    }
    else if ((cmd == "ESW") || (cmd == "esw")) {
      unsigned int hits;
      noInterrupts();                // 16-bit value changed by the end switch interrupt
      hits = end_hits;
      interrupts();
      Serial.println("OK");
      Serial.println(digitalRead(end_switch_pin_9) == LOW ? 1 : 0);    // 1 = switch 9 closed (pressed)
      Serial.println(digitalRead(end_switch_pin_10) == LOW ? 1 : 0);   // 1 = switch 10 closed (pressed)
      Serial.println(hits);
    }
    else if ((cmd == "REV") || (cmd == "rev")) {
      long s;
      noInterrupts();
      s = motor_steps[n];
      interrupts();
      Serial.println("OK");
      Serial.println(s/steps_per_revolution);
    }
    // now handle the remaining commands
    else {
      value = data.substring(6);   
      value.trim();        
      param = int(value.toInt());

      if ((cmd == "SPD") || (cmd == "spd")) {
        // change speed, param is motor steps per second
        if ((param < motor_max_speed) && (param >= motor_min_speed)) {
          motor_speed_target[n] = param; 
          Serial.println("OK");      }
        else {
          Serial.println("ERR");
        }
      }
      else if ((cmd == "ACC") || (cmd == "acc")) {
        // change acceleration, param is motor steps per second squared
        if ((param < motor_max_accel) && (param >= motor_min_accel)) {
          motor_accel[n] = param;  
          Serial.println("OK");                   
        }
        else {
          Serial.println("ERR");
        }
      }
      else if ((cmd == "RUN") || (cmd == "run")) {
        // exit jog mode (just in case), then turn motor on or off as needed
        noInterrupts();                  // the end switch interrupt must not run between these writes
        motor_run_nonstop[n] = true;
        if (param > 0 && !motor_status[n]) {
          begin_ramp(n);                 // starting from standstill
        }
        motor_status[n] = (param>0);
        if (param <= 0) {
          motor_speed_current[n] = 0;
          motor_speed_target[n] = motor_speed_default;
        }
        else if (n == end_motor) {
          end_backoff_active = false;    // new motion from the PC: the end switch may stop it again
        }
        interrupts();
        Serial.println("OK");
      }
      else if ((cmd == "DIR") || (cmd == "dir")) {
        // if motor is stopped, allow changing direction
        if (!motor_status[n]) {   
          motor_direction[n] = (param>0);
          digitalWrite(dir_pin[n], param);
          Serial.println("OK");
        }
        else {
          Serial.println("ERR");
        }
      }
      else if ((cmd == "STP") || (cmd == "stp")) {
        // if motor is stopped, allow setting fixed number of steps to move (jog mode)...
        if (!motor_status[n]) {
          if ((param>0) && (param <= 32000)) {
            noInterrupts();
            begin_ramp(n);
            motor_steps_to_go[n] = param;
            motor_run_nonstop[n] = false;
            motor_status[n] = true;
            if (n == end_motor) {
              end_backoff_active = false;  // new motion from the PC: the end switch may stop it again
            }
            interrupts();
            Serial.println("OK");
          }
          // ...or reset to continuous mode
          else if (param == -1) {
            motor_steps_to_go[n] = -1;
            motor_speed_current[n] = 0;
            motor_speed_target[n] = motor_speed_default; // reset speed whenever motor is stopped
            motor_run_nonstop[n] = true;
            Serial.println("OK");          
          }
          else {
            Serial.println("ERR");
          }
        }
        else {
          Serial.println("ERR");
        }  

      }
      else {
        Serial.println("ERR");
      }
    }

// debugging off
/*
    Serial.println("0:");
    Serial.print("speed current: ");
    Serial.println(motor_speed_current[0]);
    Serial.print("speed target: ");
    Serial.println(motor_speed_target[0]);
    Serial.print("accel: ");
    Serial.println(motor_accel[0]);
    Serial.print("period: ");
    Serial.println(motor_period[0]);
    Serial.println("1:");
    Serial.print("speed current: ");
    Serial.println(motor_speed_current[1]);
    Serial.print("speed target: ");
    Serial.println(motor_speed_target[1]);
    Serial.print("accel: ");
    Serial.println(motor_accel[1]);
    Serial.print("period: ");
    Serial.println(motor_period[1]);
    Serial.print("test period: ");
    Serial.println(test_period);
*/
  }


}
