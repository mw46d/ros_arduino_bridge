/*********************************************************************
 *  ROSArduinoBridge

    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org

    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson

    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

#define SERIAL_STREAM Serial
#define DEBUG_SERIAL_STREAM Serial

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The Pololu DRV8835 dual motor driver shield */
   // #define POLOLU_DRV8835

   /* The A-Star 32U4 Robot Controller LV with Raspberry Pi Bridge */
   #define POLOLU_ASTAR_ROBOT_CONTROLLER
   //#define USE_ENABLE_INTERRUPT

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA

   /* Encoders directly attached to Arduino board */
   // #define ARDUINO_ENC_COUNTER

   /* The RedBot encoder */
   // #define SPARKFUN_REDBOT_ENCODER
#endif

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     115200

/* Maximum PWM signal */
#define MAX_PWM        255

// merose: The A-Star has a PWM range of +/- 400 for the motors. Pololu
// calls them 6V motors, so Marco originally reduced the maximum PWM
// based on using 7.4V batteries. But Ray looked up the motor specs and
// found they are rated to 12V, so we'll use the maximum PWM available.
#define MAX_PWM        400

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

#if defined(SPARKFUN_REDBOT_ENCODER)
#include "RedBot.h"
#endif

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;

  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

#ifdef POLOLU_ASTAR_ROBOT_CONTROLLER
  #include <AStar32U4.h>
  #include <Wire.h>
/*
  #include <AnalogScanner.h>
*/
  #include "I2C.h"

  // Fake pin numbers for A-Star-specific I/O features. These pins will
  // be emulated as digital or analog I/O pins in runCommand(), below.

  // A-Star buttons as digital input pins.
  #define ASTAR_BTN_A_PIN      100
  #define ASTAR_BTN_B_PIN      101
  #define ASTAR_BTN_C_PIN      102

  // A-Star LEDs as digital output pins.
  #define ASTAR_YELLOW_LED_PIN 103
  #define ASTAR_GREEN_LED_PIN  104
  #define ASTAR_RED_LED_PIN    105

  // A-Star Battery voltage as an analog input pin.
  #define ASTAR_BATTERY_PIN    106
  
  // These objects provide access to the A-Star's on-board
  // buttons.
  AStar32U4ButtonA buttonA;
  AStar32U4ButtonB buttonB;
  AStar32U4ButtonC buttonC;
#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    SERIAL_STREAM.println(BAUDRATE);
    break;
  case ANALOG_READ:
    #ifdef POLOLU_ASTAR_ROBOT_CONTROLLER
    if (arg1 == ASTAR_BATTERY_PIN) {
      SERIAL_STREAM.println(readBatteryMillivoltsLV());
      break;
    }
    #endif
    SERIAL_STREAM.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    #ifdef POLOLU_ASTAR_ROBOT_CONTROLLER
    if (arg1 == ASTAR_BTN_A_PIN) {
      SERIAL_STREAM.println(buttonA.isPressed());
      break;
    } else if (arg1 == ASTAR_BTN_B_PIN) {
      SERIAL_STREAM.println(buttonB.isPressed());
      break;
    } else if (arg1 == ASTAR_BTN_C_PIN) {
      SERIAL_STREAM.println(buttonC.isPressed());
      break;
    }
    #endif
    SERIAL_STREAM.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    SERIAL_STREAM.println("OK"); 
    break;
  case DIGITAL_WRITE:
    #ifdef POLOLU_ASTAR_ROBOT_CONTROLLER
    if (arg1 == ASTAR_YELLOW_LED_PIN) {
      ledYellow(arg2);
      SERIAL_STREAM.println("OK"); 
      break;
    } else if (arg1 == ASTAR_GREEN_LED_PIN) {
      ledGreen(arg2);
      SERIAL_STREAM.println("OK"); 
      break;
    } else if (arg1 == ASTAR_RED_LED_PIN) {
      ledRed(arg2);
      SERIAL_STREAM.println("OK"); 
      break;
    }
    #endif
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    SERIAL_STREAM.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    SERIAL_STREAM.println("OK");
    break;
  case PING:
    SERIAL_STREAM.println(Ping(arg1));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    SERIAL_STREAM.println("OK");
    break;
  case SERVO_READ:
    SERIAL_STREAM.println(servos[arg1].getServo().read());
    break;
#endif

#ifdef USE_BASE
  case READ_ENCODERS:
    SERIAL_STREAM.print(readEncoder(LEFT));
    SERIAL_STREAM.print(" ");
    SERIAL_STREAM.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    SERIAL_STREAM.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    SERIAL_STREAM.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    SERIAL_STREAM.println("OK");
    break;
#endif
  default:
    SERIAL_STREAM.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
#ifdef USE_I2C
  initI2c();
#endif

  SERIAL_STREAM.begin(BAUDRATE);
  while (!SERIAL_STREAM) {
    // do nothing
  }

// Initialize the motor controller if used */
#ifdef USE_BASE
  initEncoder();
  initMotorController();
  resetPID();
#endif

/* Attach servos if used */
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (SERIAL_STREAM.available() > 0) {

    // Read the next character
    chr = SERIAL_STREAM.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

#ifdef USE_I2C
  runI2c();
#endif

// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}

