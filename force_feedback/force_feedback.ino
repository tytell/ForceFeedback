/*
 * Title: Force Muscle Feedback Displacement Calculation
 * Description: This program calculates the change in length of a muscle for any given
 * force input, by using a forward Euler calculation. It is implemented
 * using a timer interrupt.
 * Author: Madeleine Oliver, for Tytell Lab
 */

#include <Arduino.h>
#include "avdweb_SAMDtimer.h"

const float maxvolts = 3.3;

float voltscale = maxvolts / 2.8;
float outscale = 1.0;

char command_prompt[] = "Command:"
" c = calibrate"
" r = run"
" s = stop";

#define MODE_WAIT       0
#define MODE_CALIBRATE  1
#define MODE_RUN        2

int mode = MODE_WAIT;

int dt_us = 1000;      // microseconds
float dt = dt_us / 1e6;

float zeta = 0.2;
float omega = 2*M_PI;
float M = 1.0;

float pos = 0;
float vel = 0;

/**
 * Sets input and output pins, write and read resolution.
 */
void setup() {
  analogWriteResolution(10);
  analogReadResolution(12);
  
  pinMode(A1, INPUT);
  pinMode(A0, OUTPUT);

  SerialUSB.begin(9600);
  while (!SerialUSB);       // wait for serial to start
  SerialUSB.println("Start");

  SerialUSB.println(command_prompt);
}

/**
 * Defines timer interrupt, reads in force, then calculates
 * and outputs displacement.
 */
void ISR_timer3_loop(struct tc_module *const module_inst) 
{
  int Fint;
  int posint;
  float F;
  float dpos, dvel;
  
  // solves system of differential equations
  Fint = analogRead(A1);
  F = (((float)Fint) / 4095.0) * maxvolts;
  
  dpos = vel;
  dvel = F/M - 2*zeta*omega*vel - omega*omega*pos;

  vel += dvel * dt;
  pos += dpos * dt;

  posint = (int)((pos/outscale*voltscale)*1023);

  // writes displacement output
  analogWrite(A0,posint);
}

SAMDtimer timer = SAMDtimer(4, ISR_timer3_loop, dt_us); // every 1000 us

void loop() {
  char input;
  
  if (SerialUSB.available()) {
    input = SerialUSB.read();

    switch (input) {
      case 'c':
      case 'C':
        SerialUSB.println("Calibrate!");
        mode = MODE_CALIBRATE;
        calibrate();
        mode = MODE_WAIT;
        break;

      case 'r':
      case 'R':
        SerialUSB.println("Run!");
        timer.attachInterrupt(ISR_timer3_loop);
        timer.enableInterrupt(1);
        break;

      case 's':
      case 'S':
        SerialUSB.println("Stop!");
        timer.enableInterrupt(0);
        break;

      default:
        SerialUSB.println("Unrecognized command");
    }
    SerialUSB.println(command_prompt);
  }
  delay(200);
}

void calibrate() {
  String voltstr;
  float volts;
  
  SerialUSB.println("Outputting maximum voltage");
  analogWrite(A0, 1023);

  SerialUSB.print("What is the voltage?");

  SerialUSB.setTimeout(30000);      // 30 sec
  voltstr = SerialUSB.readStringUntil('\n');
  SerialUSB.setTimeout(1000);
  
  volts = voltstr.toFloat();
  if (volts == 0) {
    SerialUSB.println("Invalid voltage");
  }
  else {
    voltscale = 3.3 / volts;
    SerialUSB.print("New voltage scale = ");
    SerialUSB.println(voltscale);
  }

  analogWrite(A0, 0);
}
