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
float outoffset = 0.0;

char command_prompt[] = "Command:\n"
" c = calibrate\n"
" w = set resonant frequency\n"
" z = set damping coefficient\n"
" m = set mass\n"
" o = set output scale\n"
" d = set output offset\n"
" r = run\n"
" s = stop\n";

#define MODE_WAIT       0
#define MODE_CALIBRATE  1
#define MODE_RUN        2

#define ERR_LOW         1
#define ERR_HIGH        2

int mode = MODE_WAIT;
int err = 0;

int dt_us = 1000;      // microseconds
float dt = dt_us / 1e6;

float zeta = 0.2;
float omega = 2*M_PI;
float M = 0.5;

float pos = 0;
float scaledpos = 0;
float vel = 0;

int Fint;
int posint;
float F;
float dpos, dvel;

/**
 * Sets input and output pins, write and read resolution.
 */
void setup() {
  analogWriteResolution(10);
  analogReadResolution(12);
  
  pinMode(A1, INPUT);
  pinMode(A0, OUTPUT);
  pinMode(2, OUTPUT);

  SerialUSB.begin(9600);
  while (!SerialUSB);       // wait for serial to start
  SerialUSB.println("Start");

  SerialUSB.println(command_prompt);

  digitalWrite(2, LOW);
}

/**
 * Defines timer interrupt, reads in force, then calculates
 * and outputs displacement.
 */
void ISR_timer3_loop(struct tc_module *const module_inst) 
{
  digitalWrite(2, HIGH);
  
  // solves system of differential equations
  Fint = analogRead(A1);
  F = (((float)Fint) / 4095.0) * maxvolts;

  // pos = F;
  
  dpos = vel;
  dvel = F/M - 2*zeta*omega*vel - omega*omega*pos;

  vel += dvel * dt;
  pos += dpos * dt;

  scaledpos = pos*outscale*voltscale + outoffset;
  
  if (scaledpos < 0.0) {
    err = ERR_LOW;
    scaledpos = 0.0;
  }
  else if (scaledpos > 1.0) {
    err = ERR_HIGH;
    scaledpos = 1.0;
  }
  
  posint = (int)(scaledpos*1023);

  // writes displacement output
  analogWrite(A0,posint);
  
  digitalWrite(2, LOW);
}

SAMDtimer timer = SAMDtimer(4, ISR_timer3_loop, dt_us); // every 1000 us

void loop() {
  char input;
  float freq1, zeta1, mass1, os1;
  
  if (err != 0) {
    switch (err) {
      case ERR_LOW:
        SerialUSB.println("Error: Output voltage too low");
        break;
      case ERR_HIGH:
        SerialUSB.println("Error: Output voltage too high");
        break;
    }
    err = 0;
  }
  
  if (SerialUSB.available()) {
    input = SerialUSB.read();

    switch (input) {
      case 'c':
      case 'C':
        SerialUSB.println("Calibrate!");
        mode = MODE_CALIBRATE;
        timer.enableInterrupt(0);
        calibrate();
        mode = MODE_WAIT;
        break;

      case 'w':
      case 'W':
        freq1 = read_usb_float("What is the resonant frequency (Hz)? ", omega / (2*M_PI));
        if (freq1 > 0) {
          omega = 2*M_PI * freq1;
        }
        break;
                
      case 'z':
      case 'Z':
        zeta1 = read_usb_float("What is the damping coefficient? ", zeta);
        if (zeta1 > 0) {
          zeta = zeta1;
        } 
        break;

      case 'm':
      case 'M':
        mass1 = read_usb_float("What is the mass? ", M);
        if (mass1 > 0) {
          M = mass1;
        } 
        break;

      case 'o':
      case 'O':
        os1 = read_usb_float("What is the output scale? ", outscale);
        if (os1 > 0) {
          outscale = os1;
        } 
        break;

      case 'd':
      case 'D':
        os1 = read_usb_float("What is the output offset? ", outoffset);
        if (os1 > 0) {
          outoffset = os1;
        } 
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

float read_usb_float(String prompt, float current) {
  String str;
  
  SerialUSB.print(prompt + " (current = " + String(current) + ")");

  SerialUSB.setTimeout(30000);      // 30 sec
  str = SerialUSB.readStringUntil('\n');
  for (int i = 0; (i < 5) && (str.length() == 0); i++) {
    str = SerialUSB.readStringUntil('\n');
  }
  SerialUSB.setTimeout(1000);
  SerialUSB.print("DEBUG: ");
  SerialUSB.println(str);
  
  return str.toFloat();
}

void calibrate() {
  String voltstr;
  float volts;
  
  SerialUSB.println("Outputting maximum voltage");
  analogWrite(A0, 1023);

  volts = read_usb_float("What is the voltage? ", 0);
  if (volts == 0) {
    SerialUSB.println("Invalid voltage");
  }
  else {
    voltscale = 3.3 / volts;
    SerialUSB.print("New voltage scale = ");
    SerialUSB.println(voltscale);
  }

  // analogWrite(A0, 0);
}

