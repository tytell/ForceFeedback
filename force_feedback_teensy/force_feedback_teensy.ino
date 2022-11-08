/*
 * Title: Force Muscle Feedback Displacement Calculation
 * Description: This program calculates the change in length of a muscle for any given
 * force input, by using a forward Euler calculation. It is implemented
 * using a timer interrupt.
 * Author: Madeleine Oliver, for Tytell Lab
 */

#include <Arduino.h>

// Create an IntervalTimer object 
IntervalTimer myTimer;

const int AOchan = A21;

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
  pinMode(AOchan, OUTPUT);
  pinMode(2, OUTPUT);

  Serial.begin(9600);
  while (!Serial);       // wait for serial to start
  Serial.println("Start");

  show_params();
  Serial.println(command_prompt);

  digitalWrite(2, LOW);
}

/**
 * Defines timer interrupt, reads in force, then calculates
 * and outputs displacement.
 */
void feedback_loop()
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
  analogWrite(AOchan,posint);
  
  digitalWrite(2, LOW);
}

void loop() {
  char input;
  float freq1, zeta1, mass1, os1;
  
  if (err != 0) {
    switch (err) {
      case ERR_LOW:
        Serial.println("Error: Output voltage too low");
        break;
      case ERR_HIGH:
        Serial.println("Error: Output voltage too high");
        break;
    }
    err = 0;
  }
  
  if (Serial.available()) {
    input = Serial.read();

    switch (input) {
      case 'c':
      case 'C':
        Serial.println("Calibrate!");
        mode = MODE_CALIBRATE;
        myTimer.end();
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
        outscale = os1;
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
        Serial.println("Run!");
        show_params();
        
        myTimer.begin(feedback_loop, dt_us);
        break;

      case 's':
      case 'S':
        Serial.println("Stop!");
        myTimer.end();
        break;

      default:
        break;
        // Serial.println("Unrecognized command");
    }
    Serial.println(command_prompt);
  }
  delay(200);
}

float read_usb_float(String prompt, float current) {
  String str;
  
  Serial.print(prompt + " (current = " + String(current) + ")");

  Serial.setTimeout(30000);      // 30 sec
  str = Serial.readStringUntil('\n');
  for (int i = 0; (i < 5) && (str.length() == 0); i++) {
    str = Serial.readStringUntil('\n');
  }
  Serial.setTimeout(1000);
  Serial.print("DEBUG: ");
  Serial.println(str);
  
  return str.toFloat();
}

void calibrate() {
  String voltstr;
  float volts;
  
  Serial.println("Outputting maximum voltage");
  analogWrite(AOchan, 1023);

  volts = read_usb_float("What is the voltage? ", 0);
  if (volts == 0) {
    Serial.println("Invalid voltage");
  }
  else {
    voltscale = 3.3 / volts;
    Serial.print("New voltage scale = ");
    Serial.println(voltscale);
  }

  analogWrite(AOchan, 0);
}

void show_params() {
  Serial.print("Mass: ");
  Serial.println(M);

  Serial.print("zeta: ");
  Serial.println(zeta);
  
  Serial.print("omega: ");
  Serial.println(omega / (2*M_PI));
  
  Serial.print("Out offset: ");
  Serial.println(outoffset);
  
  Serial.print("Out scale: ");
  Serial.println(outscale);
}
