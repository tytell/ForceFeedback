/*
 * Title: Force Muscle Feedback Displacement Calculation
 * Description: This program calculates the change in length of a muscle for any given
 * force input, by using a forward Euler calculation. It is implemented
 * using a timer interrupt.
 * Author: Madeleine Oliver, for Tytell Lab
 */

#include "avdweb_SAMDtimer.h"
#define NFILT 3

// arbitrary value selected for zeta, omega, and M
// delT set to sample time in microseconds, dt in seconds
float zeta = .2;
float omega = 2*M_PI;
int F = 0;
int OUT;
float M = .01;
float delT = 1e3;
float dt = delT/1e6;
float ai1 = 0;
float bi1 = 0;
float aid = 0;
float bid = 0;
float ai2 = 0;
float bi2 = 0;

/**
 * Sets input and output pins, write and read resolution.
 */
void setup() {
  analogWriteResolution(10);
  analogReadResolution(12);
  pinMode(A1, INPUT);
  pinMode(A0, OUTPUT);
}

/**
 * Defines timer interrupt, reads in force, then calculates
 * and outputs displacement.
 */
void ISR_timer4_A1(struct tc_module *const module_inst) {
  static bool j;

  // solves system of differential equations
  F = analogRead(A1);
  aid = bi1;
  bid = F/M - 2*zeta*omega*bi1 - omega*omega*ai1;
  ai2 = aid*dt + ai1;
  bi2 = bid*dt + bi1;

  OUT = ((int)ai2);

  // writes displacement output
  analogWrite(A0,OUT);

  // updates variables to be used on next timer loop
  ai1 = ai2;
  bi1 = bi2;
}

/**
 * Calls timer at period dt
 */
SAMDtimer mytimer2 = SAMDtimer(4, ISR_timer4_A1, delT);

// loop not used because of real-time interrupt
void loop() {
}