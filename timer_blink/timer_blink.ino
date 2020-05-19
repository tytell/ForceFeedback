/*
 * Title: Force Muscle Feedback Displacement Calculation
 * Description: This program calculates the change in length of a muscle for any given
 * force input, by using a forward Euler calculation. It is implemented
 * using a timer interrupt.
 * Author: Madeleine Oliver, for Tytell Lab
 */
#include <Arduino.h>
#include "avdweb_SAMDtimer.h"

const int BLUE_LED = 13; // Blue "stat" LED on pin 13
const int RX_LED = PIN_LED_RXL; // RX LED on pin 25, we use the predefined PIN_LED_RXL to make sure

void ISR_timer3_LED1(struct tc_module *const module_inst) 
{ 
  static bool b;
  digitalWrite(BLUE_LED, b);
  digitalWrite(RX_LED, b);
  b = !b;
}

SAMDtimer timer4_2Hz = SAMDtimer(4, ISR_timer3_LED1, 1e5); // ISR LED2 1Hz (0.5s on, 0.5s off)

void setup()
{
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RX_LED, OUTPUT);
  timer4_2Hz.attachInterrupt(ISR_timer3_LED1);
}

// loop not used because of real-time interrupt
void loop() {
}
