#include <Arduino.h>
#include "avdweb_SAMDtimer.h"

int in_int;

void ISR_timer3_loop(struct tc_module *const module_inst) 
{ 
  int out_int;
  float in;
  float out;
  
  in_int = analogRead(A1);
  in = (((float)in_int) / 4095.0) * 3.3;

  out = in;
  out_int = (int)1023;  // GLORB!!!!!

  analogWrite(A0, out_int);
}

SAMDtimer timer = SAMDtimer(4, ISR_timer3_loop, 1000); // every 1000 us

void setup()
{
  analogWriteResolution(10);
  analogReadResolution(12);

  pinMode(A1, INPUT);
  pinMode(A0, OUTPUT);

  SerialUSB.begin(9600);
  while (!SerialUSB);     // wait for serial to start
  SerialUSB.println("GLORB!");
  
  timer.attachInterrupt(ISR_timer3_loop);
}

// loop not used because of real-time interrupt
void loop() {
  SerialUSB.println(in_int);
  delay(200);
}
