// Create an IntervalTimer object 
IntervalTimer myTimer;

const int ledPin = LED_BUILTIN;  // the pin with a LED

void setup() {
  pinMode(A21, OUTPUT);
  analogWriteResolution(10);
  Serial.begin(9600);
  myTimer.begin(pulse, 100);  // 1000 us
}

// The interrupt will blink the LED, and keep
// track of how many times it has blinked.
int onState = 0;
int onVal = 0;
int onDir = 1;
volatile unsigned long blinkCount = 0; // use volatile for shared variables

// functions called by IntervalTimer should be short, run as quickly as
// possible, and should avoid calling other functions if possible.
void pulse() {
  if (onVal == 1023) {
    onDir = -1;
  } else if (onVal == 0) {
    onDir = 1;
  }
  if (onState == 0) {
    analogWrite(A21, onVal);
    onVal = onVal + onDir;
    onState = 1;
  }
  else {
    analogWrite(A21, 0);
    onState = 0;
  }
}

// The main program will print the blink count
// to the Arduino Serial Monitor
void loop() {
  delay(100);
}
