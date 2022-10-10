#include "MeMCore.h"
#include <MeMCore.h>

#define TURNING_TIME_MS 330 // The time duration (ms) for turning

MeBuzzer buzzer; // create the buzzer object
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeRGBLed led(0,30); // Based on hardware connections on mCore; cannot change

uint8_t motorSpeed = 255;
// Setting motor speed to an integer between 1 and 255
// The larger the number, the faster the speed

void celebrate() {
  // Each of the following "function calls" plays a single tone.
  // The numbers in the bracket specify the frequency and the duration (ms)
  buzzer.tone(392, 200);
  buzzer.tone(523, 200);
  buzzer.tone(659, 200);
  buzzer.tone(784, 200);
  buzzer.tone(659, 150);
  buzzer.tone(784, 400);
  buzzer.noTone();
}

void light_show() {
  led.setColor(255, 255, 255); // set both LEDs to white colour
  led.show(); // Must use .show() to make new colour take effect
  delay(500);
  led.setColorAt(0, 255, 0, 0); // set Right LED to Red
  led.setColorAt(1, 0, 0, 255); // set Left LED to Blue
  led.show();
  delay(500);
  led.setColorAt(0, 0, 0, 255); // set Right LED to Blue
  led.setColorAt(1, 255, 0, 0); // set Left LED to Red
  led.show();
  delay(500);
}

void setup() {
  // Any setup code here runs only once:
  delay(1000); // Do nothing for 10000 ms = 10 seconds
  celebrate();
  led.setpin(13);
}

void loop() {
  // Going forward:
  leftMotor.run(-motorSpeed); // Negative: wheel turns anti-clockwise
  rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
  delay(1000); // Keep going straight for 1000 ms
  
  leftMotor.stop(); // Stop left motor
  rightMotor.stop(); // Stop right motor
  delay(1000); // Stop for 1000 ms
  
  // Turning left (on the spot):
  leftMotor.run(motorSpeed); // Positive: wheel turns clockwise
  rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
  delay(TURNING_TIME_MS); // Keep turning left for this time duration
  
  leftMotor.stop(); // Stop left motor
  rightMotor.stop(); // Stop right motor
  delay(1000); // Stop for 1000 ms
  
  light_show();
}
