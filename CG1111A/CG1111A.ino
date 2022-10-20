#include "MeMCore.h"
#include <MeMCore.h>

//Pin Definitions
#define ULTRASONIC_PIN 12 //12 for port 1, 10 for port 2
#define IR_RECEIVER_PIN A0
#define LDR_PIN A1
#define DECODER_B A2  //connect decoder connection to port 3
#define DECODER_A A3

//Value Definitions
#define SPEED_OF_SOUND 345
#define TURNING_TIME_MS 330 // The time duration (ms) for turning
#define LDR_WAIT 10
#define IR_WAIT 10
#define CORRECT_TIMES 2
#define RGB_TIME 200
#define BASELINE_AMBIENT_IR 950


MeBuzzer buzzer; // create the buzzer object
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeRGBLed led(0, 30); // Based on hardware connections on mCore; cannot change
MeLineFollower lineFinder(PORT_2);


uint8_t motorSpeed = 255 / 2;
// Setting motor speed to an integer between 1 and 255
// The larger the number, the faster the speed

int status = 0;
int red = 0;
int green = 0;
int blue = 0;

int colourArray[] = {0, 0, 0};

//credit to Dr Henry for this function
int get_LDR_avg_reading(int times) {
  int total = 0;
  for (int i = 0; i < times; i += 1) {
    total += analogRead(LDR_PIN);
    delay(LDR_WAIT);
  }
  return total / times;
}

int get_IR_avg_reading(int times) {
  int total = 0;
  for (int i = 0; i < times; i += 1) {
    total += analogRead(IR_RECEIVER_PIN);
    delay(IR_WAIT);
  }
  return total / times;
}

void stop_motor() {
  leftMotor.stop();
  rightMotor.stop();
}

void go_forward() {
  leftMotor.run(-motorSpeed);
  rightMotor.run(motorSpeed);
}

void reverse(int time) {
  leftMotor.run(motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(time);
  stop_motor();
}

void right_turn(int time) {
  leftMotor.run(-motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(time);
  stop_motor();
}

void left_turn(int time) {
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
  delay(time);
  stop_motor();
}

void right_turn_2_grid(int turn_time, int fwd_time) {
  right_turn(turn_time);
  stop_motor();
  go_forward();
  delay(fwd_time);
  stop_motor();
  right_turn(turn_time);
  stop_motor();
}

void left_turn_2_grid(int turn_time, int fwd_time) {
  left_turn(turn_time);
  go_forward();
  delay(fwd_time);
  stop_motor();
  left_turn(turn_time);
}

void rotate_back(int time) {
  right_turn(time);
  right_turn(time);
  stop_motor();
}

void calibrate_speed_of_sound() {
  float speed;
  Serial.println("Place object 10 cm away from sensor");

  delay(2000);
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(ULTRASONIC_PIN, LOW);

  pinMode(ULTRASONIC_PIN, INPUT);
  long duration = pulseIn(ULTRASONIC_PIN, HIGH);
  Serial.println(duration);

  speed = ((10.0 * 2 * 10000.0) / ((float)duration ));
  Serial.println(speed);
}

float distance_left() {
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(ULTRASONIC_PIN, LOW);

  pinMode(ULTRASONIC_PIN, INPUT);
  long duration = pulseIn(ULTRASONIC_PIN, HIGH);
  float distance = (((float)SPEED_OF_SOUND * (float)duration / 10000.0) / 2.0) - 3.5;
  //  Serial.println(duration);
  //  Serial.print("distance: ");
  //  Serial.println(distance);
  //  Serial.println("cm");
  delay(300);
  return distance;
}

void adjust_angle(float difference) {
  float distance_1 = distance_left();
  go_forward();
  delay(250);
  stop_motor();
  float distance_2 = distance_left();

  if (distance_2 > distance_1) {

    while ((distance_2 - distance_1) > difference) {
      left_turn(35);
      stop_motor();
      distance_2 = distance_left();
    }
  }

  else if (distance_2 < distance_1) {

    while ((distance_1 - distance_2) > difference) {
      right_turn(35);
      stop_motor();
      distance_2 = distance_left();
    }
  }
}

void correct_path() {
  for (int i = 0; i < CORRECT_TIMES; i += 1) {
    adjust_angle(0.00);
    reverse(200);
    delay(1000);
  }
}

void shine_red(int time) {
  analogWrite(DECODER_A, 255);
  analogWrite(DECODER_B, 255);
  delay(time);
  int reading = get_LDR_avg_reading(5);
  analogWrite(DECODER_A, LOW);
  analogWrite(DECODER_B, LOW);
  Serial.print("Red: ");
  Serial.println(reading);
  delay(LDR_WAIT);
}

void shine_green(int time) {
  analogWrite(DECODER_A, 255);
  analogWrite(DECODER_B, 0);
  delay(time);
  int reading = get_LDR_avg_reading(5);
  analogWrite(DECODER_A, LOW);
  analogWrite(DECODER_B, LOW);
  Serial.print("Green: ");
  Serial.println(reading);
  delay(LDR_WAIT);
}

void shine_blue(int time) {
  analogWrite(DECODER_A, 0);
  analogWrite(DECODER_B, 255);
  delay(time);
  int reading = get_LDR_avg_reading(5);
  analogWrite(DECODER_A, LOW);
  analogWrite(DECODER_B, LOW);
  Serial.print("Blue: ");
  Serial.println(reading);
  delay(LDR_WAIT);
}

void ir_read() {

  stop_motor();
  analogWrite(DECODER_A, 0);
  analogWrite(DECODER_B, 255);

  delay(IR_WAIT);

  int ambient = analogRead(IR_RECEIVER_PIN);
  Serial.print("ambient: ");
  Serial.println(ambient);

  analogWrite(DECODER_A, LOW);
  analogWrite(DECODER_B, LOW);

  delay(IR_WAIT);

  int reading = analogRead(IR_RECEIVER_PIN);
  Serial.print("reading: ");
  Serial.println(reading);

  Serial.print("difference: ");
  Serial.println(ambient - reading);

  analogWrite(DECODER_A, 0);
  analogWrite(DECODER_B, 255);


}

void setup() {

  delay(2000); // Do nothing for 10000 ms = 10 seconds
  Serial.begin(9600);
  //  correct_path();
  pinMode(DECODER_A, OUTPUT);
  pinMode(DECODER_B, OUTPUT);
  pinMode(IR_RECEIVER_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  analogWrite(DECODER_A, 0);
  analogWrite(DECODER_B, 0);

}

void loop() {

  ir_read();
  delay(500);

  //                 int sensorState = lineFinder.readSensors();
  //                 Serial.print("Sensor State: ");
  //                 Serial.println(sensorState);
  //
  //  if (sensorState == 3) {
  //  stop_motor();
  //
  //    shine_red(RGB_TIME);
  //    shine_green(RGB_TIME);
  //    shine_blue(RGB_TIME);
  //    delay(500);
  //  }
  //
  //  else {
  //    go_forward();
  //  }

}
