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
#define TURNING_TIME 700 // The time duration (ms) for turning
#define FORWARD_TIME 500
#define LDR_WAIT 10
#define IR_WAIT 15
#define CORRECT_TIMES 2
#define RGB_TIME 200
#define BASELINE_AMBIENT_IR 950
#define RED 1
#define GREEN 2
#define ORANGE 3
#define PURPLE 4
#define LIGHT_BLUE 5
#define WHITE 6

MeBuzzer buzzer; // create the buzzer object
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeRGBLed led(0, 30); // Based on hardware connections on mCore; cannot change
MeLineFollower lineFinder(PORT_2);

// Setting motor speed to an integer between 1 and 255
// The larger the number, the faster the speed
uint8_t motorSpeed = 255 / 2;
int status = 0;

long colour_array[3] = {0};

//find lab values
long black_array[] = {959, 800, 911};
long white_array[] = {984, 964, 989};
long diff_array[] = {25, 164, 78};
long rgb_array[3] = {0};

//credit to Dr Henry for this function
long get_LDR_avg_reading(int times) {
  long total = 0;
  for (int i = 0; i < times; i += 1) {
    total += analogRead(LDR_PIN);
    delay(LDR_WAIT);
  }
  return total / times;
}

long get_IR_avg_reading(int times) {
  long total = 0;
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
      left_turn(25);
      stop_motor();
      distance_2 = distance_left();
    }
  }

  else if (distance_2 < distance_1) {

    while ((distance_1 - distance_2) > difference) {
      right_turn(25);
      stop_motor();
      distance_2 = distance_left();
    }
  }
}

void correct_path() {
  if (distance_left() < 10) {

    for (int i = 0; i < CORRECT_TIMES; i += 1) {

      adjust_angle(0.00);
      reverse(200);
      delay(1000);
    }
  }
}

void shine_red() {
  analogWrite(DECODER_A, 255);
  analogWrite(DECODER_B, 255);
  delay(RGB_TIME);
  colour_array[0] = get_LDR_avg_reading(5);
  analogWrite(DECODER_A, LOW);
  analogWrite(DECODER_B, LOW);
  Serial.print("Red: ");
  Serial.println(colour_array[0]);
  delay(LDR_WAIT);
}

void shine_green() {
  analogWrite(DECODER_A, 255);
  analogWrite(DECODER_B, 0);
  delay(RGB_TIME);
  colour_array[1] = get_LDR_avg_reading(5);
  analogWrite(DECODER_A, LOW);
  analogWrite(DECODER_B, LOW);
  Serial.print("Green: ");
  Serial.println(colour_array[1]);
  delay(LDR_WAIT);
}

void shine_blue() {
  analogWrite(DECODER_A, 0);
  analogWrite(DECODER_B, 255);
  delay(RGB_TIME);
  colour_array[2] = get_LDR_avg_reading(5);
  analogWrite(DECODER_A, LOW);
  analogWrite(DECODER_B, LOW);
  Serial.print("Blue: ");
  Serial.println(colour_array[2]);
  delay(LDR_WAIT);
}

//1 Red
//2 Green
//3 Orange
//4 Purple
//5 Light Blue
//6 White
int identify_colour() {
  int red = rgb_array[0];
  int green = rgb_array[1];
  int blue = rgb_array[2];

  if (red > 199) {

    if (green > 240) {
      if (blue > 240) {
        Serial.println("white");
        return WHITE;
      }
    }

    if (green < 125) {
      Serial.println("red");
      return RED;
    }

    if (green > 125) {
      Serial.println("orange");
      return ORANGE;
    }
  }

  if (blue > 230) {
    Serial.println("blue");
    return LIGHT_BLUE;
  }

  if (red < 70) {
    Serial.println("green");
    return GREEN;
  }

  if (red > 70) {
    Serial.println("purple");
    return PURPLE;
  }
  delay(5000);

  return identify_colour();
}

void colour_instruction(int colour) {
  if (colour == RED) {
    left_turn(TURNING_TIME);
    return;
  }

  if (colour == GREEN) {
    right_turn(TURNING_TIME);
    return;
  }

  if (colour == ORANGE) {
    rotate_back(TURNING_TIME);
    return;
  }

  if (colour == PURPLE) {
    left_turn_2_grid(TURNING_TIME, FORWARD_TIME);
    return;
  }

  if (colour == LIGHT_BLUE) {
    right_turn_2_grid(TURNING_TIME, FORWARD_TIME);
    return;
  }

  if (colour == WHITE) {
    //do something
  }
}

//1 Red
//2 Green
//3 Orange
//4 Purple
//5 Light Blue
//6 White
void parse_colour() {
  shine_red();
  shine_green();
  shine_blue();

  for (int i = 0; i < 3; i += 1) {
    long reading = ((colour_array[i] - black_array[i]) * 255) / diff_array[i];

    if (reading < 0) {
      reading = 0;
    }
    rgb_array[i] = reading;

    Serial.print("array ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(rgb_array[i]);
  }

  int colour = identify_colour();
  delay(500);
  //  colour_instruction(colour);
}

int ir_read() {

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

void travel() {
  int sensor_state = lineFinder.readSensors();
  Serial.println(sensor_state);

  if (sensor_state != 3) {
    stop_motor();
    delay(1000);
    //parse_colour();
    delay(1000);
    //go_forward();
    delay(1000);
    stop_motor();
  }

  else {
    go_forward();

  }
}

void setup() {
  delay(2000); // Do nothing for 10000 ms = 10 seconds
  Serial.begin(9600);
  pinMode(DECODER_A, OUTPUT);
  pinMode(DECODER_B, OUTPUT);
  pinMode(IR_RECEIVER_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  analogWrite(DECODER_A, 0);
  analogWrite(DECODER_B, 0);
  led.setpin(13);
}

void loop() {
  travel();

}
