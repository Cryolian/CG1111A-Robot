#include "MeMCore.h"

//Pin Definitions
#define ULTRASONIC_PIN 12 //12 for port 1, 10 for port 2
#define IR_RECEIVER_PIN A0
#define LDR_PIN A1
#define DECODER_B A2  //connect decoder connection to port 3
#define DECODER_A A3

//Value Definitions
#define SPEED_OF_SOUND 345
#define TURNING_TIME 800 // The time duration (ms) for turning
#define FORWARD_TIME 1800
#define CORRECT_TIME 55
#define LDR_WAIT 10
#define IR_WAIT 15
#define RGB_TIME 200
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

uint8_t motorSpeed = 100; //  best value from experimentation. Faster speeds made stopping at the black lines unreliable.
int status = 0;

//Lab values for Colour and IR
long black_array[] = {920, 700, 800}; //lab readings for black paper
long white_array[] = {963, 940, 965}; //lab readings for white paper
long rgb_array[3] = {0};
long colour_array[3] = {0};
//storing ambient IR reading - distance IR cutoff readings
long ir_array[7] = {50, 150, 289, 340, 370, 384, 400}; //lab readings for drops in voltages (upper boundaries)

/**
   Takes in a given number of readings at
   the LDR pin, and returns the average.
*/
long get_LDR_avg_reading(int times) {
  long total = 0;
  for (int i = 0; i < times; i += 1) {
    total += analogRead(LDR_PIN);
    delay(LDR_WAIT);
  }
  return total / times;
}

/**
   Takes in a given number of readings
   at the IR pin, and returns the average.
*/
long get_IR_avg_reading(int times) {
  long total = 0;
  for (int i = 0; i < times; i += 1) {
    total += analogRead(IR_RECEIVER_PIN);
    delay(IR_WAIT);
  }
  return total / times;
}

/**
   Stops the motors.
*/
void stop_motor() {
  leftMotor.stop();
  rightMotor.stop();
}

/**
   Makes the robot goes forward.
*/
void go_forward() {
  leftMotor.run(-motorSpeed);
  rightMotor.run(motorSpeed);
}

/**
   Moves the robot backwards by a given time.
   Stops the motors afterwards.
*/
void reverse(int time) {
  leftMotor.run(motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(time);
  stop_motor();
}

/**
   Turns the robot right by a given time.
   Stops the motors afterwards.
*/
void right_turn(int time) {
  leftMotor.run(-motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(time);
  stop_motor();
}

/**
   Turns the robot left by a given time.
   Stops the motors afterwards.
*/
void left_turn(int time) {
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
  delay(time);
  stop_motor();
}

/*
   Turns the robot right by a given time,
   moves it forward by a given time,
   and turns it right again.
   Stops the motors afterwards.
*/
void right_turn_2_grid(int turn_time, int fwd_time) {
  right_turn(turn_time);
  go_forward();
  delay(fwd_time);
  stop_motor();
  right_turn(turn_time);
}

/*
   Turns the robot left by a given time,
   moves it forward by a given time,
   and turns it left again.
   Stops the motors afterwards.
*/
void left_turn_2_grid(int turn_time, int fwd_time) {
  left_turn(turn_time);
  go_forward();
  delay(fwd_time);
  stop_motor();
  left_turn(turn_time);
}

/**
   Turns the robot clockwise by twice the given time,
   meant to be a U-turn
   Stops the motors afterwards.
*/
void rotate_back(int time) {
  right_turn(time / 2 + 12);
  reverse(100);
  right_turn(time / 2 + 12);
  right_turn(time + 25);
  stop_motor();
}

/**
   Victory song
   Never Gonna Give You Up - Rick Astley
*/
void endSong() {
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

  int tempo = 114;

  int melody[] = {

    NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_FS5, -8, NOTE_FS5, -8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_A5, 4, NOTE_CS5, 8, NOTE_D5, -8, NOTE_CS5, 16, NOTE_B4, 8, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,

    NOTE_D5, 4, NOTE_E5, 8, NOTE_CS5, -8, NOTE_B4, 16, NOTE_A4, 4, NOTE_A4, 8, //23
    NOTE_E5, 4, NOTE_D5, 2, REST, 4,

  };

  int notes = sizeof(melody) / sizeof(melody[0]) / 2;

  int wholenote = (60000 * 4) / tempo;

  int divider = 0, noteDuration = 0;
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    divider = melody[thisNote + 1];
    if (divider > 0) {
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5;
    }
    buzzer.tone(melody[thisNote], noteDuration);
  }
}

/**
   Takes in a reading from the ultrasonic sensor,
   and returns the calculated distance from the left side.
*/
float distance_left() {
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(ULTRASONIC_PIN, LOW);

  pinMode(ULTRASONIC_PIN, INPUT);
  long duration = pulseIn(ULTRASONIC_PIN, HIGH, 3000);
  float distance = (((float)SPEED_OF_SOUND * (float)duration / 10000.0) / 2.0) - 3.5;
  //  Serial.println(duration);
  //  Serial.print("distance: ");
  //  Serial.println(distance);
  //  Serial.println("cm");
  delay(300);
  return distance;
}

/**
   Function responsible for path correction
   based on the ultrasonic sensor. If the distance
   is too close or too far from the left wall, the correction
   angle is doubled. If no wall is detected, the bot continues straight.
*/
void adjust_angle() {
  float distance = distance_left();
  if (distance > 15.5 || distance < 0) {
    return;
  }

  if (distance < 4) {
    right_turn(2 * CORRECT_TIME);
    return;
  }

  if (distance > 11) {
    left_turn(2 * CORRECT_TIME);
    return;
  }

  //using 7.5cm as the mid point
  if (distance > 8.5) {
    left_turn(CORRECT_TIME);
    return;
  }

  if (distance < 6.5) {
    right_turn(CORRECT_TIME);
    return;
  }
}

/**
   Takes in a voltage reading when the red LED
   is shone, and stores it in colour_array
*/
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

/**
   Takes in a voltage reading when the green LED
   is shone, and stores it in colour_array
*/
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

/**
   Takes in a voltage reading when the blue LED
   is shone, and stores it in colour_array
*/
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

/**
   Identifies the colour given by the RGB values calculated
   in rgb_array. Colours are sorted based on experimental
   values.
*/
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

    if (green > 200) {
      if (blue > 200) {
        Serial.println("white");
        return WHITE;
      }
    }

    if (green > 120) {
      Serial.println("orange");
      return ORANGE;
    }

    Serial.println("red");
    return RED;
  }

  if (blue > 220) {
    Serial.println("blue");
    return LIGHT_BLUE;
  }

  if (blue > 180 && green > 120) {
    Serial.println("purple");
    return PURPLE;
  }

  Serial.println("green");
  return GREEN;

  delay(5000);
  return identify_colour();
}

/**
   Given a colour, execute the movement instruction corresponding
   to the colours. Manual number additions are experimental values
   as one motor was turning less than the other in some turning
   scenarios.
*/
void colour_instruction(int colour) {
  if (colour == RED) {
    left_turn(TURNING_TIME);
    return;
  }

  if (colour == GREEN) {
    right_turn(TURNING_TIME + 55);
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
    endSong();
    return;
  }
}

/**
   Reads in the colour at the bottom of the robot, and executes the
   corresponding instruction given to the colour.
*/
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
    long reading = ((colour_array[i] - black_array[i]) * 255) / (white_array[i] - black_array[i]);
    Serial.println(reading);

    if (reading < 0) {
      reading = 0;
    }
    rgb_array[i] = reading;
    //
    //    Serial.print("array ");
    //    Serial.print(i);
    //    Serial.print(": ");
    //    Serial.println(rgb_array[i]);
  }

  int colour = identify_colour();
  colour_instruction(colour);
}

/*
   Takes the drop in voltage reading at the IR sensor,
   checks against a lookup table for the rough distance
   values, and returns the distance.
*/
int ir_read() {

  stop_motor();
  analogWrite(DECODER_A, 0);
  analogWrite(DECODER_B, 255);
  delay(IR_WAIT);
  int ambient = analogRead(IR_RECEIVER_PIN);
  //  Serial.print("ambient: ");
  //  Serial.println(ambient);
  analogWrite(DECODER_A, LOW);
  analogWrite(DECODER_B, LOW);
  delay(IR_WAIT);
  int reading = analogRead(IR_RECEIVER_PIN);
  //  Serial.print("reading: ");
  //  Serial.println(reading);
  //  Serial.print("difference: ");
  //  Serial.println(ambient - reading);
  int difference = ambient - reading;
  analogWrite(DECODER_A, 0);
  analogWrite(DECODER_B, 255);

  for (int i = 0; i < 7; i += 1) {
    if (difference > ir_array[i]) {
      return i - 1;
    }
  }
  return 8;
}

void setup() {
  delay(2000);
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

  //checks for black line
  int sensor_state = lineFinder.readSensors();

  //S1_*_S2_* did not work in our code,
  //resorted to using the base values of 0 - 3
  if (sensor_state != 3) {
    stop_motor();
    parse_colour();
    return;
  }

  go_forward();
  adjust_angle();
}
