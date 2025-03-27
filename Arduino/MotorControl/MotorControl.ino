#include <Servo.h>

const int STOP_VAL = 1500; // PWM signal value 1100 to 1900, 1500 to stop

const int PIN_0 = 3;
const int PIN_1 = 5;
const int PIN_2 = 6;
const int PIN_3 = 9;
const int PIN_4 = 10;
const int PIN_5 = 11;

const int PINS[] = {PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5};

Servo m0, m1, m2, m3, m4, m5;
Servo motors[] = {m0, m1, m2, m3, m4, m5};

void setup() {
  Serial.begin(9600);

  for(int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    motors[i].attach(PINS[i]);
    motors[i].writeMicroseconds(STOP_VAL);
  }

  delay(9000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  while (Serial.available() == 0);

  String input = Serial.readStringUntil('\n'); //"555 555 555 555 555 555";

  // data will be sent in the following format: 499 499 499 499 499 499 
  // pwm value for a motor from 000 to 999

  int inputIndex = 0;

  int motorPwm = 0;
  int motorMicroSec = 0;
  for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    if (inputIndex + 2 >= input.length()) {
      break;
    }

    motorPwm = input.substring(inputIndex, inputIndex + 3).toInt(); // get the 3 pwm digits
    motorMicroSec = convertToMicoSec(motorPwm);
    motors[i].writeMicroseconds(motorMicroSec); // set pwm frequency
    inputIndex += 4; // shift 4 to get to next pwm entry
  }
}

void printMotor(int motor, int speed) {
  Serial.print("Motor number: ");
  Serial.print(motor);
  Serial.print(", Speed: ");
  Serial.print(speed);
  Serial.println("\n");
}

int convertToMicoSec(int pwm) {
  // 1900 - 1100 = 800
  // 1100 is lowwer bounds, 1900 is upper bounds
  // 999 is original pwm value from Pi
  // x * 800/999 + 1100 = microsec value
  float conversionFactor = 800 / (float) 999; // cast to avoid integer output
  return (pwm * conversionFactor) + 1100;
}

