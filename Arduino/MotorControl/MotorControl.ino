#include <Servo.h>

const int STOP_VAL = 1500; // PWM signal value 1100 to 1900, 1500 to stop

const int PIN_0 = 0;
const int PIN_1 = 1;
const int PIN_2 = 2;
const int PIN_3 = 3;
const int PIN_4 = 4;
const int PIN_5 = 5;

const int PINS[] = {PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5};

Servo m0, m1, m2, m3, m4, m5;
Servo motors[] = {m0, m1, m2, m3, m4, m5};


void setup() {
  // Serial.begin(9600);

  for(int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    motors[i].attach(PINS[i]);
    motors[i].writeMicroseconds(STOP_VAL);
  }

  delay(9000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  if(Serial.available()) {
    String input = Serial.readStringUntil('\n');

    // data will be sent in the following format: 000 000 000 000 000 000 
    // pwm value for a motor from 000 to 255

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
      printMotor(i, motorMicroSec);
      inputIndex += 4; // shift 4 to get to next pwm entry
    }
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
  // 255 is original pwm value from Pi
  // x * 800/255 + 1100 = microsec value
  float conversionFactor = 800 / (float) 255; // cast to avoid integer output
  return (pwm * conversionFactor) + 1100;
}

