#include <Stepper.h>

const int stepsPerRevolution = 800;
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

bool motorRunning = false;
int motorDirection = 0;  // 0: Stopped, 1: Clockwise direction, -1: Counterclockwise direction

void setup() {
  Serial.begin(9600);
  myStepper.setSpeed(15); // RPM
  Serial.println("Arduino ready");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == '1' && motorDirection != 1) {
      motorDirection = 1;
      motorRunning = true;
      Serial.println("Starting motor clockwise...");
    } else if (command == '2' && motorDirection != -1) {
      motorDirection = -1;
      motorRunning = true;
      Serial.println("Starting motor counterclockwise...");
    } else if (command == '0') {
      motorDirection = 0;
      motorRunning = false;
      myStepper.step(0);  // Stop the motor
      Serial.println("Stopping motor...");
    }

    if (motorRunning) {
      myStepper.step(motorDirection * 200);
      delay(1000);  // Add a delay to control speed
    }
  }
}
