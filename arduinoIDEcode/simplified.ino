#include <Wire.h>
#include <SPI.h>
#include <SD.h> // SD library
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h> // altimeter library
#include <Adafruit_BNO055.h> // IMU library
#include <AccelStepper.h>    // stepper library

// which port the pull switch is connected to
#define PULL_SWITCH A1

#define STEPPER_POSITION_BRAKE -100
#define STEPPER_POSITION_NEUTRAL 0
#define STEPPER_POSITION_RELEASE 100

enum break_state_t
{
  BREAK_STATE_NEUTRAL = 0,
  BREAK_STATE_BREAK,
  BREAK_STATE_TURN_LEFT,
  BREAK_STATE_TURN_RIGHT
};

// Stepper objects
AccelStepper stepperRight(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper stepperLeft(AccelStepper::FULL4WIRE, 6, 7, 8, 9);

// pin out - says whether the pin is in or not
bool pinOut = false;

break_state_t state = BREAK_STATE_NEUTRAL;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Initializing Tests");

  // pin setup
  pinMode(PULL_SWITCH, INPUT_PULLUP);

  // Left Motor setup
  //  Set the maximum speed and acceleration
  stepperLeft.setMaxSpeed(1000.0);
  stepperLeft.setAcceleration(200.0); // Set your desired acceleration in steps per second squared
  // Set the initial position to 0 degrees
  stepperLeft.setCurrentPosition(0);

  // Right Motor setup
  stepperRight.setMaxSpeed(1000.0);
  stepperRight.setAcceleration(200.0);
  stepperRight.setCurrentPosition(0);
}

void loop()
{
  while (1)
  {
    if (digitalRead(PULL_SWITCH) == LOW)
    {
      pinOut = true;
      break;
    }
  }
  while (1)
  {
    // turn left
    stepperLeft.moveTo(1000);
    stepperRight.moveTo(-1000);
    // turn right
    stepperLeft.moveTo(-1000);
    stepperRight.moveTo(1000);
  }
}