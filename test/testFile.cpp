// Overshoot.pde
// -*- mode: C++ -*-
//
// Check overshoot handling
// which sets a new target position and then waits until the stepper has
// achieved it. This is used for testing the handling of overshoots
//
// Copyright (C) 2009 Mike McCauley
// $Id: Overshoot.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h> //https://www.airspayce.com/mikem/arduino/AccelStepper/
#include <MultiStepper.h>
// Motor has 200 steps/revolution -> 100 steps = 180deg
// Wokwi simulator: https://wokwi.com/projects/327324886912467538

// Define a stepper and the pins it will use
AccelStepper stepperL(AccelStepper::FULL4WIRE, 2, 3, 4, 5); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

AccelStepper stepperR(AccelStepper::FULL4WIRE, 6, 7, 8, 9); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 6,7,8,9

const int maxSpeed = 1000;
const int motorSpeed = 200;
const int turnRot = 33;

// want to turn a certain amount rather than endless
void turnLeft(AccelStepper *stepper) // passing a pointer to the stepper in memory
{
  stepper->setSpeed(motorSpeed); // with pointers, use -> instead of .
  //(*stepper).setSpeed(200); same line as the one above
}

void turnRight(AccelStepper *stepper)
{
  stepper->setSpeed(motorSpeed);
  // stepper->moveTo(-turnRot);
}

// goes back to start
void release(AccelStepper stepper)
{
}

// if turning endless, need a stop functino

////////////

void setup()
{

  stepperL.setMaxSpeed(maxSpeed);
  turnLeft(&stepperL); // reference stepperL and pass it in (pass the address in memory)

  stepperR.setMaxSpeed(maxSpeed);
  turnRight(&stepperR);
}

void loop()
{
  stepperR.runSpeed();
  stepperL.runSpeed();
}

// void setup()
// {

//   stepperL.setMaxSpeed(1000);     // Set maximum speed value for the stepper
//   stepperL.setAcceleration(500);  // Set acceleration value for the stepper
//   stepperL.setCurrentPosition(0); // Set the current position to 0 steps

//   stepperR.setMaxSpeed(1000);
//   stepperR.setAcceleration(500);
//   stepperR.setCurrentPosition(0);
// }

// void loop()
// {

//   stepperL.moveTo(1600);    // Set desired move: 1600 steps (in quater-step resolution that's one rotation)
//   stepperL.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position

//   // stepperR.moveTo(1600);
//   // stepperR.runToPosition();

//   // Move back to position 0, using run() which is non-blocking - both motors will move at the same time
//   stepperL.moveTo(0);
//   // stepperR.moveTo(0);
//   // while (stepperL.currentPosition() != 0 || stepperR.currentPosition() != 0)
//   while (stepperL.currentPosition() != 0)
//   {
//     stepperL.stepForward(); // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
//     // stepperR.run();
//     //
//     //
//   }
// }