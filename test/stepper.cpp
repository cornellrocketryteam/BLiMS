// Overshoot.pde
// -*- mode: C++ -*-
//
// Check overshoot handling
// which sets a new target position and then waits until the stepper has
// achieved it. This is used for testing the handling of overshoots
//
// Copyright (C) 2009 Mike McCauley
// $Id: Overshoot.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void setup()
{
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(1000.0);    // Set your desired maximum speed in steps per second
  stepper.setAcceleration(500.0); // Set your desired acceleration in steps per second squared

  // Set the initial position to 0 degrees
  stepper.setCurrentPosition(0);
}

void moveMotor30DegreesRight()
{
  // Move the motor 30 degrees to the right
  stepper.move(100);

  // Run the stepper motor until it reaches the target position
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
}

void loop()
{
  // Call the function to move the motor 30 degrees right
  moveMotor30DegreesRight();

  // Wait for a moment (you can adjust the delay as needed)
  delay(1000);
}