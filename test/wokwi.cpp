// Stepper motor on Wokwi!

#include <Stepper.h>
#include <AccelStepper.h>
#include <SD.h>
const int stepsPerRevolution = 200; // change this to fit the number of steps per revolution
int stepCount = 0;
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
File myFile;

void turnLeft()
{
  Serial.println("counterclockwise");
  myStepper.step(stepsPerRevolution);
  stepCount = stepCount + stepsPerRevolution;
  delay(500);
}

void turnRight()
{
  // step one revolution in the other direction:
  Serial.println("clockwise");
  myStepper.step(-stepsPerRevolution);
  stepCount = stepCount - stepsPerRevolution;
  delay(500);
}

void run(File *f)
{
  if (stepCount >= 400)
  {
    turnRight();
    myFile.println("right");
  }
  if (stepCount <= 400)
  {
    turnLeft();
    myFile.println("left");
  }
}

void setup()
{
  // set the speed at 60 rpm:
  myStepper.setSpeed(60);
  // initialize the serial port:
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(10, OUTPUT);

  if (!SD.begin(10))
  {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  myFile = SD.open("new.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile)
  {
    run(&myFile);
    Serial.println("Close File");
    myFile.close();
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void loop()
{
}