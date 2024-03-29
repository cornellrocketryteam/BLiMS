// Purpose: BLiMS Plane Drop Test 1
// Print Altimeter Data to SD card

#include <Wire.h>
#include <SPI.h>
#include <SD.h> // SD library
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h> // altimeter library
#include <Adafruit_BNO055.h> // IMU library
#include <AccelStepper.h>    // stepper library

// altimeter definitions
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
// which port the pull switch is connected to
#define PULL_SWITCH A1

#define SEALEVELPRESSURE_HPA (1013.25)
#define BNO055_SAMPLERATE_DELAY_MS (10)

enum break_state_t
{
  BREAK_STATE_NEUTRAL = 0,
  BREAK_STATE_BREAK,
  BREAK_STATE_TURN_LEFT,
  BREAK_STATE_TURN_RIGHT
};

// Stepper objects
AccelStepper stepperRight(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper stepperLeft(AccelStepper::FULL4WIRE, 2, 3, 4, 5);

// Altimeter object
Adafruit_BMP3XX bmp;
// File object
File myFile;
// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
// Starting altitude
float startAlt;
// Braking altitude
float brakeAlt = startAlt + 50;
// pin out - says whether the pin is in or not
bool pinOut = false;

break_state_t state = BREAK_STATE_NEUTRAL;

// check SD connection and file write
bool sdSetUpCheck()
{

  if (!SD.begin(10))
  {
    Serial.println("SD Setup: Fail");
    return false;
  }

  Serial.println("SD Setup: Success");

  myFile = SD.open("test.txt", FILE_WRITE);
  // if file opened
  if (myFile)
  {

    Serial.println("SD Writing to File: Success");
    myFile.println("SD Writing to File: Success");
    // close the file:
    myFile.close();
    return true;
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("SD Writing to File: Fail");
    return false;
  }
}
/// check altimeter connection
bool altimeterSetUpCheck()
{
  if (!bmp.begin_I2C())
  { // hardware I2C mode, can pass in address & alt Wire
    // if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
    // if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Altimeter Setup: Fail");
    return false;
  }
  Serial.println("Altimeter Setup: Success");
  return true;
}
// check IMU connection
bool IMUSetUpCheck()
{
  // Try to initialise and warn if we couldn't detect the chip
  if (!bno.begin())
  {
    Serial.println("IMU Setup: Fail");
    return false;
  }
  Serial.println("IMU Setup: Success");

  return true;
}
// print altimeter data to file
void altimeterFilePrint()
{

  myFile.print("temperature(*C):");
  myFile.print(bmp.temperature);
  myFile.print(";");

  myFile.print("pressure(hPa):");
  myFile.print(bmp.pressure / 100.0);
  myFile.print(";");

  myFile.print("altitude(m):");
  myFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  myFile.print(";");
}
// print altimeter data to serial
void altimeterSerialPrint()
{
  Serial.print("temperature(*C):");
  Serial.print(bmp.temperature);
  Serial.print(";");

  Serial.print("pressure(hPa):");
  Serial.print(bmp.pressure / 100.0);
  Serial.print(";");

  Serial.print("altitude(m):");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.print(";");
}
// print IMU data to file
void IMUFilePrint()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  myFile.print("IMU(deg) ");
  myFile.print("X:");
  myFile.print(euler.x());
  myFile.print(";");
  myFile.print("Y:");
  myFile.print(euler.y());
  myFile.print(";");
  myFile.print("Z:");
  myFile.print(euler.z());
  myFile.print(";");
}
// print IMU to serial monitor
void IMUSerialPrint()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("IMU(deg) ");
  Serial.print("X:");
  Serial.print(euler.x());
  Serial.print(";");
  Serial.print("Y:");
  Serial.print(euler.y());
  Serial.print(";");
  Serial.print("Z:");
  Serial.print(euler.z());
  Serial.print(";");
}
// get avg start altitude based on first 20 readings.
float avgStartAltitude()
{
  float sum = 0;
  // excludes first reading (autmatic #)
  bmp.readAltitude(SEALEVELPRESSURE_HPA);
  for (int i = 0; i < 20; i++)
  {
    sum = sum + bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  return sum / 20;
}

// turn stepper left
void releaseLine(AccelStepper stepper)
{
  // turn left
  stepper.move(-100);

  // Run the stepper motor until it reaches the target position
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
}
// turn stepper right
void pullLine(AccelStepper stepper)
{
  // turn right
  stepper.move(100);

  // Run the stepper motor until it reaches the target position
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
}

// braking function
void pullBrakes()
{

  if (state == BREAK_STATE_NEUTRAL)
  {
    // pull both breaklines
    pullLine(stepperLeft);
    pullLine(stepperRight);
  }
  else if (state == BREAK_STATE_TURN_LEFT)
  {
    // the left breakline is already pulled, so we only need to pull the right
    pullLine(stepperRight);
  }
  else if (state == BREAK_STATE_TURN_RIGHT)
  {
    // the right breakline is already pulled, so we only need to pull the left
    pullLine(stepperLeft);
  }
  else
  {
    // state == BREAK_STATE_BRAKE
    // don't do anything
  }

  state = BREAK_STATE_BREAK;

  printFile();
  myFile.println("End");

  while (1)
  {
    printSerial();
    delay(10);
  }
}

// check to see if we're at braking altitude
// returns "false" for timeout, and "true" for breaking altitude reached.
bool waitForAlt(int waitDuration)
{
  int start = millis();
  while (1)
  {
    printFile();
    printSerial();

    int alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    if (alt <= brakeAlt)
    {
      return true;
    }
    if (millis() - start >= waitDuration)
    {
      return false;
    }
    delay(10); // avoid overpolling sensor
  }
}
// print data to file
void printFile()
{
  myFile = SD.open("test.txt", FILE_WRITE);

  // Print to file
  myFile.print("{");

  myFile.print("time(millis):");
  // record time
  int time = millis();
  myFile.print(time);
  myFile.print(";");

  altimeterFilePrint();
  IMUFilePrint();

  myFile.print("pin:");
  myFile.print(pinOut);
  myFile.print(";");

  myFile.print("state:");
  myFile.print(state);
  myFile.print(";");

  myFile.print("}");
  myFile.println();
  myFile.close();
}
// print data to serial
void printSerial()
{
  // Serial testing
  Serial.print("{");
  Serial.print("time(millis):");
  int time = millis();
  Serial.print(time);
  Serial.print(";");
  altimeterSerialPrint();
  IMUSerialPrint();
  Serial.print("pin:");
  Serial.print(pinOut);
  Serial.print(";");
  Serial.print("state:");
  Serial.print(state);
  Serial.print(";");
  Serial.print("}");
  Serial.println();
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Initializing Tests");

  // pin setup
  pinMode(PULL_SWITCH, INPUT_PULLUP);
  // check SD card
  sdSetUpCheck();
  // check altimeter
  if (altimeterSetUpCheck())
  {
    startAlt = avgStartAltitude();
  }
  else
  {
    // fix
    startAlt = -1;
  }
  // check IMU
  IMUSetUpCheck();

  // Left Motor setup
  //  Set the maximum speed and acceleration
  stepperLeft.setMaxSpeed(1000.0);
  stepperLeft.setAcceleration(500.0); // Set your desired acceleration in steps per second squared
  // Set the initial position to 0 degrees
  stepperLeft.setCurrentPosition(0);

  // Right Motor setup
  stepperRight.setMaxSpeed(1000.0);
  stepperRight.setAcceleration(500.0);
  stepperRight.setCurrentPosition(0);

  ////Altimeter setup
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop()
{

  // if the altimeter is performing
  // fix
  if (!bmp.performReading())
  {
    myFile.println("Altimeter: fail");
    myFile.println(millis());
    Serial.println("Altimeter: fail");
    // return;
  }

  // wait for pin to be pulled
  printFile();
  printSerial();
  while (1)
  {
    // don't want to fill file
    //  printFile();
    printSerial();
    if (digitalRead(PULL_SWITCH) == LOW)
    {
      printFile();
      printSerial();
      pinOut = true;
      break;
    }
    delay(10);
  }

  // once pin is pulled, turn motor, check for braking altitude
  while (1)
  {
    state = BREAK_STATE_TURN_LEFT; // turn left
    // insert motor test
    releaseLine(stepperLeft);
    pullLine(stepperRight);

    if (waitForAlt(5000))
    {
      pullBrakes();
    }
    state = BREAK_STATE_NEUTRAL; // neutral
    // insert motor test
    releaseLine(stepperRight);
    pullLine(stepperLeft);
    if (waitForAlt(10000))
    {
      pullBrakes();
    }
  }
}
