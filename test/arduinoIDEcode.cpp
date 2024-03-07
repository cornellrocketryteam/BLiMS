// Purpose: BLiMS Plane Drop Test 1
// Print Altimeter Data to SD card

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
// #include <Adafruit_LSM9DS1.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

#define BNO055_SAMPLERATE_DELAY_MS (100)

// altimeter object
Adafruit_BMP3XX bmp;
// file object
File myFile;
// i2c
// Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

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
void IMUFilePrint()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  myFile.print("IMU(deg):");
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
void IMUSerialPrint()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("IMU(deg):");
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

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Initializing Tests");

  // check SD card
  sdSetUpCheck();
  // check altimeter
  altimeterSetUpCheck();
  // check IMU
  IMUSetUpCheck();

  ////Altimeter stuff
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop()
{
  // if the altimeter is performing
  if (!bmp.performReading())
  {
    Serial.println("Altimeter: fail");
    // return;
  }

  myFile = SD.open("test.txt", FILE_WRITE);
  // Print to file
  myFile.print("{");

  altimeterFilePrint();
  IMUFilePrint();
  // compass issue: something about compassFile/Serial prevents other code in loop from running

  myFile.print("}");
  myFile.println();
  myFile.close();

  // Serial testing
  Serial.print("{");
  altimeterSerialPrint();
  IMUSerialPrint();
  Serial.print("}");
  Serial.println();

  delay(1000);
}
