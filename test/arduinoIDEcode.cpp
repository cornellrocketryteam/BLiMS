
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_MMC56x3.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

// altimeter object
Adafruit_BMP3XX bmp;
// file object
File myFile;
// magnetometer object
Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Initializing Tests");

  // check SD card
  sdSetUpCheck();
  // check altimeter
  altimeterSetUpCheck();
  // check compass
  compassSetUpCheck();

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
  // compassFilePrint();
  // compass issue: something about compassFile/Serial prevents other code in loop from running

  myFile.print("}");
  myFile.println();
  myFile.close();

  // Serial testing
  Serial.print("{");
  altimeterSerialPrint();
  // compassSerialPrint();
  Serial.print("}");
  Serial.println();

  delay(3000);
}

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
bool compassSetUpCheck()
{
  if (!mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire))
  { // I2C mode
    /* There was a problem detecting the MMC5603 ... check your connections */
    Serial.println("Compass Setup: Fail");
    return false;
  }
  Serial.println("Compass Setup: Success");
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
void compassFilePrint()
{
  sensors_event_t event;
  mag.getEvent(&event);

  float Pi = 3.14159;

  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  myFile.print("heading:");
  myFile.print(heading);
}
void compassSerialPrint()
{
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  float Pi = 3.14159;

  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  Serial.print("heading:");
  Serial.print(heading);
}
