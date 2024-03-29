// Purpose: BLiMS Plane Drop Test 1
// Print Altimeter Data to SD card

#include <AccelStepper.h>    // stepper library
#include <Adafruit_BMP3XX.h> // altimeter library
#include <Adafruit_Sensor.h>
#include <SD.h>  // SD library: Allows you to read and write
#include <SPI.h> //SD library: Allows you to connect with the arduino
#include <Wire.h>

// altimeter definitions
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

// which port the pull switch is connected to
#define PULL_SWITCH A1

#define SEALEVELPRESSURE_HPA (1013.25)

#define STEPPER_POSITION_BRAKE -20000
#define STEPPER_POSITION_NEUTRAL 0
#define STEPPER_POSITION_RELEASE 20000

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

// Altimeter object
Adafruit_BMP3XX bmp;
// File object
File myFile;
// Starting altitude
float startAlt;
// Braking altitude
float brakeAlt;
// pin out - says whether the pin is in or not
bool pinOut = false;

break_state_t state = BREAK_STATE_NEUTRAL;

// check SD connection and file write
bool sdSetUpCheck()
{

    if (!SD.begin(10))
    {

        Serial.println("SDno1");
        myFile.println("SDno1");
        return false;
    }

    myFile = SD.open("test.txt", FILE_WRITE);
    // if file opened
    if (myFile)
    {
        // close the file:
        myFile.close();
        return true;
    }
    else
    {
        // if the file didn't open, print an error:
        Serial.println("SDno2");
        myFile.println("SDno2");
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
        // bmp.begin_I2C( BMP3XX_DEFAULT_ADDRESS, &Wire);
        Serial.println("Altno");
        myFile.println("Altno");
        return false;
    }
    return true;
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

// moveBothSteppers will move the left stepper to leftPosition and the right stepper to
// rightPosition. If shouldInterruptForAltitude is true, then we stop the movement if the
// altitude is less than the braking altitude. Otherwise, we keep moving the steppers until
// movement is completed no matter what. The function returns true if we stopped because we
// hit the braking altitude, and false if we stopped because we hit the target position.
bool moveBothSteppers(int leftPosition, int rightPosition, bool shouldInterruptForAltitude)
{
    stepperLeft.moveTo(leftPosition);
    stepperRight.moveTo(rightPosition);

    bool belowBrakeAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) <= brakeAlt;
    int lastAltitudeCheck = millis();

    while (stepperLeft.distanceToGo() != 0 && stepperRight.distanceToGo() != 0 && (!shouldInterruptForAltitude || !belowBrakeAltitude))
    {
        stepperLeft.run();
        stepperRight.run();
        // if we are intererupting when we hit 50 feet, and it's been awhile since we last checked the altitude, check again
        if (shouldInterruptForAltitude && millis() - lastAltitudeCheck > 100)
        {
            // check to see if we are below the break altitude
            bool belowBrakeAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) <= brakeAlt;
        }
    }

    return shouldInterruptForAltitude &&
           belowBrakeAltitude;
}

// print data to file
void printFile()
{
    myFile = SD.open("test.txt", FILE_WRITE);

    // Print to file
    myFile.print("{");
    // record time
    int time = millis();
    myFile.print(time);
    myFile.print(";");
    // record altitude
    myFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    myFile.print(";");
    // record pin out
    myFile.print(pinOut);
    myFile.print(";");
    // record state
    myFile.print(state);
    myFile.print("}");
    myFile.println();
    myFile.close();
}

// print data to serial
void printSerial()
{
    // Serial testing
    Serial.print("{");
    unsigned long time = millis();
    Serial.print(time);
    Serial.print(";");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.print(";");
    Serial.print(pinOut);
    Serial.print(";");
    Serial.print(state);
    Serial.print("}");
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    }

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
    brakeAlt = startAlt + 50;

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

    ////Altimeter setup
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // if the altimeter is performing
    // fix
    if (!bmp.performReading())
    {
        return;
    }

    // wait for pin to be pulled
    printFile();
    printSerial();
    while (digitalRead(PULL_SWITCH) == HIGH)
    {
        delay(10);
    }
    printFile();
    printSerial();

    pinOut = true;

    // TODO: re-evaluate this in a meeting
    delay(2000);

    // once pin is pulled, turn motor, check for braking altitude
    bool hitAltitude = false;
    while (1)
    {

        state = BREAK_STATE_TURN_LEFT;
        printSerial();
        printFile();
        hitAltitude = moveBothSteppers(STEPPER_POSITION_BRAKE, STEPPER_POSITION_RELEASE, true);
        if (hitAltitude)
            break;
        state = BREAK_STATE_NEUTRAL;
        printFile();
        printSerial();
        hitAltitude = moveBothSteppers(STEPPER_POSITION_NEUTRAL, STEPPER_POSITION_NEUTRAL, true);
        if (hitAltitude)
            break;
    }
    state = BREAK_STATE_BREAK;
    moveBothSteppers(STEPPER_POSITION_BRAKE, STEPPER_POSITION_BRAKE, false);
    printFile();
    printSerial();
    Serial.print("end");
    myFile.print("end");
    // don't do anything else
    for (;;)
    {
    }
}

void loop() {}