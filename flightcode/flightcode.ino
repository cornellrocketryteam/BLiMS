#include <AccelStepper.h>
#include <Adafruit_BMP3XX.h>

#define PULL_SWITCH_PINOUT A1

// altimeter pins
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

// constants
#define SEALEVELPRESSURE_HPA (1013.25)
#define STEPPER_POSITION_BRAKE -100
#define STEPPER_POSITION_NEUTRAL 0
#define STEPPER_POSITION_RELEASE 100

// Break states
enum break_state_t {
    BREAK_STATE_NEUTRAL = 0,
    BREAK_STATE_BREAK,
    BREAK_STATE_TURN_LEFT,
    BREAK_STATE_TURN_RIGHT
};

// Stepper motors
AccelStepper stepperRight(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper stepperLeft(AccelStepper::FULL4WIRE, 6, 7, 8, 9);

// Altimeter
Adafruit_BMP3XX bmp;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        // no-op
    }

    pinMode(PULL_SWITCH_PINOUT, INPUT_PULLUP);
}

void loop() {
}