/*
 * TMC2208 Stepper Motor Test Sketch
 * ==================================
 * Simple standalone test for TMC2208 drivers on CNC Shield V3.
 * Moves both motors back and forth to verify wiring, direction,
 * microstepping, and current (Vref) settings.
 *
 * Hardware: Arduino Uno/Mega + CNC Shield V3 + TMC2208 drivers
 *
 * What it does:
 *   1. Enables motors
 *   2. Moves Motor A (X-axis) +90° then back
 *   3. Moves Motor B (Y-axis) +90° then back
 *   4. Moves BOTH motors +45° then back simultaneously
 *   5. Repeats
 *
 * Open Serial Monitor at 115200 baud to see progress.
 */

#include <AccelStepper.h>

// ===================== CONFIGURATION =====================

// Motor parameters
const float STEPS_PER_REV    = 200.0;    // 1.8° NEMA-17
const float MICROSTEPS       = 16.0;     // TMC2208: both MS jumpers installed = 1/16
const float STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS) / 360.0;  // ~8.889

// TMC2208-safe speed limits (StealthChop mode)
const float MAX_SPEED = 2000.0;   // steps/sec — don't exceed ~2500 for TMC2208
const float ACCEL     = 1500.0;   // steps/sec²

// CNC Shield V3 pins
const int M1_STEP = 2,  M1_DIR = 5;   // X-axis → Motor A (left)
const int M2_STEP = 3,  M2_DIR = 6;   // Y-axis → Motor B (right)
const int EN_PIN  = 8;                 // Shared enable (active LOW)

// Test angles
const float TEST_ANGLE_SINGLE = 90.0;   // degrees for single-motor test
const float TEST_ANGLE_BOTH   = 45.0;   // degrees for dual-motor test
const float PAUSE_MS          = 1000.0;  // pause between moves (ms)

// ===================== GLOBALS =====================

AccelStepper motorA(AccelStepper::DRIVER, M1_STEP, M1_DIR);
AccelStepper motorB(AccelStepper::DRIVER, M2_STEP, M2_DIR);

int testPhase = 0;
bool moveStarted = false;

// ===================== SETUP =====================

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("=================================");
    Serial.println("  TMC2208 Stepper Motor Test");
    Serial.println("=================================");
    Serial.print("Steps/degree: ");
    Serial.println(STEPS_PER_DEGREE, 3);
    Serial.print("Max speed: ");
    Serial.print((int)MAX_SPEED);
    Serial.println(" steps/sec");
    Serial.print("Accel: ");
    Serial.print((int)ACCEL);
    Serial.println(" steps/sec^2");
    Serial.println();

    // Enable motors
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);
    Serial.println("[OK] Motors ENABLED");

    // TMC2208 needs longer STEP pulse
    motorA.setMinPulseWidth(20);
    motorB.setMinPulseWidth(20);

    motorA.setMaxSpeed(MAX_SPEED);
    motorA.setAcceleration(ACCEL);
    motorB.setMaxSpeed(MAX_SPEED);
    motorB.setAcceleration(ACCEL);

    // Start at position 0
    motorA.setCurrentPosition(0);
    motorB.setCurrentPosition(0);

    Serial.println("[OK] AccelStepper initialized");
    Serial.println();
    Serial.println("Starting test sequence...");
    Serial.println("---------------------------");
    delay(1000);

    testPhase = 0;
    moveStarted = false;
}

// ===================== LOOP =====================

void loop() {
    motorA.run();
    motorB.run();

    // Wait for current move to finish
    if (moveStarted) {
        if (motorA.isRunning() || motorB.isRunning()) {
            return;  // still moving
        }
        // Move complete
        Serial.print("  Done. A pos=");
        Serial.print(motorA.currentPosition() / STEPS_PER_DEGREE, 1);
        Serial.print("°  B pos=");
        Serial.print(motorB.currentPosition() / STEPS_PER_DEGREE, 1);
        Serial.println("°");
        moveStarted = false;
        delay((unsigned long)PAUSE_MS);
        testPhase++;
    }

    // Start next phase
    long stepsS = (long)(TEST_ANGLE_SINGLE * STEPS_PER_DEGREE);
    long stepsB = (long)(TEST_ANGLE_BOTH * STEPS_PER_DEGREE);

    switch (testPhase) {
        case 0:
            Serial.println("[1] Motor A (X-axis) → +90°");
            motorA.moveTo(stepsS);
            moveStarted = true;
            break;

        case 1:
            Serial.println("[2] Motor A (X-axis) → 0° (return)");
            motorA.moveTo(0);
            moveStarted = true;
            break;

        case 2:
            Serial.println("[3] Motor B (Y-axis) → +90°");
            motorB.moveTo(stepsS);
            moveStarted = true;
            break;

        case 3:
            Serial.println("[4] Motor B (Y-axis) → 0° (return)");
            motorB.moveTo(0);
            moveStarted = true;
            break;

        case 4:
            Serial.println("[5] BOTH motors → +45°");
            motorA.moveTo(stepsB);
            motorB.moveTo(stepsB);
            moveStarted = true;
            break;

        case 5:
            Serial.println("[6] BOTH motors → 0° (return)");
            motorA.moveTo(0);
            motorB.moveTo(0);
            moveStarted = true;
            break;

        case 6:
            Serial.println();
            Serial.println("=== Test cycle complete ===");
            Serial.println("Restarting in 3 seconds...");
            Serial.println();
            delay(3000);
            testPhase = 0;
            break;
    }
}
