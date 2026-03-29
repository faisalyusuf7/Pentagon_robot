/*
 * 5-Bar Stepper Angle Driver — Arduino + CNC Shield V3
 * =====================================================
 * The Jetson / Python handles ALL kinematics.
 * This firmware ONLY receives target angles (degrees) and moves
 * the two stepper motors to those positions.
 *
 * Hardware: Arduino Uno/Mega  +  CNC Shield V3  +  2× NEMA-17
 *
 * ANGLE CONVENTION
 * ----------------
 *   0°   = crank pointing along +X  (horizontal right)
 *   90°  = crank pointing along +Y  (straight up)   ← HOME
 *   180° = crank pointing along -X  (horizontal left)
 *
 *   These are the raw IK-plane angles.
 *   Jetson sends:  stepper_deg = 90 − theta_urdf_deg
 *
 * SERIAL PROTOCOL  (115200 baud, newline-terminated)
 * --------------------------------------------------
 *   Command              | Response       | Description
 *   ---------------------|----------------|----------------------------------
 *   A<deg1> B<deg2>      | OK             | Move both motors (absolute degrees)
 *   G28                  | OK HOME        | Move to home (90° / 90°)
 *   M114                 | POS A:xx B:xx  | Report current angles (degrees)
 *   M400                 | OK IDLE        | Block until both motors stop
 *   M17                  | OK EN          | Enable motors (LOW on CNC shield)
 *   M18                  | OK DIS         | Disable motors (HIGH)
 *   S<speed>             | OK SPD=xxxx    | Set max speed (steps/sec)
 *   ?                    | READY          | Heartbeat / connection check
 */

#include <AccelStepper.h>

// ===================== CONFIGURATION =====================

// Stepper parameters
const float STEPS_PER_REV    = 200.0;   // 1.8° per step (typical NEMA-17)
const float MICROSTEPS       = 16.0;    // CNC shield jumper setting
const float STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS) / 360.0;  // ≈ 8.889

// Speed / acceleration  (TMC2208 StealthChop — keep ≤ 2000 sps)
// NOTE: reduced from 2500 to 1800 to prevent thermal throttling
// that causes step-loss after 2-3 cycles.
float maxSpeed        = 1800.0;   // steps/sec (adjustable via S command)
const float ACCEL     = 1500.0;   // steps/sec² (lower = less heat)

// Home angle (degrees) — both motors point +Y at startup
const float HOME_ANGLE = 90.0;

// CNC Shield V3 pins
const int M1_STEP = 2,  M1_DIR = 5;   // X-axis connector → Left motor
const int M2_STEP = 3,  M2_DIR = 6;   // Y-axis connector → Right motor
const int EN_PIN  = 8;                 // Shared enable (active LOW)

// ===================== GLOBALS =====================

AccelStepper stepperL(AccelStepper::DRIVER, M1_STEP, M1_DIR);
AccelStepper stepperR(AccelStepper::DRIVER, M2_STEP, M2_DIR);

float currentAngleL = HOME_ANGLE;
float currentAngleR = HOME_ANGLE;

// Serial input buffer
const int BUF_SIZE = 64;
char buf[BUF_SIZE];
int  bufIdx = 0;

// Auto position report every 500 ms
const unsigned long REPORT_INTERVAL_MS = 500;
unsigned long lastReportMs = 0;

// ===================== SETUP =====================

void setup() {
    Serial.begin(115200);

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);  // enable motors

    // TMC2208 needs longer STEP pulses than A4988 (≥20 µs)
    stepperL.setMinPulseWidth(20);
    stepperR.setMinPulseWidth(20);

    stepperL.setMaxSpeed(maxSpeed);
    stepperL.setAcceleration(ACCEL);
    stepperR.setMaxSpeed(maxSpeed);
    stepperR.setAcceleration(ACCEL);

    // Tell AccelStepper where the motors physically are right now
    long homeSteps = (long)(HOME_ANGLE * STEPS_PER_DEGREE);
    stepperL.setCurrentPosition(homeSteps);
    stepperR.setCurrentPosition(homeSteps);

    delay(300);
    Serial.println("READY");
    Serial.flush();
}

// ===================== LOOP =====================

void loop() {
    // Non-blocking stepper tick
    stepperL.run();
    stepperR.run();

    // Periodic position broadcast
    unsigned long now = millis();
    if (now - lastReportMs >= REPORT_INTERVAL_MS) {
        lastReportMs = now;
        float posL = stepperL.currentPosition() / STEPS_PER_DEGREE;
        float posR = stepperR.currentPosition() / STEPS_PER_DEGREE;
        Serial.print("POS A:");
        Serial.print(posL, 2);
        Serial.print(" B:");
        Serial.println(posR, 2);
    }

    // Read serial bytes into buffer
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (bufIdx > 0) {
                buf[bufIdx] = '\0';
                processCommand(buf);
                bufIdx = 0;
            }
        } else if (bufIdx < BUF_SIZE - 1) {
            buf[bufIdx++] = c;
        }
    }
}

// ===================== COMMAND PROCESSOR =====================

void processCommand(const char* cmd) {

    // ---------- A<deg> B<deg>  — absolute angle move ----------
    if (cmd[0] == 'A' || cmd[0] == 'a') {
        float a = HOME_ANGLE, b = HOME_ANGLE;
        bool gotA = false, gotB = false;

        const char* p = cmd;
        while (*p) {
            if (*p == 'A' || *p == 'a') { a = atof(p + 1); gotA = true; }
            if (*p == 'B' || *p == 'b') { b = atof(p + 1); gotB = true; }
            p++;
        }

        if (gotA) {
            long stepsA = (long)(a * STEPS_PER_DEGREE);
            stepperL.moveTo(stepsA);
            currentAngleL = a;
        }
        if (gotB) {
            long stepsB = (long)(b * STEPS_PER_DEGREE);
            stepperR.moveTo(stepsB);
            currentAngleR = b;
        }
        Serial.println("OK");
    }

    // ---------- G28  — home (90° / 90°) ----------
    else if (cmd[0] == 'G' && cmd[1] == '2' && cmd[2] == '8') {
        long homeSteps = (long)(HOME_ANGLE * STEPS_PER_DEGREE);
        stepperL.moveTo(homeSteps);
        stepperR.moveTo(homeSteps);
        currentAngleL = HOME_ANGLE;
        currentAngleR = HOME_ANGLE;
        Serial.println("OK HOME");
    }

    // ---------- M114  — report position ----------
    else if (cmd[0] == 'M' && cmd[1] == '1' && cmd[2] == '1' && cmd[3] == '4') {
        // Report actual step position converted back to degrees
        float posL = stepperL.currentPosition() / STEPS_PER_DEGREE;
        float posR = stepperR.currentPosition() / STEPS_PER_DEGREE;
        Serial.print("POS A:");
        Serial.print(posL, 2);
        Serial.print(" B:");
        Serial.println(posR, 2);
    }

    // ---------- M400  — wait for idle ----------
    else if (cmd[0] == 'M' && cmd[1] == '4' && cmd[2] == '0' && cmd[3] == '0') {
        while (stepperL.isRunning() || stepperR.isRunning()) {
            stepperL.run();
            stepperR.run();
        }
        Serial.println("OK IDLE");
    }

    // ---------- M17  — enable motors ----------
    else if (cmd[0] == 'M' && cmd[1] == '1' && cmd[2] == '7') {
        digitalWrite(EN_PIN, LOW);
        Serial.println("OK EN");
    }

    // ---------- M18  — disable motors ----------
    else if (cmd[0] == 'M' && cmd[1] == '1' && cmd[2] == '8') {
        digitalWrite(EN_PIN, HIGH);
        Serial.println("OK DIS");
    }

    // ---------- S<speed>  — set max speed ----------
    else if (cmd[0] == 'S' || cmd[0] == 's') {
        float spd = atof(cmd + 1);
        if (spd > 0) {
            maxSpeed = spd;
            stepperL.setMaxSpeed(maxSpeed);
            stepperR.setMaxSpeed(maxSpeed);
        }
        Serial.print("OK SPD=");
        Serial.println((long)maxSpeed);
    }

    // ---------- ?  — heartbeat ----------
    else if (cmd[0] == '?') {
        Serial.println("READY");
    }

    else {
        Serial.print("ERR:");
        Serial.println(cmd);
    }
}
