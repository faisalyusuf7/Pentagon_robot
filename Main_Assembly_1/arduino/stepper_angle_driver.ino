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
 *   90°  = crank pointing along +Y  (straight up)
 *   180° = crank pointing along -X  (horizontal left)
 *
 *   These are raw stepper-frame angles used by this firmware.
 *   Jetson/ROS bridge sends:
 *     stepper_deg = home_offset_deg - theta_urdf_deg
 *   home_offset_deg is set in ROS launch/bridge to match mechanical calibration.
 *
 * SERIAL PROTOCOL  (115200 baud, newline-terminated)
 * --------------------------------------------------
 *   Command              | Response       | Description
 *   ---------------------|----------------|----------------------------------
 *   A<deg1> B<deg2>      | OK             | Move both motors (absolute degrees)
 *   G28                  | OK HOME        | Move to configured home angles
 *   M114                 | POS A:xx B:xx  | Report current angles (degrees)
 *   M119                 | ENDSTOP ...     | Report home switch states
 *   M400                 | OK IDLE        | Block until both motors stop
 *   M17                  | OK EN          | Enable motors (LOW on CNC shield)
 *   M18                  | OK DIS         | Disable motors (HIGH)
 *   S<speed>             | OK SPD=xxxx    | Set max speed (steps/sec)
 *   ?                    | READY          | Heartbeat / connection check
 */

#include <AccelStepper.h>

// ===================== CONFIGURATION =====================

// Set true only after homing endstops are wired and verified.
// Default false is safer for systems without physical switches.
const bool USE_HOME_SWITCHES = false;

// Auto-home once at boot when USE_HOME_SWITCHES = true.
const bool AUTO_HOME_ON_BOOT = false;

// Stepper parameters
const float STEPS_PER_REV    = 200.0;   // 1.8° per step (typical NEMA-17)
const float MICROSTEPS       = 16.0;    // CNC shield jumper setting
const float STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS) / 360.0;  // ≈ 8.889

// Speed / acceleration  (TMC2208 StealthChop — keep ≤ 2000 sps)
// NOTE: reduced from 2500 to 1800 to prevent thermal throttling
// that causes step-loss after 2-3 cycles.
float maxSpeed        = 1800.0;   // steps/sec (adjustable via S command)
const float ACCEL     = 1500.0;   // steps/sec² (lower = less heat)

// Motor-zero home in stepper frame.
// If you calibrated driver zero with both arms parallel (+X), keep 0.0.
const float HOME_ANGLE_L = 0.0;
const float HOME_ANGLE_R = 0.0;

// CNC Shield V3 pins
const int M1_STEP = 2,  M1_DIR = 5;   // X-axis connector → Left motor
const int M2_STEP = 3,  M2_DIR = 6;   // Y-axis connector → Right motor
const int EN_PIN  = 8;                 // Shared enable (active LOW)

// Home switch pins (active LOW with INPUT_PULLUP)
const int HOME_L_PIN = 9;
const int HOME_R_PIN = 10;

// Homing motion parameters
const float HOME_SEEK_SPEED = 600.0;    // steps/sec toward switch
const float HOME_LATCH_SPEED = 250.0;   // slower final approach
const long HOME_SEEK_STEPS = 1200;      // max travel for seek phase (safer default)
const long HOME_BACKOFF_STEPS = 400;    // move off switch before latch

// Homing direction per motor: -1 or +1
// If G28 moves away from a switch, flip that motor's sign.
const int HOME_DIR_L = -1;
const int HOME_DIR_R = -1;

// ===================== GLOBALS =====================

AccelStepper stepperL(AccelStepper::DRIVER, M1_STEP, M1_DIR);
AccelStepper stepperR(AccelStepper::DRIVER, M2_STEP, M2_DIR);

float currentAngleL = HOME_ANGLE_L;
float currentAngleR = HOME_ANGLE_R;

// Serial input buffer
const int BUF_SIZE = 64;
char buf[BUF_SIZE];
int  bufIdx = 0;

// Auto position report every 500 ms
const unsigned long REPORT_INTERVAL_MS = 500;
unsigned long lastReportMs = 0;

// ===================== HELPERS =====================

bool isHomeLPressed() { return digitalRead(HOME_L_PIN) == LOW; }
bool isHomeRPressed() { return digitalRead(HOME_R_PIN) == LOW; }

void reportEndstops() {
    Serial.print("ENDSTOP L:");
    Serial.print(isHomeLPressed() ? "TRIG" : "OPEN");
    Serial.print(" R:");
    Serial.println(isHomeRPressed() ? "TRIG" : "OPEN");
}

void runBothSteppers() {
    stepperL.run();
    stepperR.run();
}

bool waitForHomeOrTimeout(unsigned long timeoutMs) {
    unsigned long t0 = millis();
    bool lStopped = false;
    bool rStopped = false;

    while (true) {
        if (!lStopped && isHomeLPressed()) {
            stepperL.stop();
            lStopped = true;
        }
        if (!rStopped && isHomeRPressed()) {
            stepperR.stop();
            rStopped = true;
        }

        runBothSteppers();

        bool lDone = isHomeLPressed() || !stepperL.isRunning();
        bool rDone = isHomeRPressed() || !stepperR.isRunning();
        if (lDone && rDone) return true;

        if (millis() - t0 > timeoutMs) return false;
    }
}

bool doAutoHome() {
    if (!USE_HOME_SWITCHES) return false;

    // Apply dedicated homing speed for reliability.
    float prevMax = maxSpeed;
    stepperL.setMaxSpeed(HOME_SEEK_SPEED);
    stepperR.setMaxSpeed(HOME_SEEK_SPEED);

    // Seek: move both axes toward the switches (configured direction).
    stepperL.moveTo(stepperL.currentPosition() + HOME_DIR_L * HOME_SEEK_STEPS);
    stepperR.moveTo(stepperR.currentPosition() + HOME_DIR_R * HOME_SEEK_STEPS);
    bool seekOk = waitForHomeOrTimeout(20000);
    stepperL.stop();
    stepperR.stop();
    while (stepperL.isRunning() || stepperR.isRunning()) {
        runBothSteppers();
    }

    if (!seekOk || !isHomeLPressed() || !isHomeRPressed()) {
        stepperL.setMaxSpeed(prevMax);
        stepperR.setMaxSpeed(prevMax);
        return false;
    }

    // Back off both switches.
    stepperL.move(-HOME_DIR_L * HOME_BACKOFF_STEPS);
    stepperR.move(-HOME_DIR_R * HOME_BACKOFF_STEPS);
    while (stepperL.isRunning() || stepperR.isRunning()) {
        runBothSteppers();
    }

    // Latch: re-approach slowly for repeatable switch edge.
    stepperL.setMaxSpeed(HOME_LATCH_SPEED);
    stepperR.setMaxSpeed(HOME_LATCH_SPEED);
    stepperL.moveTo(stepperL.currentPosition() + HOME_DIR_L * HOME_BACKOFF_STEPS * 2);
    stepperR.moveTo(stepperR.currentPosition() + HOME_DIR_R * HOME_BACKOFF_STEPS * 2);
    bool latchOk = waitForHomeOrTimeout(12000);
    stepperL.stop();
    stepperR.stop();
    while (stepperL.isRunning() || stepperR.isRunning()) {
        runBothSteppers();
    }

    if (!latchOk || !isHomeLPressed() || !isHomeRPressed()) {
        stepperL.setMaxSpeed(prevMax);
        stepperR.setMaxSpeed(prevMax);
        return false;
    }

    // The switch hit point is mapped to the configured ROS home angles.
    long homeStepsL = (long)(HOME_ANGLE_L * STEPS_PER_DEGREE);
    long homeStepsR = (long)(HOME_ANGLE_R * STEPS_PER_DEGREE);
    stepperL.setCurrentPosition(homeStepsL);
    stepperR.setCurrentPosition(homeStepsR);
    stepperL.moveTo(homeStepsL);
    stepperR.moveTo(homeStepsR);
    currentAngleL = HOME_ANGLE_L;
    currentAngleR = HOME_ANGLE_R;

    stepperL.setMaxSpeed(prevMax);
    stepperR.setMaxSpeed(prevMax);
    return true;
}

// ===================== SETUP =====================

void setup() {
    Serial.begin(115200);

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);  // enable motors

    if (USE_HOME_SWITCHES) {
        pinMode(HOME_L_PIN, INPUT_PULLUP);
        pinMode(HOME_R_PIN, INPUT_PULLUP);
    }

    // TMC2208 needs longer STEP pulses than A4988 (≥20 µs)
    stepperL.setMinPulseWidth(20);
    stepperR.setMinPulseWidth(20);

    stepperL.setMaxSpeed(maxSpeed);
    stepperL.setAcceleration(ACCEL);
    stepperR.setMaxSpeed(maxSpeed);
    stepperR.setAcceleration(ACCEL);

    // If switches are not used, we assume mechanics are already at home.
    long homeStepsL = (long)(HOME_ANGLE_L * STEPS_PER_DEGREE);
    long homeStepsR = (long)(HOME_ANGLE_R * STEPS_PER_DEGREE);
    stepperL.setCurrentPosition(homeStepsL);
    stepperR.setCurrentPosition(homeStepsR);

    if (USE_HOME_SWITCHES && AUTO_HOME_ON_BOOT) {
        if (doAutoHome()) {
            Serial.println("OK AUTOHOME");
        } else {
            Serial.println("ERR AUTOHOME");
        }
    }

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
        float a = HOME_ANGLE_L, b = HOME_ANGLE_R;
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

    // ---------- G28  — home (switch-based if enabled, else configured home) ----------
    else if (cmd[0] == 'G' && cmd[1] == '2' && cmd[2] == '8') {
        if (USE_HOME_SWITCHES) {
            if (doAutoHome()) Serial.println("OK HOME");
            else Serial.println("ERR HOME");
        } else {
            long homeStepsL = (long)(HOME_ANGLE_L * STEPS_PER_DEGREE);
            long homeStepsR = (long)(HOME_ANGLE_R * STEPS_PER_DEGREE);
            stepperL.moveTo(homeStepsL);
            stepperR.moveTo(homeStepsR);
            currentAngleL = HOME_ANGLE_L;
            currentAngleR = HOME_ANGLE_R;
            Serial.println("OK HOME");
        }
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

    // ---------- M119  — report endstop states ----------
    else if (cmd[0] == 'M' && cmd[1] == '1' && cmd[2] == '1' && cmd[3] == '9') {
        reportEndstops();
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
