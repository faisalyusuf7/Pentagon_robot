// ============================================================
// Pressure Sensor Calibration (HX710/HX711-style timing)
// Wiring (as requested):
//   SCK -> A0
//   OUT -> A1
//   VCC -> 5V
//   GND -> GND
// ============================================================

#include <Arduino.h>

#define SCK A0
#define OUT A1

const unsigned long READY_TIMEOUT_MS = 1000;
const unsigned int CLOCK_DELAY_US = 5;
const int ZERO_SAMPLES = 50;

int32_t zeroOffset = 0;
float countsPerKpa = 0.0f;
bool hasScale = false;

bool waitReady() {
  unsigned long start = millis();
  while (digitalRead(OUT) == HIGH) {
    if (millis() - start > READY_TIMEOUT_MS) return false;
    delay(1);
  }
  return true;
}

bool readRaw24(int32_t &rawSigned) {
  if (!waitReady()) return false;

  uint32_t value = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(SCK, HIGH);
    delayMicroseconds(CLOCK_DELAY_US);

    value <<= 1;
    if (digitalRead(OUT)) value |= 1UL;

    digitalWrite(SCK, LOW);
    delayMicroseconds(CLOCK_DELAY_US);
  }

  // Required extra pulse to finish conversion cycle.
  digitalWrite(SCK, HIGH);
  delayMicroseconds(CLOCK_DELAY_US);
  digitalWrite(SCK, LOW);
  delayMicroseconds(CLOCK_DELAY_US);

  if (value & 0x800000UL) {
    value |= 0xFF000000UL;
  }
  rawSigned = (int32_t)value;
  return true;
}

bool averageRaw(int samples, int32_t &avgOut) {
  long total = 0;
  int good = 0;
  for (int i = 0; i < samples; i++) {
    int32_t raw = 0;
    if (readRaw24(raw)) {
      total += raw;
      good++;
    }
    delay(20);
  }

  if (good == 0) return false;
  avgOut = total / good;
  return true;
}

void calibrateZero() {
  Serial.println("Calibrating zero... keep sensor at 0 pressure");
  int32_t avg = 0;
  if (!averageRaw(ZERO_SAMPLES, avg)) {
    Serial.println("Zero calibration failed (sensor timeout)");
    return;
  }

  zeroOffset = avg;
  Serial.print("Zero offset set to: ");
  Serial.println(zeroOffset);
}

void calibrateScale(float knownKpa) {
  if (knownKpa == 0.0f) {
    Serial.println("Known pressure cannot be 0");
    return;
  }

  Serial.print("Calibrating scale at known pressure (kPa): ");
  Serial.println(knownKpa, 3);

  int32_t avg = 0;
  if (!averageRaw(ZERO_SAMPLES, avg)) {
    Serial.println("Scale calibration failed (sensor timeout)");
    return;
  }

  long delta = (long)avg - (long)zeroOffset;
  if (delta == 0) {
    Serial.println("Delta is zero; apply known pressure and retry");
    return;
  }

  countsPerKpa = (float)delta / knownKpa;
  hasScale = true;

  Serial.print("Scale set: countsPerKpa = ");
  Serial.println(countsPerKpa, 6);
}

void handleSerialCommands() {
  if (!Serial.available()) return;

  char cmd = (char)Serial.read();
  if (cmd == 'z' || cmd == 'Z') {
    calibrateZero();
    return;
  }

  if (cmd == 'c' || cmd == 'C') {
    float knownKpa = Serial.parseFloat();
    calibrateScale(knownKpa);
    return;
  }

  if (cmd == 'h' || cmd == 'H' || cmd == '?') {
    Serial.println("Commands:");
    Serial.println("  z           -> recalibrate zero at current pressure");
    Serial.println("  c <kPa>     -> calibrate scale using known pressure");
    Serial.println("  h or ?      -> show help");
    return;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(SCK, OUTPUT);
  pinMode(OUT, INPUT_PULLUP);
  digitalWrite(SCK, LOW);

  Serial.println("Pressure Sensor Calibration Script");
  Serial.println("Pins: SCK=A0 OUT=A1");
  Serial.println("Commands: z, c <kPa>, h");
  delay(300);

  calibrateZero();
}

void loop() {
  handleSerialCommands();

  int32_t raw = 0;
  if (!readRaw24(raw)) {
    Serial.println("Sensor timeout (OUT stayed HIGH)");
    delay(500);
    return;
  }

  long delta = (long)raw - (long)zeroOffset;

  Serial.print("raw=");
  Serial.print(raw);
  Serial.print(" zero=");
  Serial.print(zeroOffset);
  Serial.print(" delta=");
  Serial.print(delta);

  if (hasScale && countsPerKpa != 0.0f) {
    float kpa = ((float)delta) / countsPerKpa;
    float cmH2O = kpa * 10.1972f;
    Serial.print(" kPa=");
    Serial.print(kpa, 3);
    Serial.print(" cmH2O=");
    Serial.print(cmH2O, 2);
  } else {
    Serial.print(" (run: c <kPa> to calibrate scale)");
  }

  Serial.println();
  delay(500);
}
