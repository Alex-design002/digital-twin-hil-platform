/*
 * TRAXXAS TRX-4 HIL DIGITAL TWIN — ARDUINO NANO BENCH & PWM BRIDGE
 * Target: Arduino Nano V3.0 (ATmega328P, 5.0V TTL, 115200 baud)
 *
 * Dual-Role in Wokwi (since Wokwi has no Raspberry Pi 5 Linux part):
 *   1. USB-CDC Bridge Mode (LIVE / HOLD):
 *      Receives $<ESC_US>,<SHIFT_US>,<DIFF_F_US>,<DIFF_R_US>\n from Pi 5
 *      and drives D9..D12 at 50 Hz with strict 300ms watchdog failsafe.
 *   2. Bench Twin Demo Mode (Pushbutton D2 or Serial "BENCH"):
 *      Reads MPU-6050 (I2C A4/A5), cycles FREE -> HARSH -> STOP -> IMU_AUTO,
 *      and triggers Piezo Buzzer alarm (D4) on STOP or steep tilt (>35 deg).
 */

#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const uint8_t PIN_BTN_STEP   = 2;
const uint8_t PIN_BUZZER     = 4;
const uint8_t PIN_ESC        = 9;
const uint8_t PIN_SHIFT      = 10;
const uint8_t PIN_DIFF_FRONT = 11;
const uint8_t PIN_DIFF_REAR  = 12;
const uint8_t PIN_STATUS_LED = LED_BUILTIN;

const uint8_t MPU_ADDR       = 0x68;
const int PWM_MIN_US         = 1000;
const int PWM_MAX_US         = 2000;
const int ESC_NEUTRAL_US     = 1500;
const int SERVO_SAFE_US      = 1900; // Low gear / Locked diffs

const unsigned long FAILSAFE_TIMEOUT_MS = 300;
const size_t RX_BUF_SIZE = 48;

enum BenchState {
  BENCH_OFF = 0,
  BENCH_FREE,
  BENCH_HARSH,
  BENCH_STOP,
  BENCH_IMU_AUTO
};

Servo escThrottle;
Servo servoShift;
Servo servoDiffFront;
Servo servoDiffRear;

char rxBuffer[RX_BUF_SIZE];
size_t rxIndex = 0;

unsigned long lastValidPacketMs = 0;
unsigned long lastLedToggleMs   = 0;
unsigned long lastImuPollMs     = 0;
unsigned long lastBeepMs        = 0;
unsigned long lastBtnChangeMs   = 0;

bool failsafeActive  = true;
bool watchdogEnabled = true;
bool imuDetected     = false;
bool stopAlarmActive = false;
bool lastBtnReading  = HIGH;
bool btnLatchedState = HIGH;

BenchState benchState = BENCH_OFF;
int lastImuGovernor   = -1; // 0=FREE, 1=HARSH, 2=STOP

void writeOutputs(int esc, int shift, int diffF, int diffR) {
  escThrottle.writeMicroseconds(esc);
  servoShift.writeMicroseconds(shift);
  servoDiffFront.writeMicroseconds(diffF);
  servoDiffRear.writeMicroseconds(diffR);
}

void applyFailsafe() {
  writeOutputs(ESC_NEUTRAL_US, SERVO_SAFE_US, SERVO_SAFE_US, SERVO_SAFE_US);
  failsafeActive = true;
  stopAlarmActive = false;
  noTone(PIN_BUZZER);
}

bool readMpuPitchDeg(float &pitchDeg, int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  uint8_t n = Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  if (n < 6) {
    return false;
  }
  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());
  float denom = sqrt((float)ay * (float)ay + (float)az * (float)az);
  pitchDeg = atan2((float)ax, denom) * 57.29578f;
  return true;
}

void enterBenchStep(BenchState next) {
  benchState = next;
  lastImuGovernor = -1;

  if (next == BENCH_OFF) {
    watchdogEnabled = true;
    lastValidPacketMs = millis();
    applyFailsafe();
    Serial.println("BENCH:OFF -> MODE:LIVE (USB 300ms Watchdog restored)");
    return;
  }

  watchdogEnabled = false;
  failsafeActive = false;

  if (next == BENCH_FREE) {
    stopAlarmActive = false;
    noTone(PIN_BUZZER);
    writeOutputs(2000, 1100, 1100, 1100);
    Serial.println("BENCH:FREE -> OK:2000,1100,1100,1100 (High Gear, Open Diffs, 100% Fwd)");
  } else if (next == BENCH_HARSH) {
    stopAlarmActive = false;
    noTone(PIN_BUZZER);
    writeOutputs(1625, 1900, 1900, 1900);
    Serial.println("BENCH:HARSH -> OK:1625,1900,1900,1900 (Low Gear, Diffs Locked, Capped Throttle)");
  } else if (next == BENCH_STOP) {
    writeOutputs(1500, 1900, 1900, 1900);
    stopAlarmActive = true;
    tone(PIN_BUZZER, 2200, 150);
    lastBeepMs = millis();
    Serial.println("BENCH:STOP -> OK:1500,1900,1900,1900 (Neutral Lock + Piezo Alarm)");
  } else if (next == BENCH_IMU_AUTO) {
    Serial.println("BENCH:IMU_AUTO -> Tilt MPU-6050 (<18 deg=FREE, 18-35=HARSH, >35=STOP+Buzzer)");
  }
}

void cycleBenchState() {
  BenchState next = (BenchState)(((int)benchState + 1) % 5);
  enterBenchStep(next);
}

void updateImuGovernor(unsigned long now) {
  if (benchState != BENCH_IMU_AUTO) return;
  if (now - lastImuPollMs < 120) return;
  lastImuPollMs = now;

  float pitchDeg = 0.0f;
  int16_t ax = 0, ay = 0, az = 0;
  if (!readMpuPitchDeg(pitchDeg, ax, ay, az)) return;

  float absTilt = fabs(pitchDeg);
  int gov = (absTilt > 35.0f) ? 2 : (absTilt >= 18.0f ? 1 : 0);

  if (gov != lastImuGovernor) {
    lastImuGovernor = gov;
    if (gov == 0) {
      stopAlarmActive = false;
      noTone(PIN_BUZZER);
      writeOutputs(2000, 1100, 1100, 1100);
      Serial.print("IMU_GOV:FREE pitch="); Serial.print(pitchDeg, 1); Serial.println(" deg -> 2000,1100,1100,1100");
    } else if (gov == 1) {
      stopAlarmActive = false;
      noTone(PIN_BUZZER);
      writeOutputs(1625, 1900, 1900, 1900);
      Serial.print("IMU_GOV:HARSH pitch="); Serial.print(pitchDeg, 1); Serial.println(" deg -> 1625,1900,1900,1900");
    } else {
      writeOutputs(1500, 1900, 1900, 1900);
      stopAlarmActive = true;
      tone(PIN_BUZZER, 2200, 150);
      lastImuGovernor = gov;
      lastBeepMs = now;
      Serial.print("IMU_GOV:STOP pitch="); Serial.print(pitchDeg, 1); Serial.println(" deg -> 1500,1900,1900,1900 + ALARM");
    }
  }
}

void updateBuzzerAlarm(unsigned long now) {
  if (!stopAlarmActive) return;
  if (now - lastBeepMs >= 600) {
    lastBeepMs = now;
    tone(PIN_BUZZER, 2200, 150);
  }
}

void pollStepButton(unsigned long now) {
  bool reading = digitalRead(PIN_BTN_STEP);
  if (reading != lastBtnReading) {
    lastBtnChangeMs = now;
    lastBtnReading = reading;
  }
  if ((now - lastBtnChangeMs) > 35 && reading != btnLatchedState) {
    btnLatchedState = reading;
    if (btnLatchedState == LOW) {
      cycleBenchState();
    }
  }
}

void processLine(char *line) {
  size_t len = strlen(line);
  while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ')) {
    line[--len] = '\0';
  }
  if (len == 0) return;

  if (strcmp(line, "PING") == 0) {
    Serial.println("PONG");
    return;
  }
  if (strcmp(line, "HOLD") == 0) {
    benchState = BENCH_OFF;
    watchdogEnabled = false;
    stopAlarmActive = false;
    noTone(PIN_BUZZER);
    Serial.println("MODE:HOLD (Watchdog paused for manual testing)");
    return;
  }
  if (strcmp(line, "LIVE") == 0) {
    benchState = BENCH_OFF;
    watchdogEnabled = true;
    stopAlarmActive = false;
    noTone(PIN_BUZZER);
    lastValidPacketMs = millis();
    Serial.println("MODE:LIVE (300ms Watchdog active)");
    return;
  }
  if (strcmp(line, "BENCH") == 0) {
    cycleBenchState();
    return;
  }
  if (strcmp(line, "IMU") == 0) {
    float pitchDeg = 0.0f;
    int16_t ax = 0, ay = 0, az = 0;
    if (readMpuPitchDeg(pitchDeg, ax, ay, az)) {
      Serial.print("IMU:pitch="); Serial.print(pitchDeg, 1);
      Serial.print("deg,ax="); Serial.print(ax);
      Serial.print(",ay="); Serial.print(ay);
      Serial.print(",az="); Serial.println(az);
    } else {
      Serial.println("ERR:IMU_NACK");
    }
    return;
  }

  if (line[0] != '$') {
    Serial.println("ERR:BAD_PREFIX");
    return;
  }

  // Parse 4 comma-separated strict unsigned integers in [PWM_MIN_US, PWM_MAX_US]
  int vals[4];
  int count = 0;
  char *tok = strtok(line + 1, ",");
  while (tok != NULL && count < 4) {
    if (*tok == '\0') {
      Serial.println("ERR:RANGE");
      return;
    }
    long parsed = 0;
    for (const char *p = tok; *p != '\0'; ++p) {
      if (*p < '0' || *p > '9') {
        Serial.println("ERR:RANGE");
        return;
      }
      parsed = parsed * 10L + (*p - '0');
      if (parsed > PWM_MAX_US) {
        Serial.println("ERR:RANGE");
        return;
      }
    }
    if (parsed < PWM_MIN_US) {
      Serial.println("ERR:RANGE");
      return;
    }
    vals[count++] = (int)parsed;
    tok = strtok(NULL, ",");
  }

  if (count != 4 || tok != NULL) {
    Serial.println("ERR:BAD_FIELDS");
    return;
  }

  benchState = BENCH_OFF;
  stopAlarmActive = false;
  noTone(PIN_BUZZER);

  int esc   = vals[0];
  int shift = vals[1];
  int diffF = vals[2];
  int diffR = vals[3];

  writeOutputs(esc, shift, diffF, diffR);
  lastValidPacketMs = millis();
  failsafeActive = false;

  Serial.print("OK:");
  Serial.print(esc);   Serial.print(",");
  Serial.print(shift); Serial.print(",");
  Serial.print(diffF); Serial.print(",");
  Serial.println(diffR);
}

void updateStatusLed(unsigned long now) {
  if (!watchdogEnabled) {
    if (now - lastLedToggleMs >= 500) {
      lastLedToggleMs = now;
      digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
    }
  } else if (failsafeActive) {
    if (now - lastLedToggleMs >= 100) {
      lastLedToggleMs = now;
      digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
    }
  } else {
    digitalWrite(PIN_STATUS_LED, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BTN_STEP, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 wake
  Wire.write(0x00);
  imuDetected = (Wire.endTransmission(true) == 0);

  escThrottle.attach(PIN_ESC, PWM_MIN_US, PWM_MAX_US);
  servoShift.attach(PIN_SHIFT, PWM_MIN_US, PWM_MAX_US);
  servoDiffFront.attach(PIN_DIFF_FRONT, PWM_MIN_US, PWM_MAX_US);
  servoDiffRear.attach(PIN_DIFF_REAR, PWM_MIN_US, PWM_MAX_US);

  applyFailsafe();
  lastValidPacketMs = millis();

  Serial.println("TRX4_PWM_BRIDGE_READY");
  Serial.print("IMU_6050:");
  Serial.println(imuDetected ? "ONLINE" : "MISSING");
  Serial.println("Tip: Press blue STEP button (or send BENCH / HOLD / $2000,1100,1100,1100)");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      rxBuffer[rxIndex] = '\0';
      processLine(rxBuffer);
      rxIndex = 0;
    } else if (rxIndex < RX_BUF_SIZE - 1) {
      rxBuffer[rxIndex++] = c;
    } else {
      rxIndex = 0;
    }
  }

  unsigned long now = millis();
  pollStepButton(now);
  updateImuGovernor(now);
  updateBuzzerAlarm(now);

  if (watchdogEnabled && !failsafeActive && (now - lastValidPacketMs > FAILSAFE_TIMEOUT_MS)) {
    applyFailsafe();
    Serial.println("FAILSAFE:TIMEOUT");
  }

  updateStatusLed(now);
}
