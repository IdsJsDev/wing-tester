// Wing component tester — Arduino Nano (ATmega328P, 5 V)
// Required libraries: U8g2 (oliver) and Servo (bundled with Arduino AVR core).

#include <Wire.h>
#include <Servo.h>
#include <U8g2lib.h>
#include <math.h>
#include <string.h>

namespace cfg {
constexpr uint8_t kButtonUp = 2;
constexpr uint8_t kButtonDown = 3;
constexpr uint8_t kButtonOk = 4;
constexpr uint8_t kButtonBack = 5;
constexpr uint8_t kServoSignal = 9;

constexpr uint8_t kAirspeedAddress = 0x28;
// U8g2 uses the 8-bit form. Change to 0x3D << 1 if an I2C scan finds 0x3D.
constexpr uint8_t kOledI2cAddress8bit = 0x3C << 1;
constexpr float kPressureRangePa = 6894.76F;  // ±1 PSI
constexpr float kAirDensityKgM3 = 1.225F;

// Start conservatively. Change only after checking linkage clearances.
constexpr uint16_t kServoCentreUs = 1500;
constexpr uint16_t kServoLowUs = 1400;
constexpr uint16_t kServoHighUs = 1600;
constexpr uint16_t kServoStepMs = 1200;
}

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
Servo servo;

enum Button : uint8_t { NONE, UP, DOWN, OK, BACK };
enum Screen : uint8_t { MENU, CONFIRM_PITOT, PITOT, CONFIRM_SERVO, SERVO, MESSAGE };

Screen screen = MENU;
uint8_t selected = 0;
uint32_t stepStartedAt = 0;
uint32_t lastSampleAt = 0;
uint8_t servoStep = 0;
bool servoCancelled = false;
float zeroPressurePa = 0.0F;
float pressurePa = 0.0F;
float airspeedMs = 0.0F;
uint8_t validSamples = 0;
char message[22] = "";

Button readButton() {
  static uint8_t previous = 0x0F;
  static uint32_t changedAt = 0;
  static uint8_t reported = 0;
  uint8_t value = (digitalRead(cfg::kButtonUp) == LOW ? 1 : 0) |
                  (digitalRead(cfg::kButtonDown) == LOW ? 2 : 0) |
                  (digitalRead(cfg::kButtonOk) == LOW ? 4 : 0) |
                  (digitalRead(cfg::kButtonBack) == LOW ? 8 : 0);
  if (value != previous) {
    previous = value;
    changedAt = millis();
    return NONE;
  }
  if (value == 0) {
    reported = 0;
    return NONE;
  }
  if (millis() - changedAt < 35) return NONE;
  if (value == reported) return NONE;
  reported = value;
  if (value & 1) return UP;
  if (value & 2) return DOWN;
  if (value & 4) return OK;
  return BACK;
}

bool readPressure(float &resultPa) {
  Wire.requestFrom(cfg::kAirspeedAddress, (uint8_t)4);
  if (Wire.available() != 4) return false;
  const uint8_t first = Wire.read();
  const uint8_t second = Wire.read();
  Wire.read();  // temperature is not needed by the current test
  Wire.read();
  const uint8_t status = first >> 6;
  if (status != 0) return false;  // 2 = stale data, 3 = diagnostic fault
  const uint16_t raw = ((uint16_t)(first & 0x3F) << 8) | second;
  // 4525DO, output option A (10–90%): raw midpoint equals zero differential.
  resultPa = ((raw - 1638.3F) * (2.0F * cfg::kPressureRangePa) / 13106.4F)
             - cfg::kPressureRangePa;
  return true;
}

void setMessage(const char *text) {
  strncpy(message, text, sizeof(message) - 1);
  message[sizeof(message) - 1] = '\0';
  screen = MESSAGE;
}

void startPitot() {
  validSamples = 0;
  zeroPressurePa = 0;
  stepStartedAt = millis();
  lastSampleAt = 0;
  screen = PITOT;
}

void updatePitot() {
  if (millis() - lastSampleAt < 50) return;
  lastSampleAt = millis();
  float sample;
  if (!readPressure(sample)) {
    if (validSamples == 0 && millis() - stepStartedAt > 1500) setMessage("I2C: no ASPD-4525");
    return;
  }
  if (validSamples < 20) {
    zeroPressurePa += sample;
    ++validSamples;
    if (validSamples == 20) zeroPressurePa /= 20.0F;
    return;
  }
  pressurePa = sample - zeroPressurePa;
  airspeedMs = sqrtf(2.0F * fabsf(pressurePa) / cfg::kAirDensityKgM3);
}

void startServo() {
  servo.attach(cfg::kServoSignal, 900, 2100);
  servo.writeMicroseconds(cfg::kServoCentreUs);
  servoStep = 0;
  servoCancelled = false;
  stepStartedAt = millis();
  screen = SERVO;
}

void centreAndStopServo() {
  servoCancelled = true;
  servo.writeMicroseconds(cfg::kServoCentreUs);
  servoStep = 3;
  stepStartedAt = millis();
}

void updateServo() {
  if (millis() - stepStartedAt < cfg::kServoStepMs) return;
  stepStartedAt = millis();
  ++servoStep;
  switch (servoStep) {
    case 1: servo.writeMicroseconds(cfg::kServoLowUs); break;
    case 2: servo.writeMicroseconds(cfg::kServoHighUs); break;
    case 3: servo.writeMicroseconds(cfg::kServoCentreUs); break;
    default:
      servo.detach();
      setMessage(servoCancelled ? "Servo stopped centre" : "Servo cycle complete");
  }
}

void drawLine(uint8_t y, const char *text) { display.drawStr(0, y, text); }

void render() {
  char line[24];
  display.clearBuffer();
  display.setFont(u8g2_font_6x12_tf);
  if (screen == MENU) {
    drawLine(12, "Wing tester");
    const char *items[] = {"Pitot / ASPD-4525", "KST X10 servo", "LoRa (later)"};
    for (uint8_t i = 0; i < 3; ++i) {
      snprintf(line, sizeof(line), "%c %s", selected == i ? '>' : ' ', items[i]);
      drawLine(28 + i * 12, line);
    }
  } else if (screen == CONFIRM_PITOT) {
    drawLine(12, "Pitot test"); drawLine(28, "Keep both ports still"); drawLine(44, "OK start / Back cancel");
  } else if (screen == PITOT) {
    drawLine(12, "ASPD-4525 / I2C 0x28");
    if (validSamples < 20) { snprintf(line, sizeof(line), "Zeroing: %u/20", validSamples); drawLine(30, line); }
    else { snprintf(line, sizeof(line), "dP: %.0f Pa", pressurePa); drawLine(30, line); snprintf(line, sizeof(line), "V: %.1f m/s", airspeedMs); drawLine(46, line); }
    drawLine(62, "Back: stop");
  } else if (screen == CONFIRM_SERVO) {
    drawLine(12, "KST X10 motion test"); drawLine(28, "Clear linkage first!"); drawLine(44, "OK start / Back cancel");
  } else if (screen == SERVO) {
    drawLine(12, "KST X10: moving");
    const char *phase[] = {"Centre", "1400 us", "1600 us", "Centre"};
    snprintf(line, sizeof(line), "%s (%u us)", phase[servoStep], servoStep == 0 ? cfg::kServoCentreUs : servoStep == 1 ? cfg::kServoLowUs : cfg::kServoHighUs);
    drawLine(30, line); drawLine(62, "Back: centre + stop");
  } else {
    drawLine(12, "Result"); drawLine(32, message); drawLine(62, "OK/Back: menu");
  }
  display.sendBuffer();
}

void setup() {
  pinMode(cfg::kButtonUp, INPUT_PULLUP); pinMode(cfg::kButtonDown, INPUT_PULLUP);
  pinMode(cfg::kButtonOk, INPUT_PULLUP); pinMode(cfg::kButtonBack, INPUT_PULLUP);
  Wire.begin();
  display.setI2CAddress(cfg::kOledI2cAddress8bit);
  display.begin();
}

void loop() {
  const Button button = readButton();
  if (screen == PITOT) updatePitot();
  if (screen == SERVO) updateServo();
  if (button != NONE) {
    if (screen == MENU) {
      if (button == UP) selected = (selected + 2) % 3;
      if (button == DOWN) selected = (selected + 1) % 3;
      if (button == OK && selected == 0) screen = CONFIRM_PITOT;
      if (button == OK && selected == 1) screen = CONFIRM_SERVO;
      if (button == OK && selected == 2) setMessage("LoRa not implemented");
    } else if (screen == CONFIRM_PITOT) { screen = button == OK ? PITOT : MENU; if (button == OK) startPitot(); }
    else if (screen == CONFIRM_SERVO) { screen = button == OK ? SERVO : MENU; if (button == OK) startServo(); }
    else if (screen == PITOT && button == BACK) { setMessage("Pitot test stopped"); }
    else if (screen == SERVO && button == BACK) { centreAndStopServo(); }
    else if (screen == MESSAGE) screen = MENU;
  }
  render();
}
