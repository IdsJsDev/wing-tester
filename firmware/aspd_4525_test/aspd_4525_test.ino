// Stand-alone Matek ASPD-4525 + OLED I2C test for Arduino Nano (ATmega328P).
// Required library: U8g2 by oliver.
// Wiring: OLED and ASPD-4525 share A4/SDA, A5/SCL, 5V and GND.

#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>
#include <string.h>

namespace cfg {
constexpr uint8_t kAirspeedAddress = 0x28;
constexpr uint8_t kOledAddressA = 0x3C;
constexpr uint8_t kOledAddressB = 0x3D;
constexpr uint8_t kMaxI2cDevices = 8;
constexpr uint16_t kSamplePeriodMs = 50;
constexpr uint8_t kDisconnectAfterErrors = 5;
constexpr uint8_t kZeroSamples = 10;
constexpr float kPressureRangePa = 6894.76F;  // 1 PSI, differential ±1 PSI
constexpr float kAirDensityKgM3 = 1.225F;
}

// Page buffer: small enough for Nano SRAM and supports Cyrillic UTF-8 text.
U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
enum ReadResult : uint8_t { READ_OK, READ_I2C_ERROR, READ_STALE, READ_SENSOR_FAULT };

bool displayFound = false;
bool sensorPresent = false;
uint8_t displayAddress = 0;
float airspeedMs = 0;
float zeroPressurePa = 0;
uint8_t zeroSamples = 0;
uint16_t rawPressure = 0;
float temperatureC = 0;
uint8_t sensorStatus = 0;
uint16_t totalReadErrors = 0;
uint16_t consecutiveReadErrors = 0;
uint16_t dropoutEvents = 0;
bool inDropout = false;
uint8_t lastReceivedBytes = 0;
ReadResult lastReadResult = READ_OK;
uint8_t i2cAddresses[cfg::kMaxI2cDevices] = {};
uint8_t i2cDeviceCount = 0;
uint32_t lastSampleAt = 0;
uint32_t lastSerialAt = 0;
uint32_t lastRenderAt = 0;

bool i2cResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void scanI2c() {
  i2cDeviceCount = 0;
  Serial.println(F("I2C scan:"));
  // 0x00–0x07 and 0x78–0x7F are reserved I2C addresses.
  for (uint8_t address = 0x08; address < 0x78; ++address) {
    if (!i2cResponds(address)) continue;
    if (i2cDeviceCount < cfg::kMaxI2cDevices) {
      i2cAddresses[i2cDeviceCount++] = address;
    }
    Serial.print(F("  found 0x"));
    if (address < 16) Serial.print('0');
    Serial.println(address, HEX);
  }
  if (i2cDeviceCount == 0) Serial.println(F("  no devices"));
}

void formatI2cAddresses(char *out, size_t outSize) {
  out[0] = '\0';
  for (uint8_t i = 0; i < i2cDeviceCount; ++i) {
    char address[6];
    snprintf(address, sizeof(address), "%02X%s", i2cAddresses[i],
             i + 1 < i2cDeviceCount ? " " : "");
    if (strlen(out) + strlen(address) + 1 >= outSize) break;
    strncat(out, address, outSize - strlen(out) - 1);
  }
  if (out[0] == '\0') strncpy(out, "none", outSize);
}

bool addressWasFound(uint8_t address) {
  for (uint8_t i = 0; i < i2cDeviceCount; ++i) {
    if (i2cAddresses[i] == address) return true;
  }
  return false;
}

ReadResult readPressure(float &pressurePa) {
  // The 4525DO returns one four-byte measurement packet. Retry brief bus glitches.
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    lastReceivedBytes = Wire.requestFrom(cfg::kAirspeedAddress, (uint8_t)4);
    if (lastReceivedBytes == 4 && Wire.available() == 4) break;
    while (Wire.available()) Wire.read();
    delay(5);
  }
  if (lastReceivedBytes != 4 || Wire.available() != 4) return READ_I2C_ERROR;

  const uint8_t first = Wire.read();
  const uint8_t second = Wire.read();
  const uint8_t third = Wire.read();
  const uint8_t fourth = Wire.read();

  sensorStatus = first >> 6;
  if (sensorStatus == 2) return READ_STALE;
  if (sensorStatus == 3) return READ_SENSOR_FAULT;

  rawPressure = ((uint16_t)(first & 0x3F) << 8) | second;
  const uint16_t rawTemperature = (((uint16_t)third << 8) | fourth) >> 5;
  temperatureC = (rawTemperature * 200.0F / 2047.0F) - 50.0F;
  // TE 4525DO, output type A: 10–90% counts encode −1…+1 PSI.
  pressurePa = ((rawPressure - 1638.3F) * (2.0F * cfg::kPressureRangePa) / 13106.4F)
               - cfg::kPressureRangePa;
  return READ_OK;
}

// AVR snprintf does not format floating-point values by default.
void formatTenths(char *out, size_t outSize, float value) {
  const long tenths = lroundf(value * 10.0F);
  const unsigned long magnitude = tenths < 0 ? (unsigned long)-tenths : (unsigned long)tenths;
  snprintf(out, outSize, "%s%lu.%lu", tenths < 0 ? "-" : "", magnitude / 10, magnitude % 10);
}

void render() {
  if (!displayFound) return;
  char line[24];
  char value[10];
  display.firstPage();
  do {
    if (zeroSamples < cfg::kZeroSamples) {
      display.setFont(u8g2_font_6x12_t_cyrillic);
      display.drawUTF8(0, 18, "Калибровка");
      snprintf(line, sizeof(line), "%u / %u", zeroSamples, cfg::kZeroSamples);
      display.drawUTF8(0, 38, "Не дуйте в трубки");
      display.drawStr(0, 58, line);
    } else {
      display.setFont(u8g2_font_6x12_t_cyrillic);
      display.drawUTF8(0, 12, "Скорость, м/с");
      formatTenths(value, sizeof(value), airspeedMs);
      // Numeric-only font occupies only 421 bytes of Flash, unlike a large Cyrillic font.
      display.setFont(u8g2_font_logisoso24_tn);
      display.drawStr(0, 45, value);
      display.setFont(u8g2_font_6x12_t_cyrillic);
      snprintf(line, sizeof(line), "Сбои:%u  Ош:%u", dropoutEvents, totalReadErrors);
      display.drawUTF8(0, 62, line);
    }
  } while (display.nextPage());
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);  // conservative speed for a shared Nano I2C bus
  Wire.setWireTimeout(3000, true);  // recover instead of freezing on a stuck I2C bus
  delay(250);             // OLED needs time to finish power-up after Nano reset
  scanI2c();

  // Use the address found by the scan, rather than an earlier probe during OLED power-up.
  if (addressWasFound(cfg::kOledAddressA)) {
    displayAddress = cfg::kOledAddressA;
  } else if (addressWasFound(cfg::kOledAddressB)) {
    displayAddress = cfg::kOledAddressB;
  }
  displayFound = displayAddress != 0;
  if (displayFound) {
    // U8g2 takes the 8-bit I2C address.
    display.setI2CAddress(displayAddress << 1);
    display.setBusClock(100000);
    display.begin();
    // Some display initializers change Wire speed; the sensor needs the same 100 kHz bus.
    Wire.setClock(100000);
  }

  sensorPresent = addressWasFound(cfg::kAirspeedAddress);
  delay(100);  // let the sensor resume its normal measurement cycle after the scan
  Serial.println(F("ASPD-4525 stand-alone test"));
  Serial.print(F("OLED: "));
  Serial.println(displayFound ? F("found") : F("not found"));
  Serial.print(F("ASPD-4525 (0x28): "));
  Serial.println(sensorPresent ? F("found") : F("not found"));
  render();
  lastRenderAt = millis();
}

void loop() {
  if (sensorPresent && millis() - lastSampleAt >= cfg::kSamplePeriodMs) {
    lastSampleAt = millis();
    float sample;
    lastReadResult = readPressure(sample);
    if (lastReadResult != READ_OK) {
      ++totalReadErrors;
      ++consecutiveReadErrors;
      if (!inDropout) {
        ++dropoutEvents;
        inDropout = true;
      }
      // One partial packet is a dropout, not proof that the whole sensor is gone.
      // Declare a disconnect only after several consecutive failed transactions.
      if (consecutiveReadErrors >= cfg::kDisconnectAfterErrors) sensorPresent = false;
    } else {
      consecutiveReadErrors = 0;
      inDropout = false;
      if (zeroSamples < cfg::kZeroSamples) {
        zeroPressurePa += sample;
        ++zeroSamples;
        if (zeroSamples == cfg::kZeroSamples) zeroPressurePa /= cfg::kZeroSamples;
      } else {
        airspeedMs = sqrtf(2.0F * fabsf(sample - zeroPressurePa) / cfg::kAirDensityKgM3);
      }
    }
  }

  if (!sensorPresent && millis() - lastSampleAt >= 500) {
    lastSampleAt = millis();
    sensorPresent = i2cResponds(cfg::kAirspeedAddress);
    if (sensorPresent) {
      consecutiveReadErrors = 0;
      lastReadResult = READ_OK;
      inDropout = false;
      zeroSamples = 0;
      zeroPressurePa = 0;
    }
  }

  if (millis() - lastSerialAt >= 1000) {
    lastSerialAt = millis();
    Serial.print(F("present=")); Serial.print(sensorPresent);
    Serial.print(F(" result=")); Serial.print(lastReadResult);
    Serial.print(F(" status=")); Serial.print(sensorStatus);
    Serial.print(F(" rx=")); Serial.print(lastReceivedBytes);
    Serial.print(F(" errors=")); Serial.print(consecutiveReadErrors);
    Serial.print('/'); Serial.print(totalReadErrors);
    Serial.print(F(" raw=")); Serial.print(rawPressure);
    Serial.print(F(" air=")); Serial.print(airspeedMs, 1);
    Serial.print(F(" T=")); Serial.println(temperatureC, 1);
  }
  // Refresh slowly because OLED and sensor share one I2C bus.
  if (millis() - lastRenderAt >= 500) {
    lastRenderAt = millis();
    render();
  }
}
