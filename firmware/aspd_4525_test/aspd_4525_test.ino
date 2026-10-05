// Matek ASPD-4525: 500-slot 200 Hz test at 400 kHz on Arduino Nano.
// Five 100-slot blocks; OLED/Serial updates between blocks are excluded from timing.
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>
#include <avr/pgmspace.h>

namespace cfg {
constexpr uint8_t kSensorAddress = 0x28;
constexpr uint32_t kPeriodUs = 5000;
constexpr uint8_t kBlocks = 5;
constexpr uint16_t kSlotsPerBlock = 100;
constexpr uint32_t kModeMs = kBlocks * kSlotsPerBlock * kPeriodUs / 1000;
constexpr float kRangePa = 6894.76F;
// Provisional still-air warning limits, NOT manufacturer rejection limits.
constexpr float kNoiseSpanPa = 30.0F;
constexpr float kNoiseSdPa = 5.0F;
}
U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
enum Result : uint8_t { OK, BUS, STALE, FAULT, BAD_STATUS };
uint32_t counts[5] = {};
uint32_t timeouts = 0, missed = 0, attempts = 0;
uint32_t modeStartedMs = 0, nextSampleUs = 0, lastReportMs = 0;
uint32_t lastGoodMs = 0, maxGapMs = 0, maxReadUs = 0, maxLateUs = 0;
uint32_t sameSinceMs = 0, maxSameMs = 0, drops = 0;
uint32_t previousAttempts = 0, previousGood = 0;
uint16_t rawPressure = 0, rawTemperature = 0;
uint16_t windowMin = 16383, windowMax = 0;
uint16_t windowGood = 0;
uint8_t status = 255, received = 0, zeroSamples = 0;
float zeroSumPa = 0, zeroPa = 0;
bool hasGood = false, inDropout = false, fastBus = false, ending = false;
bool displayFound = false;
uint8_t completedBlocks = 0;
uint32_t excludedMs = 0;
Result result = BUS;
// Fixed buffer, drained only when UART space is available (no blocking print).
char report[384];
uint16_t reportLength = 0, reportPosition = 0;
uint16_t noiseMin = 16383, noiseMax = 0;
float noiseMean = 0, noiseM2 = 0;
uint16_t errorRun = 0, maxErrorRun = 0;

float noiseSpanPa() {
  return counts[OK] ? (noiseMax - noiseMin) * (2 * cfg::kRangePa / 13106.4F) : 0;
}
float noiseSdPa() {
  return counts[OK] > 1 ? sqrtf(noiseM2 / (counts[OK] - 1)) * (2 * cfg::kRangePa / 13106.4F) : 0;
}
bool noiseReady() {
  return missed == 0 && counts[OK] == attempts && counts[OK] >= 400;
}
bool noiseHigh() {
  return noiseSpanPa() > cfg::kNoiseSpanPa || noiseSdPa() > cfg::kNoiseSdPa;
}

uint32_t testMillis() { return millis() - excludedMs; }

const __FlashStringHelper *verdict() {
  if (missed != 0) return F("RETEST_TIMING");
  if (counts[OK] == 0) return F("NO_DATA");
  if (attempts != counts[OK]) return F("LINK_ERRORS");
  return F("LINK_PASS");
}

void drawFlash(uint8_t y, const __FlashStringHelper *text) {
  char textBuffer[48];
  strncpy_P(textBuffer, (const char *)text, sizeof(textBuffer) - 1);
  textBuffer[sizeof(textBuffer) - 1] = 0;
  display.drawUTF8(0, y, textBuffer);
}

void renderTest(bool finished) {
  if (!displayFound) return;
  char line[48];
  const uint32_t errors = attempts - counts[OK];
  const uint32_t errorTenths = attempts ? errors * 1000UL / attempts : 0;
  // OLED uses 100 kHz only outside the measured blocks.
  Wire.setClock(100000);
  display.firstPage();
  do {
    display.setFont(u8g2_font_6x12_t_cyrillic);
    if (!finished) {
      display.drawUTF8(0, 12, "Тест связи 400 кГц");
      snprintf_P(line, sizeof(line), PSTR("%u / %u   %u%%"), completedBlocks, cfg::kBlocks, completedBlocks * 100 / cfg::kBlocks);
      display.drawStr(0, 25, line);
      display.drawFrame(0, 30, 128, 9);
      if (completedBlocks) display.drawBox(2, 32, completedBlocks * 124 / cfg::kBlocks, 5);
      snprintf_P(line, sizeof(line), PSTR("Ошибок: %lu"), (unsigned long)errors);
      display.drawUTF8(0, 51, line);
      drawFlash(63, F("Не дуйте в трубки"));
    } else {
      drawFlash(12, missed ? F("ПОВТОРИТЕ ТЕСТ") :
                       (!counts[OK] ? F("НЕТ ДАННЫХ") : (errors ? F("СБОИ СВЯЗИ") : F("СВЯЗЬ: OK"))));
      snprintf_P(line, sizeof(line), PSTR("OK:%lu/%lu E:%lu"), (unsigned long)counts[OK], (unsigned long)attempts, (unsigned long)errors);
      display.drawStr(0, 25, line);
      drawFlash(38, !noiseReady() ? F("Шум: нет оценки") :
                    (noiseHigh() ? F("ШУМ: ПРОВЕРИТЬ") : F("Шум: в порогах")));
      snprintf_P(line, sizeof(line), PSTR("Loss:%lu.%lu%% Run:%u"),
                 (unsigned long)(errorTenths / 10), (unsigned long)(errorTenths % 10), maxErrorRun);
      display.drawStr(0, 51, line);
      drawFlash(63, F("RESET: повтор теста"));
    }
  } while (display.nextPage());
  Wire.setClock(400000);
}

float pressureFromRaw(uint16_t raw) {
  return ((raw - 1638.3F) * (2.0F * cfg::kRangePa) / 13106.4F) - cfg::kRangePa;
}

bool responds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void pumpSerial() {
  const int freeBytes = Serial.availableForWrite();
  uint16_t remaining = reportLength - reportPosition;
  if (freeBytes <= 0 || remaining == 0) return;
  // Bound time spent feeding the UART before checking the sample deadline again.
  if (remaining > 16) remaining = 16;
  if (remaining > (uint16_t)freeBytes) remaining = freeBytes;
  Serial.write((const uint8_t *)report + reportPosition, remaining);
  reportPosition += remaining;
}

Result readSensor() {
  status = 255;
  Wire.clearWireTimeoutFlag();
  received = Wire.requestFrom(cfg::kSensorAddress, (uint8_t)4);
  const bool timedOut = Wire.getWireTimeoutFlag();
  if (timedOut) ++timeouts;
  if (timedOut || received != 4 || Wire.available() != 4) {
    while (Wire.available()) Wire.read();
    return BUS;
  }
  const uint8_t a = Wire.read(), b = Wire.read(), c = Wire.read(), d = Wire.read();
  status = a >> 6;
  if (status == 2) return STALE;
  if (status == 3) return FAULT;
  if (status != 0) return BAD_STATUS;
  rawPressure = ((uint16_t)(a & 0x3F) << 8) | b;
  rawTemperature = (((uint16_t)c << 8) | d) >> 5;
  return OK;
}

void startMode(bool useFastBus) {
  fastBus = useFastBus;
  Wire.setClock(fastBus ? 400000UL : 100000UL);
  for (uint8_t i = 0; i < 5; ++i) counts[i] = 0;
  timeouts = missed = attempts = drops = 0;
  noiseMin = 16383;
  noiseMax = errorRun = maxErrorRun = 0;
  noiseMean = noiseM2 = 0;
  maxGapMs = maxReadUs = maxLateUs = maxSameMs = 0;
  previousAttempts = previousGood = 0;
  rawPressure = rawTemperature = 0;
  windowMin = 16383;
  windowMax = windowGood = 0;
  status = 255;
  received = zeroSamples = 0;
  zeroSumPa = zeroPa = 0;
  hasGood = inDropout = ending = false;
  result = BUS;
  // All blocking output is between measured runs.
  Serial.print(F("BEGIN bus_khz=")); Serial.print(fastBus ? 400 : 100);
  Serial.print(F(" target_hz=200 duration_ms="));
  Serial.println(cfg::kModeMs);
  Serial.flush();
  modeStartedMs = lastReportMs = lastGoodMs = sameSinceMs = testMillis();
  nextSampleUs = micros() + cfg::kPeriodUs;
}

void sampleSensor() {
  const uint32_t startedUs = micros();
  const uint32_t lateUs = startedUs - nextSampleUs;
  if (lateUs > maxLateUs) maxLateUs = lateUs;
  // Skip missed slots rather than generating bursts of catch-up transactions.
  const uint32_t skipped = lateUs / cfg::kPeriodUs;
  missed += skipped;
  nextSampleUs += (skipped + 1) * cfg::kPeriodUs;
  const uint16_t previousRaw = rawPressure;
  result = readSensor();
  const uint32_t readUs = micros() - startedUs;
  if (readUs > maxReadUs) maxReadUs = readUs;
  ++attempts;
  ++counts[result];
  const uint32_t now = testMillis();
  if (result != OK) {
    ++errorRun;
    if (errorRun > maxErrorRun) maxErrorRun = errorRun;
    if (!inDropout) ++drops;
    inDropout = true;
    return;
  }
  errorRun = 0;
  if (rawPressure < noiseMin) noiseMin = rawPressure;
  if (rawPressure > noiseMax) noiseMax = rawPressure;
  const float delta = rawPressure - noiseMean;
  noiseMean += delta / counts[OK];
  noiseM2 += delta * (rawPressure - noiseMean);
  const uint32_t gap = now - lastGoodMs;
  if (gap > maxGapMs) maxGapMs = gap;
  if (!hasGood || inDropout || rawPressure != previousRaw) sameSinceMs = now;
  const uint32_t sameMs = now - sameSinceMs;
  if (sameMs > maxSameMs) maxSameMs = sameMs;
  hasGood = true;
  inDropout = false;
  lastGoodMs = now;
  if (rawPressure < windowMin) windowMin = rawPressure;
  if (rawPressure > windowMax) windowMax = rawPressure;
  ++windowGood;
  if (zeroSamples < 10) {
    zeroSumPa += pressureFromRaw(rawPressure);
    if (++zeroSamples == 10) zeroPa = zeroSumPa / 10.0F;
  }
}

void queueReport(bool finalReport) {
  const uint32_t now = testMillis();
  const uint32_t elapsed = now - lastReportMs;
  if (elapsed == 0) return;
  const uint32_t pollTenths = (attempts - previousAttempts) * 10000UL / elapsed;
  const uint32_t goodTenths = (counts[OK] - previousGood) * 10000UL / elapsed;
  uint32_t gap = now - lastGoodMs;
  if (gap > maxGapMs) maxGapMs = gap;
  const float pressure = hasGood ? pressureFromRaw(rawPressure) : 0;
  const float dp = zeroSamples == 10 ? pressure - zeroPa : 0;
  // Integer fixed-point output avoids AVR printf float support.
  const long p10 = lroundf(pressure * 10);
  const long dp10 = lroundf(dp * 10);
  const long t10 = hasGood ? lroundf((rawTemperature * 200.0F / 2047.0F - 50) * 10) : 0;
  const unsigned long v10 = lroundf(sqrtf(2 * fabsf(dp) / 1.225F) * 10);
  const unsigned long vr10 = lroundf(sqrtf(2 * fabsf(pressure) / 1.225F) * 10);
  const int length = snprintf_P(report, sizeof(report), PSTR(
    "%s khz=%u ms=%lu poll_hz=%lu.%lu good_hz=%lu.%lu n=%lu ok=%lu bus=%lu stale=%lu fault=%lu bad=%lu timeout=%lu drops=%lu missed=%lu late_us=%lu read_us=%lu valid=%u age_ms=%lu gap_ms=%lu same_max_ms=%lu result=%u st=%u rx=%u raw=%u rawT=%u span=%u wn=%u z=%u zero10=%ld P10=%ld dP10=%ld T10=%ld V10=%lu Vr10=%lu\n"),
    finalReport ? "END" : "DATA", fastBus ? 400 : 100,
    (unsigned long)(now - modeStartedMs),
    (unsigned long)(pollTenths / 10), (unsigned long)(pollTenths % 10),
    (unsigned long)(goodTenths / 10), (unsigned long)(goodTenths % 10),
    (unsigned long)attempts, (unsigned long)counts[OK], (unsigned long)counts[BUS],
    (unsigned long)counts[STALE], (unsigned long)counts[FAULT], (unsigned long)counts[BAD_STATUS],
    (unsigned long)timeouts, (unsigned long)drops, (unsigned long)missed,
    (unsigned long)maxLateUs, (unsigned long)maxReadUs,
    (unsigned)(hasGood && result == OK), (unsigned long)(now - lastGoodMs),
    (unsigned long)maxGapMs, (unsigned long)maxSameMs,
    (unsigned)result, status, received, rawPressure, rawTemperature,
    windowGood ? windowMax - windowMin : 0, windowGood, zeroSamples,
    lroundf(zeroPa * 10), p10, dp10, t10, v10, vr10);
  reportLength = length < 0 ? 0 : (length >= (int)sizeof(report) ? sizeof(report) - 1 : length);
  reportPosition = 0;
  previousAttempts = attempts;
  previousGood = counts[OK];
  lastReportMs = now;
  windowMin = 16383;
  windowMax = windowGood = 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);
  delay(250);
  uint8_t oled = responds(0x3C) ? 0x3C : (responds(0x3D) ? 0x3D : 0);
  displayFound = oled != 0;
  if (oled) {
    display.setI2CAddress(oled << 1);
    display.setBusClock(100000);
    display.begin();
    renderTest(false);
  }
  Serial.println(F("ASPD-4525: 5 x 100 slots at 400 kHz/200 Hz. Keep ports open, no wind. RESET to repeat."));
  Serial.println(F("P10/dP10/zero10: 0.1 Pa; T10: 0.1 C; V10/Vr10: 0.1 m/s. z=10: zero ready."));
  startMode(true);
}

void loop() {
  if (ending) return;
  if ((int32_t)(micros() - nextSampleUs) >= 0) sampleSensor();
  if (attempts + missed >= (uint32_t)(completedBlocks + 1) * cfg::kSlotsPerBlock) {
    const uint32_t pauseStartedMs = millis();
    ++completedBlocks;
    ending = completedBlocks == cfg::kBlocks;
    queueReport(ending);
    // No sensor samples are scheduled during this explicit inter-block pause.
    while (reportPosition < reportLength) pumpSerial();
    Serial.flush();
    if (ending) {
      Serial.print(F("RESULT verdict=")); Serial.print(verdict());
      Serial.print(F(" blocks=")); Serial.print(completedBlocks);
      Serial.print(F(" errors=")); Serial.print(attempts - counts[OK]);
      Serial.print(F(" loss_pct=")); Serial.print(attempts ? (attempts - counts[OK]) * 100.0F / attempts : 0, 1);
      Serial.print(F(" max_error_run=")); Serial.print(maxErrorRun);
      Serial.print(F(" noise="));
      Serial.print(!noiseReady() ? F("INCONCLUSIVE") : (noiseHigh() ? F("CHECK") : F("WITHIN_LIMITS")));
      Serial.print(F(" noise_n=")); Serial.print(counts[OK]);
      Serial.print(F(" span_Pa=")); Serial.print(noiseSpanPa(), 2);
      Serial.print(F(" sd_Pa=")); Serial.print(noiseSdPa(), 2);
      Serial.print(F(" span_limit_Pa=")); Serial.print(cfg::kNoiseSpanPa, 1);
      Serial.print(F(" sd_limit_Pa=")); Serial.println(cfg::kNoiseSdPa, 1);
      Serial.flush();
    }
    renderTest(ending);
    excludedMs += millis() - pauseStartedMs;
    nextSampleUs = micros() + cfg::kPeriodUs;
  }
}
