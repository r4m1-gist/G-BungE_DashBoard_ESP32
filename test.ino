/*
  NodeMCU ESP32-S 4-digit 7-segment smoke test

  7-segment module pins:
    VCC, SCLK, RCLK, DIO, GND

  Suggested pin map:
    VCC  -> ESP32 3V3 first, or 5V only if the module requires it
    SCLK -> ESP32 GPIO18
    RCLK -> ESP32 GPIO19
    DIO  -> ESP32 GPIO23
    GND  -> ESP32 GND

  CAN transceiver, for later:
    SN65HVD230 TXD / CTX -> ESP32 GPIO26
    SN65HVD230 RXD / CRX -> ESP32 GPIO27
    SN65HVD230 VCC       -> ESP32 3V3
    SN65HVD230 GND       -> ESP32 GND
    CANH-CANL            -> 120 ohm only at each end of the CAN bus
*/

#include <Arduino.h>

static const uint8_t SEG_SCLK_PIN = 18;
static const uint8_t SEG_RCLK_PIN = 19;
static const uint8_t SEG_DIO_PIN = 23;

static const uint8_t CAN_TX_PIN = 26;
static const uint8_t CAN_RX_PIN = 27;

// Change these if your module displays inverted/garbled output.
static const bool SEGMENT_ACTIVE_LOW = true;
static const bool DIGIT_ACTIVE_HIGH = true;
static const bool SEND_SEGMENTS_FIRST = true;

static const uint16_t COUNT_HOLD_MS = 100;
static const uint16_t DIGIT_REFRESH_US = 250;

// Common segment bit order: bit0=a, bit1=b, ... bit6=g, bit7=dp.
static const uint8_t SEG_A  = 0b00000001;
static const uint8_t SEG_B  = 0b00000010;
static const uint8_t SEG_C  = 0b00000100;
static const uint8_t SEG_D  = 0b00001000;
static const uint8_t SEG_E  = 0b00010000;
static const uint8_t SEG_F  = 0b00100000;
static const uint8_t SEG_G  = 0b01000000;
static const uint8_t SEG_DP = 0b10000000;

static const uint8_t DIGITS[10] = {
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,          // 0
  SEG_B | SEG_C,                                          // 1
  SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,                  // 2
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,                  // 3
  SEG_B | SEG_C | SEG_F | SEG_G,                          // 4
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,                  // 5
  SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,          // 6
  SEG_A | SEG_B | SEG_C,                                  // 7
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,  // 8
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G           // 9
};

// Left to right digit select bits.
static const uint8_t DIGIT_SELECT[4] = {
  0b00001000,
  0b00000100,
  0b00000010,
  0b00000001
};

uint8_t normalizeSegments(uint8_t pattern) {
  return SEGMENT_ACTIVE_LOW ? ~pattern : pattern;
}

uint8_t normalizeDigitSelect(uint8_t digitMask) {
  return DIGIT_ACTIVE_HIGH ? digitMask : ~digitMask;
}

void writeTwoBytes(uint8_t segments, uint8_t digitSelect) {
  digitalWrite(SEG_RCLK_PIN, LOW);

  if (SEND_SEGMENTS_FIRST) {
    shiftOut(SEG_DIO_PIN, SEG_SCLK_PIN, MSBFIRST, normalizeSegments(segments));
    shiftOut(SEG_DIO_PIN, SEG_SCLK_PIN, MSBFIRST, normalizeDigitSelect(digitSelect));
  } else {
    shiftOut(SEG_DIO_PIN, SEG_SCLK_PIN, MSBFIRST, normalizeDigitSelect(digitSelect));
    shiftOut(SEG_DIO_PIN, SEG_SCLK_PIN, MSBFIRST, normalizeSegments(segments));
  }

  digitalWrite(SEG_RCLK_PIN, HIGH);
}

void clearDisplay() {
  writeTwoBytes(0x00, 0x00);
}

void refreshDigit(uint8_t position, uint8_t number) {
  writeTwoBytes(DIGITS[number], DIGIT_SELECT[position]);
  delayMicroseconds(DIGIT_REFRESH_US);
  clearDisplay();
}

void showNumberFor(uint16_t value, uint16_t holdMs) {
  uint8_t displayDigits[4] = {
    (uint8_t)((value / 1000) % 10),
    (uint8_t)((value / 100) % 10),
    (uint8_t)((value / 10) % 10),
    (uint8_t)(value % 10)
  };

  uint32_t startedAt = millis();

  while (millis() - startedAt < holdMs) {
    for (uint8_t pos = 0; pos < 4; pos++) {
      refreshDigit(pos, displayDigits[pos]);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("NodeMCU ESP32-S 4-digit 7-segment count test");
  Serial.print("Segment pins SCLK/RCLK/DIO: GPIO");
  Serial.print(SEG_SCLK_PIN);
  Serial.print(" / GPIO");
  Serial.print(SEG_RCLK_PIN);
  Serial.print(" / GPIO");
  Serial.println(SEG_DIO_PIN);
  Serial.print("CAN TX/RX later: GPIO");
  Serial.print(CAN_TX_PIN);
  Serial.print(" / GPIO");
  Serial.println(CAN_RX_PIN);

  pinMode(SEG_SCLK_PIN, OUTPUT);
  pinMode(SEG_RCLK_PIN, OUTPUT);
  pinMode(SEG_DIO_PIN, OUTPUT);

  digitalWrite(SEG_SCLK_PIN, LOW);
  digitalWrite(SEG_RCLK_PIN, LOW);
  digitalWrite(SEG_DIO_PIN, LOW);

  clearDisplay();
}

void loop() {
  for (uint16_t value = 0; value <= 9999; value++) {
    Serial.print("[SEG] showing ");
    Serial.println(value);
    showNumberFor(value, COUNT_HOLD_MS);
  }
}
