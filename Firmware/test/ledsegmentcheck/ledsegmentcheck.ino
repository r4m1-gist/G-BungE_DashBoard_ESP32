/*
  NodeMCU ESP32-S LED segment check

  Checks three 4-digit 7-segment modules and basic CAN TX/RX.

  7-segment module pins:
    VCC, SCLK, RCLK, DIO, GND

  Pin map:
    VCC  -> ESP32 3V3 first, or 5V only if the module requires it
    SEG1 SCLK -> ESP32 GPIO21
    SEG1 RCLK -> ESP32 GPIO22
    SEG1 DIO  -> ESP32 GPIO17
    SEG2 SCLK -> ESP32 GPIO18
    SEG2 RCLK -> ESP32 GPIO19
    SEG2 DIO  -> ESP32 GPIO23
    SEG3 SCLK -> ESP32 GPIO13
    SEG3 RCLK -> ESP32 GPIO14
    SEG3 DIO  -> ESP32 GPIO25
    GND  -> ESP32 GND

  CAN transceiver:
    SN65HVD230 TXD / CTX -> ESP32 GPIO26
    SN65HVD230 RXD / CRX -> ESP32 GPIO27
    SN65HVD230 VCC       -> ESP32 3V3
    SN65HVD230 GND       -> ESP32 GND
    CANH-CANL            -> 120 ohm only at each end of the CAN bus
*/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "driver/twai.h"

struct SegmentPins {
  uint8_t sclk;
  uint8_t rclk;
  uint8_t dio;
};

static const SegmentPins SEG1 = {21, 22, 17};
static const SegmentPins SEG2 = {18, 19, 23};
static const SegmentPins SEG3 = {13, 14, 25};

static const uint8_t CAN_TX_PIN = 26;
static const uint8_t CAN_RX_PIN = 27;
static const uint32_t CAN_SEND_INTERVAL_MS = 1000;
static const uint32_t DISPLAY_REFRESH_INTERVAL_US = 1200;

// Change these if your module displays inverted/garbled output.
static const bool SEGMENT_ACTIVE_LOW = true;
static const bool DIGIT_ACTIVE_HIGH = true;
static const bool SEND_SEGMENTS_FIRST = true;

static const uint16_t COUNT_HOLD_MS = 100;
static const uint16_t DIGIT_REFRESH_US = 250;
static uint16_t displayValue = 0;
static uint8_t displayPosition = 0;
static uint32_t lastDisplayRefreshUs = 0;
static uint32_t lastCountMs = 0;
static uint32_t lastCanSendMs = 0;
static uint32_t canTxCounter = 0;
static bool canReady = false;

// Common segment bit order: bit0=a, bit1=b, ... bit6=g, bit7=dp.
static const uint8_t SEG_A  = 0b00000001;
static const uint8_t SEG_B  = 0b00000010;
static const uint8_t SEG_C  = 0b00000100;
static const uint8_t SEG_D  = 0b00001000;
static const uint8_t SEG_E  = 0b00010000;
static const uint8_t SEG_F  = 0b00100000;
static const uint8_t SEG_G  = 0b01000000;
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

void writeTwoBytes(const SegmentPins &pins, uint8_t segments, uint8_t digitSelect) {
  digitalWrite(pins.rclk, LOW);

  if (SEND_SEGMENTS_FIRST) {
    shiftOut(pins.dio, pins.sclk, MSBFIRST, normalizeSegments(segments));
    shiftOut(pins.dio, pins.sclk, MSBFIRST, normalizeDigitSelect(digitSelect));
  } else {
    shiftOut(pins.dio, pins.sclk, MSBFIRST, normalizeDigitSelect(digitSelect));
    shiftOut(pins.dio, pins.sclk, MSBFIRST, normalizeSegments(segments));
  }

  digitalWrite(pins.rclk, HIGH);
}

void clearDisplay(const SegmentPins &pins) {
  writeTwoBytes(pins, 0x00, 0x00);
}

void refreshDigit(const SegmentPins &pins, uint8_t position, uint8_t number) {
  writeTwoBytes(pins, DIGITS[number], DIGIT_SELECT[position]);
  delayMicroseconds(DIGIT_REFRESH_US);
  clearDisplay(pins);
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
      refreshDigit(SEG1, pos, displayDigits[pos]);
      refreshDigit(SEG2, pos, displayDigits[pos]);
      refreshDigit(SEG3, pos, displayDigits[pos]);
    }
  }
}

void lampTestFor(uint16_t holdMs) {
  uint32_t startedAt = millis();

  while (millis() - startedAt < holdMs) {
    for (uint8_t pos = 0; pos < 4; pos++) {
      refreshDigit(SEG1, pos, 8);
      refreshDigit(SEG2, pos, 8);
      refreshDigit(SEG3, pos, 8);
    }
  }
}

void segmentModuleTest() {
  Serial.println("[SEG1] lamp test only");
  uint32_t seg1StartedAt = millis();
  while (millis() - seg1StartedAt < 1500) {
    for (uint8_t pos = 0; pos < 4; pos++) {
      refreshDigit(SEG1, pos, 8);
    }
  }

  Serial.println("[SEG2] lamp test only");
  uint32_t seg2StartedAt = millis();
  while (millis() - seg2StartedAt < 1500) {
    for (uint8_t pos = 0; pos < 4; pos++) {
      refreshDigit(SEG2, pos, 8);
    }
  }

  Serial.println("[SEG3] lamp test only");
  uint32_t seg3StartedAt = millis();
  while (millis() - seg3StartedAt < 1500) {
    for (uint8_t pos = 0; pos < 4; pos++) {
      refreshDigit(SEG3, pos, 8);
    }
  }
}

bool startCan() {
  twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN,
    (gpio_num_t)CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );
  twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t installResult = twai_driver_install(&generalConfig, &timingConfig, &filterConfig);
  if (installResult != ESP_OK) {
    Serial.print("[CAN] driver install failed: ");
    Serial.println(installResult);
    return false;
  }

  esp_err_t startResult = twai_start();
  if (startResult != ESP_OK) {
    Serial.print("[CAN] start failed: ");
    Serial.println(startResult);
    twai_driver_uninstall();
    return false;
  }

  Serial.println("[CAN] started at 500 kbps");
  return true;
}

void sendCanTestFrame() {
  twai_message_t message = {};
  message.identifier = 0x123;
  message.data_length_code = 8;
  message.data[0] = 0x47; // G
  message.data[1] = 0x42; // B
  message.data[2] = (uint8_t)((canTxCounter >> 24) & 0xFF);
  message.data[3] = (uint8_t)((canTxCounter >> 16) & 0xFF);
  message.data[4] = (uint8_t)((canTxCounter >> 8) & 0xFF);
  message.data[5] = (uint8_t)(canTxCounter & 0xFF);
  message.data[6] = (uint8_t)(millis() & 0xFF);
  message.data[7] = 0xA5;

  esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
  if (result == ESP_OK) {
    Serial.print("[CAN TX] id=0x123 count=");
    Serial.println(canTxCounter);
    canTxCounter++;
  } else {
    Serial.print("[CAN TX] failed: ");
    Serial.println(result);
  }
}

void receiveCanFrames() {
  twai_message_t message = {};

  while (twai_receive(&message, 0) == ESP_OK) {
    Serial.print("[CAN RX] id=0x");
    Serial.print(message.identifier, HEX);
    Serial.print(" dlc=");
    Serial.print(message.data_length_code);
    Serial.print(" data=");

    for (uint8_t index = 0; index < message.data_length_code; index++) {
      if (message.data[index] < 0x10) {
        Serial.print("0");
      }
      Serial.print(message.data[index], HEX);
      Serial.print(index + 1 == message.data_length_code ? "" : " ");
    }

    Serial.println();
  }
}

void refreshDisplayOnce() {
  uint32_t nowUs = micros();
  if (nowUs - lastDisplayRefreshUs < DISPLAY_REFRESH_INTERVAL_US) {
    return;
  }

  lastDisplayRefreshUs = nowUs;

  uint8_t displayDigits[4] = {
    (uint8_t)((displayValue / 1000) % 10),
    (uint8_t)((displayValue / 100) % 10),
    (uint8_t)((displayValue / 10) % 10),
    (uint8_t)(displayValue % 10)
  };

  refreshDigit(SEG1, displayPosition, displayDigits[displayPosition]);
  refreshDigit(SEG2, displayPosition, displayDigits[displayPosition]);
  refreshDigit(SEG3, displayPosition, displayDigits[displayPosition]);
  displayPosition = (displayPosition + 1) % 4;
}

void setupSegmentPins(const SegmentPins &pins) {
  pinMode(pins.sclk, OUTPUT);
  pinMode(pins.rclk, OUTPUT);
  pinMode(pins.dio, OUTPUT);

  digitalWrite(pins.sclk, LOW);
  digitalWrite(pins.rclk, LOW);
  digitalWrite(pins.dio, LOW);

  clearDisplay(pins);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("NodeMCU ESP32-S LED segment check");
  Serial.print("SEG1 pins SCLK/RCLK/DIO: GPIO");
  Serial.print(SEG1.sclk);
  Serial.print(" / GPIO");
  Serial.print(SEG1.rclk);
  Serial.print(" / GPIO");
  Serial.println(SEG1.dio);
  Serial.print("SEG2 pins SCLK/RCLK/DIO: GPIO");
  Serial.print(SEG2.sclk);
  Serial.print(" / GPIO");
  Serial.print(SEG2.rclk);
  Serial.print(" / GPIO");
  Serial.println(SEG2.dio);
  Serial.print("SEG3 pins SCLK/RCLK/DIO: GPIO");
  Serial.print(SEG3.sclk);
  Serial.print(" / GPIO");
  Serial.print(SEG3.rclk);
  Serial.print(" / GPIO");
  Serial.println(SEG3.dio);
  Serial.print("CAN TX/RX: GPIO");
  Serial.print(CAN_TX_PIN);
  Serial.print(" / GPIO");
  Serial.println(CAN_RX_PIN);

  setupSegmentPins(SEG1);
  setupSegmentPins(SEG2);
  setupSegmentPins(SEG3);

  segmentModuleTest();
  Serial.println("[SEG1/SEG2/SEG3] lamp test together: 8888");
  lampTestFor(2000);

  canReady = startCan();
  Serial.println("[CAN] sending id 0x123 every 1 second");
}

void loop() {
  refreshDisplayOnce();

  uint32_t nowMs = millis();
  if (nowMs - lastCountMs >= COUNT_HOLD_MS) {
    lastCountMs = nowMs;
    displayValue = (displayValue + 1) % 10000;
  }

  if (!canReady) {
    return;
  }

  receiveCanFrames();

  if (nowMs - lastCanSendMs >= CAN_SEND_INTERVAL_MS) {
    lastCanSendMs = nowMs;
    sendCanTestFrame();
  }
}
