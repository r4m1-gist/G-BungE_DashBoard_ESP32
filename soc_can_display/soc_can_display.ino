/*
  CAN SoC receiver for G-BungE dashboard

  Source ESP sends every 1 second:
    CAN ID 0x100
    byte 0~1: voltage
    byte 2~3: current
    byte 4:   SoC uint
    byte 5:   temperature uint, degC

  Dashboard:
    disp1 shows SoC from CAN byte 4.
    disp2 shows temperature from CAN byte 5.
*/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "driver/twai.h"

struct SegmentPins {
  uint8_t sclk;
  uint8_t rclk;
  uint8_t dio;
};

static const SegmentPins DISP1 = {21, 22, 17};
static const SegmentPins DISP2 = {18, 19, 23};

static const uint8_t CAN_TX_PIN = 26;
static const uint8_t CAN_RX_PIN = 27;
static const uint32_t BATTERY_CAN_ID = 0x100;
static const uint32_t SOC_TIMEOUT_MS = 3000;
static const uint32_t DISPLAY_REFRESH_INTERVAL_US = 1200;

static const bool SEGMENT_ACTIVE_LOW = true;
static const bool DIGIT_ACTIVE_HIGH = true;
static const bool SEND_SEGMENTS_FIRST = true;

static const uint16_t DIGIT_REFRESH_US = 250;

static uint16_t disp1Value = 0;
static uint16_t disp2Value = 0;
static uint8_t displayPosition = 0;
static uint32_t lastDisplayRefreshUs = 0;
static uint32_t lastSocRxMs = 0;
static uint32_t lastCanHeartbeatMs = 0;
static uint32_t canFrameCount = 0;
static uint32_t canTxErrorCount = 0;
static uint32_t canRxErrorCount = 0;
static bool canReady = false;
static bool socReceived = false;

static const uint8_t SEG_A  = 0b00000001;
static const uint8_t SEG_B  = 0b00000010;
static const uint8_t SEG_C  = 0b00000100;
static const uint8_t SEG_D  = 0b00001000;
static const uint8_t SEG_E  = 0b00010000;
static const uint8_t SEG_F  = 0b00100000;
static const uint8_t SEG_G  = 0b01000000;

static const uint8_t DIGITS[10] = {
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
  SEG_B | SEG_C,
  SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,
  SEG_B | SEG_C | SEG_F | SEG_G,
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
  SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
  SEG_A | SEG_B | SEG_C,
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G
};

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

bool startCan() {
  twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN,
    (gpio_num_t)CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );
  generalConfig.alerts_enabled = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_DATA;
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

void printCanStatus() {
  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) {
    return;
  }

  canTxErrorCount = status.tx_error_counter;
  canRxErrorCount = status.rx_error_counter;

  Serial.print(" state=");
  switch (status.state) {
    case TWAI_STATE_STOPPED:
      Serial.print("STOPPED");
      break;
    case TWAI_STATE_RUNNING:
      Serial.print("RUNNING");
      break;
    case TWAI_STATE_BUS_OFF:
      Serial.print("BUS_OFF");
      break;
    case TWAI_STATE_RECOVERING:
      Serial.print("RECOVERING");
      break;
    default:
      Serial.print("UNKNOWN");
      break;
  }

  Serial.print(" rxErr=");
  Serial.print(canRxErrorCount);
  Serial.print(" txErr=");
  Serial.print(canTxErrorCount);
  Serial.print(" rxQueue=");
  Serial.print(status.msgs_to_rx);
}

uint16_t readUint16LittleEndian(const uint8_t *data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

void printCanFrame(const twai_message_t &message) {
  Serial.print("[CAN RX RAW] id=0x");
  Serial.print(message.identifier, HEX);
  Serial.print(message.extd ? " ext" : " std");
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

void receiveBatteryFrame() {
  twai_message_t message = {};

  while (twai_receive(&message, 0) == ESP_OK) {
    canFrameCount++;
    printCanFrame(message);

    if (message.extd || message.identifier != BATTERY_CAN_ID || message.data_length_code < 6) {
      continue;
    }

    uint16_t voltage = readUint16LittleEndian(&message.data[0]);
    uint16_t current = readUint16LittleEndian(&message.data[2]);
    uint8_t soc = message.data[4];
    uint8_t temperature = message.data[5];

    disp1Value = soc;
    disp2Value = temperature;
    lastSocRxMs = millis();
    socReceived = true;

    Serial.print("[BATT] voltage=");
    Serial.print(voltage);
    Serial.print(" current=");
    Serial.print(current);
    Serial.print(" soc=");
    Serial.print(soc);
    Serial.print(" temp=");
    Serial.println(temperature);
    Serial.print("SoC = ");
    Serial.print(soc);
    Serial.println("%");
    Serial.print("Temp = ");
    Serial.print(temperature);
    Serial.println("C");
  }
}

void refreshDisp1() {
  uint32_t nowUs = micros();
  if (nowUs - lastDisplayRefreshUs < DISPLAY_REFRESH_INTERVAL_US) {
    return;
  }

  lastDisplayRefreshUs = nowUs;

  uint8_t disp1Digits[4] = {
    (uint8_t)((disp1Value / 1000) % 10),
    (uint8_t)((disp1Value / 100) % 10),
    (uint8_t)((disp1Value / 10) % 10),
    (uint8_t)(disp1Value % 10)
  };
  uint8_t disp2Digits[4] = {
    (uint8_t)((disp2Value / 1000) % 10),
    (uint8_t)((disp2Value / 100) % 10),
    (uint8_t)((disp2Value / 10) % 10),
    (uint8_t)(disp2Value % 10)
  };

  refreshDigit(DISP1, displayPosition, disp1Digits[displayPosition]);
  refreshDigit(DISP2, displayPosition, disp2Digits[displayPosition]);
  displayPosition = (displayPosition + 1) % 4;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("G-BungE dashboard SoC CAN receiver");
  Serial.print("DISP1 SCLK/RCLK/DIO: GPIO");
  Serial.print(DISP1.sclk);
  Serial.print(" / GPIO");
  Serial.print(DISP1.rclk);
  Serial.print(" / GPIO");
  Serial.println(DISP1.dio);
  Serial.print("DISP2 SCLK/RCLK/DIO: GPIO");
  Serial.print(DISP2.sclk);
  Serial.print(" / GPIO");
  Serial.print(DISP2.rclk);
  Serial.print(" / GPIO");
  Serial.println(DISP2.dio);
  Serial.print("CAN TX/RX: GPIO");
  Serial.print(CAN_TX_PIN);
  Serial.print(" / GPIO");
  Serial.println(CAN_RX_PIN);

  pinMode(DISP1.sclk, OUTPUT);
  pinMode(DISP1.rclk, OUTPUT);
  pinMode(DISP1.dio, OUTPUT);
  pinMode(DISP2.sclk, OUTPUT);
  pinMode(DISP2.rclk, OUTPUT);
  pinMode(DISP2.dio, OUTPUT);
  digitalWrite(DISP1.sclk, LOW);
  digitalWrite(DISP1.rclk, LOW);
  digitalWrite(DISP1.dio, LOW);
  digitalWrite(DISP2.sclk, LOW);
  digitalWrite(DISP2.rclk, LOW);
  digitalWrite(DISP2.dio, LOW);
  clearDisplay(DISP1);
  clearDisplay(DISP2);

  canReady = startCan();
  Serial.println("[CAN] waiting for ID 0x100, SoC at byte 4, temp at byte 5");
}

void loop() {
  refreshDisp1();

  if (!canReady) {
    disp1Value = 0;
    disp2Value = 0;
    return;
  }

  receiveBatteryFrame();

  if (millis() - lastCanHeartbeatMs >= 1000) {
    lastCanHeartbeatMs = millis();
    Serial.print("[CAN] frames received total=");
    Serial.print(canFrameCount);
    Serial.print(" waiting for std id 0x100 dlc>=6");
    printCanStatus();
    if (socReceived) {
      Serial.print(" last SoC=");
      Serial.print(disp1Value);
      Serial.print(" temp=");
      Serial.print(disp2Value);
    }
    Serial.println();
  }

  if (socReceived && millis() - lastSocRxMs > SOC_TIMEOUT_MS) {
    Serial.println("[BATT] timeout: no 0x100 frame for 3 seconds");
    socReceived = false;
  }
}
