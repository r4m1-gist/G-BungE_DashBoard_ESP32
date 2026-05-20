/*
  G-BungE dashboard main sketch for Sevcon Gen4 Size 6

  Reads motor speed and motor temperature from the Sevcon over CANopen SDO.

  Display mapping:
    DISP1: motor speed, rpm, absolute value, 0..9999
    DISP2: motor temperature, degC, 0..9999

  Sevcon Gen4 manual notes:
    - Motor torque/speed are available in the 0x6000..0x67FF object range.
    - CiA 402 velocity actual value is commonly 0x606C:00.
    - Motor measurements including temperature are at object 0x4600.
    - The manual mentions motor temperature via 0x4600 sub 16 for CAN based thermistor use.

  If DVT maps these values to a TPDO later, this sketch can be changed to parse that
  PDO directly. SDO polling is intentionally used here so the dashboard can pull the
  values without depending on a preconfigured TPDO.
*/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "driver/twai.h"

struct SegmentPins {
  uint8_t sclk;
  uint8_t rclk;
  uint8_t dio;
};

struct CanOpenObject {
  uint16_t index;
  uint8_t subIndex;
};

enum class PendingRequest : uint8_t {
  NONE,
  SPEED,
  MOTOR_TEMP
};

static const SegmentPins DISP1 = {21, 22, 17};
static const SegmentPins DISP2 = {18, 19, 23};

static const uint8_t CAN_TX_PIN = 26;
static const uint8_t CAN_RX_PIN = 27;

static const uint8_t SEVCON_NODE_ID = 1;
static const uint32_t SDO_REQUEST_ID = 0x600 + SEVCON_NODE_ID;
static const uint32_t SDO_RESPONSE_ID = 0x580 + SEVCON_NODE_ID;

static const CanOpenObject MOTOR_SPEED_OBJECT = {0x606C, 0x00};
static const CanOpenObject MOTOR_TEMP_OBJECT = {0x4600, 0x16};

static const uint32_t SDO_POLL_INTERVAL_MS = 100;
static const uint32_t SDO_RESPONSE_TIMEOUT_MS = 250;
static const uint32_t VALUE_TIMEOUT_MS = 2000;
static const uint32_t DISPLAY_REFRESH_INTERVAL_US = 1200;

static const bool SEGMENT_ACTIVE_LOW = true;
static const bool DIGIT_ACTIVE_HIGH = true;
static const bool SEND_SEGMENTS_FIRST = true;
static const uint16_t DIGIT_REFRESH_US = 250;

static uint16_t speedDisplayValue = 0;
static uint16_t tempDisplayValue = 0;
static uint8_t displayPosition = 0;
static uint32_t lastDisplayRefreshUs = 0;
static uint32_t lastPollMs = 0;
static uint32_t requestSentAtMs = 0;
static uint32_t lastValueRxMs = 0;
static uint32_t lastHeartbeatMs = 0;
static uint32_t canFrameCount = 0;
static uint32_t sdoAbortCount = 0;
static int32_t motorSpeedRpm = 0;
static int32_t motorTempC = 0;
static bool canReady = false;
static bool valuesReceived = false;
static PendingRequest pendingRequest = PendingRequest::NONE;
static PendingRequest nextRequest = PendingRequest::SPEED;

static const uint8_t SEG_A = 0b00000001;
static const uint8_t SEG_B = 0b00000010;
static const uint8_t SEG_C = 0b00000100;
static const uint8_t SEG_D = 0b00001000;
static const uint8_t SEG_E = 0b00010000;
static const uint8_t SEG_F = 0b00100000;
static const uint8_t SEG_G = 0b01000000;

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

uint16_t clampDisplayValue(int32_t value) {
  if (value < 0) {
    value = -value;
  }
  if (value > 9999) {
    return 9999;
  }
  return (uint16_t)value;
}

void refreshDisplays() {
  uint32_t nowUs = micros();
  if (nowUs - lastDisplayRefreshUs < DISPLAY_REFRESH_INTERVAL_US) {
    return;
  }

  lastDisplayRefreshUs = nowUs;

  uint8_t speedDigits[4] = {
    (uint8_t)((speedDisplayValue / 1000) % 10),
    (uint8_t)((speedDisplayValue / 100) % 10),
    (uint8_t)((speedDisplayValue / 10) % 10),
    (uint8_t)(speedDisplayValue % 10)
  };
  uint8_t tempDigits[4] = {
    (uint8_t)((tempDisplayValue / 1000) % 10),
    (uint8_t)((tempDisplayValue / 100) % 10),
    (uint8_t)((tempDisplayValue / 10) % 10),
    (uint8_t)(tempDisplayValue % 10)
  };

  refreshDigit(DISP1, displayPosition, speedDigits[displayPosition]);
  refreshDigit(DISP2, displayPosition, tempDigits[displayPosition]);
  displayPosition = (displayPosition + 1) % 4;
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

void sendNmtOperational() {
  twai_message_t message = {};
  message.identifier = 0x000;
  message.data_length_code = 2;
  message.data[0] = 0x01;
  message.data[1] = SEVCON_NODE_ID;

  esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(20));
  Serial.print("[CANopen] NMT operational node=");
  Serial.print(SEVCON_NODE_ID);
  Serial.print(" result=");
  Serial.println(result);
}

CanOpenObject objectForRequest(PendingRequest request) {
  if (request == PendingRequest::MOTOR_TEMP) {
    return MOTOR_TEMP_OBJECT;
  }
  return MOTOR_SPEED_OBJECT;
}

const char *labelForRequest(PendingRequest request) {
  if (request == PendingRequest::MOTOR_TEMP) {
    return "motorTemp";
  }
  if (request == PendingRequest::SPEED) {
    return "speed";
  }
  return "none";
}

void advanceNextRequest(PendingRequest completedRequest) {
  nextRequest = (completedRequest == PendingRequest::SPEED) ? PendingRequest::MOTOR_TEMP : PendingRequest::SPEED;
}

bool sendSdoRead(PendingRequest request) {
  CanOpenObject object = objectForRequest(request);

  twai_message_t message = {};
  message.identifier = SDO_REQUEST_ID;
  message.data_length_code = 8;
  message.data[0] = 0x40;
  message.data[1] = object.index & 0xFF;
  message.data[2] = object.index >> 8;
  message.data[3] = object.subIndex;
  message.data[4] = 0x00;
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x00;

  esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(20));
  if (result != ESP_OK) {
    Serial.print("[SDO] request failed ");
    Serial.print(labelForRequest(request));
    Serial.print(" result=");
    Serial.println(result);
    return false;
  }

  pendingRequest = request;
  requestSentAtMs = millis();
  return true;
}

int32_t readSdoSignedValue(const twai_message_t &message) {
  switch (message.data[0]) {
    case 0x4F:
      return (int8_t)message.data[4];
    case 0x4B:
      return (int16_t)((uint16_t)message.data[4] | ((uint16_t)message.data[5] << 8));
    case 0x43:
      return (int32_t)((uint32_t)message.data[4] |
                       ((uint32_t)message.data[5] << 8) |
                       ((uint32_t)message.data[6] << 16) |
                       ((uint32_t)message.data[7] << 24));
    default:
      return 0;
  }
}

uint32_t readSdoAbortCode(const twai_message_t &message) {
  return (uint32_t)message.data[4] |
         ((uint32_t)message.data[5] << 8) |
         ((uint32_t)message.data[6] << 16) |
         ((uint32_t)message.data[7] << 24);
}

void handleSdoResponse(const twai_message_t &message) {
  if (message.data_length_code < 8) {
    return;
  }

  uint16_t index = (uint16_t)message.data[1] | ((uint16_t)message.data[2] << 8);
  uint8_t subIndex = message.data[3];
  CanOpenObject expected = objectForRequest(pendingRequest);

  if (pendingRequest == PendingRequest::NONE || index != expected.index || subIndex != expected.subIndex) {
    return;
  }

  if (message.data[0] == 0x80) {
    sdoAbortCount++;
    Serial.print("[SDO] abort ");
    Serial.print(labelForRequest(pendingRequest));
    Serial.print(" object=0x");
    Serial.print(index, HEX);
    Serial.print(":");
    Serial.print(subIndex, HEX);
    Serial.print(" code=0x");
    Serial.println(readSdoAbortCode(message), HEX);
    advanceNextRequest(pendingRequest);
    pendingRequest = PendingRequest::NONE;
    return;
  }

  if (message.data[0] != 0x4F && message.data[0] != 0x4B && message.data[0] != 0x43) {
    Serial.print("[SDO] unsupported response command=0x");
    Serial.println(message.data[0], HEX);
    pendingRequest = PendingRequest::NONE;
    return;
  }

  int32_t value = readSdoSignedValue(message);
  if (pendingRequest == PendingRequest::SPEED) {
    motorSpeedRpm = value;
    speedDisplayValue = clampDisplayValue(motorSpeedRpm);
    advanceNextRequest(pendingRequest);
  } else if (pendingRequest == PendingRequest::MOTOR_TEMP) {
    motorTempC = value;
    tempDisplayValue = clampDisplayValue(motorTempC);
    advanceNextRequest(pendingRequest);
  }

  valuesReceived = true;
  lastValueRxMs = millis();
  pendingRequest = PendingRequest::NONE;
}

void receiveCanFrames() {
  twai_message_t message = {};

  while (twai_receive(&message, 0) == ESP_OK) {
    canFrameCount++;

    if (!message.extd && message.identifier == SDO_RESPONSE_ID) {
      handleSdoResponse(message);
    }
  }
}

void pollSevconObjects() {
  uint32_t nowMs = millis();

  if (pendingRequest != PendingRequest::NONE) {
    if (nowMs - requestSentAtMs > SDO_RESPONSE_TIMEOUT_MS) {
      Serial.print("[SDO] timeout ");
      Serial.println(labelForRequest(pendingRequest));
      advanceNextRequest(pendingRequest);
      pendingRequest = PendingRequest::NONE;
    }
    return;
  }

  if (nowMs - lastPollMs < SDO_POLL_INTERVAL_MS) {
    return;
  }

  lastPollMs = nowMs;
  sendSdoRead(nextRequest);
}

void printCanStatus() {
  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) {
    return;
  }

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
  Serial.print(status.rx_error_counter);
  Serial.print(" txErr=");
  Serial.print(status.tx_error_counter);
  Serial.print(" rxQueue=");
  Serial.print(status.msgs_to_rx);
}

void setupDisplayPins() {
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
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("G-BungE dashboard Sevcon Gen4 main");
  Serial.print("Sevcon CANopen node ID: ");
  Serial.println(SEVCON_NODE_ID);
  Serial.print("CAN TX/RX: GPIO");
  Serial.print(CAN_TX_PIN);
  Serial.print(" / GPIO");
  Serial.println(CAN_RX_PIN);
  Serial.print("Speed object: 0x");
  Serial.print(MOTOR_SPEED_OBJECT.index, HEX);
  Serial.print(":");
  Serial.println(MOTOR_SPEED_OBJECT.subIndex, HEX);
  Serial.print("Motor temp object: 0x");
  Serial.print(MOTOR_TEMP_OBJECT.index, HEX);
  Serial.print(":");
  Serial.println(MOTOR_TEMP_OBJECT.subIndex, HEX);

  setupDisplayPins();
  canReady = startCan();

  if (canReady) {
    sendNmtOperational();
    Serial.println("[SDO] polling Sevcon speed and motor temperature");
  }
}

void loop() {
  refreshDisplays();

  if (!canReady) {
    speedDisplayValue = 0;
    tempDisplayValue = 0;
    return;
  }

  receiveCanFrames();
  pollSevconObjects();

  if (valuesReceived && millis() - lastValueRxMs > VALUE_TIMEOUT_MS) {
    Serial.println("[SDO] value timeout: no recent Sevcon object responses");
    valuesReceived = false;
  }

  if (millis() - lastHeartbeatMs >= 1000) {
    lastHeartbeatMs = millis();
    Serial.print("[CAN] frames=");
    Serial.print(canFrameCount);
    Serial.print(" aborts=");
    Serial.print(sdoAbortCount);
    Serial.print(" speed=");
    Serial.print(motorSpeedRpm);
    Serial.print("rpm temp=");
    Serial.print(motorTempC);
    Serial.print("C");
    printCanStatus();
    Serial.println();
  }
}
