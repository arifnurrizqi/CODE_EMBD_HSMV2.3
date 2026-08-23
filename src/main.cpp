#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ModbusRTU.h>
#include <Wire.h>
#include "DFRobot_ADS1115.h"

DFRobot_ADS1115 ads(&Wire);

constexpr uint8_t FLOW_USAGE_PIN = 2;
constexpr uint8_t FLOW_SAMPLE_PIN = 3;
constexpr uint8_t RS485_RX_PIN = 9;
constexpr uint8_t RS485_TX_PIN = 8;
constexpr uint8_t DIP1 = 4, DIP2 = 5, DIP3 = 6, DIP4 = 7;
constexpr uint8_t PIN_5V = A7, PIN_VCC = A6;
constexpr uint8_t PH_ADC_CHANNEL = 0, TURBIDITY_ADC_CHANNEL = 1;

// ZJ-S201: frequency (Hz) = 7.5 * flow (L/min), sekitar 450 pulse/L.
constexpr float SAMPLE_FLOW_HZ_PER_LPM = 7.5F;
constexpr float PH_CURRENT_M = -0.010076F;
constexpr float PH_CURRENT_B = 36.7F;

// Regresi linear awal SEN0189: 4.1 V = 0 NTU, 2.5 V = 3000 NTU.
// Ganti dengan koefisien hasil kalibrasi lapangan.
constexpr float TURBIDITY_M = -1875.0F;
constexpr float TURBIDITY_B = 7687.5F;
constexpr unsigned long UPDATE_INTERVAL_MS = 1000UL;
constexpr unsigned long FLOW_TIMEOUT_MS = 10000UL;

SoftwareSerial rs485Serial(RS485_RX_PIN, RS485_TX_PIN);
ModbusRTU mb;
volatile uint32_t usagePulseTotal = 0;
volatile uint32_t samplePulseTotal = 0;
uint32_t lastUsagePulse = 0, lastSamplePulse = 0;
unsigned long lastUpdate = 0;
unsigned long lastSamplePulseTime = 0;
uint8_t slaveID = 1;

// Holding register, offset 0 = 40001.
enum {
  REG_SAMPLE_FLOW_RATE = 0, // 40001, L/min x100
  REG_USAGE_DELTA_PULSE,    // 40002
  REG_SAMPLE_DELTA_PULSE,   // 40003
  REG_USAGE_TOTAL_HI,       // 40004
  REG_USAGE_TOTAL_LO,       // 40005
  REG_PH_VALUE,             // 40006, pH x100
  REG_TURBIDITY,            // 40007, NTU x10
  REG_VCC_VOLTAGE,          // 40008, V x100
  REG_V5_VOLTAGE,           // 40009, V x100
  REG_SYSTEM_STATUS,        // 40010
  REG_SENSOR_STATUS,        // 40011
  REG_ERROR_CODE,           // 40012
  TOTAL_REGS
};

enum SensorStatusBit {
  STATUS_FLOW_INLET = 0,
  STATUS_TANK_LEVEL = 1,
  STATUS_FLOW_OUTLET = 2,
  STATUS_PH = 3,
  STATUS_TURBIDITY = 4,
  STATUS_VCC = 5,
  STATUS_5V = 6
};

enum SystemStatusValue {
  SYSTEM_NORMAL = 0,
  SYSTEM_WARNING = 1,
  SYSTEM_ALARM = 2,
  SYSTEM_SENSOR_ERROR = 3,
  SYSTEM_DATA_INVALID = 4
};

enum ErrorDetailValue {
  ERROR_NONE = 0,
  ERROR_FLOW_NO_PULSE = 1,
  ERROR_LEVEL_OUT_OF_RANGE = 2,
  ERROR_PH_DISCONNECTED = 3,
  ERROR_PH_OUT_OF_RANGE = 4,
  ERROR_TURBIDITY = 5,
  ERROR_ADC_FAILURE = 6,
  ERROR_SUPPLY_LOW = 7,
  ERROR_SUPPLY_HIGH = 8,
  ERROR_RS485_COMMUNICATION = 9,
  ERROR_SENSOR_TIMEOUT = 10
};

void usagePulseISR() { usagePulseTotal++; }
void samplePulseISR() { samplePulseTotal++; }

uint8_t readSlaveID() {
  uint8_t id = 0;
  id |= (!digitalRead(DIP1)) << 0;
  id |= (!digitalRead(DIP2)) << 1;
  id |= (!digitalRead(DIP3)) << 2;
  id |= (!digitalRead(DIP4)) << 3;
  return id == 0 ? 1 : id;
}

float calibrate(float input, float slope, float intercept) {
  return slope * input + intercept;
}

float readBoardVoltage(uint8_t pin, float rTop, float rBottom) {
  const float vOut = analogRead(pin) * (5.0F / 1024.0F);
  return vOut * (rTop + rBottom) / rBottom;
}

uint16_t toU16(float value) {
  if (value <= 0.0F) return 0;
  if (value >= 65535.0F) return 65535;
  return static_cast<uint16_t>(value + 0.5F);
}

void setStatusBit(uint16_t &status, uint8_t bit, bool value) {
  if (value) status |= (1U << bit);
  else status &= ~(1U << bit);
}

void setup() {
  Serial.begin(115200);

  pinMode(FLOW_USAGE_PIN, INPUT_PULLUP);
  pinMode(FLOW_SAMPLE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_USAGE_PIN), usagePulseISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(FLOW_SAMPLE_PIN), samplePulseISR, FALLING);

  pinMode(DIP1, INPUT_PULLUP);
  pinMode(DIP2, INPUT_PULLUP);
  pinMode(DIP3, INPUT_PULLUP);
  pinMode(DIP4, INPUT_PULLUP);
  delay(50);
  slaveID = readSlaveID();

  rs485Serial.begin(9600);
  mb.begin(&rs485Serial);
  mb.slave(slaveID);
  for (uint8_t i = 0; i < TOTAL_REGS; i++) mb.addHreg(i, 0);

  ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS1);
  ads.setGain(eGAIN_TWOTHIRDS);
  ads.setMode(eMODE_SINGLE);
  ads.setRate(eRATE_16);
  ads.setOSMode(eOSMODE_SINGLE);
  ads.init();

  lastUpdate = millis();
  lastSamplePulseTime = lastUpdate;
  Serial.println(F("SLAVE HSM V2.3 siap"));
  Serial.print(F("Modbus slave ID: "));
  Serial.println(slaveID);
  Serial.println(F("Usage D2, sample D3, RS485 RX D9 TX D8"));
}

void loop() {
  mb.task();

  const unsigned long now = millis();
  const unsigned long elapsedMs = now - lastUpdate;
  if (elapsedMs < UPDATE_INTERVAL_MS) return;
  lastUpdate = now;

  noInterrupts();
  const uint32_t usageTotal = usagePulseTotal;
  const uint32_t sampleTotal = samplePulseTotal;
  interrupts();

  const uint32_t usageDelta = usageTotal - lastUsagePulse;
  const uint32_t sampleDelta = sampleTotal - lastSamplePulse;
  lastUsagePulse = usageTotal;
  lastSamplePulse = sampleTotal;
  if (sampleDelta > 0) lastSamplePulseTime = now;

  const float sampleFrequencyHz = sampleDelta * (1000.0F / elapsedMs);
  const float sampleFlowLpm = sampleFrequencyHz / SAMPLE_FLOW_HZ_PER_LPM;

  float v5 = calibrate(readBoardVoltage(PIN_5V, 10000.0F, 10000.0F), 0.791F, 0.716F);
  float vcc = calibrate(readBoardVoltage(PIN_VCC, 100000.0F, 15000.0F), 0.829F, 3.19F);

  uint16_t phX100 = 0, turbidityX10 = 0;
  bool phConnected = false;
  bool phInRange = false;
  bool turbidityOK = false;
  const bool adsOK = ads.checkADS1115();
  if (adsOK) {
    const int16_t phMilliVolt = ads.readVoltage(PH_ADC_CHANNEL);
    const float rawCurrentMilliAmp = calibrate(phMilliVolt, PH_CURRENT_M, PH_CURRENT_B);
    phConnected = rawCurrentMilliAmp >= 3.5F;
    phInRange = rawCurrentMilliAmp >= 4.0F && rawCurrentMilliAmp <= 20.0F;
    float currentMilliAmp = rawCurrentMilliAmp;
    currentMilliAmp = constrain(currentMilliAmp, 4.0F, 20.0F);
    phX100 = toU16((currentMilliAmp - 4.0F) * (14.0F / 16.0F) * 100.0F);

    const int16_t turbidityMilliVolt = ads.readVoltage(TURBIDITY_ADC_CHANNEL);
    const float turbidityVolt = turbidityMilliVolt / 1000.0F;
    turbidityOK = turbidityVolt >= 0.05F && turbidityVolt <= 4.5F;
    float turbidityNtu = calibrate(turbidityMilliVolt / 1000.0F,
                                    TURBIDITY_M, TURBIDITY_B);
    turbidityNtu = constrain(turbidityNtu, 0.0F, 3000.0F);
    turbidityX10 = toU16(turbidityNtu * 10.0F);
  }

  const bool sampleFlowOK = (now - lastSamplePulseTime) < FLOW_TIMEOUT_MS;
  const bool vccLow = vcc <= 10.0F;
  const bool vccHigh = vcc >= 15.0F;
  const bool v5Low = v5 <= 4.7F;
  const bool v5High = v5 >= 5.3F;

  uint16_t sensorStatus = 0;
  setStatusBit(sensorStatus, STATUS_FLOW_INLET, true);
  setStatusBit(sensorStatus, STATUS_TANK_LEVEL, false); // tidak terpasang
  setStatusBit(sensorStatus, STATUS_FLOW_OUTLET, sampleFlowOK);
  setStatusBit(sensorStatus, STATUS_PH, adsOK && phConnected && phInRange);
  setStatusBit(sensorStatus, STATUS_TURBIDITY, adsOK && turbidityOK);
  setStatusBit(sensorStatus, STATUS_VCC, !vccLow && !vccHigh);
  setStatusBit(sensorStatus, STATUS_5V, !v5Low && !v5High);

  // Hanya satu error detail dikirim; urutan berikut adalah prioritasnya.
  uint16_t errorCode = ERROR_NONE;
  if (!adsOK) errorCode = ERROR_ADC_FAILURE;
  else if (!phConnected) errorCode = ERROR_PH_DISCONNECTED;
  else if (!phInRange) errorCode = ERROR_PH_OUT_OF_RANGE;
  else if (!turbidityOK) errorCode = ERROR_TURBIDITY;
  else if (!sampleFlowOK) errorCode = ERROR_FLOW_NO_PULSE;
  else if (vccLow || v5Low) errorCode = ERROR_SUPPLY_LOW;
  else if (vccHigh || v5High) errorCode = ERROR_SUPPLY_HIGH;

  uint16_t systemStatus = SYSTEM_NORMAL;
  if (errorCode == ERROR_SUPPLY_LOW) systemStatus = SYSTEM_WARNING;
  else if (errorCode == ERROR_SUPPLY_HIGH) systemStatus = SYSTEM_ALARM;
  else if (errorCode == ERROR_PH_OUT_OF_RANGE) systemStatus = SYSTEM_DATA_INVALID;
  else if (errorCode != ERROR_NONE) systemStatus = SYSTEM_SENSOR_ERROR;

  mb.Hreg(REG_SAMPLE_FLOW_RATE, toU16(sampleFlowLpm * 100.0F));
  mb.Hreg(REG_USAGE_DELTA_PULSE, usageDelta > 65535UL ? 65535 : usageDelta);
  mb.Hreg(REG_SAMPLE_DELTA_PULSE, sampleDelta > 65535UL ? 65535 : sampleDelta);
  mb.Hreg(REG_USAGE_TOTAL_HI, static_cast<uint16_t>(usageTotal >> 16));
  mb.Hreg(REG_USAGE_TOTAL_LO, static_cast<uint16_t>(usageTotal & 0xFFFF));
  mb.Hreg(REG_PH_VALUE, phX100);
  mb.Hreg(REG_TURBIDITY, turbidityX10);
  mb.Hreg(REG_VCC_VOLTAGE, toU16(vcc * 100.0F));
  mb.Hreg(REG_V5_VOLTAGE, toU16(v5 * 100.0F));
  mb.Hreg(REG_SYSTEM_STATUS, systemStatus);
  mb.Hreg(REG_SENSOR_STATUS, sensorStatus);
  mb.Hreg(REG_ERROR_CODE, errorCode);

  Serial.print(F("Usage total=")); Serial.print(usageTotal);
  Serial.print(F(" delta=")); Serial.print(usageDelta);
  Serial.print(F(" | Sample delta=")); Serial.print(sampleDelta);
  Serial.print(F(" flow=")); Serial.print(sampleFlowLpm, 2);
  Serial.print(F(" L/min | pH=")); Serial.print(phX100 / 100.0F, 2);
  Serial.print(F(" | Turbidity=")); Serial.print(turbidityX10 / 10.0F, 1);
  Serial.println(F(" NTU"));
}
