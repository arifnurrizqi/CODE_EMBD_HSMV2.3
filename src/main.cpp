#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ModbusRTU.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Preferences.h>

// ==========================
// KONFIGURASI MODBUS RS485
// ==========================
#define W5500_CS   5
#define W5500_RST  4
#define LCD_CS     15

byte mac[6];
EthernetClient client;

enum LanStatus {
  LAN_INIT,
  LAN_HW_NOT_FOUND,
  LAN_NO_CABLE,
  LAN_WAIT_DHCP,
  LAN_CONNECTED,
  LAN_DHCP_FAILED
};

enum InternetStatus {
  NET_UNKNOWN,
  NET_CHECKING,
  NET_CONNECTED,
  NET_DISCONNECTED
};

LanStatus lanStatus = LAN_INIT;
InternetStatus internetStatus = NET_UNKNOWN;
bool ethernetReady = false;
bool ethernetHardwareDetected = false;
uint8_t dhcpMaintenanceFailures = 0;

unsigned long lastDHCPAttempt = 0;
unsigned long lastInternetCheck = 0;
unsigned long lastDhcpMaintain = 0;
unsigned long lastPrint = 0;

const uint32_t DHCP_RETRY_INTERVAL = 10000;
const uint32_t DHCP_MAINTAIN_INTERVAL = 1000;
const uint32_t INTERNET_CHECK_INTERVAL = 30000;
const uint32_t NETWORK_STATUS_PRINT_INTERVAL = 5000;
const uint32_t DHCP_TIMEOUT = 5000;
const uint32_t DHCP_RESPONSE_TIMEOUT = 1000;
const uint16_t TCP_CONNECTION_TIMEOUT = 3000;

void initMacAddress();
void attemptDhcp();

void startupTask();
void voltageTask();
void autoPageTask();
void ethernetTask();
void checkInternet();
const char* getLanStatus();
const char* getInternetStatus();
const char* getHsmStatus();
void printStatus();



// =========================
// LCD ST7920
// =========================
U8G2_ST7920_128X64_F_SW_SPI lcd(
  U8G2_R0,
  27,   // clock
  26,   // data
  15,   // cs
  U8X8_PIN_NONE
);

// ==========================
// KONFIGURASI MODBUS RS485
// ==========================
ModbusRTU mb;

#define RXD2 17   // RX ESP32 dari TX modul RS485
#define TXD2 16   // TX ESP32 ke RX modul RS485

HardwareSerial RS485Serial(2);

const uint16_t HSM_START_ADDR = 0;   // offset 0 = 40001
enum SlaveRegister : uint8_t {
  REG_SAMPLE_FLOW = 0, REG_USAGE_DELTA, REG_SAMPLE_DELTA,
  REG_USAGE_TOTAL_HI, REG_USAGE_TOTAL_LO, REG_PH, REG_TURBIDITY,
  REG_VCC, REG_5V, REG_SYSTEM_STATUS, REG_SENSOR_STATUS,
  REG_ERROR_DETAIL, NUM_REGS
};

enum SensorStatusMask : uint16_t {
  SENSOR_FLOW_USAGE_OK = 0x0001, SENSOR_FLOW_SAMPLE_OK = 0x0004,
  SENSOR_PH_OK = 0x0008, SENSOR_TURBIDITY_OK = 0x0010,
  SENSOR_VCC_OK = 0x0020, SENSOR_5V_OK = 0x0040
};

// Buffer hasil baca register
uint16_t regData[NUM_REGS];

uint16_t ultrasonicReg = 250;
float ultrasonicDistanceCm = 25.0f;
bool ultrasonicStatus = false;
bool ultrasonicHasData = false;
bool ultrasonicAttempted = false;
unsigned long lastUltrasonicSuccess = 0;

enum PollTarget : uint8_t {
  POLL_HSM,
  POLL_ULTRASONIC
};

PollTarget pollTarget = POLL_HSM;
unsigned long lastPoll = 0;
unsigned long lastPageChange = 0;
unsigned long manualPageHoldUntil = 0;
const uint32_t MANUAL_PAGE_HOLD_MS = 20000;

struct DeviceConfig {
  uint8_t hsmSlaveId = 9;
  uint8_t ultrasonicSlaveId = 1;
  uint16_t ultrasonicAddress = 257;
  uint16_t ultrasonicScale = 10;
  uint32_t pollIntervalMs = 2000;
  bool autoPageEnabled = true;
  uint32_t autoPageIntervalMs = 5000;
};

Preferences preferences;
DeviceConfig config;
DeviceConfig editConfig;

void loadConfig() {
  preferences.begin("hydroflow", true);
  config.hsmSlaveId = preferences.getUChar("hsm_id", 9);
  config.ultrasonicSlaveId = preferences.getUChar("us_id", 1);
  config.ultrasonicAddress = preferences.getUShort("us_addr", 257);
  config.ultrasonicScale = preferences.getUShort("us_scale", 10);
  config.pollIntervalMs = preferences.getULong("poll_ms", 2000);
  config.autoPageEnabled = preferences.getBool("auto_page", true);
  config.autoPageIntervalMs = preferences.getULong("page_ms", 5000);
  preferences.end();

  if (config.hsmSlaveId < 1 || config.hsmSlaveId > 247) config.hsmSlaveId = 9;
  if (config.ultrasonicSlaveId < 1 || config.ultrasonicSlaveId > 247) config.ultrasonicSlaveId = 1;
  if (config.ultrasonicScale < 1 || config.ultrasonicScale > 1000) config.ultrasonicScale = 10;
  if (config.pollIntervalMs < 250 || config.pollIntervalMs > 60000) config.pollIntervalMs = 2000;
  if (config.autoPageIntervalMs < 2000 || config.autoPageIntervalMs > 60000) config.autoPageIntervalMs = 5000;
}

void saveConfig() {
  config = editConfig;
  preferences.begin("hydroflow", false);
  preferences.putUChar("hsm_id", config.hsmSlaveId);
  preferences.putUChar("us_id", config.ultrasonicSlaveId);
  preferences.putUShort("us_addr", config.ultrasonicAddress);
  preferences.putUShort("us_scale", config.ultrasonicScale);
  preferences.putULong("poll_ms", config.pollIntervalMs);
  preferences.putBool("auto_page", config.autoPageEnabled);
  preferences.putULong("page_ms", config.autoPageIntervalMs);
  preferences.end();
  pollTarget = POLL_HSM;
  lastPoll = millis() - config.pollIntervalMs;
  lastPageChange = millis();
}

// Status polling
bool mbBusy = false;
// Interval polling disimpan pada config.pollIntervalMs.

// =========================
// Tombol I2C PCF8574
// =========================
#define PCF8574_ADDR 0x20  // sesuaikan dengan A0-A2 (biasanya 0x20)

#define BTN_MENU  1
#define BTN_UP    2
#define BTN_DOWN  3
#define BTN_OK    4

byte lastState = 0xFF;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 150;

byte readPCF8574() {
  Wire.requestFrom(PCF8574_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

bool isPressed(byte state, byte pin) {
  return !(state & (1 << pin)); // aktif LOW
}

// =========================
// CONFIG. BOARD VOLTAGE SENSOR
// =========================
#define PIN_3V3 34
#define PIN_5V 36
#define PIN_VCC 39

// 3v3
float R1 = 10000.0;
float R2 = 10000.0;
// 5v
float R4 = 10000.0;
float R5 = 10000.0;
// vcc 🔥 Ganti jadi: Misal target max 30V: R6 = 47k ; R7 = 6.8k
float R6 = 100000.0;
float R7 = 15000.0;

float vin_3v3;
float vin_5v;
float vin_vcc;

// =========================
// Menu
// =========================
enum PageMenu {
  PAGE_HOME = 0,
  PAGE_SENSOR,
  PAGE_ULTRASONIC,
  PAGE_ALARM_1,
  PAGE_ALARM_2,
  PAGE_VOLTAGE,
  PAGE_COMM,
  PAGE_SYSTEM,
  PAGE_TOTAL
};

uint8_t currentPage = PAGE_HOME;

enum UiMode : uint8_t {
  UI_NORMAL,
  UI_MENU_LIST,
  UI_MENU_EDIT
};

enum MenuItem : uint8_t {
  MENU_HSM_ID,
  MENU_ULTRASONIC_ID,
  MENU_ULTRASONIC_ADDR,
  MENU_ULTRASONIC_SCALE,
  MENU_POLL_INTERVAL,
  MENU_AUTO_PAGE,
  MENU_AUTO_PAGE_INTERVAL,
  MENU_SAVE_EXIT,
  MENU_CANCEL,
  MENU_ITEM_COUNT
};

UiMode uiMode = UI_NORMAL;
uint8_t selectedMenuItem = MENU_HSM_ID;
bool menuIdConflict = false;

// =========================
// Data dari slave Modbus (holding register 40001-40012)
// =========================
float flowRateSample = 0;
uint16_t usagePulseDelta = 0;
uint16_t samplePulseDelta = 0;
uint32_t usagePulseTotal = 0;
float phValue = 0;
float turbidity = 0;
float slaveVcc = 0;
float slave5V = 0;
uint16_t systemStatus = 0;
uint16_t sensorStatus = 0;
uint16_t errorDetail = 0;

bool alarmPh = false;
bool alarmTurbidity = false;
bool alarmFlow = false;
bool alarmSupply = false;
bool rs485Status = false;
bool hsmHasData = false;
bool hsmAttempted = false;
unsigned long lastHsmSuccess = 0;

bool alarmFlowUsage = false;
bool alarmFlowSample = false;
// bool lanStatus = true;

// int serverStatus = -65;
unsigned long uptimeHour = 0;

unsigned long lastVoltageRead = 0;
const uint32_t VOLTAGE_READ_INTERVAL = 1000;

enum StartupState : uint8_t { START_RESET_LOW, START_RESET_WAIT, START_READY };
StartupState startupState = START_RESET_LOW;
unsigned long startupTimer = 0;
bool displayReady = false;
bool networkHardwareReady = false;

// ==========================
// CALLBACK HASIL BACA
// ==========================
bool cbRead(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  mbBusy = false;

  if (event == Modbus::EX_SUCCESS) {
    Serial.println("===== DATA DARI SLAVE =====");

    flowRateSample = regData[REG_SAMPLE_FLOW] / 100.0f;
    usagePulseDelta = regData[REG_USAGE_DELTA];
    samplePulseDelta = regData[REG_SAMPLE_DELTA];
    usagePulseTotal = ((uint32_t)regData[REG_USAGE_TOTAL_HI] << 16) | regData[REG_USAGE_TOTAL_LO];
    phValue = regData[REG_PH] / 100.0f;
    turbidity = regData[REG_TURBIDITY] / 10.0f;
    slaveVcc = regData[REG_VCC] / 100.0f;
    slave5V = regData[REG_5V] / 100.0f;
    systemStatus = regData[REG_SYSTEM_STATUS];
    sensorStatus = regData[REG_SENSOR_STATUS];
    errorDetail = regData[REG_ERROR_DETAIL];

    alarmFlowUsage = !(sensorStatus & SENSOR_FLOW_USAGE_OK);
    alarmFlowSample = !(sensorStatus & SENSOR_FLOW_SAMPLE_OK);
    alarmFlow = alarmFlowUsage || alarmFlowSample;
    alarmPh = !(sensorStatus & SENSOR_PH_OK);
    alarmTurbidity = !(sensorStatus & SENSOR_TURBIDITY_OK);
    alarmSupply = !(sensorStatus & SENSOR_VCC_OK) || !(sensorStatus & SENSOR_5V_OK);

    rs485Status = true;
    hsmHasData = true;
    hsmAttempted = true;
    lastHsmSuccess = millis();

  } else {
    rs485Status = false;
    hsmAttempted = true;
    Serial.print("Gagal baca Modbus. ResultCode: ");
    Serial.println((int)event);
  }

  return true;
}

// ==========================
// READ BOARD VOLTAGE SENSOR
// ==========================
bool cbUltrasonic(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  mbBusy = false;

  if (event == Modbus::EX_SUCCESS) {
    ultrasonicDistanceCm = ultrasonicReg / (float)config.ultrasonicScale;
    ultrasonicStatus = true;
    ultrasonicHasData = true;
    ultrasonicAttempted = true;
    lastUltrasonicSuccess = millis();
    Serial.println("===== DATA ULTRASONIC =====");
    Serial.print("Raw      : ");
    Serial.println(ultrasonicReg);
    Serial.print("Distance : ");
    Serial.print(ultrasonicDistanceCm, 1);
    Serial.println(" cm");
  } else {
    ultrasonicStatus = false;
    ultrasonicAttempted = true;
    Serial.print("Gagal baca ultrasonic. ResultCode: ");
    Serial.println((int)event);
  }

  return true;
}
void readBoardVoltage(){
  float vout_3v3 = analogRead(PIN_3V3) * (3.3 / 4095.0);
  vin_3v3  = 1.17 * (vout_3v3 * (R1 + R2) / R2);

  float vout_5v = analogRead(PIN_5V) * (3.3 / 4095.0);
  vin_5v  = 0.941 * (vout_5v * (R4 + R5) / R5) + 0.716;

  float vout_vcc = analogRead(PIN_VCC) * (3.3 / 4095.0);
  vin_vcc  = 0.841 * (vout_vcc * (R6 + R7) / R7) + 3.19;

}
// =========================
// Debounce tombol
// =========================
bool lastUpState = HIGH;
bool lastDownState = HIGH;
bool lastOkState = HIGH;

// =========================
// Helper
// =========================
bool buttonPressed(uint8_t pin, bool &lastState) {
  bool reading = digitalRead(pin);

  if (lastState == HIGH && reading == LOW && (millis() - lastDebounceTime > debounceDelay)) {
    lastDebounceTime = millis();
    lastState = reading;
    return true;
  }

  lastState = reading;
  return false;
}

int getTotalAlarm() {
  return alarmPh + alarmTurbidity + alarmFlowUsage + alarmFlowSample + alarmSupply;
}


void drawHeader(const char* title) {
  lcd.setFont(u8g2_font_5x7_tf);

  // Header background (hitam)
  lcd.drawBox(0, 0, 128, 12);

  // Title (putih)
  lcd.setDrawColor(0);
  lcd.drawStr(2, 9, title);

  // ===== PAGE INFO =====
  char pageInfo[16];
  sprintf(pageInfo, "PAGE: %d/%d", currentPage + 1, PAGE_TOTAL);

  lcd.setFont(u8g2_font_4x6_tf);

  int textWidth = lcd.getStrWidth(pageInfo);
  int textHeight = 8;

  int padding = 1;

  int boxW = textWidth + (padding * 2);
  int boxH = textHeight;

  int boxX = 128 - boxW - 2;
  int boxY = 2;

  // "clear area" (kotak hitam lagi supaya rapi)
  lcd.setDrawColor(0);
  lcd.drawBox(boxX, boxY, boxW, boxH);

  // teks putih di atas kotak hitam
  lcd.setDrawColor(1);
  lcd.drawStr(boxX + padding, boxY + textHeight + padding - 2, pageInfo);
}

void drawFooter() {
  lcd.drawLine(0, 54, 127, 54);
  lcd.setFont(u8g2_font_5x7_tf);
  lcd.drawStr(0, 63, "UP/DN:MOVE");

  char buf[18];
  const char* hsmComm = getHsmStatus();
  if (strcmp(hsmComm, "TIMEOUT") == 0 || strcmp(hsmComm, "STALE") == 0) {
    snprintf(buf, sizeof(buf), "ALM COUNT:COM");
  } else {
    int totalAlarm = getTotalAlarm();
    if (totalAlarm == 0) snprintf(buf, sizeof(buf), "ALM COUNT:--");
    else snprintf(buf, sizeof(buf), "ALM COUNT:%d", totalAlarm);
  }

  int textWidth = lcd.getStrWidth(buf);
  lcd.drawStr(126 - textWidth, 63, buf);
}

const char* getDataStatus(bool hasData, bool attempted, bool connected,
                          unsigned long lastSuccess) {
  if (!hasData) return attempted ? "TIMEOUT" : "WAITING";
  if (!connected) return "TIMEOUT";
  uint32_t staleWindow = config.pollIntervalMs * 4U;
  uint32_t staleLimit = staleWindow > 5000U ? staleWindow : 5000U;
  return millis() - lastSuccess > staleLimit ? "STALE" : "CONNECTED";
}

const char* getHsmStatus() {
  return getDataStatus(hsmHasData, hsmAttempted, rs485Status, lastHsmSuccess);
}

const char* getUltrasonicStatus() {
  return getDataStatus(ultrasonicHasData, ultrasonicAttempted,
                       ultrasonicStatus, lastUltrasonicSuccess);
}

const char* getErrorText(uint16_t code) {
  switch (code) {
    case 0: return "NO ERROR";
    case 1: return "NO FLOW";
    case 2: return "LEVEL RANGE";
    case 3: return "PH DC";
    case 4: return "PH RANGE";
    case 5: return "TURB ERROR";
    case 6: return "ADC STATUS";
    case 7: return "SUPPLY LOW";
    case 8: return "SUPPLY HIGH";
    case 9: return "RS485 ERROR";
    case 10: return "SENSOR TO";
    default: return "UNKNOWN";
  }
}
void drawHomePage() {
  char buf[32];
  drawHeader("HYDROFLOW V2.3");
  lcd.setFont(u8g2_font_5x7_tf);
  if (hsmHasData) {
    snprintf(buf, sizeof(buf), "Flow  : %.2f L/min", flowRateSample); lcd.drawStr(2, 20, buf);
    snprintf(buf, sizeof(buf), "pH    : %.2f", phValue); lcd.drawStr(2, 30, buf);
    snprintf(buf, sizeof(buf), "Turb  : %.1f NTU", turbidity); lcd.drawStr(2, 40, buf);
  } else {
    lcd.drawStr(2, 20, "Flow  : --.-- L/min");
    lcd.drawStr(2, 30, "pH    : --.--");
    lcd.drawStr(2, 40, "Turb  : --.- NTU");
  }
  if (ultrasonicHasData) snprintf(buf, sizeof(buf), "Level : %.1f cm", ultrasonicDistanceCm);
  else snprintf(buf, sizeof(buf), "Level : --.- cm");
  lcd.drawStr(2, 50, buf);
  drawFooter();
}

void drawSensorPage() {
  char buf[32];
  drawHeader("DATA SENSOR RAW");
  lcd.setFont(u8g2_font_5x7_tf);
  snprintf(buf, sizeof(buf), "Flow smp  : %.2f L/m", flowRateSample); lcd.drawStr(2, 22, buf);
  snprintf(buf, sizeof(buf), "Delta smp : %u puls", samplePulseDelta); lcd.drawStr(2, 32, buf);
  snprintf(buf, sizeof(buf), "Delta use : %u puls", usagePulseDelta); lcd.drawStr(2, 42, buf);
  snprintf(buf, sizeof(buf), "Total     : %lu puls", (unsigned long)usagePulseTotal); lcd.drawStr(2, 52, buf);
  snprintf(buf, sizeof(buf), "pH/Turb   : %.2f/%.1f", phValue, turbidity); lcd.drawStr(2, 62, buf);
}

void drawUltrasonicPage() {
  char buf[32];
  drawHeader("ULTRASONIC (US)");
  lcd.setFont(u8g2_font_5x7_tf);

  if (ultrasonicHasData) {
    snprintf(buf, sizeof(buf), "Distance : %.1f cm", ultrasonicDistanceCm);
    lcd.drawStr(2, 22, buf);
    snprintf(buf, sizeof(buf), "Raw      : %u", ultrasonicReg);
    lcd.drawStr(2, 32, buf);
  } else {
    lcd.drawStr(2, 22, "Distance : --.- cm");
    lcd.drawStr(2, 32, "Raw      : -----");
  }
  snprintf(buf, sizeof(buf), "Status   : %s", getUltrasonicStatus());
  lcd.drawStr(2, 42, buf);
  snprintf(buf, sizeof(buf), "Slave ID : %u", config.ultrasonicSlaveId);
  lcd.drawStr(2, 52, buf);
  snprintf(buf, sizeof(buf), "Register : %u", config.ultrasonicAddress);
  lcd.drawStr(2, 62, buf);
}

void drawErrorCodeRow(uint8_t row, uint8_t code) {
  char buf[32];
  const char* state = hsmHasData ? (errorDetail == code ? "ACTIVE" : "NORMAL") : "NO DATA";
  snprintf(buf, sizeof(buf), "%02u %-12s: %s", code, getErrorText(code), state);
  lcd.drawStr(2, 22 + row * 10, buf);
}

void drawAlarmPage1() {
  drawHeader("ALARM CODE 1-5");
  lcd.setFont(u8g2_font_5x7_tf);
  for (uint8_t code = 1; code <= 5; code++) {
    drawErrorCodeRow(code - 1, code);
  }
}

void drawAlarmPage2() {
  drawHeader("ALARM CODE 6-10");
  lcd.setFont(u8g2_font_5x7_tf);
  for (uint8_t code = 6; code <= 10; code++) {
    drawErrorCodeRow(code - 6, code);
  }
}

void drawVoltagePage() {
  char buf[32];
  drawHeader("VOLTAGE SYSTEM");
  lcd.setFont(u8g2_font_5x7_tf);
  snprintf(buf, sizeof(buf), "VCC Mstr  : %.2f V", vin_vcc); lcd.drawStr(2, 22, buf);
  if (hsmHasData) snprintf(buf, sizeof(buf), "VCC Slv   : %.2f V", slaveVcc);
  else snprintf(buf, sizeof(buf), "VCC Slv   : --.-- V");
  lcd.drawStr(2, 32, buf);
  snprintf(buf, sizeof(buf), "5V Mstr   : %.2f V", vin_5v); lcd.drawStr(2, 42, buf);
  if (hsmHasData) snprintf(buf, sizeof(buf), "5V Slv    : %.2f V", slave5V);
  else snprintf(buf, sizeof(buf), "5V Slv    : --.-- V");
  lcd.drawStr(2, 52, buf);
  snprintf(buf, sizeof(buf), "3.3V Mstr : %.2f V", vin_3v3); lcd.drawStr(2, 62, buf);
}

void drawCommPage() {
  char buf[32];
  drawHeader("KOMUNIKASI");
  lcd.setFont(u8g2_font_5x7_tf);
  snprintf(buf, sizeof(buf), "HSM ID %02u  : %s", config.hsmSlaveId, getHsmStatus()); lcd.drawStr(2, 22, buf);
  snprintf(buf, sizeof(buf), "US  ID %02u  : %s", config.ultrasonicSlaveId, getUltrasonicStatus()); lcd.drawStr(2, 32, buf);
  snprintf(buf, sizeof(buf), "LAN STATUS : %s", getLanStatus()); lcd.drawStr(2, 42, buf);
  snprintf(buf, sizeof(buf), "Internet   : %s", getInternetStatus()); lcd.drawStr(2, 52, buf);
  snprintf(buf, sizeof(buf), "IP Addr : %u.%u.%u.%u", Ethernet.localIP()[0], Ethernet.localIP()[1], Ethernet.localIP()[2], Ethernet.localIP()[3]);
  lcd.drawStr(2, 62, ethernetReady ? buf : "IP Addr : ---.---.---.---");
}

void drawSystemPage() {
  char buf[32];

  drawHeader("INFO SISTEM");
  lcd.setFont(u8g2_font_5x7_tf);

  lcd.drawStr(2, 22, "Device : IoT Water Monit.");
  lcd.drawStr(2, 32, "NAME   : HYDROFLOW V2.3");
  lcd.drawStr(2, 42, "BY     : ARNUR TECH");
  lcd.drawStr(2, 52, "FW Ver : v1.0");

  snprintf(buf, sizeof(buf), "Uptime : %lu Hours", uptimeHour);
  lcd.drawStr(2, 62, buf);
}

const char* getMenuLabel(uint8_t item) {
  switch (item) {
    case MENU_HSM_ID: return "HSM ID";
    case MENU_ULTRASONIC_ID: return "US ID";
    case MENU_ULTRASONIC_ADDR: return "US ADDR";
    case MENU_ULTRASONIC_SCALE: return "US SCALE";
    case MENU_POLL_INTERVAL: return "POLL MS";
    case MENU_AUTO_PAGE: return "AUTO PAGE";
    case MENU_AUTO_PAGE_INTERVAL: return "PAGE TIME";
    case MENU_SAVE_EXIT: return "SAVE & EXIT";
    case MENU_CANCEL: return "CANCEL";
    default: return "";
  }
}

void formatMenuItem(uint8_t item, char* buf, size_t size) {
  switch (item) {
    case MENU_HSM_ID:
      snprintf(buf, size, "%s: %u", getMenuLabel(item), editConfig.hsmSlaveId);
      break;
    case MENU_ULTRASONIC_ID:
      snprintf(buf, size, "%s: %u", getMenuLabel(item), editConfig.ultrasonicSlaveId);
      break;
    case MENU_ULTRASONIC_ADDR:
      snprintf(buf, size, "%s: %u", getMenuLabel(item), editConfig.ultrasonicAddress);
      break;
    case MENU_ULTRASONIC_SCALE:
      snprintf(buf, size, "%s: /%u", getMenuLabel(item), editConfig.ultrasonicScale);
      break;
    case MENU_POLL_INTERVAL:
      snprintf(buf, size, "%s: %lu", getMenuLabel(item), (unsigned long)editConfig.pollIntervalMs);
      break;
    case MENU_AUTO_PAGE:
      snprintf(buf, size, "%s: %s", getMenuLabel(item), editConfig.autoPageEnabled ? "ON" : "OFF");
      break;
    case MENU_AUTO_PAGE_INTERVAL:
      snprintf(buf, size, "%s: %lu s", getMenuLabel(item), (unsigned long)(editConfig.autoPageIntervalMs / 1000));
      break;
    default:
      snprintf(buf, size, "%s", getMenuLabel(item));
      break;
  }
}

void drawConfigMenu() {
  constexpr uint8_t VISIBLE_ITEMS = 4;
  char buf[28];

  lcd.setFont(u8g2_font_5x7_tf);
  lcd.setDrawColor(1);
  lcd.drawBox(0, 0, 128, 10);
  lcd.setDrawColor(0);
  lcd.drawStr(3, 8, uiMode == UI_MENU_EDIT ? "EDIT CONFIG" : "MENU CONFIG");
  lcd.setDrawColor(1);

  if (uiMode == UI_MENU_EDIT) {
    formatMenuItem(selectedMenuItem, buf, sizeof(buf));
    lcd.drawStr(3, 22, getMenuLabel(selectedMenuItem));
    lcd.drawFrame(2, 27, 124, 15);

    int valueWidth = lcd.getStrWidth(buf);
    int valueX = max(4, (128 - valueWidth) / 2);
    lcd.drawStr(valueX, 38, buf);

    if (menuIdConflict) {
      lcd.drawStr(3, 51, "ID HSM DAN US HARUS BEDA");
    } else {
      lcd.drawStr(3, 51, "UP/DOWN : UBAH NILAI");
    }
    lcd.drawStr(3, 62, "OK:SELESAI MENU:KEMBALI");
    return;
  }

  uint8_t firstVisible = 0;
  if (selectedMenuItem >= VISIBLE_ITEMS) {
    firstVisible = selectedMenuItem - VISIBLE_ITEMS + 1;
  }

  for (uint8_t row = 0; row < VISIBLE_ITEMS; row++) {
    uint8_t item = firstVisible + row;
    if (item >= MENU_ITEM_COUNT) break;

    int y = 12 + row * 10;
    formatMenuItem(item, buf, sizeof(buf));

    if (item == selectedMenuItem) {
      lcd.drawBox(1, y, 118, 9);
      lcd.setDrawColor(0);
      lcd.drawStr(4, y + 7, buf);
      lcd.setDrawColor(1);
    } else {
      lcd.drawStr(4, y + 7, buf);
    }
  }

  // Scrollbar: ukuran dan posisi thumb mengikuti bagian daftar yang terlihat.
  constexpr uint8_t SCROLL_Y = 12;
  constexpr uint8_t SCROLL_H = 40;
  const uint8_t thumbHeight = max(6, (SCROLL_H * VISIBLE_ITEMS) / MENU_ITEM_COUNT);
  const uint8_t maxFirstVisible = MENU_ITEM_COUNT - VISIBLE_ITEMS;
  const uint8_t thumbTravel = SCROLL_H - thumbHeight - 2;
  const uint8_t thumbY = SCROLL_Y + 1 +
                        (maxFirstVisible > 0
                            ? (thumbTravel * firstVisible) / maxFirstVisible
                            : 0);

  lcd.drawFrame(121, SCROLL_Y, 6, SCROLL_H);
  lcd.drawBox(122, thumbY, 4, thumbHeight);

  lcd.drawHLine(0, 53, 128);
  if (menuIdConflict) {
    lcd.drawStr(3, 63, "ERROR: ID HARUS BERBEDA");
  } else {
    lcd.drawStr(3, 63, "OK:PILIH MENU:BATAL");
  }
}

void adjustMenuValue(int8_t direction) {
  menuIdConflict = false;
  switch (selectedMenuItem) {
    case MENU_HSM_ID:
      editConfig.hsmSlaveId = constrain((int)editConfig.hsmSlaveId + direction, 1, 247);
      break;
    case MENU_ULTRASONIC_ID:
      editConfig.ultrasonicSlaveId = constrain((int)editConfig.ultrasonicSlaveId + direction, 1, 247);
      break;
    case MENU_ULTRASONIC_ADDR:
      editConfig.ultrasonicAddress = constrain((int32_t)editConfig.ultrasonicAddress + direction, 0L, 65535L);
      break;
    case MENU_ULTRASONIC_SCALE:
      editConfig.ultrasonicScale = constrain((int)editConfig.ultrasonicScale + direction, 1, 1000);
      break;
    case MENU_POLL_INTERVAL:
      editConfig.pollIntervalMs = constrain((int32_t)editConfig.pollIntervalMs + direction * 250L, 250L, 60000L);
      break;
    case MENU_AUTO_PAGE:
      editConfig.autoPageEnabled = !editConfig.autoPageEnabled;
      break;
    case MENU_AUTO_PAGE_INTERVAL:
      editConfig.autoPageIntervalMs = constrain((int32_t)editConfig.autoPageIntervalMs + direction * 1000L, 2000L, 60000L);
      break;
  }
}

void enterConfigMenu() {
  editConfig = config;
  selectedMenuItem = MENU_HSM_ID;
  menuIdConflict = false;
  uiMode = UI_MENU_LIST;
}

void handleMenuButton(uint8_t button) {
  if (button == BTN_MENU) {
    if (uiMode == UI_MENU_EDIT) {
      uiMode = UI_MENU_LIST;
    } else {
      editConfig = config;
      uiMode = UI_NORMAL;
    }
    return;
  }

  if (uiMode == UI_MENU_EDIT) {
    if (button == BTN_UP) adjustMenuValue(1);
    else if (button == BTN_DOWN) adjustMenuValue(-1);
    else if (button == BTN_OK) uiMode = UI_MENU_LIST;
    return;
  }

  if (button == BTN_UP) {
    selectedMenuItem = selectedMenuItem == 0 ? MENU_ITEM_COUNT - 1 : selectedMenuItem - 1;
  } else if (button == BTN_DOWN) {
    selectedMenuItem = (selectedMenuItem + 1) % MENU_ITEM_COUNT;
  } else if (button == BTN_OK) {
    if (selectedMenuItem <= MENU_AUTO_PAGE_INTERVAL) {
      uiMode = UI_MENU_EDIT;
    } else if (selectedMenuItem == MENU_SAVE_EXIT) {
      if (editConfig.hsmSlaveId == editConfig.ultrasonicSlaveId) {
        menuIdConflict = true;
        selectedMenuItem = MENU_ULTRASONIC_ID;
      } else {
        saveConfig();
        uiMode = UI_NORMAL;
        Serial.println("Konfigurasi disimpan ke NVS");
      }
    } else {
      editConfig = config;
      uiMode = UI_NORMAL;
    }
  }
}
void drawPage() {
  if (!displayReady) return;
  lcd.clearBuffer();

  if (uiMode != UI_NORMAL) {
    drawConfigMenu();
    lcd.sendBuffer();
    return;
  }

  switch (currentPage) {
    case PAGE_HOME:
      drawHomePage();
      break;
    case PAGE_SENSOR:
      drawSensorPage();
      break;
    case PAGE_ULTRASONIC:
      drawUltrasonicPage();
      break;
    case PAGE_ALARM_1:
      drawAlarmPage1();
      break;
    case PAGE_ALARM_2:
      drawAlarmPage2();
      break;
    case PAGE_VOLTAGE:
      drawVoltagePage();
      break;
    case PAGE_COMM:
      drawCommPage();
      break;
    case PAGE_SYSTEM:
      drawSystemPage();
      break;
  }

  lcd.sendBuffer();
}

void handleButtons() {
  byte state = readPCF8574();
  byte changed = lastState ^ state;

  if (millis() - lastDebounceTime > debounceDelay) {
    const uint8_t buttons[] = {BTN_UP, BTN_DOWN, BTN_OK, BTN_MENU};
    for (uint8_t button : buttons) {
      if ((changed & (1 << button)) && isPressed(state, button)) {
        manualPageHoldUntil = millis() + MANUAL_PAGE_HOLD_MS;
        lastPageChange = millis();
        if (uiMode != UI_NORMAL) {
          handleMenuButton(button);
        } else if (button == BTN_MENU) {
          enterConfigMenu();
        } else if (button == BTN_UP) {
          currentPage = currentPage == 0 ? PAGE_TOTAL - 1 : currentPage - 1;
        } else if (button == BTN_DOWN) {
          currentPage = (currentPage + 1) % PAGE_TOTAL;
        } else if (button == BTN_OK) {
          lastPoll = millis() - config.pollIntervalMs;
  lastPageChange = millis();
        }

        drawPage();
        lastDebounceTime = millis();
        break;
      }
    }
  }

  lastState = state;
}
void startupTask() {
  unsigned long now = millis();
  if (startupState == START_RESET_LOW && now - startupTimer >= 100) {
    digitalWrite(W5500_RST, HIGH);
    startupTimer = now;
    startupState = START_RESET_WAIT;
  } else if (startupState == START_RESET_WAIT && now - startupTimer >= 300) {
    SPI.begin(18, 19, 23, W5500_CS);
    Ethernet.init(W5500_CS);
    client.setConnectionTimeout(TCP_CONNECTION_TIMEOUT);
    lanStatus = LAN_INIT;
    networkHardwareReady = true;
    lastDHCPAttempt = millis() - DHCP_RETRY_INTERVAL;

    lcd.begin();
    displayReady = true;
    startupState = START_READY;
    drawPage();
  }
}

void voltageTask() {
  if (millis() - lastVoltageRead < VOLTAGE_READ_INTERVAL) return;
  lastVoltageRead = millis();
  readBoardVoltage();
}
void autoPageTask() {
  if (!config.autoPageEnabled || uiMode != UI_NORMAL || !displayReady) return;

  unsigned long now = millis();
  if ((int32_t)(now - manualPageHoldUntil) < 0) return;
  if (now - lastPageChange < config.autoPageIntervalMs) return;

  lastPageChange = now;
  currentPage = (currentPage + 1) % PAGE_TOTAL;
  drawPage();
}
void setup() {
  Serial.begin(115200);
  loadConfig();
  analogReadResolution(12);
  initMacAddress();

  // Pastikan kedua chip-select tidak aktif sejak boot.
  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, LOW);      // ST7920 aktif-HIGH
  pinMode(W5500_CS, OUTPUT);
  digitalWrite(W5500_CS, HIGH);   // W5500 aktif-LOW

  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW);
  startupTimer = millis();

  RS485Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  mb.begin(&RS485Serial);
  mb.master();

  Wire.begin();
  randomSeed(analogRead(34));
  readBoardVoltage();
}

void loop() {
  startupTask();
  voltageTask();
  autoPageTask();
  uptimeHour = (millis() / 1000UL) / 3600;
  ethernetTask();

  mb.task();
  handleButtons();

  if (!mbBusy && millis() - lastPoll >= config.pollIntervalMs) {
    lastPoll = millis();
    drawPage();

    if (pollTarget == POLL_HSM) {
      // HSM: slave ID 9, holding register 40001-40012 (offset 0-11).
      if (mb.readHreg(config.hsmSlaveId, HSM_START_ADDR, regData, NUM_REGS, cbRead)) {
        mbBusy = true;
        pollTarget = POLL_ULTRASONIC;
      } else {
        Serial.println("Request HSM gagal dikirim");
      }
    } else {
      // Ultrasonic: slave ID 1, holding register address 257, scaling /10 cm.
      if (mb.readHreg(config.ultrasonicSlaveId, config.ultrasonicAddress,
                      &ultrasonicReg, 1, cbUltrasonic)) {
        mbBusy = true;
        pollTarget = POLL_HSM;
      } else {
        Serial.println("Request ultrasonic gagal dikirim");
      }
    }
  }


  if(millis()-lastPrint > NETWORK_STATUS_PRINT_INTERVAL)
  {
    lastPrint = millis();
    printStatus();
  }
}

void initMacAddress() {
  uint64_t efuseMac = ESP.getEfuseMac();
  mac[0] = 0x02;
  mac[1] = (uint8_t)(efuseMac >> 32);
  mac[2] = (uint8_t)(efuseMac >> 24);
  mac[3] = (uint8_t)(efuseMac >> 16);
  mac[4] = (uint8_t)(efuseMac >> 8);
  mac[5] = (uint8_t)efuseMac;

  Serial.printf("Ethernet MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void attemptDhcp() {
  lanStatus = LAN_WAIT_DHCP;
  internetStatus = NET_UNKNOWN;
  ethernetReady = false;
  digitalWrite(LCD_CS, LOW);

  Serial.println("[LAN] Request DHCP...");
  int dhcpResult = Ethernet.begin(mac, DHCP_TIMEOUT, DHCP_RESPONSE_TIMEOUT);
  ethernetHardwareDetected = Ethernet.hardwareStatus() == EthernetW5500;

  if (!ethernetHardwareDetected) {
    lanStatus = LAN_HW_NOT_FOUND;
    Serial.println("[LAN] W5500 NOT FOUND - periksa power, SPI, CS dan RST.");
    return;
  }

  if (dhcpResult == 0) {
    lanStatus = Ethernet.linkStatus() == LinkOFF ? LAN_NO_CABLE : LAN_DHCP_FAILED;
    Serial.println(lanStatus == LAN_NO_CABLE
                     ? "[LAN] Kabel tidak terhubung."
                     : "[LAN] DHCP gagal, akan dicoba kembali.");
    return;
  }

  ethernetReady = true;
  dhcpMaintenanceFailures = 0;
  lanStatus = LAN_CONNECTED;
  Serial.println("[LAN] DHCP berhasil.");
  Serial.print("[LAN] IP      : "); Serial.println(Ethernet.localIP());
  Serial.print("[LAN] Gateway : "); Serial.println(Ethernet.gatewayIP());
  Serial.print("[LAN] DNS     : "); Serial.println(Ethernet.dnsServerIP());
  lastInternetCheck = millis() - INTERNET_CHECK_INTERVAL;
}

void ethernetTask() {
  if (!networkHardwareReady) return;
  unsigned long now = millis();

  if (ethernetHardwareDetected && Ethernet.linkStatus() == LinkOFF) {
    if (ethernetReady) Serial.println("[LAN] Kabel Ethernet terputus.");
    ethernetReady = false;
    lanStatus = LAN_NO_CABLE;
    internetStatus = NET_UNKNOWN;
  }

  if (!ethernetReady) {
    if (now - lastDHCPAttempt >= DHCP_RETRY_INTERVAL) {
      lastDHCPAttempt = now;
      attemptDhcp();
    }
    return;
  }

  lanStatus = LAN_CONNECTED;

  if (now - lastDhcpMaintain >= DHCP_MAINTAIN_INTERVAL) {
    lastDhcpMaintain = now;
    int result = Ethernet.maintain();
    if (result == 1 || result == 3) {
      dhcpMaintenanceFailures++;
      Serial.printf("[DHCP] Maintenance gagal (%u/3).\n", dhcpMaintenanceFailures);
      if (dhcpMaintenanceFailures >= 3) {
        ethernetReady = false;
        lanStatus = LAN_DHCP_FAILED;
        internetStatus = NET_UNKNOWN;
        lastDHCPAttempt = now - DHCP_RETRY_INTERVAL;
      }
    } else if (result == 2 || result == 4) {
      dhcpMaintenanceFailures = 0;
      Serial.println("[DHCP] Lease berhasil diperbarui.");
    }
  }

  if (ethernetReady && now - lastInternetCheck >= INTERNET_CHECK_INTERVAL) {
    lastInternetCheck = now;
    checkInternet();
  }
}

void checkInternet() {
  if (!ethernetReady || Ethernet.linkStatus() != LinkON) {
    internetStatus = NET_UNKNOWN;
    return;
  }

  internetStatus = NET_CHECKING;
  client.stop();
  digitalWrite(LCD_CS, LOW);

  if (client.connect("google.com", 80)) {
    client.println("HEAD / HTTP/1.1");
    client.println("Host: google.com");
    client.println("Connection: close");
    client.println();
    internetStatus = NET_CONNECTED;
    client.stop();
    Serial.println("[NET] Internet connected.");
  } else {
    internetStatus = NET_DISCONNECTED;
    Serial.println("[NET] Internet disconnected/DNS failed.");
  }
}

const char* getLanStatus() {
  switch (lanStatus) {
    case LAN_INIT: return "INIT";
    case LAN_HW_NOT_FOUND: return "HW NOT FOUND";
    case LAN_NO_CABLE: return "NO CABLE";
    case LAN_WAIT_DHCP: return "WAIT DHCP";
    case LAN_CONNECTED: return "CONNECTED";
    case LAN_DHCP_FAILED: return "DHCP FAILED";
    default: return "UNKNOWN";
  }
}

const char* getInternetStatus() {
  switch (internetStatus) {
    case NET_UNKNOWN: return "UNKNOWN";
    case NET_CHECKING: return "CHECKING";
    case NET_CONNECTED: return "CONNECTED";
    case NET_DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN";
  }
}

void printStatus() {
  Serial.println("--------------------------");
  Serial.print("Hardware : ");
  Serial.println(ethernetHardwareDetected ? "W5500" : "NOT READY");
  Serial.print("LAN      : "); Serial.println(getLanStatus());
  Serial.print("Internet : "); Serial.println(getInternetStatus());
  if (ethernetReady) {
    Serial.print("IP       : "); Serial.println(Ethernet.localIP());
    Serial.print("Gateway  : "); Serial.println(Ethernet.gatewayIP());
  }
  Serial.println("--------------------------");
}
