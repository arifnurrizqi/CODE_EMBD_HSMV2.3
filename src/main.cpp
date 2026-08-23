#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ModbusRTU.h>
#include <SPI.h>
#include <Ethernet.h>

// ==========================
// KONFIGURASI MODBUS RS485
// ==========================
#define W5500_CS   5
#define W5500_RST  4

byte mac[] = {0xDE,0xAD,0xBE,0xEF,0xFE,0x01};

EthernetClient client;

enum LanStatus
{
    LAN_INIT,
    LAN_NO_CABLE,
    LAN_WAIT_DHCP,
    LAN_CONNECTED,
    LAN_DHCP_FAILED
};

enum InternetStatus
{
    NET_UNKNOWN,
    NET_CONNECTED,
    NET_DISCONNECTED
};

LanStatus lanStatus = LAN_INIT;
InternetStatus internetStatus = NET_UNKNOWN;

bool ethernetReady = false;

unsigned long lastDHCPAttempt = 0;
unsigned long lastInternetCheck = 0;
unsigned long lastPrint = 0;

const uint32_t DHCP_RETRY_INTERVAL = 5000;
const uint32_t INTERNET_CHECK_INTERVAL = 30000;

void ethernetTask();
void checkInternet();
const char* getLanStatus();
const char* getInternetStatus();
void printStatus();



// =========================
// LCD ST7920
// =========================
U8G2_ST7920_128X64_F_SW_SPI lcd(
  U8G2_R0,
  18,   // clock
  23,   // data
  15,    // cs
  4     // reset
);

// ==========================
// KONFIGURASI MODBUS RS485
// ==========================
ModbusRTU mb;

#define RXD2 17   // RX ESP32 dari TX modul RS485
#define TXD2 16   // TX ESP32 ke RX modul RS485

HardwareSerial RS485Serial(2);

const uint8_t HSM_SLAVE_ID = 9;
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

const uint8_t ULTRASONIC_SLAVE_ID = 1;
const uint16_t ULTRASONIC_ADDR = 257;
uint16_t ultrasonicReg = 250;
float ultrasonicDistanceCm = 25.0f;
bool ultrasonicStatus = false;

enum PollTarget : uint8_t {
  POLL_HSM,
  POLL_ULTRASONIC
};

PollTarget pollTarget = POLL_HSM;

// Status polling
bool mbBusy = false;
unsigned long lastPoll = 0;
const unsigned long pollInterval = 1000; // tiap slave dipoll sekitar 2 detik

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
  PAGE_ALARM,
  PAGE_VOLTAGE,
  PAGE_COMM,
  PAGE_SYSTEM,
  PAGE_TOTAL
};

uint8_t currentPage = PAGE_HOME;

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
// bool lanStatus = true;

// int serverStatus = -65;
unsigned long uptimeSec = 0;

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

    alarmFlow = !(sensorStatus & SENSOR_FLOW_USAGE_OK) || !(sensorStatus & SENSOR_FLOW_SAMPLE_OK);
    alarmPh = !(sensorStatus & SENSOR_PH_OK);
    alarmTurbidity = !(sensorStatus & SENSOR_TURBIDITY_OK);
    alarmSupply = !(sensorStatus & SENSOR_VCC_OK) || !(sensorStatus & SENSOR_5V_OK);

    rs485Status = true;

  } else {
    rs485Status = false;
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
    ultrasonicDistanceCm = ultrasonicReg / 10.0f;
    ultrasonicStatus = true;
    Serial.println("===== DATA ULTRASONIC =====");
    Serial.print("Raw      : ");
    Serial.println(ultrasonicReg);
    Serial.print("Distance : ");
    Serial.print(ultrasonicDistanceCm, 1);
    Serial.println(" cm");
  } else {
    ultrasonicStatus = false;
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
  vin_vcc  = 0.829 * (vout_vcc * (R6 + R7) / R7) + 3.19;

  delay(1000);
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
  return alarmPh + alarmTurbidity + alarmFlow + alarmSupply;
}


void drawHeader(const char* title) {
  lcd.setFont(u8g2_font_6x10_tf);

  // Header background (hitam)
  lcd.drawBox(0, 0, 128, 12);

  // Title (putih)
  lcd.setDrawColor(0);
  lcd.drawStr(2, 10, title);

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

  lcd.drawStr(0, 63, "OK:RFSH UP/DN:MOVE");

  int totalAlarm = getTotalAlarm();

  char buf[12];
  if (totalAlarm == 0) {
    sprintf(buf, "   ALM:--");
  } else {
    sprintf(buf, "   ALM:%d", totalAlarm);
  }

  // tampilkan di kanan bawah
  lcd.drawStr(85, 63, buf);
}

void drawHomePage() {
  char buf[32];

  drawHeader("HYDROFLOW V2.3");

  lcd.setFont(u8g2_font_6x10_tf);

  sprintf(buf, "Flow   : %.2f L/m", flowRateSample);
  lcd.drawStr(2, 22, buf);

  sprintf(buf, "pH     : %.2f", phValue);
  lcd.drawStr(2, 34, buf);

  sprintf(buf, "Turbid : %.1f NTU", turbidity);
  lcd.drawStr(2, 46, buf);
  
  drawFooter();
}

void drawSensorPage() {
  char buf[32];

  drawHeader("DATA SENSOR");

  lcd.setFont(u8g2_font_5x7_tf);

  sprintf(buf, "Flow rate : %.2f L/m", flowRateSample);
  lcd.drawStr(2, 22, buf);

  sprintf(buf, "Pulse use : %u", usagePulseDelta);
  lcd.drawStr(2, 32, buf);

  sprintf(buf, "Pulse smp : %u", samplePulseDelta);
  lcd.drawStr(2, 42, buf);

  sprintf(buf, "Total use : %lu", (unsigned long)usagePulseTotal);
  lcd.drawStr(2, 52, buf);

  sprintf(buf, "pH/Turb   : %.2f/%.1f", phValue, turbidity);
  lcd.drawStr(2, 62, buf);
}

void drawUltrasonicPage() {
  char buf[32];

  drawHeader("ULTRASONIC");
  lcd.setFont(u8g2_font_6x10_tf);

  sprintf(buf, "Distance: %.1f cm", ultrasonicDistanceCm);
  lcd.drawStr(2, 25, buf);

  sprintf(buf, "Raw     : %u", ultrasonicReg);
  lcd.drawStr(2, 38, buf);

  sprintf(buf, "ID:%u Reg:%u", ULTRASONIC_SLAVE_ID, ULTRASONIC_ADDR);
  lcd.drawStr(2, 50, buf);

  drawFooter();
}
void drawAlarmPage() {
  drawHeader("STATUS ALARM");
  lcd.setFont(u8g2_font_5x7_tf);

  lcd.drawStr(2, 22, "Flow      :");
  lcd.drawStr(62, 22, alarmFlow ? "ERROR" : "NORMAL");

  lcd.drawStr(2, 32, "pH Error  :");
  lcd.drawStr(62, 32, alarmPh ? "ACTIVE" : "NORMAL");

  lcd.drawStr(2, 42, "Turbidity :");
  lcd.drawStr(62, 42, alarmTurbidity ? "ACTIVE" : "NORMAL");

  lcd.drawStr(2, 52, "Supply    :");
  lcd.drawStr(62, 52, alarmSupply ? "ERROR" : "NORMAL");


  char buf[22];
  sprintf(buf, "Sys:%u Error:%u", systemStatus, errorDetail);
  lcd.drawStr(2, 62, buf);
}

void drawVoltagePage(){
  readBoardVoltage();
  char buf[32];

  drawHeader("VOLTAGE SYSTEM");
  lcd.setFont(u8g2_font_5x7_tf);

  sprintf(buf, "VCC Mstr  : %.2f V", vin_vcc);
  lcd.drawStr(2, 22, buf);

  sprintf(buf, "VCC Slv   : %.2f V", slaveVcc);
  lcd.drawStr(2, 32, buf);

  sprintf(buf, "5V Mstr   : %.2f V", vin_5v);
  lcd.drawStr(2, 42, buf);
    
  sprintf(buf, "5V Slv    : %.2f V", slave5V);
  lcd.drawStr(2, 52, buf);

  sprintf(buf, "3.3V Mstr : %.2f V", vin_3v3);
  lcd.drawStr(2, 62, buf);

}

void drawCommPage() {
  char buf[32];

  drawHeader("KOMUNIKASI");
  lcd.setFont(u8g2_font_5x7_tf);

  sprintf(buf, "HSM/US ID  : %u/%u", HSM_SLAVE_ID, ULTRASONIC_SLAVE_ID);
  lcd.drawStr(2, 22, buf);

  sprintf(buf, "RS485      : %s", rs485Status ? "CONNECTED" : "TIMEOUT");
  lcd.drawStr(2, 32, buf);

  sprintf(buf, "LAN RJ45   : %s", getLanStatus());
  lcd.drawStr(2, 42, buf);

  sprintf(buf, "INTERNET   : %s", getInternetStatus());
  lcd.drawStr(2, 52, buf);

  sprintf(buf, "Status     : %u/%u", systemStatus, errorDetail);
  lcd.drawStr(2, 62, buf);
}

void drawSystemPage() {
  char buf[32];

  drawHeader("INFO SISTEM");
  lcd.setFont(u8g2_font_5x7_tf);

  lcd.drawStr(2, 22, "Device : Water Analyzer");
  lcd.drawStr(2, 32, "NAME   : HYDROFLOW V2.3");
  lcd.drawStr(2, 42, "BY     : ARNUR TECH");
  lcd.drawStr(2, 52, "FW Ver : v1.0");

  sprintf(buf, "Uptime : %lu s", uptimeSec);
  lcd.drawStr(2, 62, buf);
}

void drawPage() {
  lcd.clearBuffer();

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
    case PAGE_ALARM:
      drawAlarmPage();
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

  // deteksi perubahan
  byte changed = lastState ^ state;

  if (millis() - lastDebounceTime > debounceDelay) {

    // UP
    if ((changed & (1 << BTN_UP)) && isPressed(state, BTN_UP)) {
      if (currentPage == 0) currentPage = PAGE_TOTAL - 1;
      else currentPage--;
      drawPage();
      lastDebounceTime = millis();
    }

    // DOWN
    if ((changed & (1 << BTN_DOWN)) && isPressed(state, BTN_DOWN)) {
      currentPage++;
      if (currentPage >= PAGE_TOTAL) currentPage = 0;
      drawPage();
      lastDebounceTime = millis();
    }

    // OK
    if ((changed & (1 << BTN_OK)) && isPressed(state, BTN_OK)) {
      // Jadwalkan polling segera; data layar tidak lagi diisi simulasi.
      lastPoll = millis() - pollInterval;
      lastDebounceTime = millis();
    }

    // MENU (optional, nanti bisa dipakai untuk masuk setting)
    if ((changed & (1 << BTN_MENU)) && isPressed(state, BTN_MENU)) {
      Serial.println("MENU pressed");
      lastDebounceTime = millis();
    }
  }

  lastState = state;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(W5500_RST, OUTPUT);

  digitalWrite(W5500_RST, LOW);
  delay(100);
  digitalWrite(W5500_RST, HIGH);
  delay(300);

  SPI.begin(18,19,23,W5500_CS);

  Ethernet.init(W5500_CS);

  lanStatus = LAN_INIT;

  // Serial2 untuk RS485
  RS485Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  // Master Modbus tanpa DE/RE
  mb.begin(&RS485Serial);
  mb.master();

  lcd.begin();
  Wire.begin();

  randomSeed(analogRead(34));

  drawPage();
}

void loop() {
  ethernetTask();

  mb.task();
  handleButtons();

  if (!mbBusy && millis() - lastPoll >= pollInterval) {
    lastPoll = millis();
    drawPage();

    if (pollTarget == POLL_HSM) {
      // HSM: slave ID 9, holding register 40001-40012 (offset 0-11).
      if (mb.readHreg(HSM_SLAVE_ID, HSM_START_ADDR, regData, NUM_REGS, cbRead)) {
        mbBusy = true;
        pollTarget = POLL_ULTRASONIC;
      } else {
        Serial.println("Request HSM gagal dikirim");
      }
    } else {
      // Ultrasonic: slave ID 1, holding register address 257, scaling /10 cm.
      if (mb.readHreg(ULTRASONIC_SLAVE_ID, ULTRASONIC_ADDR,
                      &ultrasonicReg, 1, cbUltrasonic)) {
        mbBusy = true;
        pollTarget = POLL_HSM;
      } else {
        Serial.println("Request ultrasonic gagal dikirim");
      }
    }
  }

  if(ethernetReady)
  {
    Ethernet.maintain();

    if(millis()-lastInternetCheck > INTERNET_CHECK_INTERVAL)
    {
      lastInternetCheck = millis();
      checkInternet();
    }
  }

  if(millis()-lastPrint > 1000)
  {
    lastPrint = millis();
    printStatus();
  }
}

void ethernetTask()
{
    if(Ethernet.linkStatus()==LinkOFF)
    {
        ethernetReady = false;
        lanStatus = LAN_NO_CABLE;
        internetStatus = NET_UNKNOWN;
        return;
    }

    if(ethernetReady)
    {
        lanStatus = LAN_CONNECTED;
        return;
    }

    lanStatus = LAN_WAIT_DHCP;

    if(millis()-lastDHCPAttempt < DHCP_RETRY_INTERVAL)
        return;

    lastDHCPAttempt = millis();

    Serial.println("Request DHCP...");

    if(Ethernet.begin(mac))
    {
        ethernetReady = true;
        lanStatus = LAN_CONNECTED;

        Serial.println("DHCP Success");
        Serial.print("IP : ");
        Serial.println(Ethernet.localIP());

        checkInternet();
    }
    else
    {
        lanStatus = LAN_DHCP_FAILED;
    }
}

//======================================================

void checkInternet()
{
    if(client.connect("google.com",80))
    {
        internetStatus = NET_CONNECTED;
        client.stop();
    }
    else
    {
        internetStatus = NET_DISCONNECTED;
    }
}

const char* getLanStatus()
{
  switch(lanStatus)
  {
    case LAN_INIT:
      return "INIT";

    case LAN_NO_CABLE:
      return "NO CABLE";

    case LAN_WAIT_DHCP:
      return "WAIT DHCP";

    case LAN_CONNECTED:
      return "CONNECTED";

    case LAN_DHCP_FAILED:
      return "DHCP FAILED";

    default:
      return "UNKNOWN";
  }
}

const char* getInternetStatus()
{
  switch(internetStatus)
  {
    case NET_UNKNOWN:
      return "UNKNOWN";

    case NET_CONNECTED:
      return "CONNECTED";

    case NET_DISCONNECTED:
      return "DISCONNECTED";

    default:
      return "UNKNOWN";
  }
}

void printStatus()
{
    Serial.println("--------------------------");

    Serial.print("LAN      : ");
    Serial.println(getLanStatus());

    Serial.print("Internet : ");
    Serial.println(getInternetStatus());

    if(ethernetReady)
    {
        Serial.print("IP       : ");
        Serial.println(Ethernet.localIP());

        Serial.print("Gateway  : ");
        Serial.println(Ethernet.gatewayIP());
    }

    Serial.println("--------------------------");
}