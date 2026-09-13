#pragma once
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// Fixed-size, versioned record: one NVS write per successful save.
struct NetworkConfig {
  uint32_t version = 1;
  char endpoint[192] = "http://192.168.8.7:8000/api/v1/telemetry";
  char token[128] = "";
  uint32_t intervalSeconds = 30;
  bool dhcp = true;
  char ip[16] = "192.168.8.50";
  char gateway[16] = "192.168.8.1";
  char subnet[16] = "255.255.255.0";
  char dns[16] = "192.168.8.1";
  bool mqttEnabled = false;
  char mqttHost[128] = "";
  uint32_t mqttPort = 8883;
  char transport[8] = "tls";
  char mqttPath[96] = "/mqtt";
  char username[64] = "";
  char password[128] = "";
};
NetworkConfig networkConfig;
NetworkConfig pendingNetworkConfig;
WebServer setupServer(80);
bool webSetupActive = false;
bool setupRoutesRegistered = false;
bool networkApplyPending = false;
unsigned long setupLastActivity = 0;
unsigned long setupCloseAt = 0;
const uint32_t SETUP_IDLE_TIMEOUT_MS = 300000UL;
uint32_t setupDisplayedSeconds = UINT32_MAX;
void drawPage();

uint32_t setupRemainingSeconds() {
  uint32_t now = millis();
  if (setupCloseAt) {
    int32_t remaining = (int32_t)(setupCloseAt - now);
    return remaining > 0 ? ((uint32_t)remaining + 999UL) / 1000UL : 0;
  }
  uint32_t elapsed = now - setupLastActivity;
  return elapsed < SETUP_IDLE_TIMEOUT_MS
    ? (SETUP_IDLE_TIMEOUT_MS - elapsed + 999UL) / 1000UL : 0;
}
char setupSsid[24];
char setupPassword[13];
String setupNonce;
String telemetryHost;
String telemetryPath;
uint16_t telemetryPort = 8000;

bool safeText(const String& value, size_t capacity) {
  if (value.length() >= capacity) return false;
  for (size_t i = 0; i < value.length(); ++i)
    if ((uint8_t)value[i] < 32 || (uint8_t)value[i] > 126) return false;
  return true;
}
bool validHost(const String& host) {
  if (host.isEmpty() || host.length() > 127) return false;
  for (size_t i = 0; i < host.length(); ++i)
    if (!isalnum((unsigned char)host[i]) && host[i] != '.' && host[i] != '-') return false;
  return true;
}
bool numberInRange(const String& s, uint32_t low, uint32_t high, uint32_t& out) {
  if (s.isEmpty() || s.length() > 7) return false;
  uint32_t n = 0;
  for (size_t i = 0; i < s.length(); ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
    n = n * 10 + s[i] - '0';
  }
  if (n < low || n > high) return false;
  out = n;
  return true;
}
bool parseEndpoint(const String& url, String& host, uint16_t& port, String& path) {
  if (!safeText(url, 192) || !url.startsWith("http://")) return false;
  int slash = url.indexOf('/', 7);
  String authority = slash < 0 ? url.substring(7) : url.substring(7, slash);
  path = slash < 0 ? "/" : url.substring(slash);
  if (path.indexOf(' ') >= 0 || path.indexOf('#') >= 0) return false;
  int colon = authority.indexOf(':');
  host = colon < 0 ? authority : authority.substring(0, colon);
  uint32_t parsedPort = 80;
  if (colon >= 0 && !numberInRange(authority.substring(colon + 1), 1, 65535, parsedPort)) return false;
  port = parsedPort;
  return validHost(host);
}
void loadNetworkConfig() {
  Preferences p;
  NetworkConfig saved;
  if (p.begin("hydro-net", true)) {
    if (p.getBytesLength("config") == sizeof(saved) &&
        p.getBytes("config", &saved, sizeof(saved)) == sizeof(saved) && saved.version == 1)
      networkConfig = saved;
    p.end();
  }
  if (!parseEndpoint(networkConfig.endpoint, telemetryHost, telemetryPort, telemetryPath)) {
    networkConfig = NetworkConfig{};
    parseEndpoint(networkConfig.endpoint, telemetryHost, telemetryPort, telemetryPath);
  }
}
String htmlEscape(String value) {
  value.replace("&", "&amp;"); value.replace("<", "&lt;");
  value.replace(">", "&gt;"); value.replace("\"", "&quot;");
  value.replace("'", "&#39;");
  return value;
}
String setupInput(const char* label, const char* key, const String& value, size_t limit, const char* type = "text") {
  return String("<label>") + label + "<input name='" + key + "' type='" + type +
    "' maxlength='" + String(limit) + "' value='" + htmlEscape(value) + "'></label>";
}
void stopWebSetup() {
  setupServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  webSetupActive = false;
  setupCloseAt = 0;
  Serial.println("[SETUP] Hotspot ditutup.");
}
void startWebSetup(const char* deviceId) {
  if (webSetupActive) return;
  snprintf(setupSsid, sizeof(setupSsid), "Hydroflow-%06X", (unsigned)(ESP.getEfuseMac() & 0xFFFFFF));
  snprintf(setupPassword, sizeof(setupPassword), "%08X", (unsigned)esp_random());
  setupNonce = String(esp_random(), HEX) + String(esp_random(), HEX);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  if (!WiFi.softAP(setupSsid, setupPassword, 1, false, 1)) {
    WiFi.mode(WIFI_OFF);
    Serial.println("[SETUP] Gagal membuat hotspot.");
    return;
  }
  if (!setupRoutesRegistered) {
  setupServer.on("/", HTTP_GET, [deviceId]() {
    setupLastActivity = millis();
    String page;
    page.reserve(7000);
    page = F("<!doctype html><html lang='id'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Hydroflow Setup</title><style>body{font:16px system-ui;background:#eef3f6;color:#183040;max-width:650px;margin:24px auto;padding:16px}fieldset{background:white;border:1px solid #ccd;border-radius:10px;margin:18px 0;padding:18px}label{display:block;margin:12px 0}input,select,button{box-sizing:border-box;width:100%;padding:10px;font:inherit}button{background:#096e76;color:white;border:0;border-radius:6px}small{color:#456}</style><h1>Hydroflow Setup</h1>");
    page += "<p>Device: " + htmlEscape(deviceId) + "</p><p>LAN: " + String(getLanStatus()) +
      " | IP: " + Ethernet.localIP().toString() + "<br>RTC: " + (rtcTimeValid ? String("valid") : String("belum valid")) +
      " | NTP: " + (rtcSyncedFromNtp ? String("sinkron") : String("belum sinkron")) +
      "<br>HTTP terakhir: " + (!telemetryEverAttempted ? String("belum mengirim") : telemetryLastSuccess ? String("berhasil") : String("menunggu/gagal")) + "</p>";
    page += "<form method='post' action='/save'><input type='hidden' name='nonce' value='" + setupNonce + "'><fieldset><legend>HTTP telemetry</legend>";
    page += setupInput("Endpoint lengkap (http://)", "endpoint", networkConfig.endpoint, 191);
    page += setupInput("Interval (15-3600 detik)", "interval", String(networkConfig.intervalSeconds), 4, "number");
    page += setupInput("Bearer token baru (kosong = tetap)", "token", "", 127, "password");
    page += "<label><input type='checkbox' name='clearToken' value='1'> Hapus token tersimpan</label></fieldset><fieldset><legend>Ethernet</legend><label>Mode<select name='dhcp'>";
    page += String("<option value='1'") + (networkConfig.dhcp ? " selected" : "") + ">DHCP</option><option value='0'" + (!networkConfig.dhcp ? " selected" : "") + ">IP statis</option></select></label>";
    page += setupInput("IP statis", "ip", networkConfig.ip, 15);
    page += setupInput("Gateway", "gateway", networkConfig.gateway, 15);
    page += setupInput("Subnet", "subnet", networkConfig.subnet, 15);
    page += setupInput("DNS", "dns", networkConfig.dns, 15);
    page += "</fieldset><fieldset><legend>MQTT (persiapan)</legend><p>Disimpan saja; firmware ini belum menjalankan MQTT.</p>";
    page += String("<label><input type='checkbox' name='mqttEnabled' value='1'") + (networkConfig.mqttEnabled ? " checked" : "") + "> Aktif saat integrasi MQTT tersedia</label>";
    page += setupInput("Host broker", "mqttHost", networkConfig.mqttHost, 127);
    page += setupInput("Port", "mqttPort", String(networkConfig.mqttPort), 5, "number");
    page += "<label>Transport<select name='transport'>";
    for (const char* mode : {"tcp", "tls", "ws", "wss"})
      page += String("<option") + (String(networkConfig.transport) == mode ? " selected" : "") + ">" + mode + "</option>";
    page += "</select></label>";
    page += setupInput("Path WebSocket", "mqttPath", networkConfig.mqttPath, 95);
    page += setupInput("Username", "username", networkConfig.username, 63);
    page += setupInput("Password baru (kosong = tetap)", "password", "", 127, "password");
    page += "<label><input type='checkbox' name='clearPassword' value='1'> Hapus password tersimpan</label><p>Topic: devices/" + htmlEscape(deviceId) + "/telemetry</p></fieldset><button>Simpan &amp; Terapkan</button><p>Hotspot ditutup setelah disimpan, atau 5 menit tanpa aktivitas. MENU pada LCD untuk keluar.</p></form></html>";
    setupServer.sendHeader("Cache-Control", "no-store");
    setupServer.send(200, "text/html", page);
  });
  setupServer.on("/save", HTTP_POST, []() {
    if (setupCloseAt || setupServer.arg("nonce") != setupNonce) {
      setupServer.send(403, "text/plain", "Sesi tidak valid. Buka ulang halaman."); return;
    }
    setupLastActivity = millis();
    NetworkConfig candidate = networkConfig;
    auto copy = [&](const char* key, char* target, size_t size) {
      String value = setupServer.arg(key);
      if (!safeText(value, size)) return false;
      value.toCharArray(target, size); return true;
    };
    String host, path; uint16_t port;
    bool valid = copy("endpoint", candidate.endpoint, sizeof(candidate.endpoint)) &&
      parseEndpoint(candidate.endpoint, host, port, path) &&
      numberInRange(setupServer.arg("interval"), 15, 3600, candidate.intervalSeconds);
    candidate.dhcp = setupServer.arg("dhcp") == "1";
    valid &= setupServer.arg("dhcp") == "1" || setupServer.arg("dhcp") == "0";
    valid &= copy("ip", candidate.ip, sizeof(candidate.ip));
    valid &= copy("gateway", candidate.gateway, sizeof(candidate.gateway));
    valid &= copy("subnet", candidate.subnet, sizeof(candidate.subnet));
    valid &= copy("dns", candidate.dns, sizeof(candidate.dns));
    IPAddress ip, gateway, subnet, dns;
    if (!candidate.dhcp) {
      valid &= ip.fromString(candidate.ip) && gateway.fromString(candidate.gateway) &&
               subnet.fromString(candidate.subnet) && dns.fromString(candidate.dns);
      uint32_t mask = 0, address = 0, router = 0;
      for (int i = 0; i < 4; ++i) { mask = (mask << 8) | subnet[i]; address = (address << 8) | ip[i]; router = (router << 8) | gateway[i]; }
      uint32_t inverse = ~mask;
      valid &= mask != 0 && inverse >= 3 && (inverse & (inverse + 1)) == 0 &&
        (address & mask) == (router & mask) && (address & inverse) != 0 &&
        (address & inverse) != inverse && ip[0] > 0 && ip[0] < 224 && dns[0] > 0;
    }
    candidate.mqttEnabled = setupServer.hasArg("mqttEnabled");
    valid &= copy("mqttHost", candidate.mqttHost, sizeof(candidate.mqttHost));
    valid &= !candidate.mqttHost[0] ? !candidate.mqttEnabled : validHost(candidate.mqttHost);
    valid &= numberInRange(setupServer.arg("mqttPort"), 1, 65535, candidate.mqttPort);
    valid &= copy("transport", candidate.transport, sizeof(candidate.transport));
    String transport = candidate.transport;
    valid &= transport == "tcp" || transport == "tls" || transport == "ws" || transport == "wss";
    valid &= copy("mqttPath", candidate.mqttPath, sizeof(candidate.mqttPath));
    valid &= candidate.mqttPath[0] == '/' && String(candidate.mqttPath).indexOf(' ') < 0;
    valid &= copy("username", candidate.username, sizeof(candidate.username));
    if (setupServer.hasArg("clearToken")) candidate.token[0] = 0;
    else if (setupServer.arg("token").length()) valid &= copy("token", candidate.token, sizeof(candidate.token));
    if (setupServer.hasArg("clearPassword")) candidate.password[0] = 0;
    else if (setupServer.arg("password").length()) valid &= copy("password", candidate.password, sizeof(candidate.password));
    if (!valid) { setupServer.send(400, "text/plain", "Konfigurasi tidak valid. Periksa URL http://, interval, IP/subnet dan broker. Kembali untuk memperbaiki."); return; }
    Preferences p;
    if (!p.begin("hydro-net", false)) { setupServer.send(500, "text/plain", "NVS tidak dapat dibuka."); return; }
    bool saved = p.putBytes("config", &candidate, sizeof(candidate)) == sizeof(candidate);
    p.end();
    if (!saved) { setupServer.send(500, "text/plain", "Gagal menyimpan. Konfigurasi aktif tetap."); return; }
    pendingNetworkConfig = candidate;
    setupServer.send(200, "text/html", "<meta name='viewport' content='width=device-width'><h2>Konfigurasi disimpan</h2><p>Pengaturan diterapkan dan hotspot ditutup dalam 2 detik. MQTT hanya disimpan untuk integrasi berikutnya.</p>");
    setupCloseAt = millis() + 2000;
  });
  setupServer.onNotFound([]() { setupServer.send(404, "text/plain", "Buka http://192.168.4.1/"); });
  setupRoutesRegistered = true;
  }
  setupServer.begin();
  webSetupActive = true;
  setupLastActivity = millis();
  setupCloseAt = 0;
  setupDisplayedSeconds = UINT32_MAX;
}
void webSetupTask() {
  if (!webSetupActive) return;
  setupServer.handleClient();
  if (setupCloseAt && (int32_t)(millis() - setupCloseAt) >= 0) {
    networkApplyPending = true;
    stopWebSetup();
  } else if (millis() - setupLastActivity >= SETUP_IDLE_TIMEOUT_MS) stopWebSetup();
  if (!webSetupActive) {
    drawPage();
    return;
  }
  uint32_t remaining = setupRemainingSeconds();
  if (remaining != setupDisplayedSeconds) {
    setupDisplayedSeconds = remaining;
    drawPage();
  }
}
