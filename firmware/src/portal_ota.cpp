#include "portal_ota.h"
#include "github_roots.h"

#include <DNSServer.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <mbedtls/sha256.h>
#include <time.h>

namespace {
constexpr char WIFI_NAMESPACE[] = "mantwifi";
constexpr char MANIFEST_URL[] =
  "https://github.com/e-nano/open-ndvi-handheld/releases/latest/download/manifest.txt";
constexpr char FIRMWARE_URL[] =
  "https://github.com/e-nano/open-ndvi-handheld/releases/latest/download/firmware.bin";

DNSServer dns;
WebServer server(80);
String apSsid;
String apPassword;
String currentVersion;
String message;
String formToken;
bool active = false;
bool routesRegistered = false;
unsigned long rebootAt = 0;
unsigned long closeAt = 0;

struct ReleaseInfo {
  String version;
  size_t size = 0;
  String sha256;
  bool valid = false;
} releaseInfo;

String escapeHtml(const String &source) {
  String result;
  result.reserve(source.length() + 12);
  for (size_t i = 0; i < source.length(); ++i) {
    switch (source[i]) {
      case '&': result += "&amp;"; break;
      case '<': result += "&lt;"; break;
      case '>': result += "&gt;"; break;
      case '"': result += "&quot;"; break;
      case '\'': result += "&#39;"; break;
      default: result += source[i]; break;
    }
  }
  return result;
}

bool fromAp() {
  return server.client().localIP() == WiFi.softAPIP();
}

void sendPage(const String &body, int status = 200) {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.sendHeader("Content-Security-Policy", "default-src 'self' 'unsafe-inline'");
  String page;
  page.reserve(5500 + body.length());
  // Self-contained Primer-style tokens keep the portal usable without internet access.
  page += R"PORTAL(<!doctype html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#f6f8fa"><title>Manti-S setup</title>
<style>
:root{--bgColor-default:#fff;--bgColor-muted:#f6f8fa;--fgColor-default:#1f2328;
--fgColor-muted:#59636e;--fgColor-accent:#0969da;--borderColor-default:#d1d9e0;
--bgColor-success-emphasis:#1f883d}
*{box-sizing:border-box}
body{margin:0;background:var(--bgColor-muted);color:var(--fgColor-default);
font:16px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
.topbar{background:var(--bgColor-default);border-bottom:1px solid var(--borderColor-default)}
.topbar-inner{max-width:640px;margin:auto;padding:16px 20px;display:flex;align-items:center;
justify-content:space-between;gap:12px}
.brand{display:flex;align-items:center;gap:9px;margin:0;font-size:18px;font-weight:700;letter-spacing:-.02em}
.brand-dot{display:inline-block;width:18px;height:18px;border:5px solid #1f883d;border-radius:50%}
.chip{padding:3px 9px;border:1px solid var(--borderColor-default);border-radius:20px;
color:var(--fgColor-muted);font-size:12px;font-weight:600;white-space:nowrap}
main{max-width:640px;margin:auto;padding:28px 20px 40px}
.muted,.help{color:var(--fgColor-muted)}
.panel{background:var(--bgColor-default);border:1px solid var(--borderColor-default);
border-radius:12px;padding:22px;margin:0 0 16px;box-shadow:0 1px 2px #1f23280a}
.panel-head{display:flex;align-items:center;gap:10px;margin-bottom:14px}
.panel-head h2{margin:0;font-size:18px;line-height:1.3}.panel-head .chip{margin-left:auto}
.step{display:inline-grid;place-items:center;flex:none;width:25px;height:25px;border-radius:50%;
background:#ddf4ff;color:var(--fgColor-accent);font-size:13px;font-weight:700}
.meta{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin:0}
.meta div{min-width:0}.meta dt{color:var(--fgColor-muted);font-size:12px;font-weight:600}
.meta dd{margin:2px 0 0;font-size:14px;font-weight:600;overflow-wrap:anywhere}
.notice{padding:12px 14px;margin:0 0 16px;background:#ddf4ff;border:1px solid #80ccff;
border-radius:8px;color:#0550ae;font-size:14px}
.field{margin:14px 0}.field label{display:block;margin-bottom:6px;font-size:14px;font-weight:600}
input,select{display:block;width:100%;min-height:42px;padding:8px 11px;border:1px solid var(--borderColor-default);
border-radius:6px;background:#fff;color:var(--fgColor-default);font:inherit}
input:focus-visible,select:focus-visible,button:focus-visible,a:focus-visible{
outline:2px solid var(--fgColor-accent);outline-offset:2px}
.help{display:block;margin:7px 0 0;font-size:13px}.btn{display:inline-flex;align-items:center;
justify-content:center;min-height:40px;padding:8px 15px;border:1px solid var(--borderColor-default);
border-radius:6px;background:var(--bgColor-muted);color:#25292e;font:inherit;font-size:14px;
font-weight:600;line-height:1.2;
cursor:pointer;text-decoration:none;box-shadow:0 1px 0 #1f23280a}
.btn:hover{background:#eff2f5}.btn-primary{border-color:#1f883d;background:#1f883d;color:#fff}
.btn-primary:hover{background:#1a7f37}.btn:disabled{opacity:.6;cursor:wait}
.release{padding:12px 14px;margin:14px 0;border:1px solid #82d79a;border-radius:8px;
background:#dafbe1;color:#116329;font-size:14px}.release strong{display:block}
.actions{display:flex;flex-wrap:wrap;gap:10px;margin-top:16px}.actions form{margin:0}
.exit{display:flex;align-items:center;justify-content:space-between;gap:12px;
color:var(--fgColor-muted);font-size:14px}.exit form{margin:0}
footer{max-width:640px;margin:0 auto;padding:0 20px 24px;color:var(--fgColor-muted);font-size:12px}
@media(max-width:480px){main{padding:22px 16px 32px}.topbar-inner{padding:14px 16px}
.panel{padding:18px}.exit{align-items:stretch;flex-direction:column}
.actions form,.actions .btn,.exit form,.exit .btn{width:100%}}
</style></head><body><header class="topbar"><div class="topbar-inner">
<h1 class="brand"><span class="brand-dot" aria-hidden="true"></span>Manti-S</h1>
</div></header><main>)PORTAL";
  page += body;
  page += "</main><footer>Local device portal &middot; 192.168.4.1</footer></body></html>";
  server.send(status, "text/html; charset=utf-8", page);
}

void sendHome() {
  if (!fromAp()) { server.send(403, "text/plain", "Use the device Wi-Fi network"); return; }
  String page = "<section class='panel'><div class='panel-head'><h2>Device</h2>"
                "<span class='chip'>Portal active</span></div><dl class='meta'>"
                "<div><dt>Device network</dt><dd>" + escapeHtml(apSsid) +
                "</dd></div><div><dt>Firmware</dt><dd>" + escapeHtml(currentVersion) +
                "</dd></div><div><dt>Internet Wi-Fi</dt><dd>" +
                String(WiFi.status() == WL_CONNECTED ? escapeHtml(WiFi.SSID()) : "Not connected") +
                "</dd></div></dl></section>";
  if (message.length()) page += "<div class='notice' role='status'>" + escapeHtml(message) + "</div>";
  page += "<section class='panel'><div class='panel-head'><span class='step'>1</span>"
          "<h2>Connect to Wi-Fi</h2></div><p class='muted'>Choose a network with internet access. "
          "The device network stays available while you connect.</p>"
          "<form method='post' action='/connect'>"
          "<input type='hidden' name='token' value='" + formToken + "'>"
          "<div class='field'><label for='network'>Nearby networks</label>"
          "<select id='network' name='ssid'><option value=''>Choose a network</option>";
  int count = WiFi.scanNetworks();
  for (int i = 0; i < count; ++i) {
    String ssid = WiFi.SSID(i);
    if (ssid.length()) {
      page += "<option value='" + escapeHtml(ssid) + "'>" + escapeHtml(ssid) + " (" +
              String(WiFi.RSSI(i)) + " dBm)</option>";
    }
  }
  WiFi.scanDelete();
  page += "</select></div><div class='field'><label for='hidden_ssid'>Hidden network name "
          "<span class='muted'>(optional)</span></label>"
          "<input id='hidden_ssid' name='hidden_ssid' maxlength='32' autocomplete='off'>"
          "<small class='help'>If entered, this name is used instead of the selected network.</small></div>"
          "<div class='field'><label for='wifi_password'>Wi-Fi password</label>"
          "<input id='wifi_password' type='password' name='password' maxlength='63' "
          "autocomplete='off'></div><div class='actions'>"
          "<button class='btn btn-primary' type='submit'>Connect to Wi-Fi</button>"
          "</div></form></section>";
  page += "<section class='panel'><div class='panel-head'><span class='step'>2</span>"
          "<h2>Firmware update</h2></div>"
          "<p class='muted'>Check the latest GitHub Release. Nothing installs until you choose Install update.</p>"
          "<div class='actions'><form method='post' action='/check'>"
          "<input type='hidden' name='token' value='" + formToken + "'>"
          "<button class='btn' type='submit'>Check for update</button></form></div>";
  if (releaseInfo.valid) {
    page += "<div class='release'><strong>Update available</strong>Version " +
            escapeHtml(releaseInfo.version) + "</div>"
            "<div class='actions'><form method='post' action='/update' "
            "onsubmit=\"this.querySelector('button').disabled=true;this.querySelector('button').textContent='Installing...';\">"
            "<input type='hidden' name='token' value='" + formToken + "'>"
            "<button class='btn btn-primary' type='submit'>Install update</button></form></div>";
  }
  page += "<small class='help'>Keep USB power connected during installation.</small></section>";
  page += "<section class='panel exit'><span>Done? Return to normal measurement mode.</span>"
          "<form method='post' action='/finish'>"
          "<input type='hidden' name='token' value='" + formToken + "'>"
          "<button class='btn' type='submit'>Finish setup</button></form></section>";
  sendPage(page);
}

bool authorizedPost() {
  if (!fromAp() || server.arg("token") != formToken) {
    server.send(403, "text/plain", "Forbidden");
    return false;
  }
  return true;
}

bool secureClockReady() {
  if (time(nullptr) > 1700000000) return true;
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  for (int i = 0; i < 100 && time(nullptr) <= 1700000000; ++i) delay(100);
  return time(nullptr) > 1700000000;
}

String manifestField(const String &body, const char *key) {
  String prefix = String(key) + "=";
  int start = body.indexOf(prefix);
  if (start < 0 || (start > 0 && body[start - 1] != '\n')) return "";
  start += prefix.length();
  int end = body.indexOf('\n', start);
  if (end < 0) end = body.length();
  String value = body.substring(start, end);
  value.trim();
  return value;
}

bool parseManifest(const String &body, ReleaseInfo &info) {
  String version = manifestField(body, "version");
  String sizeText = manifestField(body, "size");
  String digest = manifestField(body, "sha256");
  if (version.length() < 2 || version.length() > 40 || sizeText.length() == 0 ||
      sizeText.length() > 8 || digest.length() != 64) return false;
  for (size_t i = 0; i < version.length(); ++i) {
    char c = version[i];
    if (!(isalnum((unsigned char)c) || c == '.' || c == '-' || c == '_')) return false;
  }
  for (size_t i = 0; i < sizeText.length(); ++i) {
    if (!isdigit((unsigned char)sizeText[i])) return false;
  }
  for (size_t i = 0; i < digest.length(); ++i) {
    if (!isxdigit((unsigned char)digest[i])) return false;
  }
  size_t size = strtoul(sizeText.c_str(), nullptr, 10);
  if (size < 100000 || size > 0x300000) return false;
  info.version = version;
  info.size = size;
  info.sha256 = digest;
  info.sha256.toLowerCase();
  info.valid = true;
  return true;
}

void connectWifi() {
  if (!authorizedPost()) return;
  String ssid = server.arg("hidden_ssid");
  if (ssid.isEmpty()) ssid = server.arg("ssid");
  String password = server.arg("password");
  if (ssid.isEmpty() || ssid.length() > 32 || password.length() > 63) {
    message = "Invalid network name or password";
    sendPage("<p>" + message + "</p><a href='/'>Back</a>", 400);
    return;
  }
  message = "Connecting to " + ssid;
  WiFi.begin(ssid.c_str(), password.c_str());
  for (int i = 0; i < 150 && WiFi.status() != WL_CONNECTED; ++i) delay(100);
  if (WiFi.status() == WL_CONNECTED) {
    Preferences prefs;
    if (prefs.begin(WIFI_NAMESPACE, false)) {
      prefs.putString("ssid", ssid);
      prefs.putString("pass", password);
      prefs.end();
    }
    message = "Connected. Return to this device Wi-Fi page to check for updates.";
  } else {
    message = "Connection failed. Check the password and try again.";
  }
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

void checkUpdate() {
  if (!authorizedPost()) return;
  releaseInfo = ReleaseInfo();
  if (WiFi.status() != WL_CONNECTED) {
    message = "Connect the device to Wi-Fi first.";
  } else if (!secureClockReady()) {
    message = "Could not set the clock for a secure connection.";
  } else {
    WiFiClientSecure client;
    client.setCACert(GITHUB_ROOT_CAS);
    HTTPClient http;
    http.setTimeout(15000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (!http.begin(client, MANIFEST_URL)) {
      message = "Could not open update source.";
    } else {
      int status = http.GET();
      int size = http.getSize();
      if (status != 200 || size < 0 || size > 1024) {
        message = "Update manifest unavailable.";
      } else {
        ReleaseInfo found;
        if (!parseManifest(http.getString(), found)) message = "Invalid update manifest.";
        else if (found.version == currentVersion) message = "Firmware is up to date.";
        else { releaseInfo = found; message = "Update available. Press Install update to continue."; }
      }
      http.end();
    }
  }
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

void installUpdate() {
  if (!authorizedPost()) return;
  if (!releaseInfo.valid || WiFi.status() != WL_CONNECTED || !secureClockReady()) {
    server.send(409, "text/plain", "Check for an update first");
    return;
  }
  WiFiClientSecure client;
  client.setCACert(GITHUB_ROOT_CAS);
  HTTPClient http;
  http.setTimeout(20000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  message = "Downloading update";
  if (!http.begin(client, FIRMWARE_URL) || http.GET() != 200 ||
      http.getSize() != (int)releaseInfo.size || !Update.begin(releaseInfo.size, U_FLASH)) {
    message = "Update download could not start.";
    http.end();
    sendPage("<p>" + message + "</p><a href='/'>Back</a>", 502);
    return;
  }

  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts_ret(&sha, 0);
  WiFiClient *stream = http.getStreamPtr();
  uint8_t buffer[2048];
  size_t received = 0;
  unsigned long lastData = millis();
  bool good = true;
  while (received < releaseInfo.size) {
    size_t available = stream->available();
    if (!available) {
      if (millis() - lastData > 20000 || !http.connected()) { good = false; break; }
      delay(10);
      continue;
    }
    size_t amount = min((size_t)sizeof(buffer), min(available, releaseInfo.size - received));
    size_t n = stream->readBytes(buffer, amount);
    if (!n || Update.write(buffer, n) != n || mbedtls_sha256_update_ret(&sha, buffer, n) != 0) {
      good = false;
      break;
    }
    received += n;
    lastData = millis();
    delay(1);
  }
  uint8_t digest[32];
  if (mbedtls_sha256_finish_ret(&sha, digest) != 0) good = false;
  mbedtls_sha256_free(&sha);
  char digestText[65];
  for (size_t i = 0; i < sizeof(digest); ++i) snprintf(digestText + 2 * i, 3, "%02x", digest[i]);
  digestText[64] = 0;
  if (!good || received != releaseInfo.size || releaseInfo.sha256 != digestText) {
    Update.abort();
    message = "Update failed integrity check or download interrupted.";
    http.end();
    sendPage("<p>" + message + "</p><a href='/'>Back</a>", 502);
    return;
  }
  http.end();
  if (!Update.end()) {
    message = "Firmware verification failed.";
    sendPage("<p>" + message + "</p><a href='/'>Back</a>", 502);
    return;
  }
  releaseInfo.valid = false;
  message = "Update installed. Rebooting.";
  sendPage("<p>" + message + "</p>");
  rebootAt = millis() + 2000;
}

void finishSetup() {
  if (!authorizedPost()) return;
  message = "Closing setup and returning to measurement mode.";
  sendPage("<p>" + message + "</p>");
  closeAt = millis() + 500;
}
}  // namespace

bool portalBegin(const char *deviceName, const char *firmwareVersion) {
  if (active) return true;
  Preferences prefs;
  String savedSsid;
  String savedPassword;
  if (prefs.begin(WIFI_NAMESPACE, true)) {
    savedSsid = prefs.getString("ssid", "");
    savedPassword = prefs.getString("pass", "");
    prefs.end();
  }
  currentVersion = firmwareVersion;
  apSsid = deviceName;
  constexpr char easyChars[] = "ABCDEFGHJKLMNPQRTUVWXY34679";
  char secret[9];
  for (size_t i = 0; i < sizeof(secret) - 1; ++i) {
    secret[i] = easyChars[esp_random() % (sizeof(easyChars) - 1)];
  }
  secret[sizeof(secret) - 1] = '\0';
  apPassword = secret;
  char token[17];
  snprintf(token, sizeof(token), "%08lX%08lX", (unsigned long)esp_random(), (unsigned long)esp_random());
  formToken = token;
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  if (!WiFi.softAP(apSsid.c_str(), apPassword.c_str())) {
    message = "Could not start setup Wi-Fi.";
    WiFi.mode(WIFI_OFF);
    return false;
  }
  if (!savedSsid.isEmpty()) WiFi.begin(savedSsid.c_str(), savedPassword.c_str());
  dns.start(53, "*", WiFi.softAPIP());
  if (!routesRegistered) {
    server.on("/", HTTP_GET, sendHome);
    server.on("/connect", HTTP_POST, connectWifi);
    server.on("/check", HTTP_POST, checkUpdate);
    server.on("/update", HTTP_POST, installUpdate);
    server.on("/finish", HTTP_POST, finishSetup);
    server.onNotFound([]() {
      if (!fromAp()) { server.send(403, "text/plain", "Use the device Wi-Fi network"); return; }
      server.sendHeader("Location", "http://192.168.4.1/");
      server.send(302, "text/plain", "");
    });
    routesRegistered = true;
  }
  server.begin();
  active = true;
  closeAt = 0;
  rebootAt = 0;
  releaseInfo = ReleaseInfo();
  message = "Open 192.168.4.1 on the device Wi-Fi.";
  return true;
}

void portalEnd() {
  if (!active) return;
  server.stop();
  dns.stop();
  WiFi.mode(WIFI_OFF);
  active = false;
  closeAt = 0;
  rebootAt = 0;
  releaseInfo = ReleaseInfo();
  apPassword = "";
  formToken = "";
  message = "";
}

void portalLoop() {
  if (!active) return;
  dns.processNextRequest();
  server.handleClient();
  if (rebootAt && (long)(millis() - rebootAt) >= 0) ESP.restart();
  if (closeAt && (long)(millis() - closeAt) >= 0) portalEnd();
}

bool portalActive() { return active; }
const String &portalSsid() { return apSsid; }
const String &portalPassword() { return apPassword; }
const String &portalMessage() { return message; }
