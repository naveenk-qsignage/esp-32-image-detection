#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include "esp_sleep.h"

#include "secret.h"

// Wake every N minutes: connect WiFi + AWS, take one image, publish, then deep sleep.
#define WAKE_INTERVAL_MINUTES  20
#define WAKE_INTERVAL_US       ((uint64_t)(WAKE_INTERVAL_MINUTES) * 60ULL * 1000000ULL)         // AWS_IOT_ENDPOINT, AWS_IOT_PORT, DEVICE_ID, AWS_CERT_*, WIFI_CONFIG_URL

// ===========================
// Wi-Fi config from AWS: path wifi_config/{DEVICE_ID}.json (e.g. wifi_config/esp_01.json).
// Two ways: (1) HTTP: WIFI_CONFIG_URL + DEVICE_ID + WIFI_CONFIG_SUFFIX (S3 or API Gateway).
//           (2) Endpoint/MQTT: set WIFI_CONFIG_VIA_IOT in secret.h; device publishes to
//               wifi_config/request/{DEVICE_ID}, subscribes to wifi_config/response/{DEVICE_ID}; Lambda reads S3 and publishes response.
// ===========================
#ifndef WIFI_CONFIG_URL
#define WIFI_CONFIG_URL "https://qsignage-esp32-cam.s3.ap-south-1.amazonaws.com/wifi_config/"
#endif
#ifndef WIFI_CONFIG_SUFFIX
#define WIFI_CONFIG_SUFFIX ".json"
#endif
// Uncomment in secret.h to fetch WiFi config via IoT (MQTT) instead of HTTP:
// #define WIFI_CONFIG_VIA_IOT 1
#ifndef WIFI_CONFIG_VIA_IOT
#define WIFI_CONFIG_VIA_IOT 0
#endif
#define WIFI_CONFIG_REQUEST_TOPIC  "wifi_config/request/"
#define WIFI_CONFIG_RESPONSE_TOPIC "wifi_config/response/"

#define CLOUD_WIFI_TIMEOUT_MS 15000  // try cloud WiFi this long before falling back to predefined

String wifiSsidFromCloud;
String wifiPasswordFromCloud;
bool wifiConfigFromCloud = false;

#define BOOTSTRAP_SSID    WIFI_SSID
#define BOOTSTRAP_PASSWORD WIFI_PASSWORD

// NVS (Preferences): store AWS WiFi config so it persists and is overwritten when AWS config changes
#define NVS_WIFI_NAMESPACE "wifi"
#define NVS_KEY_SSID      "ssid"
#define NVS_KEY_PASS      "pass"

static void saveWifiConfigToNVS(const String& ssid, const String& pass) {
  Preferences prefs;
  if (!prefs.begin(NVS_WIFI_NAMESPACE, false)) return;
  prefs.putString(NVS_KEY_SSID, ssid);
  prefs.putString(NVS_KEY_PASS, pass);
  prefs.end();
  Serial.printf("Saved AWS WiFi config to NVS: SSID=%s\n", ssid.c_str());
}

// Load last saved AWS WiFi config from NVS. Returns true if valid config was found.
static bool loadWifiConfigFromNVS(String& ssid, String& pass) {
  Preferences prefs;
  if (!prefs.begin(NVS_WIFI_NAMESPACE, true)) return false;
  if (!prefs.isKey(NVS_KEY_SSID)) { prefs.end(); return false; }
  ssid = prefs.getString(NVS_KEY_SSID, "");
  pass = prefs.getString(NVS_KEY_PASS, "");
  prefs.end();
  return (ssid.length() > 0);
}

// Default WiFi list when no S3 config for this DEVICE_ID (tried in order)
static const struct { const char* ssid; const char* pass; } defaultWifiList[] = {
  { BOOTSTRAP_SSID, BOOTSTRAP_PASSWORD },
  // Add more: { "OtherSSID", "OtherPassword" },
};
#define DEFAULT_WIFI_COUNT ((int)(sizeof(defaultWifiList) / sizeof(defaultWifiList[0])))

// Try each default network until one connects. Returns true if connected.
static bool connectWithDefaultWifiList() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  const unsigned long tryMs = 12000;  // per network
  for (int i = 0; i < DEFAULT_WIFI_COUNT; i++) {
    Serial.printf("Trying default WiFi [%d/%d] %s ... ", i + 1, DEFAULT_WIFI_COUNT, defaultWifiList[i].ssid);
    WiFi.disconnect(true);
    delay(300);
    WiFi.begin(defaultWifiList[i].ssid, defaultWifiList[i].pass);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < tryMs) {
      delay(300);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi connected (default list): " + WiFi.localIP().toString());
      return true;
    }
    Serial.println(" timeout");
  }
  return false;
}

// Try to connect to given SSID/password with timeout. Returns true if connected.
static bool tryConnectWifiWithTimeout(const char* ssid, const char* pass, unsigned long timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true);
  delay(300);
  WiFi.begin(ssid, pass);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    return true;
  }
  Serial.println(" timeout");
  return false;
}

// ===== AI-Thinker ESP32-CAM pins =====
#ifndef CAMERA_MODEL_AI_THINKER
#define CAMERA_MODEL_AI_THINKER
#endif
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// ===== MQTT / AWS limits =====
#define MQTT_MAX_IMAGE_BYTES (120 * 1024)   // keep < 128 KB (AWS IoT)
#define PUBLISH_CHUNK_SZ     2048

// ===== Cadence =====
const unsigned long PUBLISH_INTERVAL_MS = 1UL * 60UL * 1000UL;  // 1 minutes
const unsigned long MQTT_RETRY_MS       = 3000UL;
unsigned long lastPublishMs   = 0;
unsigned long lastMqttAttempt = 0;

// ===== Networking =====
// WiFi config via MQTT: response payload and flag (set in callback)
#define WIFI_CONFIG_RESPONSE_BUF_SZ 512
static char     wifiConfigResponseBuf[WIFI_CONFIG_RESPONSE_BUF_SZ];
static bool     wifiConfigResponseReceived = false;

WiFiClientSecure wifiClient;
PubSubClient     mqttClient(wifiClient);

// ---------- Camera init: mirror your settings ----------
bool initCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk    = XCLK_GPIO_NUM;
  config.pin_pclk    = PCLK_GPIO_NUM;
  config.pin_vsync   = VSYNC_GPIO_NUM;
  config.pin_href    = HREF_GPIO_NUM;
  config.pin_sccb_sda= SIOD_GPIO_NUM;
  config.pin_sccb_scl= SIOC_GPIO_NUM;
  config.pin_pwdn    = PWDN_GPIO_NUM;
  config.pin_reset   = RESET_GPIO_NUM;

  // exact like your snippet
  config.xclk_freq_hz = 20000000;             // 20 MHz
  config.frame_size   = FRAMESIZE_UXGA;       // start big (driver alloc)
  config.pixel_format = PIXFORMAT_JPEG;       // streaming/JPEG
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;   // PSRAM as you asked
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 25;               // your pattern
      config.fb_count     = 2;                // double-buffer
      config.grab_mode    = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size   = FRAMESIZE_SVGA;   // fallback if no PSRAM
      config.fb_location  = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count   = 2;
#endif
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  // same runtime tweak you used: drop to QVGA for faster/leaner frames
  sensor_t* s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_SXGA);     // 1024 x 1280 for the actual captures
  }

  return true;
}

// ---------- Parse WiFi config JSON into wifiSsidFromCloud / wifiPasswordFromCloud / wifiConfigFromCloud ----------
// Expects {"ssid":"...", "password":"..."} or {"wifi_ssid":"...", "wifi_password":"..."}. Returns true on success.
static bool parseWifiConfigJson(const char* payload) {
  if (!payload || !payload[0]) return false;
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, payload)) return false;
  const char* ssidStr = doc["ssid"].as<const char*>();
  if (!ssidStr || !ssidStr[0]) ssidStr = doc["wifi_ssid"].as<const char*>();
  const char* passStr = doc["password"].as<const char*>();
  if (!passStr) passStr = doc["wifi_password"].as<const char*>();
  if (!ssidStr || !ssidStr[0]) return false;
  wifiSsidFromCloud = String(ssidStr);
  wifiPasswordFromCloud = String(passStr ? passStr : "");
  wifiConfigFromCloud = true;
  return true;
}

// ---------- Fetch WiFi config from AWS (wifi_config/{DEVICE_ID}.json) ----------
#define AWS_FETCH_MAX_RETRIES 5
#define AWS_FETCH_RETRY_DELAY_MS 2000
// Call only when already connected (predefined WiFi). Expects JSON:
//   {"ssid":"...", "password":"..."}  or  {"wifi_ssid":"...", "wifi_password":"..."}
bool fetchWifiConfigFromAWS() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("fetchWifiConfig: not connected.");
    return false;
  }

  String url = String(WIFI_CONFIG_URL) + DEVICE_ID + WIFI_CONFIG_SUFFIX;
  Serial.printf("Fetching WiFi config from AWS (max %d tries): %s\n", AWS_FETCH_MAX_RETRIES, url.c_str());

  for (int attempt = 1; attempt <= AWS_FETCH_MAX_RETRIES; attempt++) {
    Serial.printf("  Attempt %d/%d ... ", attempt, AWS_FETCH_MAX_RETRIES);

    WiFiClientSecure httpSecure;
    httpSecure.setCACert(AWS_CERT_CA);
    httpSecure.setTimeout(10);

    HTTPClient http;
    if (!http.begin(httpSecure, url)) {
      Serial.println("HTTP begin failed.");
      if (attempt < AWS_FETCH_MAX_RETRIES) { delay(AWS_FETCH_RETRY_DELAY_MS); continue; }
      return false;
    }

    http.setTimeout(10000);
    int code = http.GET();

    if (code != HTTP_CODE_OK) {
      Serial.printf("HTTP %d\n", code);
      http.end();
      if (attempt < AWS_FETCH_MAX_RETRIES) { delay(AWS_FETCH_RETRY_DELAY_MS); continue; }
      return false;
    }

    String payload = http.getString();
    http.end();

    if (!parseWifiConfigJson(payload.c_str())) {
      Serial.println("JSON parse failed or no ssid.");
      if (attempt < AWS_FETCH_MAX_RETRIES) { delay(AWS_FETCH_RETRY_DELAY_MS); continue; }
      return false;
    }

    saveWifiConfigToNVS(wifiSsidFromCloud, wifiPasswordFromCloud);  // overwrite NVS when AWS config changes
    Serial.println("OK");
    Serial.printf("--- Fetched data from AWS (wifi_config/%s.json) ---\n", DEVICE_ID);
    Serial.printf("  SSID: %s\n", wifiSsidFromCloud.c_str());
    Serial.println("----------------------------");
    return true;
  }

  Serial.println("All retries failed.");
  return false;
}

#if WIFI_CONFIG_VIA_IOT
// MQTT callback: store payload when topic is wifi_config/response/{DEVICE_ID}
static void wifiConfigMqttCallback(char* topic, byte* payload, unsigned int length) {
  String responseTopic = String(WIFI_CONFIG_RESPONSE_TOPIC) + DEVICE_ID;
  if (responseTopic != topic) return;
  wifiConfigResponseReceived = true;
  size_t copyLen = (length < (unsigned)(WIFI_CONFIG_RESPONSE_BUF_SZ - 1)) ? length : (WIFI_CONFIG_RESPONSE_BUF_SZ - 1);
  memcpy(wifiConfigResponseBuf, payload, copyLen);
  wifiConfigResponseBuf[copyLen] = '\0';
  Serial.printf("JSON received on response_topic: %s\n", (const char*)wifiConfigResponseBuf);
}

// Fetch WiFi config via IoT endpoint (MQTT): publish to wifi_config/request/{DEVICE_ID}, subscribe to wifi_config/response/{DEVICE_ID}.
// Call only when MQTT is already connected. Lambda (triggered by IoT Rule) reads S3 and publishes response.
#define WIFI_CONFIG_IOT_WAIT_MS 12000  // allow time for Lambda cold start
bool fetchWifiConfigFromIoT() {
  if (!mqttClient.connected()) {
    Serial.println("fetchWifiConfigFromIoT: MQTT not connected.");
    return false;
  }
  wifiConfigResponseReceived = false;
  wifiConfigResponseBuf[0] = '\0';

  String reqTopic  = String(WIFI_CONFIG_REQUEST_TOPIC)  + DEVICE_ID;
  String respTopic = String(WIFI_CONFIG_RESPONSE_TOPIC) + DEVICE_ID;

  mqttClient.setCallback(wifiConfigMqttCallback);
  if (!mqttClient.subscribe(respTopic.c_str(), 0)) {
    Serial.println("subscribe response topic failed");
    return false;
  }
  // Give broker a moment to register subscription before publishing request
  delay(300);
  Serial.printf("Requesting WiFi config via IoT: %s -> %s\n", reqTopic.c_str(), respTopic.c_str());
  if (!mqttClient.publish(reqTopic.c_str(), "", false)) {
    Serial.println("publish request failed");
    return false;
  }

  unsigned long start = millis();
  while (!wifiConfigResponseReceived && (millis() - start) < WIFI_CONFIG_IOT_WAIT_MS) {
    mqttClient.loop();
    delay(50);
  }
  mqttClient.setCallback(nullptr);

  if (!wifiConfigResponseReceived) {
    Serial.println("WiFi config response timeout.");
    return false;
  }
  Serial.printf("JSON received on %s: %s\n", respTopic.c_str(), wifiConfigResponseBuf);
  if (!parseWifiConfigJson(wifiConfigResponseBuf)) {
    Serial.println("WiFi config response: invalid JSON.");
    return false;
  }
  saveWifiConfigToNVS(wifiSsidFromCloud, wifiPasswordFromCloud);  // overwrite NVS when AWS config changes
  Serial.printf("--- Fetched WiFi config via IoT (wifi_config/%s) ---\n", DEVICE_ID);
  Serial.printf("  SSID: %s\n", wifiSsidFromCloud.c_str());
  Serial.println("----------------------------");
  return true;
}
#endif

// ---------- Wi-Fi: ensure connected (predefined first, then try cloud from AWS if we have config) ----------
// Forward: fetch from AWS (when already connected), replace NVS if new data, try cloud WiFi or stay/default
void fetchFromAWSAndMaybeSwitch();

// Connection order: 1) Try NVS (last AWS config), 2) Try default list, 3) Fetch from AWS and replace NVS if new data, 4) If NVS and AWS both fail use default.
void ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) return;

  // Step 1: Try NVS first (last saved AWS config)
  String nvsSsid, nvsPass;
  if (loadWifiConfigFromNVS(nvsSsid, nvsPass)) {
    Serial.println("Trying saved NVS WiFi (last AWS config)...");
    if (tryConnectWifiWithTimeout(nvsSsid.c_str(), nvsPass.c_str(), CLOUD_WIFI_TIMEOUT_MS)) {
      Serial.println("Connected via NVS.");
      fetchFromAWSAndMaybeSwitch();  // fetch from AWS, replace NVS if new data, try cloud WiFi or stay/default
      return;
    }
  }
  // Step 2: NVS failed or empty - try default list
  Serial.println("NVS failed or not in range; trying default WiFi...");
  if (!connectWithDefaultWifiList()) return;
  Serial.println("Connected via default.");
  // Step 3: Fetch from AWS; if new data replace NVS, then try cloud WiFi or stay on default
  fetchFromAWSAndMaybeSwitch();
}

// Call only when already connected (NVS or default). Fetch from AWS; if success save to NVS (overwrite) and try cloud WiFi; if cloud fail use default. If AWS fail stay on current WiFi.
#if WIFI_CONFIG_VIA_IOT
void fetchFromAWSAndMaybeSwitch() {
  Serial.println("Fetching WiFi config from AWS via IoT (MQTT)...");
  ensureTimeIST();
  if (!connectAWS()) {
    Serial.println("MQTT connect failed; staying on current WiFi (NVS or default).");
    return;
  }
  if (fetchWifiConfigFromIoT()) {
    mqttClient.disconnect();
    WiFi.disconnect(true);
    delay(500);
    Serial.print("Trying cloud WiFi (from AWS)... ");
    if (tryConnectWifiWithTimeout(wifiSsidFromCloud.c_str(), wifiPasswordFromCloud.c_str(), CLOUD_WIFI_TIMEOUT_MS)) return;
    Serial.println("Cloud WiFi not found; falling back to default.");
    wifiConfigFromCloud = false;
    connectWithDefaultWifiList();
  } else {
    mqttClient.disconnect();
    Serial.println("No AWS config from IoT; staying on current WiFi (NVS or default).");
  }
}
#else
void fetchFromAWSAndMaybeSwitch() {
  Serial.println("Fetching WiFi config from AWS (HTTP)...");
  if (fetchWifiConfigFromAWS()) {
    WiFi.disconnect(true);
    delay(500);
    Serial.print("Trying cloud WiFi (from AWS)... ");
    if (tryConnectWifiWithTimeout(wifiSsidFromCloud.c_str(), wifiPasswordFromCloud.c_str(), CLOUD_WIFI_TIMEOUT_MS)) return;
    Serial.println("Cloud WiFi not found; falling back to default.");
    wifiConfigFromCloud = false;
    connectWithDefaultWifiList();
  } else {
    Serial.println("No AWS config; staying on current WiFi (NVS or default).");
  }
}
#endif

bool connectAWS() {
  if (mqttClient.connected()) return true;
  wifiClient.setCACert(AWS_CERT_CA);
  wifiClient.setCertificate(AWS_CERT_CRT);
  wifiClient.setPrivateKey(AWS_CERT_PRIVATE);
  mqttClient.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
  mqttClient.setKeepAlive(60);
  mqttClient.setBufferSize(2048);
  Serial.printf("MQTT connecting to %s ... ", AWS_IOT_ENDPOINT);
  bool ok = mqttClient.connect(DEVICE_ID);
  Serial.println(ok ? "OK" : "FAIL");
  if (!ok) { Serial.print("state="); Serial.println(mqttClient.state()); }
  return ok;
}

// ---------- Time / topic ----------
void ensureTimeIST() {
  configTime(19800, 0, "pool.ntp.org", "time.nist.gov"); // IST +5:30
  for (int i=0;i<20;i++){ if (time(nullptr) > 1700000000) break; delay(250); }
}
String topicWithDt() {
  time_t now = time(nullptr);
  struct tm tm; localtime_r(&now, &tm);
  char dt[16]; strftime(dt, sizeof(dt), "%Y%m%d_%H%M", &tm); // YYYYMMDD_HHMM
  return String("ESP32_AWS_ENDPOINT/") + DEVICE_ID + "/" + dt + ".jpg";
}

// ---------- Capture to PSRAM (copy) ----------
bool captureToPSRAM(uint8_t*& outBuf, size_t& outLen) {
  outBuf = nullptr; outLen = 0;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { Serial.println("Capture failed."); return false; }

  if (fb->len > MQTT_MAX_IMAGE_BYTES) {
    Serial.printf("Frame too large: %u\n", (unsigned)fb->len);
    esp_camera_fb_return(fb);
    return false;
  }

  // Even though fb->buf already lives in PSRAM (fb_location=PSRAM),
  // we make our OWN PSRAM copy so we can return the fb immediately.
  uint8_t* copy = (uint8_t*) ps_malloc(fb->len);
  if (!copy) copy = (uint8_t*) heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM);
  if (!copy) copy = (uint8_t*) malloc(fb->len);           // DRAM fallback
  if (!copy) { Serial.println("ps_malloc failed"); esp_camera_fb_return(fb); return false; }

  memcpy(copy, fb->buf, fb->len);
  outBuf = copy; outLen = fb->len;

  Serial.printf("Captured %dx%d, %u bytes → copy @%p (PSRAM preferred)\n",
                fb->width, fb->height, (unsigned)fb->len, (void*)copy);

  esp_camera_fb_return(fb);   // free the camera fb immediately
  return true;
}

// ---------- Publish ----------
bool publishImageBinary(const uint8_t* data, size_t len) {
  if (!data || len == 0 || !mqttClient.connected()) return false;
  String topic = topicWithDt();

  if (!mqttClient.beginPublish(topic.c_str(), len, false)) {
    Serial.println("beginPublish failed"); return false;
  }
  size_t sent = 0;
  while (sent < len) {
    size_t n = min((size_t)PUBLISH_CHUNK_SZ, len - sent);
    size_t w = mqttClient.write(data + sent, n);
    if (w == 0) { Serial.println("short write"); break; }
    sent += w;
  }
  bool ok = (sent == len) && mqttClient.endPublish();
  Serial.printf("Publish %s : %s (%u bytes)\n", topic.c_str(), ok ? "OK" : "FAIL", (unsigned)len);
  return ok;
}

// Wake every WAKE_INTERVAL_MINUTES: connect WiFi + AWS, one image, publish, deep sleep.
static void goToSleep();

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.printf("Wake: connect WiFi + AWS, one image, publish, sleep %u min.\n", (unsigned)WAKE_INTERVAL_MINUTES);

  if (!initCamera()) {
    Serial.println("FATAL: camera init failed."); goToSleep();
  }

  ensureWiFiConnected();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("FATAL: no WiFi connected."); goToSleep();
  }

  ensureTimeIST();
  if (!connectAWS()) {
    Serial.println("MQTT connect failed; skipping publish."); goToSleep();
  }

  // Single image capture and publish
  uint8_t* img = nullptr; size_t len = 0;
  if (captureToPSRAM(img, len)) {
    if (mqttClient.connected()) {
      publishImageBinary(img, len);
    }
    free(img);
  } else {
    Serial.println("Capture failed.");
  }

  mqttClient.disconnect();
  WiFi.disconnect(true);
  Serial.printf("Going to deep sleep for %u minutes...\n", (unsigned)WAKE_INTERVAL_MINUTES);
  goToSleep();
}

void loop() {
  // Never reached: setup() calls goToSleep() and device reboots on wake.
  delay(1000);
}

static void goToSleep() {
  esp_sleep_enable_timer_wakeup(WAKE_INTERVAL_US);
  esp_deep_sleep_start();
}
