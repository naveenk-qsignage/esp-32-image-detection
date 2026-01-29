#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>

#include "secret.h"         // AWS_IOT_ENDPOINT, AWS_IOT_PORT, DEVICE_ID, AWS_CERT_*

// ===========================
// Wi-Fi credentials (same variables as your example)
// ===========================
#define  ssid WIFI_SSID
#define  password WIFI_PASSWORD


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

// ---------- Wi-Fi / AWS ----------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);                 // same variables as your code
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  WiFi.setSleep(false);
}

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

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  if (!initCamera()) {
    Serial.println("FATAL: camera init failed."); while (true) delay(1000);
  }

  connectWiFi();
  ensureTimeIST();
  connectAWS();

  lastPublishMs = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) { Serial.println("WiFi lost. Reconnecting..."); connectWiFi(); }

  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastMqttAttempt > MQTT_RETRY_MS) {
      lastMqttAttempt = now;
      connectAWS();
    }
  } else {
    mqttClient.loop();
  }

  unsigned long now = millis();
  if (now - lastPublishMs >= PUBLISH_INTERVAL_MS) {
    lastPublishMs = now;

    if (mqttClient.connected()) {
      uint8_t* img = nullptr; size_t len = 0;
      if (captureToPSRAM(img, len)) {
        publishImageBinary(img, len);
        free(img); // PSRAM or DRAM — free() is fine
      }
    } else {
      Serial.println("Skip publish: MQTT not connected.");
    }
  }
}
