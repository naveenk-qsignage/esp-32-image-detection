# esp-32-image-detection

ESP32-CAM sketch: captures images and publishes them to AWS IoT. WiFi config is fetched from AWS (S3 or API) per `DEVICE_ID`; fallback is a default WiFi list. You can route images from IoT Core to S3 with an IoT Rule — see **[aws-iot-to-s3/README.md](aws-iot-to-s3/README.md)**. A **web application** can get/delete image data and put WiFi config — see **[aws-web-app/README.md](aws-web-app/README.md)**.

---

## Arduino IDE setup

1. **Board:** ESP32 Dev Module (or AI Thinker ESP32-CAM). Install ESP32 board support if needed.
2. **Libraries:** Sketch → Include Library → Manage Libraries → install **ArduinoJson** and **PubSubClient**.
3. **secret.h:** Create `secret.h` in the same folder as the `.ino` with:
   - `WIFI_SSID`, `WIFI_PASSWORD` (bootstrap / default list)
   - `AWS_IOT_ENDPOINT`, `AWS_IOT_PORT`, `DEVICE_ID`
   - `AWS_CERT_CA`, `AWS_CERT_CRT`, `AWS_CERT_PRIVATE`
4. **Upload:** Connect the board, select the COM port, then Sketch → Upload. Use Serial Monitor at **115200** baud.

---

## WiFi config from AWS (by device ID)

- **Public S3:** If `wifi_config/*` in your bucket is public read, the default URL in the sketch works. Put `{DEVICE_ID}.json` at `wifi_config/esp_01.json` etc.
- **Private S3 (HTTP + Lambda):** API Gateway + Lambda reads S3. See **[aws-wifi-config-api/README.md](aws-wifi-config-api/README.md)**. Set `WIFI_CONFIG_URL` in **secret.h** to your API URL.
- **Private S3 (HTTP, no Lambda):** API Gateway talks to S3 directly. See **[aws-wifi-config-s3/README.md](aws-wifi-config-s3/README.md)**. Same `WIFI_CONFIG_URL` / `WIFI_CONFIG_SUFFIX` pattern; URL includes bucket and `wifi_config/`.
- **Endpoint way (MQTT):** Device publishes/subscribes on IoT topics; Lambda reads S3 and publishes JSON. See **[aws-wifi-config-iot/README.md](aws-wifi-config-iot/README.md)**. In **secret.h** set `#define WIFI_CONFIG_VIA_IOT 1`.

JSON format (in S3 or API response):

```json
{ "ssid": "YourNetworkName", "password": "YourPassword" }
```

On boot the device connects using the **default WiFi list** (in the sketch), fetches config from AWS (up to 5 retries), then reconnects with cloud config if found; otherwise it stays on the default list.

---

## Default WiFi list

In `esp-32-image-detection.ino`, edit `defaultWifiList[]` to add more networks. The first entry is the bootstrap from `secret.h`. The device tries each in order when no AWS config is available.
