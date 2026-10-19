#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <BH1750.h>
#include "DHT.h"
#include <time.h>
#include "esp_camera.h"
#include "img_converters.h"

const char* wifiPortalName = "SmartEnclosure-Setup";

// Ganti dengan domain Vercel kamu. Jangan pakai https:// di sini.
const char* detection_host = "smart-enclosure.vercel.app";
const uint16_t detection_port = 443;
const char* detection_path = "/api/detect";
const char* health_path = "/api/health";
const char* notify_path = "/api/notify";

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

const char* topic_status    = "smart-enclosure-reptil/device01/status";
const char* topic_sensor    = "smart-enclosure-reptil/device01/sensor";
const char* topic_control   = "smart-enclosure-reptil/device01/control";
const char* topic_detection = "smart-enclosure-reptil/device01/detection";

#define RELAY1_PIN      1
#define RELAY2_PIN      2
#define HUMIDIFIER_PIN 19
#define RELAY_ON  LOW
#define RELAY_OFF HIGH
#define MOSFET_ON  HIGH
#define MOSFET_OFF LOW

#define DHT_WARM_PIN 47  // dekat sumber panas - HANYA dipakai untuk baca suhu
#define DHT_HUMID_PIN 14   // TODO: sesuaikan dengan pin fisik yang kamu pakai - dekat sumber lembap, HANYA dipakai untuk baca kelembapan
#define DHT_TYPE DHT22
#define BH1750_SDA 21
#define BH1750_SCL 20
#define WATER_LEVEL_PIN 3
// WAJIB KALIBRASI SENDIRI: upload firmware ini dulu, buka Serial Monitor,
// celupkan sensor ke kondisi kering -> catat angka "raw", celupkan ke air
// penuh -> catat angka "raw" lagi, lalu isi 2 nilai di bawah ini.
const int waterLevelDryRaw = 300;   // TODO: ganti dengan hasil kalibrasi kering
const int waterLevelWetRaw = 2000;   // TODO: ganti dengan hasil kalibrasi basah penuh
const float waterLevelLowThreshold = 20.0; // % - di bawah ini dianggap rendah

#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5
#define Y9_GPIO_NUM      16
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      18
#define Y6_GPIO_NUM      12
#define Y5_GPIO_NUM      10
#define Y4_GPIO_NUM      8
#define Y3_GPIO_NUM      9
#define Y2_GPIO_NUM      11
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13

WiFiClient espClient;
PubSubClient client(espClient);
DHT dhtWarm(DHT_WARM_PIN, DHT_TYPE);
DHT dhtHumid(DHT_HUMID_PIN, DHT_TYPE);
BH1750 lightMeter;

unsigned long lastPublish = 0;
const unsigned long publishInterval = 5000;
const unsigned long detectionPreLightDelay = 5000;
const unsigned long autoControlInterval = 5000;
unsigned long lastAutoControl = 0;

const long gmtOffsetSec = 7 * 3600;
const int daylightOffsetSec = 0;
const int detectionScheduleHours[4] = {6, 12, 18, 22};
int lastScheduledRunDay[4] = {-1, -1, -1, -1};
bool timeReady = false;

bool relay1State = false;
bool relay2State = false;
bool humidifierState = false;

bool activeSpeciesReady = false;
String activeSpecies = "";
float activeWarmMin = NAN;
float activeWarmMax = NAN;
float activeHumidityMin = NAN;
float activeHumidityMax = NAN;
bool warmRangeAlerted = false;
bool humidityRangeAlerted = false;

float temperatureValue = NAN;
float humidityValue = NAN;
float luxValue = -1;
float waterLevelValue = NAN;
bool waterLevelLow = false;
bool humidifierBlockedAlerted = false;
bool bh1750Ready = false;
bool cameraReady = false;
bool detectRequest = false;

void publishStatus();
void publishSensorData();
void sendNotification(const String &title, const String &body, const String &severity);

void setRelay1(bool state) {
  if (relay1State == state) return;
  relay1State = state;
  digitalWrite(RELAY1_PIN, state ? RELAY_ON : RELAY_OFF);
  Serial.print("Heatpad: ");
  Serial.println(state ? "ON" : "OFF");
  sendNotification(state ? "Heatpad Menyala" : "Heatpad Mati",
                    state ? "Heatpad diaktifkan." : "Heatpad dimatikan.",
                    "info");
}

void setRelay2(bool state) {
  if (relay2State == state) return;
  relay2State = state;
  digitalWrite(RELAY2_PIN, state ? RELAY_ON : RELAY_OFF);
  Serial.print("Lampu: ");
  Serial.println(state ? "ON" : "OFF");
  sendNotification(state ? "Lampu Menyala" : "Lampu Mati",
                    state ? "Lampu diaktifkan." : "Lampu dimatikan.",
                    "info");
}

void setHumidifier(bool state) {
  if (state && waterLevelLow) {
    if (!humidifierBlockedAlerted) {
      humidifierBlockedAlerted = true;
      Serial.println("Humidifier tidak bisa dinyalakan: water level rendah");
      sendNotification("Humidifier Diblokir", "Water level rendah, humidifier tidak bisa dinyalakan.", "warning");
    }
    return;
  }

  humidifierBlockedAlerted = false;

  if (humidifierState == state) return;
  humidifierState = state;
  digitalWrite(HUMIDIFIER_PIN, state ? MOSFET_ON : MOSFET_OFF);
  Serial.print("Humidifier: ");
  Serial.println(state ? "ON" : "OFF");
  sendNotification(state ? "Humidifier Menyala" : "Humidifier Mati",
                    state ? "Humidifier diaktifkan." : "Humidifier dimatikan.",
                    "info");
}

void allActuatorsOff() {
  setRelay1(false);
  setRelay2(false);
  setHumidifier(false);
}

void publishDetectionFailed(const char* message) {
  String payload = "{";
  payload += "\"status\":\"failed\",";
  payload += "\"species\":null,";
  payload += "\"confidence\":0,";
  payload += "\"message\":\"";
  payload += message;
  payload += "\"}";
  client.publish(topic_detection, payload.c_str());
  Serial.print("Publish detection failed: ");
  Serial.println(payload);
}

void sendNotification(const String &title, const String &body, const String &severity) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi tidak terhubung saat kirim notifikasi");
    return;
  }

  WiFiClientSecure notifyClient;
  notifyClient.setInsecure();
  notifyClient.setTimeout(10000);

  if (!notifyClient.connect(detection_host, detection_port)) {
    Serial.println("Gagal connect ke /api/notify");
    return;
  }

  String payload = "{\"title\":\"" + title + "\",\"body\":\"" + body + "\",\"severity\":\"" + severity + "\"}";

  notifyClient.print("POST ");
  notifyClient.print(notify_path);
  notifyClient.print(" HTTP/1.1\r\n");
  notifyClient.print("Host: ");
  notifyClient.print(detection_host);
  notifyClient.print("\r\n");
  notifyClient.print("User-Agent: ESP32S3-SmartEnclosure\r\n");
  notifyClient.print("Content-Type: application/json\r\n");
  notifyClient.print("Content-Length: ");
  notifyClient.print(payload.length());
  notifyClient.print("\r\n");
  notifyClient.print("Connection: close\r\n\r\n");
  notifyClient.print(payload);

  unsigned long waitStart = millis();
  while (!notifyClient.available() && millis() - waitStart < 10000) {
    client.loop();
    delay(10);
  }

  if (notifyClient.available()) {
    String statusLine = notifyClient.readStringUntil('\n');
    statusLine.trim();
    Serial.print("Notify response: ");
    Serial.println(statusLine);
  } else {
    Serial.println("Timeout menunggu response /api/notify");
  }

  notifyClient.stop();
}

void setupCamera() {
  Serial.println("Setup kamera...");
  Serial.print("PSRAM Found: ");
  Serial.println(psramFound() ? "YES" : "NO");
  Serial.print("Free Heap: ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("Free PSRAM: ");
  Serial.println(ESP.getFreePsram());

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.frame_size = FRAMESIZE_VGA;
    config.fb_count = 2;
    Serial.println("Frame buffer memakai PSRAM, resolusi VGA");
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_count = 1;
    Serial.println("PSRAM tidak ditemukan, frame buffer memakai DRAM, resolusi QVGA");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed. Error: 0x%x\n", err);
    cameraReady = false;
    return;
  }

  cameraReady = true;
  Serial.println("Camera init berhasil");

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    Serial.printf("Sensor PID: 0x%02x\n", s->id.PID);
    Serial.printf("Sensor VER: 0x%02x\n", s->id.VER);
    s->set_brightness(s, 1);
    s->set_contrast(s, 2);
    s->set_saturation(s, 0);
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_whitebal(s, 1);
    if (s->set_sharpness) s->set_sharpness(s, 2);
    if (s->set_denoise) s->set_denoise(s, 1);
    if (s->set_gainceiling) s->set_gainceiling(s, GAINCEILING_4X);
    Serial.println("Pengaturan kamera diterapkan");
  }
}

void setupTime() {
  Serial.println("Sinkronisasi waktu NTP...");
  configTime(gmtOffsetSec, daylightOffsetSec, "pool.ntp.org", "time.google.com", "time.windows.com");
  struct tm timeinfo;
  for (int i = 0; i < 20; i++) {
    if (getLocalTime(&timeinfo)) {
      timeReady = true;
      Serial.print("Waktu tersinkron: ");
      Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
      return;
    }
    delay(500);
  }
  timeReady = false;
  Serial.println("NTP belum berhasil, scheduler akan mencoba lagi di loop");
}

void checkScheduledDetection() {
  if (!timeReady) {
    struct tm retryInfo;
    if (getLocalTime(&retryInfo)) {
      timeReady = true;
      Serial.println("NTP tersinkron saat loop");
    } else {
      return;
    }
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    timeReady = false;
    return;
  }

  for (int i = 0; i < 4; i++) {
    if (timeinfo.tm_hour == detectionScheduleHours[i] && timeinfo.tm_min < 5 && lastScheduledRunDay[i] != timeinfo.tm_yday) {
      lastScheduledRunDay[i] = timeinfo.tm_yday;
      Serial.println("Scheduler memulai deteksi spesies");
      setRelay2(true);
      publishStatus();
      client.publish(topic_detection, "{\"status\":\"processing\",\"message\":\"scheduled_detection_started\"}");
      detectRequest = true;
    }
  }
}

float extractJsonNumber(const String &json, const char* key, float fallbackValue) {
  String token = "\"";
  token += key;
  token += "\":";
  int start = json.indexOf(token);
  if (start < 0) return fallbackValue;
  start += token.length();
  while (start < json.length() && (json[start] == ' ' || json[start] == '\t')) start++;
  int end = start;
  while (end < json.length()) {
    char c = json[end];
    if (!((c >= '0' && c <= '9') || c == '.' || c == '-')) break;
    end++;
  }
  if (end <= start) return fallbackValue;
  return json.substring(start, end).toFloat();
}

String extractJsonString(const String &json, const char* key) {
  String token = "\"";
  token += key;
  token += "\":\"";
  int start = json.indexOf(token);
  if (start < 0) return "";
  start += token.length();
  int end = json.indexOf("\"", start);
  if (end < 0) return "";
  return json.substring(start, end);
}

bool checkDetectionApiHealth() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi tidak terhubung saat cek detection API");
    return false;
  }

  WiFiClientSecure healthClient;
  healthClient.setInsecure();
  healthClient.setTimeout(12000);

  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Cek detection API health: https://");
  Serial.print(detection_host);
  Serial.println(health_path);

  if (!healthClient.connect(detection_host, detection_port)) {
    Serial.println("Detection API health gagal connect. Cek domain Vercel dan koneksi internet.");
    return false;
  }

  healthClient.print("GET ");
  healthClient.print(health_path);
  healthClient.print(" HTTP/1.1\r\n");
  healthClient.print("Host: ");
  healthClient.print(detection_host);
  healthClient.print("\r\n");
  healthClient.print("User-Agent: ESP32S3-SmartEnclosure\r\n");
  healthClient.print("Connection: close\r\n\r\n");

  unsigned long waitStart = millis();
  while (!healthClient.available() && millis() - waitStart < 12000) {
    client.loop();
    delay(10);
  }

  if (!healthClient.available()) {
    Serial.println("Detection API health timeout tanpa response");
    healthClient.stop();
    return false;
  }

  String statusLine = healthClient.readStringUntil('\n');
  statusLine.trim();
  Serial.print("Detection API health status: ");
  Serial.println(statusLine);
  healthClient.stop();
  return statusLine.indexOf(" 200 ") > 0 || statusLine.endsWith(" 200");
}

bool sendJpgToDetectionApi(uint8_t *jpg_buf, size_t jpg_len, String &response) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi tidak terhubung saat kirim JPG");
    return false;
  }

  WiFiClientSecure apiClient;
  apiClient.setInsecure();
  apiClient.setTimeout(30000);

  Serial.print("Mengirim JPG ke detection API: https://");
  Serial.print(detection_host);
  Serial.println(detection_path);

  if (!apiClient.connect(detection_host, detection_port)) {
    Serial.println("Gagal connect ke Vercel detection API.");
    return false;
  }

  Serial.print("Payload JPG size: ");
  Serial.print(jpg_len);
  Serial.println(" bytes");

  apiClient.print("POST ");
  apiClient.print(detection_path);
  apiClient.print(" HTTP/1.1\r\n");
  apiClient.print("Host: ");
  apiClient.print(detection_host);
  apiClient.print("\r\n");
  apiClient.print("User-Agent: ESP32S3-SmartEnclosure\r\n");
  apiClient.print("Content-Type: image/jpeg\r\n");
  apiClient.print("Content-Length: ");
  apiClient.print(jpg_len);
  apiClient.print("\r\n");
  apiClient.print("Connection: close\r\n\r\n");

  size_t totalSent = 0;
  const size_t chunkSize = 1024;
  while (totalSent < jpg_len) {
    size_t bytesLeft = jpg_len - totalSent;
    size_t bytesToWrite = bytesLeft < chunkSize ? bytesLeft : chunkSize;
    size_t written = apiClient.write(jpg_buf + totalSent, bytesToWrite);
    if (written == 0) {
      Serial.println("Gagal menulis body JPG ke koneksi detection API");
      apiClient.stop();
      return false;
    }
    totalSent += written;
    delay(1);
  }

  apiClient.flush();
  Serial.print("Total JPG terkirim: ");
  Serial.print(totalSent);
  Serial.println(" bytes");

  unsigned long waitStart = millis();
  while (!apiClient.available() && millis() - waitStart < 90000) {
    client.loop();
    delay(10);
  }

  if (!apiClient.available()) {
    Serial.println("Timeout menunggu response detection API");
    apiClient.stop();
    return false;
  }

  String statusLine = apiClient.readStringUntil('\n');
  statusLine.trim();
  Serial.print("HTTP Status Detection API: ");
  Serial.println(statusLine);

  int httpCode = -1;
  int firstSpace = statusLine.indexOf(' ');
  if (firstSpace >= 0 && statusLine.length() >= firstSpace + 4) {
    httpCode = statusLine.substring(firstSpace + 1, firstSpace + 4).toInt();
  }

  while (apiClient.available()) {
    String headerLine = apiClient.readStringUntil('\n');
    headerLine.trim();
    if (headerLine.length() == 0) break;
  }

  response = "";
  unsigned long lastData = millis();
  while (apiClient.connected() || apiClient.available()) {
    while (apiClient.available()) {
      char c = apiClient.read();
      if (response.length() < 2500) response += c;
      lastData = millis();
    }
    if (millis() - lastData > 15000) break;
    client.loop();
    delay(10);
  }

  apiClient.stop();
  response.trim();
  Serial.print("HTTP Response Code Detection API: ");
  Serial.println(httpCode);
  if (response.length() > 0) {
    Serial.println("Response detection API:");
    Serial.println(response);
  }
  return httpCode >= 200 && httpCode < 300;
}

void applyDetectionApiResponse(const String &apiResponse) {
  bool shouldAllOff = apiResponse.indexOf("\"actuatorCommand\":\"all_off\"") >= 0 ||
                      apiResponse.indexOf("\"status\":\"unknown\"") >= 0 ||
                      apiResponse.indexOf("\"status\":\"error\"") >= 0;

  if (shouldAllOff) {
    Serial.println("Detection API meminta all_off karena unknown/confidence rendah/error");
    activeSpeciesReady = false;
    activeSpecies = "";
    activeWarmMin = NAN;
    activeWarmMax = NAN;
    activeHumidityMin = NAN;
    activeHumidityMax = NAN;
    warmRangeAlerted = false;
    humidityRangeAlerted = false;
    allActuatorsOff();
    publishStatus();
    return;
  }

  if (apiResponse.indexOf("\"status\":\"classified\"") >= 0) {
    float warmMin = extractJsonNumber(apiResponse, "warmSideMin", NAN);
    float warmMax = extractJsonNumber(apiResponse, "warmSideMax", NAN);
    float humidityMin = extractJsonNumber(apiResponse, "humidityMin", NAN);
    float humidityMax = extractJsonNumber(apiResponse, "humidityMax", NAN);

    if (!isnan(warmMin) && !isnan(warmMax) && !isnan(humidityMin) && !isnan(humidityMax)) {
      activeSpeciesReady = true;
      activeSpecies = extractJsonString(apiResponse, "species");
      activeWarmMin = warmMin;
      activeWarmMax = warmMax;
      activeHumidityMin = humidityMin;
      activeHumidityMax = humidityMax;
      Serial.println("Parameter spesies aktif diperbarui:");
      Serial.print("Species: "); Serial.println(activeSpecies);
      Serial.print("Warm side: "); Serial.print(activeWarmMin); Serial.print(" - "); Serial.println(activeWarmMax);
      Serial.print("Humidity: "); Serial.print(activeHumidityMin); Serial.print(" - "); Serial.println(activeHumidityMax);
    } else {
      Serial.println("Response classified tetapi parameter numerik tidak lengkap");
    }
  }
}

void runAutoControl() {
  if (!activeSpeciesReady) return;
  if (millis() - lastAutoControl < autoControlInterval) return;
  lastAutoControl = millis();
  if (isnan(temperatureValue) || isnan(humidityValue)) {
    Serial.println("Auto control dilewati karena sensor belum valid");
    return;
  }

  bool warmOutOfRange = (temperatureValue < activeWarmMin) || (temperatureValue > activeWarmMax);
  if (warmOutOfRange && !warmRangeAlerted) {
    warmRangeAlerted = true;
    sendNotification("Suhu Di Luar Rentang Aman",
                      "Suhu saat ini " + String(temperatureValue, 1) + " C, rentang aman " +
                      String(activeWarmMin, 1) + "-" + String(activeWarmMax, 1) + " C.",
                      "warning");
  } else if (!warmOutOfRange && warmRangeAlerted) {
    warmRangeAlerted = false;
    sendNotification("Suhu Kembali Normal", "Suhu enclosure sudah kembali ke rentang aman.", "info");
  }

  bool humidityOutOfRange = (humidityValue < activeHumidityMin) || (humidityValue > activeHumidityMax);
  if (humidityOutOfRange && !humidityRangeAlerted) {
    humidityRangeAlerted = true;
    sendNotification("Kelembapan Di Luar Rentang Aman",
                      "Kelembapan saat ini " + String(humidityValue, 1) + "%, rentang aman " +
                      String(activeHumidityMin, 1) + "-" + String(activeHumidityMax, 1) + "%.",
                      "warning");
  } else if (!humidityOutOfRange && humidityRangeAlerted) {
    humidityRangeAlerted = false;
    sendNotification("Kelembapan Kembali Normal", "Kelembapan enclosure sudah kembali ke rentang aman.", "info");
  }

  bool changed = false;
  if (temperatureValue < activeWarmMin) {
    if (!relay1State) { setRelay1(true); changed = true; }
    if (!relay2State) { setRelay2(true); changed = true; }
  } else if (temperatureValue > activeWarmMax) {
    if (relay1State) { setRelay1(false); changed = true; }
    if (relay2State) { setRelay2(false); changed = true; }
  }

  if (humidityValue < activeHumidityMin) {
    if (!humidifierState) { setHumidifier(true); changed = true; }
  } else if (humidityValue > activeHumidityMax) {
    if (humidifierState) { setHumidifier(false); changed = true; }
  }

  if (changed) {
    Serial.println("Auto control memperbarui aktuator");
    publishStatus();
  }
}

void captureAndSendToBackend() {
  Serial.println("=== DETEKSI SPESIES SERVERLESS DIMULAI ===");
  if (!cameraReady) { publishDetectionFailed("camera_not_ready"); return; }
  if (WiFi.status() != WL_CONNECTED) { publishDetectionFailed("wifi_not_connected"); return; }
  if (!checkDetectionApiHealth()) {
    publishDetectionFailed("detection_api_health_failed");
    Serial.println("=== DETEKSI SPESIES SERVERLESS SELESAI ===");
    return;
  }

  client.loop();
  Serial.println("Menunggu lampu relay 2 stabil sebelum capture...");
  delay(detectionPreLightDelay);

  for (int i = 0; i < 4; i++) {
    camera_fb_t *old_fb = esp_camera_fb_get();
    if (old_fb) esp_camera_fb_return(old_fb);
    delay(120);
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { publishDetectionFailed("camera_capture_failed"); return; }

  Serial.print("Frame didapat: ");
  Serial.print(fb->len);
  Serial.println(" bytes");

  uint8_t *jpg_buf = NULL;
  size_t jpg_len = 0;
  bool jpg_ok = frame2jpg(fb, 92, &jpg_buf, &jpg_len);
  esp_camera_fb_return(fb);

  if (!jpg_ok || jpg_buf == NULL || jpg_len == 0) {
    publishDetectionFailed("jpg_conversion_failed");
    return;
  }

  Serial.print("JPG siap dikirim. Size: ");
  Serial.print(jpg_len);
  Serial.println(" bytes");

  String apiResponse = "";
  bool sendOk = sendJpgToDetectionApi(jpg_buf, jpg_len, apiResponse);
  free(jpg_buf);

  if (!sendOk) {
    publishDetectionFailed("detection_api_request_failed");
    Serial.println("=== DETEKSI SPESIES SERVERLESS SELESAI ===");
    return;
  }

  applyDetectionApiResponse(apiResponse);
  if (apiResponse.length() > 0 && apiResponse.length() < 3500) {
    client.publish(topic_detection, apiResponse.c_str());
  }
  Serial.println("=== DETEKSI SPESIES SERVERLESS SELESAI ===");
}

void readSensors() {
  temperatureValue = dhtWarm.readTemperature();
  if (isnan(temperatureValue)) {
    Serial.println("DHT22 warm (suhu) gagal dibaca");
  } else {
    Serial.print("Suhu: "); Serial.print(temperatureValue); Serial.println(" C");
  }

  humidityValue = dhtHumid.readHumidity();
  if (isnan(humidityValue)) {
    Serial.println("DHT22 humid (kelembapan) gagal dibaca");
  } else {
    Serial.print("Kelembapan: "); Serial.print(humidityValue); Serial.println(" %");
  }

  if (bh1750Ready) {
    luxValue = lightMeter.readLightLevel();
    if (luxValue < 0) Serial.println("BH1750 gagal dibaca");
    else { Serial.print("Cahaya: "); Serial.print(luxValue); Serial.println(" lux"); }
  } else {
    luxValue = -1;
    Serial.println("BH1750 belum siap");
  }

  int waterLevelRaw = analogRead(WATER_LEVEL_PIN);
  float waterLevelPercent = (float)(waterLevelRaw - waterLevelDryRaw) /
                             (float)(waterLevelWetRaw - waterLevelDryRaw) * 100.0;
  waterLevelPercent = constrain(waterLevelPercent, 0.0, 100.0);
  waterLevelValue = waterLevelPercent;
  waterLevelLow = waterLevelValue < waterLevelLowThreshold;

  Serial.print("Water level raw: "); Serial.print(waterLevelRaw);
  Serial.print(" | Persen: "); Serial.print(waterLevelValue);
  Serial.println(waterLevelLow ? " (RENDAH)" : "");

  if (waterLevelLow && humidifierState) {
    Serial.println("Water level rendah - mematikan humidifier otomatis untuk keamanan");
    setHumidifier(false);
  }
}

void publishStatus() {
  String payload = "{";
  payload += "\"relay1\":\""; payload += relay1State ? "on" : "off"; payload += "\",";
  payload += "\"relay2\":\""; payload += relay2State ? "on" : "off"; payload += "\",";
  payload += "\"humidifier\":\""; payload += humidifierState ? "on" : "off"; payload += "\"";
  payload += "}";
  client.publish(topic_status, payload.c_str());
  Serial.print("Publish status: ");
  Serial.println(payload);
}

void publishSensorData() {
  readSensors();
  runAutoControl();

  String payload = "{";
  payload += "\"temperature\":"; payload += isnan(temperatureValue) ? "null" : String(temperatureValue, 2); payload += ",";
  payload += "\"humidity\":"; payload += isnan(humidityValue) ? "null" : String(humidityValue, 2); payload += ",";
  payload += "\"lux\":"; payload += luxValue < 0 ? "null" : String(luxValue, 2); payload += ",";
  payload += "\"waterLevel\":"; payload += isnan(waterLevelValue) ? "null" : String(waterLevelValue, 2); payload += ",";
  payload += "\"relay1\":\""; payload += relay1State ? "on" : "off"; payload += "\",";
  payload += "\"relay2\":\""; payload += relay2State ? "on" : "off"; payload += "\",";
  payload += "\"humidifier\":\""; payload += humidifierState ? "on" : "off"; payload += "\",";
  payload += "\"camera\":\""; payload += cameraReady ? "ready" : "not_ready"; payload += "\",";
  payload += "\"activeSpecies\":";
  if (activeSpeciesReady) { payload += "\""; payload += activeSpecies; payload += "\""; }
  else payload += "null";
  payload += "}";
  client.publish(topic_sensor, payload.c_str());
  Serial.print("Publish sensor: ");
  Serial.println(payload);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Pesan masuk dari topic: ");
  Serial.println(topic);
  String message = "";
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  message.trim();
  Serial.print("Payload: ");
  Serial.println(message);

  if (String(topic) == topic_control) {
    if (message == "relay1_on") { setRelay1(true); publishStatus(); }
    else if (message == "relay1_off") { setRelay1(false); publishStatus(); }
    else if (message == "relay2_on") { setRelay2(true); publishStatus(); }
    else if (message == "relay2_off") { setRelay2(false); publishStatus(); }
    else if (message == "humidifier_on") { setHumidifier(true); publishStatus(); }
    else if (message == "humidifier_off") { setHumidifier(false); publishStatus(); }
    else if (message == "all_off") { activeSpeciesReady = false; activeSpecies = ""; allActuatorsOff(); publishStatus(); }
    else if (message == "read_sensor") publishSensorData();
    else if (message == "detect_species") {
      setRelay2(true);
      publishStatus();
      client.publish(topic_detection, "{\"status\":\"processing\",\"message\":\"manual_detection_started\"}");
      detectRequest = true;
    } else {
      Serial.println("Command tidak dikenal");
    }
  }
}

void setupWiFi() {
  Serial.println("Menghubungkan WiFi via WiFiManager...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setConnectTimeout(30);
  bool connected = wifiManager.autoConnect(wifiPortalName);
  if (connected && WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi terhubung");
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFiManager gagal terhubung, restart ESP...");
    delay(3000);
    ESP.restart();
  }
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT... ");
    String clientId = "ESP32S3-SmartEnclosure-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("terhubung");
      client.publish(topic_status, "ESP32-S3 online");
      client.subscribe(topic_control);
      Serial.print("Subscribe ke topic: ");
      Serial.println(topic_control);
      publishStatus();
    } else {
      Serial.print("gagal, rc=");
      Serial.print(client.state());
      Serial.println(" coba lagi 5 detik");
      delay(5000);
    }
  }
}

void setupSensors() {
  Serial.println("Setup sensor...");
  dhtWarm.begin();
  dhtHumid.begin();
  Serial.println("DHT22 dimulai");
  Wire.begin(BH1750_SDA, BH1750_SCL);
  bh1750Ready = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  Serial.println(bh1750Ready ? "BH1750 berhasil dimulai" : "BH1750 gagal dimulai");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println("=== SMART ENCLOSURE SERVERLESS MQTT + SENSOR + ACTUATOR + CAMERA + ROBOFLOW ===");
  client.setBufferSize(4096);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(WATER_LEVEL_PIN, INPUT);
  setRelay1(false);
  setRelay2(false);
  setHumidifier(false);
  setupSensors();
  setupCamera();
  setupWiFi();
  setupTime();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setKeepAlive(60);
  client.setSocketTimeout(30);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi();
    setupTime();
  }
  if (!client.connected()) reconnectMQTT();
  client.loop();
  checkScheduledDetection();
  if (detectRequest) {
    detectRequest = false;
    captureAndSendToBackend();
  }
  if (millis() - lastPublish > publishInterval) {
    lastPublish = millis();
    publishSensorData();
  }
}
