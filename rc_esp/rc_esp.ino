#include "esp_sleep.h"
#include "updatepage.h"
#include <Adafruit_INA219.h>
#include <Arduino.h> //DO NOT EDIT THIS LINE!!!

#include <Firebase_ESP_Client.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <addons/RTDBHelper.h>
#include <addons/TokenHelper.h>
#include <time.h>

static TinyGPSPlus gps;

// ---- Raspberry Pi Zero 2W serial bridge ----
static const uint32_t RP_BAUD = 115200;

#ifdef ARDUINO_ARCH_ESP32S2
// =============================================================
// ESP32-S2
// =============================================================
static HardwareSerial gpsSerial(1); // UART1
static HardwareSerial rpSerial(0);  // UART0 (bukan USB CDC di S2)

static const int RP_RX_PIN = 18; // terima dari TX Pi GPIO14
static const int RP_TX_PIN = 17; // kirim ke RX Pi GPIO15

static const int GPS_RX_PIN = 9;
static const int GPS_TX_PIN = 8;

static const int I2C_SDA = 14;
static const int I2C_SCL = 13;

static const int PIN_LEFT = 33;
static const int PIN_RIGHT = 34;
static const int PIN_FRONT_FWD = 36;
static const int PIN_FRONT_REV = 35;
static const int PIN_REAR_REV = 38;
static const int PIN_REAR_FWD = 37;

static const int MAIN_PWR_PIN = 21; // RTC PIN (ESP32-S2 RTC domain)

#else
// =============================================================
// ESP32 Classic
// =============================================================
static HardwareSerial gpsSerial(1); // UART1
static HardwareSerial rpSerial(2);  // UART2 — tersedia di ESP32 classic

static const int RP_RX_PIN = 16; // RX2 — terima dari TX Pi GPIO14
static const int RP_TX_PIN = 17; // TX2 — kirim ke RX Pi GPIO15

static const int GPS_RX_PIN = 18; // UART1 RX (bisa di-remap)
static const int GPS_TX_PIN = 19; // UART1 TX (bisa di-remap)

static const int I2C_SDA = 21; // default SDA ESP32
static const int I2C_SCL = 22; // default SCL ESP32

static const int PIN_LEFT = 25;
static const int PIN_RIGHT = 26;
static const int PIN_FRONT_FWD = 32;
static const int PIN_FRONT_REV = 33;
static const int PIN_REAR_FWD = 27;
static const int PIN_REAR_REV = 14;

static const int MAIN_PWR_PIN = 23; // RTC GPIO (32-39 RTC domain)

#endif // ARDUINO_ARCH_ESP32S2

static const uint8_t INA1_ADDR = 0x40;
static Adafruit_INA219 ina1(INA1_ADDR);

static const char *WIFI_SSID = "TP-Link dev";
static const char *WIFI_PASSWORD = "123456789";

static const char *API_KEY = "AIzaSyA_mmjlGHFOLw5xfv8VNfya7RubMs1YHH0";
static const char *DATABASE_URL =
    "https://"
    "rc-control-a07b8-default-rtdb.asia-southeast1.firebasedatabase.app/";
static const char *USER_EMAIL = "miskamumtaza123@gmail.com";
static const char *USER_PASSWORD = "apalah123";

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

static bool ina1_status = false;

// ---- OTA update state ----
static WebServer updateServer(80);
static bool updatefirmware = false;
static int updatePercent = 0;
static int updateStatus = 0; // 0=idle 1=busy 2=ok 3=fail
static String updateUrl = "";
static String updateMd5 = "";
static size_t updateExpectedSize = 0;

static bool has_bin_extension(const String &fn) {
  String s = fn;
  s.toLowerCase();
  return s.endsWith(".bin");
}

static const uint32_t GPS_BAUD = 9600;
static const uint32_t GPS_TIMEOUT_MS = 15000;

static const int PWM_FREQ_HZ = 1000;
static const int PWM_RES_BITS = 8;
static const int CH_FRONT_FWD = 3;
static const int CH_FRONT_REV = 4;
static const int CH_REAR_FWD = 5;
static const int CH_REAR_REV = 6;

static void setupWiFi() { // connect to Wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());
}

static void setupFirebase() { // connect to firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.token_status_callback = tokenStatusCallback;

  Firebase.reconnectWiFi(true);
  Firebase.begin(&config, &auth);

  Serial.print("Waiting Firebase");
  while (!Firebase.ready()) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.println("Firebase ready");
}

static void checkStatus() { // check status in to decide to sleep or not
  if (Firebase.RTDB.getBool(&fbdo, "/status")) {
    const bool status = fbdo.boolData();
    Serial.printf("status: %s\n", status ? "true" : "false");
    if (!status) {
      Serial.println("Status false, deep sleep 60s");
      digitalWrite(MAIN_PWR_PIN, LOW);
      gpio_hold_en((gpio_num_t)MAIN_PWR_PIN);
      esp_sleep_enable_timer_wakeup(60ULL * 1000000ULL); // 60 seconds
      esp_deep_sleep_start();
    }
  } else {
    Serial.print("Failed get /status: ");
    Serial.println(fbdo.errorReason());
  }
}

static void updateVolt() { // update voltage firebase (INA1 sensor)
  if (!Firebase.ready()) {
    Serial.println("Voltage update skipped: Firebase not ready");
    return;
  }

  // --- INA1 (0x40) -> /batVolt ---
  float avgV1 = 0.0f;
  if (ina1_status) {
    const uint32_t start1 = millis();
    double sum1 = 0.0;
    uint32_t count1 = 0;
    while ((millis() - start1) < 1000) {
      sum1 += ina1.getBusVoltage_V();
      count1++;
    }
    avgV1 = (count1 > 0) ? (float)(sum1 / (double)count1) : 0.0f;
  } else {
    Serial.println("INA1 fault: sending 0 to /batVolt");
  }
  if (Firebase.RTDB.setFloat(&fbdo, "/batVolt", avgV1)) {
    Serial.printf("batVolt  (INA1): %.2f V\n", avgV1);
  } else {
    Serial.print("Failed set /batVolt: ");
    Serial.println(fbdo.errorReason());
  }
}

static void updateIP() { // update IP to firebase
  if (!Firebase.ready()) {
    Serial.println("IP update skipped: Firebase not ready");
    return;
  }
  const String ipStr = WiFi.localIP().toString();
  if (Firebase.RTDB.setString(&fbdo, "/ip", ipStr.c_str())) {
    Serial.printf("IP updated to firebase: %s\n", ipStr.c_str());
  } else {
    Serial.print("Failed set /IP: ");
    Serial.println(fbdo.errorReason());
  }
}

static bool
syncTimeWithNtp(uint32_t timeoutMs = 15000) { // sync time with NTP server
  // GMT+7 (WIB) = 7 * 3600 seconds
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");

  const uint32_t start = millis();
  time_t now = 0;
  while ((millis() - start) < timeoutMs) {
    time(&now);
    if (now > 1600000000) {
      return true;
    }
    delay(250);
  }
  return false;
}

static void updatePos() { // update GPS position to firebase on boot
  if (!Firebase.ready()) {
    Serial.println("Pos update skipped: Firebase not ready");
    return;
  }

  Serial.println("Waiting for GPS fix...");
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  float lat = 0.0f, lon = 0.0f;
  bool gotFix = false;
  const uint32_t deadline = millis() + GPS_TIMEOUT_MS;

  while (millis() < deadline) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isValid() && gps.location.age() < 2000) {
      lat = (float)gps.location.lat();
      lon = (float)gps.location.lng();
      gotFix = true;
      break;
    }
    delay(10);
  }

  if (gotFix) {
    Serial.printf("GPS fix: %.6f, %.6f\n", lat, lon);
  } else {
    Serial.println("GPS fault: no fix — sending 0 to /Xpos /Ypos");
  }

  if (Firebase.RTDB.setFloat(&fbdo, "/Xpos", lat)) {
    Serial.printf("Xpos updated: %.6f\n", lat);
  } else {
    Serial.print("Failed set /Xpos: ");
    Serial.println(fbdo.errorReason());
  }
  if (Firebase.RTDB.setFloat(&fbdo, "/Ypos", lon)) {
    Serial.printf("Ypos updated: %.6f\n", lon);
  } else {
    Serial.print("Failed set /Ypos: ");
    Serial.println(fbdo.errorReason());
  }
}

static void updateTime() { // update time to firebase
  if (!Firebase.ready()) {
    Serial.println("Time update skipped: Firebase not ready");
    return;
  }
  if (!syncTimeWithNtp()) {
    Serial.println("Time update skipped: NTP not synced");
    return;
  }
  time_t now = 0;
  struct tm tm_info;
  time(&now);
  localtime_r(&now, &tm_info);
  char buf[32];
  strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &tm_info);
  if (Firebase.RTDB.setString(&fbdo, "/time", buf)) {
    Serial.printf("Time updated to firebase: %s\n", buf);
  } else {
    Serial.print("Failed set /time: ");
    Serial.println(fbdo.errorReason());
  }
}

// ===================== OTA update =====================

static bool startFirmwareUpdate(const String &url, const String &md5) {
  updatePercent = 0;
  updateStatus = 1;

  if (md5.length())
    Update.setMD5(md5.c_str());
  else
    Update.setMD5("");

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(20000);

  WiFiClient plain;
  WiFiClientSecure secure;
  secure.setInsecure();
  plain.setTimeout(20000);
  secure.setTimeout(20000);

  if (url.startsWith("https://")) {
    if (!http.begin(secure, url)) {
      updateStatus = 3;
      return false;
    }
  } else {
    if (!http.begin(plain, url)) {
      updateStatus = 3;
      return false;
    }
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("OTA error: HTTP GET failed, code=%d\n", httpCode);
    http.end();
    updateStatus = 3;
    return false;
  }

  int total = http.getSize();
  Serial.printf("OTA: total size = %d bytes\n", total);

  WiFiClient *stream = http.getStreamPtr();
  if (!stream) {
    Serial.println("OTA error: stream is null");
    http.end();
    updateStatus = 3;
    return false;
  }

  if (!Update.begin(total > 0 ? (size_t)total : UPDATE_SIZE_UNKNOWN)) {
    Serial.printf("OTA error: Update.begin failed: %s\n", Update.errorString());
    http.end();
    updateStatus = 3;
    return false;
  }

  uint8_t buf[2048];
  size_t written = 0;

  while (http.connected() && (total < 0 || (int)written < total)) {
    size_t avail = stream->available();
    if (!avail) {
      delay(1);
      updateServer.handleClient();
      continue;
    }
    int toRead = (avail > sizeof(buf)) ? sizeof(buf) : (int)avail;
    int r = stream->readBytes(buf, toRead);
    if (r <= 0) {
      Serial.println("OTA error: readBytes <= 0");
      break;
    }
    if (Update.write(buf, (size_t)r) != (size_t)r) {
      Serial.printf("OTA error: Update.write: %s\n", Update.errorString());
      Update.abort();
      http.end();
      updateStatus = 3;
      return false;
    }
    written += (size_t)r;
    if (total > 0) {
      updatePercent = (int)((written * 100ULL) / (unsigned long long)total);
      if (updatePercent > 100)
        updatePercent = 100;
    }
    updateServer.handleClient();
  }

  if (!Update.end(true)) {
    Serial.printf("OTA error: Update.end: %s\n", Update.errorString());
    http.end();
    updateStatus = 3;
    return false;
  }

  http.end();
  updatePercent = 100;
  updateStatus = 2;
  Serial.println("OTA: success");
  return true;
}

static void setupUpdateServer() {
  // Firmware update page
  updateServer.on("/update", HTTP_GET, []() {
    String page = FPSTR(updatePage);
    updateServer.send(200, "text/html", page);
  });

  // Progress polling (JSON)
  updateServer.on("/updateinfo", HTTP_GET, []() {
    String done = "wait";
    if (updateStatus == 2)
      done = "ok";
    else if (updateStatus == 3)
      done = "fail";
    String json =
        "{\"percent\":" + String(updatePercent) + ",\"done\":\"" + done + "\"}";
    updateServer.send(200, "application/json", json);
  });

  // Trigger URL-based OTA
  updateServer.on("/startupdate", HTTP_POST, []() {
    if (updateServer.hasArg("url")) {
      updateUrl = updateServer.arg("url");
      updateMd5 = updateServer.hasArg("md5") ? updateServer.arg("md5") : "";
      updatefirmware = true;
      updateServer.send(200, "text/plain", "OK");
    } else {
      updateServer.send(400, "text/plain", "Missing url");
    }
  });

  // File-upload OTA
  updateServer.on(
      "/updateViaFile", HTTP_POST,
      []() {
        updateServer.send(updateStatus == 2 ? 200 : 500, "text/plain",
                          updateStatus == 2 ? "OK" : "FAIL");
      },
      []() {
        HTTPUpload &upload = updateServer.upload();
        if (upload.status == UPLOAD_FILE_START) {
          updatefirmware = false;
          updateUrl = "";
          updateMd5 = "";
          updatePercent = 0;
          updateStatus = 1;
          updateExpectedSize = 0;
          if (!has_bin_extension(upload.filename)) {
            updateStatus = 3;
            return;
          }
          const String sz = updateServer.header("X-File-Size");
          if (sz.length())
            updateExpectedSize = (size_t)sz.toInt();
          Update.setMD5("");
          const size_t beginSize =
              updateExpectedSize > 0 ? updateExpectedSize : UPDATE_SIZE_UNKNOWN;
          if (!Update.begin(beginSize)) {
            Serial.printf("UpdateViaFile: Update.begin: %s\n",
                          Update.errorString());
            updateStatus = 3;
            return;
          }
          Serial.printf("UpdateViaFile start: %s expected=%u\n",
                        upload.filename.c_str(), (unsigned)updateExpectedSize);
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          if (updateStatus != 1)
            return;
          if (Update.write(upload.buf, upload.currentSize) !=
              upload.currentSize) {
            Serial.printf("UpdateViaFile write: %s\n", Update.errorString());
            updateStatus = 3;
            return;
          }
          updatePercent = updateExpectedSize > 0
                              ? (int)((upload.totalSize * 100ULL) /
                                      (unsigned long long)updateExpectedSize)
                              : (int)(upload.totalSize / 65536UL);
          if (updatePercent > 99)
            updatePercent = 99;
          yield();
        } else if (upload.status == UPLOAD_FILE_END) {
          if (updateStatus != 1) {
            Update.abort();
            return;
          }
          if (!Update.end(true)) {
            Serial.printf("UpdateViaFile end: %s\n", Update.errorString());
            updateStatus = 3;
            return;
          }
          updatePercent = 100;
          updateStatus = 2;
          Serial.printf("UpdateViaFile ok: %u bytes\n",
                        (unsigned)upload.totalSize);
        } else if (upload.status == UPLOAD_FILE_ABORTED) {
          Update.abort();
          updateStatus = 3;
        }
      });

  // Reboot
  updateServer.on("/reboot", HTTP_POST, []() {
    updateServer.send(200, "text/plain", "Rebooting");
    delay(200);
    ESP.restart();
  });

  updateServer.onNotFound(
      []() { updateServer.send(404, "text/plain", "Not found"); });
  updateServer.begin();
  Serial.println("OTA server started on port 80 — go to http://" +
                 WiFi.localIP().toString() + "/update");
}

static bool otaRestart = false;

static void loopUpdate() { // call in loop()
  updateServer.handleClient();
  if (updatefirmware) {
    updatefirmware = false;
    startFirmwareUpdate(updateUrl, updateMd5);
    if (updateStatus == 2)
      otaRestart = true;
  }
  if (otaRestart) {
    delay(1000);
    ESP.restart();
  }
}
/// END OF UPDATES ///

/// ALL PIN SETUP
static void setupMotorPwm() { // for motor driver
  pinMode(PIN_LEFT, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);
  digitalWrite(PIN_LEFT, LOW);
  digitalWrite(PIN_RIGHT, LOW);

  ledcSetup(CH_FRONT_FWD, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH_FRONT_REV, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH_REAR_FWD, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH_REAR_REV, PWM_FREQ_HZ, PWM_RES_BITS);

  // Set duty cycle ke 0 SEBELUM di-attach ke pin untuk mencegah glitch (pin
  // high tiba-tiba)
  ledcWrite(CH_FRONT_FWD, 0);
  ledcWrite(CH_FRONT_REV, 0);
  ledcWrite(CH_REAR_FWD, 0);
  ledcWrite(CH_REAR_REV, 0);

  ledcAttachPin(PIN_FRONT_FWD, CH_FRONT_FWD);
  ledcAttachPin(PIN_FRONT_REV, CH_FRONT_REV);
  ledcAttachPin(PIN_REAR_FWD, CH_REAR_FWD);
  ledcAttachPin(PIN_REAR_REV, CH_REAR_REV);
}

void setupPins() { // for mosfet driver
  gpio_hold_dis((gpio_num_t)MAIN_PWR_PIN);
  pinMode(MAIN_PWR_PIN, OUTPUT);
  digitalWrite(MAIN_PWR_PIN, HIGH);
}
void setupPowerMeter() { // start INA219 x1
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.printf("I2C SDA=%d SCL=%d\n", I2C_SDA, I2C_SCL);

  // INA1 @ 0x40
  if (!ina1.begin()) {
    Serial.printf("INA1 (0x%02X): NOT FOUND\n", INA1_ADDR);
  } else {
    Serial.printf("INA1 (0x%02X): ready\n", INA1_ADDR);
    ina1_status = true;
  }
}

void backgroundTask(void *pvParameters) {
  Serial.println("[BgTask] Background task started on Core 0");
  for (;;) {
    updatePos();
    updateVolt();
    checkStatus();
    vTaskDelay(pdMS_TO_TICKS(10000)); // Delay 10 seconds
  }
}

void setup() {
  // FORCE MOTORS OFF IMMEDIATELY ON BOOT
  pinMode(PIN_FRONT_FWD, OUTPUT);
  pinMode(PIN_FRONT_REV, OUTPUT);
  pinMode(PIN_REAR_FWD, OUTPUT);
  pinMode(PIN_REAR_REV, OUTPUT);
  digitalWrite(PIN_FRONT_FWD, LOW);
  digitalWrite(PIN_FRONT_REV, LOW);
  digitalWrite(PIN_REAR_FWD, LOW);
  digitalWrite(PIN_REAR_REV, LOW);

  pinMode(PIN_LEFT, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);
  digitalWrite(PIN_LEFT, LOW);
  digitalWrite(PIN_RIGHT, LOW);

  Serial.begin(115200);
  rpSerial.begin(RP_BAUD, SERIAL_8N1, RP_RX_PIN, RP_TX_PIN);
  Serial.printf("rpSerial started — RX=%d TX=%d\n", RP_RX_PIN, RP_TX_PIN);
  setupPowerMeter();   // ina219 check
  setupPins();         // aux pin setup
  setupMotorPwm();     // main motor pwm setup
  setupWiFi();         // connect to wifi
  setupFirebase();     // connect to firebase
  updateTime();        // update time to firebase
  updateIP();          // update IP to firebase

  // Create background task pinned to Core 0
  xTaskCreatePinnedToCore(
      backgroundTask,   /* Task function. */
      "backgroundTask", /* name of task. */
      8192,             /* Stack size of task */
      NULL,             /* parameter of the task */
      1,                /* priority of the task */
      NULL,             /* Task handle to keep track of created task */
      0);               /* pin task to core 0 */

  setupUpdateServer(); // start OTA web server on port 80
}

// ===================== Serial Bridge Buffers =====================
static String _rpBuf;  // akumulasi baris dari RPi  (rpSerial)
static String _usbBuf; // akumulasi baris dari USB CDC (Serial)

// ===================== Motor Control =====================
static uint32_t lastMovTime = 0;
static uint32_t lastSpoolTime = 0;

static float current_fwd = 0;
static float target_fwd = 0;
static float current_rev = 0;
static float target_rev = 0;
static int current_mode = 2; // 0=FWD, 1=RWD, 2=AWD

void spooling() {
  uint32_t now = millis();
  if (now - lastSpoolTime >= 10) {
    lastSpoolTime = now;

    // 100% (255) dalam 1 detik (1000ms) = +2.55 per 10ms
    float step = 2.55;

    // Handle FWD
    if (current_fwd < target_fwd) {
      current_fwd += step;
      if (current_fwd > target_fwd)
        current_fwd = target_fwd;
    } else if (current_fwd > target_fwd) {
      current_fwd = target_fwd; // instant spool down
    }

    // Handle REV
    if (current_rev < target_rev) {
      current_rev += step;
      if (current_rev > target_rev)
        current_rev = target_rev;
    } else if (current_rev > target_rev) {
      current_rev = target_rev; // instant spool down
    }

    // Terapkan PWM berdasarkan mode (0=FWD, 1=RWD, 2=AWD)
    int fwd_val = (int)current_fwd;
    int rev_val = (int)current_rev;

    if (current_mode == 0) { // FWD
      ledcWrite(CH_FRONT_FWD, fwd_val);
      ledcWrite(CH_FRONT_REV, rev_val);
      ledcWrite(CH_REAR_FWD, 0);
      ledcWrite(CH_REAR_REV, 0);
    } else if (current_mode == 1) { // RWD
      ledcWrite(CH_FRONT_FWD, 0);
      ledcWrite(CH_FRONT_REV, 0);
      ledcWrite(CH_REAR_FWD, fwd_val);
      ledcWrite(CH_REAR_REV, rev_val);
    } else { // AWD
      ledcWrite(CH_FRONT_FWD, fwd_val);
      ledcWrite(CH_FRONT_REV, rev_val);
      ledcWrite(CH_REAR_FWD, fwd_val);
      ledcWrite(CH_REAR_REV, rev_val);
    }
  }
}

void parse_serial(const String &line) {
  // format: "mov f,b,l,r,s,m" -> s adalah speed 0-100, m adalah mode 0-2
  if (line.startsWith("mov ")) {
    int f = 0, b = 0, l = 0, r = 0, s = 0, m = 2;
    int parsed =
        sscanf(line.c_str(), "mov %d,%d,%d,%d,%d,%d", &f, &b, &l, &r, &s, &m);
    if (parsed >= 5) {
      lastMovTime = millis();

      if (parsed == 6) {
        current_mode = m;
      }

      // Map speed 0-100 to PWM 0-255 (8-bit resolution)
      int pwmVal = (s * 255) / 100;
      if (pwmVal > 255)
        pwmVal = 255;
      if (pwmVal < 0)
        pwmVal = 0;

      // Steering: Left & Right (Digital Output)
      digitalWrite(PIN_LEFT, l ? HIGH : LOW);
      digitalWrite(PIN_RIGHT, r ? HIGH : LOW);

      // Set target untuk spooling
      target_fwd = f ? pwmVal : 0;
      target_rev = b ? pwmVal : 0;

      // Jika deselerasi/stop, langsung set current agar instan tanpa nunggu
      // 10ms
      if (current_fwd > target_fwd)
        current_fwd = target_fwd;
      if (current_rev > target_rev)
        current_rev = target_rev;

      // Terapkan langsung untuk respon instan saat turun
      int fwd_val = (int)current_fwd;
      int rev_val = (int)current_rev;

      if (current_mode == 0) { // FWD
        ledcWrite(CH_FRONT_FWD, fwd_val);
        ledcWrite(CH_FRONT_REV, rev_val);
        ledcWrite(CH_REAR_FWD, 0);
        ledcWrite(CH_REAR_REV, 0);
      } else if (current_mode == 1) { // RWD
        ledcWrite(CH_FRONT_FWD, 0);
        ledcWrite(CH_FRONT_REV, 0);
        ledcWrite(CH_REAR_FWD, fwd_val);
        ledcWrite(CH_REAR_REV, rev_val);
      } else { // AWD
        ledcWrite(CH_FRONT_FWD, fwd_val);
        ledcWrite(CH_FRONT_REV, rev_val);
        ledcWrite(CH_REAR_FWD, fwd_val);
        ledcWrite(CH_REAR_REV, rev_val);
      }
    }
  }
}
int PWM_TIMEOUT = 500;
void check_timeout() { // if lost connection, cut motor power
  if (millis() - lastMovTime > PWM_TIMEOUT) {
    digitalWrite(PIN_LEFT, LOW);
    digitalWrite(PIN_RIGHT, LOW);

    // Stop instan
    target_fwd = 0;
    target_rev = 0;
    current_fwd = 0;
    current_rev = 0;

    ledcWrite(CH_FRONT_FWD, 0);
    ledcWrite(CH_FRONT_REV, 0);
    ledcWrite(CH_REAR_FWD, 0);
    ledcWrite(CH_REAR_REV, 0);
  }
}

void loop() {
  loopUpdate(); // handle OTA firmware update

  // ---- rpSerial -> Serial (CDC USB): label [Rpi] ----
  while (rpSerial.available()) {
    char c = (char)rpSerial.read();
    if (c == '\n') {
      _rpBuf.trim();
      if (_rpBuf.length() > 0) {
        Serial.print("[Rpi] ");
        Serial.println(_rpBuf);
        parse_serial(_rpBuf); // <--- Parse the incoming command
      }
      _rpBuf = "";
    } else if (c != '\r') {
      _rpBuf += c;
    }
  }

  // ---- Serial (CDC USB) -> rpSerial: label [SERIAL], forward ke RPi ----
  while (Serial.available()) {
    char c = (char)Serial.read();
    rpSerial.write((uint8_t)c); // langsung forward tiap byte ke RPi
    if (c == '\n') {
      _usbBuf.trim();
      if (_usbBuf.length() > 0) {
        Serial.print("[SERIAL] ");
        Serial.println(_usbBuf);
      }
      _usbBuf = "";
    } else if (c != '\r') {
      _usbBuf += c;
    }
  }

  // ---- Safety Timeout ----
  check_timeout();

  // ---- Spooling Update ----
  spooling();
}
