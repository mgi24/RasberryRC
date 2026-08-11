#include <Arduino.h>

// IR sender/receiver
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Daikin.h>
#include <assert.h>
#include <IRrecv.h>
#include <IRac.h>
#include <IRtext.h>
#include <IRutils.h>

// Config storage
#include <Preferences.h>

// WiFi + HTTP
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>

// MQTT
#include <PubSubClient.h>

// Brownout control (optional)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Web pages (reuse the same templates from repo root)
#include "landing.h"
#include "update.h"

// ===================== Pins / constants =====================

#define LED_PIN 5
#define RST_BTN 16
#define LED_AC 32
#define AC_POWER_LED 2

static const uint16_t kIrLed = 22;
static const uint16_t kRecvPin = 23;

static const uint32_t kWiFiConnectTimeoutMs = 30000;

// ===================== Globals (match ac.ino logic) =====================

Preferences preferences;

String server_port = "80";

bool disable_brownout = false;

String OTAName = "ART1";
String ssid = "";
String password = "";
uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
int channel = 0;
String ip = "";
String gateway = "";
String subnet = "";
String dns = "";

String tbserver = "";
String tbuser = "";
String tbpass = "";
String deviceID = "";
String deviceToken = ""; // access token
String token = "";       // bearer token
int minTemp = 18;

bool server_ok = false;
bool user_ok = false;
bool device_ok = false;
bool mqtt_ok = false;
bool stop_request = false;

bool auto_reboot = true;

// Server-config flag: if false, skip boot IR send_remote()
bool beep = true;

// IR state
bool status = false;
bool swing = false;
int temp = 20;
String ir_protocol = "";

IRDaikinESP acDaikin(kIrLed);
IRKelvinatorAC acKelvinator(kIrLed);
IRDaikin64 acDaikin64(kIrLed);
IRPanasonicAc acPanasonic(kIrLed);

const uint8_t kTimeout = 50;
const uint16_t kCaptureBufferSize = 1024;
const uint16_t kMinUnknownSize = 12;
const uint8_t kTolerancePercentage = 30;

decode_results results;
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);

bool update_telemetry = false;

bool wifiConnected = false;

WebServer server(80);

bool debug = false;
bool restart = false;
bool disable_boot_ir = false;

bool revert_ir = false;
bool receive_ir = true;

bool trytoconnect = false;

// MQTT
WiFiClient espClient;
PubSubClient client(espClient);

bool runMQTT = false;
bool status_new = false;
bool swing_new = false;
int temp_new = 20;
bool mqtt_request = false;

// LED indicator (AC feedback)
bool use_AC_LED = true;
bool led_state = false;
bool stop_led_check = false;

// WiFi scan cache
#define wifiListLen 100
String ssidlist[wifiListLen];
String securityList[wifiListLen];
int32_t rssiList[wifiListLen];
int channelList[wifiListLen];
uint8_t maclist[wifiListLen][6];

int wifiListCount = 0;
bool wifiScan = false;
bool showWifi = false;

// Update
bool updatefirmware = false;
int updatePercent = 0;
int updateStatus = 0; // 0=idle, 1=downloading, 2=success, 3=failed
String updateUrl = "";
String updateMd5 = "";
size_t updateExpectedSize = 0;

// Logs
int logQueue = 0; // dont change this
String log_msg[50];

// ===================== Helpers =====================

static void printLog(const String& msg) {
  Serial.print(msg);
  if (logQueue >= 50) {
    for (int i = 49; i > 0; i--) log_msg[i] = log_msg[i - 1];
    log_msg[0] = msg;
  } else {
    for (int i = logQueue; i > 0; i--) log_msg[i] = log_msg[i - 1];
    log_msg[0] = msg;
    logQueue++;
  }
}

static void readPreferences() {
  preferences.begin("my-app", false);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  ip = preferences.getString("ip", "");
  gateway = preferences.getString("gateway", "");
  subnet = preferences.getString("subnet", "");
  dns = preferences.getString("dns", "");
  OTAName = preferences.getString("OTAName", OTAName);
  tbserver = preferences.getString("server", "");
  tbuser = preferences.getString("username", "");
  tbpass = preferences.getString("tbpassword", "");
  deviceID = preferences.getString("deviceID", "");
  deviceToken = preferences.getString("deviceToken", "");
  ir_protocol = preferences.getString("ir_protocol", "");
  String macStr = preferences.getString("mac", "");
  if (macStr != "") {
    sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac[0], &mac[1],
           &mac[2], &mac[3], &mac[4], &mac[5]);
  }
  channel = preferences.getInt("channel", 0);
  //disable_boot_ir = preferences.getBool("disable_boot_ir", false);
  if (ir_protocol == "DAIKIN64") {
    status = preferences.getBool("status", false);
    swing = preferences.getBool("swing", false);
    temp = preferences.getInt("temp", 24);
  }
  preferences.end();
}

static void infrared_setup() {
  acDaikin.begin();
  acKelvinator.begin();
  acDaikin64.begin();
  assert(irutils::lowLevelSanityCheck() == 0);
#if DECODE_HASH
  irrecv.setUnknownThreshold(kMinUnknownSize);
#endif
  irrecv.setTolerance(kTolerancePercentage);
  irrecv.enableIRIn();
}

static String read_ir_protocol(const decode_results* const results) {
  return typeToString(results->decode_type, results->repeat);
}

// Forward declarations (functions are kept close to original naming)
static void send_remote(bool status, bool swing, int temp);
static void reverse_ir();
static void process_ir(decode_results* results);
static void beginWifi();
static void connectWifi();
static void scanWifi();
static void setupSSID();
static void getToken(String url, String username, String password);
static void getAccessToken(String url, String deviceID);
static void getStatus();
static void send_Telemetry(bool status, bool swing, int temp);
static void send_log(String* log_array);
static void connectMQTT();
static void rpcCallback(char* topic, byte* payload, unsigned int length);
static void processMqtt(bool statusupdate, bool swingupdate, int tempupdate);
static void led_confirm_process();
static bool startFirmwareUpdate(const String& url, const String& md5);

static bool has_bin_extension(const String& filename) {
  String lower = filename;
  lower.toLowerCase();
  return lower.endsWith(".bin");
}

// ===================== Main-loop "task" services =====================

static void check_reset_btn() {
  if (digitalRead(RST_BTN) != LOW) return;

  unsigned long pressStart = millis();
  bool reseted = false;
  while (digitalRead(RST_BTN) == LOW) {
    if (millis() - pressStart > 50) {
      digitalWrite(LED_PIN, HIGH);
      printLog("Reset button pressed! Clearing preferences and rebooting...\n");
      restart = true;
      reseted = true;
    }
    delay(1);
    yield();
  }

  if (reseted) {
    preferences.begin("my-app", false);
    preferences.clear();
    preferences.end();
  }
}

static void update_receive_ir_timeout() {
  static long lastReceiveIrFalseTime = 0;
  if (!receive_ir) {
    if (lastReceiveIrFalseTime == 0) {
      lastReceiveIrFalseTime = millis();
    } else if (millis() - (unsigned long)lastReceiveIrFalseTime >= 1000UL) {
      receive_ir = true;
      lastReceiveIrFalseTime = 0;
    }
  } else {
    lastReceiveIrFalseTime = 0;
  }
}

//disable IR send at boot
void disabling_boot_ir(){//DISABLED RIGHT NOW
  preferences.begin("my-app", false);
  preferences.putBool("disable_boot_ir", true);
  preferences.end();
}

static void update_auto_reboot() {
  static unsigned long bootTime = millis();
  if (auto_reboot && (millis() - bootTime >= 3600000UL)) {
    printLog("Auto reboot triggered after 1 hour\n");
    //disabling_boot_ir();
    restart = true;
  }
}

static void force_telemetry_every_minute() {
  static unsigned long lastForceTelemetry = 0;
  if (millis() - lastForceTelemetry > 60000UL) {
    update_telemetry = true;
    lastForceTelemetry = millis();
  }
}

//hard check ac status
static void update_ac_led_indicator() {
  if (!use_AC_LED) return;
  if (stop_led_check) return;

  static const int sampleCount = 100;
  static uint8_t sampleLED[sampleCount] = {};

  int acLedValue = analogRead(LED_AC);
  digitalWrite(AC_POWER_LED, (acLedValue > 4000) ? HIGH : LOW);

  if (ir_protocol == "DAIKIN152") {
    if (acLedValue < 500) {
      for (int i = sampleCount - 1; i > 0; i--) sampleLED[i] = sampleLED[i - 1];
      sampleLED[0] = false;
    } else if (acLedValue > 2500) {
      for (int i = sampleCount - 1; i > 0; i--) sampleLED[i] = sampleLED[i - 1];
      sampleLED[0] = true;
    }

    int trueCount = 0;
    for (int i = 0; i < sampleCount; i++) if (sampleLED[i]) trueCount++;
    led_state = (trueCount > sampleCount / 2);
    digitalWrite(AC_POWER_LED, led_state ? HIGH : LOW);

    static unsigned long mismatchStartTime = 0;
    static bool mismatchActive = false;

    if (status != led_state) {
      if (!mismatchActive) {
        mismatchStartTime = millis();
        mismatchActive = true;
      } else if (millis() - mismatchStartTime >= 1000UL) {
        status = led_state;
        update_telemetry = true;
        printLog(
            "Status flipped due to consistent mismatch with LED state for 1 second (DAIKIN152)\n");
        mismatchStartTime = 0;
        mismatchActive = false;
      }
    } else {
      mismatchStartTime = 0;
      mismatchActive = false;
    }
    return;
  }

  if (ir_protocol == "DAIKIN") {
    if (acLedValue < 500) {
      for (int i = sampleCount - 1; i > 0; i--) sampleLED[i] = sampleLED[i - 1];
      sampleLED[0] = false;
    } else if (acLedValue > 1300) {
      for (int i = sampleCount - 1; i > 0; i--) sampleLED[i] = sampleLED[i - 1];
      sampleLED[0] = true;
    }

    int falseCount = 0;
    for (int i = 0; i < sampleCount; i++) if (!sampleLED[i]) falseCount++;
    led_state = !(falseCount > (int)(sampleCount * 0.1f));
    digitalWrite(AC_POWER_LED, led_state ? HIGH : LOW);

    static unsigned long mismatchStartTime = 0;
    static bool mismatchActive = false;

    if (!status) {
      if (status != led_state) {
        if (!mismatchActive) {
          mismatchStartTime = millis();
          mismatchActive = true;
        } else if (millis() - mismatchStartTime >= 1000UL) {
          status = !status;
          update_telemetry = true;
          printLog(
              "Status flipped due to consistent mismatch with LED state for 1 second (DAIKIN)\n");
          mismatchStartTime = 0;
          mismatchActive = false;
        }
      } else {
        mismatchStartTime = 0;
        mismatchActive = false;
      }
    }

    static int falseCounter = 0;
    static int totalCount = 0;
    if (status) {
      if (mismatchActive) totalCount++;
      if (status != led_state) {
        if (!mismatchActive) {
          falseCounter = 0;
          totalCount = 0;
          mismatchStartTime = millis();
          mismatchActive = true;
        } else {
          falseCounter++;
          if (millis() - mismatchStartTime >= 1000UL) {
            if (totalCount > 0 && falseCounter > (int)(totalCount * 0.7f)) {
              status = false;
              update_telemetry = true;
              printLog(
                  "Status flipped to false due to >70% mismatch with LED state for 1 second (DAIKIN)\n");
            }
            mismatchStartTime = 0;
            mismatchActive = false;
          }
        }
      }
    }
    return;
  }


if (ir_protocol == "KELVINATOR") {

  static unsigned long kelvin_true_window = 0;
  static unsigned long kelvin_total_window = 0;
  static unsigned long kelvin_window_start = 0;
  static unsigned long kelvin_lastTrueWindow = 0;
  static unsigned long kelvin_lastTotalWindow = 0;

  // NEW: OFF detection state
  static int off_candidate_streak = 0;   // total rendah
  static int fake_off_streak = 0;        // total tinggi
  static String fake_log_buffer = "";

  unsigned long kelvin_now = millis();

  // sampling (UNCHANGED)
  if (acLedValue > 2000) {
    if (acLedValue > 4000) kelvin_true_window++;
    kelvin_total_window++;

    for (int i = sampleCount - 1; i > 0; i--) sampleLED[i] = sampleLED[i - 1];
    sampleLED[0] = (acLedValue > 4000);
  }

  if (kelvin_window_start == 0) kelvin_window_start = kelvin_now;

  if (kelvin_now - kelvin_window_start >= 1000UL) {

    kelvin_lastTrueWindow = kelvin_true_window;
    kelvin_lastTotalWindow = kelvin_total_window;

    // simpan log window (buat fake dump nanti)
    fake_log_buffer += String("trueCount ") + kelvin_lastTrueWindow +
                       " Total " + kelvin_lastTotalWindow + "\n";

    kelvin_true_window = 0;
    kelvin_total_window = 0;
    kelvin_window_start = kelvin_now;

    // =========================
    // NEW OFF DETECTION LOGIC
    // =========================

    bool zero_true  = (kelvin_lastTrueWindow == 0);
    bool high_total = (kelvin_lastTotalWindow > 170);
    bool low_total  = (kelvin_lastTotalWindow < 170);
    bool valid      = (kelvin_lastTotalWindow > 50);

    if (zero_true && valid) {//valid = misal lagi kirim http client to server

      if (low_total) {
        off_candidate_streak++;
        fake_off_streak = 0;
      }
      else if (high_total) {
        fake_off_streak++;
        off_candidate_streak = 0;
      }
      else {
        off_candidate_streak = 0;
        fake_off_streak = 0;
      }

    } else {
      off_candidate_streak = 0;
      fake_off_streak = 0;
      fake_log_buffer = ""; // reset kalau balik normal
    }

    // ===== FAKE OFF DETECTED =====
    if (fake_off_streak >= 2) {
      printLog("Fake OFF detected:\n" + fake_log_buffer);
      fake_off_streak = 0;
      fake_log_buffer = "";
    }

    // ===== REAL OFF (5 detik) =====
    if (status && off_candidate_streak >= 5) {
      status = false;
      update_telemetry = true;
      printLog("AC OFF confirmed\n");

      off_candidate_streak = 0;
      fake_log_buffer = "";
    }
  }

  // =========================
  // LED STATE (JANGAN DIUBAH)
  // =========================
  int trueCount = 0;
  for (int i = 0; i < sampleCount; i++) if (sampleLED[i]) trueCount++;
  led_state = (trueCount > sampleCount / 2);

  // =========================
  // ON DETECTION (UNCHANGED)
  // =========================
  static unsigned long mismatchStartTime = 0;
  static bool missmatch = false;

  if (status != led_state) {
    if (!missmatch) {
      mismatchStartTime = millis();
      missmatch = true;
    } else {
      if (millis() - mismatchStartTime >= 1000UL) {
        if (!status) {
          // tetap sama: OFF → ON cepat
          status = true;
          update_telemetry = true;
          printLog("AC ON confirmed\n");
        }
        mismatchStartTime = 0;
        missmatch = false;
      }
    }
  } else {
    mismatchStartTime = 0;
    missmatch = false;
  }

  digitalWrite(AC_POWER_LED, led_state ? HIGH : LOW);
  return;
}


  if (ir_protocol == "PANASONIC_AC") {
    if (acLedValue > 4000) {
      for (int i = sampleCount - 1; i > 0; i--) sampleLED[i] = sampleLED[i - 1];
      sampleLED[0] = false;
    } else {
      for (int i = sampleCount - 1; i > 0; i--) sampleLED[i] = sampleLED[i - 1];
      sampleLED[0] = true;
    }

    int trueCount = 0;
    for (int i = 0; i < sampleCount; i++) if (sampleLED[i]) trueCount++;
    led_state = (trueCount > sampleCount / 2);

    static unsigned long mismatchStartTime = 0;
    static bool missmatch = false;

    if (status != led_state) {
      if (!missmatch) {
        mismatchStartTime = millis();
        missmatch = true;
      } else if (millis() - mismatchStartTime >= 1000UL) {
        status = !status;
        update_telemetry = true;
        printLog("Status flipped due to mismatch with LED state for 1 second (PANASONIC_AC)\n");
        mismatchStartTime = 0;
        missmatch = false;
      }
    } else {
      mismatchStartTime = 0;
      missmatch = false;
    }

    digitalWrite(AC_POWER_LED, led_state ? HIGH : LOW);
    return;
  }







  
  if (ir_protocol == "DAIKIN64") {//JIKA PAKE RESISTOR
    if (acLedValue > 2000) {
      if (acLedValue > 4000) {
        for (int i = sampleCount - 1; i > 0; i--) sampleLED[i] = sampleLED[i - 1];
        sampleLED[0] = false;
      } else {
        for (int i = sampleCount - 1; i > 0; i--) sampleLED[i] = sampleLED[i - 1];
        sampleLED[0] = true;
      }

      int falseCount = 0;
      for (int i = 0; i < sampleCount; i++) if (!sampleLED[i]) falseCount++;
      led_state = (falseCount < (int)(sampleCount * 0.2f));
      digitalWrite(AC_POWER_LED, led_state ? HIGH : LOW);
    }

  
    ///JIKA PAKE OPTOCOUPLER K1H2



    // if (acLedValue > 4000){
    //     // digitalWrite(AC_POWER_LED, HIGH);
    //     // geser semua ke kanan
    //     for (int i = sampleCount - 1; i > 0; i--) {
    //       sampleLED[i] = sampleLED[i - 1];
    //     }
    //     sampleLED[0] = true;
    //   }
    //   else{
    //     // digitalWrite(AC_POWER_LED, LOW);
    //     // geser semua ke kanan
    //     for (int i = sampleCount - 1; i > 0; i--) {
    //       sampleLED[i] = sampleLED[i - 1];
    //     }
    //     sampleLED[0] = false;
    //   }


    //   int falseCount = 0;
    //   for (int i = 0; i < sampleCount; i++) if (!sampleLED[i]) falseCount++;
    //   led_state = (falseCount < (int)(sampleCount * 0.2f));
    //   digitalWrite(AC_POWER_LED, led_state ? HIGH : LOW);



    // int trueCount = 0;
    // for (int i = 0; i < sampleCount; i++) {
    //   if (sampleLED[i]) {
    //     trueCount++;
    //   }
    // }
    // led_state = (trueCount > sampleCount * 0.2); // jika lebih dari 20% true, anggap true
    // digitalWrite(AC_POWER_LED, led_state ? HIGH : LOW);






    ///
    static unsigned long ledTrueStartTime = 0;
    static bool wasLedStateTrue = false;

    if (!status) {
      if (led_state) {
        if (!wasLedStateTrue) {
          ledTrueStartTime = millis();
          wasLedStateTrue = true;
        } else if (millis() - ledTrueStartTime >= 1000UL) {
          if (!status) {
            status = true;
            update_telemetry = true;
            wasLedStateTrue = false;
            printLog("Status changed to true based on LED state\n");
          }
        }
      } else {
        wasLedStateTrue = false;
        ledTrueStartTime = 0;
      }
    }

    if (status) {
      wasLedStateTrue = false;
      static int ledOnCount = 0;
      static int ledOffCount = 0;
      static unsigned long lastCheckTime = 0;

      if (lastCheckTime == 0) lastCheckTime = millis();

      if (led_state)
        ledOnCount++;
      else
        ledOffCount++;

      if (millis() - lastCheckTime >= 5000UL) {
        if (ledOffCount > ledOnCount) {
          if (status) {
            status = false;
            update_telemetry = true;
            Serial.println("Status changed to false based on LED state (mayoritas off)");
          }
        }
        ledOnCount = 0;
        ledOffCount = 0;
        lastCheckTime = millis();
      }
    }
    return;
  }
}

static void service_loop() {
  server.handleClient();

  // IR receive (moved from IRReceiverTask)
  if (irrecv.decode(&results)) {
    process_ir(&results);
    irrecv.resume();
  }

  update_receive_ir_timeout();
  check_reset_btn();
  update_auto_reboot();
  update_ac_led_indicator();
  force_telemetry_every_minute();

  if (restart) {
    delay(1000);
    ESP.restart();
  }
}

// ===================== IR logic (ported from ac.ino) =====================
bool last_ir_status = false;
static void process_ir(decode_results* resultsPtr) {
  String protocol = read_ir_protocol(resultsPtr);
  Serial.println(kTolerancePercentage);
  if (!receive_ir) {
    printLog("RPC signal received, ignored.\n");
    receive_ir = true;
    return;
  }

  if (protocol == "UNKNOWN") {
    Serial.println("Not detect anything, ignored.");
    return;
  }

  if (ir_protocol == "") {
    Serial.println("IR protocol not set, ignoring signal.");
    return;
  }

  if (ir_protocol == "KELVINATOR") {
    if (!(protocol == "KELVINATOR" || protocol == "GREE")) {
      Serial.printf("%s/GREE not detected, ignoring.\n", ir_protocol.c_str());
      return;
    }
  }

  if (ir_protocol == "DAIKIN" || ir_protocol == "DAIKIN152") {
    if (protocol != "DAIKIN" && protocol != "DAIKIN152" && protocol != "DAIKIN216") {
      Serial.printf("%s OR DAIKIN152 not detected, ignoring.\n", ir_protocol.c_str());
      return;
    }
  }

  if (ir_protocol == "DAIKIN64") {
    if (protocol != "DAIKIN64") {
      Serial.printf("%s not detected, ignoring.\n", ir_protocol.c_str());
      return;
    }
  }

  if (ir_protocol == "PANASONIC_AC") {
    if (protocol != "PANASONIC_AC") {
      Serial.printf("%s not detected, ignoring.\n", ir_protocol.c_str());
      return;
    }
  }

  String description = IRAcUtils::resultAcToString(resultsPtr);

  if (ir_protocol == "DAIKIN" || ir_protocol == "DAIKIN152") {
    int idxPower = description.indexOf("Power: ");
    int idxPowerEnd = description.indexOf(',', idxPower);
    String powerStr = description.substring(idxPower + 7, idxPowerEnd);
    powerStr.trim();
    status = powerStr.equalsIgnoreCase("On");

    int idxTemp = description.indexOf("Temp: ");
    int idxTempEnd = description.indexOf('C', idxTemp);
    String tempStr = description.substring(idxTemp + 6, idxTempEnd);
    tempStr.trim();
    temp = tempStr.toInt();

    int idxSwing = description.indexOf("Swing(V): ");
    int idxSwingEnd = description.indexOf(',', idxSwing);
    String swingStr = description.substring(idxSwing + 10, idxSwingEnd);
    swingStr.trim();
    swing = swingStr.equalsIgnoreCase("On");

    Serial.print("Power: ");
    Serial.println(status);
    Serial.print("Temp: ");
    Serial.println(temp);
    Serial.print("Swing(V): ");
    Serial.println(swing);

    yield();
  }

  if (ir_protocol == "DAIKIN64") {
    int idxPower = description.indexOf("Power Toggle: ");
    int idxPowerEnd = description.indexOf(',', idxPower);
    String powerStr = description.substring(idxPower + 14, idxPowerEnd);
    powerStr.trim();
    if (powerStr.equalsIgnoreCase("On")) {
      Serial.println("STATUS DIUBAH VIA IR");
      status = !status;
    }

    int idxTemp = description.indexOf("Temp: ");
    int idxTempEnd = description.indexOf('C', idxTemp);
    String tempStr = description.substring(idxTemp + 6, idxTempEnd);
    tempStr.trim();
    temp = tempStr.toInt();

    int idxSwing = description.indexOf("Swing(V): ");
    int idxSwingEnd = description.indexOf(',', idxSwing);
    String swingStr = description.substring(idxSwing + 10, idxSwingEnd);
    swingStr.trim();
    swing = swingStr.equalsIgnoreCase("On");

    Serial.print("Power (toggled): ");
    Serial.println(status);
    Serial.print("Temp: ");
    Serial.println(temp);
    Serial.print("Swing(V): ");
    Serial.println(swing);

    yield();
  }

  if (ir_protocol == "KELVINATOR") {
    if (protocol == ir_protocol || protocol == "GREE") {
      int idxPower = description.indexOf("Power: ");
      int idxPowerEnd = description.indexOf(',', idxPower);
      String powerStr = description.substring(idxPower + 7, idxPowerEnd);
      powerStr.trim();
      status = powerStr.equalsIgnoreCase("On");

      int idxTemp = description.indexOf("Temp: ");
      int idxTempEnd = description.indexOf('C', idxTemp);
      String tempStr = description.substring(idxTemp + 6, idxTempEnd);
      tempStr.trim();
      temp = tempStr.toInt();

      int idxSwing = description.indexOf("Swing(V): ");
      int idxSwingEnd = description.indexOf(',', idxSwing);
      String swingStr = description.substring(idxSwing + 10, idxSwingEnd);
      swingStr.trim();
      int spaceIdx = swingStr.indexOf(' ');
      String swingVal = (spaceIdx != -1) ? swingStr.substring(0, spaceIdx) : swingStr;
      swingVal.trim();
      swing = (swingVal == "1");

      Serial.print("Power: ");
      Serial.println(status);
      Serial.print("Temp: ");
      Serial.println(temp);
      Serial.print("Swing(V): ");
      Serial.println(swing);

      yield();
    }
  }

  if (ir_protocol == "PANASONIC_AC") {
    int idxPower = description.indexOf("Power: ");
    int idxPowerEnd = description.indexOf(',', idxPower);
    String powerStr = description.substring(idxPower + 7, idxPowerEnd);
    powerStr.trim();
    status = powerStr.equalsIgnoreCase("On");

    int idxTemp = description.indexOf("Temp: ");
    int idxTempEnd = description.indexOf('C', idxTemp);
    String tempStr = description.substring(idxTemp + 6, idxTempEnd);
    tempStr.trim();
    temp = tempStr.toInt();

    int idxSwing = description.indexOf("Swing(V): ");
    int idxSwingEnd = description.indexOf(',', idxSwing);
    String swingStr = description.substring(idxSwing + 10, idxSwingEnd);
    swingStr.trim();
    int spaceIdx = swingStr.indexOf(' ');
    String swingVal = (spaceIdx != -1) ? swingStr.substring(0, spaceIdx) : swingStr;
    swingVal.trim();
    swing = (swingVal == "15");

    Serial.print("Power: ");
    Serial.println(status);
    Serial.print("Temp: ");
    Serial.println(temp);
    Serial.print("Swing(V): ");
    Serial.println(swing);

    yield();
  }
  last_ir_status = status;
  if (!last_ir_status){
    revert_ir = false;
  }
  if ((temp < minTemp) && status) {
    revert_ir = true;
    temp = minTemp;
    printLog("Temp reverted to minTemp: " + String(minTemp) + "status: " + String(status) + "\n");
  }
  if (temp < minTemp) {
    temp = minTemp;
  }
  update_telemetry = true;
}

static void reverse_ir() {
  if (ir_protocol == "DAIKIN64") {
    send_remote(false, swing, temp);
  } else {
    
    send_remote(last_ir_status, swing, temp);
    
  }
}

static void send_remote(bool statusToSend, bool swingToSend, int tempToSend) {
  digitalWrite(LED_PIN, HIGH);

  if (ir_protocol == "DAIKIN" || ir_protocol == "DAIKIN152") {
    printLog("Sending DAIKIN IR command\n");
    acDaikin.setMode(kDaikinCool);
    acDaikin.setFan(3);
    if (statusToSend)
      acDaikin.on();
    else
      acDaikin.off();

    acDaikin.setSwingVertical(swingToSend);
    acDaikin.setTemp(tempToSend);
    acDaikin.send();
    Serial.println(acDaikin.toString());
  }

  if (ir_protocol == "KELVINATOR") {
    printLog("Sending KELVINATOR IR command\n");
    acKelvinator.setMode(kKelvinatorCool);
    acKelvinator.setFan(1);
    acKelvinator.setXFan(true);
    acKelvinator.setIonFilter(false);
    acKelvinator.setLight(true);
    if (statusToSend)
      acKelvinator.on();
    else
      acKelvinator.off();

    if (swingToSend)
      acKelvinator.setSwingVertical(true, kKelvinatorSwingVAuto);
    else
      acKelvinator.setSwingVertical(false, kKelvinatorSwingVOff);

    acKelvinator.setTemp(tempToSend);
    acKelvinator.send();
    Serial.println(acKelvinator.toString());
  }

  if (ir_protocol == "DAIKIN64") {
    printLog("Sending DAIKIN64 IR command\n");
    acDaikin64.setMode(kDaikinCool);
    acDaikin64.setFan(1);
    acDaikin64.setPowerToggle(statusToSend);
    acDaikin64.setSwingVertical(swingToSend);
    acDaikin64.setTemp(tempToSend);
    acDaikin64.send();
    Serial.println(acDaikin64.toString());
  }

  if (ir_protocol == "PANASONIC_AC") {
    printLog("Sending PANASONIC AC IR command\n");
    acPanasonic.setModel(kPanasonicRkr);
    if (statusToSend)
      acPanasonic.on();
    else
      acPanasonic.off();

    acPanasonic.setFan(kPanasonicAcFanAuto);
    acPanasonic.setMode(kPanasonicAcCool);
    acPanasonic.setTemp(tempToSend);
    if (swingToSend)
      acPanasonic.setSwingVertical(kPanasonicAcSwingVAuto);
    else
      acPanasonic.setSwingVertical(kPanasonicAcSwingVHigh);

    acPanasonic.setSwingHorizontal(kPanasonicAcSwingHAuto);
    acPanasonic.send();
    Serial.println(acPanasonic.toString());
  }
}

// ===================== WiFi logic (ported) =====================

static void beginWifi() {
  if (channel > 0 && (mac[0] != 0 || mac[1] != 0 || mac[2] != 0 || mac[3] != 0 || mac[4] != 0 || mac[5] != 0)) {
    WiFi.begin(ssid.c_str(), password.c_str(), channel, mac);
    Serial.printf("Menggunakan MAC: %02X:%02X:%02X:%02X:%02X:%02X dan channel %d\n", mac[0], mac[1], mac[2], mac[3],
                  mac[4], mac[5], channel);
  } else if (channel > 0) {
    WiFi.begin(ssid.c_str(), password.c_str(), channel);
    Serial.printf("Menggunakan channel %d\n", channel);
  } else {
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.println("Menggunakan default channel");
  }
}

static void connectWifi() {
  Serial.println("menghubungkan ke wifi");
  if (ip != "" && gateway != "" && subnet != "") {
    IPAddress local_IP, local_gateway, local_subnet, local_dns;
    local_IP.fromString(ip);
    local_gateway.fromString(gateway);
    local_subnet.fromString(subnet);
    if (dns != "") {
      local_dns.fromString(dns);
      WiFi.config(local_IP, local_gateway, local_subnet, local_dns);
    } else {
      WiFi.config(local_IP, local_gateway, local_subnet);
    }
    Serial.printf("Menggunakan IP static: %s\n", ip.c_str());
  }

  beginWifi();

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startAttemptTime > kWiFiConnectTimeoutMs) {
      Serial.println("\nGagal terhubung ke WiFi setelah 30 detik.");
      stop_request = true;
      break;
    }
    delay(50);
    Serial.print('.');
    service_loop();
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nTerhubung ke WiFi!");
  } else {
    Serial.println("\nGagal terhubung ke WiFi. retry");
  }
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP());
}
bool server_connected = false;
//auto reconnect after wifi disconenct
static void reconnectWifiBlocking(const char* reason) {
  if (reason && reason[0]) {
    printLog(String("reconnect wifi: ") + reason + "\n");
  } else {
    printLog("reconnect wifi.\n");
  }

  wifiConnected = false;

  uint8_t storedMac[6]; //save mac to try connect other mac
  memcpy(storedMac, mac, sizeof(storedMac));
  const int storedChannel = channel;
  
  auto isAllZeroMac = [](const uint8_t* m) -> bool {
    for (int i = 0; i < 6; i++) {
      if (m[i] != 0) return false;
    }
    return true;
  };

  auto macToString = [](const uint8_t* m) -> String {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
    return String(buf);
  };

  const bool hasStoredBssid = !isAllZeroMac(storedMac);
  bool scanDone = false;
  bool pendingPersistNewBssid = false;//check client + server after connect with new bssid, if ok then save to preferences

  const unsigned long kTimeoutMs = 60000UL * 10;
  const unsigned long kScanAfterMs = 60000UL;  // 1 menit
  unsigned long reconnectStart = millis();

  while (WiFi.status() != WL_CONNECTED) {
    if (!scanDone && (millis() - reconnectStart > kScanAfterMs)) {
      scanDone = true;
      printLog("WiFi reconnect masih gagal >1 menit. Scan WiFi...\n");
      WiFi.disconnect(true);
      scanWifi();

      int bestSameSsidIndex = -1;
      int32_t bestSameSsidRssi = -99999;
      bool foundExactSsidAndMac = false;

      for (int i = 0; i < wifiListCount; i++) {
        if (ssidlist[i] != ssid) continue;//skip ssid yang beda

        if (hasStoredBssid && memcmp(maclist[i], storedMac, 6) == 0) {
          foundExactSsidAndMac = true;//SSID dan mac lama masih ada, continue reconnect
          break;
        }

        if (rssiList[i] > bestSameSsidRssi) {//search for best rssi with same ssid (wifi mesh)
          bestSameSsidRssi = rssiList[i];
          bestSameSsidIndex = i;
        }
      }

      if (foundExactSsidAndMac) {
        printLog("Scan: ditemukan SSID + MAC yang sama. Lanjut retry sampai reboot bila perlu.\n");
      } else if (bestSameSsidIndex >= 0) {
        // SSID sama ada, tapi BSSID berbeda (router/AP berubah). Pakai BSSID + channel baru.
        memcpy(mac, maclist[bestSameSsidIndex], 6);
        channel = channelList[bestSameSsidIndex];
        pendingPersistNewBssid = true;

        printLog("Scan: SSID sama ditemukan tapi MAC beda. Coba pakai MAC baru: " + macToString(mac) +
                 " channel " + String(channel) + "\n");
      } else {
        printLog("Scan: SSID tidak ditemukan. Lanjut retry biasa sampai reboot.\n");
      }
    }

    WiFi.disconnect(false);
    delay(300);

    beginWifi();



    //main reconnect code
    for (int retry = 0; retry < 20 && WiFi.status() != WL_CONNECTED; retry++) {
      if (millis() - reconnectStart > kTimeoutMs) {
        printLog("Reconnect WiFi timeout 10 menit. Reboot...\n");
        disabling_boot_ir();
        delay(500);
        ESP.restart();
      }
      delay(500);
      Serial.print('.');
      service_loop();
    }


    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi reconnected!");
      wifiConnected = true;

      if (pendingPersistNewBssid) {
        // Test MQTT connect + telemetry sebelum simpan MAC baru ke preferences.
        bool mqttTestOk = false;
        bool telemetryTestOk = false;

        if (tbserver != "" && deviceToken != "") {
          connectMQTT();
          unsigned long waitStart = millis();
          while (!client.connected() && (millis() - waitStart < 5000UL)) {
            delay(50);
            service_loop();
          }
          mqttTestOk = client.connected();
        }

        if (tbserver != "" && tbuser != "" && tbpass != "" && deviceID != "") {
          server_connected = false;
          send_Telemetry(status, swing, temp);
          telemetryTestOk = server_connected;
        }

        if (mqttTestOk && telemetryTestOk) {
          const String newMacStr = macToString(mac);
          preferences.begin("my-app", false);
          preferences.putString("mac", newMacStr);
          preferences.putInt("channel", channel);
          preferences.end();

          printLog("MAC baru tersimpan ke preferences: " + newMacStr + " (channel " + String(channel) + ")\n");
        } else {
          printLog("WiFi tersambung dengan MAC baru, tapi test MQTT/telemetry gagal. MAC tidak disimpan dan dikembalikan ke MAC lama.\n");

          // Kembalikan ke MAC/channel lama untuk percobaan berikutnya.
          memcpy(mac, storedMac, 6);
          channel = storedChannel;
          pendingPersistNewBssid = false;

          // Putuskan koneksi yang saat ini terhubung (pakai MAC baru), lalu lanjut loop agar reconnect memakai MAC lama.
          WiFi.disconnect(true);
          delay(200);
          wifiConnected = false;
          Serial.println("\nRetrying WiFi connection (reverted MAC)...");
          service_loop();
          continue;
        }
      }

      break;
    }

    Serial.println("\nRetrying WiFi connection...");
    delay(500);
    service_loop();
  }
}

static void scanWifi() {
  int n = WiFi.scanNetworks();
  Serial.printf("scan n=%d status=%d\n", n, WiFi.status());
  if (n <= 0) {
    wifiListCount = 0;
    WiFi.scanDelete();
    wifiScan = false;
    showWifi = false;
    return;
  }

  if (n > 0) {
    for (int i = 0; i < wifiListLen; i++) {
      ssidlist[i] = "";
      securityList[i] = "";
      rssiList[i] = 0;
    }
    wifiListCount = (n < wifiListLen) ? n : wifiListLen;
    for (int i = 0; i < wifiListCount; i++) {
      ssidlist[i] = WiFi.SSID(i);
      rssiList[i] = WiFi.RSSI(i);
      channelList[i] = WiFi.channel(i);

      uint8_t* bssid = WiFi.BSSID(i);
      memcpy(maclist[i], bssid, 6);

      switch (WiFi.encryptionType(i)) {
        case WIFI_AUTH_OPEN:
          securityList[i] = "OPEN";
          break;
        case WIFI_AUTH_WEP:
        case WIFI_AUTH_WPA_PSK:
        case WIFI_AUTH_WPA2_PSK:
        case WIFI_AUTH_WPA_WPA2_PSK:
        case WIFI_AUTH_WPA2_ENTERPRISE:
          securityList[i] = "SECURED";
          break;
        default:
          securityList[i] = "UNKNOWN";
          break;
      }

      char macStr[18];
      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", maclist[i][0], maclist[i][1], maclist[i][2], maclist[i][3],
               maclist[i][4], maclist[i][5]);
      Serial.printf("SSID: %s, RSSI: %d, Channel: %d, MAC: %s, Security: %s\n", ssidlist[i].c_str(), rssiList[i], channelList[i],
                    macStr, securityList[i].c_str());
    }

    WiFi.scanDelete();
    wifiScan = false;
    showWifi = false;
  }
}

bool getStatusSuccess = false;
static void setupSSID() {
  if (ssid == "") {
    Serial.println("SSID & password empty, using AP mode");
    String apSuffix = "";
    for (int i = 0; i < 4; i++) apSuffix += char(random(65, 91));

    WiFi.mode(WIFI_AP_STA);
    String apSSID = "SmartAC-" + ((OTAName.length() > 0) ? OTAName : apSuffix);
    WiFi.softAP(apSSID.c_str());
    Serial.print("Please Connect to: ");
    Serial.println(apSSID);
    Serial.printf("Then open http://%s in your browser\n", WiFi.softAPIP().toString().c_str());
    wifiScan = true;
    return;
  }

  Serial.println("SSID terisi");
  connectWifi();
  getToken(tbserver, tbuser, tbpass);
  getAccessToken(tbserver, deviceID);
  connectMQTT();
  service_loop();
  getStatus();


  wifiConnected = true;
}

// ===================== HTTP client (ported) =====================

static void getToken(String url, String username, String password) {
  HTTPClient http;
  http.begin("http://" + url + ":" + server_port + "/api/auth/login");
  http.addHeader("Content-Type", "application/json");
  String payload = "{\"username\":\"" + username + "\",\"password\":\"" + password + "\"}";
  int httpResponseCode = http.POST(payload);
  if (httpResponseCode > 0) {
    String tbresponse = http.getString();
    int tokenIndex = tbresponse.indexOf("\"token\":\"");
    if (tokenIndex != -1) {
      int start = tokenIndex + 9;
      int end = tbresponse.indexOf("\"", start);
      if (end != -1) {
        token = tbresponse.substring(start, end);
        Serial.println("Token found: " + token);
        server_ok = true;
        user_ok = true;
      }
    } else {
      server_ok = true;
      user_ok = false;
      device_ok = false;
      mqtt_ok = false;
      stop_request = true;
      Serial.println("HTTP Response: " + tbresponse);
    }
  } else {
    server_ok = false;
    user_ok = false;
    device_ok = false;
    mqtt_ok = false;
    stop_request = true;
    Serial.printf("HTTP POST failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  http.end();
}

static void getAccessToken(String url, String deviceIDLocal) {
  getToken(tbserver, tbuser, tbpass);
  HTTPClient http;
  http.begin("http://" + url + ":" + server_port + "/api/device/" + deviceIDLocal + "/credentials");
  http.addHeader("Authorization", "Bearer " + token);
  int httpResponseCode = http.GET();
  if (httpResponseCode > 0) {
    String response = http.getString();
    int credIndex = response.indexOf("\"credentialsId\":\"");
    if (credIndex != -1) {
      int start = credIndex + 17;
      int end = response.indexOf("\"", start);
      if (end != -1) {
        deviceToken = response.substring(start, end);
        Serial.println("Device Token found: " + deviceToken);
        device_ok = true;
      }
    } else {
      device_ok = false;
      mqtt_ok = false;
      stop_request = true;
      Serial.println("Device Credentials Response: " + response);
    }
  } else {
    server_ok = false;
    user_ok = false;
    device_ok = false;
    mqtt_ok = false;
    stop_request = true;
    Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  http.end();
}

static void send_Telemetry(bool statusLocal, bool swingLocal, int tempLocal) {
  if (tbserver != "" && tbuser != "" && tbpass != "") {
    getToken(tbserver, tbuser, tbpass);
    HTTPClient http;
    http.begin("http://" + tbserver + ":" + server_port + "/api/plugins/telemetry/DEVICE/" + deviceID +
               "/timeseries/SERVER_SCOPE");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + token);

    String payload = "{\"temp\":" + String(tempLocal) + ",\"status\":" + String(statusLocal ? "true" : "false") +
                     ",\"swing\":" + String(swingLocal ? "true" : "false") + "}";

    int httpResponseCode = http.POST(payload);
    if (httpResponseCode == 200) {
      Serial.printf("Telemetry sent successfully, response code: %d\n", httpResponseCode);
      update_telemetry = false;
      server_connected = true;
      digitalWrite(LED_PIN, LOW);
    } else {
      server_ok = false;
      server_connected = false;
      Serial.printf("HTTP POST failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    http.end();
  }
}

static void send_log(String* log_array) {
  if (tbserver != "" && tbuser != "" && tbpass != "" && logQueue > 0) {
    int oldest_index = logQueue - 1;
    String oldest_message = log_array[oldest_index];

    oldest_message.replace("\\", "\\\\");
    oldest_message.replace("\"", "\\\"");
    oldest_message.replace("\r", "\\r");
    oldest_message.replace("\n", "\\n");

    getToken(tbserver, tbuser, tbpass);
    HTTPClient http;
    http.begin("http://" + tbserver + ":" + server_port + "/api/plugins/telemetry/DEVICE/" + deviceID +
               "/timeseries/SERVER_SCOPE");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + token);

    String payload = "{\"ip\":\"" + WiFi.localIP().toString() + "\",\"message\":\"" + oldest_message + "\"}";

    int httpResponseCode = http.POST(payload);
    if (httpResponseCode == 200) {
      Serial.printf("Log sent successfully, response code: %d\n", httpResponseCode);
      logQueue--;
    } else {
      Serial.printf("HTTP POST failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    http.end();
  }
}

static void getStatus() {
  if (tbserver != "" && tbuser != "" && tbpass != "") {
    getToken(tbserver, tbuser, tbpass);
    HTTPClient http;
    http.begin("http://" + tbserver + ":" + server_port +
               "/api/plugins/telemetry/DEVICE/" + deviceID +
               "/values/timeseries?keys=status,swing,temp,minTemp,beep&useStrictDataTypes=true");
    http.addHeader("Authorization", "Bearer " + token);
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      String response = http.getString();
      if (debug) Serial.println("Telemetry Response: " + response);



      int statusArrIdx = response.indexOf("\"status\":[");
      if (statusArrIdx != -1) {
        int valueIdx = response.indexOf("\"value\":", statusArrIdx);
        if (valueIdx != -1) {
          int valueStart = valueIdx + 8;
          int valueEnd = response.indexOf("}", valueStart);
          String statusValue = response.substring(valueStart, valueEnd);
          statusValue.trim();
          status = (statusValue == "true");
        }
      }

      int swingArrIdx = response.indexOf("\"swing\":[");
      if (swingArrIdx != -1) {
        int valueIdx = response.indexOf("\"value\":", swingArrIdx);
        if (valueIdx != -1) {
          int valueStart = valueIdx + 8;
          int valueEnd = response.indexOf("}", valueStart);
          String swingValue = response.substring(valueStart, valueEnd);
          swingValue.trim();
          swing = (swingValue == "true");
        }
      }

      int tempArrIdx = response.indexOf("\"temp\":[");
      if (tempArrIdx != -1) {
        int valueIdx = response.indexOf("\"value\":", tempArrIdx);
        if (valueIdx != -1) {
          int valueStart = valueIdx + 8;
          int valueEnd = response.indexOf("}", valueStart);
          String tempValue = response.substring(valueStart, valueEnd);
          tempValue.trim();
          temp = tempValue.toInt();
        }
      }

      int minTempArrIdx = response.indexOf("\"minTemp\":[");
      if (minTempArrIdx != -1) {
        int valueIdx = response.indexOf("\"value\":", minTempArrIdx);
        if (valueIdx != -1) {
          int valueStart = valueIdx + 8;
          int valueEnd = response.indexOf("}", valueStart);
          String minTempValue = response.substring(valueStart, valueEnd);
          minTempValue.trim();
          minTemp = minTempValue.toInt();
        }
      }

      int beepArrIdx = response.indexOf("\"beep\":[");
      if (beepArrIdx != -1) {
        int valueIdx = response.indexOf("\"value\":", beepArrIdx);
        if (valueIdx != -1) {
          int valueStart = valueIdx + 8;
          int valueEnd = response.indexOf("}", valueStart);
          String beepValue = response.substring(valueStart, valueEnd);
          beepValue.trim();
          if (beepValue.length() > 0 && beepValue != "null") {
            beep = (beepValue == "true");
          }
        }
      }

      if (!disable_boot_ir) {
        if (beep) {
          if (ir_protocol == "DAIKIN64") {
            send_remote(false, swing, temp);
          } else {
            send_remote(status, swing, temp);
          }
        }
      }

      // preferences.begin("my-app", false);
      // preferences.putBool("disable_boot_ir", false);
      // preferences.end();

      digitalWrite(LED_PIN, LOW);
      Serial.printf("Parsed status: %d, swing: %d, temp: %d, minTemp: %d\n", status, swing, temp, minTemp);
      getStatusSuccess = true;
    } else {
      Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
      getStatusSuccess = false;
    }
    http.end();
  }
}

// ===================== OTA update (ported) =====================

static bool startFirmwareUpdate(const String& url, const String& md5) {
  updatePercent = 0;
  updateStatus = 1;

  if (md5.length()) Update.setMD5(md5.c_str());
  else Update.setMD5("");

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
      Serial.println("OTA error: http.begin(https) failed");
      updateStatus = 3;
      return false;
    }
  } else {
    if (!http.begin(plain, url)) {
      Serial.println("OTA error: http.begin(http) failed");
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

  WiFiClient* stream = http.getStreamPtr();
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
      service_loop();
      continue;
    }

    int toRead = (avail > sizeof(buf)) ? sizeof(buf) : (int)avail;
    int r = stream->readBytes(buf, toRead);
    if (r <= 0) {
      Serial.println("OTA error: readBytes returned <= 0");
      break;
    }

    if (Update.write(buf, (size_t)r) != (size_t)r) {
      Serial.printf("OTA error: Update.write failed: %s\n", Update.errorString());
      Update.abort();
      http.end();
      updateStatus = 3;
      return false;
    }

    written += (size_t)r;
    if (total > 0) {
      updatePercent = (int)((written * 100ULL) / (unsigned long long)total);
      if (updatePercent > 100) updatePercent = 100;
    }
    service_loop();
  }

  if (!Update.end(true)) {
    Serial.printf("OTA error: Update.end failed: %s\n", Update.errorString());
    http.end();
    updateStatus = 3;
    return false;
  }

  http.end();
  updatePercent = 100;
  updateStatus = 2;
  Serial.println("OTA: Update success");
  return true;
}

// ===================== MQTT (ported) =====================

static void processMqtt(bool statusupdate, bool swingupdate, int tempupdate) {
  if (use_AC_LED) {
    status_new = statusupdate;
    swing_new = swingupdate;
    temp_new = tempupdate;
    mqtt_request = true;
  } else {
    send_remote(statusupdate, swingupdate, tempupdate);
    if (ir_protocol == "DAIKIN64") {
      if (statusupdate == true) status = !status;
      swing = swingupdate;
      temp = tempupdate;
    } else {
      status = statusupdate;
      swing = swingupdate;
      temp = tempupdate;
    }
    update_telemetry = true;
  }
}

static void rpcCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  printLog("RPC Received: " + message + "\n");

  if (message.indexOf("\"method\":\"setState\"") != -1) {
    int paramsIndex = message.indexOf("\"params\":");
    if (paramsIndex != -1) {
      int valueStart = message.indexOf(":", paramsIndex) + 1;
      int valueEnd = message.indexOf("}", valueStart);
      String paramValue = message.substring(valueStart, valueEnd);
      paramValue.trim();
      digitalWrite(LED_PIN, HIGH);

      receive_ir = false;
      if (ir_protocol == "DAIKIN64") {
        if (paramValue == (status ? "true" : "false"))
          processMqtt(false, swing, temp);
        else
          processMqtt(true, swing, temp);
      } else {
        processMqtt((paramValue == "true"), swing, temp);
      }
    }
  }

  if (message.indexOf("\"method\":\"setSwing\"") != -1) {
    int paramsIndex = message.indexOf("\"params\":");
    if (paramsIndex != -1) {
      int valueStart = message.indexOf(":", paramsIndex) + 1;
      int valueEnd = message.indexOf("}", valueStart);
      String paramValue = message.substring(valueStart, valueEnd);
      paramValue.trim();
      digitalWrite(LED_PIN, HIGH);

      receive_ir = false;
      if (ir_protocol == "DAIKIN64")
        processMqtt(false, (paramValue == "true"), temp);
      else
        processMqtt(status, (paramValue == "true"), temp);
    }
  }

  if (message.indexOf("\"method\":\"setTemp\"") != -1) {
    int paramsIndex = message.indexOf("\"params\":");
    if (paramsIndex != -1) {
      int valueStart = message.indexOf(":", paramsIndex) + 1;
      int valueEnd = message.indexOf("}", valueStart);
      String paramValue = message.substring(valueStart, valueEnd);
      paramValue.trim();
      int newTemp = paramValue.toInt();
      digitalWrite(LED_PIN, HIGH);

      if (newTemp > 17) {
        if (newTemp < minTemp) {
          newTemp = minTemp;
          printLog("Temp reverted to minTemp: " + String(minTemp) + "\n");
        }
        receive_ir = false;
        if (ir_protocol == "DAIKIN64")
          processMqtt(false, swing, newTemp);
        else
          processMqtt(status, swing, newTemp);
      }
    }
  }
}

static void connectMQTT() {
  if (tbserver != "" && deviceToken != "") {
    client.setServer(tbserver.c_str(), 1883);
    client.connect(deviceID.c_str(), deviceToken.c_str(), NULL);
    client.subscribe("v1/devices/me/rpc/request/+");
    client.setCallback(rpcCallback);
    runMQTT = true;
    if (client.connected()) {
      mqtt_ok = true;
      stop_request = true;
      printLog("Connected to MQTT broker: " + tbserver + "\n");
    } else {
      mqtt_ok = false;
      stop_request = true;
      printLog("MQTT DISCONNECTED\n");
    }
  }
}

static void led_confirm_process() {
  bool lastLedState = led_state;
  int acResponseTimeout = 3000;
  int retryCount = 0;
  unsigned long startTime = millis();
  bool irSuccess = false;

  stop_led_check = true;

  if (ir_protocol == "DAIKIN64") {
    for (retryCount = 0; retryCount < 10; retryCount++) {
      mqtt_request = false;
      stop_led_check = true;
      send_remote(status_new, swing_new, temp_new);
      startTime = millis();
      delay(200);
      stop_led_check = false;

      while (millis() - startTime < (unsigned long)acResponseTimeout) {
        update_ac_led_indicator();
        if (lastLedState != led_state) {
          if (status_new == true) {
            Serial.println("UPDATE STATUS VIA MQTT");
            status = !status;
          }
          swing = swing_new;
          temp = temp_new;
          irSuccess = true;
          update_telemetry = true;
          return;
        }
        delay(1);
        service_loop();
      }

      printLog("Retry " + String(retryCount + 1) + ": AC did not respond within " + String(acResponseTimeout) + "ms\n");
    }
  }

  if (ir_protocol == "KELVINATOR" || ir_protocol == "PANASONIC_AC") {
    for (retryCount = 0; retryCount < ((status_new != status) ? 10 : 1); retryCount++) {
      mqtt_request = false;
      stop_led_check = true;
      send_remote(status_new, swing_new, temp_new);
      startTime = millis();
      delay(200);
      stop_led_check = false;

      if (status_new == status) {
        status = status_new;
        swing = swing_new;
        temp = temp_new;
        irSuccess = true;
        update_telemetry = true;
        return;
      }

      while (millis() - startTime < (unsigned long)acResponseTimeout) {
        update_ac_led_indicator();
        if (led_state == status_new) {
          status = status_new;
          swing = swing_new;
          temp = temp_new;
          irSuccess = true;
          update_telemetry = true;
          return;
        }
        delay(1);
        service_loop();
      }
    }
  }

  if (ir_protocol == "DAIKIN" || ir_protocol == "DAIKIN152") {
    for (retryCount = 0; retryCount < ((status_new != status) ? 10 : 1); retryCount++) {
      mqtt_request = false;
      stop_led_check = true;
      send_remote(status_new, swing_new, temp_new);
      startTime = millis();
      delay(200);
      stop_led_check = false;

      if (status_new == status) {
        status = status_new;
        swing = swing_new;
        temp = temp_new;
        irSuccess = true;
        update_telemetry = true;
        return;
      }

      int sameCount = 0;
      while (millis() - startTime < (unsigned long)acResponseTimeout) {
        update_ac_led_indicator();
        if (led_state == status_new) {
          sameCount++;
          if (sameCount > 10) {
            status = status_new;
            swing = swing_new;
            temp = temp_new;
            irSuccess = true;
            update_telemetry = true;
            return;
          }
        } else {
          sameCount = 0;
        }
        delay(1);
        service_loop();
      }
    }
  }

  if (!irSuccess) {
    printLog("AC did not respond after 10 retries, sending old status\n");
    update_telemetry = true;
    stop_led_check = false;
  }
}

// ===================== Web server (WebServer.h) =====================

static void serverResponse() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", landingPage); });

  server.on("/getWifiList", HTTP_GET, []() {
    Serial.printf("wifiscan: %d, showWifi: %d\n", wifiScan, showWifi);
    if (!wifiScan && !showWifi) {
      Serial.println("showing wifi list, next time will rescan");
      showWifi = true;
      String json = "[";
      for (int i = 0; i < wifiListCount; i++) {
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", maclist[i][0], maclist[i][1], maclist[i][2], maclist[i][3],
                 maclist[i][4], maclist[i][5]);

        json += "{\"ssid\":\"" + ssidlist[i] + "\",";
        json += "\"rssi\":" + String(rssiList[i]) + ",";
        json += "\"security\":\"" + securityList[i] + "\",";
        json += "\"mac\":\"" + String(macStr) + "\",";
        json += "\"channel\":" + String(channelList[i]);
        json += "}";
        if (i < wifiListCount - 1) json += ",";
      }
      json += "]";
      server.send(200, "application/json", json);
    } else if (showWifi && !wifiScan) {
      Serial.println("rescan mode");
      wifiScan = true;
      server.send(200, "application/json", "[]");
    } else {
      server.send(200, "application/json", "[]");
    }
  });

  server.on("/setDevice", HTTP_POST, []() {
    ssid = server.arg("ssid");
    password = server.arg("password");
    ip = server.arg("ip");
    gateway = server.arg("gateway");
    subnet = server.arg("subnet");
    dns = server.arg("dns");

    if (server.hasArg("deviceName")) {
      String deviceName = server.arg("deviceName");
      if (deviceName.length() > 0) OTAName = deviceName;
    }

    tbserver = server.arg("serverIp");
    tbuser = server.arg("username");
    tbpass = server.arg("serverPassword");
    deviceID = server.arg("deviceId");
    ir_protocol = server.arg("acmodel");

    String macStr = server.arg("mac");
    String channelStr = server.arg("channel");

    Serial.println("Received WiFi and Server Config:");
    Serial.println("SSID: " + ssid);
    Serial.println("Password: " + password);
    Serial.println("IP: " + ip);
    Serial.println("Gateway: " + gateway);
    Serial.println("Subnet: " + subnet);
    Serial.println("DNS: " + dns);
    Serial.println("Device Name: " + OTAName);
    Serial.println("Server IP: " + tbserver);
    Serial.println("Username: " + tbuser);
    Serial.println("Server Password: " + tbpass);
    Serial.println("Device ID: " + deviceID);
    Serial.println("IR Protocol: " + ir_protocol);
    Serial.println("MAC: " + macStr);
    Serial.println("Channel: " + channelStr);

    channel = channelStr.toInt();
    sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);

    trytoconnect = true;
    server.send(200, "text/plain", "Config received");
  });

  server.on("/getConnect", HTTP_GET, []() {
    String json = "{";
    json += "\"wifiConnected\":" + String(wifiConnected ? "true" : "false") + ",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"server_ok\":" + String(server_ok ? "true" : "false") + ",";
    json += "\"user_ok\":" + String(user_ok ? "true" : "false") + ",";
    json += "\"device_ok\":" + String(device_ok ? "true" : "false") + ",";
    json += "\"mqtt_ok\":" + String(mqtt_ok ? "true" : "false") + ",";
    json += "\"stop_request\":" + String(stop_request ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/getInfo", HTTP_GET, []() {
    String wifiStatus = wifiConnected ? "connected to " : "disconnected";
    String ipstatus = "Not Set";
    if (wifiConnected) ipstatus = WiFi.localIP().toString() + " (DHCP / auto)";
    if (ip != "" && gateway != "" && subnet != "") ipstatus = ip + " " + gateway + " " + subnet;

    String serverStatus = (token != "") ? "online" : "Offline";
    String usernameStatus = (token != "") ? tbuser : "Not Set";
    String devicestatus = (deviceToken != "") ? "bind to " + deviceID : "Not Set";
    String mqttStatus = client.connected() ? "connected" : "disconnected";

    String statusStr = "unknown";
    String swingStr = "unknown";
    String tempStr = "unknown";
    if (String(temp) != "") {
      statusStr = status ? "true" : "false";
      swingStr = swing ? "true" : "false";
      tempStr = String(temp);
    }

    String json = "{\"wifi\":\"" + wifiStatus + ssid + "\",\"ip\":\"" + ipstatus + "\",\"server\":\"" + serverStatus +
                  "\",\"username\":\"" + usernameStatus + "\",\"deviceId\":\"" + devicestatus + "\",\"mqtt\":\"" +
                  mqttStatus + "\",\"status\":\"" + statusStr + "\",\"swing\":\"" + swingStr + "\",\"temp\":\"" + tempStr +
                  "\"}";

    server.send(200, "application/json", json);
  });

  server.on("/saveConfig", HTTP_POST, []() {
    if (String(temp) != "") {
      preferences.begin("my-app", false);
      preferences.putString("ssid", ssid);
      preferences.putString("password", password);
      preferences.putString("ip", ip);
      preferences.putString("gateway", gateway);
      preferences.putString("subnet", subnet);
      preferences.putString("dns", dns);
      preferences.putString("OTAName", OTAName);
      preferences.putString("server", tbserver);
      preferences.putString("username", tbuser);
      preferences.putString("tbpassword", tbpass);
      preferences.putString("deviceID", deviceID);
      preferences.putString("deviceToken", deviceToken);
      preferences.putString("ir_protocol", ir_protocol);
      preferences.putString("mac", String(mac[0], HEX) + ":" + String(mac[1], HEX) + ":" + String(mac[2], HEX) + ":" +
                                  String(mac[3], HEX) + ":" + String(mac[4], HEX) + ":" + String(mac[5], HEX));
      preferences.putInt("channel", channel);

      if (ir_protocol == "DAIKIN64") {
        preferences.putBool("status", status);
        preferences.putBool("swing", swing);
        preferences.putInt("temp", temp);
      }

      preferences.end();
      server.send(200, "text/plain", "ok");
      restart = true;
    } else {
      server.send(400, "text/plain", "error: some parameter is not set");
    }
  });

  server.on("/reset", HTTP_POST, []() {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Reset button pressed! Clearing preferences and rebooting...");
    preferences.begin("my-app", false);
    preferences.clear();
    preferences.end();
    restart = true;
    server.send(200, "text/plain", "Device will reset");
  });

  server.on("/reboot", HTTP_POST, []() {
    server.send(200, "text/plain", "Rebooting");
    restart = true;
  });

  server.on("/update", HTTP_GET, []() {
    
    String page = FPSTR(updatePage);
    page.replace("{{OTA_NAME}}", OTAName);
    server.send(200, "text/html", page);
    
  });

  server.on("/updateinfo", HTTP_GET, []() {
    String done = "wait";
    if (updateStatus == 2)
      done = "ok";
    else if (updateStatus == 3)
      done = "fail";

    String json = "{";
    json += "\"percent\":" + String(updatePercent) + ",";
    json += "\"done\":\"" + done + "\"";
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/startupdate", HTTP_POST, []() {
    if (server.hasArg("url")) {
      updateUrl = server.arg("url");
      updateMd5 = server.hasArg("md5") ? server.arg("md5") : "";
      updatefirmware = true;
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Missing url");
    }
  });

  server.on(
      "/updateViaFile", HTTP_POST,
      []() {
        if (updateStatus == 2) {
          server.send(200, "text/plain", "OK");
        } else {
          server.send(500, "text/plain", "FAIL");
        }
      },
      []() {
        HTTPUpload& upload = server.upload();

        if (upload.status == UPLOAD_FILE_START) {
          updatefirmware = false;
          updateUrl = "";
          updateMd5 = "";

          updatePercent = 0;
          updateStatus = 1;
          updateExpectedSize = 0;

          if (!has_bin_extension(upload.filename)) {
            Serial.printf("UpdateViaFile error: invalid extension: %s\n", upload.filename.c_str());
            updateStatus = 3;
            return;
          }

          const String sizeHeader = server.header("X-File-Size");
          if (sizeHeader.length()) {
            updateExpectedSize = (size_t)sizeHeader.toInt();
          }

          Update.setMD5("");

          const size_t beginSize = updateExpectedSize > 0 ? updateExpectedSize : UPDATE_SIZE_UNKNOWN;
          if (!Update.begin(beginSize)) {
            Serial.printf("UpdateViaFile error: Update.begin failed: %s\n", Update.errorString());
            updateStatus = 3;
            return;
          }

          Serial.printf("UpdateViaFile start: filename=%s expected=%u\n", upload.filename.c_str(), (unsigned)updateExpectedSize);
          return;
        }

        if (upload.status == UPLOAD_FILE_WRITE) {
          if (updateStatus != 1) return;

          const size_t wrote = Update.write(upload.buf, upload.currentSize);
          if (wrote != upload.currentSize) {
            Serial.printf("UpdateViaFile error: Update.write failed: %s\n", Update.errorString());
            updateStatus = 3;
            return;
          }

          if (updateExpectedSize > 0) {
            updatePercent = (int)((upload.totalSize * 100ULL) / (unsigned long long)updateExpectedSize);
            if (updatePercent > 99) updatePercent = 99;
          } else {
            // Fallback progress when total size is unknown: 1% per 64KB.
            updatePercent = (int)(upload.totalSize / 65536UL);
            if (updatePercent > 99) updatePercent = 99;
          }

          yield();
          return;
        }

        if (upload.status == UPLOAD_FILE_END) {
          if (updateStatus != 1) {
            Update.abort();
            return;
          }

          if (!Update.end(true)) {
            Serial.printf("UpdateViaFile error: Update.end failed: %s\n", Update.errorString());
            updateStatus = 3;
            return;
          }

          updatePercent = 100;
          updateStatus = 2;
          Serial.printf("UpdateViaFile success: total=%u bytes\n", (unsigned)upload.totalSize);
          return;
        }

        if (upload.status == UPLOAD_FILE_ABORTED) {
          Serial.println("UpdateViaFile aborted");
          Update.abort();
          updateStatus = 3;
          return;
        }
      });

  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
}

// ===================== Arduino setup/loop =====================
#include "driver/gpio.h"

static constexpr gpio_num_t PIN_HEARTBEAT = GPIO_NUM_13;
static hw_timer_t* hbTimer = nullptr;
static volatile bool hbLevel = false;

void IRAM_ATTR onHbTimer() {
  hbLevel = !hbLevel;
  gpio_set_level(PIN_HEARTBEAT, hbLevel);
}

void setupHeartbeat() {
  pinMode((int)PIN_HEARTBEAT, OUTPUT);
  gpio_set_level(PIN_HEARTBEAT, 0);

  hbTimer = timerBegin(0, 80, true);          // 80MHz/80 = 1MHz tick (1 us)
  timerAttachInterrupt(hbTimer, &onHbTimer, true);
  timerAlarmWrite(hbTimer, 1000000, true);     // toggle tiap 1000ms => 1Hz
  timerAlarmEnable(hbTimer);
}
void setup() {
  Serial.begin(115200);
  infrared_setup();
  readPreferences();
  setupHeartbeat();

  pinMode(LED_AC, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(RST_BTN, INPUT);
  pinMode(AC_POWER_LED, OUTPUT);
  digitalWrite(AC_POWER_LED, LOW);

  if (disable_brownout) {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  }

  delay(1000);
  setupSSID();

  // Web server (ESP32 WebServer.h)
  serverResponse();

  const char* kHeaderKeys[] = {"X-File-Size"};
  server.collectHeaders(kHeaderKeys, 1);

  server.begin();

  if (disable_brownout) {
    printLog("BOOTING COMPLETE (BROWNOUT DISABLED)\n");
  } else {
    printLog("BOOTING COMPLETE (BROWNOUT ENABLED)\n");
  }
}

void loop() {
  if (!getStatusSuccess){
    getStatus();
  }
  service_loop();

  if (wifiScan) {
    scanWifi();
  }

  // SETUP connect pipeline (same logic as ac.ino)
  if (trytoconnect) {
    stop_request = false;
    server_ok = false;
    user_ok = false;
    device_ok = false;
    mqtt_ok = false;

    if (wifiConnected) {
      Serial.println("Disconnecting from current WiFi...");
      WiFi.disconnect(true);
      delay(1000);
      wifiConnected = false;
    }

    if (ssid != "") {
      Serial.println("Trying to connect to WiFi...");
      WiFi.disconnect(true);
      connectWifi();
      Serial.println("WiFi connected: " + String(wifiConnected) );
    }

    if (wifiConnected) {
      getToken(tbserver, tbuser, tbpass);
      if (user_ok) {
        getStatus();
        getAccessToken(tbserver, deviceID);
        if (device_ok) {
          connectMQTT();
        }
      }
    }

    trytoconnect = false;
  }

  // RECONNECT WIFI (same as ac.ino)
  if (!trytoconnect && wifiConnected && WiFi.status() != WL_CONNECTED) {
    reconnectWifiBlocking("WiFi link down");
  }

  if (update_telemetry) {
    send_Telemetry(status, swing, temp);
  }

  if (runMQTT) {
    client.loop();
    if (!client.connected()) {
      connectMQTT();
      delay(1000);
    }
  }

  // Force WiFi reconnect if WiFi looks connected, but both MQTT + server connectivity are down.
  // This is rate-limited to avoid reconnect loops when credentials/server are wrong.
  static unsigned long lastCommsForcedReconnectMs = 0;
  const unsigned long kCommsForcedReconnectCooldownMs = 60000UL;
  const bool comms_down = runMQTT && (!client.connected()) && (!server_connected);

  if (!trytoconnect && wifiConnected && (WiFi.status() == WL_CONNECTED) && comms_down) {
    if (millis() - lastCommsForcedReconnectMs >= kCommsForcedReconnectCooldownMs) {
      lastCommsForcedReconnectMs = millis();
      WiFi.disconnect(true);
      delay(200);
      reconnectWifiBlocking("MQTT & server disconnected");
    }
  }

  if (mqtt_request && use_AC_LED) {
    led_confirm_process();
  }

  // IR revert timer
  static unsigned long revertIrStartTime = 0;
  if (revert_ir) {
    if (revertIrStartTime == 0) revertIrStartTime = millis();
    if (millis() - revertIrStartTime > 2000UL) {
      Serial.println("Reverting IR command...");
      reverse_ir();
      revert_ir = false;
      revertIrStartTime = 0;
      digitalWrite(LED_PIN, LOW);
    }
  } else {
    revertIrStartTime = 0;
  }

  // Firmware update
  if (updatefirmware) {
    startFirmwareUpdate(updateUrl, updateMd5);
    updatefirmware = false;
  }

  if (logQueue > 0) {
    send_log(log_msg);
  }
}
