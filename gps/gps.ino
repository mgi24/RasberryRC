#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// GPS module wired to Serial1: TX(module) -> RX pin 9, RX(module) -> TX pin 8
static const int GPS_RX = 9;
static const int GPS_TX = 8;
static const uint32_t GPS_BAUD = 9600;

// Timeout: if TinyGPS++ receives chars but no valid fix for this long,
// fall back to printing raw NMEA data.
static const uint32_t FIX_TIMEOUT_MS = 10000;

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

static bool fallbackMode = false;
static uint32_t firstCharTime = 0;
static bool gotFirstChar = false;

void setup() {
  Serial.begin(115200); // USB CDC
  while (!Serial) delay(10);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("GPS reader started — TX=8, RX=9 @ 9600 baud");
  Serial.println("Waiting for GPS data...");
}

void loop() {
  while (gpsSerial.available() > 0) {
    char c = (char)gpsSerial.read();

    // Track when we first see any character
    if (!gotFirstChar) {
      gotFirstChar = true;
      firstCharTime = millis();
    }

    gps.encode(c);

    // In fallback mode: just dump raw bytes to USB serial
    if (fallbackMode) {
      Serial.write(c);
    }
  }

  // Check if we should switch to fallback (raw) mode
  if (!fallbackMode && gotFirstChar &&
      (millis() - firstCharTime) > FIX_TIMEOUT_MS &&
      gps.location.age() > FIX_TIMEOUT_MS) {
    fallbackMode = true;
    Serial.println();
    Serial.println("=== No valid GPS fix — switching to RAW NMEA output ===");
  }

  // Normal mode: print fix when updated
  if (!fallbackMode && gps.location.isUpdated()) {
    Serial.printf("Lat: %.6f  Lon: %.6f  Sats: %u  HDOP: %.2f  Age: %lums\n",
                  gps.location.lat(),
                  gps.location.lng(),
                  gps.satellites.value(),
                  gps.hdop.hdop(),
                  gps.location.age());
  }
}
