#include <Wire.h>
#include <Adafruit_INA219.h>

// Custom I2C pins
static const int I2C_SDA = 14;
static const int I2C_SCL = 13;

// Create INA219 instances with fixed addresses
Adafruit_INA219 ina1(0x40); // Default address (neither A0 nor A1 bridged)
Adafruit_INA219 ina2(0x41); // A0 bridged to VS+

bool ina1_status = false;
bool ina2_status = false;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to connect
  
  // Initialize I2C with custom SDA and SCL pins
  Wire.begin(I2C_SDA, I2C_SCL);
  
  Serial.println("\n--- INA219 Dual Initialization ---");
  Serial.printf("I2C Pins -> SDA: %d, SCL: %d\n", I2C_SDA, I2C_SCL);

  // Initialize INA219 at 0x40
  if (!ina1.begin()) {
    Serial.println("[-] Failed to find INA219 at address 0x40");
    ina1_status = false;
  } else {
    Serial.println("[+] INA219 at address 0x40 initialized successfully!");
    ina1_status = true;
  }

  // Initialize INA219 at 0x41
  if (!ina2.begin()) {
    Serial.println("[-] Failed to find INA219 at address 0x41");
    ina2_status = false;
  } else {
    Serial.println("[+] INA219 at address 0x41 initialized successfully!");
    ina2_status = true;
  }
}

void loop() {
  Serial.println("\n--- INA219 Readings ---");

  // Read INA219 at 0x40
  if (ina1_status) {
    float busvoltage1 = ina1.getBusVoltage_V();
    float current_mA1 = ina1.getCurrent_mA();
    Serial.printf("[0x40] Bus Voltage: %.3f V | Current: %.2f mA\n", busvoltage1, current_mA1);
  } else {
    Serial.println("[0x40] Sensor Offline");
  }

  // Read INA219 at 0x41
  if (ina2_status) {
    float busvoltage2 = ina2.getBusVoltage_V();
    float current_mA2 = ina2.getCurrent_mA();
    Serial.printf("[0x41] Bus Voltage: %.3f V | Current: %.2f mA\n", busvoltage2, current_mA2);
  } else {
    Serial.println("[0x41] Sensor Offline");
  }

  delay(2000); // Read every 2 seconds
}
