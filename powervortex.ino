#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Arduino.h>

// Wi-Fi credentials
const char* ssid = "NORD";
const char* password = "zxcvbnml";

#define RELAY_PIN 15           // Pin connected to the relay module
#define VOLTAGE_SENSOR_PIN 4 // Pin connected to the ZMPT101B voltage sensor
#define VOLTAGE_THRESHOLD 243 // Threshold voltage in Volts

// Firebase configuration
FirebaseConfig firebaseConfig;
FirebaseAuth firebaseAuth;

// Initialize Firebase Data object for each device
FirebaseData firebaseData1;
FirebaseData firebaseData2;

// Pin for controlling the bulbs
const int RELAY_PIN_1 = 26;
const int RELAY_PIN_2 = 27;

// ACS712 parameters for both devices
const int ACS712_PIN_1 = 34;
const int ACS712_PIN_2 = 35;
const float ACS712_ZERO_CURRENT = 2500.0;  // ADC value for zero current
const float ACS712_SCALE = 66.0;  // Sensitivity for ACS712-30A module

// Power consumption variables for both devices
float current1, current2;
float powerConsumed1, powerConsumed2;
float energyConsumed1, energyConsumed2;
unsigned long startTime1, startTime2;  // Start time of the current 10-second interval for each device

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  firebaseConfig.database_url = "https://powervortex-1-default-rtdb.asia-southeast1.firebasedatabase.app";
  firebaseConfig.signer.tokens.legacy_token = "O3PzcEbfxd5nLfMxk6hgBj7uia0u8koTehMnUuUp";
  
  Firebase.begin(&firebaseConfig, &firebaseAuth);

  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);

  // Initialize variables
  current1 = current2 = 0.0;
  powerConsumed1 = powerConsumed2 = 0.0;
  energyConsumed1 = energyConsumed2 = 0.0;
  startTime1 = startTime2 = millis();
  
  // Fetch initial energy consumed values from Firebase for both devices
  fetchAndUpdateEnergyConsumed(firebaseData1, "/boards/abcabcab1/1/consumption", energyConsumed1);
  fetchAndUpdateEnergyConsumed(firebaseData2, "/boards/abcabcab1/2/consumption", energyConsumed2);
}

void fetchAndUpdateEnergyConsumed(FirebaseData& firebaseData, const String& path, float& energyConsumed) {
  if (Firebase.getFloat(firebaseData, path)) {
      energyConsumed = firebaseData.floatData();
      Serial.print("Fetched Energy Consumed for ");
      Serial.print(path);
      Serial.print(": ");
      Serial.println(energyConsumed);
  } else {
    Serial.print("Firebase getFloat failed: ");
    Serial.println(firebaseData.errorReason());
  }
}

void loop() {

  float voltage = readVoltage();
  Serial.print("Voltage: ");
  Serial.println(voltage);
  
  if (voltage < VOLTAGE_THRESHOLD) {
    digitalWrite(RELAY_PIN, HIGH); // Turn on relay
  } else {
    digitalWrite(RELAY_PIN, HIGH); // Turn off relay
  }

  manageDevice(firebaseData1, RELAY_PIN_1, ACS712_PIN_1, "/boards/abcabcab1/1/status", "/boards/abcabcab1/1/consumption", current1, powerConsumed1, energyConsumed1, startTime1);
  manageDevice(firebaseData2, RELAY_PIN_2, ACS712_PIN_2, "/boards/abcabcab1/2/status", "/boards/abcabcab1/2/consumption", current2, powerConsumed2, energyConsumed2, startTime2);

  delay(1000); // To prevent excessive Firebase requests
}

void manageDevice(FirebaseData& firebaseData, int relayPin, int acs712Pin, const String& statusPath, const String& consumptionPath, float& current, float& powerConsumed, float& energyConsumed, unsigned long& startTime) {
  unsigned long currentMillis = millis();

  if (Firebase.ready() && Firebase.getBool(firebaseData, statusPath)) {
    bool deviceStatus = firebaseData.boolData();
    digitalWrite(relayPin, deviceStatus ? LOW : HIGH);
    Serial.println(deviceStatus ? "Device ON" : "Device OFF");

    if (deviceStatus && currentMillis - startTime >= 10000) {
      startTime = currentMillis;

      int sensorValue = analogRead(acs712Pin);
      // Corrected to use the known ADC max value for ESP32
      current = ((sensorValue - ACS712_ZERO_CURRENT) * 5.0) / (4095.0 * ACS712_SCALE);

      powerConsumed = abs(current) * 230.0; // Assuming 230V AC mains voltage
      float intervalEnergy = (powerConsumed / (3600.0 * 1000.0)) * 10.0; // Energy in kWh for 10 seconds
      energyConsumed += intervalEnergy;

      Serial.print("Power consumed (10s interval): ");
      Serial.print(powerConsumed);
      Serial.println(" Watts");
      Serial.print("Energy consumed: ");
      Serial.print(energyConsumed);
      Serial.println(" kWh");

      if (!Firebase.setFloat(firebaseData, consumptionPath, energyConsumed)) {
        Serial.println("Failed to update energy consumption in Firebase");
        Serial.println(firebaseData.errorReason());
      }
    }
  } else if (!Firebase.ready()) {
    Serial.println("Firebase not ready, attempting to reconnect...");
    delay(5000); // Delay more before retrying
  }
}

float readVoltage() {
  // Read voltage from ZMPT101B sensor
  int sensorValue = analogRead(VOLTAGE_SENSOR_PIN);
  // Convert analog value to voltage (assuming a 3.3V reference voltage and 12-bit ADC)
  float voltage = sensorValue * (3.3 / 4095);
  return voltage;
}
