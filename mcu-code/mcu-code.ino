#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
// SparkFun sensor libraries
#include "SparkFun_ENS160.h"
#include "SparkFunBME280.h"
#include "Adafruit_SHT31.h"
// DHT sensor library (Adafruit)
#include <DHT.h>
// ------------------- Wi-Fi Credentials ------------------- //
const char *ssid     = "";
const char *password = "";
// IPAddress for the server once DNS is resolved
IPAddress ip;
// We'll include the device MAC for the HTTP request¸¸
String macAddress;
// ------------------- ENS160, BME280, SHT31 Instances ------------------- //
SparkFun_ENS160 myENS;
BME280 myBME280;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
// ------------------- DHT22 (AM2302) on GPIO pin 2 ------------------- //
#define DHTPIN  2
#define DHTTYPE DHT22  // AM2302 is typically treated as DHT22
DHT dht(DHTPIN, DHTTYPE);
// Timer variables
unsigned long lastTime   = 0;
unsigned long timerDelay = 2000;
// ------------------------------------------------------------------
//  SETUP
// ------------------------------------------------------------------
void setup() {
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  Serial.begin(115200);
  delay(500);
  // Initialize I2C on pins 18 (SCL) and 19 (SDA)
  Wire.begin(18, 19);
  // ------------------- Connect to WiFi ------------------- //
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  // Optional: resolve a domain name to an IP
  if (WiFi.hostByName("opuntia.cc", ip)) {
    Serial.print("Resolved domain to IP: ");
    Serial.println(ip);
  } else {
    Serial.println("Failed to resolve domain name.");
    // You could fallback to a static IP if needed
  }
  // ------------------- Initialize ENS160 ------------------- //
  int count = 0;
  if (!myENS.begin()) {
    Serial.println("ENS160 did not begin.");
    while (count < 10) {
      count++;
      delay(100);
    }
  }
  myENS.setOperatingMode(SFE_ENS160_STANDARD);
  // ------------------- Initialize BME280 ------------------- //
  count = 0;
  if (!myBME280.beginI2C()) {
    Serial.println("BME280 did not respond.");
    while (count < 10) {
      count++;
      delay(100);
    }
  }
  // ------------------- Initialize SHT31 ------------------- //
  count = 0;
  if (!sht31.begin(0x44)) {
    Serial.println("SHT31 did not begin.");
    while (count < 10) {
      count++;
      delay(100);
    }
  }
  sht31.begin(0x44);
  // ------------------- Initialize DHT22 ------------------- //
  dht.begin(); // Start reading from the DHT sensor
  // Grab the ESP32 MAC address for labeling
  macAddress = WiFi.macAddress();
  Serial.print("MAC Address: ");
  Serial.println(macAddress);
}
// ------------------------------------------------------------------
//  HELPER FUNCTIONS FOR DHT22
// ------------------------------------------------------------------
float readDHT22Temperature() {
  float t = dht.readTemperature(); // Celsius by default
  if (isnan(t)) {
    Serial.println("DHT22: Failed to read temperature!");
    return NAN;
  }
  return t;
}
float readDHT22Humidity() {
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("DHT22: Failed to read humidity!");
    return NAN;
  }
  return h;
}
// ------------------------------------------------------------------
//  CREATE SENSOR DATA STRING FOR HTTP GET
// ------------------------------------------------------------------
String createSensorDataString(float dhtTemp,
                              float dhtHum,
                              float sht31Temp,
                              float sht31Hum,
                              int   aqi,
                              int   tvoc,
                              int   eco2,
                              float bme280Hum,
                              float bme280Press)
{
  String dataString = "";
  // DHT22
  dataString += "&am_temp=" + (isnan(dhtTemp) ? "" : String(dhtTemp));
  dataString += "&am_hum="  + (isnan(dhtHum)  ? "" : String(dhtHum));
  // SHT31
  dataString += "&soil_temp=" + (isnan(sht31Temp) ? "" : String(sht31Temp));
  dataString += "&soil_hum="  + (isnan(sht31Hum)  ? "" : String(sht31Hum));
  // ENS160
  dataString += "&aqi="   + (aqi  < 0 ? "" : String(aqi));
  dataString += "&tvoc="  + (tvoc < 0 ? "" : String(tvoc));
  dataString += "&eco2="  + (eco2 < 0 ? "" : String(eco2));
  // BME280
  dataString += "&hum="       + (isnan(bme280Hum)   ? "" : String(bme280Hum));
  dataString += "&pressure="  + (isnan(bme280Press) ? "" : String(bme280Press / 100.0));
  return dataString;
}
// ------------------------------------------------------------------
//  LOOP
// ------------------------------------------------------------------
void loop() {
  // Check if it's time to send data
  if ((millis() - lastTime) > timerDelay) {
    if (WiFi.status() == WL_CONNECTED) {
      WiFiClient client;
      HTTPClient http;
      // ---- Read DHT22 (Temperature & Humidity) ----
      float amTemp = readDHT22Temperature();
      float amHum  = readDHT22Humidity();
      Serial.print("DHT22 Temp (C): ");
      Serial.println(amTemp);
      Serial.print("DHT22 Hum (%): ");
      Serial.println(amHum);
      // ---- Read SHT31 (Temp & Humidity) ----
      float shtTemp = sht31.readTemperature();
      float shtHum  = sht31.readHumidity();
      Serial.print("SHT31 Temp (C): ");
      Serial.println(shtTemp);
      Serial.print("SHT31 Hum (%): ");
      Serial.println(shtHum);
      // ---- Read ENS160 (AQI, TVOC, eCO2) ----
      int aqi   = myENS.getAQI();
      int tvoc  = myENS.getTVOC();
      int eco2  = myENS.getECO2();
      Serial.print("ENS160 -> AQI: ");
      Serial.print(aqi);
      Serial.print("  TVOC: ");
      Serial.print(tvoc);
      Serial.print("  eCO2: ");
      Serial.println(eco2);
      // ---- Read BME280 (Humidity & Pressure) ----
      float bmeHum   = myBME280.readFloatHumidity();
      float bmePress = myBME280.readFloatPressure();
      Serial.print("BME280 Hum (%): ");
      Serial.println(bmeHum);
      Serial.print("BME280 Pressure (Pa): ");
      Serial.println(bmePress);
      // ---- Build the sensor data string for the HTTP GET ----
      String sensorData = createSensorDataString(
        amTemp,      // from DHT22
        amHum,       // from DHT22
        shtTemp,     // from SHT31
        shtHum,      // from SHT31
        aqi,         // from ENS160
        tvoc,        // from ENS160
        eco2,        // from ENS160
        bmeHum,      // from BME280
        bmePress     // from BME280
      );
      // Debug print the data
      Serial.println("Sensor data: " + sensorData);
      // ---- Construct the full server path for HTTP GET ----
      // Example: http://<ip>:8089/sendData?mac=<mac>&am_temp=...&am_hum=...
      String serverPath = "http://" + ip.toString() + ":8089/sendData?mac="
                          + String(macAddress)
                          + sensorData;
      Serial.println("Server path: " + serverPath);
      // ---- Send HTTP GET request ----
      http.begin(client, serverPath.c_str());
      int httpResponseCode = http.GET();
      // Handle server response
      if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println("Server response: " + payload);
      } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();  // Update last time data was sent
    Serial.println("------------------------------------");
  }
}