#include <esp_task_wdt.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>

// sensor libraries
#include "SparkFun_ENS160.h"
#include "SparkFunBME280.h"
#include "Adafruit_SHT31.h"

// ------------------- Wi-Fi Credentials ------------------- //
const char *ssid     = "mesquite";
const char *password = "12345678";

// IPAddress for the server once DNS is resolved
IPAddress ip;

// We'll include the device MAC for the HTTP request
String macAddress;

// ------------------- ENS160, BME280, SHT31 Instances ------------------- //
SparkFun_ENS160 myENS;
BME280 myBME280;
Adafruit_SHT31 sht31 = Adafruit_SHT31();


// Deep sleep duration (in microseconds)
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  60          // Time ESP32 will go to sleep (in seconds)

// ------------------------------------------------------------------
//  SETUP
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize and enable watchdog timer (timeout: 30 seconds)
  esp_task_wdt_init(30, true); // 30s timeout, panic on trigger
  esp_task_wdt_add(NULL);      // Add current thread to WDT

  // Initialize I2C on pins 41 (SCL) and 40 (SDA)
  Wire.begin(41, 40);

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

  // Grab the ESP32 MAC address for labeling
  macAddress = WiFi.macAddress();
  Serial.print("MAC Address: ");
  Serial.println(macAddress);
}

// ------------------------------------------------------------------
//  CREATE SENSOR DATA STRING FOR HTTP GET
//  (No DHT; BME280 provides temp + humidity + pressure)
// ------------------------------------------------------------------
String createSensorDataString(float sht31Temp,
                              float sht31Hum,
                              int   aqi,
                              int   tvoc,
                              int   eco2,
                              float bmeTemp,
                              float bmeHum,
                              float bmePress)
{
  String dataString = "";

  // SHT31 (keeping these as "soil_*" like your original)
  dataString += "&soil_temp=" + (isnan(sht31Temp) ? "" : String(sht31Temp));
  dataString += "&soil_hum="  + (isnan(sht31Hum)  ? "" : String(sht31Hum));

  // ENS160
  dataString += "&aqi="  + (aqi  < 0 ? "" : String(aqi));
  dataString += "&tvoc=" + (tvoc < 0 ? "" : String(tvoc));
  dataString += "&eco2=" + (eco2 < 0 ? "" : String(eco2));

  // BME280 (NOW includes temperature + humidity + pressure)
  dataString += "&temp="      + (isnan(bmeTemp)    ? "" : String(bmeTemp));
  dataString += "&hum="       + (isnan(bmeHum)     ? "" : String(bmeHum));
  dataString += "&pressure="  + (isnan(bmePress)   ? "" : String(bmePress / 100.0)); // Pa -> hPa

  return dataString;
}

// ------------------------------------------------------------------
//  LOOP
// ------------------------------------------------------------------
void loop() {
  // Feed the watchdog at the start of loop
  esp_task_wdt_reset();
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;

    // ---- Read SHT31 (Temp & Humidity) ----
    float shtTemp = sht31.readTemperature();
    float shtHum  = sht31.readHumidity();
    Serial.print("SHT31 Temp (C): ");
    Serial.println(shtTemp);
    Serial.print("SHT31 Hum (%): ");
    Serial.println(shtHum);

    esp_task_wdt_reset(); // Feed watchdog after SHT31

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

    esp_task_wdt_reset(); // Feed watchdog after ENS160

    // ---- Read BME280 (Temperature, Humidity & Pressure) ----
    float bmeTemp  = myBME280.readTempC();
    float bmeHum   = myBME280.readFloatHumidity();
    float bmePress = myBME280.readFloatPressure();
    Serial.print("BME280 Temp (C): ");
    Serial.println(bmeTemp);
    Serial.print("BME280 Hum (%): ");
    Serial.println(bmeHum);
    Serial.print("BME280 Pressure (Pa): ");
    Serial.println(bmePress);

    esp_task_wdt_reset(); // Feed watchdog after BME280

    // ---- Build the sensor data string for the HTTP GET ----
    String sensorData = createSensorDataString(
      shtTemp,
      shtHum,
      aqi,
      tvoc,
      eco2,
      bmeTemp,
      bmeHum,
      bmePress
    );

    Serial.println("Sensor data: " + sensorData);

    // ---- Construct the full server path for HTTP GET ----
    String serverPath = "http://" + ip.toString() + ":8089/sendData?mac="
                        + String(macAddress)
                        + sensorData;

    Serial.println("Server path: " + serverPath);

    // ---- Send HTTP GET request ----
    http.begin(client, serverPath.c_str());
    int httpResponseCode = http.GET();

    esp_task_wdt_reset(); // Feed watchdog after HTTP GET

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
  } else {
    Serial.println("WiFi Disconnected");
  }

  Serial.println("------------------------------------");

  // Deep sleep for TIME_TO_SLEEP seconds
  Serial.println("Going to deep sleep for 60 seconds...");
  esp_task_wdt_delete(NULL); // Remove WDT before deep sleep
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.flush();
  esp_deep_sleep_start();
}
