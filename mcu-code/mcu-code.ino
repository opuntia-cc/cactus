#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "SparkFun_ENS160.h"
#include "SparkFunBME280.h"
#include "Adafruit_SHT31.h"
#include <DHT.h>


// Wi-Fi credentials
const char *ssid = "mesquiteMocap";
const char *password = "movement";

String macAddress;
IPAddress ip;

SparkFun_ENS160 myENS;
BME280 myBME280;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a DHT22(AM2302) on data pin 2
#define DHT22PIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//Time variables

unsigned long lastTime = 0;
unsigned long timerDelay = 2000;

void setup() {
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  Serial.begin(115200);
  delay(500);

  Wire.begin(18, 19);
  WiFi.begin(ssid, password);
  Serial.print("Connected to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi");


  if (WiFi.hostByName("opuntia.cc", ip)) {
    Serial.println(ip);
  } else {
    Serial.println("Failed to resolve domain name.");
  }

  // check if the ENS sensor is connected
  int count = 0;
  if (!myENS.begin()) {
    Serial.println("ENS160 did not begin.");
    while (count < 10)
      count++;
  }
  // check if the BME280 sensor is connected
  count = 0;
  if (!myBME280.beginI2C()) {
    Serial.println("BME280 did not respond.");
    while (count < 10)
      count++;
  }
  // check if the SHT31 sensor is connected
  count = 0;
  if (!sht31.begin(0x44)) {
    Serial.println("SHT31 did not begin.");
    while (count < 10)
      count++;
  }

  //cehck if the DHT22 sensor is connected
  dht.bbgin();
  macaddress = WiFi.macAddress();
  serial.print("Mac Address: ");
  serial.ptintln(mackAddress);
  myENS.setOperatingMode(SFE_ENS160_STANDARD);
  sht31.begin(0x44);


// Hlepfer function for DHT22
float readDHT22Temperature() {
  float t = dht.readTemperature();
  if (isnan(t)){
    Serial.println("Failed to read from DHT sensor!");
    return -1;
  }
  return t;
}
float readDHT22Humidity() {
  float h = dht.readHumidity();
  if (isnan(h)){
    Serial.println("Failed to read from DHT sensor!");
    return -1;
  }
  return h;
}
  Serial.print("Locating DS devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 

    Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();

  macAddress = WiFi.macAddress();  // Get MAC address

  // print macAddress
  Serial.println(macAddress);
}

String createSensorDataString(float dhtTemp,
                              float dhtHum,
                              float sht31Temp,
                              float sht31Hum,
                              int aqi,
                              int tvoc,
                              int eco2,
                              float bme280Hum,
                              float bme280Press) {
  String dataString = "";

  // DHT22 Sensor 
  dataString += "&DHT22_temp=" + (isnan(dhtTemp) ? "" : String(dhtTemp));
  dataString += "&DHT22_hum=" + (isnan(dhtHum) ? "" : String(dhtHum));

  // SHT31 Sensor (Common)
  dataString += "&soil_temp=" + (isnan(sht31Temp) ? "" : String(sht31Temp));
  dataString += "&soil_hum=" + (isnan(sht31Hum) ? "" : String(sht31Hum));

  // ENS160 Sensor (New Device)
  dataString += "&aqi=" + (aqi < 0 ? "" : String(aqi));
  dataString += "&tvoc=" + (tvoc < 0 ? "" : String(tvoc));
  dataString += "&eco2=" + (eco2 < 0 ? "" : String(eco2));

  // BME280 Sensor (New Device)
  dataString += "&hum=" + (isnan(bme280Hum) ? "" : String(bme280Hum));
  dataString += "&pressure=" + (isnan(bme280Press) ? "" : String(bme280Press / 100.0));  // Convert to hPa

  return dataString;
}

void loop() {
  // Check if it's time to send data
  if ((millis() - lastTime) > timerDelay) {
    if (WiFi.status() == WL_CONNECTED) {
      WiFiClient client;
      HTTPClient http;
      // ---- Read DHT22 (Temperature & Humidity) ----
      float DHT22_Temp = readDHT22Temperature();
      float DHT22_Hum  = readDHT22Humidity();
      Serial.print("DHT22 Temp (C): ");
      Serial.println(DHT22_Temp);
      Serial.print("DHT22 Hum (%): ");
      Serial.println(DHT22_Hum);
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
        DHT22_Temp,      // from DHT22
        DHT22_Hum,       // from DHT22
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