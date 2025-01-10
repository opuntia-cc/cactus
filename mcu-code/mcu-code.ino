#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "SparkFun_ENS160.h"
#include "SparkFunBME280.h"
#include "Adafruit_SHT31.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// Wi-Fi credentials
const char *ssid = "mesquite";
const char *password = "movement";

String macAddress;
IPAddress ip;

SparkFun_ENS160 myENS;
BME280 myBME280;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

unsigned long lastTime = 0;
unsigned long timerDelay = 2000;

void setup() {
  Serial.begin(115200);

  Wire.begin(18, 19);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected to WiFi");


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

  myENS.setOperatingMode(SFE_ENS160_STANDARD);
  sht31.begin(0x44);

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

String createSensorDataString(float temp,
                              float sht31Temp, float sht31Hum,
                              int aqi, int tvoc, int eco2,
                              float bme280Hum, float bme280Press) {
  String dataString = "";

  // Light Sensor (Old Device)
  dataString += "&temp=" + (isnan(temp) ? "" : String(temp));

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

      sensors.requestTemperatures(); 

      String sensorData = createSensorDataString(printTemperature(insideThermometer),                                    // Old device sensors
                                                 sht31.readTemperature(), sht31.readHumidity(),     // Common sensor
                                                 myENS.getAQI(), myENS.getTVOC(), myENS.getECO2(),  // New device sensors
                                                 myBME280.readFloatHumidity(), myBME280.readFloatPressure());
      Serial.println(sensorData);

      String serverPath = "http://" + ip.toString() + ":8089/sendData?mac=" + String(macAddress) + sensorData;
      Serial.println(serverPath);

      http.begin(client, serverPath.c_str());

      // Send HTTP GET request
      int httpResponseCode = http.GET();

      // Handle server response
      if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }

      // Free resources
      http.end();
    } else {
      Serial.println("WiFi Disconnected");
    }

    lastTime = millis();  // Update last time data was sent
  }
}


float printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return -1;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
  return tempC;
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
