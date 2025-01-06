#include <Wire.h>
#include "SparkFun_ENS160.h"
#include "SparkFunBME280.h"
#include "Adafruit_SHT31.h"
#include <OneWire.h>
#include <DallasTemperature.h>


String macAddress;

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
  Wire.begin(18, 19);
  Serial.begin(115200);

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

}



void loop() {
  // Check if it's time to send data
  if ((millis() - lastTime) > timerDelay) {
    sensors.requestTemperatures(); 
    printTemperature(insideThermometer);
    Serial.println("SHT31 temp/moisture: " + String(sht31.readTemperature()) + ", " + String(sht31.readHumidity()));
    Serial.println("ENS AQI/TVOC/ECO2: " +  String(myENS.getAQI()) + ", " + String(myENS.getTVOC()) + ", " + String(myENS.getECO2()));
    Serial.println("BME Humidity/Pressure: " + String(myBME280.readFloatHumidity()) + ", " + String(myBME280.readFloatPressure()));

    lastTime = millis();  // Update last time data was sent
    Serial.println();
  }
}


void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

