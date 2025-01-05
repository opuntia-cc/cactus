#include <Wire.h>
#include "SparkFun_ENS160.h"
#include "SparkFunBME280.h"
#include "Adafruit_SHT31.h"


String macAddress;

SparkFun_ENS160 myENS;
BME280 myBME280;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

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
}



void loop() {
  // Check if it's time to send data
  if ((millis() - lastTime) > timerDelay) {
    Serial.println("SHT31 temp/moisture: " + String(sht31.readTemperature()) + String(sht31.readHumidity()));
    Serial.println("ENS AQI/TVOC/ECO2: " +  String(myENS.getAQI()) + String(myENS.getTVOC()) + String(myENS.getECO2()));
    Serial.println("BME Humidity/Pressure: " + String(myENS.getAQI()) + String(myENS.getTVOC()) + String(myENS.getECO2()));

    lastTime = millis();  // Update last time data was sent
  }
}
