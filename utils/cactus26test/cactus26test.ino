#include <Wire.h>
// sensor libraries
#include "SparkFun_ENS160.h"
#include "SparkFunBME280.h"
#include "Adafruit_SHT31.h"



// ------------------- ENS160, BME280, SHT31 Instances ------------------- //
SparkFun_ENS160 myENS;
BME280 myBME280;
Adafruit_SHT31 sht31 = Adafruit_SHT31();




// ------------------------------------------------------------------
//  SETUP
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);



  // Initialize I2C on pins 41 (SCL) and 40 (SDA)
  Wire.begin(41, 40);



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


}



// ------------------------------------------------------------------
//  LOOP
// ------------------------------------------------------------------
void loop() {
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

  Serial.println("------------------------------------");
  delay(1000); // Print every second
}
