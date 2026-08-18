#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>

// SparkFun sensor libraries
#include "SparkFun_ENS160.h"
#include "SparkFunBME280.h"
#include "Adafruit_SHT31.h"

// ------------------- Wi-Fi Credentials ------------------- //
const char *ssid     = "ATT-WIFI-no5J";
const char *password = "Wn7oUJj5";

// IPAddress for the server once DNS is resolved
IPAddress ip;

// We'll include the device MAC for the HTTP request
String macAddress;

// ------------------- ENS160, BME280, SHT31 Instances ------------------- //
SparkFun_ENS160 myENS;
BME280 myBME280;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// ------------------- Power / Timing ------------------- //
// If you are NOT using GPIO3 as a sensor power-enable pin, set this to -1.
const int SENSOR_EN_PIN = 3;

// Watchdog every 2 min
static const uint32_t WDT_TIMEOUT_SEC = 120;

// Deep sleep every 2 min
static const uint64_t DEEP_SLEEP_US = 120ULL * 1000000ULL;

// Wi-Fi association timeout. A phone hotspot is a single AP, physically close,
// with no enterprise auth and no captive portal, so association is normally
// 1-3 s. 8 s leaves headroom for a slow first boot without burning battery on
// a doomed cycle. The cached-AP fast path gets a third of this (~2.7 s) before
// falling back to a full scan.
static const uint32_t WIFI_TIMEOUT_MS = 8000;

// Bench debugging: wait for the USB serial monitor to reattach after deep sleep
// before printing. Set to 0 for field deployment.
#define WAIT_FOR_SERIAL 0
static const uint32_t SERIAL_WAIT_MS = 2500;

// ------------------- Wi-Fi fast-reconnect cache ------------------- //
// RTC memory survives deep sleep. Storing the AP's channel and BSSID lets the
// next wake skip the full channel scan and go straight to the known AP.
RTC_DATA_ATTR uint8_t rtcBssid[6] = {0};
RTC_DATA_ATTR int32_t rtcChannel  = 0;
RTC_DATA_ATTR bool    rtcHasAp    = false;

// ------------------------------------------------------------------
//  WATCHDOG
// ------------------------------------------------------------------
void initWatchdog() {
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_task_wdt_config_t wdt_config = {};
  wdt_config.timeout_ms = WDT_TIMEOUT_SEC * 1000;
  wdt_config.idle_core_mask = (1 << portNUM_PROCESSORS) - 1;
  wdt_config.trigger_panic = true;
  esp_task_wdt_init(&wdt_config);
#else
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
#endif
  esp_task_wdt_add(NULL); // current task
}

inline void feedWatchdog() {
  esp_task_wdt_reset();
}

// ------------------------------------------------------------------
//  HELPERS
// ------------------------------------------------------------------
// Poll every 50 ms instead of 500 ms so we notice the association the moment
// it completes instead of sitting in a delay for up to half a second after.
static bool waitForConnect(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(50);
    feedWatchdog();
    if (((millis() - start) % 500) < 50) Serial.print(".");
  }
  Serial.println();
  return (WiFi.status() == WL_CONNECTED);
}

bool connectWiFi(uint32_t timeoutMs = WIFI_TIMEOUT_MS) {
  WiFi.persistent(false);   // skip writing credentials to NVS every wake
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);     // keep the radio awake during this short burst

  uint32_t t0 = millis();
  bool connected = false;

  // Fast path: go straight to the AP we used last cycle.
  if (rtcHasAp && rtcChannel > 0) {
    Serial.print("Connecting to WiFi (cached AP, ch ");
    Serial.print(rtcChannel);
    Serial.print(")");
    WiFi.begin(ssid, password, rtcChannel, rtcBssid);
    connected = waitForConnect(timeoutMs / 3);

    if (!connected) {
      Serial.println("Cached AP failed, falling back to full scan.");
      rtcHasAp = false;
      WiFi.disconnect(true, true);
      delay(100);
    }
  }

  // Normal path: first boot, or the cached AP did not answer.
  if (!connected) {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    connected = waitForConnect(timeoutMs);
  }

  if (connected) {
    Serial.print("Connected to WiFi in ");
    Serial.print(millis() - t0);
    Serial.println(" ms");
    Serial.print("IP: ");
    Serial.print(WiFi.localIP());
    Serial.print("   RSSI: ");
    Serial.println(WiFi.RSSI());

    // Remember this AP for the next wake.
    const uint8_t *bssid = WiFi.BSSID();
    if (bssid != NULL) {
      memcpy(rtcBssid, bssid, 6);
      rtcChannel = WiFi.channel();
      rtcHasAp   = true;
    }
    return true;
  }

  Serial.println("WiFi connect timeout");
  return false;
}

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

  // SHT31
  dataString += "&soil_temp=" + (isnan(sht31Temp) ? "" : String(sht31Temp));
  dataString += "&soil_hum="  + (isnan(sht31Hum)  ? "" : String(sht31Hum));

  // ENS160
  dataString += "&aqi="  + (aqi  < 0 ? "" : String(aqi));
  dataString += "&tvoc=" + (tvoc < 0 ? "" : String(tvoc));
  dataString += "&eco2=" + (eco2 < 0 ? "" : String(eco2));

  // BME280
  dataString += "&temp="      + (isnan(bmeTemp)  ? "" : String(bmeTemp));
  dataString += "&hum="       + (isnan(bmeHum)   ? "" : String(bmeHum));
  dataString += "&pressure="  + (isnan(bmePress) ? "" : String(bmePress / 100.0)); // Pa -> hPa

  return dataString;
}

void goToDeepSleep() {
  Serial.println("Entering deep sleep for 120 seconds...");
  Serial.flush();

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_US);
  esp_deep_sleep_start();
}

// ------------------------------------------------------------------
//  SETUP (run once each wake)
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

#if WAIT_FOR_SERIAL
  // This board uses native USB CDC for Serial. Deep sleep drops the USB device,
  // and the host needs a moment to re-enumerate on wake. Without this wait, any
  // print that happens in the first second or two is sent into a closed port and
  // lost, which is why early lines appear to be missing from the monitor.
  // Set WAIT_FOR_SERIAL to 0 before deploying, or every wake burns the full
  // timeout waiting for a host that is not there.
  {
    uint32_t serialStart = millis();
    while (!Serial && (millis() - serialStart) < SERIAL_WAIT_MS) {
      delay(10);
    }
    delay(200); // let the monitor finish attaching
  }
#else
  delay(500);
#endif

  initWatchdog();
  feedWatchdog();

  // ---------- MAC address ----------
  // Bringing up the station interface is enough to read the MAC; no need to
  // be associated. This way the MAC always prints, even if Wi-Fi later fails.
  WiFi.mode(WIFI_STA);
  macAddress = WiFi.macAddress();
  Serial.print("MAC Address: ");
  Serial.println(macAddress);

  // Optional sensor power-enable pin
  if (SENSOR_EN_PIN >= 0) {
    pinMode(SENSOR_EN_PIN, OUTPUT);
    digitalWrite(SENSOR_EN_PIN, HIGH);
    delay(2000); // let sensors power up
    feedWatchdog();
  }

  // I2C
  Wire.begin(41, 40);

  // WiFi
  bool wifiOK = connectWiFi();

  // Resolve domain (only if WiFi connected)
  if (wifiOK) {
    if (WiFi.hostByName("opuntia.cc", ip)) {
      Serial.print("Resolved domain to IP: ");
      Serial.println(ip);
    } else {
      Serial.println("Failed to resolve domain name.");
      wifiOK = false;
    }
  }

  // Sensor init
  bool ensOK = myENS.begin();
  if (!ensOK) Serial.println("ENS160 did not begin.");
  else myENS.setOperatingMode(SFE_ENS160_STANDARD);

  bool bmeOK = myBME280.beginI2C();
  if (!bmeOK) Serial.println("BME280 did not respond.");

  bool shtOK = sht31.begin(0x44);
  if (!shtOK) Serial.println("SHT31 did not begin.");

  // Extra warm-up before reading (important after wake/power-up)
  delay(1500);
  feedWatchdog();

  // Read sensors (fallback-safe)
  float shtTemp = NAN, shtHum = NAN;
  int aqi = -1, tvoc = -1, eco2 = -1;
  float bmeTemp = NAN, bmeHum = NAN, bmePress = NAN;

  if (shtOK) {
    shtTemp = sht31.readTemperature();
    shtHum  = sht31.readHumidity();
    Serial.print("SHT31 Temp (C): "); Serial.println(shtTemp);
    Serial.print("SHT31 Hum (%): ");  Serial.println(shtHum);
  }

  if (ensOK) {
    aqi  = myENS.getAQI();
    tvoc = myENS.getTVOC();
    eco2 = myENS.getECO2();
    Serial.print("ENS160 -> AQI: "); Serial.print(aqi);
    Serial.print("  TVOC: ");        Serial.print(tvoc);
    Serial.print("  eCO2: ");        Serial.println(eco2);
  }

  if (bmeOK) {
    bmeTemp  = myBME280.readTempC();
    bmeHum   = myBME280.readFloatHumidity();
    bmePress = myBME280.readFloatPressure();
    Serial.print("BME280 Temp (C): ");    Serial.println(bmeTemp);
    Serial.print("BME280 Hum (%): ");     Serial.println(bmeHum);
    Serial.print("BME280 Pressure (Pa): "); Serial.println(bmePress);
  }

  feedWatchdog();

  // Send once per wake
  if (wifiOK) {
    String sensorData = createSensorDataString(
      shtTemp, shtHum, aqi, tvoc, eco2, bmeTemp, bmeHum, bmePress
    );

    Serial.println("Sensor data: " + sensorData);

    String serverPath = "http://" + ip.toString() + ":8089/sendData?mac="
                        + String(macAddress) + sensorData;

    Serial.println("Server path: " + serverPath);

    WiFiClient client;
    HTTPClient http;
    http.setConnectTimeout(10000);
    http.setTimeout(10000);
    http.begin(client, serverPath.c_str());

    feedWatchdog();
    int httpResponseCode = http.GET();
    feedWatchdog();

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
    Serial.println("Skipping send (WiFi/domain not ready).");
  }

  Serial.println("------------------------------------");

  // Sleep for 2 minutes
  goToDeepSleep();
}

// ------------------------------------------------------------------
//  LOOP (not used; device sleeps from setup each cycle)
// ------------------------------------------------------------------
void loop() {
  // Not used
}
