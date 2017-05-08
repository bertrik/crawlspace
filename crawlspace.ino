/**
 * ESP8266 application for monitoring temperature, humidity and water level in the crawl space under a house.
 *
 * See also https://revspace.nl/CrawlSpaceSensor
 *
 * Dependencies:
 * - WifiManager
 * - PubSubClient
 * - pololu VL53L0X library
 * - NTPClient
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <NTPClient.h>
#include "dht11.h"

// measurement period (seconds)
#define PERIOD          60
#define PIN_SHUT        D3
#define PIN_DHT_GND     D5
#define PIN_DHT_DAT     D6
#define PIN_DHT_VCC     D7

#define MQTT_HOST       "aliensdetected.com"
#define MQTT_PORT       1883
#define MQTT_TOPIC      "crawlspace"

#define NTP_HOST        "ntp.vanheusden.com"
#define NTP_PORT        123
#define NTP_LOCALPORT   1234
#define NTP_TIMEOUT     3000

// measurement data
typedef struct {
    char id[8];
    uint32_t time;
    int humidity;
    int temperature;
    uint16_t range;
} meas_t;

// prototype of step_XXX functions
typedef bool (step_fn_t)(meas_t *meas);

typedef struct {
    const char *desc;
    step_fn_t *fn;
} step_t;

static step_t *step;
static meas_t measurement;
static WiFiManager wifiManager;
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static VL53L0X lidar;
static WiFiUDP udp;
static NTPClient ntpClient(udp, NTP_HOST);
static DHT11 dht11(PIN_DHT_DAT);

static void print(const char *fmt, ...)
{
    char line[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    Serial.print(line); 
}

static bool step_connect_wifi(meas_t *meas)
{
    sprintf(meas->id, "%06X", ESP.getChipId());
    wifiManager.setDebugOutput(false);
    bool res = wifiManager.autoConnect("ESP-crawlspace");
    if (res) {
        print("connected");
    }    
    return res;
}

static bool step_request_sntp(meas_t *meas)
{
    bool result;

    ntpClient.begin();
    result = ntpClient.forceUpdate();
    if (result) {
        meas->time = ntpClient.getEpochTime();
        print("%u s", meas->time);
    }

    return result;
}

static bool step_prepare_lidar(meas_t *meas)
{
    lidar.setMeasurementTimingBudget(200000);
    return true;
}

static bool step_measure_range(meas_t *meas)
{
    meas->range = lidar.readRangeSingleMillimeters();
    print("%u mm", meas->range);
    return true;
}

static bool step_measure_temp_hum(meas_t *meas)
{
    int temp, humi;
    bool result;

    result = dht11.read(&humi, &temp);
    if (result) {
        meas->temperature = temp;
        meas->humidity = humi;
        print("%d%% / %d*C", humi, temp);
    }
    return result;
}

static bool step_publish_mqtt(meas_t *meas)
{
    static char value[128];
    snprintf(value, sizeof(value),
             "{\"id\":\"%s\",\"time\":%u,\"humidity\":%d,\"temperature\":%d,\"range\":%u}",
             meas->id, meas->time, meas->humidity, meas->temperature, meas->range);
    print(value);

    if (!mqttClient.connected()) {
        mqttClient.setServer(MQTT_HOST, MQTT_PORT);
        mqttClient.connect(meas->id);
    }
    if (mqttClient.connected()) {
        mqttClient.publish(MQTT_TOPIC, value, true);
        return true;
    } else {
        return false;
    }
}

static bool step_deep_sleep(meas_t *meas)
{
    uint32_t seconds;
    
    // shut down the LIDAR
    digitalWrite(PIN_SHUT, LOW);
    
    // calculate sleep time and enter deep sleep
    seconds = PERIOD - (meas->time % PERIOD);
    print("sleeping %d seconds", seconds);
    ESP.deepSleep(1000000UL * seconds);
    return false;
}

// table of consecutive steps involved in one measurement cycle
static step_t steps[] = {
    { "Connecting to WiFi",         step_connect_wifi },
    { "Retrieving time (SNTP)",     step_request_sntp },
    { "Preparing LIDAR",            step_prepare_lidar },
    { "Measuring range",            step_measure_range },
    { "Measuring humidity/temp",    step_measure_temp_hum },
    { "Publishing to MQTT",         step_publish_mqtt },
    { "Entering deep sleep",        step_deep_sleep }
};

void setup(void)
{
    Serial.begin(115200);

    // keep RESET de-asserted
    pinMode(D0, INPUT_PULLUP);
    
    // configure DHT power
    pinMode(PIN_DHT_GND, OUTPUT);
    digitalWrite(PIN_DHT_GND, LOW);
    pinMode(PIN_DHT_VCC, OUTPUT);
    digitalWrite(PIN_DHT_VCC, HIGH);

    // wake up the LIDAR
    pinMode(PIN_SHUT, OUTPUT);
    digitalWrite(PIN_SHUT, HIGH);
    Wire.begin();
    delay(2);
    lidar.init();
    lidar.setTimeout(500);

    // init DHT11
    dht11.init();

    step = steps;
    memset(&measurement, 0, sizeof(measurement));
    print("\nCrawlspace sensor\n");
}

void loop(void)
{
    bool result;
    unsigned long ms;
    
    ms = millis();
    print("[%6d] %s ... ", ms, step->desc);
    result = step->fn(&measurement);
    if (result) {
        print("\n");
        step++;
    } else {
        print("FAIL\n");
        step_deep_sleep(&measurement);
    }
}

