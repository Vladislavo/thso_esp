#include <Arduino.h>

#include <DHT.h>
#include <Wire.h>
#include <SHTSensor.h>
#include <HIHReader.h>

#define BAUDRATE                            115200

#define DHT22_READ_RETRIES                  100

#define DHT22_PIN                           13
#define I2C_SCL                             22
#define I2C_SDA                             21

typedef struct {
    float dht22_t = .0;
    float dht22_h = .0;
    float sht85_t = .0;
    float sht85_h = .0;
    float hih8121_t = .0;
    float hih8121_h = .0;
} sensor_data_t;

DHT dht(DHT22_PIN, DHT22);
SHTSensor sht85;
HIHReader hih8121(0x27);

sensor_data_t sensor_data;

void read_dht22(sensor_data_t *sensor_data);

void setup() {
    Serial.begin(BAUDRATE);
    Wire.begin();

    dht.begin();
    sht85.init();
    sht85.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH); // only supported by SHT3x
}

float t, h;

void loop() {
    sht85.readSample();
    sensor_data.sht85_t = sht85.getTemperature();
    sensor_data.sht85_h = sht85.getHumidity();

    hih8121.read(&sensor_data.hih8121_t, &sensor_data.hih8121_h);

    Serial.printf("s t = %.2f, h = %.2f\r\n", sensor_data.sht85_t, sensor_data.sht85_h);
    Serial.printf("h t = %.2f, h = %.2f\r\n", sensor_data.hih8121_t, sensor_data.hih8121_h);
    delay(1000);
}

void read_dht22(sensor_data_t *sensor_data) {
    uint8_t retries = DHT22_READ_RETRIES;

    do {
        delay(100);
        sensor_data->dht22_t = dht.readTemperature();
        sensor_data->dht22_h = dht.readHumidity();
        if (isnan(sensor_data->dht22_t) || isnan(sensor_data->dht22_h)) {
            ESP_LOGE(TAG, "Failed to read from DHT sensor!\n");
            delay(100);
        }
        retries--;
    } while ((isnan(isnan(sensor_data->dht22_t)) || isnan(sensor_data->dht22_h)) && retries);
}