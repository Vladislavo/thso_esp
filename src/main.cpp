#include <Arduino.h>

#include <DHT.h>
#include <Wire.h>

#define BAUDRATE                            115200

#define DHT22_READ_RETRIES                  100

#define DHT22_PIN                           13

typedef struct {
    float dht22_t = .0;
    float dht22_h = .0;
} sensor_data_t;

DHT dht(DHT22_PIN, DHT22);

sensor_data_t sensor_data;

void read_dht22(sensor_data_t *sensor_data);

void setup() {
    Serial.begin(BAUDRATE);

    dht.begin();
}

float t, h;

void loop() {
    read_dht22(&sensor_data);

    Serial.printf("t = %.2f, h = %.2f\r\n", sensor_data.dht22_t, sensor_data.dht22_h);
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