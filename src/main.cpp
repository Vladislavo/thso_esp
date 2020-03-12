#include <Arduino.h>

#include <DHT.h>
#include <Wire.h>
#include <SHTSensor.h>
#include <HIHReader.h>

#include <Adafruit_ADS1015.h>

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
    float tmp36_0 = .0;
    float tmp36_1 = .0;
    float tmp36_2 = .0;
    float hih4030 = .0;
} sensor_data_t;

DHT dht(DHT22_PIN, DHT22);
SHTSensor sht85;
HIHReader hih8121(0x27);
Adafruit_ADS1115 ads;

sensor_data_t sensor_data;

void read_dht22(sensor_data_t *sensor_data);
void read_sht85(sensor_data_t *sensor_data);
void read_hih8121(sensor_data_t *sensor_data);
void read_hih4040(sensor_data_t *sensor_data);
void read_tmp36(sensor_data_t *sensor_data);

void setup() {
    Serial.begin(BAUDRATE);
    Wire.begin();

    dht.begin();
    sht85.init();
    sht85.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH); // only supported by SHT3x

    ads.begin();
    ads.setGain(GAIN_TWOTHIRDS); // +/-6.144V range
}

float t, h;

void loop() {
    read_tmp36(&sensor_data);
    Serial.printf("t0 = %.2f, t1 = %.2f, t2 = %.2f\r\n", sensor_data.tmp36_0, sensor_data.tmp36_1, sensor_data.tmp36_2);
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

void read_sht85(sensor_data_t *sensor_data) {
    sht85.readSample();
    sensor_data->sht85_t = sht85.getTemperature();
    sensor_data->sht85_h = sht85.getHumidity();
}

void read_hih8121(sensor_data_t *sensor_data) {
    hih8121.read(&sensor_data->hih8121_t, &sensor_data->hih8121_h);
}

// read after the sht85
void read_hih4040(sensor_data_t *sensor_data) {
    uint16_t adc_s = ads.readADC_SingleEnded(3);
    // magic numbers? -> read ads & hih4030 datasheets
    sensor_data->hih4030 = ((adc_s*0.1875)/1000)/0.031 - 25.8;
    // temperature adjustment (refer datasheet)
    sensor_data->hih4030 = sensor_data->hih4030 / (1.0546 - 0.0026 * sensor_data->sht85_t);
}

void read_tmp36(sensor_data_t *sensor_data) {
    sensor_data->tmp36_0 = (((ads.readADC_SingleEnded(0)*0.1875)/1000) - 0.5)*100;
    sensor_data->tmp36_1 = (((ads.readADC_SingleEnded(1)*0.1875)/1000) - 0.5)*100;
    sensor_data->tmp36_2 = (((ads.readADC_SingleEnded(2)*0.1875)/1000) - 0.5)*100;
}