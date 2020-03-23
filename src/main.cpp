#include <Arduino.h>

#include <DHT.h>
#include <Wire.h>
#include <SHTSensor.h>
#include <HIHReader.h>
#include <bus_protocol/bus_protocol.h>

#include <Adafruit_ADS1015.h>

#define BAUDRATE                            115200

#define DHT22_READ_RETRIES                  100

#define DHT22_PIN                           13
#define WIS_SYNC_PIN                        25
#define MKR_SYNC_PIN                        19

#define BUS_PROTOCOL_MAX_PACKET_SIZE        128
#define BUS_PROTOCOL_MAX_WAITING_TIME       300

#define WIS_TX_PIN                          17
#define WIS_RX_PIN                          16
#define MKR_TX_PIN                          26
#define MKR_RX_PIN                          27

HardwareSerial bus_wis(2);

typedef struct {
    float dht22_t   = .0;
    float dht22_h   = .0;
    float sht85_t   = .0;
    float sht85_h   = .0;
    float hih8121_t = .0;
    float hih8121_h = .0;
    float tmp36_0   = .0;
    float tmp36_1   = .0;
    float tmp36_2   = .0;
    float hih4030   = .0;
    float hh10d     = .0;
} esp_sensor_data_t;

typedef struct {
    float dht22_t   = .0;
    float dht22_h   = .0;
    float sht85_t   = .0;
    float sht85_h   = .0;
    float hih8121_t = .0;
    float hih8121_h = .0;
    float tmp102    = .0;
    float hh10d     = .0;
} wis_sensor_data_t;

typedef struct {
    float dht22_t   = .0;
    float dht22_h   = .0;
    float sht85_t   = .0;
    float sht85_h   = .0;
    float hih8121_t = .0;
    float hih8121_h = .0;
    float hh10d     = .0;
} mkr_sensor_data_t;

typedef struct {
    esp_sensor_data_t esp_sensor_data;
    wis_sensor_data_t wis_sensor_data;
    mkr_sensor_data_t mkr_sensor_data;
} sensor_data_t;

DHT dht(DHT22_PIN, DHT22);
SHTSensor sht85;
HIHReader hih8121(0x27);
Adafruit_ADS1115 ads;

void setup_hh10d();

void read_dht22(esp_sensor_data_t *sensor_data);
void read_sht85(esp_sensor_data_t *sensor_data);
void read_hih8121(esp_sensor_data_t *sensor_data);
void read_hih4040(esp_sensor_data_t *sensor_data);
void read_tmp36(esp_sensor_data_t *sensor_data);
void read_tmp102(esp_sensor_data_t *sensor_data);
void read_hh10d(esp_sensor_data_t *sensor_data);

void read_sensors(esp_sensor_data_t *sensor_data);

sensor_data_t sensor_data;
uint8_t buffer[BUS_PROTOCOL_MAX_PACKET_SIZE];
uint8_t buffer_length = 0;

void setup() {
    Serial.begin(BAUDRATE);
    Wire.begin();

    bus_wis.begin(BAUDRATE, SERIAL_8N1, WIS_RX_PIN, WIS_TX_PIN);

    dht.begin();
    sht85.init();
    sht85.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH); // only supported by SHT3x

    ads.begin();
    ads.setGain(GAIN_TWOTHIRDS); // +/-6.144V range

    setup_hh10d();
}

float t, h;

int sens;
int ofs;

void loop() {
    // give signals to mkr and wis to collect the data
    digitalWrite(WIS_SYNC_PIN, HIGH);
    delay(10);
    digitalWrite(WIS_SYNC_PIN, LOW);
    
    digitalWrite(MKR_SYNC_PIN, HIGH);
    delay(10);
    digitalWrite(MKR_SYNC_PIN, LOW);
    
    read_sensors(&sensor_data.esp_sensor_data);

    while (!digitalRead(WIS_SYNC_PIN)) { }
    
    bus_protocol_data_request_encode(BUS_PROTOCOL_BOARD_ID_ESP, buffer, &buffer_length);

    bus_wis.write(buffer, buffer_length);

    if (bus_protocol_serial_receive(&bus_wis, buffer, &buffer_length, BUS_PROTOCOL_MAX_WAITING_TIME)) {
            switch (bus_protocol_packet_decode(buffer, buffer_length, buffer, &buffer_length)) {
                case BUS_PROTOCOL_PACKET_TYPE_DATA_SEND :
                    digitalWrite(LED_SERIAL_RECEIVE, HIGH);
                    ESP_LOGD(TAG, "WIS DATA SEND");
                    ESP_LOGD(TAG, "%d bytes", buffer_length);
                    
                    bus_protocol_data_send_decode(&sensors_data, buffer, buffer_length);

                    bus_protocol_packet_encode(BUS_PROTOCOL_PACKET_TYPE_ACK, buffer, 0, buffer, &buffer_length);
                    bus_wis.write(buffer, buffer_length);

                    digitalWrite(LED_SERIAL_RECEIVE, LOW);
                default:
                    break;
            }
        }

}

// function to intitialize HH10D
int i2cRead2bytes(int deviceaddress, byte address) {
    // SET ADDRESS
    Wire.beginTransmission(deviceaddress);
    Wire.write(address); // address for sensitivity
    Wire.endTransmission();

    Wire.requestFrom(deviceaddress, 2);

    int rv = 0;
    for (int c = 0; c < 2; c++ ) {
        if (Wire.available()) {
            rv = rv * 256 + Wire.read();
        }
    }

    return rv;
}

void setup_hh10d() {
    const int HH10D_I2C_ADDRESS = 81;
    sens = i2cRead2bytes(HH10D_I2C_ADDRESS, 10); 
	  ofs  = i2cRead2bytes(HH10D_I2C_ADDRESS, 12);
}

void read_dht22(esp_sensor_data_t *sensor_data) {
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

void read_sht85(esp_sensor_data_t *sensor_data) {
    sht85.readSample();
    sensor_data->sht85_t = sht85.getTemperature();
    sensor_data->sht85_h = sht85.getHumidity();
}

void read_hih8121(esp_sensor_data_t *sensor_data) {
    hih8121.read(&sensor_data->hih8121_t, &sensor_data->hih8121_h);
}

// read after the sht85
void read_hih4040(esp_sensor_data_t *sensor_data) {
    uint16_t adc_s = ads.readADC_SingleEnded(3);
    // magic numbers? -> read ads & hih4030 datasheets
    sensor_data->hih4030 = ((adc_s*0.1875)/1000)/0.031 - 25.8;
    // temperature adjustment (refer datasheet)
    sensor_data->hih4030 = sensor_data->hih4030 / (1.0546 - 0.0026 * sensor_data->sht85_t);
}

void read_tmp36(esp_sensor_data_t *sensor_data) {
    sensor_data->tmp36_0 = (((ads.readADC_SingleEnded(0)*0.1875)/1000) - 0.5)*100;
    sensor_data->tmp36_1 = (((ads.readADC_SingleEnded(1)*0.1875)/1000) - 0.5)*100;
    sensor_data->tmp36_2 = (((ads.readADC_SingleEnded(2)*0.1875)/1000) - 0.5)*100;
}

void read_hh10d(esp_sensor_data_t *sensor_data) {
    const int HH10D_FOUT_PIN    = 23;
    float freq = .0;
      for (int j=0; j < 256; j++) {
          freq += 500000/pulseIn(HH10D_FOUT_PIN, HIGH, 250000);
      }
    freq /= 256;

    sensor_data->hh10d = float((ofs - freq)* sens)/float(4096);
}

void read_sensors(esp_sensor_data_t *sensor_data) {
    read_dht22(sensor_data);
    read_sht85(sensor_data);
    read_hih8121(sensor_data);
    read_hih4040(sensor_data);
    read_tmp36(sensor_data);
    read_tmp102(sensor_data);
    read_hh10d(sensor_data);
}

uint8_t bus_protocol_serial_receive(Stream *serial, uint8_t *data, uint8_t *data_length, const uint32_t timeout) {
    *data_length = 0;
    uint32_t start_millis = millis();
    while(start_millis + timeout > millis() && *data_length < BUS_PROTOCOL_MAX_PACKET_SIZE) {
        if (serial->available()) {
            data[(*data_length)++] = serial->read();
            // update wating time
            start_millis = millis();
        }
    }

    return *data_length;
}
