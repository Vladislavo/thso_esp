#include <Arduino.h>
#include <WiFi.h>

#include <DHT.h>
#include <Wire.h>
#include <SHTSensor.h>
#include <HIHReader.h>

#include <bus_protocol/bus_protocol.h>
#include <gateway_protocol/gateway_protocol.h>
#include <device_control.h>

#include <Adafruit_ADS1015.h>

#include <Time.h>

#include "driver/periph_ctrl.h"

#define BAUDRATE                            115200

#define DHT22_READ_RETRIES                  100

#define DHT22_PIN                           13
#define WIS_SYNC_PIN                        25
#define MKR_SYNC_PIN                        19

#define BUS_PROTOCOL_MAX_PACKET_SIZE        128
#define GATEWAY_PROTOCOL_MAX_PACKET_SIZE    150
#define BUS_PROTOCOL_MAX_WAITING_TIME       300

#define WIS_TX_PIN                          17
#define WIS_RX_PIN                          16
#define MKR_TX_PIN                          26
#define MKR_RX_PIN                          27

#define LED_SERIAL                          2

#define WIFI_SSID                           "ISRcomunicaciones34"
#define WIFI_PASSWORD                       "16818019"

#define GATEWAY_IP_ADDRESS                  IPAddress(192,168,0,123)
#define GATEWAY_PORT                        9043

#define TIME_ZONE                           2 // +2 Madrid
#define TIME_REQUEST_RETRIES                5

#define GET_DATA                            0
#define SET_CONF                            1
#define DEV_REBOOT                          2

#define DEFAULT_SAMPLE_PERIOD               60000

HardwareSerial bus_wis(2);
HardwareSerial bus_mkr(1);

typedef struct {
    uint32_t sample_period; // 1 min default
} dev_conf_t;

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
    uint32_t utc;
    char timedate[32];
    esp_sensor_data_t esp_sensor_data;
    wis_sensor_data_t wis_sensor_data;
    mkr_sensor_data_t mkr_sensor_data;
} sensor_data_t;

DHT dht(DHT22_PIN, DHT22);
SHTSensor sht85;
HIHReader hih8121(0x27);
Adafruit_ADS1115 ads;

WiFiUDP clientUDP;

void setup_hh10d();

void read_dht22(esp_sensor_data_t *sensor_data);
void read_sht85(esp_sensor_data_t *sensor_data);
void read_hih8121(esp_sensor_data_t *sensor_data);
void read_hih4040(esp_sensor_data_t *sensor_data);
void read_tmp36(esp_sensor_data_t *sensor_data);
void read_hh10d(esp_sensor_data_t *sensor_data);

void read_sensors(esp_sensor_data_t *sensor_data);

uint8_t bus_protocol_serial_receive(
    Stream *serial, 
    uint8_t *data, 
    uint8_t *data_length, 
    const uint32_t timeout);
uint8_t bus_protocol_data_send_decode(
    sensor_data_t *sensor_data,
    const uint8_t *payload,
    const uint8_t payload_length);

time_t gateway_protocol_get_time();
void  gateway_protocol_send_data_payload_encode (
    const sensor_data_t *sensor_data, 
    uint8_t *payload, 
    uint8_t *payload_length);
gateway_protocol_stat_t send_sensor_data(const sensor_data_t *sensor_data);

void source_modulation(Stream *stream, sensor_data_t *sensor_data);
void print_array_hex(uint8_t *array, uint8_t array_length, const char *sep);
void set_timestamp(sensor_data_t *sensor_data);
time_t time_sync();
uint8_t send_udp_datagram (
    const IPAddress ip, 
    const uint16_t port, 
    const uint8_t *packet, 
    const uint8_t packet_length);

sensor_data_t sensor_data;

uint8_t buffer[GATEWAY_PROTOCOL_MAX_PACKET_SIZE];
uint8_t buffer_length = 0;
uint8_t payload[GATEWAY_PROTOCOL_MAX_PACKET_SIZE];
uint8_t payload_length = 0;

dev_conf_t dev_conf;

float t, h;

int sens;
int ofs;

gateway_protocol_stat_t g_stat = GATEWAY_PROTOCOL_STAT_NACK;

void setup() {
    Serial.begin(BAUDRATE);
    Wire.begin();

    bus_wis.begin(BAUDRATE, SERIAL_8N1, WIS_RX_PIN, WIS_TX_PIN);
    bus_mkr.begin(BAUDRATE, SERIAL_8N1, MKR_RX_PIN, MKR_TX_PIN);

    dht.begin();
    sht85.init();
    sht85.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH); // only supported by SHT3x

    ads.begin();
    ads.setGain(GAIN_TWOTHIRDS); // +/-6.144V range

    setup_hh10d();

    pinMode(WIS_SYNC_PIN, OUTPUT);
    pinMode(MKR_SYNC_PIN, OUTPUT);

    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setSleep(false);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
    Serial.println("IP address set: ");
    Serial.println(WiFi.localIP()); //print LAN IP

    clientUDP.begin(GATEWAY_PORT);

    setSyncProvider(gateway_protocol_get_time);
    setSyncInterval(300);

    dev_conf.sample_period = DEFAULT_SAMPLE_PERIOD;
}

void loop() {
    read_sensors(&sensor_data.esp_sensor_data);
    source_modulation(&bus_wis, &sensor_data);
    source_modulation(&bus_mkr, &sensor_data);

    set_timestamp(&sensor_data);

    ESP_LOGD(TAG,   "WIS data: \r\n"
                    "\t%0.2f, %0.2f\r\n"
                    "\t%0.2f, %0.2f\r\n"
                    "\t%0.2f, %0.2f\r\n"
                    "\t%0.2f\r\n"
                    "\t%0.2f",
                    sensor_data.wis_sensor_data.dht22_t, sensor_data.wis_sensor_data.dht22_h,
                    sensor_data.wis_sensor_data.sht85_t, sensor_data.wis_sensor_data.sht85_h,
                    sensor_data.wis_sensor_data.hih8121_t, sensor_data.wis_sensor_data.hih8121_h,
                    sensor_data.wis_sensor_data.hh10d,
                    sensor_data.wis_sensor_data.tmp102);

    ESP_LOGD(TAG,   "MKR data: \r\n"
                    "\t%0.2f, %0.2f\r\n"
                    "\t%0.2f, %0.2f\r\n"
                    "\t%0.2f, %0.2f\r\n"
                    "\t%0.2f\r\n",
                    sensor_data.mkr_sensor_data.dht22_t, sensor_data.mkr_sensor_data.dht22_h,
                    sensor_data.mkr_sensor_data.sht85_t, sensor_data.mkr_sensor_data.sht85_h,
                    sensor_data.mkr_sensor_data.hih8121_t, sensor_data.mkr_sensor_data.hih8121_h,
                    sensor_data.mkr_sensor_data.hh10d);

    ESP_LOGD(TAG,   "ESP data: \r\n"
                    "\t%0.2f, %0.2f\r\n"
                    "\t%0.2f, %0.2f\r\n"
                    "\t%0.2f, %0.2f\r\n"
                    "\t%0.2f\r\n"
                    "\t%0.2f\r\n"
                    "\t%0.2f, %0.2f, %0.2f\r\n",
                    sensor_data.esp_sensor_data.dht22_t, sensor_data.esp_sensor_data.dht22_h,
                    sensor_data.esp_sensor_data.sht85_t, sensor_data.esp_sensor_data.sht85_h,
                    sensor_data.esp_sensor_data.hih8121_t, sensor_data.esp_sensor_data.hih8121_h,
                    sensor_data.esp_sensor_data.hh10d,
                    sensor_data.esp_sensor_data.hih4030,
                    sensor_data.esp_sensor_data.tmp36_0, sensor_data.esp_sensor_data.tmp36_1, sensor_data.esp_sensor_data.tmp36_2);

    
    g_stat = send_sensor_data(&sensor_data);
    
    if (g_stat == GATEWAY_PROTOCOL_STAT_ACK) {
        ESP_LOGD(TAG, "ACK received");
    } else if (g_stat == GATEWAY_PROTOCOL_STAT_ACK_PEND) {
        ESP_LOGD(TAG, "ACK_PEND received");
        gateway_protocol_packet_encode(
            BUS_PROTOCOL_BOARD_ID_ESP,
            GATEWAY_PROTOCOL_PACKET_TYPE_PEND_REQ,
            0, buffer,
            &buffer_length, buffer);

        send_udp_datagram(GATEWAY_IP_ADDRESS, GATEWAY_PORT, buffer, buffer_length);
        
        uint32_t wait_ms = millis() + 1000;
        while(!clientUDP.parsePacket() && wait_ms > millis()) {}
        if ((buffer_length = clientUDP.read(buffer, sizeof(buffer)))) {
            uint8_t dev_id;
            gateway_protocol_packet_type_t p_type;
            if (gateway_protocol_packet_decode(
                &dev_id,
                &p_type,
                &buffer_length, buffer,
                buffer_length, buffer))
            {
                if (dev_id == BUS_PROTOCOL_BOARD_ID_ESP &&
                    p_type == GATEWAY_PROTOCOL_PACKET_TYPE_PEND_SEND) 
                {
                    ESP_LOGD(TAG, "PEND SEND received");
                    print_array_hex(buffer, buffer_length, " : ");

                    uint8_t op, arg_len, args[32];
                    device_control_packet_decode(&op, &arg_len, args, buffer_length, buffer);

                    if (op == GET_DATA) {
                        // extra data_send
                    } else if (op == SET_CONF) {
                        // set 
                    } else if (op == DEV_REBOOT) {
                        periph_module_reset(PERIPH_WIFI_MODULE);
                        ESP.restart();
                        // see peripherals reset
                    } else {
                        // error
                    }
                }
            }
        } else {
            ESP_LOGD(TAG, "NO PEND SEND received");
        }
    } else {
        ESP_LOGD(TAG, "NACK %02X", g_stat);
    }

    delay(7000);
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

uint8_t bus_protocol_data_send_decode(
    sensor_data_t *sensor_data,
    const uint8_t *payload,
    const uint8_t payload_length)
{
    uint8_t p_len = 0;
    board_id_t board_id = BUS_PROTOCOL_BOARD_ID_UNKNOWN;

    board_id = (board_id_t) payload[0];
    p_len++;

    switch (board_id) {
    case BUS_PROTOCOL_BOARD_ID_WIS:
        memcpy(&sensor_data->wis_sensor_data.dht22_t, &payload[p_len], sizeof(sensor_data->wis_sensor_data.dht22_t));
        p_len += sizeof(sensor_data->wis_sensor_data.dht22_t);

        memcpy(&sensor_data->wis_sensor_data.dht22_h, &payload[p_len], sizeof(sensor_data->wis_sensor_data.dht22_h));
        p_len += sizeof(sensor_data->wis_sensor_data.dht22_h);

        memcpy(&sensor_data->wis_sensor_data.sht85_t, &payload[p_len], sizeof(sensor_data->wis_sensor_data.sht85_t));
        p_len += sizeof(sensor_data->wis_sensor_data.sht85_t);

        memcpy(&sensor_data->wis_sensor_data.sht85_h, &payload[p_len], sizeof(sensor_data->wis_sensor_data.sht85_h));
        p_len += sizeof(sensor_data->wis_sensor_data.sht85_h);

        memcpy(&sensor_data->wis_sensor_data.hih8121_t, &payload[p_len], sizeof(sensor_data->wis_sensor_data.hih8121_t));
        p_len += sizeof(sensor_data->wis_sensor_data.hih8121_t);

        memcpy(&sensor_data->wis_sensor_data.hih8121_h, &payload[p_len], sizeof(sensor_data->wis_sensor_data.hih8121_h));
        p_len += sizeof(sensor_data->wis_sensor_data.hih8121_h);
        
        memcpy(&sensor_data->wis_sensor_data.hh10d, &payload[p_len], sizeof(sensor_data->wis_sensor_data.hh10d));
        p_len += sizeof(sensor_data->wis_sensor_data.hh10d);

        memcpy(&sensor_data->wis_sensor_data.tmp102, &payload[p_len], sizeof(sensor_data->wis_sensor_data.tmp102));
        p_len += sizeof(sensor_data->wis_sensor_data.tmp102);
        break;

    case BUS_PROTOCOL_BOARD_ID_MKR:
        memcpy(&sensor_data->mkr_sensor_data.dht22_t, &payload[p_len], sizeof(sensor_data->mkr_sensor_data.dht22_t));
        p_len += sizeof(sensor_data->mkr_sensor_data.dht22_t);

        memcpy(&sensor_data->mkr_sensor_data.dht22_h, &payload[p_len], sizeof(sensor_data->mkr_sensor_data.dht22_h));
        p_len += sizeof(sensor_data->mkr_sensor_data.dht22_h);

        memcpy(&sensor_data->mkr_sensor_data.sht85_t, &payload[p_len], sizeof(sensor_data->mkr_sensor_data.sht85_t));
        p_len += sizeof(sensor_data->mkr_sensor_data.sht85_t);

        memcpy(&sensor_data->mkr_sensor_data.sht85_h, &payload[p_len], sizeof(sensor_data->mkr_sensor_data.sht85_h));
        p_len += sizeof(sensor_data->mkr_sensor_data.sht85_h);

        memcpy(&sensor_data->mkr_sensor_data.hih8121_t, &payload[p_len], sizeof(sensor_data->mkr_sensor_data.hih8121_t));
        p_len += sizeof(sensor_data->mkr_sensor_data.hih8121_t);

        memcpy(&sensor_data->mkr_sensor_data.hih8121_h, &payload[p_len], sizeof(sensor_data->mkr_sensor_data.hih8121_h));
        p_len += sizeof(sensor_data->mkr_sensor_data.hih8121_h);
        
        memcpy(&sensor_data->mkr_sensor_data.hh10d, &payload[p_len], sizeof(sensor_data->mkr_sensor_data.hh10d));
        p_len += sizeof(sensor_data->mkr_sensor_data.hh10d);
        break;
    
    default:
        break;
    }

    return p_len == payload_length;
}

time_t gateway_protocol_get_time() {
    uint32_t utc = 0;
    uint8_t buf[32];
    uint8_t buf_len = 0;
    uint8_t retries = 0;

    do {
        gateway_protocol_packet_encode(
                                (uint8_t) BUS_PROTOCOL_BOARD_ID_ESP, 
                                GATEWAY_PROTOCOL_PACKET_TYPE_TIME_REQ,
                                0, buf,
                                &buf_len, buf);

        if (send_udp_datagram(GATEWAY_IP_ADDRESS, GATEWAY_PORT, buf, buf_len)) {
            ESP_LOGD(TAG, "Time request complete!");
        } else {
            ESP_LOGE(TAG, "Time request failed!");
        }
        // delay(100);
        uint32_t wait_ms = millis() + 1000;
        while(!clientUDP.parsePacket() && wait_ms > millis()) {}
        if ((buf_len = clientUDP.read((unsigned char *)buf, sizeof(buf)))) {
            uint8_t dev_id;
            gateway_protocol_packet_type_t p_type;
            if (gateway_protocol_packet_decode(
                &dev_id,
                &p_type,
                &buf_len, buf,
                buf_len, buf))
            {
                if (dev_id == BUS_PROTOCOL_BOARD_ID_ESP &&
                    p_type == GATEWAY_PROTOCOL_PACKET_TYPE_TIME_SEND &&
                    buf_len == sizeof(uint32_t)) 
                {
                    memcpy(&utc, buf, sizeof(uint32_t));

                    struct timeval now = { .tv_sec = utc };
                    settimeofday(&now, NULL);

                    setTime(utc);
                } else {
                    ESP_LOGE(TAG, "time content decode error : %02X, %02X, %d", dev_id, p_type, buf_len);
                }
            } else {
                ESP_LOGE(TAG, "time pck decode error");
            }
        } else {
            ESP_LOGE(TAG, "no time response");
        }
    } while (utc == 0 && retries++ < TIME_REQUEST_RETRIES);

    return (time_t) utc;
}

void  gateway_protocol_send_data_payload_encode (
    const sensor_data_t *sensor_data, 
    uint8_t *payload, 
    uint8_t *payload_length) 
{
    *payload_length = 0;

    memcpy(&payload[*payload_length], &sensor_data->utc, sizeof(sensor_data->utc));
    (*payload_length) += sizeof(sensor_data->utc);

    memcpy(&payload[*payload_length], sensor_data->timedate, sizeof(sensor_data->timedate));
    // (*payload_length) += sizeof(sensor_data->timedate);
    (*payload_length) += 32;


    memcpy(&payload[*payload_length], &sensor_data->esp_sensor_data.dht22_t, sizeof(sensor_data->esp_sensor_data.dht22_t));
    (*payload_length) += sizeof(sensor_data->esp_sensor_data.dht22_t);

    memcpy(&payload[*payload_length], &sensor_data->esp_sensor_data.dht22_h, sizeof(sensor_data->esp_sensor_data.dht22_h));
    (*payload_length) += sizeof(sensor_data->esp_sensor_data.dht22_h);

    memcpy(&payload[*payload_length], &sensor_data->esp_sensor_data.sht85_t, sizeof(sensor_data->esp_sensor_data.sht85_t));
    (*payload_length) += sizeof(sensor_data->esp_sensor_data.sht85_t);

    memcpy(&payload[*payload_length], &sensor_data->esp_sensor_data.sht85_h, sizeof(sensor_data->esp_sensor_data.sht85_h));
    (*payload_length) += sizeof(sensor_data->esp_sensor_data.sht85_h);

    memcpy(&payload[*payload_length], &sensor_data->esp_sensor_data.hih8121_t, sizeof(sensor_data->esp_sensor_data.hih8121_t));
    (*payload_length) += sizeof(sensor_data->esp_sensor_data.hih8121_t);

    memcpy(&payload[*payload_length], &sensor_data->esp_sensor_data.hih8121_h, sizeof(sensor_data->esp_sensor_data.hih8121_h));
    (*payload_length) += sizeof(sensor_data->esp_sensor_data.hih8121_h);

    memcpy(&payload[*payload_length], &sensor_data->esp_sensor_data.tmp36_0, sizeof(sensor_data->esp_sensor_data.tmp36_0));
    (*payload_length) += sizeof(sensor_data->esp_sensor_data.tmp36_0);

    memcpy(&payload[*payload_length], &sensor_data->esp_sensor_data.tmp36_1, sizeof(sensor_data->esp_sensor_data.tmp36_1));
    (*payload_length) += sizeof(sensor_data->esp_sensor_data.tmp36_1);

    memcpy(&payload[*payload_length], &sensor_data->esp_sensor_data.tmp36_2, sizeof(sensor_data->esp_sensor_data.tmp36_2));
    (*payload_length) += sizeof(sensor_data->esp_sensor_data.tmp36_2);

    memcpy(&payload[*payload_length], &sensor_data->esp_sensor_data.hih4030, sizeof(sensor_data->esp_sensor_data.hih4030));
    (*payload_length) += sizeof(sensor_data->esp_sensor_data.hih4030);

    memcpy(&payload[*payload_length], &sensor_data->esp_sensor_data.hh10d, sizeof(sensor_data->esp_sensor_data.hh10d));
    (*payload_length) += sizeof(sensor_data->esp_sensor_data.hh10d);


    memcpy(&payload[*payload_length], &sensor_data->mkr_sensor_data.dht22_t, sizeof(sensor_data->mkr_sensor_data.dht22_t));
    (*payload_length) += sizeof(sensor_data->mkr_sensor_data.dht22_t);

    memcpy(&payload[*payload_length], &sensor_data->mkr_sensor_data.dht22_h, sizeof(sensor_data->mkr_sensor_data.dht22_h));
    (*payload_length) += sizeof(sensor_data->mkr_sensor_data.dht22_h);

    memcpy(&payload[*payload_length], &sensor_data->mkr_sensor_data.sht85_t, sizeof(sensor_data->mkr_sensor_data.sht85_t));
    (*payload_length) += sizeof(sensor_data->mkr_sensor_data.sht85_t);

    memcpy(&payload[*payload_length], &sensor_data->mkr_sensor_data.sht85_h, sizeof(sensor_data->mkr_sensor_data.sht85_h));
    (*payload_length) += sizeof(sensor_data->mkr_sensor_data.sht85_h);

    memcpy(&payload[*payload_length], &sensor_data->mkr_sensor_data.hih8121_t, sizeof(sensor_data->mkr_sensor_data.hih8121_t));
    (*payload_length) += sizeof(sensor_data->mkr_sensor_data.hih8121_t);

    memcpy(&payload[*payload_length], &sensor_data->mkr_sensor_data.hih8121_h, sizeof(sensor_data->mkr_sensor_data.hih8121_h));
    (*payload_length) += sizeof(sensor_data->mkr_sensor_data.hih8121_h);

    memcpy(&payload[*payload_length], &sensor_data->mkr_sensor_data.hh10d, sizeof(sensor_data->mkr_sensor_data.hh10d));
    (*payload_length) += sizeof(sensor_data->mkr_sensor_data.hh10d);


    memcpy(&payload[*payload_length], &sensor_data->wis_sensor_data.dht22_t, sizeof(sensor_data->wis_sensor_data.dht22_t));
    (*payload_length) += sizeof(sensor_data->wis_sensor_data.dht22_t);

    memcpy(&payload[*payload_length], &sensor_data->wis_sensor_data.dht22_h, sizeof(sensor_data->wis_sensor_data.dht22_h));
    (*payload_length) += sizeof(sensor_data->wis_sensor_data.dht22_h);

    memcpy(&payload[*payload_length], &sensor_data->wis_sensor_data.sht85_t, sizeof(sensor_data->wis_sensor_data.sht85_t));
    (*payload_length) += sizeof(sensor_data->wis_sensor_data.sht85_t);

    memcpy(&payload[*payload_length], &sensor_data->wis_sensor_data.sht85_h, sizeof(sensor_data->wis_sensor_data.sht85_h));
    (*payload_length) += sizeof(sensor_data->wis_sensor_data.sht85_h);

    memcpy(&payload[*payload_length], &sensor_data->wis_sensor_data.hih8121_t, sizeof(sensor_data->wis_sensor_data.hih8121_t));
    (*payload_length) += sizeof(sensor_data->wis_sensor_data.hih8121_t);

    memcpy(&payload[*payload_length], &sensor_data->wis_sensor_data.hih8121_h, sizeof(sensor_data->wis_sensor_data.hih8121_h));
    (*payload_length) += sizeof(sensor_data->wis_sensor_data.hih8121_h);

    memcpy(&payload[*payload_length], &sensor_data->wis_sensor_data.tmp102, sizeof(sensor_data->wis_sensor_data.tmp102));
    (*payload_length) += sizeof(sensor_data->wis_sensor_data.tmp102);

    memcpy(&payload[*payload_length], &sensor_data->wis_sensor_data.hh10d, sizeof(sensor_data->wis_sensor_data.hh10d));
    (*payload_length) += sizeof(sensor_data->wis_sensor_data.hh10d);
}

gateway_protocol_stat_t send_sensor_data(const sensor_data_t *sensor_data) {
    gateway_protocol_stat_t g_stat = GATEWAY_PROTOCOL_STAT_NACK;
    gateway_protocol_send_data_payload_encode(sensor_data, payload, &payload_length);
    gateway_protocol_packet_encode(
        BUS_PROTOCOL_BOARD_ID_ESP,
        GATEWAY_PROTOCOL_PACKET_TYPE_DATA_SEND,
        payload_length, payload,
        &buffer_length, buffer);

    ESP_LOGD(TAG, "sending %d bytes...", buffer_length);

    if (send_udp_datagram(GATEWAY_IP_ADDRESS, GATEWAY_PORT, buffer, buffer_length)) {
        ESP_LOGD(TAG, "data send done!");
    } else {
        ESP_LOGD(TAG, "data send error");
    }
    
    uint32_t wait_ms = millis() + 1000;
    while(!clientUDP.parsePacket() && wait_ms > millis()) {}
    if ((buffer_length = clientUDP.read((unsigned char *)buffer, sizeof(buffer)))) {
        uint8_t dev_id;
        gateway_protocol_packet_type_t p_type;
        if (gateway_protocol_packet_decode(
            &dev_id,
            &p_type,
            &buffer_length, buffer,
            buffer_length, buffer)) 
        {
            if (dev_id == BUS_PROTOCOL_BOARD_ID_ESP &&
                p_type == GATEWAY_PROTOCOL_PACKET_TYPE_STAT &&
                buffer_length == 1) 
            {
                g_stat = (gateway_protocol_stat_t) buffer[0];
                ESP_LOGD(TAG, "STAT RECEIVED %02X", g_stat);
            }
        }
    } else {
        ESP_LOGD(TAG, "NO STAT RECEIVED");
    }

    return g_stat;
}

void source_modulation(Stream *stream, sensor_data_t *sensor_data) {
    bus_protocol_data_request_encode(BUS_PROTOCOL_BOARD_ID_ESP, buffer, &buffer_length);
    stream->write(buffer, buffer_length);

    while(!bus_protocol_serial_receive(stream, buffer, &buffer_length, BUS_PROTOCOL_MAX_WAITING_TIME)) {
        digitalWrite(LED_SERIAL, HIGH);
        stream->write(buffer, buffer_length);
        digitalWrite(LED_SERIAL, LOW);
    }

    switch (bus_protocol_packet_decode(buffer, buffer_length, payload, &payload_length)) {
        case BUS_PROTOCOL_PACKET_TYPE_DATA_SEND :
            digitalWrite(LED_SERIAL, HIGH);
            // ESP_LOGD(TAG, "DATA SEND");
            // ESP_LOGD(TAG, "%d bytes", buffer_length);
            
            bus_protocol_data_send_decode(  sensor_data, 
                                            payload, 
                                            payload_length);

            digitalWrite(LED_SERIAL, LOW);
        default:
            break;
    }
}

void print_array_hex(uint8_t *array, uint8_t array_length, const char *sep) {
    for(uint8_t i = 0; i < array_length-1; i++) {
        Serial.printf("%02X%s", array[i], sep);
    }
    Serial.printf("%02X\r\n", array[array_length-1]);
}

void set_timestamp(sensor_data_t *sensor_data) {
    if (now() < 10000) {
        gateway_protocol_get_time();
    }
    struct timeval tv;
    gettimeofday(&tv, NULL);

    sensor_data->utc = tv.tv_sec;
    sprintf(sensor_data->timedate, "%d/%d/%d %d:%d:%d", day(), month(), year(), hour()+TIME_ZONE, minute(), second());
}

time_t time_sync() {
    gateway_protocol_get_time();

    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    return tv.tv_sec;
}

uint8_t send_udp_datagram (
    const IPAddress ip, 
    const uint16_t port, 
    const uint8_t *packet, 
    const uint8_t packet_length) 
{
    clientUDP.beginPacket(GATEWAY_IP_ADDRESS, GATEWAY_PORT);
    clientUDP.write(packet, packet_length);

    return clientUDP.endPacket();
}