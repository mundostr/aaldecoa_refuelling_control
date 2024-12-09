/**
 * https://cfsensor.com/product/digital-sensor-xgzp6859d/
 * Dirección I2C por defecto sensor: 0X6D
 * https://cfsensor.com/wp-content/uploads/2022/11/XGZP6859D-Pressure-Sensor-V2.8.pdf
 * https://285624.selcdn.ru/syms1/iblock/86d/86de8d04aca354b601bbe58fb5c83577/e8405e72be3d9a460e4616f02ff6572d.pdf
 */

#include <Arduino.h>
#include <Wire.h>
#include <XGZP6897D.h> // https://github.com/fanfanlatulipe26/XGZP6897D

#define I2C_device_address 0x6D
#define SDA_PIN 6
#define SCL_PIN 7
#define SERIAL_BAUDS 115200
#define PRESSURE_SENSOR_K_FACTOR 64 // 0-100 kPa
#define PRESSURE_SENSOR_FREQ 1000

XGZP6897D pressureSensor(PRESSURE_SENSOR_K_FACTOR);

void sensorTask(void* parameter);

void scanI2CDevices() {
    uint8_t error, address;
    int nDevices = 0;
    
    Serial.println("Escaneando...");
    
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.print("ENCONTRADO 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        }
    }

    Serial.println(nDevices == 0 ? "No se encontraron dispositivos\n": "Scan finalizado\n");
}

void initI2C() {
    Wire.begin(SDA_PIN, SCL_PIN);
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
}

void init_sensor() {
    if (!pressureSensor.begin()) {
        Serial.println("Sensor ERROR");
        while (1);
    }
    Serial.println("Sensor OK");

    xTaskCreate(sensorTask, "sensorTask", 2048, NULL, 1, NULL);
}

float read_sensor() {
    float temperature, pressure;
    pressureSensor.readSensor(temperature, pressure);
    return pressure;
}

void sensorTask(void* parameter) {
    while (true) {
        float reading = read_sensor();
        Serial.printf("Presion: %.1f Pa\n", reading);
        
        vTaskDelay(PRESSURE_SENSOR_FREQ / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(SERIAL_BAUDS);
    Serial.println("SISTEMA INICIADO");

    initI2C();
    // scanI2CDevices();
    init_sensor();
}

void loop() {
    delay(1);
}
