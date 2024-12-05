/**
 * https://cfsensor.com/product/digital-sensor-xgzp6859d/
 * Dirección I2C por defecto sensor: 0X6D
 */

#include <Arduino.h>
#include <Wire.h>
#include <XGZP6897D.h> // https://github.com/fanfanlatulipe26/XGZP6897D

#define SDA_PIN 4
#define SCL_PIN 5
#define SERIAL_BAUDS 115200
#define PRESSURE_SENSOR_K_FACTOR 64 // 0-100 kPa: https://cfsensor.com/wp-content/uploads/2022/11/XGZP6859D-Pressure-Sensor-V2.8.pdf
#define PRESSURE_SENSOR_FREQ 1000

XGZP6897D pressureSensor(PRESSURE_SENSOR_K_FACTOR);

void sensorTask(void* parameter);

void init_sensor() {
    Wire.begin(SDA_PIN, SCL_PIN);
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);

    if (!pressureSensor.begin()) {
        Serial.println("Sensor ERROR");
        while (1);
    }
    Serial.println("Sensor OK");

    // xTaskCreate(sensorTask, "sensorTask", 2048, NULL, 1, NULL);
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

    init_sensor();
}

void loop() {
    delay(1);
}
