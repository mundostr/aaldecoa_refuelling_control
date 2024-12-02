#include <Arduino.h>
#include <Wire.h>
#include <XGZP6897D.h> // https://github.com/fanfanlatulipe26/XGZP6897D

#define SERIAL_BAUDS 115200
#define PRESSURE_SENSOR_K_FACTOR 64 // 0-100 kPa: https://cfsensor.com/wp-content/uploads/2022/11/XGZP6859D-Pressure-Sensor-V2.8.pdf
#define PRESSURE_SENSOR_FREQ 1000

XGZP6897D pressureSensor(PRESSURE_SENSOR_K_FACTOR);

void init_sensor() {
    Wire.begin();

    if (!pressureSensor.begin()) {
        Serial.println("ERROR al inicializar sensor");
        while (1);
    }
}

float read_sensor() {
    float temperature, pressure;
    pressureSensor.readSensor(temperature, pressure);
    return pressure;
}

void setup() {
    Serial.begin(SERIAL_BAUDS);

    init_sensor();
}

void loop() {
    static uint32_t sensor_timer;

    if (millis() - sensor_timer >= PRESSURE_SENSOR_FREQ) {
        float reading = read_sensor();
        Serial.printf("Presion: %.1f Pa", reading);

        sensor_timer = millis();
    }
}
