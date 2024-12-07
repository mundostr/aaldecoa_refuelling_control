/**
 * https://cfsensor.com/product/digital-sensor-xgzp6859d/
 * Dirección I2C por defecto sensor: 0X6D
 * https://cfsensor.com/wp-content/uploads/2022/11/XGZP6859D-Pressure-Sensor-V2.8.pdf
 * https://285624.selcdn.ru/syms1/iblock/86d/86de8d04aca354b601bbe58fb5c83577/e8405e72be3d9a460e4616f02ff6572d.pdf
 * https://github.com/fanfanlatulipe26/XGZP6897D
 * 
 * Lecturas:
 * 0.5 46.900
 * 0.4 36.300
 * 0.3 27.350
 * 0.2 18.500
 * 0.1 9.200
 */

#include <Arduino.h>
#include <Wire.h>
#include <XGZP6897D.h>

#define DEBUG
#define I2C_device_address 0x6D
#define SDA_PIN 6
#define SCL_PIN 7
#define SERIAL_BAUDS 115200
#define PRESSURE_SENSOR_K_FACTOR 64 // 0-100 kPa
#define PRESSURE_SENSOR_FREQ 1000
#define POT_PIN 3
#define PUMP_CONTROL_PIN 8
#define PRESSURE_MIN 0.1 // bar
#define PRESSURE_MAX 0.5
#define HYSTERESIS 0.03
#define ADC_RESOLUTION 4096 // 12 bits

XGZP6897D pressureSensor(PRESSURE_SENSOR_K_FACTOR);

void sensor_task(void* parameter);

float mapPressure(float analogValue) {
    return map(analogValue, 0, ADC_RESOLUTION, PRESSURE_MIN * 100, PRESSURE_MAX * 100) / 100.0;
}

void init_ios() {
    pinMode(POT_PIN, INPUT);
    pinMode(PUMP_CONTROL_PIN, OUTPUT);
    digitalWrite(PUMP_CONTROL_PIN, LOW);
}

void scan_i2c_devices() {
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

void init_i2c() {
    Wire.begin(SDA_PIN, SCL_PIN);
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
}

void init_sensor() {
    if (!pressureSensor.begin()) {
        #ifdef DEBUG
        Serial.println("Sensor ERROR");
        #endif
        while (1);
    }
    #ifdef DEBUG
    Serial.println("Sensor OK");
    #endif

    xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 1, NULL);
}

float read_sensor() {
    float temperature, pressure;
    pressureSensor.readSensor(temperature, pressure);
    return pressure;
}

void sensor_task(void* parameter) {
    float targetPressure = 0.0;
    bool pumpOn = false;

    while (true) {
        float currentPressurePa = read_sensor();
        float currentPressureBar = currentPressurePa / 100000.0;
        int potValue = analogRead(POT_PIN);
        targetPressure = mapPressure(potValue);

        if (currentPressureBar < targetPressure - HYSTERESIS) {
            digitalWrite(PUMP_CONTROL_PIN, HIGH);
            pumpOn = true;
        } else if (currentPressureBar > targetPressure + HYSTERESIS) {
            digitalWrite(PUMP_CONTROL_PIN, LOW);
            pumpOn = false;
        }

        #ifdef DEBUG
        Serial.printf("Lectura: %.2f bar, obj: %.2f bar, bomba: %s\n", currentPressureBar, targetPressure, pumpOn ? "ON" : "OFF");
        #endif
        
        vTaskDelay(PRESSURE_SENSOR_FREQ / portTICK_PERIOD_MS);
    }
}

void setup() {
    #ifdef DEBUG
    Serial.begin(SERIAL_BAUDS);
    Serial.println("SISTEMA INICIADO");
    #endif

    init_i2c();
    init_sensor();
    init_ios();
}

void loop() {
}
