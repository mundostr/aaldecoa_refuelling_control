/**
 * https://cfsensor.com/product/digital-sensor-xgzp6859d/
 * Dirección I2C por defecto sensor: 0X6D
 * https://cfsensor.com/wp-content/uploads/2022/11/XGZP6859D-Pressure-Sensor-V2.8.pdf
 * https://285624.selcdn.ru/syms1/iblock/86d/86de8d04aca354b601bbe58fb5c83577/e8405e72be3d9a460e4616f02ff6572d.pdf
 * https://github.com/fanfanlatulipe26/XGZP6897D
 * 
 * Lecturas:
 * 0.5 46.900, 0.4 36.300, 0.3 27.350, 0.2 18.500, 0.1 9.200
 */

#include <Wire.h>
#include <Arduino.h>
#include <algorithm>
#include <XGZP6897D.h>

#define BATTERY_PIN 2
#define BATTERY_LED_PIN 5
#define POT_PIN 3
#define SDA_PIN 6
#define SCL_PIN 7
#define PUMP_PIN 8

#define DEBUG
#define SERIAL_BAUDS 115200
#define BATTERY_DIVIDER 4.647 // resistencias de 27k y 100k
#define BATTERY_LOW 3.3
#define PRESSURE_MIN 0.1 // bar
#define PRESSURE_MAX 0.5
#define PRESSURE_OFFSET 600 // en kPa
#define PRESSURE_SAMPLES 10
#define PRESSURE_SENSOR_K 64 // 0-100 kPa
#define PRESSURE_HYSTERESIS 0.03
#define PUMP_ADJUST_INTERVAL 250
#define MEDIAN_READ_INTERVAL 30
#define ADC_RESOLUTION 4096 // 12 bits

int sampleIndex = 0;
bool samplesReady = false;
float readingSamples[PRESSURE_SAMPLES];

XGZP6897D pressureSensor(PRESSURE_SENSOR_K);

void sensor_task(void* parameter);
void pump_task(void* parameter);

float mapPressure(float analogValue) {
    return map(analogValue, 0, ADC_RESOLUTION, PRESSURE_MIN * 100, PRESSURE_MAX * 100) / 100.0;
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

void init_ios() {
    pinMode(POT_PIN, INPUT);
    pinMode(PUMP_PIN, OUTPUT);
    pinMode(BATTERY_PIN, INPUT);
    pinMode(BATTERY_LED_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
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

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 1, NULL);
    xTaskCreate(pump_task, "pump_task", 4096, NULL, 1, NULL);
}

float read_sensor() {
    float temperature, pressure;
    pressureSensor.readSensor(temperature, pressure);
    return pressure - PRESSURE_OFFSET;
}

float read_battery_voltage() {
    int adcValue = analogRead(BATTERY_PIN);
    float vOut = (adcValue / (float)(ADC_RESOLUTION - 1)) * 3.3;
    float vIn = vOut * BATTERY_DIVIDER;
    return vIn;
}

void pump_task(void* parameter) {
    float pressure = 0.0;

    while(true) {
        pressure = read_sensor();
        
        if (pressure != -1) {
            readingSamples[sampleIndex] = pressure;
            sampleIndex++;
            
            if (sampleIndex >= PRESSURE_SAMPLES) {
                sampleIndex = 0;
                samplesReady = true;
            }
        }

        vTaskDelay(MEDIAN_READ_INTERVAL / portTICK_PERIOD_MS);
    }
}

void sensor_task(void* parameter) {
    bool pumpOn = false;
    float medianReadingPa = 0.0;
    float currentPressureBar = 0.0;
    float targetPressureBar = 0.0;
    float batteryVoltage = 0.0;

    while(true) {
        batteryVoltage = read_battery_voltage();
        digitalWrite(BATTERY_LED_PIN, batteryVoltage < BATTERY_LOW ? HIGH: LOW);

        if (samplesReady) {
            std::sort(readingSamples, readingSamples + PRESSURE_SAMPLES);
            medianReadingPa = readingSamples[PRESSURE_SAMPLES / 2];
            currentPressureBar = medianReadingPa / 100000.0;
            targetPressureBar = mapPressure(analogRead(POT_PIN));

            if (currentPressureBar < targetPressureBar - PRESSURE_HYSTERESIS) {
                digitalWrite(PUMP_PIN, HIGH);
                pumpOn = true;
            } else if (currentPressureBar > targetPressureBar + PRESSURE_HYSTERESIS) {
                digitalWrite(PUMP_PIN, LOW);
                pumpOn = false;
            }

            #ifdef DEBUG
            Serial.printf("Lectura: %.2f bar, obj: %.2f bar, bomba: %s\n", currentPressureBar, targetPressureBar, pumpOn ? "ON" : "OFF");
            #endif
            
            samplesReady = false;
        }
        
        vTaskDelay(PUMP_ADJUST_INTERVAL / portTICK_PERIOD_MS);
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
