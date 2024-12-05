#include <Arduino.h>
#include <Wire.h>

#define FILTER_SIZE 10
#define I2C_device_address 0x6D

XGZP6897D(uint16_t K, TwoWire* theWire = &Wire) {
    _i2c_address = I2C_device_address;
}

bool XGZP6897D::begin() {
    Wire.begin();
    for (int i = 0; i < FILTER_SIZE; i++) pressureBuffer[i] = 0;
    bufferIndex = 0;
    
    return true;
}

float XGZP6897D::readRawPressure() {
    uint8_t data[3];

    Wire.beginTransmission(_i2c_address);
    Wire.write(0xF1); // Read pressure
    Wire.endTransmission();
    delay(5);
    Wire.requestFrom(_i2c_address, (uint8_t)3);

    for (int i = 0; i < 3; i++) data[i] = Wire.read();

    int32_t rawPressure = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    rawPressure >>= 8; // Adjust the 24-bit data to 16-bit
    
    return (float)rawPressure;
}

float XGZP6897D::readPressure() {
    float rawPressure = readRawPressure();
    
    // Apply scaling for 20-50 kPa range
    const float RAW_MIN = 3277;  // Example raw value for 20 kPa (adjust based on datasheet)
    const float RAW_MAX = 8192;  // Example raw value for 50 kPa (adjust based on datasheet)
    return (rawPressure - RAW_MIN) / (RAW_MAX - RAW_MIN) * (50.0 - 20.0) + 20.0;
}

float XGZP6897D::readFilteredPressure() {
    float pressure = readPressure();

    pressureBuffer[bufferIndex] = pressure;
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;

    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) sum += pressureBuffer[i];
    
    return sum / FILTER_SIZE;
}

void XGZP6897D::calibrate(float pressureAt20kPa, float pressureAt50kPa) {
    float rawAt20kPa = readRawPressure(); // Read raw pressure at 20 kPa
    float rawAt50kPa = readRawPressure(); // Read raw pressure at 50 kPa

    zeroOffset = pressureAt20kPa - rawAt20kPa;
    scaleFactor = (50.0 - 20.0) / (rawAt50kPa - rawAt20kPa);
}

float XGZP6897D::readCalibratedPressure() {
    float rawPressure = readRawPressure();
    
    return (rawPressure + zeroOffset) * scaleFactor;
}

float XGZP6897D::readClampedPressure() {
    float pressure = readPressure();

    if (pressure < 20.0) {
        pressure = 20.0;
    } else if (pressure > 50.0) {
        pressure = 50.0;
    }

    return pressure;
}

float XGZP6897D::readCompensatedPressure(float temperature) {
    float pressure = readPressure();
    // Example temperature compensation factor
    float temperatureCompensationFactor = 1.0 + 0.001 * (temperature - 25.0);
    
    return pressure * temperatureCompensationFactor;
}

float XGZP6897D::readTemperature() {
    uint8_t data[2];

    Wire.beginTransmission(_i2c_address);
    Wire.write(0xE3); // Read temperature
    Wire.endTransmission();
    delay(5);
    Wire.requestFrom(_i2c_address, (uint8_t)2);

    for (int i = 0; i < 2; i++) data[i] = Wire.read();
    int16_t rawTemperature = ((int16_t)data[0] << 8) | data[1];
    
    return (float)rawTemperature * 0.01; // Adjust scale as per datasheet
}
