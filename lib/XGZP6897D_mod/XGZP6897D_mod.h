#ifndef XGZP6897D_H
#define XGZP6897D_H
#include <Arduino.h>
#include <Wire.h>

class XGZP6897D {
    public:
    XGZP6897D(uint16_t K, TwoWire* theWire = &Wire);
    bool begin();
    void readSensor(float &temperature, float &pressure);
    void readRawSensor(int16_t &rawTemperature, int32_t &rawPressure);
    
    private:
    float _K;
    uint8_t _i2c_address;
    TwoWire* _Wire;
};

#endif
