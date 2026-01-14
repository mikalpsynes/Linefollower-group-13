#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include <QTRSensors.h>

class LineSensor {
public:
    static const uint8_t SENSOR_PIN_COUNT = 11;

    LineSensor();
    void init();
    void calibrate();
    uint16_t readLinePosition();
    void printCalibration();

private:
    uint8_t sensorPins[SENSOR_PIN_COUNT];
    uint16_t sensorValues[SENSOR_PIN_COUNT];
    const uint8_t sensorCount;
    QTRSensors qtr;
};

#endif
