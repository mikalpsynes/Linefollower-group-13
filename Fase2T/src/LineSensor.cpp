#include "LineSensor.h"

LineSensor::LineSensor() : sensorCount(SENSOR_PIN_COUNT) {
    // Initialize sensor pins
    uint8_t pins[SENSOR_PIN_COUNT] = {
        GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14,
        GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_23, GPIO_NUM_22, GPIO_NUM_21,
        GPIO_NUM_19
    };

    for (uint8_t i = 0; i < SENSOR_PIN_COUNT; i++) {
        sensorPins[i] = pins[i];
    }
}

void LineSensor::init() {
    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, sensorCount);
    qtr.setEmitterPin(255);

    delay(100);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void LineSensor::calibrate() {
    Serial.println("Starting QTR calibration...");

    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();
        if ((i & 31) == 0) Serial.print('.');
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.println();
    Serial.println("Calibration complete!");

    printCalibration();
    delay(1000);
}

uint16_t LineSensor::readLinePosition() {
    return qtr.readLineBlack(sensorValues);
}

void LineSensor::printCalibration() {
    Serial.println("Calibration minimum values:");
    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    Serial.println("Calibration maximum values:");
    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
}

