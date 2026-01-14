// cpp
#include <Arduino.h>
#include <QTRSensors.h>

// PID
float Kp = 1;
float Ki = 0.04;
float Kd = 7;
int P;
int I;
int D;
int lastError = 0;

const uint8_t maxSpeedA = 250;
const uint8_t maxSpeedB = 250;
const uint8_t baseSpeedA = 200;
const uint8_t baseSpeedB = 200;

const int PWMA = GPIO_NUM_4;
const int AIN2 = GPIO_NUM_17;
const int AIN1 = GPIO_NUM_16;


const int BIN1 = GPIO_NUM_5;
const int BIN2 = GPIO_NUM_15;
const int PWMB = GPIO_NUM_32;


const int driveTime = 20;
const int turnTime = 8;

QTRSensors qtr;

const uint8_t SensorCount = 11;
uint16_t sensorValues[SensorCount];

// Map sensors to ESP32 ADC-capable GPIOs
const uint8_t sensorPins[SensorCount] = {GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14,
  GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_23, GPIO_NUM_22, GPIO_NUM_21,
  GPIO_NUM_19};

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// PWM channels for ESP32 ledc
const uint8_t PWMA_channel = 0;
const uint8_t PWMB_channel = 1;
const uint32_t PWM_freq = 5000;
const uint8_t PWM_resolution = 8; // 8-bit -> 0..255

void rightMotor(int motorSpeed);
void leftMotor(int motorSpeed);
void PID_control();

void setup()
{
  // Motor A pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // Motor B pins
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // PWM setup (remove duplicate pinMode for PWMA/PWMB)
  ledcSetup(PWMA_channel, PWM_freq, PWM_resolution);
  ledcAttachPin(PWMA, PWMA_channel);
  ledcSetup(PWMB_channel, PWM_freq, PWM_resolution);
  ledcAttachPin(PWMB, PWMB_channel);

  // Ensure motors are stopped during calibration
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA_channel, 0);
  ledcWrite(PWMB_channel, 0);

  Serial.begin(115200);
  Serial.println("QTR calibration");

  // Use analog mode for ADC pins on ESP32
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(255); // use pin 17 to control the IR LEDs

  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Calibrating");
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    if ((i & 31) == 0) Serial.print('.'); // sparse progress
  }
  digitalWrite(LED_BUILTIN, LOW);

  // print calibration min/max
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(1000);
}

void loop()
{
  PID_control();
}

void PID_control()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 5000 - position;
  Serial.println(error);

  P = error;
  I = I + error;
  if (I > 5000) I = 5000;
  if (I < -5000) I = -5000;
  D = error - lastError;
  lastError = error;

  float motorSpeedF = P * Kp + I * Ki + D * Kd;
  int motorspeed = (int)motorSpeedF;

  int motorSpeedA = baseSpeedA + motorspeed;
  int motorspeedB = baseSpeedB - motorspeed;
  Serial.print("HøyreM=");
  Serial.println(motorSpeedA);
  Serial.print("VenstreM=");
  Serial.println(motorspeedB);
  if (motorSpeedA > maxSpeedA) motorSpeedA = maxSpeedA;
  if (motorspeedB > maxSpeedB) motorspeedB = maxSpeedB;
  if (motorSpeedA < 0) motorSpeedA = -50;
  if (motorspeedB < 0) motorspeedB = -50;


  if ((error >= -750) && (error <= 750)) {
    rightMotor(maxSpeedA);
    leftMotor(maxSpeedB);
  } else {
    rightMotor(motorSpeedA);
    leftMotor(motorspeedB);
  }
}

void rightMotor(int motorSpeed)
{
  if (motorSpeed > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if (motorSpeed < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  uint8_t pwm = (uint8_t)min(255, abs(motorSpeed));
  ledcWrite(PWMA_channel, pwm);
}

void leftMotor(int motorSpeed)
{
  if (motorSpeed > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else if (motorSpeed < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  uint8_t pwm = (uint8_t)min(255, abs(motorSpeed));
  ledcWrite(PWMB_channel, pwm);
}