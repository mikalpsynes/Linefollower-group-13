#include <Arduino.h>
#include <QTRSensors.h>

// PID-parametre
float Kp = 0.22;
float Ki = 0.015;
float Kd = 2.1;
int P;
int I;
int D;
int lastError = 0;
float dFiltered = 0;   // filtrert D-ledd

// Hastighetsgrenser
const uint8_t maxSpeedA = 250;
const uint8_t maxSpeedB = 250;
const uint8_t baseSpeedA = 225;
const uint8_t baseSpeedB = 225;

// Motorpinner
const int AIN1 = 13;
const int AIN2 = 12;
const int PWMA = 11;
const int PWMB = 10;
const int BIN2 = 9;
const int BIN1 = 8;

// QTR-sensorer
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// For slew-rate limiter
int lastA = 0, lastB = 0;

void setup()
{
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);

  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // --- manuell kalibrering ---
  Serial.println("Kalibrering – skyv roboten over linja...");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    if (i % 50 == 0) {
      Serial.print("Kalibrering steg: ");
      Serial.println(i);
    }
    delay(20); // litt tid til å flytte roboten manuelt
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Kalibrering ferdig!");

  // Skriv min/max verdier
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void loop()
{
  PID_control();
}

void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 2500 - position;

  // Deadband for rett linje
  if (abs(error) < 40) error = 0;


  P = error;
  I = I + error;

  // Leaky integrator (hindrer oppbygging)
  I = (int)(0.95f * I);


  // Anti-windup
  if (I > 1000) I = 1000;
  if (I < -1000) I = -1000;

  D = error - lastError;
  lastError = error;

  // Filtrert D
  const float alpha = 0.8;
  dFiltered = alpha * (float)D + (1.0 - alpha) * dFiltered;

  int motorspeed = P*Kp + I*Ki + dFiltered*Kd;

  // --- fartsreduksjon i sving ---
  float e = abs((float)error);
  float scale = 1.0 - 0.5 * (e / 2500.0);
  if (scale < 0.5) scale = 0.5;
  uint8_t baseA = (uint8_t)(baseSpeedA * scale);
  uint8_t baseB = (uint8_t)(baseSpeedB * scale);

  int motorSpeedA = baseA + motorspeed;
  int motorSpeedB = baseB - motorspeed;

  if (motorSpeedA > maxSpeedA) motorSpeedA = maxSpeedA;
  if (motorSpeedB > maxSpeedB) motorSpeedB = maxSpeedB;
  if (motorSpeedA < 40) motorSpeedA = 40;
  if (motorSpeedB < 40) motorSpeedB = 40;

  // --- slew-rate limiter ---
  const int step = 40;
  if (motorSpeedA > lastA + step) motorSpeedA = lastA + step;
  if (motorSpeedA < lastA - step) motorSpeedA = lastA - step;
  if (motorSpeedB > lastB + step) motorSpeedB = lastB + step;
  if (motorSpeedB < lastB - step) motorSpeedB = lastB - step;
  lastA = motorSpeedA;
  lastB = motorSpeedB;

  rightMotor(-motorSpeedB);
  leftMotor(motorSpeedA);
}

void rightMotor(int motorSpeed) {
  if (motorSpeed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if (motorSpeed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, abs(motorSpeed));
}

void leftMotor(int motorSpeed) {
  if (motorSpeed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else if (motorSpeed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, abs(motorSpeed));
}
