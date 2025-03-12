#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <DHT.h>
#include <time.h>

const int ACS712Pins[4] = {34, 35, 32, 33};  // Pins for ACS712 sensors
const float sensitivity = 0.185;             // Sensitivity 185 mV/A
const unsigned long measurementInterval = 2000;
#define TRIG_PIN1 18
#define ECHO_PIN1 19
#define DHTPIN 27
#define DHTTYPE DHT11
#define MQ135_PIN 26

// Global variables
float temperature = 0.0;
float humidity = 0.0;
int airQuality = 0;
float distance1 = 0.0;
const float distanceOrigin = 8.0;
long duration1;
float rate1;

float offsetVoltages[4] = {0.0, 0.0, 0.0, 0.0};
unsigned long lastMeasurementTime = 0;
bool calibrating = true;
const int calibrationSamples = 1000;
long calibrationSums[4] = {0, 0, 0, 0};
int calibrationCount = 0;
float currents[4] = {0.0, 0.0, 0.0, 0.0};
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // UART communication with ESP32 #2
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT_PULLDOWN);
  dht.begin();
}

float readUltrasonicSensor() {
  digitalWrite(TRIG_PIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN1, LOW);

  duration1 = pulseIn(ECHO_PIN1, HIGH);
  float distance = (duration1 / 2.0) / 29.412;
  return distance;
}

float calculateFoodRate(float distance) {
  float foodAvailable = (1.0 - (distance / distanceOrigin)) * 100.0;
  return foodAvailable;
}

float readCurrent(int pin, float offsetVoltage) {
  const int samplingTime = 100;
  unsigned long startTime = millis();
  long sum = 0;
  int numSamples = 0;

  while (millis() - startTime < samplingTime) {
    sum += analogRead(pin);
    numSamples++;
  }

  float voltage = (sum / (float)numSamples) * (5.0 / 1023.0); // Keeping original formula
  float current = abs((voltage - offsetVoltage) / sensitivity) / 2.5;
  return current;
}

void readSensors() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  airQuality = analogRead(MQ135_PIN);
  distance1 = readUltrasonicSensor();
  rate1 = calculateFoodRate(distance1);

  for (int i = 0; i < 4; i++) {
    currents[i] = readCurrent(ACS712Pins[i], offsetVoltages[i]);
  }
}

void calibrateSensors() {
  if (calibrationCount < calibrationSamples) {
    for (int i = 0; i < 4; i++) {
      calibrationSums[i] += analogRead(ACS712Pins[i]);
    }
    calibrationCount++;
  } else {
    for (int i = 0; i < 4; i++) {
      offsetVoltages[i] = (calibrationSums[i] / (float)calibrationSamples) * (5.0 / 1023.0);
    }
    calibrating = false;
  }
}

void sendDataOverUART() {
  unsigned long currentMillis = millis();
     if (currentMillis - lastMeasurementTime >= 2000) {
         lastMeasurementTime = currentMillis;
  String data = String("{") +
                currents[0] + "," +
                currents[1] + "," +
                currents[2] + "," +
                currents[3] + "," +
                rate1 + "," +
                offsetVoltages[0] + "," +
                offsetVoltages[1] + "," +
                offsetVoltages[2] + "," +
                offsetVoltages[3] + "," +
                temperature + "," +
                humidity + "," +
                airQuality +
                "}";

  Serial2.println(data);
}
}

void loop() {
  
  if (calibrating) {
    calibrateSensors();
  } else {
    readSensors();
    sendDataOverUART();
  }
}
