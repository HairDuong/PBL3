#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <DHT.h>
#include<ESP32Servo.h>
#include <time.h>
#include <PCF8574.h>


PCF8574 pcf8574(0x20); 

// Configuration for NTP Time
const long gmtOffset_sec = 7 * 3600;   
const int daylightOffset_sec = 0;

// Wi-Fi and MQTT settings
const char *ssid PROGMEM = "Hoang11";
const char *password PROGMEM = "0";
const char *mqtt_broker PROGMEM = "broker.emqx.io";
const char *topic0 PROGMEM = "esp32/temp";
const char *topic1 PROGMEM = "esp32/hum";
const char *topic2 PROGMEM = "esp32/air";
const char *topic3 PROGMEM = "esp32/foodrate";
const char *topic4 PROGMEM = "esp32/fan/control";
const char *topic5 PROGMEM = "esp32/pump/control";
const char *topic6 PROGMEM = "esp32/mode";
const char *topic7 PROGMEM = "esp32/servo/control";
const char *topic8 PROGMEM = "esp32/bulb/control";
const char *topic9 PROGMEM = "esp32/pump2/control";
const char *topic10 PROGMEM = "esp32/fan/test";
const char *topic11 PROGMEM = "esp32/pump/test";
const char *topic12 PROGMEM = "esp32/pump2/test";
const char *topic13 PROGMEM = "esp32/bulb/test";
const char *topic14 PROGMEM = "esp32/waterrate";
const char *topic15 PROGMEM = "targetHour";
const char *topic16 PROGMEM = "targetMinute";
const char *topic17 PROGMEM = "targetSecond";
const char *mqtt_username PROGMEM = "hoangpham1";
const char *mqtt_password PROGMEM = "123456";
const int mqtt_port PROGMEM = 1883;

// Pin definitions
#define servoPin 17
#define DHTPIN 5
#define DHTTYPE DHT11
#define MQ135_PIN 32
#define TRIG_PIN1 19
#define ECHO_PIN1 18 
#define TRIG_PIN2 14
#define ECHO_PIN2 12
#define FAN_BUTTON_PIN 0
#define FAN_RELAY_PIN 27
#define PUMP_BUTTON_PIN 1
#define PUMP_RELAY_PIN 25
#define PUMP2_BUTTON_PIN 2
#define PUMP2_RELAY_PIN 23
#define MODE_BUTTON_PIN 5
#define BULB_RELAY_PIN 33
#define BULB_BUTTON_PIN 15
#define SERVO_BUTTON_PIN 4 
#define OLED_BUTTON_PIN 6 
#define SENSITIVITY 0.185  
#define ADC_RESOLUTION 4095.0  
#define ADC_VOLTAGE 3.3 
#define SAMPLING_COUNT 1000  
#define NO_CURRENT_THRESHOLD 0.1  
const int ACS712Pin1 = 35;       
const int ACS712Pin2 = 34;       
const int ACS712Pin3 = 39;      
const int ACS712Pin4 = 36; 

// Global variables
float temperature = 0.0;
float humidity = 0.0;
int airQuality = 0;
long duration1;
long duration2;
float distance1=0.0;
float distance2=0.0;
float distanceOrigin = 8.0;
String rate1;
String rate2;
bool isFanOn = false;
bool lastFanButtonState = HIGH; 
bool isPumpOn = false;
bool lastPumpButtonState = HIGH;
bool isPump2On = false;
bool lastPump2ButtonState = HIGH;
bool isBulbOn = false;
bool lastBulbButtonState = HIGH; 
bool lastOledButtonState = HIGH;
unsigned long lastfeeding = 0;
unsigned long autofeedinginterval = 1000;
int targetHour = 13;
int targetMinute = 43;
int targetSecond = 0;
int targetSecondclose = 0;
const unsigned long debounceTime = 50;
int lastModeButtonState = HIGH;
bool isServoAt90 = false;  
int lastServoButtonState = HIGH;  
struct tm timeinfo;
int currentScreen = 0;

unsigned long previousMillisPump = 0; 
unsigned long previousMillisPump2 = 0;
unsigned long previousMillisServo = 0;
unsigned long previousMillisFan = 0;
unsigned long previousMillisBulb = 0;
unsigned long previousMillisMode = 0;
unsigned long previousMillisOled = 0;
const long debounceInterval = 50;  
unsigned long previousMillis = 0;
const long interval = 5000;
float zeroPointVoltage1 = 2.5; 
unsigned long previousMillisWiFi = 0;
unsigned long previousMillisMQTT = 0;
const long wifiInterval = 500;  
const long mqttInterval = 2000; 
const float sensitivity = 0.185; 
const int numSamples = 100;      
float offsetVoltage1 = 0.0;      
float offsetVoltage2 = 0.0;      
float offsetVoltage3 = 0.0;      
float offsetVoltage4 = 0.0;      
unsigned long lastMeasurementTime = 0; 
bool calibrating = true;        
int calibrationSamples = 1000;  
long calibrationSum1 = 0;        
long calibrationSum2 = 0;        
long calibrationSum3 = 0;       
long calibrationSum4 = 0;        
int calibrationCount = 0;
float current1;
float current2;
float current3;
float current4;

WiFiClient espClient;
PubSubClient client(espClient);
Servo myServo;
DHT dht(DHTPIN, DHTTYPE);
enum ControlMode { MANUAL, AUTOMATIC };
ControlMode currentMode = MANUAL;

// OLED Display 
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire, -1);
unsigned long previousSensorMillis = 0; 
const unsigned long updateSensorInterval = 2000; 
bool showSensorData = true;

bool connectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisWiFi >= wifiInterval) {
      previousMillisWiFi = currentMillis;
      Serial.println("Connecting to WiFi...");
      WiFi.begin(ssid, password);
      Serial.println("Connected to the WiFi network");
    }
    return false;
  }
  
  return true;
}
bool connectMQTT() {
  if (!client.connected()) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisMQTT >= mqttInterval) {
      previousMillisMQTT = currentMillis;
       String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public emqx mqtt broker connected");
      } else {
        Serial.println("Failed to connect to MQTT, retrying...");
      }
    }
    return false;
  }
  return true;
}
void callback(char *topic, byte *payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += (char)payload[i];

  if (String(topic) == topic4) isFanOn = (message == "ON");
  if (String(topic) == topic5) isPumpOn = (message == "ON");
  if (String(topic) == topic6) currentMode = (message == "Manual") ? MANUAL : AUTOMATIC;
  if (String(topic) == topic7) myServo.write(message == "ON" ? 90 : 0);
  if (String(topic) == topic8) isBulbOn = (message == "ON");
  if (String(topic) == topic9) isPump2On = (message == "ON");
  if (String(topic) == topic15) targetHour = message.toInt() ;
  if (String(topic) == topic16) targetMinute = message.toInt() ;
  if (String(topic) == topic17) 
  {targetSecond = message.toInt() ;
   targetSecondclose = targetSecond +10;}
  
  digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);
  digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);
  digitalWrite(PUMP2_RELAY_PIN, isPump2On ? HIGH : LOW);
  digitalWrite(BULB_RELAY_PIN, isBulbOn ? HIGH : LOW);
}
void automaticfeeding() {
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Không thể lấy thời gian từ NTP Server");
    return;
  }

  if (timeinfo.tm_hour == targetHour && timeinfo.tm_min == targetMinute && timeinfo.tm_sec == targetSecond) 
  {
    if (myServo.read() != 90) 
    {
      Serial.println("Đúng giờ! Điều khiển servo đến góc 90 độ.");
      myServo.write(90);
    }
  }
     if (timeinfo.tm_hour == targetHour && timeinfo.tm_min == targetMinute && timeinfo.tm_sec == (targetSecondclose)) {
     {
      Serial.println("Servo đã ở góc 90 độ trong 10 giây. Quay lại góc 0 độ.");
      myServo.write(0);  
     }
  }
}
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pcf8574.begin();
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT_PULLDOWN);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT_PULLDOWN);
  dht.begin();
  myServo.attach(servoPin);
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");
  myServo.write(0);
  automaticfeeding();
  if (!display.begin(0x3C, true)) {
  Serial.println("OLED initialization failed");
  while (1);
  }
  display.display();
  display.clearDisplay();

  pinMode(FAN_RELAY_PIN, OUTPUT);        
  digitalWrite(FAN_RELAY_PIN, LOW);      
  pinMode(PUMP_RELAY_PIN, OUTPUT);        
  digitalWrite(PUMP_RELAY_PIN, LOW);   
  pinMode(PUMP2_RELAY_PIN, OUTPUT);        
  digitalWrite(PUMP2_RELAY_PIN, LOW); 
  pinMode(BULB_RELAY_PIN, OUTPUT);        
  digitalWrite(BULB_RELAY_PIN, LOW);   


  WiFi.begin(ssid, password);
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  client.subscribe(topic4);
  client.subscribe(topic5);
  client.subscribe(topic6);
  client.subscribe(topic7);
  client.subscribe(topic8);
  client.subscribe(topic9);
  client.subscribe(topic15);
  client.subscribe(topic16);
  client.subscribe(topic17);
}
void calibrateSensors() {
  if (calibrationCount < calibrationSamples) {
    calibrationSum1 += analogRead(ACS712Pin1);
    calibrationSum2 += analogRead(ACS712Pin2);
    calibrationSum3 += analogRead(ACS712Pin3);
    calibrationSum4 += analogRead(ACS712Pin4); 
    calibrationCount++;
  } else {
    offsetVoltage1 = (calibrationSum1 / (float)calibrationSamples) * (5.0 / 1023.0);
    offsetVoltage2 = (calibrationSum2 / (float)calibrationSamples) * (5.0 / 1023.0);
    offsetVoltage3 = (calibrationSum3 / (float)calibrationSamples) * (5.0 / 1023.0);
    offsetVoltage4 = (calibrationSum4 / (float)calibrationSamples) * (5.0 / 1023.0);

    calibrating = false; 
  }
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

  float voltage = (sum / (float)numSamples) * (5.0 / 1023.0);
  float current = abs((voltage - offsetVoltage) / sensitivity) / 3.2;
  return current;
}
String readUltrasonicSensor1() {
  digitalWrite(TRIG_PIN1,0); 
  delayMicroseconds(2); 
  digitalWrite(TRIG_PIN1,1); 
  delayMicroseconds(10); 
  digitalWrite(TRIG_PIN1,0);

  duration1 = pulseIn (ECHO_PIN1, HIGH);
  
   distance1 = (duration1 / 2 / 29.412); 
  return String(distance1);
}
String readUltrasonicSensor2() {
  digitalWrite(TRIG_PIN2,0); 
  delayMicroseconds(2); 
  digitalWrite(TRIG_PIN2,1);
  delayMicroseconds(10); 
  digitalWrite(TRIG_PIN2,0);

  duration2 = pulseIn (ECHO_PIN2, HIGH);
  
   distance2 = (duration2 / 2 / 29.412); 
  return String(distance2);
}
String foodrate (){
  float foodAvailable = (1.0- (distance1/distanceOrigin))*100.0;
  return String(foodAvailable);
}
String waterrate (){
  float waterAvailable = (1.0- (distance2/distanceOrigin))*100.0;
  return String(waterAvailable);
}
void readSensors() {
  unsigned long currentMillis = millis();
    if (currentMillis - previousSensorMillis >= updateSensorInterval) 
    {
        previousSensorMillis = currentMillis;
        temperature = dht.readTemperature();
        humidity = dht.readHumidity();
        airQuality = analogRead(MQ135_PIN);
        String ultrasonicDistance1 = readUltrasonicSensor1();
        rate1 = foodrate();
        String ultrasonicDistance2 = readUltrasonicSensor2();
        rate2 = waterrate();
        current1 = readCurrent(ACS712Pin1, offsetVoltage1);
        current2 = readCurrent(ACS712Pin2, offsetVoltage2);
        current3 = readCurrent(ACS712Pin3, offsetVoltage3);
        current4 = readCurrent(ACS712Pin4, offsetVoltage4); 
    }
}
void publishDeviceStatus() { 

  readSensors();
  String payload1 = "Fan active: " + String(current1, 3) + " A";
  String payload2 = "Pump active: " + String(current2, 3) + " A";
  String payload3 = "Pump2 active: " + String(current3, 3) + " A";
  String payload4 = "Bulb active: " + String(current4, 3) + " A";
  String payload5 = "Fan unactive: " + String(current1, 3) + " A";
  String payload6 = "Pump unactive: " + String(current2, 3) + " A";
  String payload7 = "Pump2 unactive: " + String(current3, 3) + " A";
  String payload8 = "Bulb unactive: " + String(current3, 3) + " A";
  client.publish(topic0, String(temperature).c_str());
  client.publish(topic1, String(humidity).c_str());
  client.publish(topic2, String(airQuality).c_str());
  client.publish(topic3, String(rate1).c_str());
  client.publish(topic4, isFanOn ? "ON" : "OFF");
  client.publish(topic5, isPumpOn ? "ON" : "OFF");
  client.publish(topic6, currentMode == MANUAL ? "Manual" : "Automatic");
  client.publish(topic7, isServoAt90 ? "ON" : "OFF");
  client.publish(topic8, isBulbOn ? "ON" : "OFF");
  client.publish(topic9, isPump2On ? "ON" : "OFF");
  if (current1 > 0.1) {
  client.publish(topic10,payload1.c_str());
  } else {
  client.publish(topic10,payload5.c_str());
  };
  if (current2 > 0.1) {
  client.publish(topic11,payload2.c_str());
  } else {
  client.publish(topic11,payload6.c_str());
  };
  if (current3 > 0.1) {
  client.publish(topic12,payload3.c_str());
  } else {
  client.publish(topic12,payload7.c_str());
  };
  if (current4 > 0.1) {
  client.publish(topic13,payload4.c_str());
  } else {
  client.publish(topic13,payload8.c_str());
  };
  client.publish(topic14, String(rate2).c_str());
}
void handleServoControl() {
  unsigned long currentMillis = millis(); 

  int currentServoButtonState = pcf8574.read(4); 
  if (currentServoButtonState == LOW && lastServoButtonState == HIGH && (currentMillis - previousMillisServo >= debounceInterval)) {
    previousMillisServo = currentMillis;
    isServoAt90 = !isServoAt90;
    if (isServoAt90) {
      myServo.write(90);  
    } else {
      myServo.write(0);   
    }
    client.publish(topic7, isServoAt90 ? "ON" : "OFF");
  }
  lastServoButtonState = currentServoButtonState;
}
void handleFanControl() {
  unsigned long currentMillis = millis();  

  switch (currentMode) {
    case MANUAL: {
      int fanButtonState = pcf8574.read(0); 
      if (fanButtonState == LOW && lastFanButtonState == HIGH && (currentMillis - previousMillisFan >= debounceInterval)) {
        previousMillisFan = currentMillis;
        isFanOn = !isFanOn;  
        digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);  
        publishDeviceStatus();
      }
      lastFanButtonState = fanButtonState; 
      break;
    }
    case AUTOMATIC: {
      if (temperature > 32.0) {
        isFanOn = true;  
      } else {
        isFanOn = false; 
      }
      digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);  
      publishDeviceStatus();
      break;
    }
  }

  int modeButtonState = pcf8574.read(5); 
  if (modeButtonState == LOW && lastModeButtonState == HIGH && (currentMillis - previousMillisMode >= debounceInterval)) {
    previousMillisMode = currentMillis;
    currentMode = (currentMode == MANUAL) ? AUTOMATIC : MANUAL; 
    publishDeviceStatus();
  }
  lastModeButtonState = modeButtonState; 
}
void handlePumpControl() {
  unsigned long currentMillis = millis();  

  bool PumpButtonState = pcf8574.read(1);  
  if (PumpButtonState == LOW && lastPumpButtonState == HIGH && (currentMillis - previousMillisPump >= debounceInterval)) {
    previousMillisPump = currentMillis;
    isPumpOn = !isPumpOn;  
    digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);  
  }

  lastPumpButtonState = PumpButtonState;  
}
void handlePump2Control() {
  unsigned long currentMillis = millis();  

  switch (currentMode) {
    case MANUAL: {
      int Pump2ButtonState = pcf8574.read(2); 
      if (Pump2ButtonState == LOW && lastPump2ButtonState == HIGH && (currentMillis - previousMillisPump2 >= debounceInterval)) {
        previousMillisPump2 = currentMillis;
        isPump2On = !isPump2On;  
        digitalWrite(PUMP2_RELAY_PIN, isPump2On ? HIGH : LOW);  
        publishDeviceStatus();  
      }
      lastPump2ButtonState = Pump2ButtonState;
      break;
    }

    case AUTOMATIC: {
      if (airQuality > 900) {
        isPump2On = true;  
      } else {
        isPump2On = false; 
      }
      digitalWrite(PUMP2_RELAY_PIN, isPump2On ? HIGH : LOW);  
      publishDeviceStatus();  
      break;
    }
  }

  int modeButtonState = pcf8574.read(5); 
  if (modeButtonState == LOW && lastModeButtonState == HIGH && (currentMillis - previousMillisMode >= debounceInterval)) {
    previousMillisMode = currentMillis;
    currentMode = (currentMode == MANUAL) ? AUTOMATIC : MANUAL;  
    publishDeviceStatus();  
  }
  lastModeButtonState = modeButtonState; 
}
void handleBulbControl() {
  unsigned long currentMillis = millis();  

  int BulbButtonState = pcf8574.read(3);
  if (BulbButtonState == LOW && lastBulbButtonState == HIGH && (currentMillis - previousMillisBulb >= debounceInterval)) {
    previousMillisBulb = currentMillis;
    isBulbOn = !isBulbOn;
    digitalWrite(BULB_RELAY_PIN, isBulbOn ? HIGH : LOW);
  }

  lastBulbButtonState = BulbButtonState;
}
void updateStatusDisplay() {
    readSensors();
    automaticfeeding(); 
    char timeStringBuff[10];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S", &timeinfo); 
    unsigned long currentMillis = millis();
    
    int OledButtonState = pcf8574.read(6);
    if (OledButtonState == LOW && lastOledButtonState == HIGH && (currentMillis - previousMillisOled >= debounceInterval)) {
        previousMillisOled = currentMillis;
        currentScreen = (currentScreen + 1) % 3;  
    }
    lastOledButtonState = OledButtonState;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    
    switch (currentScreen) {
        case 0: 
            display.print(F("Time: "));
            display.println(timeStringBuff);

            display.print(F("Temp: "));
            display.print(temperature);
            display.println(F(" C"));

            display.print(F("Humidity: "));
            display.print(humidity);
            display.println(F(" %"));

            display.print(F("Air Quality: "));
            display.println(airQuality);

            display.print(F("Food Rate: "));
            display.print(rate1);
            display.println(F("%"));

            display.print(F("Water Rate: "));
            display.print(rate2);
            display.println(F("%"));
            break;

        case 1: 
            display.println(F("Device Status:"));
                      
            display.print(F("Fan: "));    
            display.println(isFanOn ? F("ON") : F("OFF"));

            display.print(F("Fan mode: "));
            display.println(currentMode == AUTOMATIC ? F("AUTOMATIC") : F("MANUAL"));

            display.print(F("Pump: "));
            display.println(isPumpOn ? F("ON") : F("OFF"));

            display.print(F("Pump2: "));
            display.println(isPump2On ? F("ON") : F("OFF"));

            display.print(F("Bulb: "));
            display.println(isBulbOn ? F("ON") : F("OFF"));

            display.print(F("Servo: "));
            display.println(isServoAt90 ? F("ON") : F("OFF"));
            break;

        case 2:  
            if (current1 > 0.1)
             {
            display.print(F("Fan active: "));
            display.print(String(current1, 3));
            display.println(F(" A"));
             }
             else 
             {
            display.print(F("Fan unactive: "));
            display.print(String(current1, 3));
            display.println(F(" A"));
             };
             if (current2 > 0.1)
             {
            display.print(F("Pump active: "));
            display.print(String(current2, 3));
            display.println(F(" A"));
             }
             else
             {
            display.print(F("Pump unactive: "));
            display.print(String(current2, 3));
            display.println(F(" A"));
             };
             if (current3 > 0.1)
             {
            display.print(F("Pump2 active: "));
            display.print(String(current3, 3));
            display.println(F(" A"));
             }
             else
             {
            display.print(F("Pump2 unactive: "));
            display.print(String(current3, 3));
            display.println(F(" A"));
             };
             if (current4 > 0.1)
             {
            display.print(F("Bulb active: "));
            display.print(String(current4, 3));
            display.println(F(" A"));
             }
              else
            {
            display.print(F("Bulb unactive: "));
            display.print(String(current4, 3));
            display.println(F(" A"));
            };
                
            break;
        default:
            break;
    }
    display.display();
}
void loop() {
 readSensors();
 handleFanControl();
 handlePumpControl();
 handlePump2Control();
 handleBulbControl();
 handleServoControl();
 updateStatusDisplay();
 if (calibrating) {
  calibrateSensors();
 }
 if (connectWiFi()) {
  connectMQTT();
  }
  if (client.connected()) {
  publishDeviceStatus();
  client.loop();
  }
}
