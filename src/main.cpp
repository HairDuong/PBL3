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
const char *ssid PROGMEM = "Wifi HauDepTrai";
const char *password PROGMEM = "hoang123";
const char *mqtt_broker PROGMEM = "broker.emqx.io";
const char *topic0 PROGMEM = "esp32/tem";
const char *topic1 PROGMEM = "esp32/hum";
const char *topic2 PROGMEM = "esp32/air";
const char *topic3 PROGMEM = "esp32/foodrate";
const char *topic4 PROGMEM = "esp32/quat/control";
const char *topic5 PROGMEM = "esp32/maybom/control";
const char *topic6 PROGMEM = "esp32/fan/mode";
const char *topic7 PROGMEM = "esp32/servo/control";
const char *topic8 PROGMEM = "esp32/den/control";
const char *topic9 PROGMEM = "esp32/maybom2/control";
const char *topic10 PROGMEM = "esp32/quat/test";
const char *topic11 PROGMEM = "esp32/maybom1/test";
const char *topic12 PROGMEM = "esp32/maybom2/test";
const char *topic13 PROGMEM = "esp32//test";
const char *topic14 PROGMEM = "esp32/waterrate";
const char *topic15 PROGMEM = "targetHour";
const char *topic16 PROGMEM = "targetMinute";
const char *topic17 PROGMEM = "targetSecond";
const char *mqtt_username PROGMEM = "hoangpham1";
const char *mqtt_password PROGMEM = "123456";
const int mqtt_port PROGMEM = 1883;

// Pin definitions
#define servoPin 15
#define TRIG_PIN2 18
#define ECHO_PIN2 19
#define FAN_BUTTON_PIN 0
#define FAN_RELAY_PIN 26
#define PUMP_BUTTON_PIN 1
#define PUMP_RELAY_PIN 27
#define PUMP2_BUTTON_PIN 2
#define PUMP2_RELAY_PIN 14
#define MODE_BUTTON_PIN 5
#define BULB_RELAY_PIN 12
#define BULB_BUTTON_PIN 3
#define SERVO_BUTTON_PIN 4 
#define OLED_BUTTON_PIN 6 
 

// Global variables
float temperature = 0.0;
float humidity = 0.0;
int airQuality = 0;
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

unsigned long previousRecevie=0;
unsigned long previousMillisPump = 0; 
unsigned long previousMillisPump2 = 0;
unsigned long previousMillisServo = 0;
unsigned long previousMillisFan = 0;
unsigned long previousMillisBulb = 0;
unsigned long previousMillisMode = 0;
unsigned long previousMillisOled = 0;
unsigned long previousMillispublish = 0;
const long debounceInterval1 = 50;  
const long debounceInterval2 = 50;  
const long debounceInterval3 = 50;  
const long debounceInterval4 = 50;  
const long debounceInterval5 = 50;  
const long debounceInterval6 = 50;  
const long debounceInterval7 = 50;  
const long debounceInterval8 = 50;  
unsigned long previousMillis = 0;
const long interval = 5000;
float zeroPointVoltage1 = 2.5; 
unsigned long previousMillisWiFi = 0;
unsigned long previousMillisMQTT = 0;
const long wifiInterval = 500;  
const long mqttInterval = 2000; 
const long publishInterval = 2000; 
float current1;
float current2;
float current3;
float current4;
float offsetVoltage1 = 0.0;      
float offsetVoltage2 = 0.0;      
float offsetVoltage3 = 0.0;      
float offsetVoltage4 = 0.0;     


WiFiClient espClient;
PubSubClient client(espClient);
Servo myServo;

enum ControlMode { MANUAL, AUTOMATIC };
ControlMode currentMode = MANUAL;

// OLED Display 
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire, -1);
unsigned long previousSensorMillis = 0; 
const unsigned long updateSensorInterval = 2000; 
bool showSensorData = true;



void callback(char *topic, byte *payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += (char)payload[i];

  if (String(topic) == topic4) isFanOn = (message == "ON");
  if (String(topic) == topic5) isPumpOn = (message == "ON");
   if (String(topic) == topic6) currentMode = (message == "MANUAL") ? MANUAL : AUTOMATIC;
  if (String(topic) == topic7) myServo.write(message == "ON" ? 90 : 0);
  if (String(topic) == topic8) isBulbOn = (message == "ON");
  if (String(topic) == topic9) isPump2On = (message == "ON");
  if (String(topic) == topic15) targetHour = message.toInt() ;
  if (String(topic) == topic16) targetMinute = message.toInt() ;
  if (String(topic) == topic17) 
  {targetSecond = message.toInt() ;
   targetSecondclose = targetSecond +5;}
  
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
    if (myServo.read() != 0) 
    {
      myServo.write(0);
    }
  }
     if (timeinfo.tm_hour == targetHour && timeinfo.tm_min == targetMinute && timeinfo.tm_sec == (targetSecondclose)) {
     {
      myServo.write(90);  
     }
  }
}
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pcf8574.begin();
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT_PULLDOWN);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); 

 
  myServo.attach(servoPin);
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");
  myServo.write(90);
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
  while (WiFi.status() != WL_CONNECTED) {
     unsigned long currentMillis = millis();
    if (currentMillis - previousMillisWiFi >= wifiInterval) {
      previousMillisWiFi = currentMillis;
      Serial.println("Connecting to WiFi...");
      WiFi.begin(ssid, password);
      Serial.println("Connected to the WiFi network");
    }
  }
  Serial.println("Connected to the WiFi network");
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisMQTT >= mqttInterval) {
      previousMillisMQTT = currentMillis;
       String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public emqx mqtt broker connected");
      }
  }
  }
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
String waterrate (){
  float waterAvailable = (1.0- (distance2/distanceOrigin))*100.0;
  return String(waterAvailable);
}
void receive()
{
    unsigned long currentMillis = millis();
     if (currentMillis - previousRecevie >= 100) {
         previousRecevie = currentMillis;
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');  // Đọc dữ liệu từ UART
    Serial.print("Dữ liệu nhận được: ");
    Serial.println(data);

    // Nếu bạn muốn tách từng giá trị dòng điện từ dữ liệu
    //float current1, current2, current3, current4;
    int commaIndex1 = data.indexOf(',');
    int commaIndex2 = data.indexOf(',', commaIndex1 + 1);
    int commaIndex3 = data.indexOf(',', commaIndex2 + 1);
    int commaIndex4 = data.indexOf(',', commaIndex3 + 1);
    int commaIndex5 = data.indexOf(',', commaIndex4 + 1);
    int commaIndex6 = data.indexOf(',', commaIndex5 + 1);
    int commaIndex7 = data.indexOf(',', commaIndex6 + 1);
    int commaIndex8 = data.indexOf(',', commaIndex7 + 1);
    int commaIndex9 = data.indexOf(',', commaIndex8 +1);
    int commaIndex10 = data.indexOf(',', commaIndex9 + 1);
    int commaIndex11 = data.indexOf(',', commaIndex10 + 1);
    int commaIndex12 = data.indexOf(',', commaIndex11 +1);
    

    current1 = data.substring(1, commaIndex1).toFloat();
    current2 = data.substring(commaIndex1 + 1, commaIndex2).toFloat();
    current3 = data.substring(commaIndex2 + 1, commaIndex3).toFloat();
    current4 = data.substring(commaIndex3 + 1,commaIndex4).toFloat();
    rate1    = data.substring(commaIndex4 + 1,commaIndex5);
    offsetVoltage1    = data.substring(commaIndex5 + 1,commaIndex6).toFloat();
    offsetVoltage2    = data.substring(commaIndex6 + 1, commaIndex7).toFloat();
    offsetVoltage3    = data.substring(commaIndex7 + 1,commaIndex8).toFloat();
    offsetVoltage4    = data.substring(commaIndex8 + 1,commaIndex9).toFloat();
    temperature    = data.substring(commaIndex9 + 1, commaIndex10).toFloat();
    humidity    = data.substring(commaIndex10 + 1,commaIndex11).toFloat();
    airQuality    = data.substring(commaIndex11 + 1,commaIndex12).toFloat();
  }
   
  }
}
void readSensors() {
  unsigned long currentMillis = millis();
    if (currentMillis - previousSensorMillis >= updateSensorInterval) {
        previousSensorMillis = currentMillis;
            
        String ultrasonicDistance2 = readUltrasonicSensor2();
        rate2 = waterrate();
    }
}
void publishDeviceStatus() { 

  readSensors();
  unsigned long currentMillis = millis(); 
   if (currentMillis - previousMillispublish >= publishInterval) {
      previousMillispublish = currentMillis;

  client.publish(topic0, String(temperature).c_str());
  client.publish(topic1, String(humidity).c_str());
  client.publish(topic2, String(airQuality).c_str());
  client.publish(topic3, String(rate1).c_str());
  client.publish(topic10, current1 > 0.1 ? "Active" : "Unactive");
  client.publish(topic11, current2 > 0.1 ? "Active" : "Unactive");
  client.publish(topic12, current3 > 0.1 ? "Active" : "Unactive");
  client.publish(topic13, current4 > 0.1 ? "Active" : "Unactive");
  client.publish(topic14, String(rate2).c_str());
}
}
void handleServoControl() {
  unsigned long currentMillis = millis(); 

  int currentServoButtonState = pcf8574.read(4); 
  if (currentServoButtonState == LOW && lastServoButtonState == HIGH && (currentMillis - previousMillisServo >= debounceInterval1)) {
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

  if (currentMode == MANUAL) {
    int fanButtonState = pcf8574.read(0); 
    if (fanButtonState == LOW && lastFanButtonState == HIGH && (currentMillis - previousMillisFan >= debounceInterval2)) {
      previousMillisFan = currentMillis;
      isFanOn = !isFanOn;  
      digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);  
      client.publish(topic4, isFanOn ? "ON" : "OFF");
    }
    lastFanButtonState = fanButtonState;

  } else if (currentMode == AUTOMATIC) {
    bool newFanState = temperature > 32.0;
    if (isFanOn != newFanState) {
      isFanOn = newFanState;
      digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);
      client.publish(topic4, isFanOn ? "ON" : "OFF");
    }
  }

  int modeButtonState = pcf8574.read(5);
  if (modeButtonState == LOW && lastModeButtonState == HIGH && (currentMillis - previousMillisMode >= debounceInterval3)) {
    previousMillisMode = currentMillis;
    currentMode = (currentMode == MANUAL) ? AUTOMATIC : MANUAL;
    client.publish(topic6, currentMode == MANUAL ? "MANUAL" : "AUTOMATIC");

    if (currentMode == AUTOMATIC) {
      isFanOn = (temperature > 32.0);
    }
    digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);
    client.publish(topic4, isFanOn ? "ON" : "OFF");
  }
  lastModeButtonState = modeButtonState;
}
void handlePumpControl() {
  unsigned long currentMillis = millis();  

  bool PumpButtonState = pcf8574.read(1);  
  if (PumpButtonState == LOW && lastPumpButtonState == HIGH && (currentMillis - previousMillisPump >= debounceInterval4)) {
    previousMillisPump = currentMillis;
    isPumpOn = !isPumpOn;
    digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);  
    client.publish(topic5, isPumpOn ? "ON" : "OFF");  
  }

  lastPumpButtonState = PumpButtonState;  
}
void handlePump2Control() {
  unsigned long currentMillis = millis();

  if (currentMode == MANUAL) {
    int pump2ButtonState = pcf8574.read(2);
    if (pump2ButtonState == LOW && lastPump2ButtonState == HIGH && (currentMillis - previousMillisPump2 >= debounceInterval5)) {
      previousMillisPump2 = currentMillis;
      isPump2On = !isPump2On;
      digitalWrite(PUMP2_RELAY_PIN, isPump2On ? HIGH : LOW);
      client.publish(topic9, isPump2On ? "ON" : "OFF");
    }
    lastPump2ButtonState = pump2ButtonState;
  } else if (currentMode == AUTOMATIC) {
    if (airQuality > 3000) {
      isPump2On = true;
    } else {
      isPump2On = false;
    }
    digitalWrite(PUMP2_RELAY_PIN, isPump2On ? HIGH : LOW);
    client.publish(topic9, isPump2On ? "ON" : "OFF");
  }

  int modeButtonState = pcf8574.read(5);
  if (modeButtonState == LOW && lastModeButtonState == HIGH && (currentMillis - previousMillisMode >= debounceInterval6)) {
  previousMillisMode = currentMillis;
  currentMode = (currentMode == MANUAL) ? AUTOMATIC : MANUAL;
  

  if (currentMode == MANUAL) {
    digitalWrite(PUMP2_RELAY_PIN, isPump2On ? HIGH : LOW);
  } else if (currentMode == AUTOMATIC) {
    isPump2On = (airQuality > 3000);
    digitalWrite(PUMP2_RELAY_PIN, isPump2On ? HIGH : LOW);
  }
  client.publish(topic9, isPump2On ? "ON" : "OFF");
 }

  lastModeButtonState = modeButtonState;
}

void handleBulbControl() {
  unsigned long currentMillis = millis();  

  int BulbButtonState = pcf8574.read(3);
  if (BulbButtonState == LOW && lastBulbButtonState == HIGH && (currentMillis - previousMillisBulb >= debounceInterval7)) {
    previousMillisBulb = currentMillis;
    isBulbOn = !isBulbOn;
    digitalWrite(BULB_RELAY_PIN, isBulbOn ? HIGH : LOW);
    client.publish(topic8, isBulbOn ? "ON" : "OFF");
  }

  lastBulbButtonState = BulbButtonState;
}
void updateStatusDisplay() {
    readSensors();
    receive();
    automaticfeeding(); 
    char timeStringBuff[10];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S", &timeinfo); 
    unsigned long currentMillis = millis();
   
    int OledButtonState = pcf8574.read(6);
    if (OledButtonState == LOW && lastOledButtonState == HIGH && (currentMillis - previousMillisOled >= debounceInterval8)) {
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
            display.print(F(" A - "));
            display.print(String(offsetVoltage1, 2));
            display.println(F("V"));
             }
             else 
             {
            display.print(F("Fan unactive: "));
            display.print(String(current1, 3));
            display.print(F(" A - "));
            display.print(String(offsetVoltage1, 2));
            display.println(F("V"));
             };
             if (current2 > 0.1)
             {
            display.print(F("Pump active: "));
            display.print(String(current2, 3));
            display.print(F(" A - "));
            display.print(String(offsetVoltage2, 2));
            display.println(F("V"));
             }
             else
             {
            display.print(F("Pump unactive: "));
            display.print(String(current2, 3));
            display.print(F(" A - "));
            display.print(String(offsetVoltage2, 2));
            display.println(F("V"));
             };
             if (current3 > 0.1)
             {
            display.print(F("Pump2 active: "));
            display.print(String(current3, 3));
            display.println(F(" A"));
            display.print(String(offsetVoltage3, 2));
            display.println(F("V"));
             }
             else
             {
            display.print(F("Pump2 unactive: "));
            display.print(String(current3, 3));
            display.print(F(" A - "));
            display.print(String(offsetVoltage3, 2));
            display.println(F("V"));
             };
             if (current4 > 0.1)
             {
            display.print(F("Bulb active: "));
            display.print(String(current4, 3));
            display.print(F(" A - "));
            display.print(String(offsetVoltage4, 2));
            display.println(F("V"));
             }
              else
            {
            display.print(F("Bulb unactive: "));
            display.print(String(current4, 3));
            display.print(F(" A - "));
            display.print(String(offsetVoltage4, 2));
            display.println(F("V"));
            };
                
            break;
        default:
            break;
    }
    display.display();
}
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

void loop() {
  if (connectWiFi()) {
  connectMQTT();
  }
  receive();
  handleFanControl();
  handlePumpControl();
  handlePump2Control();
  handleBulbControl();
  handleServoControl();
  updateStatusDisplay();
  publishDeviceStatus();
  client.loop();
 

  
}