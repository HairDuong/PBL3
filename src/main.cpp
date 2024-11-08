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
const char *ssid = "CAFE DU BIEN";
const char *password = "dubien123";
const char *mqtt_broker = "broker.emqx.io";
const char *topic0 = "esp32/temp";
const char *topic1 = "esp32/hum";
const char *topic2 = "esp32/air";
const char *topic3 = "esp32/foodrate";
const char *topic4 = "esp32/quat/control";
const char *topic5 = "esp32/maybom/control";
const char *topic6 = "esp32/fan/mode";
const char *topic7 = "esp32/servo/control";
const char *topic8 = "esp32/bongden/control";
const char *topic9 = "esp32/maybom2/control";
const char *topic10 = "targetHour";
const char *topic11 = "targetMinute";
const char *topic12 = "targetSecond";
const char *mqtt_username = "hoangpham1";
const char *mqtt_password = "123456";
const int mqtt_port = 1883;

// Pin definitions
#define servoPin 17
#define DHTPIN 5
#define DHTTYPE DHT11
#define MQ135_PIN 34
#define TRIG_PIN 19
#define ECHO_PIN 18
#define FAN_BUTTON_PIN 0
#define FAN_RELAY_PIN 27
#define PUMP_BUTTON_PIN 1
#define PUMP_RELAY_PIN 25
#define PUMP2_BUTTON_PIN 2
#define PUMP2_RELAY_PIN 32
#define MODE_BUTTON_PIN 5
#define BULB_RELAY_PIN 33
#define BULB_BUTTON_PIN 15
#define SERVO_BUTTON_PIN 4 
#define OLED_BUTTON_PIN 6 

// Global variables
float temperature = 0.0;
float humidity = 0.0;
int airQuality = 0;
float distance = 0.0;
long duration;
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

unsigned long previousMillisPump = 0; 
unsigned long previousMillisPump2 = 0;
unsigned long previousMillisServo = 0;
unsigned long previousMillisFan = 0;
unsigned long previousMillisBulb = 0;
unsigned long previousMillisMode = 0;
unsigned long previousMillisOled = 0;
const long debounceInterval = 50;  


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
void callback(char *topic, byte *payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += (char)payload[i];

  if (String(topic) == topic4) isFanOn = (message == "ON");
  if (String(topic) == topic5) isPumpOn = (message == "ON");
  if (String(topic) == topic7) myServo.write(message == "ON" ? 90 : 0);
  if (String(topic) == topic8) isBulbOn = (message == "ON");
  if (String(topic) == topic9) isPump2On = (message == "ON");
  if (String(topic) == topic10) targetHour = message.toInt() ;
  if (String(topic) == topic11) targetMinute = message.toInt() ;
  if (String(topic) == topic12) 
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
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLDOWN);
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
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public emqx mqtt broker connected");
    } else {
      delay(2000);
    }
  }
  client.subscribe(topic4);
  client.subscribe(topic5);
  client.subscribe(topic7);
  client.subscribe(topic8);
  client.subscribe(topic9);
  client.subscribe(topic10);
  client.subscribe(topic11);
  client.subscribe(topic12);
}
String readUltrasonicSensor() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return "Error";
  distance = (duration * 0.034) / 2;
  return String(distance);
}
void handleServoControl() {
  unsigned long currentMillis = millis();  // Get current time

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

String ratefood (){
  float distanceOrigin = 8.0;
  float foodAvailable = (1.0- (distance/distanceOrigin))*100.0;
  return String(foodAvailable);
}

void publishDeviceStatus() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousSensorMillis >= updateSensorInterval) {
        previousSensorMillis = currentMillis;
        temperature = dht.readTemperature();
        humidity = dht.readHumidity();
        airQuality = analogRead(MQ135_PIN);
        String ultrasonicDistance = readUltrasonicSensor();
        String rate = ratefood();
        
        client.publish(topic0, String(temperature).c_str());
        client.publish(topic1, String(humidity).c_str());
        client.publish(topic2, String(airQuality).c_str());
        client.publish(topic3, rate.c_str());
        client.publish(topic4, isFanOn ? "ON" : "OFF");
        client.publish(topic5, isPumpOn ? "ON" : "OFF");
        client.publish(topic6, currentMode == MANUAL ? "Manual" : "Automatic");
        client.publish(topic7, isServoAt90 ? "ON" : "OFF");
        client.publish(topic8, isBulbOn ? "ON" : "OFF");
        client.publish(topic9, isPump2On ? "ON" : "OFF");
    }
}
void handleFanControl() {
  unsigned long currentMillis = millis();  // Get current time

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
  unsigned long currentMillis = millis();  // Lấy thời gian hiện tại

  bool PumpButtonState = pcf8574.read(1);  
  if (PumpButtonState == LOW && lastPumpButtonState == HIGH && (currentMillis - previousMillisPump >= debounceInterval)) {
    previousMillisPump = currentMillis;
    isPumpOn = !isPumpOn;  
    digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);  
  }

  lastPumpButtonState = PumpButtonState;  
}
void handlePump2Control() {
  unsigned long currentMillis = millis();  // Get current time

  int Pump2ButtonState = pcf8574.read(2);
  if (Pump2ButtonState == LOW && lastPump2ButtonState == HIGH && (currentMillis - previousMillisPump2 >= debounceInterval)) {
    previousMillisPump2 = currentMillis;
    isPump2On = !isPump2On;
    digitalWrite(PUMP2_RELAY_PIN, isPump2On ? HIGH : LOW);
  }

  lastPump2ButtonState = Pump2ButtonState;
}
void handleBulbControl() {
  unsigned long currentMillis = millis();  // Get current time

  int BulbButtonState = pcf8574.read(3);
  if (BulbButtonState == LOW && lastBulbButtonState == HIGH && (currentMillis - previousMillisBulb >= debounceInterval)) {
    previousMillisBulb = currentMillis;
    isBulbOn = !isBulbOn;
    digitalWrite(BULB_RELAY_PIN, isBulbOn ? HIGH : LOW);
  }

  lastBulbButtonState = BulbButtonState;
}

void updateStatusDisplay() {
    automaticfeeding(); 
    char timeStringBuff[10];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S", &timeinfo); 
    unsigned long currentMillis = millis();
    
    int OledButtonState = pcf8574.read(6);
    if (OledButtonState == LOW && lastOledButtonState == HIGH && (currentMillis - previousMillisOled >= debounceInterval)) {
        previousMillisOled = currentMillis;
        showSensorData = !showSensorData;  
    }
    lastOledButtonState = OledButtonState;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    if (showSensorData) {
        display.println("Time: " + String(timeStringBuff));  
        display.println("Sensor Data:");                   
        display.println("Temp: " + String(temperature) + " C");  
        display.println("Humidity: " + String(humidity) + " %"); 
        display.println("Air Quality: " + String(airQuality));  
        display.println("Food: " + ratefood() + " %");           
    } 
    else {
        display.println("Device Status:");                       
        display.println(isFanOn ? "Fan: ON" : "Fan: OFF");       
        display.println(currentMode == AUTOMATIC ? "Fan mode: AUTOMATIC" : "Fan mode: MANUAL"); 
        display.println(isPumpOn ? "Pump: ON" : "Pump: OFF");    
        display.println(isPump2On ? "Pump2: ON" : "Pump2: OFF");    
        display.println(isBulbOn ? "Bulb: ON" : "Bulb: OFF");    
        display.println(isServoAt90 ? "Servo: ON" : "Servo: OFF"); 
    }
    display.display();  
}
void loop() {
  handleFanControl();
  handlePumpControl();
  handlePump2Control();
  handleBulbControl();
  handleServoControl();
  updateStatusDisplay();
  publishDeviceStatus();
  Serial.print(targetSecondclose);
  client.loop();
}