#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <DHT.h>
#include<ESP32Servo.h>
#include <time.h>

// Configuration for NTP Time
const long gmtOffset_sec = 7 * 3600;   
const int daylightOffset_sec = 0;

// Wi-Fi and MQTT settings
const char *ssid = "Eduy";
const char *password = "11111111";
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
#define FAN_BUTTON_PIN 23
#define FAN_RELAY_PIN 27
#define PUMP_BUTTON_PIN 4
#define PUMP_RELAY_PIN 25
#define MODE_BUTTON_PIN 12
#define BULB_RELAY_PIN 33
#define BULB_BUTTON_PIN 15
#define SERVO_BUTTON_PIN 14  

// Global variables
float temperature = 0.0;
float humidity = 0.0;
int airQuality = 0;
float distance = 0.0;
long duration;
bool isFanOn = false;
int lastFanButtonState = HIGH; 
bool isPumpOn = false;
int lastPumpButtonState = HIGH;
bool isBulbOn = false;
int lastBulbButtonState = HIGH; 
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


WiFiClient espClient;
PubSubClient client(espClient);
Servo myServo;
DHT dht(DHTPIN, DHTTYPE);
enum ControlMode { MANUAL, AUTOMATIC };
ControlMode currentMode = MANUAL;

// OLED Display 
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire, -1);
unsigned long previousSensorMillis = 0; 
unsigned long previousDisplayMillis = 0; 
const unsigned long updateSensorInterval = 2000; 
const unsigned long displayInterval = 5000; 
bool showSensorData = true;
void callback(char *topic, byte *payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += (char)payload[i];

  if (String(topic) == topic4) isFanOn = (message == "ON");
  if (String(topic) == topic5) isPumpOn = (message == "ON");
  if (String(topic) == topic7) myServo.write(message == "ON" ? 90 : 0);
  if (String(topic) == topic8) isBulbOn = (message == "ON");

  digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);
  digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);
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
      myServo.write(0);  // Xoay servo về góc 0 độ
     }
  }
}
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLDOWN);
  dht.begin();
  pinMode(SERVO_BUTTON_PIN, INPUT_PULLUP); 
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

  pinMode(FAN_BUTTON_PIN, INPUT_PULLUP); 
  pinMode(FAN_RELAY_PIN, OUTPUT);        
  digitalWrite(FAN_RELAY_PIN, LOW);      

  pinMode(PUMP_BUTTON_PIN, INPUT_PULLUP); 
  pinMode(PUMP_RELAY_PIN, OUTPUT);        
  digitalWrite(PUMP_RELAY_PIN, LOW);    

  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);  

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
  int currentServoButtonState = digitalRead(SERVO_BUTTON_PIN); 
  if (currentServoButtonState == LOW && lastServoButtonState == HIGH) {
    delay(50);
    if (digitalRead(SERVO_BUTTON_PIN) == LOW) { 
      isServoAt90 = !isServoAt90;
      if (isServoAt90) {
        myServo.write(90);  
      } else {
        myServo.write(0);   
      }
      client.publish(topic7, isServoAt90 ? "ON" : "OFF");
    }
  }
  lastServoButtonState = currentServoButtonState;
}
void publishDeviceStatus() {
    unsigned long currentMillis = millis();

    // Kiểm tra nếu đã qua khoảng thời gian cần để cập nhật dữ liệu cảm biến lên MQTT
    if (currentMillis - previousSensorMillis >= updateSensorInterval) {
        previousSensorMillis = currentMillis;

        // Đọc dữ liệu từ các cảm biến
        temperature = dht.readTemperature();
        humidity = dht.readHumidity();
        airQuality = analogRead(MQ135_PIN);
        String ultrasonicDistance = readUltrasonicSensor();
        String rate = ratefood();
        
        // Gửi dữ liệu cảm biến lên các topic MQTT
        client.publish(topic0, String(temperature).c_str());
        client.publish(topic1, String(humidity).c_str());
        client.publish(topic2, String(airQuality).c_str());
        client.publish(topic3, rate.c_str());

        Serial.println("Air Quality: " + String(airQuality));  

        // Gửi trạng thái của các thiết bị lên MQTT
        client.publish(topic4, isFanOn ? "ON" : "OFF");
        client.publish(topic5, isPumpOn ? "ON" : "OFF");
        client.publish(topic6, currentMode == MANUAL ? "Manual" : "Automatic");
        client.publish(topic7, isServoAt90 ? "ON" : "OFF");
        client.publish(topic8, isBulbOn ? "ON" : "OFF");
    }
}

void handleFanControl() {
  switch (currentMode) {
    case MANUAL: {
      int fanButtonState = digitalRead(FAN_BUTTON_PIN); 
      if (fanButtonState == LOW && lastFanButtonState == HIGH) {
        delay(50); 
        if (digitalRead(FAN_BUTTON_PIN) == LOW) {
          isFanOn = !isFanOn;  
          digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);  
          publishDeviceStatus();
        }
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
  int modeButtonState = digitalRead(MODE_BUTTON_PIN); 
  if (modeButtonState == LOW && lastModeButtonState == HIGH) {
    delay(50);  
    if (digitalRead(MODE_BUTTON_PIN) == LOW) {
      currentMode = (currentMode == MANUAL) ? AUTOMATIC : MANUAL; 
      publishDeviceStatus();
    }
  }
  lastModeButtonState = modeButtonState; 
}
void handlePumpControl() {
  int PumpButtonState = digitalRead(PUMP_BUTTON_PIN);
  if (PumpButtonState == LOW && lastPumpButtonState == HIGH) {
    delay(50);
    if (digitalRead(PUMP_BUTTON_PIN) == LOW) {
      isPumpOn = !isPumpOn;
      digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);

      client.publish(topic5, isPumpOn ? "ON" : "OFF");
    }
  }
  lastPumpButtonState = PumpButtonState;
}
void handleBulbControl() {
  int BulbButtonState = digitalRead(BULB_BUTTON_PIN);
  if (BulbButtonState == LOW && lastBulbButtonState == HIGH) {
    delay(50);
    if (digitalRead(BULB_BUTTON_PIN) == LOW) {
      isBulbOn = !isBulbOn;
      digitalWrite(BULB_RELAY_PIN, isBulbOn ? HIGH : LOW);
      client.publish(topic8, isBulbOn ? "ON" : "OFF");
    }
  }
  lastBulbButtonState = BulbButtonState;
}
String ratefood (){
  float distanceOrigin = 8.0;
  float foodAvailable = (1.0- (distance/distanceOrigin))*100.0;
  return String(foodAvailable);
}
void updateStatusDisplay() {
    automaticfeeding();  // Cập nhật trạng thái tự động cho cơ chế cho ăn
    char timeStringBuff[10];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S", &timeinfo); // Lấy thời gian hiện tại

    unsigned long currentMillis = millis();

    // Luân phiên hiển thị giữa dữ liệu cảm biến và trạng thái thiết bị sau mỗi 5 giây
    if (currentMillis - previousDisplayMillis >= displayInterval) {
        previousDisplayMillis = currentMillis;
        showSensorData = !showSensorData;  // Chuyển đổi cờ hiển thị
    }

    // Xóa màn hình OLED trước khi hiển thị nội dung mới
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);

    // Hiển thị dữ liệu cảm biến
    if (showSensorData) {
        display.println("Time: " + String(timeStringBuff));  // Hiển thị thời gian hiện tại
        display.println("Sensor Data:");                    // Tiêu đề cho phần dữ liệu cảm biến
        display.println("Temp: " + String(temperature) + " C");  // Nhiệt độ
        display.println("Humidity: " + String(humidity) + " %"); // Độ ẩm
        display.println("Air Quality: " + String(airQuality));   // Chất lượng không khí
        display.println("Food: " + ratefood() + " %");           // Mức thức ăn còn lại
    } 
    // Hiển thị trạng thái thiết bị
    else {
        display.println("Device Status:");                       // Tiêu đề cho phần trạng thái thiết bị
        display.println(isFanOn ? "Fan: ON" : "Fan: OFF");       // Trạng thái quạt
        display.println(currentMode == AUTOMATIC ? "Fan mode: AUTOMATIC" : "Fan mode: MANUAL");  // Chế độ quạt
        display.println(isPumpOn ? "Pump: ON" : "Pump: OFF");    // Trạng thái máy bơm
        display.println(isBulbOn ? "Bulb: ON" : "Bulb: OFF");    // Trạng thái bóng đèn
        display.println(isServoAt90 ? "Servo: ON" : "Servo: OFF");  // Trạng thái servo
    }

    display.display();  // Cập nhật màn hình OLED với nội dung mới
}

void loop() {
  handleFanControl();
  handlePumpControl();
  handleServoControl();
  updateStatusDisplay();
  publishDeviceStatus();
  client.loop();
}
