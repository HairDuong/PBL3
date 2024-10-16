
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <DHT.h>
#include<ESP32Servo.h>
#include <time.h>

// Configuration
const long gmtOffset_sec = 7 * 3600;   // GMT+7
const int daylightOffset_sec = 0;

const char *ssid = "Hoang11";
const char *password = "1234567890";
const char *mqtt_broker = "broker.emqx.io";
const char *topic0 = "esp32/temp";
const char *topic1 = "esp32/hum";
const char *topic2 = "esp32/air";
const char *topic3 = "esp32/foodrate";
const char *topic4 = "esp32/quat/control";
const char *topic5 = "esp32/maybom/control";
const char *topic6 = "esp32/fan/mode";
const char *topic7 = "esp32/servo/control";
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
#define MODE_BUTTON_PIN 17 

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
unsigned long lastfeeding = 0;
unsigned long autofeedinginterval = 1000; 
int targetHour = 13;
int targetMinute = 43;
int targetSecond = 0;
int targetSecondclose =0;
unsigned long lastModeButtonPress = 0; 
unsigned long lastFanButtonPress = 0;  
const unsigned long debounceTime = 50;
int lastModeButtonState = HIGH;

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
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == topic4) {
    isFanOn = message == "ON";
    digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);
  }
  if (String(topic) == topic5) {
    isPumpOn = message == "ON";
    digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);
  }
  if (String(topic) == topic7)
  {
    if (message== "ON")
    { myServo.write(90);}
    if (message == "OFF")
    { myServo.write(0);}
    
  }
}

void automaticfeeding() 
{
  struct tm timeinfo;
  // Lấy thời gian thực
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Không thể lấy thời gian từ NTP Server");
    return;
  }

  // In ra thời gian hiện tại
  Serial.println(&timeinfo, "Thời gian hiện tại: %H:%M:%S");

  // Kiểm tra xem có đúng thời gian điều khiển servo không
  if (timeinfo.tm_hour == targetHour && timeinfo.tm_min == targetMinute && timeinfo.tm_sec == targetSecond) 
  {
    

    // Nếu servo chưa ở góc 90 và chưa ghi nhận thời điểm xoay
    if (myServo.read() != 90) 
    {
      Serial.println("Đúng giờ! Điều khiển servo đến góc 90 độ.");
      myServo.write(90);
      
    }
  }
    // Kiểm tra nếu đã đủ 10 giây  kể từ khi servo xoay đến góc 90 độ
     if (timeinfo.tm_hour == targetHour && timeinfo.tm_min == targetMinute && timeinfo.tm_sec == (targetSecondclose)) {
     {
      Serial.println("Servo đã ở góc 90 độ trong 10 giây. Quay lại góc 0 độ.");
      myServo.write(0);  // Xoay servo về góc 0 độ
      
     }
}
}

void setup() {
  Serial.begin(115200);

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
  client.subcribe(topic7);
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

void handleFanControl() {
  // Switch case for manual and automatic mode
  switch (currentMode) {
    case MANUAL: {
      int fanButtonState = digitalRead(FAN_BUTTON_PIN); // Read the fan button state
      if (fanButtonState == LOW && lastFanButtonState == HIGH) {
        delay(50); // Simple debounce
        if (digitalRead(FAN_BUTTON_PIN) == LOW) {
          isFanOn = !isFanOn;  // Toggle fan state
          digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);  // Control relay
          Serial.println(isFanOn ? "Fan ON (Manual)" : "Fan OFF (Manual)");  // Debugging output
          
          // Publish fan state to MQTT
          client.publish(topic4, isFanOn ? "ON" : "OFF");
        }
      }
      lastFanButtonState = fanButtonState; // Store the button state for comparison
      break;
    }

    case AUTOMATIC: {
      // Automatic fan control based on temperature
      if (temperature > 32.0) {
        isFanOn = true;  // Turn on the fan if the temperature is higher than 32°C
      } else {
        isFanOn = false; // Turn off the fan otherwise
      }

      digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW);  // Control the relay
      Serial.println(isFanOn ? "Fan ON (Auto)" : "Fan OFF (Auto)");  // Debugging output
      
      // Publish the fan state to MQTT
      client.publish(topic4, isFanOn ? "ON" : "OFF");
      break;
    }
  }

  // Handle mode switch (manual/automatic) with simple debounce
  int modeButtonState = digitalRead(MODE_BUTTON_PIN); // Read the mode button state
  if (modeButtonState == LOW && lastModeButtonState == HIGH) {
    delay(50);  // Simple debounce
    if (digitalRead(MODE_BUTTON_PIN) == LOW) {
      currentMode = (currentMode == MANUAL) ? AUTOMATIC : MANUAL; // Toggle the mode
      Serial.println(currentMode == MANUAL ? "Mode: Manual" : "Mode: Automatic");

      // Optional: publish the mode change via MQTT if needed
      client.publish(topic6, currentMode == MANUAL ? "Manual" : "Automatic");
    }
  }
  lastModeButtonState = modeButtonState;  // Store the button state for comparison
}

void handlePumpControl() {
  int PumpButtonState = digitalRead(PUMP_BUTTON_PIN);
  if (PumpButtonState == LOW && lastPumpButtonState == HIGH) {
    delay(50);
    if (digitalRead(PUMP_BUTTON_PIN) == LOW) {
      isPumpOn = !isPumpOn;
      digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW);
      Serial.println(isPumpOn ? "Pump ON" : "Pump OFF");
      
      // Gửi trạng thái máy bơm lên MQTT
      client.publish(topic5, isPumpOn ? "ON" : "OFF");
    }
  }
  lastPumpButtonState = PumpButtonState;
}

String ratefood (){
  float distanceOrigin = 8.0;
  float foodAvailable = (1.0- (distance/distanceOrigin))*100.0;
  return String(foodAvailable);
}

void updateStatusDisplay() {
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
  }

  if (currentMillis - lastfeeding >= autofeedinginterval) 
  {
    lastfeeding = currentMillis;
    automaticfeeding();
  }

  if (currentMillis - previousDisplayMillis >= displayInterval) {
    previousDisplayMillis = currentMillis;
    showSensorData = !showSensorData; // Đảo trạng thái hiển thị
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(0, 0);
  if (showSensorData) {
    display.println("Sensor Data:");
    display.println("Temp: " + String(temperature) + " C");
    display.println("Humidity: " + String(humidity) + " %");
    display.println("Air Quality: " + String(airQuality));
    display.println("Food: " + ratefood() + " %");
  } else {
    display.println("Device Status:");
    display.println(isFanOn ? "Fan: ON" : "Fan: OFF");
    display.println(currentMode ? "Mode: AUTOMATIC " : "Mode: MANUAL");
    display.println(isPumpOn ? "Pump: ON" : "Pump: OFF");
  }

  display.display();
}

void loop() {
  handleFanControl();
  handlePumpControl();
  updateStatusDisplay();
  client.loop();
}
