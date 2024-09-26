#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <DHT.h>
#include <ESP32Servo.h>
 
 // test gitclone

// Nhiet do và do am
#define DHTPIN 5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// MQ135
#define MQ135_PIN 34

// Sieu am
#define TRIG_PIN 19
#define ECHO_PIN 18
long duration;
float distance;

//Quat
#define FAN_BUTTON_PIN 23  
#define FAN_RELAY_PIN  27  

bool isFanOn = false;  // Biến để theo dõi trạng thái quạt
int lastFanButtonState = HIGH;  // Biến để theo dõi trạng thái nút trước đó

//May bơm
#define PUMP_BUTTON_PIN 4
#define PUMP_RELAY_PIN 25

bool isPumpOn = false;  // Biến để theo dõi trạng thái máy bơm
int lastPumpButtonState = HIGH;  // Biến để theo dõi trạng thái nút trước đó

// OLED Display 
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire, -1);
unsigned long previousMillis = 0; // Thời gian trước đó
const long interval = 5000; // Thay đổi sau mỗi 5 giây
bool showSensorData = true; // Biến để theo dõi trạng thái hiển thị


// WiFi và MQTT
const char *ssid = "Hoang11"; // Nhập tên WiFi của bạn
const char *password = "1234567890"; // Nhập mật khẩu WiFi của bạn

// MQTT Broker
const char *mqtt_broker = "broker.emqx.io";
const char *topic0 = "esp32/temp";
const char *topic1 = "esp32/hum";
const char *topic2 = "esp32/air";
const char *topic3 = "khoangcach";
const char *topic4 = "esp32/quat/control";
const char *topic5 = "esp32/maybom/control";
const char *topic6 = "esp32/den/control";
const char *topic7 = "esp32/servo/control";
const char *mqtt_username = "hoangpham1";
const char *mqtt_password = "123456";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  // Khởi tạo Serial
  Serial.begin(115200);

  // Khởi tạo chân cảm biến siêu âm
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLDOWN);

  // Khởi tạo DHT11 sensor
  dht.begin();

  // Khởi tạo OLED Display
  if (!display.begin(0x3C, true)) {
    Serial.println("OLED initialization failed");
    while (1);
  }
  display.display();
  delay(100);
  display.clearDisplay();

 // Khởi tạo Button và relay quạt
  pinMode(FAN_BUTTON_PIN, INPUT_PULLUP); // Thiết lập chân GPIO là input pull-up
  pinMode(FAN_RELAY_PIN, OUTPUT);        // Thiết lập chân GPIO là output
  digitalWrite(FAN_RELAY_PIN, LOW);      // Đảm bảo quạt tắt khi khởi động

  // Khởi tạo Button và relay máy bơm
  pinMode(PUMP_BUTTON_PIN, INPUT_PULLUP); // Thiết lập chân GPIO là input pull-up
  pinMode(PUMP_RELAY_PIN, OUTPUT);        // Thiết lập chân GPIO là output
  digitalWrite(PUMP_RELAY_PIN, LOW);      // Đảm bảo máy bơm tắt khi khởi động

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  // Kết nối MQTT
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public emqx mqtt broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

String readDHTTemperature() {
    
    float t = dht.readTemperature();
    
    if (isnan(t)) {
        Serial.println("Failed to read from DHT sensor!");
        return "-.-";
    } else {
        Serial.println(t);
        return String(t);
    }
}

String readDHTHumidity() {
 
    float h = dht.readHumidity();
    if (isnan(h)) {
        Serial.println("Failed to read from DHT sensor!");
        return "-.-";
    } else {
        Serial.println(h);
        return String(h);
    }
}

String mq135() {
  int digitalNumber = analogRead(MQ135_PIN); // Đọc giá trị từ cảm biến
  if (digitalNumber < 10) { // Giả sử giá trị nhỏ hơn 10 là lỗi (không có cảm biến)
    Serial.println("Error: No sensor detected or sensor not connected properly.");
    return "Error"; // Trả về thông báo lỗi
  } else {
    String airQuality = String(digitalNumber);
    Serial.println("Air quality: " + airQuality);
    return airQuality; // Trả về giá trị dưới dạng chuỗi
  }
}

String readUltrasonicSensor() {
  // Ensure the trigger pin is low before starting
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);  // Small delay to settle

  // Trigger the ultrasonic pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);  // Pulse for 10 microseconds
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pin with a timeout
  duration = pulseIn(ECHO_PIN, HIGH, 30000UL);  // Timeout at 30ms (for max ~5m)

  // Check for timeout (no echo received or no sensor connected)
  if (duration == 0) {
    Serial.println("Error: No echo received or sensor not connected.");
    return "Error";  // Return error if no echo or sensor not connected
  }

  // Sanity check: discard unusually long or short pulse durations
  if (duration > 30000UL || duration < 100) {  // If duration is out of realistic range
    Serial.println("Error: Invalid pulse duration (sensor disconnected or noise).");
    return "Error";  // Return error if duration is too long or too short
  }

  // Calculate distance in centimeters
  distance = (duration * 0.034) / 2;  // Speed of sound in air is ~0.034 cm/µs

  // Validate the measured distance (valid range is typically 2 cm to 400 cm)
  if (distance < 2 || distance > 400) {
    Serial.println("Error: Distance out of range or sensor malfunction.");
    return "Out of range";  // Return error if out of valid range
  }

  // Print distance in the serial monitor for debugging
  Serial.print("Distance (cm): ");
  Serial.println(distance);

  return String(distance);  // Return the distance as a string
}

void handleFanControl() {
  int FanButtonState = digitalRead(FAN_BUTTON_PIN);

  // Chỉ kiểm tra trạng thái nếu nút được nhấn
  if (FanButtonState == LOW && lastFanButtonState == HIGH) {
    delay(50);  // Đợi một chút để tránh nhấp nháy
    if (digitalRead(FAN_BUTTON_PIN) == LOW) {
      isFanOn = !isFanOn;  // Đảo trạng thái quạt
      digitalWrite(FAN_RELAY_PIN, isFanOn ? HIGH : LOW); // Bật/tắt quạt
      Serial.println(isFanOn ? "ON" : "OFF");
    }
  }

  lastFanButtonState = FanButtonState;  // Cập nhật trạng thái nút bấm trước đó
}

void handlePumpControl() {
  int PumpButtonState = digitalRead(PUMP_BUTTON_PIN);

  // Chỉ kiểm tra trạng thái nếu nút được nhấn
  if (PumpButtonState == LOW && lastPumpButtonState == HIGH) {
    delay(50);  // Đợi một chút để tránh nhấp nháy
    if (digitalRead(PUMP_BUTTON_PIN) == LOW) {
      isPumpOn = !isPumpOn;  // Đảo trạng thái máy bơm
      digitalWrite(PUMP_RELAY_PIN, isPumpOn ? HIGH : LOW); // Bật/tắt máy bơm
      Serial.println(isPumpOn ? "ON" : "OFF");
    }
  }

  lastPumpButtonState = PumpButtonState;  // Cập nhật trạng thái nút bấm trước đó
}

void updateFanStatusDisplay() {
  display.setCursor(0, 50); // Vị trí hiển thị trạng thái quạt
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  
  String fanStatus = isFanOn ? "Fan: ON" : "Fan: OFF";
  display.println(fanStatus);
  display.display(); // Cập nhật hiển thị OLED
}

void updatePumpStatusDisplay() {
  display.setCursor(0, 50); // Vị trí hiển thị trạng thái máy bơm
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  
  String pumpStatus = isPumpOn ? "Pump: ON" : "Pump: OFF";
  display.println(pumpStatus);
  display.display(); // Cập nhật hiển thị OLED
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println(message);
  Serial.println("-----------------------");

  // Check if the message is for controlling the fan
  if (String(topic) == topic4) {
    if (message == "ON") {
      isFanOn = true;
      digitalWrite(FAN_RELAY_PIN, HIGH);
      Serial.println("ON");
    } else if (message == "OFF") {
      isFanOn = false;
      digitalWrite(FAN_RELAY_PIN, LOW);
      Serial.println("OFF");
    }
  }

   // Check if the message is for controlling the pump
  if (String(topic) == topic5) {
    if (message == "ON") {
      isPumpOn = true;
      digitalWrite(PUMP_RELAY_PIN, HIGH);
      Serial.println("ON");
    } else if (message == "OFF") {
      isPumpOn = false;
      digitalWrite(PUMP_RELAY_PIN, LOW);
      Serial.println("OFF");
    }
  }
}

void loop() {
 
  // Đọc dữ liệu cảm biến
  String temperature = readDHTTemperature();
  String humidity = readDHTHumidity();
  String airQuality = mq135();
  String distanceString = readUltrasonicSensor();
  handleFanControl();
  handlePumpControl();

  // Publish data to MQTT broker
  client.publish(topic0, temperature.c_str());
  client.publish(topic1, humidity.c_str());
  client.publish(topic2, airQuality.c_str());
  client.publish(topic3, distanceString.c_str());

 // Publish fan status to MQTT broker
  static bool lastFanState = false; // Lưu trạng thái quạt trước đó
  if (lastFanState != isFanOn) {
    String fanStatus = isFanOn ? "ON" : "OFF";
    client.publish(topic4, fanStatus.c_str());
    client.subscribe(topic4);
    lastFanState = isFanOn; // Cập nhật trạng thái quạt trước đó
  } 

  // Publish pump status to MQTT broker
  static bool lastPumpState = false; // Lưu trạng thái máy bơm trước đó
  if (lastPumpState != isPumpOn) {
    String pumpStatus = isPumpOn ? "ON" : "OFF";
    client.publish(topic5, pumpStatus.c_str());
    client.subscribe(topic5);
    lastPumpState = isPumpOn; // Cập nhật trạng thái máy bơm trước đó
  } 


  // OLED
  unsigned long currentMillis = millis(); // Lấy thời gian hiện tại
  // Nếu đã qua 5 giây, đổi trạng thái hiển thị
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Cập nhật thời gian trước đó
    showSensorData = !showSensorData; // Đảo trạng thái hiển thị
  }

  // Cập nhật màn hình OLED
  display.clearDisplay(); // Xóa màn hình trước khi cập nhật
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  if (showSensorData) {
    // Hiển thị dữ liệu cảm biến
    display.setCursor(0, 0);
    display.println("Sensor Data:");

    display.setCursor(0, 10);
    display.println("Temp: " + temperature + " C");

    display.setCursor(0, 20);
    display.println("Humidity: " + humidity + " %");

    display.setCursor(0, 30);
    display.println("Air Quality: " + airQuality);

    display.setCursor(0, 40);
    display.println("Distance: " + distanceString + " cm");
  } else {
    // Hiển thị trạng thái quạt
    String fanStatus = isFanOn ? "Fan: ON" : "Fan: OFF";
    String pumpStatus = isPumpOn ? "Pump: ON" : "Pump: OFF";
    display.setCursor(0, 0);
    display.println("Fan Status:");
    display.setCursor(0, 10);
    display.println(fanStatus);
    display.setCursor(0, 20);
    display.println(pumpStatus);
  }
  display.display(); // Cập nhật màn hình OLED

  client.loop();
}

