#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

// LCD Config
#define LCD_ADDRESS 0x27  
#define LCD_COLUMNS 16     
#define LCD_ROWS 2         
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// Core selection for ESP32
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Pin definitions
#define SOIL_MOISTURE_PIN 34  
#define DHT_PIN 32           
#define DHT_TYPE DHT11        
#define RELAY_PIN 25
#define PH_SENSOR_PIN A0   // GPIO 36 for pH sensor

// pH sensor calibration
const float CALIBRATION_VOLTAGE_7 = 3.2;  
const float VOLTAGE_PER_PH = 0.18;

volatile int soil_moisture_value = 0;
volatile float temperature = 0.0;
volatile float humidity = 0.0;
String weather_info = "Loading...";
float weather_temp = 0.0;
bool rainExpected = false;
float pH_value = 7.0;

// Create a binary semaphore
static SemaphoreHandle_t bin_sem;

// WiFi & MQTT Credentials
const char* ssid = "V";
const char* password = "loonnii123";
const char* mqtt_server = "test.mosquitto.org"; 

// MQTT Topics
const char* temp_topic = "home/temperature";  
const char* hum_topic = "home/humidity";  
const char* soil_topic = "home/soil_moisture";  
const char* weather_topic = "home/weather";  
const char* relay_topic = "home/relay";
const char* ph_topic = "home/ph";  

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHT_PIN, DHT_TYPE);  // Instantiate DHT sensor

// OpenWeatherMap API
String city = "Chennai";
String apiKey = "a7c57ae70c4cf46c79d4b7d6e407334e";
String serverPath = "http://api.openweathermap.org/data/2.5/weather?q=" + city + "&appid=" + apiKey + "&units=metric";

// WiFi Connection
void connectToWiFi() {
  lcd.clear();
  lcd.print("Connecting...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    lcd.print(".");
  }

  lcd.clear();
  lcd.print("WiFi Connected");
  delay(1000);
  lcd.clear();
}

// MQTT Reconnect Logic
void reconnect() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32_Smart_Agri")) {
      Serial.println("Connected to MQTT Broker!");
      client.subscribe(relay_topic);
    } else {
      Serial.print("MQTT connect failed, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// Handle MQTT Messages
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == relay_topic) {
    if (message == "ON") {
      digitalWrite(RELAY_PIN, HIGH);
    } else if (message == "OFF") {
      digitalWrite(RELAY_PIN, LOW);
    }
  }
}

// FreeRTOS Tasks
void weatherTask(void *parameters) {
  while (1) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverPath);
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0) {
        String payload = http.getString();
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, payload);

        if (!error) {
          weather_temp = doc["main"]["temp"].as<float>();
          weather_info = doc["weather"][0]["main"].as<const char*>();
          rainExpected = doc.containsKey("rain") && doc["rain"]["1h"].as<float>() > 0;

          String weatherMsg = "Weather: " + weather_info + ", Temp: " + String(weather_temp) + "C";
          client.publish(weather_topic, weatherMsg.c_str());
        }
      }
      http.end();
    }
    vTaskDelay(60000 / portTICK_PERIOD_MS);
  }
}

void soilMoistureTask(void *parameters) {
  while (1) {
    int raw_value = analogRead(SOIL_MOISTURE_PIN);
    soil_moisture_value = map(raw_value, 4095, 0, 0, 100);
    client.publish(soil_topic, String(soil_moisture_value).c_str());

    if (!rainExpected && soil_moisture_value < 20) {
      digitalWrite(RELAY_PIN, HIGH);
    } else {
      digitalWrite(RELAY_PIN, LOW);
    }
    vTaskDelay(4000 / portTICK_PERIOD_MS);
  }
}

void dhtSensorTask(void *parameters) {
  while (1) {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    if (!isnan(humidity) && !isnan(temperature)) {
      client.publish(temp_topic, String(temperature).c_str());
      client.publish(hum_topic, String(humidity).c_str());
    }
    vTaskDelay(4000 / portTICK_PERIOD_MS);
  }
}

void pHTask(void *parameters) {
  while (1) {
    int raw_value = analogRead(PH_SENSOR_PIN);
    float voltage = (raw_value / 4095.0) * 3.3;
    pH_value = 7.0 - ((voltage - CALIBRATION_VOLTAGE_7) / VOLTAGE_PER_PH);

    client.publish(ph_topic, String(pH_value).c_str());
    Serial.printf("Voltage: %.2f V, pH: %.2f\n", voltage, pH_value);

    vTaskDelay(4000 / portTICK_PERIOD_MS);
  }
}

void displayTask(void *parameters) {
  int displayIndex = 0;
  while (1) {
    if (xSemaphoreTake(bin_sem, portMAX_DELAY) == pdTRUE) {
      lcd.clear();
      switch (displayIndex) {
        case 0: lcd.print("Soil: " + String(soil_moisture_value) + " %"); break;
        case 1: lcd.print("Temp: " + String(temperature) + " C"); break;
        case 2: lcd.print("Humidity: " + String(humidity) + " %"); break;
        case 3: lcd.print(weather_info); break;
        case 4: lcd.print("pH: " + String(pH_value, 2)); break;
      }
      displayIndex = (displayIndex + 1) % 5;
      xSemaphoreGive(bin_sem);
    }
    vTaskDelay(4000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  dht.begin();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  connectToWiFi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  bin_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(bin_sem);

  xTaskCreatePinnedToCore(soilMoistureTask, "Soil Task", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(dhtSensorTask, "DHT Task", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(weatherTask, "Weather Task", 4096, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(pHTask, "pH Task", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(displayTask, "Display Task", 2048, NULL, 1, NULL, app_cpu);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
