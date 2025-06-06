#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <LiquidCrystal_I2C.h>
#include <DHT20.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Mlt_ESP32OTA.h>
#include <ArduinoOTA.h>

// LCD
#define I2C_ADDR    0x27
#define LCD_COLUMNS 20
#define LCD_LINES   4
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);

// DHT20 Sensor
DHT20 dht20;

// WiFi & ThingsBoard Settings
#define YOUR_WIFI_SSID "ACLAB"
#define YOUR_WIFI_PASSWORD "ACLAB2023"
#define YOUR_THINGSBOARD_SERVER "thingsboard.cloud"
#define YOUR_TOKEN "8AIEOQRywKJ8NLTDdgcqDVUsdBO7kE8WV3624LIc" // change as needed

// LED Pins
constexpr int LED_PIN1 = 25;
constexpr int LED_PIN2 = 33;
constexpr int LED_PIN3 = 32;
constexpr int LED_PIN4 = 26;
constexpr int LED_PIN5 = 27;

// PIR Sensor
constexpr int PIR_PIN = 14;
bool motionDetected = false;

// RFID Reader
constexpr int SS_PIN = 5;
constexpr int RST_PIN = 22;
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
String rfidUid = "";

// MQTT Configuration
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, 1024);
uint32_t previousDataSend;
constexpr int telemetrySendInterval = 2000U;

// Shared Attributes
volatile bool deviceState1 = false, deviceState2 = false, deviceState3 = false, deviceState4 = false, deviceState5 = false;
volatile int deviceValue5 = 0;
bool attributesChanged = false;

// Function Declarations
void InitWiFi();
void processSwitchChange(const RPC_Data &data);
void processSharedAttributes(const Shared_Attribute_Data &data);
void initRFID();

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Air Conditioner");

  dht20.begin();
  pinMode(PIR_PIN, INPUT);
  initRFID();

  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
  pinMode(LED_PIN4, OUTPUT);
  pinMode(LED_PIN5, OUTPUT);

  InitWiFi();

  // OTA Setup
  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle(); // Handle OTA requests

  if (!tb.connected()) {
    if (!reconnect()) {
      return;
    }
  }

  tb.loop();

  // Read sensors
  motionDetected = digitalRead(PIR_PIN);
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    rfidUid = "";
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      rfidUid += String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
      rfidUid += String(mfrc522.uid.uidByte[i], HEX);
    }
  }

  // Send telemetry every interval
  if (millis() - previousDataSend > telemetrySendInterval) {
    previousDataSend = millis();

    float h = dht20.getHumidity();
    float t = dht20.getTemperature();

    tb.sendTelemetryData("tempIndoor", t);
    tb.sendTelemetryData("humidity", h);
    tb.sendTelemetryData("deviceValue5", deviceValue5);
    tb.sendTelemetryData("motionDetected", motionDetected);
    if (rfidUid.length() > 0) {
      tb.sendTelemetryData("rfidUid", rfidUid);
    }

    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());

    // Display on LCD
    char tempStr[8];
    dtostrf(t, 4, 1, tempStr);
    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    lcd.print(tempStr);
    lcd.print(" C");
  }

  delay(100);
}

void InitWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(YOUR_WIFI_SSID, YOUR_WIFI_PASSWORD);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

bool reconnect() {
  if (!wifiClient.connect(YOUR_THINGSBOARD_SERVER, 1883)) {
    Serial.println("MQTT connection failed!");
    return false;
  }
  if (!tb.connect(YOUR_THINGSBOARD_SERVER, YOUR_TOKEN)) {
    Serial.println("ThingsBoard connection failed!");
    return false;
  }

  tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

  tb.RPC_Subscribe([](const RPC_Data &data) {
    processSwitchChange(data);
  });

  tb.Shared_Attributes_Subscribe([](const Shared_Attribute_Data &data) {
    processSharedAttributes(data);
  });

  return true;
}

void processSwitchChange(const RPC_Data &data) {
  deviceValue5 = data["switch"];
  attributesChanged = true;
}

void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    const char* key = it->key().c_str();
    bool value = it->value().as<bool>();

    if (strcmp(key, "deviceState1") == 0) {
      deviceState1 = value;
      digitalWrite(LED_PIN1, deviceState1);
    } else if (strcmp(key, "deviceState2") == 0) {
      deviceState2 = value;
      digitalWrite(LED_PIN2, deviceState2);
    } else if (strcmp(key, "deviceState3") == 0) {
      deviceState3 = value;
      digitalWrite(LED_PIN3, deviceState3);
    } else if (strcmp(key, "deviceState4") == 0) {
      deviceState4 = value;
      digitalWrite(LED_PIN4, deviceState4);
    } else if (strcmp(key, "deviceState5") == 0) {
      deviceState5 = value;
      digitalWrite(LED_PIN5, deviceState5);
    }
  }
}

void initRFID() {
  SPI.begin();
  mfrc522.PCD_Init();
}