#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <mcp_can.h>

// FreeRTOS handles
TaskHandle_t TaskGPSHandle;
TaskHandle_t TaskUltrasonicHandle;
TaskHandle_t TaskCANHandle;
TaskHandle_t TaskDisplayHandle;

// GPS Module
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

float currentLat = 0.0;
float currentLon = 0.0;

// Ultrasonic Sensor
const int trigPin = 4;
const int echoPin = 5;
float obstacleDistance = 0.0;

// CAN Bus
const int SPI_CS_PIN = 15;
MCP_CAN CAN(SPI_CS_PIN);
int currentSpeedLimit = 60;

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Zones
float schoolZoneLat = 10.1234;
float schoolZoneLon = 78.5678;
float zoneRadius = 0.1; // in km

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
  }
  display.clearDisplay();
  display.display();

  // CAN
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN init OK!");
  } else {
    Serial.println("CAN init FAIL!");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);

  // Tasks
  xTaskCreatePinnedToCore(TaskGPS, "TaskGPS", 4096, NULL, 1, &TaskGPSHandle, 1);
  xTaskCreatePinnedToCore(TaskUltrasonic, "TaskUltrasonic", 2048, NULL, 1, &TaskUltrasonicHandle, 1);
  xTaskCreatePinnedToCore(TaskCAN, "TaskCAN", 2048, NULL, 1, &TaskCANHandle, 0);
  xTaskCreatePinnedToCore(TaskDisplay, "TaskDisplay", 2048, NULL, 1, &TaskDisplayHandle, 0);
}

void TaskGPS(void *pvParameters) {
  for (;;) {
    while (ss.available()) {
      gps.encode(ss.read());
      if (gps.location.isUpdated()) {
        currentLat = gps.location.lat();
        currentLon = gps.location.lng();

        float distance = TinyGPSPlus::distanceBetween(currentLat, currentLon, schoolZoneLat, schoolZoneLon) / 1000.0;

        if (distance < zoneRadius) {
          currentSpeedLimit = 30; // school zone
        } else {
          currentSpeedLimit = 60;
        }
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void TaskUltrasonic(void *pvParameters) {
  for (;;) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    obstacleDistance = duration * 0.034 / 2;

    if (obstacleDistance < 100.0) {
      currentSpeedLimit = 20; // obstacle too close
    }
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

void TaskCAN(void *pvParameters) {
  for (;;) {
    byte data[1];
    data[0] = currentSpeedLimit;
    CAN.sendMsgBuf(0x70, 0, 1, data);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskDisplay(void *pvParameters) {
  for (;;) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Lat: "); display.println(currentLat, 6);
    display.print("Lon: "); display.println(currentLon, 6);
    display.print("Obstacle: "); display.print(obstacleDistance); display.println(" cm");
    display.print("SpeedLimit: "); display.println(currentSpeedLimit);
    display.display();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
