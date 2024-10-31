#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <math.h>

#define sensorPin 2  // Pin connected to the flow sensor
#define relayPin 4   // Pin connected to the relay controlling the solenoid valve

// Cloud variables
float flowRate;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
int solenoidStatus = 1;  // Default solenoid status: 1 (open)

// Vibration and flow thresholds
const float vibrationThreshold = 10.0;
const float flowThreshold = 20.0;

// WiFi credentials
const char SSID[] = "Airtel_Sundar";
const char PASS[] = "Rakesh531";

// WiFi server settings
WiFiServer server(80);
Adafruit_MPU6050 mpu;

volatile int pulseCount = 0;
unsigned long oldTime = 0;

void pulseCounter() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);

  // Initialize cloud
  ArduinoCloud.begin(SSID, PASS);

  // Configure cloud properties
  ArduinoCloud.addProperty(flowRate, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(accelX, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(accelY, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(accelZ, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(gyroX, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(gyroY, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(gyroZ, READ, ON_CHANGE, NULL);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 initialized.");

  // Setup flow sensor
  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);

  // Setup relay
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);  // Initial state OFF

  // Connect to WiFi and start server
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  server.begin();
}

void loop() {
  ArduinoCloud.update();

  // Calculate flow rate
  unsigned long currentTime = millis();
  if (currentTime - oldTime >= 1000) {
    noInterrupts();
    flowRate = pulseCount / 7.5;  // Calculate flow rate in L/min
    pulseCount = 0;
    interrupts();
    oldTime = currentTime;
    Serial.print("Flow Rate: ");
    Serial.print(flowRate);
    Serial.println(" L/min");
  }

  // Read accelerometer and gyroscope data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;

  // Calculate vibration magnitude
  float vibrationMagnitude = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));
  Serial.print("Vibration Magnitude: ");
  Serial.println(vibrationMagnitude);

  // Determine solenoid status based on vibration and flow rate
  if (vibrationMagnitude > vibrationThreshold || flowRate > flowThreshold) {
    solenoidStatus = 0;  // High vibration or high flow, close valve
    digitalWrite(relayPin, HIGH);  // Activate relay to close solenoid valve
  } else {
    solenoidStatus = 1;  // Low vibration and low flow, open valve
    digitalWrite(relayPin, LOW);  // Deactivate relay to open solenoid valve
  }

  // Check for client connection to send data
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");

    if (client.available()) {
      String request = client.readStringUntil('\r');
      Serial.println(request);

      // Send JSON data including solenoid status and flow rate
      client.print("{\"solenoidStatus\":");
      client.print(solenoidStatus);
      client.print(",\"flowRate\":");
      client.print(flowRate);
      client.print("}");

      delay(1);
      client.stop();
      Serial.println("Client disconnected");
    }
  }

  delay(1000);  // Adjust as necessary
}
