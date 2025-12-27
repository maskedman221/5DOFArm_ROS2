#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#define TCP_PORT 3232

const char* ssid = "new-robotic";
const char* password = "3newprintersforsale";
WiFiServer server(TCP_PORT);
WiFiClient client;
const int NUM_SERVOS = 6;

Servo servos[NUM_SERVOS];

int servoPins[NUM_SERVOS] = {4, 16, 26, 25, 33, 32};

int servoMin[NUM_SERVOS] = {0, 10, 20, 30, 40, 50};
int servoMax[NUM_SERVOS] = {90, 120, 140, 150, 160, 170};

float resetAngles[NUM_SERVOS]   = {0, 130, 85, 0, 120, 0};
float resetAngles1[NUM_SERVOS]   = {0, 9,39, 0,0, 0};
float currentAngles[NUM_SERVOS] = {0, 70, 85, 0, 120, 180};
float startAngles[NUM_SERVOS];
float targetAngles[NUM_SERVOS];
float deltaAngles[NUM_SERVOS];
bool gripped = false;
unsigned long moveStartTime;
unsigned long moveDuration;
bool isMoving = false;

int jointAngles[NUM_SERVOS] = {0};

const int JSON_BUFFER_SIZE = 256;
char jsonBuffer[JSON_BUFFER_SIZE];
int bufferIndex = 0;

void onOTAStart() {
  Serial.println("OTA start - detaching servos");
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].detach();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname("esp32-servo");

  ArduinoOTA.onStart(onOTAStart);

  ArduinoOTA.begin();
  server.begin();
  Serial.println("TCP server started");
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i], 500, 2400); 
    servos[i].write(resetAngles[i]);
  }
  client.println("ESP32 Servo Controller Ready");
}

void loop() {
  ArduinoOTA.handle(); 
   if (!client || !client.connected()) {
      WiFiClient newClient = server.available();
      if (newClient) {
          client = newClient;
          Serial.println("Client connected");
      }
  }
  while (client && client.available()) {
    char c = client.read();
    Serial.println("connected");
    Serial.println(c);
    if (c == '\n') {
      jsonBuffer[bufferIndex] = '\0';
      processJSON(jsonBuffer);
      bufferIndex = 0;
    }
    else if (bufferIndex < JSON_BUFFER_SIZE - 1) {
      jsonBuffer[bufferIndex++] = c;
    }
    else {
      Serial.println("ERROR: JSON buffer overflow!");
      bufferIndex = 0;
    }
  }

  if (isMoving) {
    moveArm();
  }
}

bool processJSON(const char* jsonStr) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);

  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return false;
  }

  const char* msgType = doc["type"];
  if (strcmp(msgType, "joints") == 0) {
    JsonObject joints = doc["joints"];

    for (int i = 0; i < NUM_SERVOS; i++) {
      jointAngles[i] = joints[String("j") + (i + 1)];
    }

    Serial.println("start movement");
    startTimerMovement();
    return true;
  }
  return false;
}

void startTimerMovement() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    startAngles[i]  = currentAngles[i];
    targetAngles[i] = jointAngles[i];
    deltaAngles[i]  = targetAngles[i] - startAngles[i];
  }

  moveStartTime = millis();
  moveDuration  = 2000;
  isMoving      = true;
}

void moveArm() {
  unsigned long elapsed = millis() - moveStartTime;

  if (elapsed >= moveDuration) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      currentAngles[i] = targetAngles[i];
      servos[i].write(currentAngles[i]);
    }
    // if(gripped)
    // servos[5].write(0);
    gripped= false;
    isMoving = false;
    client.println("finished movement");
    return;
  }

  float t = (float)elapsed / (float)moveDuration;

  for (int i = 0; i < NUM_SERVOS; i++) {
    float newAngle = startAngles[i] + deltaAngles[i] * t;
    int rounded = round(newAngle);

    if (rounded != currentAngles[i]) {
      currentAngles[i] = rounded;
      servos[i].write(rounded);
    }
  }

  delay(10);
}
