#include <Servo.h>
#include <ArduinoJson.h>
const int NUM_SERVOS = 6;

Servo servos[NUM_SERVOS];
int servoPins[NUM_SERVOS] = {3, 5, 6, 9, 10, 11};
int servoMin[NUM_SERVOS] = {0, 10, 20, 30, 40, 50};
int servoMax[NUM_SERVOS] = {90, 120, 140, 150, 160, 170};
int resetAngles[NUM_SERVOS] = {0, 70, 85, 0, 120, 0}; 
int jointAngles[6] = {0};
const int JSON_BUFFER_SIZE = 256;
char jsonBuffer[JSON_BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
   for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(resetAngles[i]);
  }
}

void loop() {
 
  // Move from 0 to 180 degrees
  // for (int angle = 0; angle <= 180; angle++) {
  //       servos[i].write(angle);
  //       delay(10);
  //   }

  // // Move from 180 back to 0 degrees
  // for (int angle = 180; angle >= 0; angle--) {
  //       servos[i].write(angle);
  //       delay(10);
  //   }
  while(Serial.available())
  {
    char c = Serial.read();
    if(c=='\n')
    {
      Serial.println("end of json");
      jsonBuffer[bufferIndex] = '\0';
      processJSON(jsonBuffer);
      bufferIndex = 0;
    }
    else if(bufferIndex < JSON_BUFFER_SIZE -1){
      jsonBuffer[bufferIndex++] = c;

    }
    else{
      Serial.println("ERROR: JSON buffer overflow!");
      bufferIndex=0;
    }
  }
  // Serial.println("Hello ROS2");
}

void processJSON(const char* jsonStr){
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc , jsonStr);
  if(error){
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  const char* msgType = doc["type"];
  if(strcmp(msgType, "joints") == 0){
    JsonObject joints = doc["joints"];
    jointAngles[0] = joints["j1"];
    jointAngles[1] = joints["j2"];
    jointAngles[2] = joints["j3"];
    jointAngles[3] = joints["j4"];
    jointAngles[4] = joints["j5"];
    Serial.println("start movement");
    moveArm();
  }
}

void moveArm()
{
   for (int i = 0; i < NUM_SERVOS; i++)
   {
    
    servos[i].write(jointAngles[i]);
   }
   Serial.println("finished movement");
}