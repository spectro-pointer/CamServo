#include <Arduino.h>
#include <capsule.h>
#include <Servo.h>

#define CAPSULE_ID_ZOOM 0x13
#define CAPSULE_ID_RECORD 0x14

void handlePacket(uint8_t, uint8_t*, uint32_t);
CapsuleStatic capsuleRaspberry(handlePacket);

#define SERVO_PIN 23
#define SERVO_NEUTRAL 90
#define SERVO_ZOOM_PLUS 180
#define SERVO_ZOOM_MINUS 0
#define SERVO_PULSE_TIME 1000

Servo servo;

struct dataStruct {
  int zoom;
};

dataStruct lastCmd;

void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
}

void loop() {
  while(Serial.available()) {
    capsuleRaspberry.decode(Serial.read());
  }
  // digitalWrite(14, LOW);
  // delay(100);
  // digitalWrite(14, HIGH);
  // servo.write(SERVO_NEUTRAL);
  // analogWrite(13, 103);
  // delay(5000);
  // servo.write(SERVO_ZOOM_PLUS);
  // analogWrite(13, 203);
  // delay(5000);
  // servo.write(SERVO_ZOOM_MINUS);
  // analogWrite(13, 3);
  // delay(5000);
}

void handlePacket(uint8_t id, uint8_t* data, uint32_t size) {

  memcpy(&lastCmd, data, sizeof(dataStruct));

  switch (id) {
    case CAPSULE_ID_ZOOM:
      Serial.println("Received packet from Raspberry");
      if (lastCmd.zoom) {
        servo.write(lastCmd.zoom+90);
        analogWrite(13, lastCmd.zoom+103);
        delay(SERVO_PULSE_TIME);
        analogWrite(13, 103);
        servo.write(SERVO_NEUTRAL);
      }
    break;

    case CAPSULE_ID_RECORD:
      Serial.println("Received packet from Raspberry");
      digitalWrite(14, LOW);
      delay(100);
      digitalWrite(14, HIGH);
    default:
    break;
  }
}