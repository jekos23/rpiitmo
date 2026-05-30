#include <Servo.h>

const int potPin = A0;
const int servoPin = 9;
const int servoUpAngle = 0;
const int servoDownAngle = 180;
const unsigned long telemetryIntervalMs = 50;

Servo bucketServo;
String serialBuffer = "";
int potValue = 0;
int currentServoAngle = servoUpAngle;
unsigned long lastTelemetryAt = 0;

void moveServoTo(int angle) {
  currentServoAngle = constrain(angle, 0, 180);
  bucketServo.write(currentServoAngle);
  Serial.print("OK:SERVO:");
  Serial.println(currentServoAngle);
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  command.toUpperCase();

  if (command == "PING") {
    Serial.println("OK:PONG");
    return;
  }

  if (command == "SERVO:UP") {
    moveServoTo(servoUpAngle);
    return;
  }

  if (command == "SERVO:DOWN") {
    moveServoTo(servoDownAngle);
    return;
  }

  if (command.startsWith("SERVO:")) {
    String angleText = command.substring(6);
    angleText.trim();
    int targetAngle = angleText.toInt();
    moveServoTo(targetAngle);
    return;
  }

  Serial.print("ERR:UNKNOWN:");
  Serial.println(command);
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();
    if (incoming == '\n' || incoming == '\r') {
      if (serialBuffer.length() > 0) {
        handleCommand(serialBuffer);
        serialBuffer = "";
      }
    } else if (serialBuffer.length() < 48) {
      serialBuffer += incoming;
    }
  }
}

void sendPotentiometerTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetryAt < telemetryIntervalMs) {
    return;
  }

  lastTelemetryAt = now;
  potValue = analogRead(potPin);
  Serial.println(potValue);
}

void setup() {
  Serial.begin(9600);
  bucketServo.attach(servoPin);
  moveServoTo(servoUpAngle);
}

void loop() {
  readSerialCommands();
  sendPotentiometerTelemetry();
}
