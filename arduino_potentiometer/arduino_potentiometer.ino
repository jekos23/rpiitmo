#include <Servo.h>

const int servoPin = 9;
const int servoUpAngle = 0;
const int servoDownAngle = 90;
const int servoStepDegrees = 2;
const unsigned long servoStepDelayMs = 18;

Servo bucketServo;
String serialBuffer = "";
int currentServoAngle = servoDownAngle;

void moveServoTo(int angle) {
  const int targetAngle = constrain(angle, 0, 180);

  if (targetAngle == currentServoAngle) {
    bucketServo.write(currentServoAngle);
    Serial.print("OK:SERVO:");
    Serial.println(currentServoAngle);
    return;
  }

  const int direction = targetAngle > currentServoAngle ? 1 : -1;
  while (currentServoAngle != targetAngle) {
    currentServoAngle += direction * servoStepDegrees;

    if ((direction > 0 && currentServoAngle > targetAngle) ||
        (direction < 0 && currentServoAngle < targetAngle)) {
      currentServoAngle = targetAngle;
    }

    bucketServo.write(currentServoAngle);
    delay(servoStepDelayMs);
  }

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

void setup() {
  Serial.begin(9600);
  bucketServo.attach(servoPin);
  bucketServo.write(currentServoAngle);
  delay(150);
  Serial.println("OK:READY");
}

void loop() {
  readSerialCommands();
}
