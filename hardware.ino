#include <Servo.h>

constexpr uint8_t TRACKING_SERVO_PIN = 2;

constexpr int SERVO_MIN = 10;
constexpr int SERVO_MAX = 170;
constexpr int DEFAULT_ANGLE = 90;

Servo trackingServo;

int currentAngle = DEFAULT_ANGLE;
int targetAngle = DEFAULT_ANGLE;

void setup() {
  Serial.begin(115200);

  trackingServo.attach(TRACKING_SERVO_PIN);
  trackingServo.write(currentAngle);

  delay(500);
  Serial.println("READY");
}

void loop() {
  if (currentAngle < targetAngle) {
    ++currentAngle;
  } else if (currentAngle > targetAngle) {
    --currentAngle;
  }

  trackingServo.write(currentAngle);
  delay(6);

  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command.startsWith("S")) {
    targetAngle = constrain(
      command.substring(1).toInt(),
      SERVO_MIN,
      SERVO_MAX
    );
  } else if (command.startsWith("R")) {
    targetAngle = constrain(
      command.substring(1).toInt(),
      0,
      180
    );

    currentAngle = targetAngle;
    trackingServo.write(currentAngle);

    else if (data == "FIRE") {
      fire();
    }

  } else if (command == "STOP") {
    // Hold the current tracking position.
  }
}
