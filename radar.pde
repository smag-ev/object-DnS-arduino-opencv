import processing.serial.*;

String SERIAL_PORT = "COM9";
int BAUD_RATE = 9600;

float MAX_RANGE = 200;
float BLIP_LIFETIME = 2500;
boolean DEBUG = false;

Serial serialPort;
PFont mono;
String inputBuffer = "";

float angle = 0;
float distance = 0;
float radarRadius;

int lastContactTime = -10000;
float lastContactAngle = 0;
float lastContactDistance = 0;

ArrayList<Blip> blips = new ArrayList<Blip>();

class Blip {
  float angle;
  float distance;
  int born;

  Blip(float angle, float distance) {
    this.angle = angle;
    this.distance = distance;
    born = millis();
  }
}

void setup() {
  fullScreen();
  surface.setTitle("Object Tracking Radar");
  smooth(8);

  mono = createFont("Consolas", 18);
  textFont(mono);

  radarRadius = height * 0.42;

  println("Available serial ports:");
  printArray(Serial.list());

  serialPort = new Serial(this, SERIAL_PORT, BAUD_RATE);
}

void draw() {
  readSerial();

  background(0);

  pushMatrix();
  translate(width / 2, height - height * 0.11);

  drawGrid();
  drawLabels();
  drawSweep();
  drawBlips();

  popMatrix();

  drawHUD();
  drawScanlines();
}

void readSerial() {
  if (serialPort == null) {
    return;
  }

  while (serialPort.available() > 0) {
    char character = (char) serialPort.read();

    if (
      character == '.' ||
      character == '\n' ||
      character == '\r'
    ) {
      parseMessage(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += character;

      if (inputBuffer.length() > 80) {
        inputBuffer = "";
      }
    }
  }
}

void parseMessage(String message) {
  message = trim(message);

  if (message.length() == 0) {
    return;
  }

  if (DEBUG) {
    println("RX: [" + message + "]");
  }

  String cleaned = message.replaceAll("[^0-9.-]", " ");
  String[] parts = splitTokens(cleaned);

  if (parts.length < 2) {
    return;
  }

  float parsedAngle = float(parts[0]);
  float parsedDistance = float(parts[1]);

  if (Float.isNaN(parsedAngle) || Float.isNaN(parsedDistance)) {
    return;
  }

  angle = constrain(parsedAngle, 0, 180);
  distance = parsedDistance;

  if (distance > 0 && distance <= MAX_RANGE) {
    blips.add(new Blip(angle, distance));

    lastContactTime = millis();
    lastContactAngle = angle;
    lastContactDistance = distance;
  }
}

void drawGrid() {
  noFill();
  strokeWeight(1);

  for (int ring = 1; ring <= 4; ring++) {
    float diameter = radarRadius * 2 * ring / 4.0;

    stroke(0, 180, 0, 130);
    arc(0, 0, diameter, diameter, PI, TWO_PI);
  }

  for (int degrees = 0; degrees <= 180; degrees += 30) {
    float x = cos(radians(degrees)) * radarRadius;
    float y = -sin(radians(degrees)) * radarRadius;

    stroke(0, 180, 0, 90);
    line(0, 0, x, y);
  }

  stroke(0, 180, 0, 160);
  line(-radarRadius, 0, radarRadius, 0);
}

void drawLabels() {
  fill(0, 200, 0);
  textAlign(CENTER, CENTER);
  textSize(13);

  int[] degrees = {0, 30, 60, 90, 120, 150, 180};

  for (int degree : degrees) {
    float x = cos(radians(degree)) * (radarRadius + 26);
    float y = -sin(radians(degree)) * (radarRadius + 26);

    text(degree + "\u00b0", x, y);
  }

  fill(0, 160, 0);
  textSize(11);
  textAlign(CENTER, BOTTOM);

  for (int ring = 1; ring <= 4; ring++) {
    float radius = radarRadius * ring / 4.0;
    int range = int(MAX_RANGE * ring / 4.0);

    text(range + "cm", 0, -radius - 3);
  }
}

void drawSweep() {
  strokeWeight(1.5);

  for (int trail = 0; trail < 30; trail++) {
    float sweepAngle = angle - trail * 1.2;

    if (sweepAngle < 0 || sweepAngle > 180) {
      continue;
    }

    float alpha = map(trail, 0, 30, 160, 0);

    stroke(0, 200, 0, alpha);

    float x = cos(radians(sweepAngle)) * radarRadius;
    float y = -sin(radians(sweepAngle)) * radarRadius;

    line(0, 0, x, y);
  }

  float x = cos(radians(angle)) * radarRadius;
  float y = -sin(radians(angle)) * radarRadius;

  stroke(0, 255, 0, 220);
  strokeWeight(2);
  line(0, 0, x, y);
}

void drawBlips() {
  int now = millis();

  for (int index = blips.size() - 1; index >= 0; index--) {
    Blip blip = blips.get(index);
    float age = now - blip.born;

    if (age > BLIP_LIFETIME) {
      blips.remove(index);
      continue;
    }

    float alpha = map(age, 0, BLIP_LIFETIME, 255, 0);

    float pixels = constrain(
      map(
        blip.distance,
        0,
        MAX_RANGE,
        0,
        radarRadius
      ),
      0,
      radarRadius
    );

    float x = cos(radians(blip.angle)) * pixels;
    float y = -sin(radians(blip.angle)) * pixels;

    noStroke();
    fill(255, 220, 0, alpha);
    ellipse(x, y, 10, 10);

    noFill();
    stroke(255, 220, 0, alpha * 0.6);
    strokeWeight(1.2);
    ellipse(x, y, 24, 24);

    fill(255, 220, 0, alpha);
    textSize(10);
    textAlign(LEFT, BOTTOM);
    text(int(blip.distance) + "cm", x + 14, y - 4);
  }
}

void drawHUD() {
  boolean contact = millis() - lastContactTime < 500;

  fill(0, 0, 0, 180);
  noStroke();
  rect(20, 20, 420, 220);

  stroke(0, 200, 0, 100);
  strokeWeight(1);
  noFill();
  rect(20, 20, 420, 220);

  fill(0, 255, 0);
  textAlign(LEFT, TOP);
  textSize(16);
  text("OBJECT TRACKING RADAR", 36, 32);

  stroke(0, 180, 0, 80);
  line(36, 58, 420, 58);

  textSize(15);
  fill(0, 200, 0);

  text(
    "BEARING  : " + nf(angle, 0, 1) + "\u00b0",
    36,
    68
  );

  text(
    "RANGE    : " + nf(distance, 0, 0) + " CM",
    36,
    92
  );

  text(
    "CONTACTS : " + blips.size(),
    36,
    116
  );

  line(36, 140, 420, 140);

  if (contact) {
    if ((millis() / 400) % 2 == 0) {
      fill(255, 220, 0);
      textSize(16);
      text("OBJECT DETECTED", 36, 150);

      textSize(13);
      text(
        "ANGLE : " + nf(lastContactAngle, 0, 1) + "\u00b0",
        36,
        174
      );

      text(
        "DIST  : " + nf(lastContactDistance, 0, 0) + " CM",
        36,
        196
      );
    }
  } else {
    fill(0, 180, 0);
    textSize(15);
    text("STATUS   : SCANNING...", 36, 150);
  }

  fill(0, 0, 0, 160);
  noStroke();
  rect(0, height - 38, width, 38);

  fill(0, 180, 0);
  textSize(12);
  textAlign(CENTER, CENTER);

  text(
    "OBJECT TRACKING RADAR  |  SERIAL @ " +
    BAUD_RATE +
    "  |  C = CLEAR CONTACTS",
    width / 2,
    height - 18
  );
}

void drawScanlines() {
  stroke(0, 0, 0, 35);
  strokeWeight(1);

  for (int y = 0; y < height; y += 3) {
    line(0, y, width, y);
  }

  float barY =
    (millis() * 0.08) % (height + 100) - 50;

  noStroke();
  fill(0, 255, 0, 8);
  rect(0, barY, width, 30);
}

void keyPressed() {
  if (key == 'c' || key == 'C') {
    blips.clear();
  }
}
