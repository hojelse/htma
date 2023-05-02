/*
 Stepper Motor Control -  revolution
 */

const int step_pin = 11;
const int dir_pin = 10;
const int steps_per_rev = 200;

void setup() {
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    int num = Serial.parseInt();
    Serial.print("Received: ");
    Serial.println(num);
    revolve(num);
  }
}

void revolve(int deg) {
  if (0 < deg) {
    digitalWrite(dir_pin, LOW);
  } else if (deg < 0) {
    digitalWrite(dir_pin, HIGH);
    deg = abs(deg);
  } else {
    return;
  }

  double step_per_deg = steps_per_rev / 360.0;

  int steps = deg * step_per_deg;

  Serial.print("Rotating ");
  Serial.print(deg);
  Serial.print(" degrees (");
  Serial.print(steps);
  Serial.println(" steps) ...");

  digitalWrite(step_pin, LOW);

  int delayms = 15;

  for (int i = 0; i < steps; i++) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(step_pin, LOW);
    delay(delayms);
  }
}
