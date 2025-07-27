#define STEP_PIN 2
#define DIR_PIN 5
#define EN_PIN 8

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW); // Enable driver
  digitalWrite(DIR_PIN, HIGH); // Set direction

  Serial.begin(9600);
  Serial.println("Setup started...");
}

void loop() {
  for (int i = 0; i < 200; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}
