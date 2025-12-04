// --- PINS ----------------------------------------------------
const int LED_WAITING   = 2;
const int LED_DETECTING = 3;
const int LED_FEEDING   = 4;
const int LED_EMPTY     = 5;
const int MOTOR_PIN     = 6;

void turnOnLEDs() {
  digitalWrite(LED_WAITING,   HIGH);
  digitalWrite(LED_DETECTING,   HIGH);
  digitalWrite(LED_FEEDING,   HIGH);
  digitalWrite(LED_EMPTY,   HIGH);
  digitalWrite(MOTOR_PIN,   HIGH);
}

void turnOffLEDs() {
  digitalWrite(LED_WAITING,   LOW);
  digitalWrite(LED_DETECTING,   LOW);
  digitalWrite(LED_FEEDING,   LOW);
  digitalWrite(LED_EMPTY,   LOW);
  digitalWrite(MOTOR_PIN,   LOW);
}


void setup() {


  pinMode(LED_WAITING, OUTPUT);
  pinMode(LED_DETECTING, OUTPUT);
  pinMode(LED_FEEDING, OUTPUT);
  pinMode(LED_EMPTY, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

}

void loop() {
  turnOnLEDs();

  delay(1000);

  turnOffLEDs();

  delay(1000);

}
