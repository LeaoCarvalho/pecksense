// --- PINS ----------------------------------------------------
const int LED_WAITING   = 2;
const int LED_DETECTING = 3;
const int LED_FEEDING   = 4;
const int LED_EMPTY     = 5;
const int MOTOR_PIN     = 6;

// --- STATE MACHINE -------------------------------------------
enum State {WAITING, DETECTING, FEEDING, EMPTY};
State currentState = WAITING;

// --- TIMERS ---------------------------------------------------
unsigned long motorStartTime = 0;
const unsigned long motorDuration = 5000; // 5 seconds
bool motor_working = false;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);  // 100ms = 0.1 second


  pinMode(LED_WAITING, OUTPUT);
  pinMode(LED_DETECTING, OUTPUT);
  pinMode(LED_FEEDING, OUTPUT);
  pinMode(LED_EMPTY, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  updateLEDs();
}

// Updates LED based on the current state
void updateLEDs() {
  digitalWrite(LED_WAITING,   currentState == WAITING);
  digitalWrite(LED_DETECTING, currentState == DETECTING);
  digitalWrite(LED_FEEDING,   currentState == FEEDING);
  digitalWrite(LED_EMPTY,     currentState == EMPTY);
}

bool randomEmpty() {
  // 25% chance of being empty
  return random(0, 4) == 0;
}

bool randomFill() {
  // 5% chance of being filled
  return random(0, 20) == 0;
}

void startMotor() {
  digitalWrite(MOTOR_PIN, HIGH);
  motorStartTime = millis();
  motor_working = true;
}

void stopMotor() {
  digitalWrite(MOTOR_PIN, LOW);
  motor_working = false;
}

void loop() {

  // --- Handle incoming serial commands -----------------------
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');

    msg.trim();

    if (msg == "STATE_WAITING") {
      currentState = WAITING;
      updateLEDs();
    }

    else if (msg == "STATE_DETECTING") {
      currentState = DETECTING;
      updateLEDs();
    }

    else if (msg == "STATE_FEEDING") {
      currentState = FEEDING;
      updateLEDs();

      bool empty = randomEmpty();

      if (empty) {
        currentState = EMPTY;
        updateLEDs();
        Serial.println("EMPTY");
      } else {
        startMotor();
      }
    }
  }

  // --- FEEDING: handle fake motor timing ---------------------
  if (motor_working) {
    if (millis() - motorStartTime > motorDuration) {
      stopMotor();

      // Feeding completed -> back to WAITING
      currentState = WAITING;
      updateLEDs();
      Serial.println("DONE_FEEDING");
    }
  }
  if(currentState == EMPTY){
    if(randomFill()){
        currentState = WAITING;
        updateLEDs();
        Serial.println("FILLED");
    }

  }
}
