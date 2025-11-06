/*
 * Relay control via button toggle with Arduino Uno
 */

// --- Pines ---
const int LED_PIN = 2;   // LED to indicate circuit closed
const int BTN_PIN = 3;   // Toggle button
const int RLY_PIN = 7;   // Relay signal 

// RELAY is 'Active-LOW'
const int RLY_ON = LOW;
const int RLY_OFF = HIGH;

// LED is 'Active-HIGH'
const int LED_ON = HIGH;
const int LED_OFF = LOW;

// --- Global Vars ---
bool Serial_ON = false;    // trun true for debugging via Serial Monitor
bool rlyState = false;     // realy current state
volatile int btnSt;        // current button state
int btnLastState = HIGH;   // prev state of button (PULLUP -> HIGH)

void setup() {
  // pin config
  pinMode(RLY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Initial state: OFF
  digitalWrite(RLY_PIN, RLY_OFF);
  digitalWrite(LED_PIN, LED_OFF);

  // button config as PULLUP resistance
  pinMode(BTN_PIN, INPUT_PULLUP);
  
  // for debugging
  if(Serial_ON){
    Serial.begin(9600);
    Serial.println("System ready.");
  }
}

void loop() {
  btnSt = digitalRead(BTN_PIN); // read current state
    if (btnSt == LOW && btnLastState == HIGH) { // button was pressed
      rlyState = !rlyState; // swap relay state
      if (rlyState) { // NO - COM circuit is closed
        digitalWrite(RLY_PIN, RLY_ON);
        digitalWrite(LED_PIN, LED_ON);
        if(Serial_ON) Serial.println("Relay --> ON | NO >-< COM connected");
      } else { // NC - COM circuit is closed
        digitalWrite(RLY_PIN, RLY_OFF);
        digitalWrite(LED_PIN, LED_OFF);
        if(Serial_ON) Serial.println("Relay --> OFF | NC >-< COM connected");
      }
    }
  // update button state
  btnLastState = btnSt;
}
