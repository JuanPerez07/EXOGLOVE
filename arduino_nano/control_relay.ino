/*
 * Controla un relé (D7) y un LED (D2) con un botón (D3)
 * - Lógica de 'toggle' (interruptor)
 * - Anti-rebote (debounce) robusto
 * - Corrige el problema de arranque (Active-LOW)
 */

// --- Pines ---
const int LED_PIN = 2;   // LED de estado externo
const int BTN_PIN = 3;   // Pulsador
const int RLY_PIN = 7;   // Módulo de relé

// --- Lógica de Hardware ---
// (CORREGIDO) El relé es 'Active-LOW': LOW lo enciende
const int RLY_ON = LOW;
const int RLY_OFF = HIGH;

// El LED es 'Active-HIGH': HIGH lo enciende
const int LED_ON = HIGH;
const int LED_OFF = LOW;

// --- Variables Globales ---
bool rlyState = false;       // Estado lógico actual (false=OFF, true=ON)
int btnLastState = HIGH;     // Estado anterior del botón (PULLUP es HIGH)
long lastDebounceTime = 0;   // Última vez que el botón cambió
long debounceDelay = 50;     // Tiempo de anti-rebote (50ms)

void setup() {
  // Configurar pines de salida
  pinMode(RLY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Estado inicial: APAGADO
  // (Ahora RLY_OFF envía HIGH, manteniendo el relé apagado)
  digitalWrite(RLY_PIN, RLY_OFF);
  digitalWrite(LED_PIN, LED_OFF);

  // Configurar botón con resistencia PULLUP interna
  pinMode(BTN_PIN, INPUT_PULLUP);
  
  // Opcional: Iniciar Serial para depuración
  // Serial.begin(9600);
  // Serial.println("Sistema listo.");
}

void loop() {
  // 1. Leer el estado actual del botón
  int btnReading = digitalRead(BTN_PIN);

  // 2. Comprobar si el estado del botón ha cambiado (inicio de un rebote)
  if (btnReading != btnLastState) {
    // Reiniciar el temporizador de anti-rebote
    lastDebounceTime = millis();
  }

  // 3. Comprobar si ha pasado el tiempo de anti-rebote
  if ((millis() - lastDebounceTime) > debounceDelay) {
    
    // Si el estado se ha estabilizado, comprobar si fue una pulsación
    // (Buscamos un flanco de bajada: de HIGH a LOW)
    if (btnReading == LOW && btnLastState == HIGH) {
      
      // 4. Invertir el estado lógico (toggle)
      rlyState = !rlyState; // Invierte true/false

      // 5. Aplicar el nuevo estado al relé y al LED
      if (rlyState) { // Si rlyState es true (ENCENDIDO)
        digitalWrite(RLY_PIN, RLY_ON);
        digitalWrite(LED_PIN, LED_ON);
        // Serial.println("--> ON");
      } else { // Si rlyState es false (APAGADO)
        digitalWrite(RLY_PIN, RLY_OFF);
        digitalWrite(LED_PIN, LED_OFF);
        // Serial.println("--> OFF");
      }
    }
  }

  // 6. Guardar el estado actual para la próxima iteración
  btnLastState = btnReading;
}
