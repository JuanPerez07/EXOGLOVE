/*
 * Código para controlar un módulo de relé (D7) y un LED de estado (D2)
 * con un botón (D3) usando un Arduino UNO.
 *
 * - El botón actúa como un interruptor (toggle).
 * - El LED de D2 se enciende/apaga junto con el relé.
 */

// --- Configuración de Pines ---
const int PIN_LED = 2;   // Pin para el LED de estado externo
const int PIN_BOTON = 3; // Pin para el pulsador (MOVIDO a D3)
const int PIN_RELE = 7;  // Pin para la señal "IN" del relé

// --- Lógica del Relé (IMPORTANTE) ---
// La mayoría de estos módulos se activan con un estado BAJO (LOW).
const int RELE_ENCENDIDO = LOW;
const int RELE_APAGADO = HIGH;

// --- Lógica del LED (ESTÁNDAR) ---
// El LED se enciende con un estado ALTO (HIGH).
const int LED_ENCENDIDO = HIGH;
const int LED_APAGADO = LOW;

// --- Variables de Estado ---
bool estadoRele = false;     // Estado lógico del relé (false=apagado, true=encendido)
int lastButtonSt; // prev button state
int currentButtonSt; // current button state
//long ultimoTiempoRebote = 0;  // Para el anti-rebote
//long retardoRebote = 50;      // 50 milisegundos

void setup() {
  // Configurar los pines de SALIDA
  pinMode(PIN_RELE, OUTPUT);
  pinMode(PIN_LED, OUTPUT); // <-- NUEVO: Configura el pin del LED

  // Estado inicial: todo apagado
  digitalWrite(PIN_RELE, RELE_APAGADO);
  digitalWrite(PIN_LED, LED_APAGADO); // <-- NUEVO: Apaga el LED al iniciar

  // Configurar el pin del botón como ENTRADA con PULLUP
  // El pin estará en HIGH, y pasará a LOW al presionarlo.
  pinMode(PIN_BOTON, INPUT_PULLUP); // <-- ACTUALIZADO: Usa el Pin 3

  // Iniciar monitor serie (opcional, para ver qué pasa)
  Serial.begin(9600);
  Serial.println("Sistema listo. Esperando pulso...");
  Serial.println("LED -> D2, Boton -> D3, Relay -> D7.");
  lastButtonSt = HIGH; // init
}

void loop() {
  currentButtonSt = digitalRead(PIN_BOTON);

    
  // Comprobar si el botón FUE PRESIONADO (pasó de HIGH a LOW)
  if (currentButtonSt == LOW && lastButtonSt == HIGH) {     
    //Serial.println("Botón presionado!");
      
    // 4. Invertir el estado del relé (Toggle)
    estadoRele = !estadoRele; // Cambia de true a false, o de false a true

    // 5. Actuar sobre el relé Y EL LED
    if (estadoRele == true) {
      digitalWrite(PIN_RELE, RELE_ENCENDIDO);
      digitalWrite(PIN_LED, LED_ENCENDIDO); // <-- NUEVO: Enciende el LED
      Serial.println("--> RELAY ON");
    } else {
      digitalWrite(PIN_RELE, RELE_APAGADO);
      digitalWrite(PIN_LED, LED_APAGADO); // <-- NUEVO: Apaga el LED
      Serial.println("--> RELAY OFF");
    }
  }
  // 6. Guardar el estado actual del botón para la próxima iteración
  lastButtonSt = currentButtonSt;
}
