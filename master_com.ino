#include <ArduinoBLE.h>
// button pins
#define PIN_RS_CLOSE 24 // A7
#define PIN_RS_OPEN 23 // A6
#define PIN_RS_PRON 22 // A5
#define PIN_RS_SUP 21 // A4
#define PIN_RS_ENABLE 20 // A3

#define MAX_BUTTONS 5
#define MAX_ACTIONS 4

// BLE 
BLEDevice peripheral;  // BLE Peripheral Device (Slave Nano)
BLECharacteristic txCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 512);

// array of button pins
const int button_pins[MAX_BUTTONS] = {PIN_RS_CLOSE, PIN_RS_OPEN, PIN_RS_PRON, PIN_RS_SUP, PIN_RS_ENABLE};
// COMMANDS (START and STOP could be added) --> ClsOpnPronSup -> 4 bit code where all of them are XOR

void setup() {
    // Initialize Serial for UART comms
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Master Nano: BLE Central Initialized");
    // Initialize BLE
    if (!BLE.begin()) {
        Serial.println("Failed to initialize BLE!");
        while (1);
    }
    // Set switches as input pullup
    for (int i = 0; i < MAX_BUTTONS; i++) {
        pinMode(button_pins[i], INPUT_PULLUP); // resistence as input pullup
    }
    Serial.println("Scanning for Slave Nano...");
}
// returns a single bit, 1 if the button is ON (LOW) and 0 if the button is OFF (HIGH)
String readButtons(const int &pin) {
    return digitalRead(pin) == LOW ? "1" : "0";
}
void loop() {
    // Try to find the slave device
    BLE.scanForName("NanoESP32_BLE");  // Match with the advertised name of the slave

    peripheral = BLE.available();
    static int prev_st = HIGH; // previous state of the enable button
    if (peripheral) {
        Serial.println("Connected to Slave Nano!");
        BLE.stopScan();
        if (peripheral.connect()) {
            Serial.println("Connection successful!");

            // Discover the UART Service and Characteristics
            if (peripheral.discoverService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")) {
                txCharacteristic = peripheral.characteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

                if(txCharacteristic) {
                  while (true) {
                      Serial.println("Waiting to be enabled");

                      // Esperar a que el botón sea presionado (LOW → HIGH)
                      while (digitalRead(PIN_RS_ENABLE) == prev_st);

                      prev_st = digitalRead(PIN_RS_ENABLE); // Actualizar estado

                      Serial.println("Ready to send data!");
                      // Mientras el periférico esté conectado y el botón no se suelte
                      while (peripheral.connected() && digitalRead(PIN_RS_ENABLE) == prev_st) {
                          String command = "";
                          // create the binary encoded command
                          for (int i = 0; i < MAX_ACTIONS; i++) {
                              command += readButtons(button_pins[i]);
                              command.trim();
                          }
                          // send the command and update display
                          if (!command.isEmpty()) {
                            txCharacteristic.writeValue(command.c_str());
                            Serial.println("Sent: " + command);
                          }
                          // reading frequency
                          delay(2000);
                      }

                      prev_st = digitalRead(PIN_RS_ENABLE); // Actualizar estado al soltar
                  }
                } else {
                    Serial.println("Characteristic not found!");
                }
            } else {
                Serial.println("Service not found!");
            }
        } else {
            Serial.println("Failed to connect. Retrying...");
        }
    }
    delay(1000);  // Short delay before trying again
}
