#include <ArduinoBLE.h>
#define PIN_RS_CLOSE 24 // A7
#define PIN_RS_OPEN 23 // A6
#define PIN_RS_PRON 22 // A5
#define PIN_RS_SUP 21 // A4
#define PIN_RS_ENABLE 20 // A3
#define LED 2 // D2
#define MAX_BUTTONS 5
#define MAX_CODES 4

BLEDevice peripheral;  // BLE Peripheral Device (Slave Nano)
BLECharacteristic txCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 512);
// array of button pins
const int button_pins[MAX_BUTTONS] = {PIN_RS_CLOSE, PIN_RS_OPEN, PIN_RS_PRON, PIN_RS_SUP, PIN_RS_ENABLE};
// COMMANDS (START and STOP could be added)
const String grasp_str = "CLOSE";
const String open_str = "OPEN";
const String pron_str = "PRON";
const String sup_str = "SUPI";
// array of possible codes
const String COMMANDS[MAX_CODES] = {grasp_str, open_str, pron_str, sup_str};
void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Master Nano: BLE Central Initialized");

    if (!BLE.begin()) {
        Serial.println("Failed to initialize BLE!");
        while (1);
    }

    Serial.println("Scanning for Slave Nano...");
    // Set switches as input pullup
    for (int i = 0; i < MAX_BUTTONS; i++) {
        pinMode(button_pins[i], INPUT_PULLUP); // resistence as input pullup
    }
    pinMode(LED, OUTPUT);
}
String readButtons(const int pin){
    int st = digitalRead(pin);
    String str = "";
    if (st == LOW)
      str = COMMANDS[PIN_RS_CLOSE - pin];
    return str;
}
void loop() {
    // Try to find the slave device
    BLE.scanForName("NanoESP32_BLE");  // Match with the advertised name of the slave

    peripheral = BLE.available();
    static int prev_st = HIGH; 
    if (peripheral) {
        Serial.println("Connected to Slave Nano!");
        BLE.stopScan();
        if (peripheral.connect()) {
            Serial.println("Connection successful!");

            // Discover the UART Service and Characteristics
            if (peripheral.discoverService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")) {
                txCharacteristic = peripheral.characteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

                if(txCharacteristic) {
                  while(1){
                    bool en = false;
                    bool disabled = false;
                    do{
                        Serial.println("Waiting to be enabled");
                        int st = digitalRead(PIN_RS_ENABLE); // button state
                        en = (st == LOW and prev_st == HIGH); // pressed
                        prev_st = st; // update button state
                    }while(!en || disabled); // wait for comms to be enable
                    while (peripheral.connected() && ! disabled){
                        int st = digitalRead(PIN_RS_ENABLE); // button state
                        disabled = (st == HIGH and prev_st == LOW); // unpressed
                        if(disabled) break;
                        Serial.println("Ready to send data!");
                        //while (!Serial.available());  // Wait for user input
                        //String userInput = Serial.readStringUntil('\n');  // Read input
                        for(int i = 0; i < MAX_BUTTONS; i++){
                          String command = readButtons(button_pins[i]);
                          command.trim();  // Remove whitespace or newlines
                          if (command.length() > 0 || i < MAX_BUTTONS) {
                            txCharacteristic.writeValue(command.c_str());  // Send data
                            Serial.println("Sent: " + command);
                          }
                        }
                        delay(250);
                    }
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