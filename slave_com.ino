#define PIN_LED_CLOSE 24
#define PIN_LED_OPEN 23
#define PIN_LED_PRON 22
#define PIN_LED_SUP 21
#define MAX_CODES 4
#define FREQ 5 
// this code writes the message received from the phone (works as slave)
#include <ArduinoBLE.h>  // Include the ArduinoBLE library for Bluetooth Low Energy (BLE) functionality

// Define a BLE service using the Nordic UART Service (NUS) UUID
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

// Define a BLE characteristic for receiving data (RX)
// This allows the phone to send data to the Arduino
BLECharacteristic rxCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", 
                                   BLEWrite | BLEWriteWithoutResponse, 512);

// Define a BLE characteristic for sending data (TX)
// This allows the Arduino to send data back to the phone
BLECharacteristic txCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", 
                                   BLENotify, 512);
// define led freq
//static const int freq = 5; // Hz
// array of button pins
//const int button_pins[MAX_CODES] = {PIN_RS_CLOSE, PIN_RS_OPEN, PIN_RS_PRON, PIN_RS_SUP};
// array of led pins
const int led_pins[MAX_CODES] = {PIN_LED_CLOSE, PIN_LED_OPEN, PIN_LED_PRON, PIN_LED_SUP};
// array of led states
bool led_state[MAX_CODES] = {false,false,false,false}; // Close, Open, Pron, Supi
// COMMANDS (START and STOP could be added)
const String grasp_str = "CLOSE";
const String open_str = "OPEN";
const String pron_str = "PRON";
const String sup_str = "SUPI";
const String STOP = "STOP";
// array of possible codes
const String codes[MAX_CODES] = {grasp_str, open_str, pron_str, sup_str};
// id of the last action
static int id = -1;
void setup() {
    Serial.begin(9600);  // Start the serial monitor with a baud rate of 9600
    while (!Serial);  // Wait for the Serial Monitor to open (only needed on some boards)
    delay(1000);  // Short delay to stabilize Serial output
    Serial.println("Serial monitor ready!");  // Print message to Serial Monitor

    // Initialize BLE module
    if (!BLE.begin()) {
        Serial.println("Failed to initialize BLE!");  // Print error message if BLE fails
        while (1);  // Stop execution if BLE initialization fails
    }

    // Set the name of the BLE device that will appear in scanning apps
    BLE.setLocalName("NanoESP32_BLE");

    // Add the defined BLE service to the device
    BLE.setAdvertisedService(uartService);

    // Add RX and TX characteristics to the service
    uartService.addCharacteristic(rxCharacteristic);
    uartService.addCharacteristic(txCharacteristic);

    // Add the service to the BLE stack
    BLE.addService(uartService);

    // Start advertising the BLE device so it can be discovered
    BLE.advertise();

    Serial.println("BLE UART Ready. Connect using a BLE terminal app.");  // Print status message
    // assign freq
    //freq = 0;
    // Set LED pins as OUTPUT and switches as input pullup
    for (int i = 0; i < MAX_CODES; i++) {
        pinMode(led_pins[i], OUTPUT);
        digitalWrite(led_pins[i], LOW);  // Turn off LEDs initially
    }

}
// toggles the LED state
void toggle_led(int f, int pin_led, bool st){
  int value = LOW;
  if (st) value = HIGH;
  digitalWrite(pin_led, value);
  int t = (1000/f); // time in miliseconds
  delay(t);
}
// returns True if action was properly performed
bool doAction(int idx){
  if(idx > 3 || idx < 0) return false;
  Serial.println("Performing action...");
  //  the idx ranges from 0 to 3
  // 0 -> pin 24, 1-> pin 23 , 2 -> pin 22, 3 -> pin 21 
  led_state[idx] = !led_state[idx]; // change the state of the led dir
  if(id >= 0 ) led_state[id] = !led_state[id]; // turn OFF the last action carried
  // iterate on each led to keep the XOR relation between actions
  for(int i = 0; i < MAX_CODES; i++){
    toggle_led(FREQ, PIN_LED_CLOSE - i, led_state[i]);
  }
  // update last action id
  id = idx;
  return true;
}
/**/
void loop() {
    // Wait for a BLE central device (such as a phone) to connect
    BLEDevice central = BLE.central();
    if (central) {  // If a device is connected
        Serial.print("Connected to: ");
        Serial.println(central.address());  // Print the MAC address of the connected device

        // Keep checking for data while the central device remains connected
        while (central.connected()) {
            if (rxCharacteristic.written()) {  // Check if new data has been received
                int len = rxCharacteristic.valueLength();  // Get the length of received data
                const uint8_t* data = rxCharacteristic.value();  // Get a pointer to the received data
                
                // Convert received bytes into a String
                String receivedText = "";
                for (int i = 0; i < len; i++) {
                    receivedText += (char)data[i];  // Append each byte as a character
                }

                Serial.print("Received: '");
                Serial.print(receivedText);  // Print received data to Serial Monitor 
                Serial.println("'");
                // check if the text is correct
                bool isOk = false;
                for(int i = 0; i < MAX_CODES; i++)
                  if (receivedText == codes[i])
                    isOk = doAction(i);
                // feedback on the user input
                if(!isOk){
                  Serial.println("Wrong command!");
                  Serial.println("Try 'CLOSE' | 'OPEN' | 'PRONE' | 'SUPI' ");
                }
                // Convert the String into a C-style string (null-terminated char array)
                //txCharacteristic.writeValue(receivedText.c_str());  // Send the received data back to the phone
            }
            // 
        }
        Serial.println("Disconnected.");  // Print message when the device disconnects
    }
}
/*
void loop(){
  int st = digitalRead(PIN_RS_CLOSE);

  if (st == LOW){
    digitalWrite(PIN_LED_CLOSE, HIGH);
  }
  else{
    digitalWrite(PIN_LED_CLOSE, LOW);
  }
}
*/