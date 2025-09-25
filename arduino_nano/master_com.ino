#include <ArduinoBLE.h>
#include <U8g2lib.h>
#include <Wire.h>
// button pins
#define PIN_RS_CLOSE 6 // D6
#define PIN_RS_OPEN 5 // D5
#define PIN_RS_PRON 4 // D4
#define PIN_RS_SUP 3 // D3
#define PIN_RS_ENABLE 2 // D2

#define FREQ_DATASEND 2000 // send data each 2 seconds
/* SDA and SCL pins for I2C comms
    SDA -> A4
    SCL -> A5
*/
#define MAX_BUTTONS 5
#define MAX_ACTIONS 4

// BLE 
BLEDevice peripheral;  // BLE Peripheral Device (Slave Nano)
BLECharacteristic txCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 512);

// initialize the display (check I2C address)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /*reset*/ U8X8_PIN_NONE);

// array of button pins
const int button_pins[MAX_BUTTONS] = {PIN_RS_CLOSE, PIN_RS_OPEN, PIN_RS_PRON, PIN_RS_SUP, PIN_RS_ENABLE};
// COMMANDS (START and STOP could be added) --> ClsOpnPronSup -> 4 bit code where all of them are XOR
void initialize_display(){
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_8x13B_tr); // Font reference: https://github.com/olikraus/u8g2/wiki/fntlistall
  
  // Start drawing text at the top (x: 0, y: 10)
  int y_position = 10; // Initial Y position for the first word
  
  // Define the text to display
  String text = "Scanning for Bluetooth devices...";
  
  // Split the text into words
  String word = "";
  for (int i = 0; i < text.length(); i++) {
    if (text[i] == ' ' || i == text.length() - 1) { // Space or last character
      if (i == text.length() - 1 && text[i] != ' ') { // If last character is not a space
        word += text[i];
      }
      display.drawStr(0, y_position, word.c_str()); // Draw the word
      y_position += 15; // Move the Y position down for the next word (font height + margin)
      word = ""; // Clear the word string for the next word
    } else {
      word += text[i]; // Add the character to the current word
    }
  }

  display.sendBuffer(); // Send the buffer to the screen
}
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
    // Initialize the OLED display
    initialize_display();
}
// returns a single bit, 1 if the button is ON (LOW) and 0 if the button is OFF (HIGH)
String readButtons(const int &pin) {
    return digitalRead(pin) == LOW ? "1" : "0";
}
// Function to update the display based on the action
void updateDisplayAction(String command, bool waitingForComms) {
  display.clearBuffer();  // Clear the buffer before updating the display
  display.setFont(u8g2_font_8x13B_tr);  // Set font
  
  // Draw "ACTIONS" in the first line, centered
  display.drawStr(64 - (display.getStrWidth("ACTIONS") / 2), 10, "ACTIONS");
  
  // set the font smaller
  display.setFont(u8g2_font_6x13B_tr);
  // Draw the action labels ("STOP", "OPEN", "CLOSE", "PRON", "SUPI") in the second line, evenly spaced
  display.drawStr(5, 30, "STOP");
  display.drawStr(32, 30, "OPN");
  display.drawStr(56, 30, "CLS");
  display.drawStr(80, 30, "PRN");
  display.drawStr(104, 30, "SUP");
  // set the font bigger
  display.setFont(u8g2_font_8x13B_tr);  // Set font
  // Determine the binary values for each action based on the `command` string
  String stop = waitingForComms ? "1" : "0";
  String open = String(command[1]);
  String close = String(command[0]);
  String pron = String(command[2]);
  String supi = String(command[3]);

  // Draw the binary values in the third line, evenly spaced
  display.drawStr(8, 50, stop.c_str());
  display.drawStr(32, 50, open.c_str());
  display.drawStr(56, 50, close.c_str());
  display.drawStr(80, 50, pron.c_str());
  display.drawStr(104, 50, supi.c_str());

  display.sendBuffer();  // Send the buffer to the screen
}
// shows in the oled screen the BLE state connection
void updateBLEConnected(bool connected) {
  display.clearBuffer();  // Clear the display buffer before updating the screen
  display.setFont(u8g2_font_8x13B_tr);  // Set the font for the text
  
  if (connected) {
    // If connected, display the connection success message
    display.drawStr(0, 10, "Slave device");  // First line
    display.drawStr(0, 25, "connected via");  // Second line
    display.drawStr(0, 40, "BLE");  // Third line
  } else {
    // If disconnected, display the disconnection message
    display.drawStr(0, 10, "Slave device");  // First line
    display.drawStr(0, 25, "disconnected");  // Second line
  }
  
  display.sendBuffer();  // Send the updated content to the OLED display
  delay(2000);
}
void loop() {
    // Try to find the slave device
    BLE.scanForName("NanoESP32_BLE");  // Match with the advertised name of the slave

    peripheral = BLE.available();
    static int prev_st = HIGH; // previous state of the enable button
    if (peripheral) {
        Serial.println("Connected to Raspberry Pi 5!");
        BLE.stopScan();
        while (peripheral.connect()) {
            updateBLEConnected(true); // show the successful connection in the display
            Serial.println("Connection successful!");
            // Discover the UART Service and Characteristics
            if (peripheral.discoverService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")) {
                txCharacteristic = peripheral.characteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

                if(txCharacteristic) {
                  while (true) {
                      String waiting = "XXXX";
                      Serial.println("Waiting for comms to be enabled");
                      // update display 
                      updateDisplayAction(waiting, true);
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
                            updateDisplayAction(command, false); // update the display 
                          }
                          // reading frequency
                          delay(FREQ_DATASEND);
                      }
                      updateBLEConnected(false); // show that connection was lost
                      prev_st = digitalRead(PIN_RS_ENABLE); // Actualizar estado al soltar
                  }
                } else {
                    Serial.println("Characteristic not found!");
                }
            } else {
                Serial.println("Service not found!");
            }
        } 
        //else {
        
        Serial.println("Disconnected of the Rpi5");
        delay(1000);
        //}
    }
    delay(1000);  // Short delay before trying again
}
