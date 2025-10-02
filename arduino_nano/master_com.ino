#include <ArduinoBLE.h>
#include <U8g2lib.h>
#include <Wire.h>

// button pins
#define PIN_RS_CLOSE 6 // D6
#define PIN_RS_OPEN 5 // D5
#define PIN_RS_PRON 4 // D4
#define PIN_RS_SUP 3 // D3
#define PIN_RS_ENABLE 2 // D2

#define FREQ_DATASEND 250 // send data each 250 ms
#define MAX_BUTTONS 5
#define MAX_ACTIONS 4

// BLE 
BLEDevice peripheral;  // BLE Peripheral Device (Slave Nano)
BLECharacteristic txCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 512);

// initialize the display (check I2C address)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /*reset*/ U8X8_PIN_NONE);

// array of button pins
const int button_pins[MAX_BUTTONS] = {PIN_RS_CLOSE, PIN_RS_OPEN, PIN_RS_PRON, PIN_RS_SUP, PIN_RS_ENABLE};

// ---------- FUNCIONES DE PANTALLA ----------
void showMessageOLED(String msg) {
  display.clearBuffer();
  display.setFont(u8g2_font_8x13B_tr);

  // Partir el texto en varias líneas si es largo
  int y = 15;
  String line = "";
  for (int i = 0; i < msg.length(); i++) {
    if (msg[i] == '\n' || i == msg.length() - 1) {
      if (i == msg.length() - 1 && msg[i] != '\n') line += msg[i];
      display.drawStr(0, y, line.c_str());
      y += 15;
      line = "";
    } else {
      line += msg[i];
    }
  }

  display.sendBuffer();
}

void initialize_display(){
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_8x13B_tr);
  display.drawStr(0, 15, "Scanning for");
  display.drawStr(0, 30, "Bluetooth devices...");
  display.sendBuffer();
}

// ---------- FUNCIONES DE BOTONES Y ESTADOS ----------
String readButtons(const int &pin) {
  return digitalRead(pin) == LOW ? "1" : "0";
}

void updateDisplayAction(String command, bool waitingForComms) {
  display.clearBuffer();  
  display.setFont(u8g2_font_8x13B_tr);  
  display.drawStr(64 - (display.getStrWidth("ACTIONS") / 2), 10, "ACTIONS");
  
  display.setFont(u8g2_font_6x13B_tr);
  display.drawStr(5, 30, "STOP");
  display.drawStr(32, 30, "OPN");
  display.drawStr(56, 30, "CLS");
  display.drawStr(80, 30, "PRN");
  display.drawStr(104, 30, "SUP");

  display.setFont(u8g2_font_8x13B_tr);  
  String stop = waitingForComms ? "1" : "0";
  String open = String(command[1]);
  String close = String(command[0]);
  String pron = String(command[2]);
  String supi = String(command[3]);

  display.drawStr(8, 50, stop.c_str());
  display.drawStr(32, 50, open.c_str());
  display.drawStr(56, 50, close.c_str());
  display.drawStr(80, 50, pron.c_str());
  display.drawStr(104, 50, supi.c_str());

  display.sendBuffer();  
}

void updateBLEConnected(bool connected) {
  display.clearBuffer();  
  display.setFont(u8g2_font_8x13B_tr);  
  
  if (connected) {
    display.drawStr(0, 10, "Slave device");
    display.drawStr(0, 25, "connected via");
    display.drawStr(0, 40, "BLE");
  } else {
    display.drawStr(0, 10, "Slave device");
    display.drawStr(0, 25, "disconnected");
  }
  
  display.sendBuffer();  
  delay(2000);
}

// ---------- SETUP ----------
void setup() {
  initialize_display();
  if (!BLE.begin()) {
      showMessageOLED("Failed to init BLE!");
      while (1);
  }
  for (int i = 0; i < MAX_BUTTONS; i++) {
      pinMode(button_pins[i], INPUT_PULLUP);
  }
}

// ---------- LOOP ----------
void loop() {
  BLE.scanForName("NanoESP32_BLE");  

  peripheral = BLE.available();
  static int prev_st = HIGH; 

  if (peripheral) {
      showMessageOLED("Connected to\nRaspberry Pi 5!");
      BLE.stopScan();
      while (peripheral.connect()) {
          updateBLEConnected(true);
          showMessageOLED("Connection\nsuccessful!");

          if (peripheral.discoverService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")) {
              txCharacteristic = peripheral.characteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

              if(txCharacteristic) {
                while (true) {
                    String waiting = "XXXX";
                    showMessageOLED("Waiting for\ncomms to be\nenabled...");
                    updateDisplayAction(waiting, true);

                    while (digitalRead(PIN_RS_ENABLE) == prev_st);
                    prev_st = digitalRead(PIN_RS_ENABLE);

                    showMessageOLED("Ready to\nsend data!");
                    
                    while (peripheral.connected() && digitalRead(PIN_RS_ENABLE) == prev_st) {
                        String command = "";
                        for (int i = 0; i < MAX_ACTIONS; i++) {
                            command += readButtons(button_pins[i]);
                            command.trim();
                        }
                        if (!command.isEmpty()) {
                          txCharacteristic.writeValue(command.c_str());
                          updateDisplayAction(command, false); 
                        }
                        delay(FREQ_DATASEND);
                    }
                    updateBLEConnected(false); 
                    prev_st = digitalRead(PIN_RS_ENABLE); 
                }
              } else {
                  showMessageOLED("Characteristic\nnot found!");
              }
          } else {
              showMessageOLED("Service not\nfound!");
          }
      }
      showMessageOLED("Disconnected\nof Rpi5");
      delay(100);
  }
  delay(100); 
}
