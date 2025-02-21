#include <ArduinoBLE.h>

BLEDevice peripheral;  // BLE Peripheral Device (Slave Nano)
BLECharacteristic txCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 512);

void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Master Nano: BLE Central Initialized");

    if (!BLE.begin()) {
        Serial.println("Failed to initialize BLE!");
        while (1);
    }

    Serial.println("Scanning for Slave Nano...");
}

void loop() {
    // Try to find the slave device
    BLE.scanForName("NanoESP32_BLE");  // Match with the advertised name of the slave

    peripheral = BLE.available();
    if (peripheral) {
        Serial.println("Connected to Slave Nano!");
        BLE.stopScan();
        if (peripheral.connect()) {
            Serial.println("Connection successful!");

            // Discover the UART Service and Characteristics
            if (peripheral.discoverService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")) {
                txCharacteristic = peripheral.characteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

                if (txCharacteristic) {
                    Serial.println("Ready to send data!");

                    while (peripheral.connected()) {
                        Serial.print("Enter command: ");
                        while (!Serial.available());  // Wait for user input
                        
                        String userInput = Serial.readStringUntil('\n');  // Read input
                        userInput.trim();  // Remove whitespace or newlines

                        if (userInput.length() > 0) {
                            txCharacteristic.writeValue(userInput.c_str());  // Send data
                            Serial.println("Sent: " + userInput);
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
