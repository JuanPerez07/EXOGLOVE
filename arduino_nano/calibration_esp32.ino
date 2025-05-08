#include <ODriveUART.h>
// Test calibracion usando protocolo UART2 del ESP32
#define RX2 16
#define TX2 17
// Documentation for this example can be found here:
// https://docs.odriverobotics.com/v/latest/guides/arduino-uart-guide.html


////////////////////////////////
// Set up serial pins to the ODrive
///////////////////////////////

// Using UART2 from ESP32
// pin 16: RX - connect to ODrive TX
// pin 17: TX - connect to ODrive RX
HardwareSerial odrive_serial(2);
unsigned long baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
// HardwareSerial& odrive_serial = Serial1;
// int baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
// HardwareSerial& odrive_serial = Serial1;
// int baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)


ODriveUART odrive(odrive_serial);

void setup() {
  Serial.println("Program Start");
  odrive_serial.begin(baudrate, SERIAL_8N1, RX2, TX2);

  Serial.begin(115200); // Serial to PC
  
  delay(10);

  Serial.println("Waiting for ODrive...");
  int st = -1;
  while (st == AXIS_STATE_UNDEFINED || st < 0 ) {
    st = odrive.getState();
    delay(100);
  }

  Serial.println("found ODrive");
  
  Serial.print("DC voltage: ");
  Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
  
  Serial.println("Setting full calib sequence...");
  
  odrive.clearErrors();
  odrive.setState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
  delay(10);
  while(odrive.getState() != AXIS_STATE_IDLE)
    Serial.println("Calibrating...");
  
  Serial.println("ODrive calibrated!");
}

void loop() {
  /*
  float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

  float t = 0.001 * millis();
  
  float phase = t * (TWO_PI / SINE_PERIOD);
  
  odrive.setPosition(
    sin(phase), // position
    cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
  );

  ODriveFeedback feedback = odrive.getFeedback();
  Serial.print("pos:");
  Serial.print(feedback.pos);
  Serial.print(", ");
  Serial.print("vel:");
  Serial.print(feedback.vel);
  Serial.println();
  */
}
