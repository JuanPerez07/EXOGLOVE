#include <ODriveUART.h>
#include <config.h> // not used
#define RX2 16
#define TX2 17
#define MAX_REFS 4 // vel_cmd references
using ODrive_t = ODriveUART; 

HardwareSerial odrive_serial(2);
ODrive_t odrive(odrive_serial);

unsigned long baudrate = 115200; // baudrate configured in ODrive
const float vref = 0.5; // rev/s
const float vel_cmd[MAX_REFS] = {0, vref, 0, -vref}; // velocity command
const float total_time = 200000; // (ms) total simulation time
const float interval = 1000; // (ms) time between each reference
bool sim_ready = false; // to regulate the simulation
bool sim_stop = false; // to stop the sim
unsigned long simStartTime = 0;
unsigned long lastStepTime = 0;
unsigned long calibTime = 0;
int idx = 0; // to regulate the vel_cmd sent

  
void setup() {
  Serial.begin(115200); // Serial to PC
  Serial.println("🔌 Iniciando...");
  odrive_serial.begin(baudrate, SERIAL_8N1, RX2, TX2); // UART to ODrive board
  
  delay(10);

  Serial.println("🔍 Esperando ODrive...");
  int st = -1;
  while (st == AXIS_STATE_UNDEFINED || st < 0) {
    st = odrive.getState();
    delay(1000);
  }
  Serial.println("✅ ODrive encontrado");
 
  // perform both motor and encoder calib  
  odrive.clearErrors();
  odrive.setState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
  delay(10);
  while (odrive.getState() != AXIS_STATE_IDLE){
    Serial.println("⏳ Calibrando...");
    delay(1000);
  }
  sim_ready = true;
  // wait for button push ? 
  Serial.println("Motor calibrated!");
  odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  Serial.println("🚀 Closed loop control activado");
  calibTime = millis(); // tiempo invertido en setup
}

void loop() {
// sim ready to start: button push ??
  if (sim_ready && !sim_stop) {
    unsigned long currentTime = millis() - calibTime; // tiempo real tras calib
    // Verificar si se alcanzó el tiempo total
    if (currentTime - simStartTime >= total_time) {
      Serial.println("✅ Simulación finalizada.");
      odrive.setVelocity(0); // Detener motor
      sim_stop = true;    // Finaliza lógica
    }

    // Verificar si es hora de cambiar velocidad
    if (currentTime - lastStepTime >= interval) {
      // check any axis errors
      int err = odrive.getParameterAsInt("axis0.motor.error");
      if (err != 0){
        Serial.println("Codigo de error: " + String(err)); 
        sim_stop = true;
        int motor_error = odrive.getParameterAsInt("axis0.motor.error");
        int encoder_error = odrive.getParameterAsInt("axis0.encoder.error");

        Serial.printf("❌ Motor Error: 0x%08X\n", motor_error);
        Serial.printf("❌ Encoder Error: 0x%08X\n", encoder_error);
      }
      // send references to the velocity controller
      if (idx < MAX_REFS) {
        float cmd = vel_cmd[idx];
        odrive.setVelocity(cmd);
        Serial.print("⏱ t = ");
        Serial.print(currentTime - simStartTime);
        Serial.print(" ms -> Enviando velocidad: ");
        Serial.println(cmd);
        idx++;
      }
      else{ // reset the idx to keep period
        idx = 0;
        //sim_stop = true;
        Serial.println("🔁 Fin de un periodo.");
      }
      // actualizar ultimo step 
     lastStepTime = currentTime;
    }
  }
}

