#include <ODriveUART.h>
#include <config.h>
#define RX2 16
#define TX2 17
#define MAX_REFS 4 // vel_cmd references
using ODrive_t = ODriveUART; 

HardwareSerial odrive_serial(2);
ODrive_t odrive(odrive_serial);

unsigned long baudrate = 115200; // baudrate configured in ODrive
const float vref = 8.0; // rev/s
const float vel_cmd[MAX_REFS] = {0, vref, 0, -vref}; // velocity command
const float total_time = 20000; // (ms) total simulation time
const float interval = 500; // (ms) time between each reference
bool motorCalib; // checks calibration
bool sim_ready = false; // to regulate the simulation
bool sim_stop = false; // to stop the sim
unsigned long simStartTime = 0;
unsigned long lastStepTime = 0;
int idx = 0; // to regulate the vel_cmd sent

void configureODrive(Config& cfg) {
  // Helper lambda para escribir y confirmar
  auto setAndConfirm = [](const String& param, float value) {
    odrive.setParameter(param, value);
    delay(2); // Breve delay para evitar sobrecarga inicial
    Serial.print("Set "); Serial.print(param); Serial.print(" → ");
    Serial.println(odrive.getParameterAsFloat(param)); // Confirmación inmediata
  };

  auto setAndConfirmInt = [](const String& param, int value) {
    odrive.setParameter(param, value);
    delay(2);
    Serial.print("Set "); Serial.print(param); Serial.print(" → ");
    Serial.println(odrive.getParameterAsInt(param));
  };

  // Motor
  setAndConfirmInt("axis0.motor.config.motor_type", cfg.motor_type);
  setAndConfirm("axis0.motor.config.current_lim", cfg.current_lim);
  setAndConfirmInt("axis0.motor.config.pole_pairs", cfg.pole_pairs);
  setAndConfirm("axis0.motor.config.resistance_calib_max_voltage", cfg.resistance_calib_max_voltage);
  setAndConfirm("axis0.motor.config.requested_current_range", cfg.requested_current_range);
  setAndConfirm("axis0.motor.config.current_control_bandwidth", cfg.current_control_bandwidth);
  setAndConfirm("axis0.motor.config.torque_constant", cfg.torque_constant);

  // Encoder
  setAndConfirmInt("axis0.encoder.config.cpr", cfg.encoder_cpr);
  setAndConfirmInt("axis0.encoder.config.mode", cfg.encoder_mode);
  setAndConfirm("axis0.encoder.config.calib_scan_distance", cfg.calib_scan_distance);
  setAndConfirm("axis0.encoder.config.bandwidth", cfg.encoder_bandwidth);
  setAndConfirmInt("axis0.encoder.config.use_index", cfg.use_index);
  setAndConfirmInt("axis0.encoder.config.pre_calibrated", 1);  // true como entero

  // Controlador
  setAndConfirm("axis0.controller.config.pos_gain", cfg.kp);
  setAndConfirm("axis0.controller.config.vel_gain", cfg.kv);
  setAndConfirm("axis0.controller.config.vel_integrator_gain", cfg.ki);
  setAndConfirm("axis0.controller.config.vel_limit", cfg.vel_limit);
  setAndConfirmInt("axis0.controller.config.control_mode", cfg.control_mode);
  setAndConfirmInt("axis0.controller.config.input_mode", cfg.input_mode);

  // General
  setAndConfirm("config.dc_max_negative_current", cfg.dc_max_negative_current);
  setAndConfirmInt("config.enable_brake_resistor", cfg.enable_brake_resistor);

  Serial.println("✅ Configuración enviada desde clase Config");
}

bool doFullCalibration(){
  bool isCalib;
  Serial.println("🚦 Calibrando ODrive...");
  odrive.clearErrors();
  odrive.setState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
  delay(10);

  while (odrive.getState() != AXIS_STATE_IDLE)
    Serial.println("⏳ Calibrando...");

  if (odrive.getParameterAsInt("axis0.motor.error") != 0){
    isCalib = false;
  }
  else{
    isCalib = true;
    Serial.println("✅ Calibración completada");
  }
  return isCalib;
}
  
void setup() {
  Serial.begin(115200); // Serial to PC
  Serial.println("🔌 Iniciando...");
  odrive_serial.begin(baudrate, SERIAL_8N1, RX2, TX2); // UART to ODrive board
  
  delay(10);

  Serial.println("🔍 Esperando ODrive...");
  int st = -1;
  while (st == AXIS_STATE_UNDEFINED || st < 0) {
    st = odrive.getState();
    delay(10);
  }
  Serial.println("✅ ODrive encontrado");
 
  // Configuracion (sobre todo el controlador)
  //Serial.println("⚙️ Configurando ODrive...");
  //Config cfg; // object type ConfigClass 
  //configureODrive(cfg);
  
  // perform both motor and encoder calib  
  odrive.clearErrors();
  odrive.setState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
  delay(10);
  while (odrive.getState() != AXIS_STATE_IDLE)
    Serial.println("⏳ Calibrando...");
  // wait for button push ? 
  Serial.println("Motor calibrated");
  
  Serial.println(odrive.getParameterAsFloat("axis0.controller.config.pos_gain"));
  //Serial.println(odrive.getParameterAsFloat("axis0.controller.config.vel_gain"));
  //Serial.println(odrive.getParameterAsFloat("axis0.controller.config.vel_integrator_gain"));
 
}

void loop() {
  /*
  if (motorCalib && !sim_ready) {
    // Activar control en lazo cerrado solo una vez
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    Serial.println("🚀 Closed loop control activado");

    float kp = odrive.getParameterAsFloat("axis0.controller.config.pos_gain");
    float kv = odrive.getParameterAsFloat("axis0.controller.config.vel_gain");
    float ki = odrive.getParameterAsFloat("axis0.controller.config.vel_integrator_gain");
    Serial.println("📊 Ganancias del controlador: " + String(kp) + ", " + String(kv) + ", " + String(ki));

    // Guardar tiempo de inicio
    simStartTime = millis();
    lastStepTime = millis();
    idx = 0;
    sim_ready = true; // enable sim
  }

  // sim ready to start
  if (motorCalib && sim_ready && !sim_stop) {
    unsigned long currentTime = millis();

    // Verificar si se alcanzó el tiempo total
    if (currentTime - simStartTime >= total_time) {
      Serial.println("✅ Simulación finalizada.");
      odrive.setVelocity(0); // Detener motor
      sim_stop = true;    // Finaliza lógica
      return;
    }

    // Verificar si es hora de cambiar velocidad
    if (currentTime - lastStepTime >= interval) {
      if (idx < MAX_REFS) {
        float cmd = vel_cmd[idx];
        odrive.setVelocity(cmd);
        Serial.print("⏱ t = ");
        Serial.print(currentTime - simStartTime);
        Serial.print(" ms -> Enviando velocidad: ");
        Serial.println(cmd);
        idx++;
        lastStepTime = currentTime;
      }
      else{ // reset the idx to keep period
        idx = 0;
      }
    }
  }
  */
// sim ready to start: button push ??
  if (!sim_stop) {
    unsigned long currentTime = millis();

    // Verificar si se alcanzó el tiempo total
    if (currentTime - simStartTime >= total_time) {
      Serial.println("✅ Simulación finalizada.");
      odrive.setVelocity(0); // Detener motor
      sim_stop = true;    // Finaliza lógica
      return;
    }

    // Verificar si es hora de cambiar velocidad
    if (currentTime - lastStepTime >= interval) {
      if (idx < MAX_REFS) {
        float cmd = vel_cmd[idx];
        odrive.setVelocity(cmd);
        Serial.print("⏱ t = ");
        Serial.print(currentTime - simStartTime);
        Serial.print(" ms -> Enviando velocidad: ");
        Serial.println(cmd);
        idx++;
        lastStepTime = currentTime;
      }
      else{ // reset the idx to keep period
        idx = 0;
      }
    }
  }
}

