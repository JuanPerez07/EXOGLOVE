#ifndef CONFIG_H
#define CONFIG_H

class Config {
public:
  // Motor
  int motor_type = 0;
  float current_lim = 10.0;
  int pole_pairs = 7;
  float resistance_calib_max_voltage = 2.0;
  float requested_current_range = 25.0;
  float current_control_bandwidth = 100.0;
  float torque_constant = 0.05;

  // Encoder
  int encoder_cpr = 20480;
  int encoder_mode = 0;
  float calib_scan_distance = 50.0;
  float encoder_bandwidth = 100.0;
  bool use_index = false;

  // Controlador
  float kp = 20.0;
  float kv = 1.0;
  float ki = 0.1;
  float vel_limit = 10.0;
  int control_mode = 2;
  int input_mode = 1; // INPUT_MODE_PASSTHROUGH

  // ODrive generales
  float dc_max_negative_current = -2.0;
  bool enable_brake_resistor = false;
};

#endif
