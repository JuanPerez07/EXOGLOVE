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
  float torque_constant = 0.031;
  float calibration_current = 5.0;

  // Encoder
  int encoder_cpr = 20480;
  int encoder_mode = 0;
  float calib_scan_distance = 150.0;
  float encoder_bandwidth = 100.0;
  bool use_index = true;
  bool pre_calibrated = false;

  // Controlador
  float kp = 20.0;
  float kv = 0.117;
  float ki = 0.01;
  float vel_limit = 5.0;
  int control_mode = 2;
  int input_mode = 1; // INPUT_MODE_PASSTHROUGH
  float input_filter_bandwidth = 20.0;

  // ODrive generales
  float dc_max_negative_current = -2.0;
  bool enable_brake_resistor = false;

  // Estado de inicio y configuración de arranque
  int requested_state = 3;
  bool startup_motor_calibration = true;
  bool startup_encoder_index_search = false;
  bool startup_encoder_offset_calibration = true;
  bool startup_closed_loop_control = true;
  bool startup_sensorless_control = false;
};

#endif
