#ifndef DRIVER_PARAMETER_H
#define DRIVER_PARAMETER_H

#include <vector>
typedef struct driver_parameter {
  // Settings
  int control_mode;
  int position_factor;
  int direction;

  int absolute_2pi_count;
  int encoder_2pi_count;

  double origin_offset_angle;

  double rated_current_limit;
  double max_target_current;
  double torque_constant; // Nm / A

  double Pgain;
  double Dgain;
  double Igain;

  double motor_resistance;
  double motor_thermal_resistance;
  double motor_heat_capacity;
} driver_parameter;

typedef struct driver : driver_parameter {
  // Buffer
  int id;
  bool configured;
  double abs_count_to_radian;
  double enc_count_to_radian;
  int absolute_origin_count;
  int encoder_origin_count;
  double integral_value;
  double motor_temp;
  double motor_energy;

  driver &operator= (const driver_parameter &o)
  {
    driver_parameter::operator=(o);
    return *this;
  }
} driver;

extern bool parse_driver_parameter_yaml(std::string &fname, std::vector<driver_parameter> &params);

#endif /* DRIVER_PARAMETER_H */
