#include <yaml-cpp/yaml.h>

#include <vector>
#include <iostream>
#include "driver_parameter.h"

//read_string
bool read_value(YAML::Node &node, std::string &key, int *value)
{
  if ( node[key] ) {
    try {
      int i = node[key].as<int>();
      *value = i;
      return true;
    } catch (const std::exception&) {
      return false;
    }
  }
  return false;
}
bool read_value(YAML::Node &node, std::string &key, double *value)
{
  if ( node[key] ) {
    try {
      double d = node[key].as<double>();
      *value = d;
      return true;
    } catch (const std::exception&) {
      return false;
    }
  }
  return false;
}
bool read_value(YAML::Node &node, std::string &key, std::vector<int> &vec)
{
  if ( node[key] ) {
    YAML::Node n = node[key];
    if (n.IsSequence()) {
      int size = n.size();
      for(int i = 0; i < size; i++) {
        try {
          int v = n[i].as<int>();
          vec.push_back(v);
        } catch (const std::exception&) {
          return false;
        }
      }
      return true;
    } else {
      return false;
    }
  }
  return false;
}
bool read_value(YAML::Node &node, std::string &key, std::vector<double> &vec)
{
  if ( node[key] ) {
    YAML::Node n = node[key];
    if (n.IsSequence()) {
      int size = n.size();
      for(int i = 0; i < size; i++) {
        try {
          double v = n[i].as<double>();
          vec.push_back(v);
        } catch (const std::exception&) {
          return false;
        }
      }
      return true;
    } else {
      return false;
    }
  }
  return false;
}

#define macro_read_intvec(name) {                                       \
    std::string key = #name ;                                           \
    std::vector<int> vec;                                               \
    if ( read_value(node, key, vec) ) {                                 \
      if (vec.size() != num) {                                          \
        std::cerr << "key: " << key << " parameter length (" << vec.size() \
                  << ") is not equal " << num << std::endl;             \
        return false;                                                   \
      }                                                                 \
      for(unsigned int i = 0; i < num; i++)                             \
        params[i].name = vec[i];                                      \
    } else {                                                            \
      std::cerr << "key: " << key << " was not found" << std::endl;     \
      return false;                                                     \
    }                                                                   \
  }
#define macro_read_floatvec(name) {                                     \
    std::string key = #name ;                                           \
    std::vector<double> vec;                                            \
    if ( read_value(node, key, vec) ) {                                 \
      if (vec.size() != num) {                                          \
        std::cerr << "key: " << key << " parameter length (" << vec.size() \
                  << ") is not equal " << num << std::endl;             \
        return false;                                                   \
      }                                                                 \
      for(unsigned int i = 0; i < num; i++)                             \
        params[i].name = vec[i];                                      \
    } else {                                                            \
      std::cerr << "key: " << key << " was not found" << std::endl;     \
      return false;                                                     \
    }                                                                   \
  }

bool parse_node(YAML::Node &node, std::vector<driver_parameter> &params)
{
  if (!node.IsMap()) {
    return false;
  }
  unsigned int num = 0;
  {
    std::string key = "number_of_drivers";
    int n;
    if( read_value(node, key, &n) ) {
      num = n;
      params.resize(num);
    } else {
      std::cerr << "key: " << key << " was not found" << std::endl;
      return false;
    }
  }
  macro_read_floatvec(origin_offset_angle);

  macro_read_intvec(shm_id);
  macro_read_intvec(control_mode);
  macro_read_intvec(position_factor);
  macro_read_intvec(direction);

  macro_read_intvec(absolute_2pi_count);
  macro_read_intvec(encoder_2pi_count);

  macro_read_floatvec(rated_current_limit);
  macro_read_floatvec(max_target_current);
  macro_read_floatvec(torque_constant); // Nm / A

  macro_read_floatvec(Pgain);
  macro_read_floatvec(Dgain);
  macro_read_floatvec(Igain);

  macro_read_floatvec(motor_resistance);
  macro_read_floatvec(motor_thermal_resistance);
  macro_read_floatvec(motor_heat_capacity);

  return true;
}
bool parse_driver_parameter_yaml(std::string &fname, std::vector<driver_parameter> &params)
{
  try {
    // check fname
    YAML::Node config = YAML::LoadFile(fname);
    return parse_node(config, params);
  } catch (const std::exception&) {
    std::cerr << "parameter file [" << fname << "] can not open" << std::endl;
  }
  return false;
}
