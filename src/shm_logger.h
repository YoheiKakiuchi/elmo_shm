#include <deque>
#include "servo_shm.h"
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

template <typename T> class shm_port {
  std::string  suffix;
  //std::ostream *os;
  std::ofstream *os;

  int size;
  int offset;
  int step;

public: shm_port(std::string suf, int in_size, int in_offset, int in_step) : suffix(suf), size(in_size), offset(in_offset), step(in_step)
  { }

public: shm_port(std::string suf, int in_size, int in_offset) : shm_port(suf, in_size, in_offset, 1) { }

public:
  void open(std::string &fname) {
    std::string name = fname;
    name.append(".");
    name.append(suffix);

    std::cerr << "open : " << name << std::endl;
    os = new std::ofstream (name.c_str());

    if (!os->is_open()) {
      std::cerr << "failed to open " << name << std::endl;
    }
  }

  void close() {
    os->close();
  }

  void print(servo_shm &shm) {
    T* pt = (T *)(((long)(void *)&shm) + offset);

    *os << (shm.frame) * 0.001 << " ";
    for(int i = 0; i < size; i++) {
      *os << *pt << " ";
      pt += step;
    }
    *os << std::endl;
  }
};

std::vector<shm_port <float> > log_f_ports;
std::vector<shm_port <int> >   log_i_ports;
std::vector<shm_port <char> >  log_c_ports;

class shm_logger {

public: shm_logger () {
  _m_maxLength = 1000 * 300;
}

public:
  void clear () {
    //std::cerr << "clear log" << std::endl;
    _m_log.clear();
  }
  void log(servo_shm *shm) {
    //std::cerr << "log0 " << _m_log.size() << std::endl;
    _m_log.push_back(*shm);
    //std::cerr << "log1 " << _m_log.size() << std::endl;
    while (_m_log.size() > _m_maxLength) {
      _m_log.pop_front();
    }
  }
  void open (std::string &fname) {
    for(unsigned int i = 0; i < log_f_ports.size(); i++) {
      log_f_ports[i].open(fname);
    }
    for(unsigned int i = 0; i < log_i_ports.size(); i++) {
      log_i_ports[i].open(fname);
    }
    for(unsigned int i = 0; i < log_c_ports.size(); i++) {
      log_c_ports[i].open(fname);
    }
  }
  void close (std::string &fname) {
    for(unsigned int i = 0; i < log_f_ports.size(); i++) {
      log_f_ports[i].close();
    }
    for(unsigned int i = 0; i < log_i_ports.size(); i++) {
      log_i_ports[i].close();
    }
    for(unsigned int i = 0; i < log_c_ports.size(); i++) {
      log_c_ports[i].close();
    }
  }
  void save (std::string &fname) {
    std::cerr << "save : " << fname << ", size = " << _m_log.size() << std::endl;
    if (_m_log.size() == 0) {
      std::cerr << "do not save log" << std::endl;
      return;
    }
    open(fname);
    for(unsigned int j = 0; j < _m_log.size(); j++) {
      servo_shm &shm = _m_log[j];
      for(unsigned int i = 0; i < log_f_ports.size(); i++) {
        log_f_ports[i].print(shm);
      }
      for(unsigned int i = 0; i < log_i_ports.size(); i++) {
        log_i_ports[i].print(shm);
      }
      for(unsigned int i = 0; i < log_c_ports.size(); i++) {
        log_c_ports[i].print(shm);
      }
    }
    close(fname);
  }
public:
  std::deque<servo_shm> _m_log;
  long _m_maxLength;
};

void create_logger ()
{
{
  shm_port<float> p("ref_angle", 28, 0);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("cur_angle", 28, 256);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("abs_angle", 28, 512);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("ref_vel", 28, 768);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("cur_vel", 28, 1024);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("ref_torque", 28, 1280);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("cur_torque", 28, 1536);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("pgain", 28, 1792);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("dgain", 28, 2048);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("torque_pgain", 28, 2304);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("torque_dgain", 28, 2560);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("prev_angle", 28, 12040);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("abs_vel", 28, 12552);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("motor_temp_0", 28, 4096);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("motor_temp_1", 28, 4352);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("motor_outer_temp_0", 28, 4608);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("motor_outer_temp_1", 28, 4864);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("motor_current_0", 28, 5120);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("motor_current_1", 28, 5376);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("motor_output_0", 28, 5632);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("motor_output_1", 28, 5888);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("board_vin_0", 28, 6144);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("board_vin_1", 28, 6400);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("board_vdd_0", 28, 6656);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("board_vdd_1", 28, 6912);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("h817_rx_error0_0", 28, 7680);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("h817_rx_error0_1", 28, 7936);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("h817_rx_error1_0", 28, 8192);
  log_f_ports.push_back(p);
}
{
  shm_port<float> p("h817_rx_error1_1", 28, 8448);
  log_f_ports.push_back(p);
}
{
  shm_port<int> p("motor_num", 28, 3840);
  log_i_ports.push_back(p);
}
{
  shm_port<int> p("controlmode", 28, 9576);
  log_i_ports.push_back(p);
}
{
  shm_port<int> p("joint_offset", 28, 11264);
  log_i_ports.push_back(p);
}
{
  shm_port<int> p("interpolation_counter", 28, 12296);
  log_i_ports.push_back(p);
}
{
  shm_port<int> p("comm_normal_0", 28, 7168);
  log_i_ports.push_back(p);
}
{
  shm_port<int> p("comm_normal_1", 28, 7424);
  log_i_ports.push_back(p);
}
{
  shm_port<int> p("servo_state_0", 28, 10088);
  log_i_ports.push_back(p);
}
{
  shm_port<int> p("servo_state_1", 28, 10344);
  log_i_ports.push_back(p);
}
{
  shm_port<int> p("hole_status_0", 28, 10684);
  log_i_ports.push_back(p);
}
{
  shm_port<int> p("hole_status_1", 28, 10940);
  log_i_ports.push_back(p);
}
{
  shm_port<char> p("is_servo_on", 28, 9832);
  log_c_ports.push_back(p);
}
{
  shm_port<char> p("servo_on", 28, 9896);
  log_c_ports.push_back(p);
}
{
  shm_port<char> p("servo_off", 28, 9960);
  log_c_ports.push_back(p);
}
{
  shm_port<char> p("torque0", 28, 10024);
  log_c_ports.push_back(p);
}
{
  shm_port<char> p("loopback", 28, 10620);
  log_c_ports.push_back(p);
}
{
  shm_port<char> p("joint_enable", 28, 11198);
  log_c_ports.push_back(p);
}

}
