#include <deque>
#include "wheel_shm.h"
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
  // TODO: "name" size(?) offset
{
  shm_port<float> p("ref_angle", 28, 0);
  log_f_ports.push_back(p);
}

}
