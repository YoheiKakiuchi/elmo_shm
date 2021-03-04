#include "elmo_common.h"

int jsk_elmo_settings(int dev_no,
                      uint8_t control_mode,
                      int cycle_timeout, /* unit: 10us, e.g. cycle_timeout=200 => 2000us(2ms) */
                      int aux_position,
                      bool debug) /* debug print */
{
  // SETTING : control mode
  while(1)  {
    int ret = 0;
    // position control
    // set control mode 0x6060 <=: 0x08 (cyclic synchronous position)
    // torque control
    // set control mode 0x6060 <=: 0x0a (cyclic synchronous torque)
    uint8_t ctl_mode = control_mode;
    ret += ec_SDOwrite(dev_no, 0x6060, 0x00, FALSE, sizeof(ctl_mode), &ctl_mode, EC_TIMEOUTRXM);
    if (ret == 1) break;
  }

  // TODO: using cycle_timeout
  // SETTING : cycle timeout // servo may stop if there is no command within this time period. (just guess??)
  while(1) {
    int ret = 0;
    // set intrepolation time period 0x60c2:01 <=: 200
    // set intrepolation time period 0x60c2:02 <=:  -5
    //   time period ==> 200*10^-5 sec (2ms)
    uint8_t tm_period1 = cycle_timeout;
    ret += ec_SDOwrite(dev_no, 0x60C2, 0x01, FALSE, sizeof(tm_period1), &tm_period1, EC_TIMEOUTRXM);
    if (ret == 1) break;
  }
  while(1) {
    int ret = 0;
    int8_t tm_period2 = -5;
    ret += ec_SDOwrite(dev_no, 0x60C2, 0x02, FALSE, sizeof(tm_period2), &tm_period2, EC_TIMEOUTTXM);
    if (ret == 1) break;
  }

  if (debug) {
    fprintf(stderr, "DB: start-read\n");
    while(1) {
      int ret = 0;
      int psize = 1;
      char val;
      ret += ec_SDOread (dev_no, 0x60C2, 0x02, FALSE, &psize, &val, EC_TIMEOUTRXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x602C:2 -> %d\n", dev_no, val);
        break;
      }
    }
    while(1) {
      int ret = 0;
      int psize = 1;
      char val;
      ret += ec_SDOread (dev_no, 0x20B0, 0x00, FALSE, &psize, &val, EC_TIMEOUTRXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x02B0:0 -> %d\n", dev_no, val);
        break;
      }
    }
  }

  // TODO: using aux_position
  // SETTING (Auxiliary position)
  /*
    Object 0x20B0 : Socket Additional Function
    Sub-index      / 9
    Description    / Socket used for Additional Sensor 0x20A0 read out, CA[79]
    Entry category / Mandatory
    Access         / Read/Write
    PDO mapping    / No
    Value range    / 0...4
    Default value  / 1
  */
  while(1) {
    int ret = 0;
    int setting = aux_position;
    ret += ec_SDOwrite(dev_no, 0x20B0, 0x09, FALSE, sizeof(setting), &setting, EC_TIMEOUTTXM);
    if (ret == 1) break;
  }

  if (debug) {
    while(1) {
      int ret = 0;
      int psize = 4;
      int val;
      ret += ec_SDOread (dev_no, 0x20B0, 0x09, FALSE, &psize, &val, EC_TIMEOUTRXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x02B0:9 -> %d\n", dev_no, val);
        break;
      }
    }
  }

  /*
    Object 0x2F75 – Extrapolation Cycles Timeout
    The object can be accessed by the OV[63] command.
    Name        / Extrapolation Cycles Timeout
    Object code / VAR
    Data type   / INTEGER16
    Category    / Optional
    Entry description:
    Access      / Read/Write
    PDO mapping / No
    Value range / 1...32767
    Default value 1
   */
  while(1) {
    int ret = 0;
    short setting = 6; //
    ret += ec_SDOwrite(dev_no, 0x2F75, 0x00, FALSE, sizeof(setting), &setting, EC_TIMEOUTTXM);
    if (ret == 1) break;
  }

  if (debug) {
    while(1) {
      int ret = 0;
      int psize = 2;
      short val;
      ret += ec_SDOread (dev_no, 0x2F75, 0x00, FALSE, &psize, &val, EC_TIMEOUTRXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x2F75 -> %d\n", dev_no, val);
        break;
      }
    }
  }

  return 0;
}

int elmo_scale_factor_settings(int dev_no,
                               int position_factor,
                               bool debug) /* debug print */
{
  // position factor
  while(1) {
    int ret = 0;
    unsigned int factor = position_factor; //
    ret += ec_SDOwrite(dev_no, 0x6092, 0x01, FALSE, sizeof(factor), &factor, EC_TIMEOUTTXM);
    if (ret == 1) break;
  }
#if 0 // not required
  // velocity factor
  while(1) {
    int ret = 0;
    unsigned int factor = velocity_factor; //
    ret += ec_SDOwrite(dev_no, 0x6096, 0x01, FALSE, sizeof(factor), &factor, EC_TIMEOUTTXM);
    if (ret == 1) break;
  }
#endif
  if (debug) {
    while(1) {
      int ret = 0;
      int psize = 4;
      unsigned int factor; //
      ret += ec_SDOread (dev_no, 0x6092, 0x01, FALSE, &psize, &factor, EC_TIMEOUTTXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x6092.1 -> %d\n", dev_no, factor);
        break;
      }
    }
    while(1) {
      int ret = 0;
      int psize = 4;
      unsigned int factor; //
      ret += ec_SDOread (dev_no, 0x6096, 0x01, FALSE, &psize, &factor, EC_TIMEOUTTXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x6096.1 -> %d\n", dev_no, factor);
        break;
      }
    }
  }

  return 0;
}

int elmo_current_limit_settings(int dev_no,
                                float peak_current,
                                float peak_duration,
                                bool debug)
{
  if (peak_current > 0.0) {
    while(1) {
      int ret = 0;
      float val = peak_current;
      ret += ec_SDOwrite(dev_no, 0x3191, 0x01, FALSE, sizeof(val), &val, EC_TIMEOUTTXM);
      if (ret == 1) break;
    }
  }
  if (peak_duration > 0.0) {
    while(1) {
      int ret = 0;
      float val = peak_duration;
      ret += ec_SDOwrite(dev_no, 0x3191, 0x02, FALSE, sizeof(val), &val, EC_TIMEOUTTXM);
      if (ret == 1) break;
    }
  }
  if (debug) {
    while(1) {
      int ret = 0;
      int psize = 4;
      float val;
      ret += ec_SDOread (dev_no, 0x303F, 0x01, FALSE, &psize, &val, EC_TIMEOUTTXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x303F.1(continuous current) -> %f\n", dev_no, val);
        break;
      }
    }
    while(1) {
      int ret = 0;
      int psize = 4;
      float val;
      ret += ec_SDOread (dev_no, 0x3191, 0x01, FALSE, &psize, &val, EC_TIMEOUTTXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x3191.1(peak current) -> %f\n", dev_no, val);
        break;
      }
    }
    while(1) {
      int ret = 0;
      int psize = 4;
      float val;
      ret += ec_SDOread (dev_no, 0x3191, 0x02, FALSE, &psize, &val, EC_TIMEOUTTXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x3191.2(peak duration) -> %f\n", dev_no, val);
        break;
      }
    }
  }

  return 0;
}

int elmo_control_parameter(int dev_no,
                           bool debug)
{
  if (debug) {
    while(1) {
      int ret = 0;
      int psize = 4;
      float val;
      // KI 2 velocity integral
      ret += ec_SDOread (dev_no, 0x310C, 0x02, FALSE, &psize, &val, EC_TIMEOUTTXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x310C.2(velocity integral) -> %f\n", dev_no, val);
        break;
      }
    }
    while(1) {
      int ret = 0;
      int psize = 4;
      float val;
      // KP 2 velocity gain
      ret += ec_SDOread (dev_no, 0x3113, 0x02, FALSE, &psize, &val, EC_TIMEOUTTXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x3113.2(velocity gain) -> %e\n", dev_no, val);
        break;
      }
    }
    while(1) {
      int ret = 0;
      int psize = 4;
      float val;
      // KP 3 position gain
      ret += ec_SDOread (dev_no, 0x3113, 0x03, FALSE, &psize, &val, EC_TIMEOUTTXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x3113.3(position gain) -> %f\n", dev_no, val);
        break;
      }
    }
    while(1) {
      int ret = 0;
      int psize = 4;
      float val;
      // FF 1 (acc feedforward)
      ret += ec_SDOread (dev_no, 0x3087, 0x01, FALSE, &psize, &val, EC_TIMEOUTTXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x3087.1(acc feedforward) -> %e\n", dev_no, val);
        break;
      }
    }
    while(1) {
      int ret = 0;
      int psize = 4;
      float val;
      // FF 2 (vel feedforward)
      ret += ec_SDOread (dev_no, 0x3087, 0x02, FALSE, &psize, &val, EC_TIMEOUTTXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x3087.1(vel feedforward) -> %f\n", dev_no, val);
        break;
      }
    }
  }

  return 0;
}

int elmo_read_error(int dev_no, bool debug)
{
  int size = 0;
  while(1) {
    int ret = 0;
    int psize = 1;
    unsigned char val;
    ret += ec_SDOread (dev_no, 0x1003, 0x00, FALSE, &psize, &val, EC_TIMEOUTTXM);
    if (ret == 1) {
      fprintf(stderr, "dev[%d]: 0x1003(error code) -> %X\n", dev_no, val);
      size = val;
      break;
    }
  }
  for (int i = 1; i <= size; i++) {
    while(1) {
      int ret = 0;
      int psize = 4;
      unsigned int val;
      ret += ec_SDOread (dev_no, 0x1003, i, FALSE, &psize, &val, EC_TIMEOUTTXM);
      if (ret == 1) {
        fprintf(stderr, "dev[%d]: 0x1003/%d (error code) -> %X\n", dev_no, i, val);
        break;
      }
    }
  }
  return 0;
}
int jsk_elmo_PDO_mapping(int dev_no, uint16_t *rxpdo_list, int rxpdo_num,
                         uint16_t *txpdo_list, int txpdo_num)
{
  /*
    setting PDO mapping
  */
  while(1) { // rxpdo clear settings
    int ret = 0;
    uint16_t num_pdo = 0;
    ret += ec_SDOwrite(dev_no, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
    if (ret == 1) break;
  }
  while(1) { // txpdo clear settings
    int ret = 0;
    uint16_t num_pdo = 0;
    ret += ec_SDOwrite(dev_no, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
    if (ret == 1) break;
  }

  // setting rxpdo (device receive)
  for (int idx = 0; idx < rxpdo_num; idx++) {
    uint16_t pdo_idx = rxpdo_list[idx];
    uint8_t  idx_pos = (uint8_t)(idx+1);
    while(1) {
      int ret = 0;
      ret += ec_SDOwrite(dev_no, 0x1c12, idx_pos, FALSE, sizeof(pdo_idx), &pdo_idx, EC_TIMEOUTRXM);
      if (ret == 1) {
        // error
        break;
      }
    }
  }
  while(1) {
    int ret = 0;
    uint16_t num_pdo = (uint16_t)rxpdo_num;
    ret += ec_SDOwrite(dev_no, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
    if (ret == 1) {
      //error
      break;
    }
  }
  // end of setting rxpdo

  // setting txpdo (device send)
  for (int idx = 0; idx < txpdo_num; idx++) {
    uint16_t pdo_idx = txpdo_list[idx];
    uint8_t  idx_pos = (uint8_t)(idx+1);
    while(1) {
      int ret = 0;
      ret += ec_SDOwrite(dev_no, 0x1c13, idx_pos, FALSE, sizeof(pdo_idx), &pdo_idx, EC_TIMEOUTRXM);
      if (ret == 1) {
        // error
        break;
      }
    }
  }
  while(1) {
    int ret = 0;
    uint16_t num_pdo = (uint16_t)txpdo_num;
    ret += ec_SDOwrite(dev_no, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
    if (ret == 1) {
      // error
      break;
    }
  }
  // end of setting txpdo

  return 0;
}
