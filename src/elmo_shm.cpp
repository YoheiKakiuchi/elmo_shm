/**
MIT License

Copyright (c) 2021 Yohei Kakiuchi (youhei@jsk.imi.i.u-tokyo.ac.jp)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <pthread.h>

// for parsing option
#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include "realtime_task.h"    // elmo_shm/include
#include "elmo_common.h"      // elmo_shm/include
#include "driver_parameter.h" // elmo_shm/include
#include <errno.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <sys/time.h> // gettimeofday
#include <signal.h>

#include <vector>
#include <string>
#include <cmath>
#include <memory> // shared_ptr
#include "IIRFilter.h"
#include "servo_shm.h"

extern void *display_thread_fun (void *arg);
extern void exit_with_no(int);

// log
#include "shm_logger.h"
shm_logger shm_log;

//
uint16_t rxpdo_set[] = {0x160F, 0x160C, 0x1618, 0x160A, 0x160B};
uint16_t txpdo_set[] = {0x1A0E, 0x1A11, 0x1A1E, 0x1A1F, 0x1A18,
                        0x1A22, 0x1A21,
                        0x1A0A, 0x1A0B};

#pragma pack(2)
typedef struct _rxpdo_buffer {
  int32 target_position; //0x160F 0x607A s32 Target Position
  //  int32 target_velocity; //0x161C 0x60FF s32 Target Velocity
  //  int32 digital_output;  //0x161D 0x60FE:1 s32 Digital Output
  int16 target_torque;   //0x160C 0x6071 s16 Target Torque
  int16 torque_offset;   //0x1618 0x60B2 16 Torque Offset
  uint16 control_word;   //0x160A 0x6040 u16 Control Word
  uint8  mode_of_op;     //0x160B 0x6060 u8  Mode Of Operation
} rxpdo_buffer;
typedef struct _txpdo_buffer {
  int32 position_actual; //0x1A0E 0x6064 32 Position actual value
  int32 velocity_actual; //0x1A11 0x606C 32 Velocity actual value
  int32 aux_position;    //0x1A1E 0x20A0 32 Auxiliary position actual value
  //  uint32 digital_input;  //0x1A1C 0x60FD 32 Digital Inputs
  //  int16 torque_actual;   //0x1A13 0x6077 16 Torque actual value
  int16 current_actual;  //0x1A1F 0x6078 16 Current actual value
  // int16 analog_input;    //0x1A1D 0x2205 16 Analog input
  // int16 torque_demand;   //0x1A12 0x6074 16 Torque demand value
  uint32 dc_volt;      //0x1A18 0x6079 32 DC link circuit voltage
  uint32 elmo_status;  //0x1A22 0x1002 32 ELMO Status Register
  uint16 extra_status; //0x1A21 0x2085 16 Extra Status Register
  uint16 status_word;    //0x1A0A 0x6041 16 Status word
  uint8 mode_of_op;      //0x1A0B 0x6061 8 Mode of operation display
  } txpdo_buffer;
#pragma pack(0)

#define MAX_WORKER_NUM 15
//
//#define REALTIME_CYCLE 250 // 250us
//#define REALTIME_CYCLE 500 // 500us
#define REALTIME_CYCLE 1000 // 1000us

// for jsk_single_axis settings
#define MY_PI 3.141592653589793116

#define MOTOR_START_TEMP 50  // degree celsius
#define MOTOR_MAX_TEMP  120  // degree celsius with safty gap (MAX: 155 deg celsius)
#define ATMOSPHERE_TEMP  25  // degree celsius

enum SHM_STATE {
  SHM_DEFAULT,
  SHM_SERVO_ON,
  SHM_SERVO_OFF,
  SHM_IN_OPERATION,
  SHM_WAITING,
};
typedef enum SHM_STATE shm_state;

//// SOEM global variables
char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;

// TODO: should be thread_safe
bool needlf;
volatile int wkc;
bool inOP = false;
uint8 currentgroup = 0;

//// global variables
volatile int stop_flag = 0;
bool calibration_mode = false;
bool use_hrpsys = true;
std::vector<driver_parameter> drv_params;
long zero_count = 0;

typedef std::shared_ptr<IIRFilter> IIRFilterPtr;
std::vector<IIRFilterPtr> pos_filters;
std::vector<IIRFilterPtr> vel_filters;

//// gloabl variables for display (should be thread_safe)
double jitter = 0;
double max_interval = 0;
servo_shm *shm;

double m0_jitter  = 0;
double m0_max_int = 0;
double m0_min_int = 0;
double m1_jitter  = 0;
double m1_max_int = 0;
double m1_min_int = 0;

/*
ethercatmain.h:431:extern ecx_contextt  ecx_context;
ethercatmain.h:433:extern ec_slavet   ec_slave[EC_MAXSLAVE];
ethercatmain.h:435:extern int         ec_slavecount;
ethercatmain.h:437:extern ec_groupt   ec_group[EC_MAXGROUP];
ethercatmain.h:438:extern boolean     EcatError;
ethercatmain.h:439:extern int64       ec_DCtime;
*/
//
// initialize_elmo
// control_elmo
void *set_shared_memory(key_t _key, size_t _size)
{
  int  shm_id;
  void *ptr;
  int  err;
  // First, try to allocate more memory than needed.
  // If this is the first shmget after reboot or
  // valid size of memory is already allocated,
  // shmget will succeed.
  // If the size of memory allocated is less than
  // _size*2,  shmget will fail.
  // e.g. Change the servo_shm.h then _size may increase.
  size_t size = _size * 2;
  key_t key   = _key;
  shm_id = shmget(key, size, 0666|IPC_CREAT);
  err    = errno;
  if(shm_id == -1 && err == EINVAL) {
    // if fail, retry with _size
    size   = _size;
    shm_id = shmget(key, size, 0666|IPC_CREAT);
    err    = errno;
  }
  if(shm_id == -1) {
    fprintf(stderr, "shmget failed, key=%d, size=%ld, errno=%d (%s)\n", key, size, err, strerror(err));
    return NULL;
  }
  ptr = (struct shared_data *)shmat(shm_id, (void *)0, 0);
  if(ptr == (void *)-1) {
    int err = errno;
    fprintf(stderr, "shmget failed, key=%d, size=%ld, shm_id=%d, errno=%d (%s)\n", key, size, shm_id, err, strerror(err));
    return NULL;
  }
  //fprintf(stderr, "shmget ok, size=%d\n", size);
  return ptr;
}

bool initialize_ethercat (const char *ifname)
{
  /* initialise SOEM, bind socket to ifname */
  if ( !ec_init(ifname) ) {
    fprintf(stderr, "No socket connection on %s\nExcecute as root\n",ifname);
    return false;
  }
  fprintf(stderr, "ec_init on %s succeeded.\n", ifname);

  /* find and auto-config slaves */
  if ( ec_config_init(FALSE) <= 0 )  {
    fprintf(stderr, "No slaves found!\n");
    ec_close();
    return false;
  }
  fprintf(stderr, "%d workers found and configured.\n", ec_slavecount);
  return true;
}

void ethercat_loop (const char *ifname)
{
  bool ethercat_open = true;
  if (!initialize_ethercat(ifname)) {
    // error
    ethercat_open = false;
    ec_slavecount = MAX_WORKER_NUM;
    //return;
  }
  if (ec_slavecount > MAX_WORKER_NUM) {
    fprintf(stderr, "ec_slavecount(%d) is lerger than MAX_WORKER_NUM(= %d)\n",
            ec_slavecount, MAX_WORKER_NUM);
  }
  std::vector<driver> drv(ec_slavecount);

  for(int workerid = 1, cntr = 0; workerid <= ec_slavecount ; workerid++) {
    std::string name;
    if (ethercat_open) {
      name = std::string (ec_slave[workerid].name);
    } else {
      name = "";
    }
    int cur_id = workerid-1;
    if (name == "GX-JC03") { // EtherCAT Hub
      drv[cur_id].configured = false;
      drv[cur_id].id = -1;
    } else {
      if ( cntr >= drv_params.size() ) {
        drv[cur_id].configured = false;
        drv[cur_id].id = -1;
        continue;
      }
      drv[cur_id] = drv_params[cntr];

      drv[cur_id].absolute_2pi_count *= drv[cur_id].position_factor;
      //drv[cur_id].encoder_2pi_count *= drv[cur_id].position_factor;
      drv[cur_id].Pgain *= 1000 / ( drv[cur_id].torque_constant * drv[cur_id].rated_current_limit);
      drv[cur_id].Dgain *= 1000 / ( drv[cur_id].torque_constant * drv[cur_id].rated_current_limit);
      drv[cur_id].Igain *= 1000 / ( drv[cur_id].torque_constant * drv[cur_id].rated_current_limit);
      fprintf(stderr, "%d P: %f, D: %f, I: %f\n", cur_id,
              drv[cur_id].Pgain, drv[cur_id].Dgain, drv[cur_id].Igain);
      drv[cur_id].motor_resistance         *= 1.5; // オーム
      drv[cur_id].motor_thermal_resistance *= 1.3; // K/W, 1.3 is a safty ratio
      drv[cur_id].motor_heat_capacity      *= 0.8; // J/K, 0.8 is a safty ratio

      // fill buffer
      drv[cur_id].id = cntr;
      drv[cur_id].configured = true;

      drv[cur_id].abs_count_to_radian = (2 * MY_PI) / drv[cur_id].absolute_2pi_count;
      drv[cur_id].enc_count_to_radian = (2 * MY_PI) / drv[cur_id].encoder_2pi_count;

      drv[cur_id].absolute_origin_count =
        (drv[cur_id].origin_offset_angle / drv[cur_id].abs_count_to_radian)
        * drv[cur_id].direction;

      drv[cur_id].integral_value = 0.0;

      drv[cur_id].motor_temp   = MOTOR_START_TEMP;  // degree celsius
      drv[cur_id].motor_energy = MOTOR_START_TEMP * drv[cur_id].motor_heat_capacity; // J

      //
      vel_filters.push_back(IIRFilterPtr( new IIRFilter("")));
      // parameter for 40Hz low pass / 1KHz sample
      // octave --no-gui
      // pkg load signal
      // output_precision(16)
      // [b, a] = butter(3, 2*40*0.001)
      std::vector<double> A(4);
      std::vector<double> B(4);
#if 1      // 80Hz
      A[0] =  1.000000000000000e+00;
      A[1] = -2.003797477370017e+00;
      A[2] =  1.447054019489380e+00;
      A[3] = -3.617959282278672e-01;
      B[0] =  1.018257673643694e-02;
      B[1] =  3.054773020931083e-02;
      B[2] =  3.054773020931083e-02;
      B[3] =  1.018257673643694e-02;
#endif
#if 0      // 40Hz
      A[0] = 1.0;
      A[1] = -2.498608344691178e+00;
      A[2] =  2.115254127003158e+00;
      A[3] = -6.041096995072749e-01;
      B[0] = 1.567010350588269e-03;
      B[1] = 4.701031051764808e-03;
      B[2] = 4.701031051764808e-03;
      B[3] = 1.567010350588269e-03;
#endif
#if 0      // 28 Hz
      A[0] =  1.000000000000000e+00;
      A[1] = -2.648584482072109e+00;
      A[2] =  2.356243849450552e+00;
      A[3] = -7.030581159474827e-01;
      B[0] = 5.751564288701216e-04;
      B[1] = 1.725469286610365e-03;
      B[2] = 1.725469286610365e-03;
      B[3] = 5.751564288701216e-04;
#endif
#if 0      //20Hz
      A[0] =  1.000000000000000e+00;
      A[1] = -2.748835809214675e+00;
      A[2] =  2.528231219142559e+00;
      A[3] = -7.776385602380802e-01;
      B[0] =  2.196062112253621e-04;
      B[1] =  6.588186336760862e-04;
      B[2] =  6.588186336760862e-04;
      B[3] =  2.196062112253621e-04;
#endif
#if 0      // 14Hz
      A[0] =  1.000000000000000e+00;
      A[1] = -2.824127155861567e+00;
      A[2] =  2.663381387731544e+00;
      A[3] = -8.386297075830645e-01;
      B[0] =  7.806553586407756e-05;
      B[1] =  2.341966075922327e-04;
      B[2] =  2.341966075922327e-04;
      B[3] =  7.806553586407756e-05;
#endif
#if 0      // 10Hz
      A[0] =  1.000000000000000e+00;
      A[1] = -2.874356892677485e+00;
      A[2] =  2.756483195225695e+00;
      A[3] = -8.818931305924856e-01;
      B[0] =  2.914649446569765e-05;
      B[1] =  8.743948339709296e-05;
      B[2] =  8.743948339709296e-05;
      B[3] =  2.914649446569765e-05;
#endif
      vel_filters[cntr]->setParameter(3, A, B);

      cntr++;
    }
  }

  //// device settings for all drivers
  //// assume: all workers may be a elmo driver and all elmo driver should be the same settings
  for(int workerid = 1; workerid <= ec_slavecount ; workerid++) {
    /// can we check is it elmo?
    if(!drv[workerid-1].configured) continue;

    if(!ethercat_open) continue;
    ///
    jsk_elmo_settings(workerid,
                      10, // default_control_mode
                      (REALTIME_CYCLE/10),
                      1, // aux position := socket1
                      false); // debug
    ///
    elmo_scale_factor_settings(workerid, drv[workerid-1].position_factor, true);

    ///
    elmo_current_limit_settings(workerid, -1.0, -1.0, true);

    ///
    elmo_control_parameter(workerid, true);

    ///
    elmo_read_error(workerid, true);

    ///
    jsk_elmo_PDO_mapping(workerid, rxpdo_set, sizeof(rxpdo_set)/sizeof(uint16_t),
                         txpdo_set, sizeof(txpdo_set)/sizeof(uint16_t));
  }
#if 0
  {
    rxpdo_buffer rp;
    txpdo_buffer tp;

    fprintf(stderr, "rp.target_torque %d\n", (long)(&(rp.target_torque)) - (long)(&rp));
    fprintf(stderr, "rp.control_word %d\n", (long)(&(rp.control_word)) - (long)(&rp));
    fprintf(stderr, "rp.mode_of_op %d\n", (long)(&(rp.mode_of_op)) - (long)(&rp));

    fprintf(stderr, "tp.position_actual %d\n", (long)(&(tp.position_actual)) - (long)(&tp));
    fprintf(stderr, "tp.velocity_actual %d\n", (long)(&(tp.velocity_actual)) - (long)(&tp));
    fprintf(stderr, "tp.aux_position %d\n", (long)(&(tp.aux_position)) - (long)(&tp));
    fprintf(stderr, "tp.torque_actual %d\n", (long)(&(tp.torque_actual)) - (long)(&tp));
    fprintf(stderr, "tp.current_actual %d\n", (long)(&(tp.current_actual)) - (long)(&tp));
    fprintf(stderr, "tp.status_word %d\n", (long)(&(tp.status_word)) - (long)(&tp));
    fprintf(stderr, "tp.mode_of_op %d\n", (long)(&(tp.mode_of_op)) - (long)(&tp));
  }
#endif
  //// try to go to operational state...
  osal_usleep(200*1000);// just for debug
  ec_config_map(&IOmap); // OR overlap...

  osal_usleep(300*1000);// just for debug
  ec_configdc();

  osal_usleep(100*1000);// just for debug

  // to STATE(SAFE_OP)
  fprintf(stderr, "Workers mapped, state to SAFE_OP.\n");

  /* wait for all Workers to reach SAFE_OP state */
  ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

  fprintf(stderr, "Request operational state for all worker\n");
  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  fprintf(stderr, "Calculated workcounter %d\n", expectedWKC);

  // to STATE(OPERATIONAL)
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  /* send one valid process data to make outputs in slaves happy*/
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  /* request OP state for all slaves */
  ec_writestate(0);

  int chk = 40;
  /* wait for all slaves to reach OP state */
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  }
  while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  osal_usleep(250*1000);// just for debug

  if (ec_slave[0].state != EC_STATE_OPERATIONAL )  {
    //// Error
    fprintf(stderr, "Not all slaves reached operational state.\n");
    ec_readstate();
    for(int workerid = 1; workerid <= ec_slavecount ; workerid++) {
      if(ec_slave[workerid].state != EC_STATE_OPERATIONAL) {
        fprintf(stderr, "Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
               workerid, ec_slave[workerid].state, ec_slave[workerid].ALstatuscode,
               ec_ALstatuscode2string(ec_slave[workerid].ALstatuscode));
      }
    }
    fprintf(stderr, "\nRequest init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    /* request INIT state for all slaves */
    ec_writestate(0);

    osal_usleep(100*1000);// just for debug

    fprintf(stderr, "End ethercat_loop, close socket\n");
    /* stop SOEM, close socket */
    ec_close();

    return;
  }
  //// succesfully reached to operational state...

  // loop
  inOP = true;

  ec_send_processdata();
  wkc = ec_receive_processdata(EC_TIMEOUTRET);
  {
    // initial data
    txpdo_buffer *rx_obj = (txpdo_buffer *)(ec_slave[0].inputs);
    rxpdo_buffer *tx_obj = (rxpdo_buffer *)(ec_slave[0].outputs);
    for(int workerid = 1; workerid <= ec_slavecount ; workerid++) {
      if (!drv[workerid-1].configured) continue;
      driver &cur_drv = drv[workerid-1];
      int cur_id = cur_drv.id;
      int shm_id = cur_drv.shm_id;
      txpdo_buffer *a_rx_obj = & (rx_obj[cur_id]);
      rxpdo_buffer *a_tx_obj = & (tx_obj[cur_id]);

      if (calibration_mode) {
        // TODO
      }
#if 0
      //// for incremental encoder -> add to parameters
      if (cur_id == 6 || cur_id == 13) {
        cur_drv.absolute_origin_count = a_rx_obj->position_actual;
      }
#endif
      { // initialize absolute count
        if (!calibration_mode) {
          fprintf(stderr, "%d => act: %d, abs_orig: %d\n", cur_id,
                  a_rx_obj->position_actual, cur_drv.absolute_origin_count);
        }

        double tmp_0 = fabs(((a_rx_obj->position_actual) -
                             cur_drv.absolute_origin_count)
                            * ( cur_drv.abs_count_to_radian ));
        double tmp_p = fabs(((a_rx_obj->position_actual) -
                             (cur_drv.absolute_origin_count + cur_drv.absolute_2pi_count))
                            * ( cur_drv.abs_count_to_radian ));
        double tmp_m = fabs(((a_rx_obj->position_actual) -
                             (cur_drv.absolute_origin_count - cur_drv.absolute_2pi_count))
                            * ( cur_drv.abs_count_to_radian ));
        if (tmp_m < tmp_p) {
          if (tmp_m < tmp_0) {
            // tmp_m
            cur_drv.absolute_origin_count =
              (cur_drv.absolute_origin_count - cur_drv.absolute_2pi_count);
          }
        } else if (tmp_p < tmp_0) {
            // tmp_p
            cur_drv.absolute_origin_count =
              (cur_drv.absolute_origin_count + cur_drv.absolute_2pi_count);
        }
        if (!calibration_mode) {
          fprintf(stderr, "%d =>   %f %f %f\n", cur_id, tmp_0, tmp_p, tmp_m);
        }
      }

      double cur_abs = ((a_rx_obj->position_actual) - cur_drv.absolute_origin_count) * ( cur_drv.abs_count_to_radian );
      shm->abs_angle[shm_id]  = cur_abs * cur_drv.direction;
      shm->ref_angle[shm_id]  = shm->abs_angle[shm_id];
      // check initial absolute count
      int  cur_count = (a_rx_obj->aux_position);
      cur_drv.encoder_origin_count = cur_count - (int)(cur_abs / ( cur_drv.enc_count_to_radian ));
      double cur_ang = ((a_rx_obj->aux_position) - cur_drv.encoder_origin_count) * ( cur_drv.enc_count_to_radian );
      shm->cur_angle[shm_id] = cur_ang * cur_drv.direction;
    }
    if (calibration_mode) {
      exit(-1);
    }
  }

  // for measuring cycle time
  realtime_task::IntervalStatics m0(0);
  realtime_task::IntervalStatics m1(0);
  m0.reset();
  m1.reset();

  // setting realtime-loop
  long counter = 0;
  realtime_task::Context rt_context(REALTIME_PRIO_MAX, REALTIME_CYCLE);
  //txpdo_buffer *rx_obj = (txpdo_buffer *)malloc(sizeof(txpdo_buffer) * ec_slavecount);
  //rxpdo_buffer *tx_obj = (rxpdo_buffer *)malloc(sizeof(rxpdo_buffer) * ec_slavecount);
  while (inOP) {
    m0.start(false);
    // send data
    ec_send_processdata();
    // receive data
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    //
    m0.sync();

    if(wkc < expectedWKC) {
      fprintf(stderr, "wkc(%d) is less than expected(%d)\n", wkc, expectedWKC);
      continue;
    }
    // data process
    txpdo_buffer *rx_obj = (txpdo_buffer *)(ec_slave[0].inputs);
    rxpdo_buffer *tx_obj = (rxpdo_buffer *)(ec_slave[0].outputs);
    //memcpy(rx_obj, ec_slave[0].inputs,  sizeof(txpdo_buffer)*ec_slavecount);
    //memcpy(tx_obj, ec_slave[0].outputs, sizeof(rxpdo_buffer)*ec_slavecount);

    bool zero_exist = false;
    for(int workerid = 1; workerid <= ec_slavecount ; workerid++) {
      if (!drv[workerid-1].configured) continue;
      driver &cur_drv = drv[workerid-1];
      int cur_id = cur_drv.id;
      int shm_id = cur_drv.shm_id;
      txpdo_buffer *a_rx_obj = & (rx_obj[cur_id]);
      rxpdo_buffer *a_tx_obj = & (tx_obj[cur_id]);

      // START: single joint process
      shm_state shm_st = SHM_DEFAULT;
      if (shm->servo_off[shm_id]) {
        shm_st = SHM_SERVO_OFF;
        shm->servo_on[shm_id] = 0;
      } else
      if (shm->servo_on[shm_id]) {
        //fprintf(stderr, "servo.on\n");
        shm_st = SHM_SERVO_ON;
      } else
      if (shm->is_servo_on[shm_id]) {
        shm_st = SHM_IN_OPERATION;
      } else {
        shm_st = SHM_WAITING;
      }
      elmo_state st = elmo_state_machine(a_rx_obj->status_word);
      switch(st) {
      case S_NOT_READY_TO_SWITCH_ON:
        // do nothing
        break;
      case S_SWITCH_ON_DISABLED:
        if (shm_st == SHM_SERVO_ON) {
          // trans 2
          elmo_control_command(&(a_tx_obj->control_word), C_SHUTDOWN);
          shm->prev_angle[shm_id] = shm->ref_angle[shm_id] = shm->cur_angle[shm_id];
          shm->interpolation_counter[shm_id] = 0;
        } else if (shm_st == SHM_SERVO_OFF) {
          shm->servo_off[shm_id]   = 0;
          shm->is_servo_on[shm_id] = 0;
        }
        break;
      case S_READY_TO_SWITCH_ON:
        if (shm_st == SHM_SERVO_ON) {
          // trans 3
          //elmo_control_command(&(a_tx_obj->control_word), C_SWITCH_ON);
          elmo_control_command(&(a_tx_obj->control_word), C_SWITCH_ON_AND_ENABLE_OPERATION);
        } else if (shm_st == SHM_SERVO_OFF) {
          // trans 7
          elmo_control_command(&(a_tx_obj->control_word), C_DISABLE_VOLTAGE);
        }
        shm->is_servo_on[shm_id] = 0;
        break;
      case S_SWITCHED_ON:
        if (shm_st == SHM_SERVO_ON) {
          // trans 4
          elmo_control_command(&(a_tx_obj->control_word), C_ENABLE_OPERATION);
        } else if (shm_st == SHM_SERVO_OFF) {
          // trans 10
          elmo_control_command(&(a_tx_obj->control_word), C_DISABLE_VOLTAGE);
        }
        shm->is_servo_on[shm_id] = 0;
        break;
      case S_OPERATION_ENABLED:
        if (shm_st == SHM_SERVO_ON) {
          shm->servo_on[shm_id] = 0;
          elmo_control_command(&(a_tx_obj->control_word), C_ENABLE_OPERATION);
        } else if (shm_st == SHM_SERVO_OFF) {
          // trans 5
          elmo_control_command(&(a_tx_obj->control_word), C_DISABLE_OPERATION);
        } else if (shm_st == SHM_WAITING) {
          // error
        } else {
          // in operation
          elmo_control_command(&(a_tx_obj->control_word), C_ENABLE_OPERATION);
        }
        shm->is_servo_on[shm_id] = 1;
        break;
      case S_QUICK_STOP_ACTIVE:
        shm->is_servo_on[shm_id] = 0;
        shm->servo_on[shm_id]    = 0;
        shm->servo_off[shm_id]   = 0;
        // trans 12
        elmo_control_command(&(a_tx_obj->control_word), C_DISABLE_VOLTAGE);
        break;
      case S_FAULT_REACTION_ACTIVE:
        shm->is_servo_on[shm_id] = 0;
        // do nothing
        break;
      case S_FAULT:
        if (shm_st == SHM_SERVO_ON) {
          // trans 15
          elmo_control_command(&(a_tx_obj->control_word), C_FAULT_RESET);
        }
        if (shm_st == SHM_SERVO_OFF) {
          // trans 15
          elmo_control_command(&(a_tx_obj->control_word), C_FAULT_RESET);
        }
        shm->is_servo_on[shm_id] = 0;
        break;
      case S_DETECT_ERROR:
        // do nothing
        break;
      }

      //
      // operation command for each joint
      //
      shm->servo_state[0][shm_id] = 0x00; // this will be sent to hrpsys
      shm->hole_status[0][shm_id] = (uint32)(a_rx_obj->status_word << 16) | (uint32)a_rx_obj->extra_status;
      shm->servo_state[1][shm_id] = (int)a_rx_obj->elmo_status;
      double cur_ang = ((a_rx_obj->aux_position) - cur_drv.encoder_origin_count) * ( cur_drv.enc_count_to_radian );
      cur_ang *= cur_drv.direction;
      shm->cur_angle[shm_id] = cur_ang;

      shm->board_vdd[0][shm_id] = (a_rx_obj->dc_volt)*0.001;
      // absolute
      double cur_abs = ((a_rx_obj->position_actual) - cur_drv.absolute_origin_count) * ( cur_drv.abs_count_to_radian );
      double abs_vel = (a_rx_obj->velocity_actual) * ( cur_drv.abs_count_to_radian );
      cur_abs *= cur_drv.direction;
      abs_vel *= cur_drv.direction;
      shm->abs_angle[shm_id] = cur_abs;
      shm->abs_vel[shm_id]   = abs_vel;

      double cur_tq  = a_rx_obj->current_actual * cur_drv.torque_constant;
      cur_tq *= cur_drv.direction;
      shm->cur_torque[shm_id] = cur_tq;
      double cur_cur = ((a_rx_obj->current_actual) * cur_drv.rated_current_limit) * 0.001; // current [A]
      cur_cur *= cur_drv.direction;
      shm->motor_current[0][shm_id] = cur_cur;
      shm->motor_current[1][shm_id] = cur_cur / cur_drv.rated_current_limit;
      // shm->motor_output[0][shm_id]  = cur_cur * cur_drv.direction;

      // motor temp estimation
      {
        cur_drv.motor_energy += cur_cur * cur_cur * cur_drv.motor_resistance * (REALTIME_CYCLE/1000000.0);
        cur_drv.motor_energy -= (cur_drv.motor_temp - ATMOSPHERE_TEMP)/cur_drv.motor_thermal_resistance * (REALTIME_CYCLE/1000000.0);

        cur_drv.motor_temp = cur_drv.motor_energy / cur_drv.motor_heat_capacity;

        shm->motor_temp[0][shm_id] = cur_drv.motor_temp;
      }

      //a_tx_obj->mode_of_op = shm->controlmode[shm_id];
      a_tx_obj->mode_of_op = (cur_drv.control_mode & 0x0F);
      a_tx_obj->target_torque = 0;
      a_tx_obj->target_position =
        (a_rx_obj->position_actual / cur_drv.position_factor) * cur_drv.position_factor;
      a_tx_obj->torque_offset = 0;

      if (shm->is_servo_on[shm_id]) {
        // zero count
        if (a_rx_obj->current_actual == 0) {
          zero_exist = true;
        }
        double actual_ref = shm->ref_angle[shm_id];
        double dt_act_ref = 0.0;
        // simple seq using counter
        if (shm->interpolation_counter[shm_id] > 0) {
          actual_ref = shm->prev_angle[shm_id] + ((shm->ref_angle[shm_id] - shm->prev_angle[shm_id]) / (1 + shm->interpolation_counter[shm_id]));
          dt_act_ref = ((actual_ref - shm->prev_angle[shm_id]) * 1000000.0) / REALTIME_CYCLE;
          shm->prev_angle[shm_id] = actual_ref;
          shm->interpolation_counter[shm_id]--;
        } else {
          dt_act_ref = ((actual_ref - shm->prev_angle[shm_id]) * 1000000.0) / REALTIME_CYCLE;
          shm->prev_angle[shm_id] = actual_ref;
        }
        // debug using shm_log
        //shm->prev_angle[shm_id + 14] = actual_ref;

        //// feedback process
        /// USE absolute for feedback
        //double diff_ang = (shm->abs_angle[shm_id] - shm->ref_angle[shm_id]);
        double diff_ang = (shm->abs_angle[shm_id] - actual_ref);
        double vel      = (shm->abs_vel[shm_id] - dt_act_ref);
        //double vel      = (shm->abs_vel[shm_id]);

        // loopback mode
        if (use_hrpsys && shm->loopback[shm_id]) {
          shm->ref_angle[shm_id] = shm->abs_angle[shm_id];
          diff_ang = 0;
          vel = 0;
          cur_drv.integral_value = 0;
        }

        double pgain = cur_drv.Pgain;
        double dgain = cur_drv.Dgain;
        double igain = cur_drv.Igain;
        { // set gain on hrpsys as 0.0 to 1.0
          pgain *= shm->pgain[shm_id];
          dgain *= shm->dgain[shm_id];
          igain *= shm->pgain[shm_id];
          //fprintf(stderr, "%d  gain: %f %f => %f %f",
          //shm_id, shm->pgain[shm_id], shm->dgain[shm_id], pgain, dgain);
        }

        if (cur_drv.control_mode == 0x08) {
        //// POSITION CONTROL mode
        if (fabs(diff_ang) > 0.174) { //
          if ((shm_id != 6) && (shm_id != 13)) {
          fprintf(stderr, "very large difference %d abs:%f ref:%f\n",
                  shm_id, shm->abs_angle[shm_id], shm->ref_angle[shm_id]);
          exit_with_no(1);
          } else {
#if 0
            if (fabs(diff_ang) > (4 * 0.174)) {
              fprintf(stderr, "very large difference %d abs:%f ref:%f\n",
                      shm_id, shm->abs_angle[shm_id], shm->ref_angle[shm_id]);
              shm->loopback[shm_id] = 1;
            }
#endif
          }
        }

        if (use_hrpsys && (shm->pgain[shm_id] < 0.99999)) {
          a_tx_obj->target_position =
            (a_rx_obj->position_actual / cur_drv.position_factor) * cur_drv.position_factor;
          continue;
        }

        long pos_com = (long)((actual_ref * cur_drv.direction) / cur_drv.abs_count_to_radian) + (long)cur_drv.absolute_origin_count;
        a_tx_obj->target_position = (int32)pos_com;
        if (shm->loopback[shm_id]) {
          a_tx_obj->target_position =
            (a_rx_obj->position_actual / cur_drv.position_factor) * cur_drv.position_factor;
        }
        //
        shm->joint_offset[shm_id] = (int32)pos_com;
        //shm->joint_offset[shm_id+14] = a_rx_obj->position_actual;
        //
#if 0 // add torque_offset for compensating friction torque
        if (shm_id == 6 || shm_id == 13) {
          if(dt_act_ref == 0.0) continue;
#define torque_Ts 1400.0
#define torque_Tc 1000.0
#define torque_D  0.01
#define velocity_Vstr 0.1
          double tq_vel = shm->abs_vel[shm_id];
          if (tq_vel == 0.0) {
            //vel
            if (dt_act_ref > 0) {
              a_tx_obj->torque_offset = (int)torque_Ts * cur_drv.direction;
            } else if (dt_act_ref < 0) {
              a_tx_obj->torque_offset = - (int)torque_Ts * cur_drv.direction;
            }
          } else {
            double off_tq = torque_Tc + (torque_Ts - torque_Tc) * std::exp(- (tq_vel/velocity_Vstr) * (tq_vel/velocity_Vstr));
            if (dt_act_ref > 0) {
              off_tq =   off_tq + torque_D * tq_vel;
            } else {
              off_tq = - off_tq + torque_D * tq_vel;
            }
            a_tx_obj->torque_offset = (int)off_tq * cur_drv.direction;
          }
        }
#endif
        continue;

        } else
        if (cur_drv.control_mode == 0x0a) {
        //// TORQUE CONTROL mode

        if (fabs(diff_ang) > 0.174) { // TODO: error detection
          if ((shm_id != 6) && (shm_id != 13)) {
          fprintf(stderr, "very large difference %d abs:%f ref:%f\n",
                  shm_id, shm->abs_angle[shm_id], shm->ref_angle[shm_id]);
          exit(1);
          } else {
#if 0
            if (fabs(diff_ang) > (4 * 0.174)) {
              fprintf(stderr, "very large difference %d abs:%f ref:%f\n",
                      shm_id, shm->abs_angle[shm_id], shm->ref_angle[shm_id]);
              shm->loopback[shm_id] = 1;
            }
#endif
          }
        }

        // debug for using shm_log
        //shm->ref_vel[shm_id] = dt_act_ref;
        //shm->ref_angle[14 + shm_id] = - (diff_ang * pgain);
        //shm->ref_vel[14 + shm_id]   = - (vel * dgain);

        cur_drv.integral_value += diff_ang;
        if (cur_drv.integral_value >  3.14) cur_drv.integral_value =  3.14;
        if (cur_drv.integral_value < -3.14) cur_drv.integral_value = -3.14;

        double target_cur = - (diff_ang * pgain) - (vel * dgain) - (igain * cur_drv.integral_value);
        int cur_limit = (cur_drv.max_target_current/cur_drv.rated_current_limit)*1000.0;

        if (cur_drv.motor_temp > MOTOR_MAX_TEMP) {
          cur_limit = 700; // rated_current
        }
        if (target_cur >=   cur_limit) target_cur =   cur_limit;
        if (target_cur <= - cur_limit) target_cur = - cur_limit;

        // debug for using shm_log
        //shm->cur_torque[14 + shm_id] = target_cur;

        //fprintf(stderr, ", out: %d\n", (int)(target_cur * cur_drv.direction));
        a_tx_obj->target_torque = (int16)(target_cur * cur_drv.direction);
        //
        shm->motor_output[0][shm_id]  = target_cur * 0.001 * cur_drv.rated_current_limit;

        //
        } else
        if (cur_drv.control_mode == 0x1a) {
          // target torque mode
          double target_cur = 1000.0 * (shm->ref_torque[shm_id] / cur_drv.rated_current_limit);
          int cur_limit = (cur_drv.max_target_current/cur_drv.rated_current_limit)*1000.0;

          if (cur_drv.motor_temp > MOTOR_MAX_TEMP) {
            cur_limit = 700; // rated_current
          }
          if (target_cur >=   cur_limit) target_cur =   cur_limit;
          if (target_cur <= - cur_limit) target_cur = - cur_limit;

          a_tx_obj->target_torque = (int16)(target_cur * cur_drv.direction);
          shm->motor_output[0][shm_id]  = target_cur * 0.001 * cur_drv.rated_current_limit;

        //
        } else
        if (cur_drv.control_mode == 0x2a) {
          // target velocity mode
          double ref_vel = shm->ref_vel[shm_id];
          // filter
          double filtered_vel = vel_filters[shm_id]->passFilter(abs_vel);

          double target_cur = (ref_vel - filtered_vel) * dgain;
          int cur_limit = (cur_drv.max_target_current/cur_drv.rated_current_limit)*1000.0;

          if (cur_drv.motor_temp > MOTOR_MAX_TEMP) {
            cur_limit = 700; // rated_current
          }
          if (target_cur >=   cur_limit) target_cur =   cur_limit;
          if (target_cur <= - cur_limit) target_cur = - cur_limit;

          a_tx_obj->target_torque = (int16)(target_cur * cur_drv.direction);
          shm->motor_output[0][shm_id]  = target_cur * 0.001 * cur_drv.rated_current_limit;

        //
        } else {
          // control mode error
        }
      }
      // END: single joint process
    }
    // data process end

    jitter       = rt_context.statistics_get_norm();
    max_interval = rt_context.statistics_get_max_interval();
    m0_jitter = m0.get_norm();
    m1_jitter = m1.get_norm();
    m0_max_int = m0.get_max_interval();
    m1_max_int = m1.get_max_interval();
    m0_min_int = m0.get_min_interval();
    m1_min_int = m1.get_min_interval();
    if( counter % 1000 == 0 ) {
      rt_context.statistics_reset();
      m0.reset();
      m1.reset();
    }
    shm->frame = counter++;
    //
    if (shm->disable_alert_on_servo_on == 1) {
      shm_log.log(shm);
    } else if (shm->disable_alert_on_servo_on == 2) {
      shm_log.clear();
      shm->disable_alert_on_servo_on = 1;
    }
    //zero count
    if(zero_exist) {
      zero_count++;
    } else {
      zero_count = 0;
    }
    if (zero_count > 5000) {
      fprintf(stderr, "many zero count\n");
      exit(1);
    }
    //
    m1.start(false);
    rt_context.wait(); // real-time look (keep cycle)
    m1.sync();
  }
}

/* threading this func */
//// TODO: Is it thread safe ???
OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
  int slave;
  (void)ptr;                  /* Not used */

  while (1) {
    if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
      if (needlf) {
        needlf = FALSE;
        fprintf(stderr, ">ck\n");
      }

      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate(); ////

      for (slave = 1; slave <= ec_slavecount; slave++) {
        if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            fprintf(stderr, "ck> ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          } else if(ec_slave[slave].state == EC_STATE_SAFE_OP) {
            fprintf(stderr, "ck> WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          } else if(ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              fprintf(stderr, "ck> MESSAGE : slave %d reconfigured\n",slave);
            }
          } else if(!ec_slave[slave].islost) {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
              ec_slave[slave].islost = TRUE;
              fprintf(stderr, "ck> ERROR : slave %d lost\n",slave);
            }
          }
        }

        if (ec_slave[slave].islost) {
          if(ec_slave[slave].state == EC_STATE_NONE) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              fprintf(stderr, "ck> MESSAGE : slave %d recovered\n",slave);
            }
          } else {
            ec_slave[slave].islost = FALSE;
            fprintf(stderr, "ck> MESSAGE : slave %d found\n",slave);
          }
        }
      } /* for ... */
      if(!ec_group[currentgroup].docheckstate) {
        fprintf(stderr, "ck> OK : all slaves resumed OPERATIONAL.\n");
      }
    } /* if ( inOP */
    osal_usleep(10000);
  } /* while(1) */
}

std::string logfilename("shm_log");

void exit_with_no(int exitno) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  fprintf(stderr, "exit %ld.%06ld\n", tv.tv_sec, tv.tv_usec);
  shm_log.save(logfilename);
  exit(exitno);
}
void cchandler(int) {
  exit_with_no(0);
}

int main(int argc, char *argv[])
{
   fprintf(stderr, "JSK: elmo_shm / SOEM (Simple Open EtherCAT Master)\n");

   boost::program_options::options_description opt("Options");
   opt.add_options()
     ("help,h", "Show help.")
     ("nodisplay,n", "without display mode")
     ("testmode,t", "test without controller")
     ("calibration,c", "calibration mode")
     ("startlog,l", "start log")
     ("without_hrpsys,w", "use without hrpsys")
     ("device,d", boost::program_options::value<std::string>(), "device name")
     ("parameterfile,f", boost::program_options::value<std::string>(), "parameter file name")
     ("logfile,L", boost::program_options::value<std::string>(), "log file name");
   boost::program_options::variables_map argmap;
   boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opt), argmap);
   boost::program_options::notify(argmap);

   std::string device("eth0");
   std::string pfname("../driver_settings.yaml");
   bool start_display = true;

   if( argmap.count( "help" ) ) {
     std::cerr << opt << std::endl;
     return 1;
   }
   if( argmap.count( "nodisplay" ) ) {
     start_display = false;
   }
   if( argmap.count( "testmode" ) ) {
     //std::cerr << "testmode" << std::endl;
   }
   if( argmap.count( "calibration" ) ) {
     //std::cerr << "calibration" << std::endl;
     calibration_mode = true;
   }
   if( argmap.count( "startlog" ) ) {
     //std::cerr << "startlog" << std::endl;
   }
   if( argmap.count( "without_hrpsys" ) ) {
     std::cerr << "without_hrpsys" << std::endl;
     use_hrpsys = false;
   }
   if( argmap.count( "device" ) ) {
    std::string str(argmap["device"].as<std::string>());
    device = str;
   }
   if( argmap.count( "parameterfile" ) ) {
    std::string str(argmap["parameterfile"].as<std::string>());
    pfname = str;
   }
   if( argmap.count( "logfile" ) ) {
    std::string str(argmap["logfile"].as<std::string>());
    logfilename = str;
   }

   // read settings
   bool ret = parse_driver_parameter_yaml(pfname, drv_params);

   signal( SIGINT, &cchandler );

   shm = (servo_shm *) set_shared_memory(5555, sizeof(servo_shm));
   if (shm == NULL) {
     return -1;
   }
   shm->disable_alert_on_servo_on = 0;
   for (int i=0; i<MAX_JOINT_NUM; i++) {
     shm->servo_state[0][i] = 0x00;
     shm->servo_state[1][i] = 0x00;
     shm->hole_status[0][i] = 0x00;
     shm->motor_num[i] = 0;
     shm->servo_on[i]  = 0;
     shm->servo_off[i] = 0;
     shm->is_servo_on[i]  = 0;
     shm->torque0[i]   = 0;
     shm->loopback[i]  = 0;
     //shm->joint_enable[i] = 1;
     shm->joint_offset[i] = 0;

     //
     shm->ref_angle[i] = 0.0;
     shm->ref_torque[i] = 0.0;
     shm->ref_vel[i] = 0.0;
     shm->cur_vel[i] = 0.0;
     shm->abs_vel[i] = 0.0;
     shm->pgain[i] = 0.0;
     shm->dgain[i] = 0.0;
     shm->motor_temp[0][i] = 0.0;
     shm->motor_current[0][i] = 0.0;
     shm->motor_output[0][i] = 0.0;
   }


   if (calibration_mode) {
     ethercat_loop(device.c_str());
   } else {
     /* create thread to handle slave error handling in OP */
     osal_thread_create(&thread1, 128000, (void *) &ecatcheck, (void*) &ctime);

     if (start_display) {
       // display thread
       pthread_t display_thread;
       long d_args[3];
       d_args[0] = 500000; // period
       d_args[1] = REALTIME_PRIO_MAX - 30;
       d_args[2] = (long)(&stop_flag);

       if ( pthread_create( &display_thread, NULL, display_thread_fun, (void *)(&d_args)) ) {
         fprintf(stderr, "pthread_create (display) was filed");
         exit(1);
       }
     }

     create_logger();

     /* start cyclic part */
     ethercat_loop(device.c_str());
   }

   return (0);
}
