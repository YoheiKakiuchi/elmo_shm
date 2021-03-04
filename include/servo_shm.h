#ifndef __SERVO_SHM_H__
#define __SERVO_SHM_H__
#include <pthread.h>

#define MAX_JOINT_NUM 32
#define MAX_MOTOR_NUM 2
#define MAX_IMU_NUM 2
#define MAX_FSENSOR_NUM 4

#define SERVOMODE_FREE 8
#define SERVOMODE_POSITION 0
#define SERVOMODE_ABSPOSITION_CURRENT 1
#define SERVOMODE_POSITION_TORQUE 2
#define SERVOMODE_POSITION_FFTORQUE 3

struct servo_shm {
  float ref_angle[MAX_JOINT_NUM];
  float cur_angle[MAX_JOINT_NUM];
  float abs_angle[MAX_JOINT_NUM];
  float ref_vel[MAX_JOINT_NUM];
  float cur_vel[MAX_JOINT_NUM];
  float ref_torque[MAX_JOINT_NUM];
  float cur_torque[MAX_JOINT_NUM];
  float pgain[MAX_JOINT_NUM];
  float dgain[MAX_JOINT_NUM];
  float torque_pgain[MAX_JOINT_NUM];
  float torque_dgain[MAX_JOINT_NUM];
  float subgain[MAX_JOINT_NUM][4];
  int motor_num[MAX_JOINT_NUM];
  float motor_temp[MAX_MOTOR_NUM][MAX_JOINT_NUM];
  float motor_outer_temp[MAX_MOTOR_NUM][MAX_JOINT_NUM];
  float motor_current[MAX_MOTOR_NUM][MAX_JOINT_NUM];
  float motor_output[MAX_MOTOR_NUM][MAX_JOINT_NUM];
  float board_vin[MAX_MOTOR_NUM][MAX_JOINT_NUM];
  float board_vdd[MAX_MOTOR_NUM][MAX_JOINT_NUM];
  int   comm_normal[MAX_MOTOR_NUM][MAX_JOINT_NUM];
  float   h817_rx_error0[MAX_MOTOR_NUM][MAX_JOINT_NUM];
  float   h817_rx_error1[MAX_MOTOR_NUM][MAX_JOINT_NUM];
  float body_omega[MAX_IMU_NUM][3];
  float body_acc[MAX_IMU_NUM][3];
  float body_posture[MAX_IMU_NUM][4]; //Quaternion
  float zero_acc[MAX_IMU_NUM][3];
  float ekf_cov[MAX_IMU_NUM][6];
  float gyro_bias[MAX_IMU_NUM][3];

  float external_protractor[2];

  float reaction_force[MAX_FSENSOR_NUM][6]; //IFS ch1, 2, 3, 4
  float reaction_force_f1[MAX_FSENSOR_NUM][6]; //IFS ch1, 2, 3, 4 Filter No.1
  float reaction_force_f2[MAX_FSENSOR_NUM][6]; //IFS ch1, 2, 3, 4 Filter No.2
  float reaction_force_f3[MAX_FSENSOR_NUM][6]; //IFS ch1, 2, 3, 4 Filter No.3
  float reaction_force_f4[MAX_FSENSOR_NUM][6]; //IFS ch1, 2, 3, 4 Filter No.4
  float reaction_force_f5[MAX_FSENSOR_NUM][6]; //IFS ch1, 2, 3, 4 Filter No.5
  float reaction_force_f6[MAX_FSENSOR_NUM][6]; //IFS ch1, 2, 3, 4 Filter No.6
  int cal_reaction_force[MAX_FSENSOR_NUM]; // reset reactionforce to 0

  int controlmode[MAX_JOINT_NUM];
  char is_servo_on[MAX_JOINT_NUM];
  char servo_on[MAX_JOINT_NUM];
  char servo_off[MAX_JOINT_NUM];
  char torque0[MAX_JOINT_NUM];
  int servo_state[MAX_MOTOR_NUM][MAX_JOINT_NUM];
  int frame;
  int received_packet_cnt;
  double jitter;
  int set_ref_vel;
  char loopback[MAX_JOINT_NUM];
  int hole_status[MAX_MOTOR_NUM][MAX_JOINT_NUM];
  char calib_mode;
  char disable_alert_on_servo_on;
  char joint_enable[MAX_JOINT_NUM];
  int joint_offset[MAX_JOINT_NUM];

  unsigned short torque_coef_current[MAX_JOINT_NUM];
  unsigned short torque_coef_inertia[MAX_JOINT_NUM];
  unsigned short torque_coef_coulombfric[MAX_JOINT_NUM];
  unsigned short torque_coef_viscousfric[MAX_JOINT_NUM];

#ifdef USE_PTHREAD_MUTEX
  pthread_mutex_t cmd_lock;
  pthread_mutex_t info_lock;
#else
  int cmd_lock;
  int info_lock;
#endif

  // extra data
  float prev_angle[MAX_JOINT_NUM];
  int interpolation_counter[MAX_JOINT_NUM];
  float abs_vel[MAX_JOINT_NUM];
};

int shm_lock_init(struct servo_shm *shm);
int cmd_shm_lock(struct servo_shm *shm);
int info_shm_lock(struct servo_shm *shm);
int cmd_shm_unlock(struct servo_shm *shm);
int info_shm_unlock(struct servo_shm *shm);

#endif
