#ifndef __WHEEL_SHM_H__
#define __WHEEL_SHM_H__

#define MAX_JOINT_NUM 8
#define WHEEL_RIGHT 0
#define WHEEL_LEFT  1

#define SERVOMODE_FREE 8
#define SERVOMODE_POSITION 0
#define SERVOMODE_ABSPOSITION_CURRENT 1
#define SERVOMODE_POSITION_TORQUE 2
#define SERVOMODE_POSITION_FFTORQUE 3

struct servo_shm {
  // current values from robot
  float cur_angle[MAX_JOINT_NUM];
  float cur_vel[MAX_JOINT_NUM];
  float cur_torque[MAX_JOINT_NUM];

  // filtered current values
  float filtered_angle[MAX_JOINT_NUM];
  float filtered_vel[MAX_JOINT_NUM];
  //float filtered_torque[MAX_JOINT_NUM];

  // reference from upper layer
  float ref_angle[MAX_JOINT_NUM];
  float ref_vel[MAX_JOINT_NUM];
  float ref_torque[MAX_JOINT_NUM];

  // gain settings
  float pgain[MAX_JOINT_NUM];
  float dgain[MAX_JOINT_NUM];
  float torque_pgain[MAX_JOINT_NUM];
  float torque_dgain[MAX_JOINT_NUM];

  // extra information
  float motor_temp[MAX_JOINT_NUM];
  float motor_outer_temp[MAX_JOINT_NUM];
  float motor_current[MAX_JOINT_NUM];
  float motor_output[MAX_JOINT_NUM];
  float motor_rate[MAX_JOINT_NUM]; //
  float board_vdd[MAX_JOINT_NUM];

  // IMU
  float IMU_accel[3];
  float IMU_gyro[3];
  float IMU_filtered_orientation1[4]; //quaternion
  float IMU_filtered_orientation2[4]; //quaternion
  float IMU_sensor_orientation[4]; // read from sensor if available

  // command for 2 wheels robot 
  float com_forward;
  float com_turn;
  int com_wt_flag;
  int com_rd_flag;

  //
  int  controlmode[MAX_JOINT_NUM];
  int  servo_state[MAX_JOINT_NUM];
  int  hole_status[MAX_JOINT_NUM];
  int  frame;
  int  start_log;
  double jitter;

  char is_servo_on[MAX_JOINT_NUM];
  char servo_on[MAX_JOINT_NUM];
  char servo_off[MAX_JOINT_NUM];
  char torque0[MAX_JOINT_NUM];
  char loopback[MAX_JOINT_NUM];
  char joint_enable[MAX_JOINT_NUM];
  char ref_wt_flag[MAX_JOINT_NUM];//
  char ref_rd_flag[MAX_JOINT_NUM];//
};

#endif
