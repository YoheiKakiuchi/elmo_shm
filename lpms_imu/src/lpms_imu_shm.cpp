#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"

#include <iostream>
#include <list>
#include <iomanip>
#include <stdexcept>

#include "ExtKalmanFilter.h"
//#include "conio.h" // for non ANSI _kbhit() and _getch()

/// add
#include <sys/time.h>  // gettimeofday
#include <time.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h> // sleep, usleep
//#include <sys/types.h>
//#include <signal.h>
extern "C" {
#include "wheel_shm.h"
}

struct servo_shm *s_shm;
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

struct timespec gyro_tm;
struct timespec pose_tm;
static int gyro_counter = 0;
static int pose_counter = 0;

#define SERIAL_PORT "/dev/LPMS_IMU"

#define ACCEL_ONE_G 9.80665
#define DEG2RAD (M_PI/180)
//--------------------------------------------------------------------------------
int main(int argc, char **argv) {
  s_shm = (struct servo_shm *)set_shared_memory(5555, sizeof(struct servo_shm));
  if (!s_shm) {
    // error message
    std::cerr << "Can not open shared memory" << std::endl;
    return -1;
  }
  std::cout << "Open shared memory" << std::endl;

  /* initial dummy write */
  s_shm->IMU_accel[0] = 0.0;
  s_shm->IMU_accel[1] = 0.0;
  s_shm->IMU_accel[2] = 9.8;
  s_shm->IMU_gyro[0] = 0.0;
  s_shm->IMU_gyro[1] = 0.0;
  s_shm->IMU_gyro[2] = 0.0;

  bool use_local_ekf = false;
  bool is_calibrate_gyro = false;
  for (int i = 1; i < argc; i++) {
    if(strcmp(argv[i], "-e") == 0) use_local_ekf = true;
    if(strcmp(argv[i], "-g") == 0) is_calibrate_gyro = true;
  }
  //
  EKFilter ekf;
  ekf.setdt(1/400.0); // 400Hz
  ekf.resetKalmanFilterState();

  /* open device */
  ImuData d;

  // Gets a LpmsSensorManager instance
  LpmsSensorManagerI* manager = LpmsSensorManagerFactory();
  LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_U2, SERIAL_PORT);
  // settings
  int counter = 0;
  while (1) {
    if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED) {
      //lpms->pause();
      if (lpms->setConfigurationPrm(PRM_SAMPLING_RATE, 400)) { // 400 Hz
        std::cerr << "set sample rate: success" << std::endl;
      } else {
        std::cerr << "set sample rate: failed" << std::endl;
      }
      sleep(1);
      if (lpms->setConfigurationPrm(PRM_ACC_RANGE, SELECT_ACC_RANGE_8G)) {
        std::cerr << "set acc range: success" << std::endl;
      } else {
        std::cerr << "set acc range: failed" << std::endl;
      }
      sleep(1);
      if (lpms->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_500DPS)) {
        std::cerr << "set gyro range: success" << std::endl;
      } else {
        std::cerr << "set gyro range: failed" << std::endl;
      }
      sleep(1);
      if (lpms->setConfigurationPrm(PRM_SELECT_DATA,
                                    (SELECT_LPMS_QUAT_OUTPUT_ENABLED |
                                     //SELECT_LPMS_EULER_OUTPUT_ENABLED |
                                     SELECT_LPMS_LINACC_OUTPUT_ENABLED |
                                     //SELECT_LPMS_PRESSURE_OUTPUT_ENABLED |
                                     SELECT_LPMS_GYRO_OUTPUT_ENABLED |
                                     SELECT_LPMS_ACC_OUTPUT_ENABLED |
                                     //SELECT_LPMS_MAG_OUTPUT_ENABLED |
                                     //SELECT_LPMS_GYRO_TEMP_OUTPUT_ENABLED |
                                     //SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED |
                                     //SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED |
                                     SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED )
                                    )) {
        std::cerr << "select data : success" << std::endl;
      } else {
        std::cerr << "select data : failed" << std::endl;
      }
      sleep(1);
      if (lpms->setConfigurationPrm(PRM_FILTER_MODE, SELECT_FM_MADGWICK_GYRO_ACC_MAG)) {
          std::cerr << "select filter mode : success" << std::endl;
      } else {
          std::cerr << "select filter mode : failed" << std::endl;
      }
      sleep(1);
      if (lpms->updateParameters()) {
        std::cerr << "update param : success" << std::endl;
      } else {
        std::cerr << "update param : failed" << std::endl;
      }
      if (is_calibrate_gyro) {
        std::cout << "Started gyro calibration. Do NOT touch the robot." << std::endl;
        lpms->startCalibrateGyro(); // start calibrate and sleep, is this required???
        sleep(30); // It takes 30 s to calibrate gyro
        std::cout << "Finished gyro calibration." << std::endl;
      }
      sleep(1);
      if (lpms->setConfigurationPrm(PRM_GYR_AUTOCALIBRATION, 0)) { // stop auto gyro calibration
        std::cerr << "disable auto gyro calibration: success" << std::endl;
      } else {
        std::cerr << "disable auto gyro calibration: failed" << std::endl;
      }
      sleep(1);
      if(lpms->updateParameters()) {
        std::cerr << "update param : success" << std::endl;
      } else {
        std::cerr << "update param : failed" << std::endl;
      }
      break;
    }
    usleep(100);
    if(counter++ > 100000) {
      std::cerr << "connection timeout" << std::endl;
      return 0;
    }
  }

  sleep(1);
  std::cout << "orientation reset" << std::endl;
  lpms->setOrientationOffset(0); // I don't know what the
  sleep(1);

  clock_gettime(CLOCK_MONOTONIC, &pose_tm);
  clock_gettime(CLOCK_MONOTONIC, &gyro_tm);

  while (1) {
    // Checks, if sensor is connected
    if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
        lpms->hasImuData())  {
      // Reads quaternion data
      d = lpms->getCurrentData();
#if 0
      {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double time_diff = ((now.tv_sec - gyro_tm.tv_sec) + (now.tv_nsec - gyro_tm.tv_nsec)/1000000000.0);
        printf("%f\n", time_diff);
        gyro_tm.tv_sec = now.tv_sec;
        gyro_tm.tv_nsec = now.tv_nsec;
      }
#endif
      s_shm->IMU_sensor_orientation[0] = d.q[1];//x
      s_shm->IMU_sensor_orientation[1] = d.q[2];//y
      s_shm->IMU_sensor_orientation[2] = d.q[3];//z
      s_shm->IMU_sensor_orientation[3] = d.q[0];//w
      pose_counter++;

      if(pose_counter % 201 == 0) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        printf("\r");
        printf(" pose frequency:  %5.1f Hz", 200.0 / ((now.tv_sec - pose_tm.tv_sec) + (now.tv_nsec - pose_tm.tv_nsec)/1000000000.0));
        pose_tm.tv_sec = now.tv_sec;
        pose_tm.tv_nsec = now.tv_nsec;
        pose_counter = 1;
      }

      // why invert accel direction??
      s_shm->IMU_accel[0] = - (d.a[0] * ACCEL_ONE_G);
      s_shm->IMU_accel[1] = - (d.a[1] * ACCEL_ONE_G);
      s_shm->IMU_accel[2] = - (d.a[2] * ACCEL_ONE_G);

      s_shm->IMU_gyro[0] = d.g[0] * DEG2RAD;
      s_shm->IMU_gyro[1] = d.g[1] * DEG2RAD;
      s_shm->IMU_gyro[2] = d.g[2] * DEG2RAD;

#if 0
      // use angular velocity, what is the difference between gyro value and angular velocity ???
      s_shm->IMU_gyro2[0] = - (d.w[0] * DEG2RAD);
      s_shm->IMU_gyro2[1] = - (d.w[1] * DEG2RAD);
      s_shm->IMU_gyro2[2] = - (d.w[2] * DEG2RAD);
#endif

#if 0
      // use raw value ??
      s_shm->IMU_accel2[0] = - (d.aRaw[0] * ACCEL_ONE_G);
      s_shm->IMU_accel2[1] = - (d.aRaw[1] * ACCEL_ONE_G);
      s_shm->IMU_accel2[2] = - (d.aRaw[2] * ACCEL_ONE_G);

      s_shm->IMU_gyro2[0] = d.gRaw[0] * DEG2RAD;
      s_shm->IMU_gyro2[1] = d.gRaw[1] * DEG2RAD;
      s_shm->IMU_gyro2[2] = d.gRaw[2] * DEG2RAD;
#endif

      if (use_local_ekf) {
        Eigen::Vector3d acc(s_shm->IMU_accel[0],
                            s_shm->IMU_accel[1],
                            s_shm->IMU_accel[2]);
        Eigen::Vector3d gyro(s_shm->IMU_gyro[0],
                             s_shm->IMU_gyro[1],
                             s_shm->IMU_gyro[2]);
        Eigen::Quaternion<double> q;
        ekf.main_one(q, acc, gyro);

        s_shm->IMU_filtered_orientation1[0] = q.x();
        s_shm->IMU_filtered_orientation1[1] = q.y();
        s_shm->IMU_filtered_orientation1[2] = q.z();
        s_shm->IMU_filtered_orientation1[3] = q.w();
      }
#if 0
      printf("Timestamp=%f, ax=%f, ay=%f, ax=%f, wx=%f, wy=%f, wz=%f\n",
             d.timeStamp, d.a[0], d.a[1], d.a[2],
             d.g[0], d.g[1], d.g[2]);
#endif
    }
    usleep(25);
    fflush(stdout);
  }

  return 0;
}
