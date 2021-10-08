#include "realtime_task.h" // elmo_shm/include

#include "list_display.h"  // elmo_shm/include
#include "servo_shm.h"     // elmo_shm/include

#define DISPLAY_THREAD_PERIOD   500000

// ListDisplay disp(2);
void *list_display_thread_fun (void *arg)
{
  int period = ((int *)arg)[0];
  int prio   = ((int *)arg)[1];
  volatile int *stop_flag = (int *)(((unsigned long *)arg)[2]);
  ListDisplay *disp = (ListDisplay *)(((int *)arg)[3]);

  realtime_task::Context rt_context(prio, period, false);

  disp->flush();

  try {
    while(!(*stop_flag)) {
      rt_context.wait();
      /* display code here */
    }
  } catch (std::runtime_error &e) {
    disp->printf("display_thread_fun: %s\n", e.what());
  } catch (...) {
    disp->printf("display_thread_fun: ???\n");
  }
  *stop_flag = 1;

  return 0;
}

/*
  Simple display code
 */
///// DISPLAY methods
#define START_INDEX  0
#define COLUMN_NUM   7
#define ROW_NUM      2
#define COLUMN_WIDTH 11
#define ROW_WIDTH    10

inline void move_cursor_up(FILE* strm, int i)
{
  fprintf(strm, "\x1b[%dA", i);
}
inline void move_cursor_down(FILE* strm, int i)
{
  fprintf(strm, "\x1b[%dB", i);
}
inline void move_cursor_right(FILE* strm, int i)
{
  fprintf(strm, "\x1b[%dC", i);
}
inline void move_cursor_left(FILE* strm, int i)
{
  fprintf(strm, "\x1b[%dD", i);
}
inline void move_cursor(FILE* strm, int i, int j)
{
  fprintf(strm, "\x1b[%d;%dH", i, j);
}
inline void clear_screen(FILE* strm)
{
  fprintf(strm, "\x1b[2J");
  fprintf(strm, "\x1b[1;1H");
}
inline void char_color(FILE* strm, int i)
{ //背景色
  //40 : 黒
  //41 : 赤
  //42 : 緑
  //43 : 黄
  //44 : 青
  //45 : 紫
  //46 : 水
  //47 : 白
  fprintf(strm, "\x1b[%dm", i);
}
inline void next_row(FILE* strm)
{
  move_cursor_down(strm, 1);
  move_cursor_left(strm, COLUMN_WIDTH);
}
inline void next_column(FILE* strm)
{
  move_cursor_up(strm, ROW_WIDTH);
}
inline void next_limb(FILE* strm)
{
  move_cursor_down(strm, ROW_WIDTH);
  fprintf(strm, "\n\n");
}
inline void print_int_with_col(FILE* strm, int size, unsigned int val, unsigned int col)
{
  for (int i = size - 1; i >= 0; i--) {
    switch ( 0xF & (col >> (4*i)) ) {
    case 0:
      char_color(strm, 0);
      break;
    case 1:
      char_color(strm, 41); // red
      break;
    case 2:
      char_color(strm, 42); // green
      break;
    case 3:
      char_color(strm, 43); // yellow
      break;
    case 4:
      char_color(strm, 44); // blue
      break;
    case 5:
      char_color(strm, 45); // purple
      break;
    case 6:
      char_color(strm, 46); // cyan
      break;
    }
    fprintf(strm, "%1X", 0xF & (val >> (4*i)) );
  }
  char_color(strm, 0);
}
// DISPLAY methods

extern double jitter;
extern double max_interval;
extern servo_shm *shm;

extern double m0_jitter;
extern double m1_jitter;
extern double m0_max_int;
extern double m1_max_int;
extern double m0_min_int;
extern double m1_min_int;

void *display_thread_fun (void *arg)
{
  int period = ((long *)arg)[0];
  int prio   = ((long *)arg)[1];
  volatile int *stop_flag = (int *)((unsigned long *)arg)[2];

  fprintf(stderr, "prio:%d period:%d\n", prio, period);
  realtime_task::Context rt_context(prio, period);

  FILE *print_strm = stdout;

  while( !(*stop_flag) ) {
    // display
    clear_screen(print_strm);
    fprintf(print_strm, "jitter: %4.2f [us] / max_interval: %6.4f [ms]", jitter, max_interval/1000.0);
    fprintf(print_strm, "/ m0 j: %4.2f [us], max: %6.4f [ms], min: %6.4f [ms]",
            m0_jitter, m0_max_int/1000.0, m0_min_int/1000.0);
    fprintf(print_strm, "/ m1 j: %4.2f [us], max: %6.4f [ms], min: %6.4f [ms]\n",
            m1_jitter, m1_max_int/1000.0, m1_min_int/1000.0);

    //int shm_idx;
    for(int n = 0; n < ROW_NUM; n++) {
      fprintf(print_strm, "       id#:");
      next_row(print_strm);
      fprintf(print_strm, "     sstat:");
      next_row(print_strm);
      fprintf(print_strm, "       ref:");
      next_row(print_strm);
      fprintf(print_strm, "       ang:");
      next_row(print_strm);
      fprintf(print_strm, "       abs:");
      next_row(print_strm);
      fprintf(print_strm, "       vel:");
      next_row(print_strm);
      fprintf(print_strm, "       cur:");
      next_row(print_strm);
      fprintf(print_strm, "      temp:");
      next_row(print_strm);
      fprintf(print_strm, "     DC in:");
      next_row(print_strm);
      fprintf(print_strm, "    extra0:");
      next_row(print_strm);
      fprintf(print_strm, "    extra1:");
      next_column(print_strm);
      for(int i = 0; i < COLUMN_NUM; i++) {
        int idx = i + n*COLUMN_NUM + START_INDEX;
        { // id num
          if (shm->loopback[idx]) char_color(print_strm, 47);
          fprintf(print_strm, "    %3d    ", idx);
          char_color(print_strm, 0);
          next_row(print_strm);
        }
        { // servo state
          unsigned short stat = ((unsigned int)(shm->hole_status[0][idx]) & 0xFFFF0000) >> 16;
          if ((stat & 0x0080) != 0x0080) char_color(print_strm, 47); // white
          fprintf(print_strm, "w"); // warning
          if ((stat & 0x0010) == 0x0010) char_color(print_strm, 42); // green
          fprintf(print_strm, " v"); // voltage
          if (((stat & 0x0040) == 0x0000) && ((stat & 0x000F) == 0x0000)) {
            // S_NOT_READY_TO_SWITCH_ON ( purple )
            char_color(print_strm, 45);
          } else if (((stat & 0x0040) == 0x0040) && ((stat & 0x000F) == 0x0000)) {
            // S_SWITCH_ON_DISABLED (?? 0000 -> xxxx) (yellow)
            char_color(print_strm, 43);
          } else if ((stat & 0x0060) == 0x0020) {
            if ((stat & 0x0007) == 0x0007) {
              // servo on ( green )
              char_color(print_strm, 42);
            } else {
              // white
              char_color(print_strm, 47);
            }
          } else if (((stat & 0x0060) == 0x0000) && ((stat & 0x0007) == 0x0007)) {
            // S_QUICK_STOP_ACTIVE (red)
            char_color(print_strm, 41);
          } else if (((stat & 0x0040) == 0x0000) && ((stat & 0x000F) == 0x000F)) {
            // S_FAULT_REACTION_ACTIVE ( cyan )
            char_color(print_strm, 46);
          } else if (((stat & 0x0040) == 0x0000) && ((stat & 0x0008) == 0x0008)) {
            // S_FAULT ( blue )
            char_color(print_strm, 44);
          }
          fprintf(print_strm, "0x%04X", stat & 0xFFFF);
          char_color(print_strm, 0);
          fprintf(print_strm, " ");
#if 0
          fprintf(print_strm, "  ");
          char_color(print_strm, 0); if (stat & 0x0080) char_color(print_strm, 42); // green
          fprintf(print_strm, " ");
          char_color(print_strm, 0); if (stat & 0x0040) char_color(print_strm, 42); // green
          fprintf(print_strm, " ");
          char_color(print_strm, 0); if (stat & 0x0020) char_color(print_strm, 42); // green
          fprintf(print_strm, "0");
          char_color(print_strm, 0); if (stat & 0x0010) char_color(print_strm, 42); // green
          fprintf(print_strm, "x");
          char_color(print_strm, 0); if (stat & 0x0008) char_color(print_strm, 42); // green
          fprintf(print_strm, "%1X", (stat & 0xF000) >> 12);
          char_color(print_strm, 0); if (stat & 0x0004) char_color(print_strm, 42); // green
          fprintf(print_strm, "%1X", (stat & 0x0F00) >> 8);
          char_color(print_strm, 0); if (stat & 0x0002) char_color(print_strm, 42); // green
          fprintf(print_strm, "%1X", (stat & 0x00F0) >> 4);
          char_color(print_strm, 0); if (stat & 0x0001) char_color(print_strm, 42); // green
          fprintf(print_strm, "%1X", (stat & 0x000F));
          char_color(print_strm, 0);
          fprintf(print_strm, " ");
          // fprintf(print_strm, "    0x%4X ", stat & 0xFFFF);
#endif
          next_row(print_strm);
        }
        { // ref angle
          double diff = (shm->ref_angle[idx] - shm->cur_angle[idx]);
          diff = fabs(diff);
          if (diff > 10) {
            char_color(print_strm, 41); // red
          } else if (diff > 5) {
            char_color(print_strm, 43); // yellow
          } else if (diff > 2.5) {
            char_color(print_strm, 42); // green
          } else if (diff > 1.0) {
            char_color(print_strm, 46); // cyan
          }
          fprintf(print_strm, "%10.4f ", shm->ref_angle[idx]);
          char_color(print_strm, 0);
          next_row(print_strm);
        }
        { // cur angle
          fprintf(print_strm, "%10.4f ", shm->cur_angle[idx]);
          next_row(print_strm);
        }
        { // abs angle
          fprintf(print_strm, "%10.4f ", shm->abs_angle[idx]);
          next_row(print_strm);
        }
        { // cur velocity
          //fprintf(print_strm, "%12.3f ", shm->cur_vel[idx]);
          fprintf(print_strm, "%10.4f ", shm->abs_vel[idx]);
          next_row(print_strm);
        }
        { // current
          double cur_rate = shm->motor_current[1][idx];
          cur_rate = fabs(cur_rate);
          // continuous 1.22 A
          // stole 3.3
          if(cur_rate > 2.8) {
            char_color(print_strm, 45); // purple
          } else if (cur_rate > 2.0) {
            char_color(print_strm, 41); // red
          } else if (cur_rate > 1.4) {
            char_color(print_strm, 43); // yellow
          } else if (cur_rate > 1.0) {
            char_color(print_strm, 42); // green
          } else if (cur_rate > 0.5) {
            char_color(print_strm, 46); // cyan
          } else if (cur_rate > 0.25) {
            char_color(print_strm, 44); // blue
          }
          fprintf(print_strm, "%10.3f ", shm->motor_current[0][idx]);
          char_color(print_strm, 0);
          next_row(print_strm);
        }
        { // temp
          double temp = shm->motor_temp[0][idx];
          if (temp > 110.0) {
            char_color(print_strm, 45); // purple
          } else if (temp > 100.0) {
            char_color(print_strm, 41); // red
          } else if (temp > 90.0) {
            char_color(print_strm, 43); // yellow
          } else if (temp > 80.0) {
            char_color(print_strm, 42); // green
          } else if (temp > 60.0) {
            char_color(print_strm, 46); // cyan
          } else if (temp > 40.0) {
            char_color(print_strm, 44); // blue
          }
          fprintf(print_strm, "%10.2f ", temp);
          char_color(print_strm, 0);
          next_row(print_strm);
        }
        { // DC in
          double vlt = shm->board_vdd[0][idx];
          if (vlt < 5.0) {
            char_color(print_strm, 41); // red
          }
          fprintf(print_strm, "%10.2f ", vlt);
          char_color(print_strm, 0);
          next_row(print_strm);
        }
        { // extra
          unsigned short stat_extra = (shm->hole_status[0][idx] & 0x0000FFFF);
          unsigned   int stat_elmo  = shm->servo_state[1][idx];
          { // elmo
            unsigned int col = 0;
            if (0x3 & stat_elmo) col |= 0x1;               //0 err
            if ((0x1 <<  4) & stat_elmo) col |= 0x2 << 4;  //1 servo enable
            if ((0x1 <<  6) & stat_elmo) col |= 0x1 << 8;  //2 fault
            if ((0x1 << 13) & stat_elmo) col |= 0x1 << 12; //3 current limit
            if ((0x1 << 14) & stat_elmo) col |= 0x2 << 16; //4 STO0
            if ((0x1 << 15) & stat_elmo) col |= 0x2 << 20; //5 STO1
            if ((0x1 << 22) & stat_elmo) col |= 0x2 << 24; //6 motor on
            if ((0x1 << 27) & stat_elmo) col |= 0x1 << 28; //7
            fprintf(print_strm, "  ");
            print_int_with_col(print_strm, 8, stat_elmo, col);
            fprintf(print_strm, " ");
          }
          next_row(print_strm);
          { // extra
            fprintf(print_strm, "      %04X ", stat_extra);
          }
        }
        next_column(print_strm);
      }
      next_limb(print_strm);
    }
    rt_context.wait();
  }
  //
  return 0;
}
/*
//// extra state
0  / For a planar motor, this bit informs that the commutation has performed successfully.
  0: Commutation is not known.
  1: Commutation known.
1  / Informs that the levels of the analog signals used for feedback (sine/cosine) are above the minimum defined range.
  0: Analog signals amplitude is lower than the defined in CA[52]
  1: Analog signals amplitude is above or equals to the requested.
2  / Low voltage sensed
3  / High voltage sensed
4  / Analog sensor low voltage
5  / High temperature sensed
6  / Bit is set if there is a battery alarm (low voltage or no battery) in the digital absolute sensor of Yaskawa, Panasonic, Tamagawa, EnDat, Nikon or Warning for General BiSS if defined by CA[60] bit 10. The indication, as read from the sensor, is reflected in this bit.
7  / Yaskawa absolute serial encoder was reset
9  / Gurley absolute sensor data valid
10 / Auto focus mode out of limit.
12 / Home Attained indication

//// elmo state
0 - 3 / Amplifier Status - reports the instantaneous state of the power drive.
    0x00 All OK
    0x03 Undervoltage:
    0x05 Overvoltage:
    0x07 Safety:
    0x09 Feedback:
    0x0B Short Protection: The current has exceeded a range which is considered as a phase to phase or phase to ground short.
    0x0D Over-temperature:
4     / The servo is enabled. The reference command can be processed by the profiler (SO) / 1: The servo is enabled
5     / Reference Mode
6     / A fault occurred while the motor was enabled.
7     / In Elmo's homing or capture sequence is active.
8 - 11/Reports the actual profiler according to the motion mode.
    0  No motion was selected.
    1  Profile position mode (PTP)
    2  N/A
    3  Profile Velocity mode (JV)
    4  Profile Torque mode (TC)
    5  N/A
    6  Homing mode (DS-402 only)
    7  Interpolated position mode (DS-402 only)
    8  Cyclic sync position mode (DS-402 only)
    9  Cyclic sync velocity mode (DS-402 only)
    10 Cyclic sync torque mode (DS-402 only)
12    / User Program is running / 1: The user program is running.
13    / Current Limit is on     / 1: Current is limited to CL[1].
14    / Safety Input 1 (STO_DSP)
15    / Safety Input 2 (STO_PWM)
16-17 / Recorder Status
    0 The recorder is not active.
    1 Waiting for a trigger.
    2 The recorder has completed its task. Valid data is ready for uploading.
    3 Recording is now active. Data is been fetched by the drive.
18    / Target Reached
21    / Shunt is active / 0: Shunt is switched on
22    / Motor On (MO) / 1: Motor is On (MO=1)
23    / Movement defined by:
24-26 / Hall A, Hall B, Hall C state
27    / Safe Torque Off diagnostics error
28    / The profiler stopped due to a switch.
30    / PTP buffer is full

 */
