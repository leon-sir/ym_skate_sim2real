
#ifndef TRANSMIT_H
#define TRANSMIT_H

#define SLAVE_NUMBER 2

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "sys/time.h"
// #include "imu_data_serial.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    double pos_, vel_, tau_;
    double pos_des_, vel_des_, kp_, kd_, ff_;
  } YKSMotorData;

  extern YKSMotorData motorDate_recv[14];
  // typedef struct
  // {
  //   float angle_float[3];
  //   float gyro_float[3];
  //   float accel_float[3];
  //   float mag_float[3];
  //   float quat_float[4];
  // } YKSIMUData;
  // extern YKSIMUData imuData_recv;
  int EtherCAT_Init(char *ifname);
  // int IMU_Init();
  void EtherCAT_Run();
  void EtherCAT_Command_Set();
  void EtherCAT_Send_Command(YKSMotorData *data);
  void EtherCAT_Send_Command_Position(YKSMotorData *data);
  void EtherCAT_Send_Command_Speed(float *data);
  void EtherCAT_Get_State();
  void Revert_State(YKSMotorData *motor_data);
  void startRun();

#ifdef __cplusplus
};
#endif

#endif // PROJECT_RT_ETHERCAT_H