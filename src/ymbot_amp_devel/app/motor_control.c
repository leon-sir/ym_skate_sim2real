/********************************************************************************
Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website 
at www.bridgedp.com.
********************************************************************************/

#include "motor_control.h"
#include <stdio.h>

#define KP_MIN 0.0f
#define KP_MAX 500.0f

#define KD_MIN 0.0f
#define KD_MAX 50.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f

const float tor_min[6] = {-90, -90, -120, -150, -30, -30};
const float tor_max[6] = {90, 90, 120, 150, 30, 30};
const float i_min[6] = {-60, -60, -60, -70, -30, -30};
const float i_max[6] = {60, 60, 60, 70, 30, 30};

union RV_TypeConvert
{
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
} rv_type_convert;

union RV_TypeConvert2
{
    int16_t to_int16;
    uint16_t to_uint16;
    uint8_t buf[2];
} rv_type_convert2;

MotorCommFbd motor_comm_fbd;
OD_Motor_Msg rv_motor_msg[6];
// IMU_Msg imu_msg;

// MOTOR SETTING
/*
cmd:
0x00:NON
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.
0x03:set the current position to zero.
*/
void MotorSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint8_t cmd)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].dlc = 4;

    if (cmd == 0)
        return;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = cmd;
}

// Reset Motor ID
void MotorIDReset(EtherCAT_Msg *TxMessage)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 6;
    TxMessage->motor[0].rtr = 0;

    TxMessage->motor[0].data[0] = 0x7F;
    TxMessage->motor[0].data[1] = 0x7F;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x05;
    TxMessage->motor[0].data[4] = 0x7F;
    TxMessage->motor[0].data[5] = 0x7F;
}
// set motor new ID
void MotorIDSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t motor_id_new)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 6;
    TxMessage->motor[0].rtr = 0;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x04;
    TxMessage->motor[0].data[4] = motor_id_new >> 8;
    TxMessage->motor[0].data[5] = motor_id_new & 0xff;
}
// read motor communication mode
void MotorCommModeReading(EtherCAT_Msg *TxMessage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 4;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x81;
}
// read motor ID
void MotorIDReading(EtherCAT_Msg *TxMessage)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 4;

    TxMessage->motor[0].data[0] = 0xFF;
    TxMessage->motor[0].data[1] = 0xFF;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x82;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
kp:0~500
kd:0~50
pos:-12.5rad~12.5rad
spd:-18rad/s~18rad/s
tor:-30Nm~30Nm
*/
void send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint16_t motor_id, float kp, float kd, float pos, float spd, float tor)
{
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;
    if (data_channel < 1 || data_channel > 6)
        return;
    TxMessage->can_ide = 1;
    TxMessage->motor[data_channel - 1].id = motor_id;
    TxMessage->motor[data_channel - 1].rtr = 0;
    TxMessage->motor[data_channel - 1].dlc = 8;

        if (kp > KP_MAX)
            kp = KP_MAX;
        else if (kp < KP_MIN)
            kp = KP_MIN;
        if (kd > KD_MAX)
            kd = KD_MAX;
        else if (kd < KD_MIN)
            kd = KD_MIN;
        if (pos > POS_MAX)
            pos = POS_MAX;
        else if (pos < POS_MIN)
            pos = POS_MIN;
        if (spd > SPD_MAX)
            spd = SPD_MAX;
        else if (spd < SPD_MIN)
            spd = SPD_MIN;
        kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
        kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
        pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
        spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
        switch (motor_id)
        {
        case 1:
            if (tor > tor_max[0])
                tor = tor_max[0];
            else if (tor < tor_min[0])
                tor = tor_min[0];
            tor_int = float_to_uint(tor, tor_min[0], tor_max[0], 12);
            break;
        case 2:
            if (tor > tor_max[1])
                tor = tor_max[1];
            else if (tor < tor_min[1])
                tor = tor_min[1];
            tor_int = float_to_uint(tor, tor_min[1], tor_max[1], 12);
            break;
        case 3:
            if (tor > tor_max[2])
                tor = tor_max[2];
            else if (tor < tor_min[2])
                tor = tor_min[2];
            tor_int = float_to_uint(tor, tor_min[2], tor_max[2], 12);
            break;
        case 4:
            if (tor > tor_max[3])
                tor = tor_max[3];
            else if (tor < tor_min[3])
                tor = tor_min[3];
            tor_int = float_to_uint(tor, tor_min[3], tor_max[3], 12);
            break;
        case 5:
            if (tor > tor_max[4])
                tor = tor_max[4];
            else if (tor < tor_min[4])
                tor = tor_min[4];
            tor_int = float_to_uint(tor, tor_min[4], tor_max[4], 12);
            break;
        case 6:
            if (tor > tor_max[5])
                tor = tor_max[5];
            else if (tor < tor_min[5])
                tor = tor_min[5];
            tor_int = float_to_uint(tor, tor_min[5], tor_max[5], 12);
            break;
        default:
            break;
        }
 

    TxMessage->motor[data_channel - 1].data[0] = 0x00 | (kp_int >> 7);                             // kp5
    TxMessage->motor[data_channel - 1].data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    TxMessage->motor[data_channel - 1].data[2] = kd_int & 0xFF;
    TxMessage->motor[data_channel - 1].data[3] = pos_int >> 8;
    TxMessage->motor[data_channel - 1].data[4] = pos_int & 0xFF;
    TxMessage->motor[data_channel - 1].data[5] = spd_int >> 4;
    TxMessage->motor[data_channel - 1].data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    TxMessage->motor[data_channel - 1].data[7] = tor_int & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
pos:float
spd:0~18000
cur:0~3000
ack_status:0~3
*/
void set_motor_position(EtherCAT_Msg *TxMessage, uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status)
{
    int i = motor_id - 1;

    TxMessage->can_ide = 0;
    TxMessage->motor[i].rtr = 0;
    TxMessage->motor[i].id = motor_id;
    TxMessage->motor[i].dlc = 8;

    if (ack_status > 3)
        return;

    rv_type_convert.to_float = pos;
    TxMessage->motor[i].data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
    TxMessage->motor[i].data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
    TxMessage->motor[i].data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
    TxMessage->motor[i].data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
    TxMessage->motor[i].data[4] = (rv_type_convert.buf[0] << 5) | (spd >> 10);
    TxMessage->motor[i].data[5] = (spd & 0x3FC) >> 2;
    TxMessage->motor[i].data[6] = (spd & 0x03) << 6 | (cur >> 6);
    TxMessage->motor[i].data[7] = (cur & 0x3F) << 2 | ack_status;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
spd:-18000~18000
cur:0~3000
ack_status:0~3
*/
void set_motor_speed(EtherCAT_Msg *TxMessage, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status)
{
    int i = motor_id - 1;

    TxMessage->can_ide = 0;
    TxMessage->motor[i].rtr = 0;
    TxMessage->motor[i].id = motor_id;
    TxMessage->motor[i].dlc = 7;

    rv_type_convert.to_float = spd;
    TxMessage->motor[i].data[0] = 0x40 | ack_status;
    TxMessage->motor[i].data[1] = rv_type_convert.buf[3];
    TxMessage->motor[i].data[2] = rv_type_convert.buf[2];
    TxMessage->motor[i].data[3] = rv_type_convert.buf[1];
    TxMessage->motor[i].data[4] = rv_type_convert.buf[0];
    TxMessage->motor[i].data[5] = cur >> 8;
    TxMessage->motor[i].data[6] = cur & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
cur:-3000~3000
ctrl_status:
    0:current control
    1:torque control
    2:variable damping brake control(also call full brake)
    3:dynamic brake control
    4:regenerative brake control
    5:NON
    6:NON
    7:NON
ack_status:0~3
*/
void set_motor_cur_tor(EtherCAT_Msg *TxMessage, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status)
{
    int i = motor_id - 1;

    TxMessage->can_ide = 0;
    TxMessage->motor[i].rtr = 0;
    TxMessage->motor[i].id = motor_id;
    TxMessage->motor[i].dlc = 3;

    if (ack_status > 3)
        return;
    if (ctrl_status > 7)
        return;
    if (ctrl_status) 
    {
        if (cur_tor > 3000)
            cur_tor = 3000;
        else if (cur_tor < -3000)
            cur_tor = -3000;
    }
    else
    {
        if (cur_tor > 2000)
            cur_tor = 2000;
        else if (cur_tor < -2000)
            cur_tor = -2000;
    }

    TxMessage->motor[i].data[0] = 0x60 | ctrl_status << 2 | ack_status;
    TxMessage->motor[i].data[1] = cur_tor >> 8;
    TxMessage->motor[i].data[2] = cur_tor & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
acc:0~2000
ack_status:0~3
*/
void set_motor_acceleration(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t acc, uint8_t ack_status)
{
    int i = motor_id - 1;

    TxMessage->can_ide = 0;
    TxMessage->motor[i].rtr = 0;
    TxMessage->motor[i].id = motor_id;
    TxMessage->motor[i].dlc = 4;

    if (ack_status > 2)
        return;
    if (acc > 2000)
        acc = 2000;

    TxMessage->motor[i].data[0] = 0xC0 | ack_status;
    TxMessage->motor[i].data[1] = 0x01;
    TxMessage->motor[i].data[2] = acc >> 8;
    TxMessage->motor[i].data[3] = acc & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
linkage:0~10000
speedKI:0~10000
ack_status:0/1
*/
void set_motor_linkage_speedKI(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t linkage, uint16_t speedKI, uint8_t ack_status)
{
    int i = motor_id - 1;

    TxMessage->can_ide = 0;
    TxMessage->motor[i].rtr = 0;
    TxMessage->motor[i].id = motor_id;
    TxMessage->motor[i].dlc = 6;

    if (ack_status > 2)
        return;
    if (linkage > 10000)
        linkage = 10000;
    if (speedKI > 10000)
        speedKI = 10000;

    TxMessage->motor[i].data[0] = 0xC0 | ack_status;
    TxMessage->motor[i].data[1] = 0x02;
    TxMessage->motor[i].data[2] = linkage >> 8;
    TxMessage->motor[i].data[3] = linkage & 0xff;
    TxMessage->motor[i].data[4] = speedKI >> 8;
    TxMessage->motor[i].data[5] = speedKI & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
fdbKP:0~10000
fbdKD:0~10000
ack_status:0/1
*/
void set_motor_feedbackKP_KD(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t fdbKP, uint16_t fdbKD, uint8_t ack_status)
{
    int i = motor_id - 1;

    TxMessage->can_ide = 0;
    TxMessage->motor[i].rtr = 0;
    TxMessage->motor[i].id = motor_id;
    TxMessage->motor[i].dlc = 6;

    if (ack_status > 2)
        return;
    if (fdbKP > 10000)
        fdbKP = 10000;
    if (fdbKD > 10000)
        fdbKD = 10000;

    TxMessage->motor[i].data[0] = 0xC0 | ack_status;
    TxMessage->motor[i].data[1] = 0x03;
    TxMessage->motor[i].data[2] = fdbKP >> 8;
    TxMessage->motor[i].data[3] = fdbKP & 0xff;
    TxMessage->motor[i].data[4] = fdbKD >> 8;
    TxMessage->motor[i].data[5] = fdbKD & 0xff;
}
// This function use in ask communication mode.
/*
motor_id:1~0x7FE
param_cmd:1~9
*/
void get_motor_parameter(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint8_t param_cmd) // 未使用
{
    int i = motor_id - 1; 

    TxMessage->can_ide = 0;
    TxMessage->motor[i].rtr = 0;
    TxMessage->motor[i].id = motor_id;
    TxMessage->motor[i].dlc = 2;

    TxMessage->motor[i].data[0] = 0xE0;
    TxMessage->motor[i].data[1] = param_cmd;
}

void Rv_Message_Print(uint8_t ack_status) {
  if (ack_status == 0) {
    if (motor_comm_fbd.motor_fbd == 0x01) {
      printf("自动模式.\n");
    } else if (motor_comm_fbd.motor_fbd == 0x02) {
      printf("问答模式.\n");
    } else if (motor_comm_fbd.motor_fbd == 0x03) {
      printf("零点设置成功.\n");
    } else if (motor_comm_fbd.motor_fbd == 0x04) {
      printf("新设置的Id为: %d\n", motor_comm_fbd.motor_id);
    } else if (motor_comm_fbd.motor_fbd == 0x05) {
      printf("重置Id成功.\n");
    } else if (motor_comm_fbd.motor_fbd == 0x06) {
      printf("当前电机Id: %d\n", motor_comm_fbd.motor_id);
    } else if (motor_comm_fbd.motor_fbd == 0x80) {
      printf("查询失败.\n");
    }
  } else {
    for (int i = 0; i < 6; ++i) {                  
      if (rv_motor_msg[i].motor_id == 0) {
        continue;
      }
      switch (ack_status) {
      case 1:
        printf("motor id: %d\n", rv_motor_msg[i].motor_id);
        printf("angle_actual_rad: %f\n", rv_motor_msg[i].angle_actual_rad);
        printf("speed_actual_rad: %f\n", rv_motor_msg[i].speed_actual_rad);
        printf("current_actual_float: %f\n",
               rv_motor_msg[i].current_actual_float);
        printf("temperature: %d\n", rv_motor_msg[i].temperature);
        break;
      case 2:
        printf("motor id: %d\n", rv_motor_msg[i].motor_id);
        printf("angle_actual_float: %f\n", rv_motor_msg[i].angle_actual_float);
        printf("current_actual_int:%d\n", rv_motor_msg[i].current_actual_int);
        printf("temperature:%d\n", rv_motor_msg[i].temperature);
        printf("current_actual_float:%f\n",
               rv_motor_msg[i].current_actual_float);
        break;
      case 3:
        printf("motor id: %d\n", rv_motor_msg[i].motor_id);
        printf("speed_actual_float: %f\n", rv_motor_msg[i].speed_actual_float);
        printf("current_actual_int:%d\n", rv_motor_msg[i].current_actual_int);
        printf("temperature:%d\n", rv_motor_msg[i].temperature);
        printf("current_actual_float:%f\n",
               rv_motor_msg[i].current_actual_float);
        break;
      case 4:
        if (motor_comm_fbd.motor_fbd == 0) {
          printf("配置成功.\n");
        } else {
          printf("配置失败.\n");
        }
        break;
      case 5:
        printf("motor id: %d\n", rv_motor_msg[i].motor_id);
        if (motor_comm_fbd.INS_code == 1) {
          printf("angle_actual_float: %f\n",
                 rv_motor_msg[i].angle_actual_float);
        } else if (motor_comm_fbd.INS_code == 2) {
          printf("speed_actual_float: %f\n",
                 rv_motor_msg[i].speed_actual_float);
        } else if (motor_comm_fbd.INS_code == 3) {
          printf("current_actual_float: %f\n",
                 rv_motor_msg[i].current_actual_float);
        } else if (motor_comm_fbd.INS_code == 4) {
          printf("power: %f\n", rv_motor_msg[i].power);
        } else if (motor_comm_fbd.INS_code == 5) {
          printf("acceleration: %d\n", rv_motor_msg[i].acceleration);
        } else if (motor_comm_fbd.INS_code == 6) {
          printf("linkage_KP: %d\n", rv_motor_msg[i].linkage_KP);
        } else if (motor_comm_fbd.INS_code == 7) {
          printf("speed_KI: %d\n", rv_motor_msg[i].speed_KI);
        } else if (motor_comm_fbd.INS_code == 8) {
          printf("feedback_KP: %d\n", rv_motor_msg[i].feedback_KP);
        } else if (motor_comm_fbd.INS_code == 9) {
          printf("feedback_KD: %d\n", rv_motor_msg[i].feedback_KD);
        }
        break;
      case 6:
        printf("motor id: %d\n", rv_motor_msg[i].motor_id);
        printf("angle_actual_int: %d\n", rv_motor_msg[i].angle_actual_int);
        printf("speed_actual_int: %d\n", rv_motor_msg[i].speed_actual_int);
        printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
        printf("temperature: %d\n", rv_motor_msg[i].temperature);
        break;
      default:
        break;
      }
    }
  }
}

uint16_t motor_id_check = 0;
void RV_can_data_repack(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t slave_id)
{
    // printf("slave %d msg:\n", slave_id);
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;
    for (int i = 0; i < 6; ++i)
    {
        if (RxMessage->motor[i].dlc == 0 )//&& RxMessage->motor[i].id == 0
        {
        // printf("RxMessage->motor[i].id == %d\n",RxMessage->motor[i].id);
            continue;
        }
        
        if (RxMessage->motor[i].id == 0x7FF)
        {
            printf("111111111111111111111111111111111111111111111111111:\n");
            if (RxMessage->motor[i].data[2] != 0x01) 
                return;                              
            if ((RxMessage->motor[i].data[0] == 0xff) && (RxMessage->motor[i].data[1] == 0xFF))
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[3] << 8 | RxMessage->motor[i].data[4];
                motor_comm_fbd.motor_fbd = 0x06;
            }
            else if ((RxMessage->motor[i].data[0] == 0x80) && (RxMessage->motor[i].data[1] == 0x80)) 
            {
                motor_comm_fbd.motor_id = 0;
                motor_comm_fbd.motor_fbd = 0x80;
            }
            else if ((RxMessage->motor[i].data[0] == 0x7F) && (RxMessage->motor[i].data[1] == 0x7F)) 
            {
                motor_comm_fbd.motor_id = 1;
                motor_comm_fbd.motor_fbd = 0x05;
            }
            else
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[3];
            }
            Rv_Message_Print(0);
        }
        else if (comm_mode == 0x00 && RxMessage->motor[i].dlc != 0) 
        {
            // printf("22222222222222222222222222222222222222222222222222222:\n");
            ack_status = RxMessage->motor[i].data[0] >> 5; 
            motor_id_t = RxMessage->motor[i].id - 1;
            motor_id_check = RxMessage->motor[i].id;
            rv_motor_msg[motor_id_t].motor_id = motor_id_check;
            rv_motor_msg[motor_id_t].error = RxMessage->motor[i].data[0] & 0x1F;

            // rv_motor_msg[i].angle_actual_rad = 0.0;
            // rv_motor_msg[i].speed_actual_rad = 0.0;
            // rv_motor_msg[i].current_actual_float = 0.0;

            if (ack_status == 1) // response frame 1
            {
                pos_int = RxMessage->motor[i].data[1] << 8 | RxMessage->motor[i].data[2];
                spd_int = RxMessage->motor[i].data[3] << 4 | (RxMessage->motor[i].data[4] & 0xF0) >> 4;
                cur_int = (RxMessage->motor[i].data[4] & 0x0F) << 8 | RxMessage->motor[i].data[5];
                rv_motor_msg[motor_id_t].angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
                rv_motor_msg[motor_id_t].speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[6] - 50) / 2;
                // printf("rv_motor_msg[%d]:%f\n",motor_id_t,rv_motor_msg[motor_id_t].speed_actual_rad);
                switch (motor_id_t)
                {
                case 0:
                    rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, i_min[0], i_max[0], 12);
                    break;
                case 1:
                    rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, i_min[1], i_max[1], 12);
                    break;
                case 2:
                    rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, i_min[2], i_max[2], 12);
                    break;
                case 3:
                    rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, i_min[3], i_max[3], 12);
                    break;
                case 4:
                    rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, i_min[4], i_max[4], 12);
                    break;
                case 5:
                    rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, i_min[5], i_max[5], 12);
                    break;
                default:
                break;
                }
            }
            else if (ack_status == 2) 
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
                rv_motor_msg[motor_id_t].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[7] - 50) / 2;
                rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
            }
            else if (ack_status == 3) 
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
                rv_motor_msg[motor_id_t].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[7] - 50) / 2;
                rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
            }
            else if (ack_status == 4) 
            {
                if (RxMessage->motor[i].dlc != 3)
                    return;
                motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[2];
            }
            else if (ack_status == 5) 
            {
                motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
                if (motor_comm_fbd.INS_code == 1 && RxMessage->motor[i].dlc == 6) 
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 2 && RxMessage->motor[i].dlc == 6) 
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 3 && RxMessage->motor[i].dlc == 6) 
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].current_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 4 && RxMessage->motor[i].dlc == 6) 
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].power = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 5 && RxMessage->motor[i].dlc == 4) 
                {
                    rv_motor_msg[motor_id_t].acceleration = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 6 && RxMessage->motor[i].dlc == 4) 
                {
                    rv_motor_msg[motor_id_t].linkage_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 7 && RxMessage->motor[i].dlc == 4) 
                {
                    rv_motor_msg[motor_id_t].speed_KI = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 8 && RxMessage->motor[i].dlc == 4) 
                {
                    rv_motor_msg[motor_id_t].feedback_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 9 && RxMessage->motor[i].dlc == 4) 
                {
                    rv_motor_msg[motor_id_t].feedback_KD = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
            }
        }
        else if (comm_mode == 0x01 && RxMessage->motor[i].dlc != 0) 
        {
            motor_id_t = RxMessage->motor[i].id - 0x205;
            rv_motor_msg[motor_id_t].motor_id = RxMessage->motor[i].id;
            rv_motor_msg[motor_id_t].angle_actual_int = (uint16_t)(RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1]);
            rv_motor_msg[motor_id_t].speed_actual_int = (int16_t)(RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3]);
            rv_motor_msg[motor_id_t].current_actual_int = (RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5]);
            rv_motor_msg[motor_id_t].temperature = RxMessage->motor[i].data[6];
            rv_motor_msg[motor_id_t].error = RxMessage->motor[i].data[7];
            Rv_Message_Print(6);
        }
    }
    // Rv_Message_Print(1);
}

// void RV_can_imu_data_repack(EtherCAT_Msg *RxMessage)
// {
//     uint16_t angle_uint[3], gyro_uint[3], accel_uint[3], mag_uint[3];

//     rv_type_convert2.buf[0] = RxMessage->motor[0].data[1];
//     rv_type_convert2.buf[1] = RxMessage->motor[0].data[0];
//     angle_uint[0] = rv_type_convert2.to_uint16;
//     rv_type_convert2.buf[0] = RxMessage->motor[0].data[3];
//     rv_type_convert2.buf[1] = RxMessage->motor[0].data[2];
//     angle_uint[1] = rv_type_convert2.to_uint16;
//     rv_type_convert2.buf[0] = RxMessage->motor[0].data[5];
//     rv_type_convert2.buf[1] = RxMessage->motor[0].data[4];
//     angle_uint[2] = rv_type_convert2.to_uint16;

//     rv_type_convert2.buf[0] = RxMessage->motor[0].data[7];
//     rv_type_convert2.buf[1] = RxMessage->motor[0].data[6];
//     gyro_uint[0] = rv_type_convert2.to_uint16;
//     rv_type_convert2.buf[0] = RxMessage->motor[1].data[1];
//     rv_type_convert2.buf[1] = RxMessage->motor[1].data[0];
//     gyro_uint[1] = rv_type_convert2.to_uint16;
//     rv_type_convert2.buf[0] = RxMessage->motor[1].data[3];
//     rv_type_convert2.buf[1] = RxMessage->motor[1].data[2];
//     gyro_uint[2] = rv_type_convert2.to_uint16;

//     rv_type_convert2.buf[0] = RxMessage->motor[1].data[5];
//     rv_type_convert2.buf[1] = RxMessage->motor[1].data[4];
//     accel_uint[0] = rv_type_convert2.to_uint16;
//     rv_type_convert2.buf[0] = RxMessage->motor[1].data[7];
//     rv_type_convert2.buf[1] = RxMessage->motor[1].data[6];
//     accel_uint[1] = rv_type_convert2.to_uint16;
//     rv_type_convert2.buf[0] = RxMessage->motor[2].data[1];
//     rv_type_convert2.buf[1] = RxMessage->motor[2].data[0];
//     accel_uint[2] = rv_type_convert2.to_uint16;

//     rv_type_convert2.buf[0] = RxMessage->motor[2].data[3];
//     rv_type_convert2.buf[1] = RxMessage->motor[2].data[2];
//     mag_uint[0] = rv_type_convert2.to_uint16;
//     rv_type_convert2.buf[0] = RxMessage->motor[2].data[5];
//     rv_type_convert2.buf[1] = RxMessage->motor[2].data[4];
//     mag_uint[1] = rv_type_convert2.to_uint16;
//     rv_type_convert2.buf[0] = RxMessage->motor[2].data[7];
//     rv_type_convert2.buf[1] = RxMessage->motor[2].data[6];
//     mag_uint[2] = rv_type_convert2.to_uint16;

//     rv_type_convert.buf[0] = RxMessage->motor[3].data[3];
//     rv_type_convert.buf[1] = RxMessage->motor[3].data[2];
//     rv_type_convert.buf[2] = RxMessage->motor[3].data[1];
//     rv_type_convert.buf[3] = RxMessage->motor[3].data[0];

//     imu_msg.quat_float[0] = rv_type_convert.to_float;
//     rv_type_convert.buf[0] = RxMessage->motor[3].data[7];
//     rv_type_convert.buf[1] = RxMessage->motor[3].data[6];
//     rv_type_convert.buf[2] = RxMessage->motor[3].data[5];
//     rv_type_convert.buf[3] = RxMessage->motor[3].data[4];

//     imu_msg.quat_float[1] = rv_type_convert.to_float;
//     rv_type_convert.buf[0] = RxMessage->motor[4].data[3];
//     rv_type_convert.buf[1] = RxMessage->motor[4].data[2];
//     rv_type_convert.buf[2] = RxMessage->motor[4].data[1];
//     rv_type_convert.buf[3] = RxMessage->motor[4].data[0];
   
//     imu_msg.quat_float[2] = rv_type_convert.to_float;
//     rv_type_convert.buf[0] = RxMessage->motor[4].data[7];
//     rv_type_convert.buf[1] = RxMessage->motor[4].data[6];
//     rv_type_convert.buf[2] = RxMessage->motor[4].data[5];
//     rv_type_convert.buf[3] = RxMessage->motor[4].data[4];
   
//     imu_msg.quat_float[3] = rv_type_convert.to_float;

//     imu_msg.angle_float[0] = uint_to_float(angle_uint[0], -3.5, 3.5, 16);
//     imu_msg.angle_float[1] = uint_to_float(angle_uint[1], -3.5, 3.5, 16);
//     imu_msg.angle_float[2] = uint_to_float(angle_uint[2], -3.5, 3.5, 16);

//     imu_msg.gyro_float[0] = uint_to_float(gyro_uint[0], -35.0, 35.0, 16);
//     imu_msg.gyro_float[1] = uint_to_float(gyro_uint[1], -35.0, 35.0, 16);
//     imu_msg.gyro_float[2] = uint_to_float(gyro_uint[2], -35.0, 35.0, 16);

//     imu_msg.accel_float[0] = uint_to_float(accel_uint[0], -240.0, 240.0, 16);
//     imu_msg.accel_float[1] = uint_to_float(accel_uint[1], -240.0, 240.0, 16);
//     imu_msg.accel_float[2] = uint_to_float(accel_uint[2], -240.0, 240.0, 16);

//     imu_msg.mag_float[0] = uint_to_float(mag_uint[0], -250.0, 250.0, 16);
//     imu_msg.mag_float[1] = uint_to_float(mag_uint[1], -250.0, 250.0, 16);
//     imu_msg.mag_float[2] = uint_to_float(mag_uint[2], -250.0, 250.0, 16);
// }
