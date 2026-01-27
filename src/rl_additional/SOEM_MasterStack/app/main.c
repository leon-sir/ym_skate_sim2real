/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-13 19:00:55
 * @LastEditTime: 2022-11-13 17:09:03
 */

#include <stdio.h>
#include "config.h"
#include "ethercat.h"
#include "motor_control.h"
#include "unistd.h"
int i, j;
int main()
{
    printf("SOEM 主站测试\n");

    // 这里填自己电脑上的网卡
    EtherCAT_Init("enp2s0"); // enx68da73a9766aenx68da73a9766a:enx68da73a9766a:

    if (ec_slavecount <= 0)
    {
        printf("未找到从站, 程序退出！");
        return 1;
    }
    else
        printf("从站数量： %d\r\n", ec_slavecount);

    while (1)
    {
        // if(j<200)
        // {
        EtherCAT_Run();
        usleep(1000);
        //     }
        //  j++;
    }

    return 0;
}
