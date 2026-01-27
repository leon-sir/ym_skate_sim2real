/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-13 19:00:55
 * @LastEditTime: 2022-11-13 17:09:03
 */

#include <stdio.h>
#include "config.h"
#include "unistd.h"
#include <chrono>
#include <iostream>
#include <thread>
#include "transmit.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sched.h>
// namespace cr = CppReadline;
// using ret = cr::Console::ReturnCode;

using Clock = std::chrono::high_resolution_clock;
int main()
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    pthread_t thread = pthread_self();
    struct sched_param param;
    param.sched_priority = 99; // 设置为最高优先级
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        std::cerr << "Failed to set realtime scheduler" << std::endl;
    }
    else
    {
        std::cout << "Realtime scheduler set to FIFO with priority 99" << std::endl;
    }

    printf("SOEM 主站测试\n");

    EtherCAT_Init("enp2s0"); //  enx000ec62dfedd

    if (ec_slavecount <= 0)
    {
        printf("未找到从站, 程序退出！");
        return 1;
    }
    else
        printf("从站数量： %d\r\n", ec_slavecount);
    //  std::chrono::high_resolution_clock::time_point lastTime_;

    // while (1)
    // {
    //     // EtherCAT_Run();
    //     // usleep(10000);}
    //     auto start_time11 = Clock::now();
    //     std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(start_time11 - lastTime_);

    //     lastTime_ = start_time11;

    //     std::cout << "cycle time: " << time_span.count() << std::endl;

    //     EtherCAT_Run();
    //     auto end_time = Clock::now();
    //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time11).count();
    //     // std::cout << "duration time2:2 " << duration << std::endl;
    //     // 计算需要睡眠的时间，以保持100Hz的频率
    //     std::this_thread::sleep_for(std::chrono::milliseconds(2) - std::chrono::milliseconds(duration));
    //     // std::chrono::microseconds sleep_time(std::chrono::microseconds(2000) - std::chrono::microseconds(duration));
    // }
    auto lastTime_ = std::chrono::steady_clock::now();
    std::ofstream outFile("example.txt");

    while (true)
    {
        // 获取当前时间
        auto start_time = std::chrono::steady_clock::now();
        auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(lastTime_ - start_time).count();
        //  std::cout << "cycle time: " << duration2 << std::endl;
        lastTime_ = start_time;
        // 运行 EtherCAT
        EtherCAT_Run();

        // 获取结束时间
        auto end_time = std::chrono::steady_clock::now();

        // 计算运行时间
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        outFile << duration2 << ",";
        outFile << duration << ",";

        // 目标周期时间 2ms = 2000 microseconds
        constexpr int target_cycle_time_us = 2000;

        // 计算需要休眠的时间
        int sleep_time_us = target_cycle_time_us - duration;
        outFile << sleep_time_us << "\n";
        if (sleep_time_us > 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_us));
        }
        else
        {
            // 如果执行时间超过 2ms，可以在这里记录警告或处理逻辑
            std::cerr << "Warning: Cycle overrun by " << -sleep_time_us << " microseconds" << std::endl;
        }
    }
    // std::chrono::microseconds sleep_time(10 * 1000); // 10毫秒转换为微秒
    // sleep_time -= std::chrono::microseconds(duration_us);

    return 0;
}