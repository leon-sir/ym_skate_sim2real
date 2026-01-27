#include <stdio.h>     /*标准输入输出的定义*/
#include <stdlib.h>    /*标准函数库定义*/
#include <unistd.h>    /*UNIX 标准函数定义*/
#include <sys/types.h> /**/
#include <sys/stat.h>
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h>   /*错误号定义*/
#include <sys/time.h>
#include <string.h>
#include <getopt.h>
#include "analysis_data.h"
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
#include <sched.h>
#include <ros/ros.h>
#include <ros/assert.h>
#include <serial/serial.h>
#include <pthread.h>
#include <sched.h>
#include <math.h>

#include <iostream>
#include <fstream> // 引入文件流库
#include <chrono>
#include <ctime>

#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
/*----------------------------------------------------------------------*/
#define TRUE 1
#define FALSE -1
#define RX_BUF_LEN 512

// using namespace boost::interprocess;
std::string port_;
/*----------------------------------------------------------------------*/
unsigned char g_recv_buf[512] = {0};
unsigned short g_recv_buf_idx = 0;
protocol_info_t g_output_info = {0};

/*----------------------------------------------------------------------*/
// struct IMUData {
//     double time;
//     float orientation_x;
//     float orientation_y;
//     float orientation_z;
//     float orientation_w;
//     float angular_velocity_x;
//     float angular_velocity_y;
//     float angular_velocity_z;
//     float linear_acceleration_x;
//     float linear_acceleration_y;
//     float linear_acceleration_z;
// };
typedef struct
{
    float angle_float[3];
    float gyro_float[3];
    float accel_float[3];
    float mag_float[3];
    float quat_float[4];
    float time;
} YKSIMUData;

void *thread_function(void *arg)
{
    // 线程代码
    // int *result = (int *)malloc(sizeof(int));
    // *result = 1; // 假设返回值为42

    pthread_mutex_t lock;
    const char *SHM_NAME = "/imu_shared";
    const size_t SHM_SIZE = sizeof(YKSIMUData);

    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        perror("shm_open");
        exit(0);
        // return 1;
        // pthread_exit((void *)result);
    }
    ftruncate(shm_fd, SHM_SIZE);

    YKSIMUData *shm_ptr = static_cast<YKSIMUData *>(mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));

    if (shm_ptr == MAP_FAILED)
    {
        std::cerr << "Failed to map shared memory" << std::endl;
        close(shm_fd);
        // return -1;
    }

    // struct IMUData *imu = (struct IMUData*)shm_ptr;
    // std::ofstream outFile("/home/pc/beijing_data/imudata_node.txt");
    // if (!outFile)
    // {
    //     std::cerr << "无法打开文件！" << std::endl;
    //     exit(0);
    // }

    int fd;
    int nread;
    char buffer[RX_BUF_LEN];
    char *dev = NULL;
    struct termios oldtio, newtio;

    unsigned short cnt = 0;
    int pos = 0;

    speed_t speed = B460800;
    // dev = "/dev/ttyUSB1";
    fd = open(port_.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        printf("Can't Open Serial Port!\n");
        exit(0);
    }

    printf("open serial port to decode msg!\n");

    // save to oldtio
    tcgetattr(fd, &oldtio);
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = speed | CS8 | CLOCAL | CREAD;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~PARENB;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSAFLUSH, &newtio);
    tcgetattr(fd, &oldtio);

    memset(buffer, 0, sizeof(buffer));

    ros::Rate rate(100);
    double timeloop = 0.0;

    while (ros::ok())
    {
        ros::Time last_time = ros::Time::now();
        double timestamp = ros::Time::now().toSec();
        nread = read(fd, buffer, RX_BUF_LEN);
        if (nread > 0)
        {
            // printf("nread = %d\n", nread);
            memcpy(g_recv_buf + g_recv_buf_idx, buffer, nread);
            g_recv_buf_idx += nread;
        }

        cnt = g_recv_buf_idx;
        pos = 0;
        if (cnt < PROTOCOL_MIN_LEN)
        {
            continue;
        }
        yis_cmd_response_t response;
        while (cnt > (unsigned int)0)
        {
            int ret = analysis_data(g_recv_buf + pos, cnt, &g_output_info, &response);
            if (analysis_done == ret) /*未查找到帧头*/
            {
                pos++;
                cnt--;
            }
            else if (data_len_err == ret)
            {
                break;
            }
            else if (crc_err == ret || analysis_ok == ret) /*删除已解析完的完整一帧*/
            {
                output_data_header_t *header = (output_data_header_t *)(g_recv_buf + pos);
                unsigned int frame_len = header->len + PROTOCOL_MIN_LEN;
                cnt -= frame_len;
                pos += frame_len;
                // memcpy(g_recv_buf, g_recv_buf + pos, cnt);

                if (analysis_ok == ret)
                {
                    printf("w: %f, x: %f, y: %f, z: %f\n",
                           g_output_info.quaternion_data0, g_output_info.quaternion_data1,
                           g_output_info.quaternion_data2, g_output_info.quaternion_data3);

                    printf("ax: %f, ay: %f, az: %f\n", g_output_info.accel_x, g_output_info.accel_y, g_output_info.accel_z);

                    // sensor_msgs::Imu g_imu_;

                    // g_imu_.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                    //     g_output_info.roll / 180.0 * M_PI, g_output_info.pitch / 180.0 * M_PI,
                    //     g_output_info.yaw / 180.0 * M_PI);

                    pthread_mutex_lock(&lock);
                    shm_ptr->time = timeloop;
                    shm_ptr->quat_float[0] = g_output_info.quaternion_data0;
                    shm_ptr->quat_float[1] = g_output_info.quaternion_data1;
                    shm_ptr->quat_float[2] = g_output_info.quaternion_data2;
                    shm_ptr->quat_float[3] = g_output_info.quaternion_data3;
                    // outFile << timeloop << ",";
                    // outFile << shm_ptr->quat_float[0] << ',' << shm_ptr->quat_float[1] << ',' << shm_ptr->quat_float[2] << ',' << shm_ptr->quat_float[3] << ',';
                    // shm_ptr->quat_float[0] = g_imu_.orientation.w;
                    // shm_ptr->quat_float[1] = g_imu_.orientation.x;
                    // shm_ptr->quat_float[2] = g_imu_.orientation.y;
                    // shm_ptr->quat_float[3] = g_imu_.orientation.z;

                    shm_ptr->accel_float[0] = g_output_info.accel_x;
                    shm_ptr->accel_float[1] = g_output_info.accel_y;
                    shm_ptr->accel_float[2] = g_output_info.accel_z;
                    // outFile << shm_ptr->accel_float[0] << ',' << shm_ptr->accel_float[1] << ',' << shm_ptr->accel_float[2] << ',';

                    shm_ptr->angle_float[0] = g_output_info.angle_x / 180.0 * M_PI;
                    shm_ptr->angle_float[1] = g_output_info.angle_y / 180.0 * M_PI;
                    shm_ptr->angle_float[2] = g_output_info.angle_z / 180.0 * M_PI;
                    // outFile << shm_ptr->angle_float[0] << ',' << shm_ptr->angle_float[1] << ',' << shm_ptr->angle_float[2] << std::endl;
                    pthread_mutex_unlock(&lock);
                    printf("Wx: %f, Wy: %f, Wz: %f\n", shm_ptr->angle_float[0], shm_ptr->angle_float[1], shm_ptr->angle_float[2]);
                }
            }
        }

        memcpy(g_recv_buf, g_recv_buf + pos, cnt);
        g_recv_buf_idx = cnt;
        tcflush(fd, TCIFLUSH);
        ros::Time now = ros::Time::now();
        ros::Duration elapsed = now - last_time;

        if (elapsed.toSec() > 0.011)
        { // 期望 0.01s 循环
            ROS_WARN("outoftime: %.4f sec", elapsed.toSec());
        }

        last_time = now; // 更新时间戳
        timeloop += 0.002;
        ros::spinOnce();
        rate.sleep();
    }
    munmap(shm_ptr, SHM_SIZE);

    close(shm_fd);
    close(fd);
    // outFile.close(); // 关闭文件
    // return NULL;
}

// int main(int argc, char **argv)
// {

//     pthread_t thread;
//     struct sched_param param;
//     int policy = SCHED_FIFO; // 使用实时调度策略

//     // 创建线程
//     pthread_create(&thread, NULL, thread_function, NULL);

//     // 设置线程优先级
//     param.sched_priority = 20; // 设置适当的优先级（范围取决于系统）
//     pthread_setschedparam(thread, policy, &param);

//     pthread_join(thread, NULL);
//     return 0;
//     // // 创建共享内存对象
//     // shared_memory_object shm(open_or_create, "IMUSharedMemory", read_write);
//     // shm.truncate(sizeof(SharedMemoryIMU));
//     // mapped_region region(shm, read_write);
//     // void *addr = region.get_address();
//     // SharedMemoryIMU *shared_mem = new (addr) SharedMemoryIMU;

//     //  // 删除共享内存
//     // shared_memory_object::remove("IMUSharedMemory");

// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yesense_imu");
    ros::NodeHandle nh, nh_private("~");

    nh_private.param("yesense_port", port_, port_);

    // 创建线程属性
    pthread_attr_t attr;
    pthread_attr_init(&attr);

    // 设置线程调度策略为实时优先级调度
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    // 设置线程优先级
    sched_param param;
    param.sched_priority = 20; // 根据系统设置合适的优先级范围
    pthread_attr_setschedparam(&attr, &param);

    // 创建线程并运行 yesense_driver.run()
    pthread_t imu_thread;
    if (pthread_create(&imu_thread, &attr, thread_function, NULL) != 0)
    {
        ROS_ERROR("Failed to create IMU thread with custom priority.");
        return 1;
    }

    // 销毁线程属性
    pthread_attr_destroy(&attr);

    // 主线程继续运行其他 ROS 回调
    ros::spin();
    // 等待 IMU 线程结束
    pthread_join(imu_thread, nullptr);
    return 0;
}
