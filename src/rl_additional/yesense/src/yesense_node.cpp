#include "yesense_driver.h"
#include <pthread.h>
#include <sched.h>


// void* imuThreadFunction(void* arg) {
//     yesense::YesenseDriver* yesense_driver = static_cast<yesense::YesenseDriver*>(arg);
//     yesense_driver->run();
//     return nullptr;
// }
void* imuThreadFunction(void* arg) {
    yesense::YesenseDriver* yesense_driver = static_cast<yesense::YesenseDriver*>(arg);
    
    // 设置线程CPU亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset); // 清空CPU集
    CPU_SET(1, &cpuset); // 设置线程绑定到CPU核心1（从0开始计数）

    // 获取当前线程ID
    pthread_t thread = pthread_self();
    // 设置线程亲和性
    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0) {
        ROS_ERROR("Failed to set thread affinity");
    }

    yesense_driver->run();
    return nullptr;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"yesense_imu");
    ros::NodeHandle nh, nh_private("~");

    yesense::YesenseDriver yesense_driver(nh,nh_private);
    // 创建线程属性
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    
    
    // 设置线程调度策略为实时优先级调度
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    // 设置线程优先级
    sched_param param;
    param.sched_priority = 20;  // 根据系统设置合适的优先级范围
    pthread_attr_setschedparam(&attr, &param);

    // 创建线程并运行 yesense_driver.run()
    pthread_t imu_thread;
    if (pthread_create(&imu_thread, &attr, imuThreadFunction, &yesense_driver) != 0) {
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
