#pragma once
#ifndef SHAREDMEMORY_HPP
#define SHAREDMEMORY_HPP
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <cstring>
#include <iostream>
#include <sys/stat.h> // 为了 fchmod()

#define JOINT_MOTOR_NUMBER 14
// #define SHM_NAME "/joint_shared_memory"
#define SHM_NAME_MOTOR "/joint_shared_memory"

struct EncosMotorData
{
    double pos_, vel_, tau_;
    double pos_des_, vel_des_, kp_, kd_, ff_;
};
struct RecEncosMotorData
{
    double pos_, vel_, tau_;
};
struct SendEncosMotorData
{
    double pos_des_, vel_des_, kp_, kd_, ff_;
};
struct SharedData
{
    pthread_mutex_t mutex;
    // EncosMotorData motorData[JOINT_MOTOR_NUMBER];
    RecEncosMotorData jointDataRec_[JOINT_MOTOR_NUMBER];
    SendEncosMotorData jointDataSend_[JOINT_MOTOR_NUMBER];
};

class SharedMemory
{
public:
    SharedMemory(bool isCreator)
    {
        int shm_fd_joint;
        if (isCreator)
        {
            shm_fd_joint = shm_open(SHM_NAME_MOTOR, O_CREAT | O_RDWR, 0666);
            fchmod(shm_fd_joint, 0666); // 加上这一行
            ftruncate(shm_fd_joint, sizeof(SharedData));
        }
        else
        {
            shm_fd_joint = shm_open(SHM_NAME_MOTOR, O_RDWR, 0666);
            fchmod(shm_fd_joint, 0666); // 加上这一行
        }

        if (shm_fd_joint < 0)
        {
            perror("shm_open");
            exit(1);
        }

        data_ = (SharedData *)mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_joint, 0);
        if (data_ == MAP_FAILED)
        {
            perror("mmap");
            exit(1);
        }

        if (isCreator)
        {
            pthread_mutexattr_t attr;
            pthread_mutexattr_init(&attr);
            pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
            pthread_mutex_init(&data_->mutex, &attr);
            pthread_mutexattr_destroy(&attr);

            // 初始化数据
            memset(data_->jointDataRec_, 0, sizeof(data_->jointDataRec_));
            memset(data_->jointDataSend_, 0, sizeof(data_->jointDataSend_));
        }
    }

    ~SharedMemory()
    {
        munmap(data_, sizeof(SharedData));
        // Note: 不在析构中 unlink，防止多进程误删
    }

    void writeJointData(const RecEncosMotorData (&input)[JOINT_MOTOR_NUMBER])
    {
        pthread_mutex_lock(&data_->mutex);
        memcpy(data_->jointDataRec_, input, sizeof(data_->jointDataRec_));
        pthread_mutex_unlock(&data_->mutex);
    }

    void readJointData(SendEncosMotorData (&output)[JOINT_MOTOR_NUMBER])
    {
        pthread_mutex_lock(&data_->mutex);
        memcpy(output, data_->jointDataSend_, sizeof(data_->jointDataSend_));
        //   std::cout<< "data_->jointDataSend_ .."<<data_->jointDataSend_[0].pos_des_ << std::endl;
       
        pthread_mutex_unlock(&data_->mutex);
    }

    void writeJointDatatoMotor(const SendEncosMotorData (&input)[JOINT_MOTOR_NUMBER])
    {
        pthread_mutex_lock(&data_->mutex);
        memcpy(data_->jointDataSend_, input, sizeof(data_->jointDataSend_));

        // std::cout<< "data_->jointDataSend_ .."<<data_->jointDataSend_[0].pos_des_ << std::endl;
        
        pthread_mutex_unlock(&data_->mutex);
    }

    void readJointDatafromMotor(RecEncosMotorData (&output)[JOINT_MOTOR_NUMBER])
    {
        pthread_mutex_lock(&data_->mutex);
        memcpy(output, data_->jointDataRec_, sizeof(data_->jointDataRec_));
        // std::cout<< "data_->data_->jointDataRec_ .."<<data_->jointDataRec_[0].pos_ << std::endl;
        pthread_mutex_unlock(&data_->mutex);
    }

private:
    SharedData *data_;
};

#endif