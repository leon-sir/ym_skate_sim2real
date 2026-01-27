#pragma once
#ifndef SHAREDMEMORYARM_HPP
#define SHAREDMEMORYARM_HPP
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <cstring>
#include <iostream>
#include <sys/stat.h> // 为了 fchmod()

#define JOINT_ARM_NUMBER 10
// #define SHM_NAME "/joint_shared_memory"
#define SHM_NAME_ARM "/arm_shared_memory"

struct EuArmData
{
    double pos_, vel_;
};
struct SharedDataArm
{
    pthread_mutex_t mutex;
    double pos_des_[JOINT_ARM_NUMBER];
    EuArmData dataRes_[JOINT_ARM_NUMBER];
};

class SharedMemoryArm
{
public:
    SharedMemoryArm(bool isCreator)
    {
        int shm_fd_joint;
        if (isCreator)
        {
            shm_fd_joint = shm_open(SHM_NAME_ARM, O_CREAT | O_RDWR, 0666);
            fchmod(shm_fd_joint, 0666); // 加上这一行
            ftruncate(shm_fd_joint, sizeof(SharedDataArm));
        }
        else
        {
            shm_fd_joint = shm_open(SHM_NAME_ARM, O_RDWR, 0666);
            fchmod(shm_fd_joint, 0666); // 加上这一行
        }

        if (shm_fd_joint < 0)
        {
            perror("shm_open  arm");
            exit(1);
        }

        dataArm_ = (SharedDataArm *)mmap(0, sizeof(SharedDataArm), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_joint, 0);
        if (dataArm_ == MAP_FAILED)
        {
            perror("mmap");
            exit(1);
        }

        if (isCreator)
        {
            pthread_mutexattr_t attr;
            pthread_mutexattr_init(&attr);
            pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
            pthread_mutex_init(&dataArm_->mutex, &attr);
            pthread_mutexattr_destroy(&attr);

            // 初始化数据
            memset(dataArm_->dataRes_, 0, sizeof(dataArm_->dataRes_));
            memset(dataArm_->pos_des_, 3.14, sizeof(dataArm_->pos_des_));
        }
    }

    ~SharedMemoryArm()
    {
        munmap(dataArm_, sizeof(SharedDataArm));
        // Note: 不在析构中 unlink，防止多进程误删
    }
    // write data ,read from motor
    void writeJointDataArm(const EuArmData (&input)[JOINT_ARM_NUMBER])
    {
        pthread_mutex_lock(&dataArm_->mutex);
        memcpy(dataArm_->dataRes_, input, sizeof(dataArm_->dataRes_));
        pthread_mutex_unlock(&dataArm_->mutex);
    }

    void readJointDataArm(double (&output)[JOINT_ARM_NUMBER])
    {
        pthread_mutex_lock(&dataArm_->mutex);
        memcpy(output, dataArm_->pos_des_, sizeof(dataArm_->pos_des_));
        // std::cout << "dataArm_->pos_des_ .." << dataArm_->pos_des_[4] << std::endl;

        pthread_mutex_unlock(&dataArm_->mutex);
    }

    void writeJointDatatoMotorArm(const double (&input)[JOINT_ARM_NUMBER])
    {
        pthread_mutex_lock(&dataArm_->mutex);
        memcpy(dataArm_->pos_des_, input, sizeof(dataArm_->pos_des_));
        // std::cout << "dataArm_->jointDataSend_ .." << dataArm_->pos_des_[4] << std::endl;

        pthread_mutex_unlock(&dataArm_->mutex);
    }

    void readJointDatafromMotorArm(EuArmData (&output)[JOINT_ARM_NUMBER])
    {
        pthread_mutex_lock(&dataArm_->mutex);
        memcpy(output, dataArm_->dataRes_, sizeof(dataArm_->dataRes_));
        pthread_mutex_unlock(&dataArm_->mutex);
    }

private:
    SharedDataArm *dataArm_;
};

#endif