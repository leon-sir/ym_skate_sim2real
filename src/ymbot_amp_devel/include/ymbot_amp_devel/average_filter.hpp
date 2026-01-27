#ifndef AVERAGE_FILTER_HPP
#define AVERAGE_FILTER_HPP
#include <iostream>
#include <deque>
#include <Eigen/Dense>

struct SensorData
{
    double x;
    double y;
    double z;
};

class AverageFilter
{
public:
    AverageFilter(size_t window_size)
        : window_size_(window_size) {}

    Eigen::VectorXd filter(const Eigen::VectorXd &new_sample)
    {
        // 如果第一次输入，则初始化维度
        if (buffer_.empty())
        {
            dimension_ = new_sample.size();
        }

        buffer_.push_back(new_sample);

        if (buffer_.size() > window_size_)
        {
            buffer_.pop_front();
        }

        Eigen::VectorXd sum = Eigen::VectorXd::Zero(dimension_);
        for (const auto &sample : buffer_)
        {
            sum += sample;
        }

        Eigen::VectorXd avg = sum / buffer_.size();
        return avg;
    }

private:
    size_t window_size_;
    size_t dimension_;
    std::deque<Eigen::VectorXd> buffer_;
};

#endif