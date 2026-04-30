#pragma once
#include <vector>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <cmath>


//循环队列：用于存储固定大小的数据窗
class CircularQueue {
public:
    int cap_;                           // 队列容量
    std::vector<float> circularQueue_;  // 存储数据的容器 
    int front_index_;                   // 队头索引
    int rear_index_;                    // 队尾索引
    
    CircularQueue(int capacity);
    void push(const float& data);       // 添加数据
    void pop();                         // 移除最旧数据
    size_t get_size() const;            // 获取当前数据数量
    bool isFull() const;                // 检查是否已满
    bool isEmpty() const;               // 检查是否为空
    const float& get_front() const;     // 获取最新添加的数据（队头）
    const float& get_rear() const;      // 获取最早添加的数据（队尾）
}; 

//滑动平均滤波器：计算窗口内数据的平均值
//窗口越大，平滑效果越好，但延迟越大
class MovAvg {
private:
    int window_size_;               // 窗口大小
    CircularQueue dataQueue_;       // 用于存储数据的循环队列

public:
    MovAvg(int window_size = 7);
    float update(float data);
};

//中值滤波器：计算窗口内数据的中值
//对突变噪声有很好的抑制效果，同时保持边缘特征
class MedianFilter {
private:
    int window_size_;               // 窗口大小
    std::deque<float> data_buffer_; // 存储历史数据

public:
    MedianFilter(int window_size = 5);
    float update(float data);
};

