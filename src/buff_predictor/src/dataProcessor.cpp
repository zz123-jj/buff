#include "dataProcessor.hpp"

//循环队列
CircularQueue::CircularQueue(int capacity) 
    : cap_(capacity),                   // 队列的最大容量
      circularQueue_(capacity, -1.0f),  // 创建一个vector，初始值都是-1.0
      front_index_(0),                  // 队头索引，指向下一个要插入的位置
      rear_index_(0) {}                 // 队尾索引，指向最早插入的数据

// 向队列中添加数据
void CircularQueue::push(const float& data) {
    if (isFull()) {
        throw std::runtime_error("队列满了，无法压入");
    }
    // 使用模运算实现循环：当索引超过容量时，自动回到0
    circularQueue_[front_index_ % cap_] = data;
    front_index_++;  // 队头索引向前移动
}

// 从队列中移除最旧的数据
void CircularQueue::pop() {
    if (isEmpty()) {
        throw std::runtime_error("队列空了，无法弹出");
    }
    rear_index_++;  // 队尾索引向前移动，相当于"删除"了最旧的数据
}

// 获取队列中当前有多少个数据
size_t CircularQueue::get_size() const {
    return front_index_ - rear_index_;
}

// 检查队列是否已满
bool CircularQueue::isFull() const {
    return get_size() == static_cast<size_t>(cap_);
}

// 检查队列是否为空
bool CircularQueue::isEmpty() const {
    return front_index_ == rear_index_;
}

// 获取最新添加的数据（队头）
const float& CircularQueue::get_front() const {
    if (isEmpty()) {
        throw std::runtime_error("队列空了，获取头部数据");
    }
    // front_index_-1 是最后一次push的位置
    return circularQueue_[(front_index_ - 1) % cap_];
}

// 获取最早添加的数据（队尾）
const float& CircularQueue::get_rear() const {
    if (isEmpty()) {
        throw std::runtime_error("队列空了，获取尾部数据");
    }
    return circularQueue_[rear_index_ % cap_];
}

//滑动平均滤波器
//维护一个固定大小的窗口，计算窗口内所有数据的平均值
MovAvg::MovAvg(int window_size) 
        : window_size_(window_size),  // 窗口大小
          dataQueue_(window_size){}   // 用于存储数据的循环队列
        
// 更新数据并返回滑动平均值
float MovAvg::update(float data) {
        if (dataQueue_.isFull()) {
            dataQueue_.pop();
        }
        dataQueue_.push(data);
        float sum = 0.0f;
        int size = dataQueue_.get_size();
        for (int i = 0; i < size; ++i) {
            int idx = (dataQueue_.rear_index_ + i) % dataQueue_.cap_;
            sum += dataQueue_.circularQueue_[idx];
        }
        return sum / size;
    }

//中值滤波器
//维护一个固定大小的窗口，计算窗口内数据的中值
MedianFilter::MedianFilter(int window_size)
    : window_size_(window_size) {}

// 更新数据并返回中值
float MedianFilter::update(float data) {
    // 添加新数据
    data_buffer_.push_back(data);
    
    // 如果缓冲区超过窗口大小，移除最旧的数据
    if (data_buffer_.size() > static_cast<size_t>(window_size_)) {
        data_buffer_.pop_front();
    }
    
    // 复制数据并排序
    std::vector<float> sorted_data(data_buffer_.begin(), data_buffer_.end());
    std::sort(sorted_data.begin(), sorted_data.end());
    
    // 返回中值
    return sorted_data[sorted_data.size() / 2];
}
