#pragma once

#include <deque>

class MedianFilter
{
public:
    explicit MedianFilter(int window_size = 5);
    float update(float data);

private:
    int window_size_;
    std::deque<float> data_buffer_;
};
