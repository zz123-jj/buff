#include "dataProcessor.hpp"

#include <algorithm>
#include <vector>

MedianFilter::MedianFilter(int window_size)
    : window_size_(window_size)
{
}

float MedianFilter::update(float data)
{
    data_buffer_.push_back(data);
    if (data_buffer_.size() > static_cast<std::size_t>(window_size_)) {
        data_buffer_.pop_front();
    }

    std::vector<float> sorted_data(data_buffer_.begin(), data_buffer_.end());
    std::sort(sorted_data.begin(), sorted_data.end());
    return sorted_data[sorted_data.size() / 2];
}
