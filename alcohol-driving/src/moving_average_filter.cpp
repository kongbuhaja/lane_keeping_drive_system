#ifndef MOVING_AVERAGE_FILTER_H_
#define MOVING_AVERAGE_FILTER_H_
#include <deque>
#include <iostream>
#include <vector>

namespace xycar
{
    class MovingAverageFilter final
    {
    public:
        // Construct a new Moving Average Filter object
        MovingAverageFilter(int sample_size)
            : kSampleSize_(sample_size)
        {
            weight_.reserve(kSampleSize_);
            for (uint8_t i = 1; i <= kSampleSize_; ++i)
            {
                weight_.push_back(i);
            }
        }
        // Add new data to filter
        void addSample(int new_sample)
        {
            samples_.push_back(new_sample);
            if (samples_.size() > kSampleSize_)
            {
                samples_.pop_front();
            }
        }
        // Get filtered data
        float getWeightedMovingAverage()
        {
            int sum = 0, weight_sum = 0;
            for (uint8_t i = 0; i < samples_.size(); ++i)
            {
                sum += samples_[i] * weight_[i];
                weight_sum += weight_[i];
            }
            if (weight_sum == 0)
            {
                throw std::runtime_error("Weight sum is zero");
            }
            return (float)sum / weight_sum;
        }
        // Get filtered data
        float getMovingAverage()
        {
            int sum = 0, sample_size = samples_.size();
            for (uint8_t i = 0; i < sample_size; ++i)
            {
                sum += samples_[i];
            }
            return (float)sum / sample_size;
        }

    private:
        const int kSampleSize_;
        std::deque<int> samples_;
        std::vector<int> weight_;
    };
} // namespace xycar
#endif // MOVING_AVERAGE_FILTER_H_
/**
 * @file moving_average_filter.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @brief Moving Average Filter Class source file
 * @version 0.2
 * @date 2022-11-27
 */
#include "xyanteam1/moving_average_filter.h"

namespace xycar {
MovingAverageFilter::MovingAverageFilter(int sample_size)
    : kSampleSize_(sample_size) {
  weight_.reserve(kSampleSize_);
  for (uint8_t i = 1; i <= kSampleSize_; ++i) {
    weight_.push_back(i);
  }
}

void MovingAverageFilter::addSample(int new_sample) {
  samples_.push_back(new_sample);
  if (samples_.size() > kSampleSize_) {
    samples_.pop_front();
  }
}

float MovingAverageFilter::getMovingAverage() {
  int sum = 0, sample_size = samples_.size();
  for (uint8_t i = 0; i < sample_size; ++i) {
    sum += samples_[i];
  }
  return (float)sum / sample_size;
}

float MovingAverageFilter::getWeightedMovingAverage() {
  int sum = 0, weight_sum = 0;
  for (uint8_t i = 0; i < samples_.size(); ++i) {
    sum += samples_[i] * weight_[i];
    weight_sum += weight_[i];
  }
  if (weight_sum == 0) {
   throw std::runtime_error("Weight sum is zero");
  }
  return (float)sum / weight_sum;
}
}  // namespace xycar
