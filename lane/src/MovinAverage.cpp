#include "alcoholdriving/vision.h"

alcoholdriving::MovingAverage::MovingAverage() {}


alcoholdriving::MovingAverage::MovingAverage(const int n) : samples(n)
{
    for (int i = 1; i < n + 1; i++)
    {
        weights.push_back(i);
    }
}
void alcoholdriving::MovingAverage::add_sample(const float new_samples)
{
    if (data.size() == samples)
        data.erase(data.begin(), data.begin() + 1);
    data.push_back(new_samples);
}
float alcoholdriving::MovingAverage::get_mm()
{
    return std::accumulate(data.begin(), data.end(), float(0));
}

float  alcoholdriving::MovingAverage::get_wmm()
{
    float s = 0;
    for (int i = 0; i < data.size(); i++)
        s += data[i] * weights[i];
    return float(s) / std::accumulate(weights.begin(), weights.begin() + data.size(), float(0));
}
