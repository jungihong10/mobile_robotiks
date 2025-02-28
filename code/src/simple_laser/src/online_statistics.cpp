#include "../include/simple_laser/online_statistics.h"
#include <cmath>

// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm

OnlineStatistics::OnlineStatistics()
    :measurements(0), mean(0.0), std(0.0)
{}

void OnlineStatistics::update(double value) {
    measurements = measurements + 1;
    double delta = value - mean;
    mean = mean+ delta / measurements;
    double delta2 = value - mean;
    std = std+  delta * delta2;
}

double OnlineStatistics::getMean() {
    return mean;
}

double OnlineStatistics::getStandardDeviation() {
    if (measurements < 2) {
        return 0.0;  // ignoreing
    }
    return std::sqrt(std / (measurements - 1));
}

int OnlineStatistics::getSampleMeasurements() const{
    return measurements;
}

