#pragma once

class OnlineStatistics {
public:
    OnlineStatistics();

    void update(double);

    double getMean();
    double getStandardDeviation();

    int getSampleMeasurements() const;

private:
    int measurements;
    double mean;
    double std;
};
