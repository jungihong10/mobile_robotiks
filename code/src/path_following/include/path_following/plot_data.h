#ifndef PLOT_DATA_H
#define PLOT_DATA_H

#include <fstream>

struct PlotData {
    double measured;
    double desired;
    double error;
};

std::ostream &operator<<(std::ostream &out, const PlotData &data);

#endif
