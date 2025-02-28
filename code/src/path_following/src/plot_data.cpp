#include "plot_data.h"

std::ostream &operator<<(std::ostream &out, const PlotData &data) {
    out << data.measured << " " << data.desired << " " << data.error << std::endl;
    return out;
}
