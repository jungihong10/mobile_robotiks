#ifndef WRITE_PLOT_DATA_HPP
#define WRITE_PLOT_DATA_HPP

#include <iostream>
#include <fstream>
#include <stdexcept>

namespace write_plot_data {
struct PlotDataWriter {
    std::ofstream out;

    PlotDataWriter(const std::string &path) :
        out(path.c_str(), std::ios::app)
    {
        if(!out.is_open()) {
            throw std::ios_base::failure("Couldn't open path '" + path + "'!");
        }
    }
        
    void write(const double &measured,
               const double &desired,
               const double &error)
    {
        out << measured << " " << desired << " " << error << std::endl;
    }

    void close()
    {
        out.close();
    }
};
}

#endif // WRITE_PLOT_DATA_HPP
