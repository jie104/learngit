/**
 * @file distribution_plot
 *
 * @author pengjiali
 * @date 2020/9/11.
 *
 * @describe
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#include "distribution_plot.h"
#include <iomanip>
#include <cmath>
#include <math.h>
#include <stdlib.h>

#include "time.h"

DistributionPlot::DistributionPlot(const std::string& name, int distribution_segment_width)
    : distribution_segment_width_(distribution_segment_width), name_(name) {}

void DistributionPlot::shooting(int value) {
    distribution_map_[value / distribution_segment_width_] += 1;
    ++current_shoot_count_;
}

void calcSdValue(const DistributionPlot& plot, double & average, double& sd_value) {

    double sum = 0;
    int count = 0;
    double idex = 1.0;
    for(auto iter = plot.distribution_map_.begin(); iter != plot.distribution_map_.end(); iter++) {
        double temp = idex ;
        temp = temp * iter->second;
        sum += temp;
        count += iter->second;
        idex += 1;
    }

    average = sum / count;

    sd_value = 0.0;
    idex = 1.0;
    for(auto iter = plot.distribution_map_.begin(); iter != plot.distribution_map_.end(); iter++) {
        double temp = idex;
        temp = fabs(temp - average);
        double pj_value = pow(temp, 2);
        pj_value = pj_value * iter->second;
        sd_value += pj_value;
        idex += 1;
    }
    sd_value = sd_value / count;

    sd_value = sqrt(sd_value);

    average *= plot.distribution_segment_width_;
}

std::ostream& operator<<(std::ostream& out, const DistributionPlot& plot) {
    double average; double sd_value;
    calcSdValue(plot, average, sd_value);
    out << plot.name_ << "：\ntotal count: " << plot.current_shoot_count_ << ",   segment width: " << plot.distribution_segment_width_
        << ",   average: " << average  << ",   sd_value: " << sd_value  << "\n";

    auto printLineFunc = [](std::ostream& out, int key, int value, int total_count, int width, char chflag = ' ') {
        auto percentage = value * 100 / total_count;
        out << chflag << std::setfill(' ') << std::setw(10) << key * width << " ~ " << std::setfill(' ') << std::setw(10)
            << (key + 1) * width - 1 << "   [" << std::setfill(' ') << std::setw(2) << percentage << "%] |";
        for (auto i = 0; i < percentage; ++i) {
            out << "█";
        }
        out << " " << value << "\n";
    };

    auto last_key = -1;
    int idex = 1;
    int line = round(average);
    for (auto it : plot.distribution_map_) {
        if (last_key != -1) {
            for (auto i = last_key + 1; i < it.first; ++i) {
                printLineFunc(out, i, 0, plot.current_shoot_count_, plot.distribution_segment_width_);
                if (i > last_key + 1 + 1) {
                    out << "......\n";
                    break;
                }
            }
        }

        if(idex == line) {
            printLineFunc(out, it.first, it.second, plot.current_shoot_count_, plot.distribution_segment_width_, '*');
        } else {
            printLineFunc(out, it.first, it.second, plot.current_shoot_count_, plot.distribution_segment_width_);
        }
        idex += 1;

        last_key = it.first;
    }
    return out;
}