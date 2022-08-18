//
// Created by lfc on 18-12-10.
//

#ifndef PROJECT_CURVE_SAMPLE_FACTORY_H
#define PROJECT_CURVE_SAMPLE_FACTORY_H

#include "base_curve_sample.hpp"

namespace sample{
class CurveSampleFactory {
public:
    static BaseCurveSample_Ptr getCurveSample(const sros::core::NavigationPath<double>& path,double step,int direction);

};
}



#endif //PROJECT_CURVE_SAMPLE_FACTORY_H
