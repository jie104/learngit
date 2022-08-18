//
// Created by lfc on 18-12-10.
//

#include "curve_sample_factory.h"
#include "line_curve_sample.hpp"
#include "circle_curve_sample.hpp"
#include "rotate_curve_sample.hpp"
#include "bezier_curve_sample.hpp"
namespace sample{
BaseCurveSample_Ptr CurveSampleFactory::getCurveSample(const sros::core::NavigationPath<double> &path,double step,int direction) {
    BaseCurveSample_Ptr base_sample;
    switch (path.type_) {
        case sros::core::PATH_LINE:
            base_sample.reset(new sample::LineCurveSample(path, step,direction));
            break;
        case sros::core::PATH_ARC:
            base_sample.reset(new sample::CircleCurveSample(path, step, direction));
            break;
        case sros::core::PATH_BEZIER:
            base_sample.reset(new sample::BezierCurveSample(path, step,direction));
            break;
        case sros::core::PATH_ROTATE:
            base_sample.reset(new sample::RotateCurveSample(path, step,direction));
            break;
        default:
            LOG(INFO) << "err toget type!" << path.type_;
    }
    return base_sample;
}

}


