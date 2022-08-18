//
// Created by lfc on 17-9-25.
//

#ifndef PROJECT_FITTING_PROCESSOR_HPP
#define PROJECT_FITTING_PROCESSOR_HPP

#include <memory>
#include "../base_recognition.hpp"
namespace fitting{
enum FitProcType {
    TYPE_FITPROC_CIRCLE = 1,
    TYPE_FITPROC_LINE = 2,
};
struct FeatPara{
    float a;
    float b;
    float r;
    float rmse;
    Eigen::Matrix2f cov = Eigen::Matrix2f::Identity();
};
typedef std::shared_ptr<FeatPara> FeatPara_Ptr;
class FittingProcessor {
public:
    FittingProcessor(FitProcType type_):type( type_){

    }

    virtual ~FittingProcessor(){

    }

    virtual void computeFeatPara(recog::FeatureCluster& cluster,FeatPara_Ptr feat_para){

    }

    FitProcType getType() {
        return type;
    }
private:
    FitProcType type;

};

typedef std::shared_ptr<FittingProcessor> FittingProcessor_Ptr;

}


#endif //PROJECT_FITTING_PROCESSOR_HPP
