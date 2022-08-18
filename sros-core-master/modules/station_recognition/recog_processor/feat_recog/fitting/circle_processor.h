//
// Created by lfc on 17-9-25.
//

#ifndef PROJECT_CIRCLE_PROCESSOR_H
#define PROJECT_CIRCLE_PROCESSOR_H

#include "fitting_processor.hpp"

namespace fitting{
class CircleProcessor: public FittingProcessor {
public:
    CircleProcessor();

    virtual ~CircleProcessor();

    virtual void computeFeatPara(recog::FeatureCluster& cluster,FeatPara_Ptr feat_para);
private:

    virtual void computeCov(recog::FeatureCluster& cluster,FeatPara_Ptr feat_para);

};

}


#endif //PROJECT_CIRCLE_PROCESSOR_H
