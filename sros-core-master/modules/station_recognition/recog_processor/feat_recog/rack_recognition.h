//
// Created by lfc on 17-9-25.
//

#ifndef PROJECT_RACK_RECOGNITION_H
#define PROJECT_RACK_RECOGNITION_H

#include "base_recognition.hpp"
#include "fitting/fitting_processor.hpp"
#include "gicp.h"
namespace recog{



class RackRecognition: public BaseRecognition {
public:
    RackRecognition();

    virtual ~RackRecognition();

    virtual bool extractFeature(Scan_Ptr scan,FeatureClusters_Ptr clusters);

    virtual bool getPose(FeatureClusters_Ptr clusters,std::vector<fitting::FeatPara_Ptr>& feat_paras,Eigen::Vector3f& out_pose);
private:

    fitting::FeatPara_Ptr getMinDistPara(std::vector<fitting::FeatPara_Ptr>& feat_paras);

    Eigen::Vector3f computePose(fitting::FeatPara_Ptr &this_para, fitting::FeatPara_Ptr &that_para);




    fitting::FittingProcessor_Ptr fitting_proc;
    icp::Gicp icp_processor;

};

}


#endif //PROJECT_RACK_RECOGNITION_H
