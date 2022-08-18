//
// Created by lfc on 19-1-20.
//

#ifndef SROS_RACK_OPERATOR_INSTANCE_HPP
#define SROS_RACK_OPERATOR_INSTANCE_HPP

#include <memory>
#include <boost/thread.hpp>
#include <glog/logging.h>
#include "rack_para.hpp"

namespace rack{
class RackOperatorInstance {
public:
    static std::shared_ptr<RackOperatorInstance> getInstance();

    virtual ~RackOperatorInstance(){

    }

    const rack::RackInfo_Ptr &getRackInfo() {
        return rack_info;
    }

    void updateRackInfo(rack::RackInfo_Ptr& info) {
        LOG(INFO) << "have update Rack Info!";
        rack_info = info;
    }
private:
    RackOperatorInstance() {
//        rack_info.reset(new rack::RackInfo);
    }
    static std::shared_ptr<RackOperatorInstance> rack_operator;
    static boost::mutex shared_mutex;

    rack::RackInfo_Ptr rack_info;
};

}


#endif //SROS_RACK_OPERATOR_INSTANCE_HPP
