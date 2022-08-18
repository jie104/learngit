//
// Created by lfc on 19-1-20.
//

#include "rack_operator_instance.hpp"

namespace rack{
std::shared_ptr<RackOperatorInstance> RackOperatorInstance::rack_operator;
boost::mutex RackOperatorInstance::shared_mutex;

std::shared_ptr<RackOperatorInstance> RackOperatorInstance::getInstance() {
    if (!rack_operator) {
        boost::mutex::scoped_lock scoped_lock(shared_mutex);
        if (!rack_operator) {
            rack_operator.reset(new RackOperatorInstance());
        }
    }
    return rack_operator;
}


}