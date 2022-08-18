#ifndef SROS_PATH_MSG_HPP_
#define SROS_PATH_MSG_HPP_

#include "base_msg.h"

#include <memory>

#include "core/navigation_path.h"

namespace sros {
namespace core {

class PathMsg : public BaseMsg {
public:
    PathMsg(const topic_t &topic) : BaseMsg(topic, TYPE_PATH_DATA) {
        paths.clear();
    }

    virtual ~PathMsg() { }

    virtual void getTime() { };

    NavigationPath_vector paths;

};

typedef std::shared_ptr <PathMsg> PathMsg_ptr;

} /* core */
} /* sros */

#endif //SROS_PATH_MSG_HPP_
