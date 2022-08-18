//
// Created by caoyan on 5/28/21.
//

#include "action_255.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action255::doStart() {

    sendEacActionTask();
}

}