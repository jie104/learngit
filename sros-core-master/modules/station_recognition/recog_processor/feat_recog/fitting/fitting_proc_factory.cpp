//
// Created by lfc on 17-9-25.
//

#include <glog/logging.h>
#include "fitting_proc_factory.h"
#include "circle_processor.h"
#include "line_processor.h"

fitting::FittingProcessor_Ptr fitting::FittingProcFactory::getFittingProcessor(fitting::FitProcType type) {
    FittingProcessor_Ptr processor;
    switch (type) {
        case TYPE_FITPROC_CIRCLE: {
            processor.reset(new CircleProcessor);
            return processor;
        }
        case TYPE_FITPROC_LINE:{
            processor.reset(new LineProcessor);
            return processor;
        }
        default:
            LOG(INFO) << "err to get the processor";
            break;
    }
    return std::shared_ptr<FittingProcessor>();
}
