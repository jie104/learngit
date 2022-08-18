//
// Created by lfc on 17-10-16.
//

#ifndef SROS_RECOG_PROCESSOR_FACTORY_H
#define SROS_RECOG_PROCESSOR_FACTORY_H

#include "base_recog_processor.hpp"
namespace recog {
    class RecogProcessorFactory {
    public:
        static BaseRecogProcessor_Ptr getRecogProcessor(RecogProType type);

        static BaseRecogPara_Ptr getRecogParam(RecogProType type);

    };

}
#endif //SROS_RECOG_PROCESSOR_FACTORY_H
