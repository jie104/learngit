//
// Created by lfc on 17-7-7.
//

#ifndef SROS_LOC_PROCESSOR_FACTORY_H
#define SROS_LOC_PROCESSOR_FACTORY_H
#include "../location_processor.h"
namespace location{
class LocProcessorFactory {
public:
    LocProcessorFactory(){

    }

    virtual ~LocProcessorFactory(){

    }

    static LocationProcessor_Ptr creatLocProcessor(LOC_PROCESSOR_TYPE type);


};
}



#endif //SROS_LOC_PROCESSOR_FACTORY_H
