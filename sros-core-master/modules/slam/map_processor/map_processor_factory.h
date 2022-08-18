//
// Created by lfc on 17-7-12.
//

#ifndef SROS_MAP_PROCESSOR_FACTORY_H
#define SROS_MAP_PROCESSOR_FACTORY_H
#include "../mapping_processor.h"

namespace mapping{
class MapProcessorFactory {
public:

    MapProcessorFactory(){

    }
    virtual ~MapProcessorFactory(){

    }

    static MappingProcessor_Ptr creatMapProcessor(MAP_PROCESSOR_TYPE type);
};
}



#endif //SROS_MAP_PROCESSOR_FACTORY_H
