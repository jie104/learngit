//
// Created by lfc on 17-7-12.
//

#ifndef SROS_PGV_MAP_PROCESSOR_H
#define SROS_PGV_MAP_PROCESSOR_H

#include "../mapping_processor.h"

namespace mapping {
class PgvMapProcessor : public MappingProcessor {
public:
    PgvMapProcessor() : MappingProcessor(PGV_MAP_TYPE) {

    }

    virtual ~PgvMapProcessor(){

    }

};
}


#endif //SROS_PGV_MAP_PROCESSOR_H
