//
// Created by lfc on 17-7-7.
//

#ifndef SROS_PGV_PROCESSOR_HPP
#define SROS_PGV_PROCESSOR_HPP

#include "../location_processor.h"
namespace location{
class PgvProcessor : public LocationProcessor {
public:
    PgvProcessor():LocationProcessor(PGV_LOC_TYPE){

    }

    virtual ~PgvProcessor(){

    }
private:

};
}



#endif //SROS_PGV_PROCESSOR_HPP
