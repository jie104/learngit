//
// Created by lfc on 17-7-7.
//

#include "loc_processor_factory.h"
#include "pgv_processor.hpp"
#include "lmk_loc_processor.h"
#include "slam_loc_processor.h"
namespace location{

LocationProcessor_Ptr LocProcessorFactory::creatLocProcessor(LOC_PROCESSOR_TYPE type) {
    LocationProcessor_Ptr processor;
    switch (type) {
        case LOC_PROCESSOR_TYPE::PGV_LOC_TYPE: {
            processor = std::make_shared<PgvProcessor>();
            break;
        }
        case LOC_PROCESSOR_TYPE::LMK_LOC_TYPE: {
            processor = std::make_shared<LmkLocProcessor>();
            break;
        }
        case LOC_PROCESSOR_TYPE::SLAM_LOC_TYPE: {
//            LOG(INFO) << "have not encode the slam loc type!";
            processor = std::make_shared<SlamLocProcessor>();
            break;
        }
        default:
            LOG(INFO) << "cannot resolve the type! will return default processor!" << type;
            break;
    }
    return processor;
}
}