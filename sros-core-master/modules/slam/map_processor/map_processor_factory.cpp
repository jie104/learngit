//
// Created by lfc on 17-7-12.
//

#include "map_processor_factory.h"
#include "pgv_map_processor.h"
#include "lmk_map_processor.h"
#include "slam_map_processor.h"
namespace mapping{
MappingProcessor_Ptr MapProcessorFactory::creatMapProcessor(MAP_PROCESSOR_TYPE type) {
    MappingProcessor_Ptr processor;
    switch (type) {
        case MAP_PROCESSOR_TYPE::PGV_MAP_TYPE: {
            processor = std::make_shared<PgvMapProcessor>();
            break;
        }
        case MAP_PROCESSOR_TYPE::LMK_MAP_TYPE: {
            processor = std::make_shared<LmkMapProcessor>();
            break;
        }
        case MAP_PROCESSOR_TYPE::SLAM_MAP_TYPE: {
            processor = std::make_shared<SlamMapProcessor>();
            break;
        }
        default:
            LOG(INFO) << "cannot resolve the type! will return default processor!" << type;
            break;
    }
    return processor;
}
}

