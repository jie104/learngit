//
// Created by lfc on 17-10-16.
//

#include "recog_processor_factory.h"
#include "rack_lmk_processor.hpp"
#include "angle_recog_processor.hpp"
#include "charging_pile_recog_processor.hpp"

namespace recog{

BaseRecogProcessor_Ptr RecogProcessorFactory::getRecogProcessor(RecogProType type) {
    BaseRecogProcessor_Ptr recog_processor;
    switch (type) {
        case TYPE_RECOGPRO_ANGLE:
            LOG(INFO) << "angle recog!";
            recog_processor.reset(new AngleRecogProcessor);
            break;
        case TYPE_RECOGPRO_LMKRACK:
            LOG(INFO) << "hack recog!";
            recog_processor.reset(new HackLmkProcessor);
            break;
        case TYPE_RECOGPRO_CHARGINGPILE:
            recog_processor.reset(new ChargingPileRecogProcessor);
    }
    return recog_processor;
}

    BaseRecogPara_Ptr RecogProcessorFactory::getRecogParam(RecogProType type) {
        BaseRecogPara_Ptr recog_param;
        switch (type) {
            case TYPE_RECOGPRO_ANGLE:
                LOG(INFO) << "angle recog!";
                recog_param.reset(new RecogPara);
                break;
            case TYPE_RECOGPRO_LMKRACK:
                LOG(INFO) << "hack recog!";
                recog_param.reset(new RecogRackPara);
                break;
            case TYPE_RECOGPRO_CHARGINGPILE:
                recog_param.reset(new RecogChargingPilePara);
        }
        return recog_param;
    }
}