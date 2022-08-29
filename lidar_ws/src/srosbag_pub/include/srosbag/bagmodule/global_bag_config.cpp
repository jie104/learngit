//
// Created by lfc on 16-8-6.
//

#include "global_bag_config.h"

namespace bag {
GlobalBagConfig::GlobalBagConfig() : bag_pause(false), bag_play(false), bag_record(false), time_scale(1.0f) {


}

GlobalBagConfig::~GlobalBagConfig() {

}

GlobalBagConfig globalbagconfig;

void GlobalBagConfig::sethandleStartRecordCallback(CommandCallbackFunc func) {
    handleStartRecord = func;

}

void GlobalBagConfig::sethandleStopRecordCallback(CommandCallbackFunc func) {
    handleStopRecord = func;
}

void GlobalBagConfig::sethandleCancelBagCallback(CommandCallbackFunc func) {
    handleCancelBag = func;
}

void GlobalBagConfig::sethandleStartPlayCallback(CommandCallbackFunc func) {
    handleStartPlay = func;
}

void GlobalBagConfig::sethandleStopPlayCallback(CommandCallbackFunc func) {
    handleStopPlay = func;
}

void GlobalBagConfig::sethandlePausePlayCallback(CommandCallbackFunc func) {
    handlePausePlay = func;
}

void GlobalBagConfig::sethandleContinuePlayCallback(CommandCallbackFunc func) {
    handleContinuePlay = func;
}

void GlobalBagConfig::setbagscanCallback(msgCallbackFunc func) {
    bagscanCallback = func;

}

void GlobalBagConfig::setposestampedCallback(msgCallbackFunc func) {
    posestampedCallback = func;
}
}
