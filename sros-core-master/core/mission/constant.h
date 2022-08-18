//
// Created by huangwuxian on 19-3-1.
//

#ifndef SROS_CONSTANT_H
#define SROS_CONSTANT_H

namespace sros {
namespace core {

const int TIMER_UNIT_200MS = 200;

enum class MissionStepType {
    StepNone = 0,
    StepBegin, // 开始
    StepEnd, // 结束
    StepMoveToStation, // 移动到站点
    StepAction, // 执行动作
    StepSetReg, // 设置寄存器的值
    StepWaitRegUpdate, // 等待寄存器更新
    StepDecision, // 判断条件
    StepWait, // 延时等待
    StepPause, // 暂停
    StepMission, // 子任务
    StepParallel, // 并行步骤
};

enum class ExpressionType {
    ExpLessThan = -1,
    ExpEqual,
    ExpLarger
};

enum class DecisionType {
    DecisionNa = 0,
    DecisionLoop,
    DecisionReg,
    DecisionLastStepResult,
};

}
}

#endif //SROS_CONSTANT_H
