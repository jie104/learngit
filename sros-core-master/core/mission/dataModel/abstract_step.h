//
// Created by huangwuxian on 19-3-1.
//

#ifndef SROS_ABSTRACT_STEP_H
#define SROS_ABSTRACT_STEP_H

#include <glog/logging.h>

#include "core/mission/constant.h"
#include "core/util/json.h"

namespace sros {
namespace core {

class AbstractStep {
public:
    explicit AbstractStep(MissionStepType eType);
    virtual ~AbstractStep();

    inline std::string getStepId() const { return step_id_; }
    inline MissionStepType getStepType() const { return step_type_; }

    // 从json中解析步骤数据
    bool fromJson(const nlohmann::json &dat);

    // 解析步骤信息（不包括连接）
    virtual bool parseStepInfo (const nlohmann::json &step_info);

    virtual std::string toString() const;

protected:
    ExpressionType getExpressionType(const std::string &expression);

private:
    nlohmann::json originalDat_; // 保存原始数据
    std::string step_id_;
    MissionStepType step_type_;

    std::vector<std::string> source_conn_ids_;
    std::vector<std::string> target_conn_ids_;
};
typedef std::shared_ptr<AbstractStep> AbstractStepPtr;
}
}

#endif //SROS_ABSTRACT_STEP_H
