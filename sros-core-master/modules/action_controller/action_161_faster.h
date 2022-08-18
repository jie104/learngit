#ifndef SROS_ACTION_161_FASTER_H
#define SROS_ACTION_161_FASTER_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//导航到取货站点前的抬臂动作
class Action161Faster : public BaseAction {
public:
  enum Step {
    WAITING,
    AC_RUNING,
    FINISH
  };
  Action161Faster(): BaseAction(ACTION_ID_LOAD_SEARCH_GOOGS) {}
  virtual ~Action161Faster() {}

  void doStart() override;

  void doCancel() override;

  void onSrcAcFinishSucceed(int result_value) override;
  void onSrcAcFinishFailed(int result_value) override;
  void onSrcAcFinishCanceled(int result_value) override;

  void waitActionStart();

 private:
  int height_;
  Step step_ = WAITING;
  double action_start_distance_;

  //站点位姿（货物中心点）
  Pose dst_pose_;
};


}


#endif  // SROS_ACTION_161_FASTER_H
