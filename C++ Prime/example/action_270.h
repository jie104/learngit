//
// Created by zxj on 2023/1/4.
//

#ifndef EXAMPLE_ACTION_270_H
#define EXAMPLE_ACTION_270_H
//
// Created by zxj on 1/3/23.
//

#ifndef SROS_ACTION_270_H
#define SROS_ACTION_270_H

#include "base_action.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


using namespace std;
using namespace sros::core;

namespace ac {

    class Action270 : public BaseAction {
    public:
        Action270(): BaseAction(ACTION_ID_OBASTACLE_PCL_CALIB) {}
        virtual ~Action270() {}

        void doStart() override;
        void doCancel() override;


    private:
        Eigen::Isometry2f CoordinateTf(const Eigen::Vector3f& src_point);

    }
}

#endif  // SROS_ACTION_206_H

#endif //EXAMPLE_ACTION_270_H
