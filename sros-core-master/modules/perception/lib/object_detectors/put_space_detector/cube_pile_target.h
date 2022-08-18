
#ifndef SRC_TARGET_H
#define SRC_TARGET_H

// INCLUDE
#include "../base/cluster.h"
#include "../base/target.hpp"

#include <vector>
#include <opencv/cv.h>

//CODE
namespace perception {

/**
 * @brief target of the card detect.
 */
class CubePileTarget : public Target {
public:

    /**
     * @brief constructor.
     */
    CubePileTarget();

    /**
     * @brief destructor.
     */
    ~CubePileTarget() override;

    /**
     * @brief  reset
     */
    void reset() override;

public:

    Cluster front_side_;

    Cluster top_side_;


};

using CubePileTarget = perception::CubePileTarget;

} // end of namespace perception
#endif //SRC_TARGET_H
