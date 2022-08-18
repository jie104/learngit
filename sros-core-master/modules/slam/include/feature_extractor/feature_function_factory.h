//
// Created by getup on 18-1-10.
//

#ifndef SROS_FEATURE_FUNCTION_FACTORY_H
#define SROS_FEATURE_FUNCTION_FACTORY_H


namespace slam {
class FeatureFunctionFactory {
public:
    static float getScanDirection(slam::ScanMsg_Ptr scan); //根据scan获得该帧的朝向
};
}

#endif //SROS_FEATURE_FUNCTION_FACTORY_H
