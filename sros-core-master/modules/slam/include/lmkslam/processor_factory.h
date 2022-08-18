//
// Created by lfc on 17-7-8.
//

#ifndef PROJECT_PROCESSOR_FACTORY_H
#define PROJECT_PROCESSOR_FACTORY_H
#include <memory>
namespace slam{
class FeatureExtractProcessor;
class LoopDectectProcessor;
class MapProcessor;
class MatchProcessor;
class PoseProcessor;
}
namespace slam{
class ProcessorFactory {
public:
    ProcessorFactory(){

    }

    virtual ~ProcessorFactory(){

    }

    static std::shared_ptr<FeatureExtractProcessor> getFeatureExtractProcessor(std::string type_str);
    static std::shared_ptr<LoopDectectProcessor> getLoopDetectProcessor(std::string type_str);
    static std::shared_ptr<MapProcessor> getMapProcessor(std::string type_str);
    static std::shared_ptr<MatchProcessor> getMatchProcessor(std::string type_str);
    static std::shared_ptr<PoseProcessor> getPoseProcessor(std::string type_str);

};
}



#endif //PROJECT_PROCESSOR_FACTORY_H
