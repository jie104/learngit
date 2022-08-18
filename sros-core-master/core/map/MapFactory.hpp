
/**
 * @file MapFactory.hpp
 *
 * @author lhx
 * @date 2015年9月22日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef MAPFACTORY_H_
#define MAPFACTORY_H_

#include "DynamicBlockGrayMap.hpp"
#include "DynamicGrayMap.hpp"
#include "GrayMap.hpp"
#include "NavigationMap.hpp"
#include "StaticGrayMap.hpp"

namespace sros {
namespace map {

class MapFactory {
 public:
    MapFactory() = default;

    virtual ~MapFactory() = default;

    static GrayMap_ptr getStaticMap() { return std::make_shared<StaticGrayMap>(); }

    static GrayMap_ptr getDynamicMap() { return std::make_shared<DynamicGrayMap>(); }

    static GrayMap_ptr getDynamicBlockMap() { return std::make_shared<DynamicBlockGrayMap>(); }

    static GrayMap_ptr convertToGrayMap(NavigationMap_ptr navigation_map) { return std::shared_ptr<GrayMap>(); }

    static NavigationMap_ptr getNavigationMap() { return std::shared_ptr<NavigationMap>(); }

    static NavigationMap_ptr convertToNavigationMap(GrayMap_ptr gray_map) { return std::shared_ptr<NavigationMap>(); }

    static StaticGrayMap *getStaticMap_test() { return (new StaticGrayMap()); }

    static DynamicGrayMap *getDynamicMap_test() { return (new DynamicGrayMap()); }
};

}  // namespace map
} /* namespace sros */
#endif /* MAPFACTORY_H_ */
