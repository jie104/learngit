/**
 * @file factory.cpp
 * @author zmy (626670628@qq.com)
 * @brief 工厂类的接口文件
 * @version 0.1
 * @date 2021-05-17
 * 
 * 
 */

#include "factory/my_factory.h"
#include <glog/logging.h>
#include <memory>

namespace bag
{
    namespace factory
    {

        bool Factory::registerClass(const std::string &name, const size_t class_id, const bool is_compress)
        {
            if (name2Id_.find(name) != name2Id_.end())
            {
                LOG(WARNING) << "The class name has multiple registration!!";
                return false;
            }
            name2Id_[name] = class_id;
            name_is_compress_[name] = is_compress;
            return true;
        }

        void Factory::unregisterClass(const std::string &name)
        {
            auto it = name2Id_.find(name);
            if (it != name2Id_.end())
            {
                name2Id_.erase(it);
                name_is_compress_.erase(name);
            }
        }

        size_t Factory::getClassID(const std::string &name)
        {
            auto it = name2Id_.find(name);
            if (it != name2Id_.end())
                return name2Id_[name];
            return -1;
        }
        void Factory::printInfo()
        {
            LOG(INFO) << "Msg size:" << name2Id_.size();
            for (const auto &p : name2Id_)
            {
                LOG(INFO) << p.first << "--" << p.second;
            }
        }

        bool Factory::isCompress(const std::string &name)
        {
            auto it = name_is_compress_.find(name);
            if (it != name_is_compress_.end())
                return name_is_compress_[name];
            LOG(WARNING) << "query a unregigstration topic " << name;
            return true;
        }

        std::shared_ptr<Factory> getFactory()
        {
            static std::shared_ptr<Factory> instance = std::make_shared<Factory>();
            return instance;
        }
    }
}