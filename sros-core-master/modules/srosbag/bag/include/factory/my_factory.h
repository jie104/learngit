/**
 * @file my_factory.h
 * @author zmy (626670628@qq.com)
 * @brief 工厂类的接口文件
 * @version 0.1
 * @date 2021-05-17
 * 
 * 
 */

#ifndef MY_FACTORY_H
#define MY_FACTORY_H

// #pragma once

#include "processed_types.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <iostream>

namespace bag
{
    namespace factory
    {
        class Factory
        {

        public:
            Factory() = default;
            ~Factory() = default;

            bool registerClass(const std::string &name, const size_t class_id, const bool is_compress = true);
            void unregisterClass(const std::string &name);
            size_t getClassID(const std::string &name);
            bool isCompress(const std::string &name);
            int typeSize() { return Container::types_size; }
            void printInfo();

        private:
            Factory(const Factory &) = delete;
            Factory &operator=(const Factory &) = delete;

        private:
            std::unordered_map<std::string, size_t> name2Id_;
            std::unordered_map<std::string, bool> name_is_compress_;
        };

        std::shared_ptr<Factory> getFactory();

    } // namespace factory

} // namespace bag

#endif