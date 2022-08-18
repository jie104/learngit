/**
 * @file json_test
 *
 * @author pengjiali
 * @date 20-4-28.
 *
 * @describe
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <exception>
#include <iostream>
#include <regex>
#include <stdexcept>
#include "core/util/json.h"

using namespace std;
using namespace nlohmann;
using namespace boost::filesystem;

#define GET_VALUE(p, key, cast_type) getValue<cast_type>(p, key, __FILE__, __LINE__)
template <class CastType>
CastType getValue(const json &p, std::string key, const char *file, int line) {
    try {
        return p[key].get<CastType>();
    } catch (std::exception &e) {
        auto str = std::string(e.what()) + std::string("  key:") + key + ", " + path(file).filename().c_str() + ":" +
                   std::to_string(line);
        throw std::logic_error(str);
    }
}

TEST(json, json) {
    json j = {{"pi", 3.141},
              {"happy", true},
              {"name", "Niels"},
              {"nothing", nullptr},
              {"answer", {{"everything", 42}}},
              {"list", {1, 0, 2}},
              {"object", {{"currency", "USD"}, {"value", 42.99}}}};

    //    try {
    const auto pi = j["pii"];
    std::cout << "pii size: " << pi.size() << std::endl;
    //    } catch (json::type_error& e)
    //    {
    //        // output exception information
    //        std::cout << "message: " << e.what() << '\n'
    //                  << "exception id: " << e.id << std::endl;
    //    }

    try {
        const auto &pii = GET_VALUE(j, "pii", double);
    } catch (const std::exception &e) {
        cout << e.what() << endl;
    }
}