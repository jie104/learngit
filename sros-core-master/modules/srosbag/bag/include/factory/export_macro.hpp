/**
 * @file export_macro.hpp
 * @author zmy (626670628@qq.com)
 * @brief 将待处理的类型注册到factory的宏定义
 * @version 0.1
 * @date 2021-05-17
 * 
 * 
 */

#ifndef EXPORT_MACRO_HPP
#define EXPORT_MACRO_HPP

#include "my_factory.h"

namespace bag
{

#define EXPORT_MSG_TWO(Name, Class, UniqueID)                                \
    namespace                                                                \
    {                                                                        \
        struct AnonymousMsg##UniqueID                                        \
        {                                                                    \
            AnonymousMsg##UniqueID()                                         \
            {                                                                \
                auto factory = bag::factory::getFactory();                   \
                factory->registerClass(Name, Container::typeToPos<Class>()); \
            }                                                                \
        };                                                                   \
                                                                             \
        AnonymousMsg##UniqueID g_anonymous_msg_##UniqueID;                   \
                                                                             \
    } // namespace

#define EXPORT_MSG_THREE(Name, Class, IsCompress, UniqueID)                              \
    namespace                                                                            \
    {                                                                                    \
        struct AnonymousMsg##UniqueID                                                    \
        {                                                                                \
            AnonymousMsg##UniqueID()                                                     \
            {                                                                            \
                auto factory = bag::factory::getFactory();                               \
                factory->registerClass(Name, Container::typeToPos<Class>(), IsCompress); \
            }                                                                            \
        };                                                                               \
                                                                                         \
        AnonymousMsg##UniqueID g_anonymous_msg_##UniqueID;                               \
                                                                                         \
    } // namespace

#define EXPORT_MSG_INTERNAL_TWO(Name, Class, UniqueID) \
    EXPORT_MSG_TWO(Name, Class, UniqueID)

#define EXPORT_MSG_INTERNAL_THREE(Name, Class, IsCompress, UniqueID) \
    EXPORT_MSG_THREE(Name, Class, IsCompress, UniqueID)

#define GET_EXPROT_MACRO(_1, _2, _3, EXPORT_MSG_INTERNAL, ...) EXPORT_MSG_INTERNAL

#define EXPORT_MSG(...)                                                                    \
    GET_EXPROT_MACRO(__VA_ARGS__, EXPORT_MSG_INTERNAL_THREE, EXPORT_MSG_INTERNAL_TWO, ...) \
    (__VA_ARGS__, __COUNTER__)

} // namespace bag

#endif
