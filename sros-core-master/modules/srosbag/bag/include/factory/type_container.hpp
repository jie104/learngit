/**
 * @file type_container.hpp
 * @author zmy (626670628@qq.com)
 * @brief 在编译期实现的用于存储类型的容器
 * @version 0.1
 * @date 2021-05-17
 * 
 * 
 */

#ifndef TYPE_CONTAINER_HPP
#define TYPE_CONTAINER_HPP

#include <cstddef>
#include <iostream>
#include <type_traits>
#include <boost/variant.hpp>

/****************************Type2Pos****************************************************/

template <typename FindT, size_t N, typename... Types>
struct Type2Pos_;
template <typename FindT, size_t N, typename CurT, typename... Types>
struct Type2Pos_<FindT, N, CurT, Types...>
{
    constexpr static size_t value = Type2Pos_<FindT, N + 1, Types...>::value;
};

template <typename FindT, size_t N, typename... Types>
struct Type2Pos_<FindT, N, FindT, Types...>
{
    constexpr static size_t value = N;
};

template <typename FindT, size_t N>
struct Type2Pos_<FindT, N>
{
    constexpr static size_t value = -1;
};

template <typename FindT, typename... Types>
constexpr size_t Type2Pos = Type2Pos_<FindT, 0, Types...>::value;

/****************************Pos2Type******************************************************/

template <typename T>
struct Identity_
{
    using type = T;
};

template <size_t Pos, typename... Types>
struct Pos2Type_
{
    static_assert((Pos != 0), "Invalid position.");
};

template <size_t Pos, typename CurT, typename... Types>
struct Pos2Type_<Pos, CurT, Types...>
{
    using type = typename std::conditional_t<(Pos == 0),
                                             Identity_<CurT>,
                                             Pos2Type_<Pos - 1, Types...>>::type;
};

// template <typename... Types>
// struct Pos2Type_<-1UL,Types...>
// {
//     using type = typename NULL;
// };

template <size_t Pos, typename... Types>
using Pos2Type = typename Pos2Type_<Pos, Types...>::type;

/*****************************TypeContainer********************************************************/

#define ERROR_TYPE(T) \
    std::cout << "The type " << #T << " is not set in processed_types.h!!!!" << std::endl;

template <typename... Types>
struct TypeContainer
{
    template <typename Type>
    static size_t typeToPos()
    {
        constexpr size_t type_pos = Type2Pos<Type, Types...>;
        if (type_pos == -1)
            ERROR_TYPE(Type);
        return type_pos;
    }

    template <size_t Pos>
    static auto createMsg() -> Pos2Type<Pos, Types...>
    {
        using type = Pos2Type<Pos, Types...>;
        return type{};
    }

    static auto createMsg(const size_t idx)
    {
        return _createMsg<0, Types...>(idx);
    }

    template <size_t Pos>
    using type = Pos2Type<Pos, Types...>;

    static constexpr size_t types_size = sizeof...(Types);

private:
    template <size_t n, typename... T>
    static constexpr boost::variant<T...> _createMsg(size_t i)
    {
        if (n >= sizeof...(T))
            throw std::out_of_range("越界");
        if (i == n)
            return boost::variant<T...>{Pos2Type<n, Types...>{}};
        return _createMsg<(n < sizeof...(T) - 1 ? n + 1 : 0), T...>(i);
    }
};

#define HANDLE_TYPES(types...) \
    using Container = TypeContainer<types>;

#endif