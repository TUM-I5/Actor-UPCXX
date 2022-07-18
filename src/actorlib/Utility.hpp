#pragma once
#include <chrono>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <upcxx/upcxx.hpp>
#include <utility>

class InPortInformationBase;
class OutPortInformationBase;

/*
    Utility functions and additional definitions to help calls in library
*/

enum TaskType
{
    Act,
    Flush,
    Notify,
    Unspecified
};

class ActorImpl;
using GlobalActorRef = upcxx::global_ptr<ActorImpl *>;
using GlobalChannelRef = upcxx::global_ptr<void>;

template <typename T>
using expr_type = std::remove_cv_t<std::remove_reference_t<T>>; // used to get pure type info of an Actor

// std::cout << for std::optionals
template <class T> constexpr std::ostream &operator<<(std::ostream &os, std::optional<T> const &opt)
{
    if (opt.has_value())
    {
        os << *opt;
    }
    else
    {
        os << "{}";
    }
    return os;
}

namespace util
{

constexpr int too_long = 1000000;

// checks for overflow, complains if there is an overflow
template <class T> T addNoOverflow(const T a, const T b)
{
    if (a > std::numeric_limits<T>::max() - b)
    {
        throw std::runtime_error("Overflow");
    }
    return a + b;
}

// return true if it overflows
template <class T> bool addOverflows(const T a, const T b)
{
    if (a <= std::numeric_limits<T>::max() - b)
    {
        return false;
    }
    else
    {
        return true;
    }
}

// prints std::optional<T>
template <typename T> std::string optional2Str(const std::optional<T> &opt)
{
    if (opt.has_value())
    {
        return std::to_string(*opt);
    }
    else
    {
        return "None";
    }
}

// adds a and b with overflow
std::pair<bool, uint64_t> addWOverflow(uint64_t a, uint64_t b);

// converts saved nanoseconds to seconds
double nanoToSec(uint64_t nano);

// combines the vector of futures and wait for all of them
upcxx::future<> combineFutures(const std::vector<upcxx::future<>> &futs);
upcxx::future<> combineFutures(std::vector<upcxx::future<>> &&futs);

// prints a connection document
void printConnectionTuple(const std::tuple<std::string, std::string, std::string, std::string> &tup);

// print the connection tuple but rvalue
void printConnectionTuple(std::tuple<std::string, std::string, std::string, std::string> &&tup);

// makes a string of migratio list (needs rank too)
std::string migrationList2Str(int rank, const std::unordered_map<std::string, upcxx::intrank_t> &migList);

// creates a string of vector of pairs
template <typename A, typename B> std::string vector2Str(const std::vector<std::pair<A, B>> &vec)
{
    std::stringstream ss;
    ss << "{ ";
    for (auto &el : vec)
    {
        ss << "(" << el.first << ", " << el.second << ")";
    }
    ss << "}";

    return ss.str();
}

// creates a string of type T
template <typename T> std::string vector2Str(const std::vector<T> &vec)
{
    std::stringstream ss;
    ss << "{ ";
    for (auto &el : vec)
    {
        ss << el << " ";
    }
    ss << "}";

    return ss.str();
}

// creates a string of type map T,U
template <typename T, typename U> std::string map2Str(const std::unordered_map<T, U> &map)
{
    {
        std::stringstream ss;
        ss << "{";
        for (auto &el : map)
        {
            ss << "(" << el.first << " -> " << el.second << ")";
        }
        ss << "}";

        return ss.str();
    }
}

// creates a string of type map T,U
template <typename T, typename U> std::string map2Ptr2Str(const std::unordered_map<T, U> &map)
{
    {
        std::stringstream ss;
        ss << "{";
        for (auto &el : map)
        {
            ss << "(" << el.first << " -> " << *el.second.get() << ")";
        }
        ss << "}";

        return ss.str();
    }
}

// compile time variadic template lest length function
template <class I> constexpr size_t counter() { return 1; }

template <class I, class J, class... Is> constexpr size_t counter()
{
    return 1 + counter<J, Is...>();
} // compile time variadic template lest length function

template <class I, class J, class... Is> constexpr bool multiple()
{
    return true;
} // compile time variadic template lest length >=1 function

template <class I> constexpr bool multiple()
{
    return false;
} // compile time variadic template lest length >= 1 function

template <class Reader, class TypeToRead> upcxx::deserialized_type_t<TypeToRead> *read(Reader &reader)
{
    auto storage = new typename std::aligned_storage<sizeof(upcxx::deserialized_type_t<TypeToRead>),
                                                     alignof(upcxx::deserialized_type_t<TypeToRead>)>::type;

    reader.template read_into<TypeToRead>(storage);
    upcxx::deserialized_type_t<TypeToRead> *ad = reinterpret_cast<upcxx::deserialized_type_t<TypeToRead> *>(storage);

    static_assert(sizeof(typename std::aligned_storage<sizeof(upcxx::deserialized_type_t<TypeToRead>),
                                                       alignof(upcxx::deserialized_type_t<TypeToRead>)>::type) ==
                  sizeof(upcxx::deserialized_type_t<TypeToRead>));
    return ad;
}

template <class Reader, class TypeToRead>
std::unique_ptr<upcxx::deserialized_type_t<TypeToRead>> read_unique(Reader &reader)
{
    auto storage = new typename std::aligned_storage<sizeof(upcxx::deserialized_type_t<TypeToRead>),
                                                     alignof(upcxx::deserialized_type_t<TypeToRead>)>::type;

    reader.template read_into<TypeToRead>(storage);
    upcxx::deserialized_type_t<TypeToRead> *ad = reinterpret_cast<upcxx::deserialized_type_t<TypeToRead> *>(storage);

    static_assert(sizeof(typename std::aligned_storage<sizeof(upcxx::deserialized_type_t<TypeToRead>),
                                                       alignof(upcxx::deserialized_type_t<TypeToRead>)>::type) ==
                  sizeof(upcxx::deserialized_type_t<TypeToRead>));

    std::unique_ptr<upcxx::deserialized_type_t<TypeToRead>> uptr(ad);
    return uptr;
}

int64_t intPow(int x, unsigned int p);

upcxx::intrank_t rank_n();

double percentage(unsigned int part, unsigned int total);

template <typename F, typename Tuple, std::size_t... I> auto apply_tup_impl(F &&f, Tuple &&t, std::index_sequence<I...>)
{
    return std::forward<F>(f)(std::get<I>(std::forward<Tuple>(t))...);
}
template <typename F, typename Tuple> auto apply_tup(F &&f, Tuple &&t)
{
    using Indices = std::make_index_sequence<std::tuple_size<std::decay_t<Tuple>>::value>;
    return apply_tup_impl(std::forward<F>(f), std::forward<Tuple>(t), Indices());
}

template <size_t I, typename S> constexpr std::optional<size_t> typeIndex()
{
    return {};
} // return {} because it is not in it

template <size_t I, typename S, typename Act, typename... Acts>
constexpr std::optional<size_t> typeIndex() // return the offset of the element
{
    if constexpr (std::is_same<expr_type<S>, expr_type<Act>>::value)
    {
        return I;
    }
    // else
    {
        return typeIndex<(I + 1), S, Acts...>();
    }
}

std::chrono::_V2::steady_clock::time_point timepoint();

double timerDifference(std::chrono::_V2::steady_clock::time_point &beg);

template <typename T> T runWithTimerRet(std::function<T()> &foo, double &collector)
{
    auto t1 = util::timepoint();
    T rt = foo();
    double time = timerDifference(t1);
    collector += time;
    return rt;
}

void runWithTimer(std::function<void()> &foo, double &collector);

void parseBooleanEnvVar(const std::string &key, bool &val);

void parseBooleanEnvVar(const char *key, bool &val);

void parseIntEnvVar(const char *key, int &val);

} // namespace util
