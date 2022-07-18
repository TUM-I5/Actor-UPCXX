#pragma once
#include <iostream>
#include <optional>
#include <string>
#include <upcxx/upcxx.hpp>
/*

Intel Compiler has problems with serializing std::optional
therefore I provide user defined serialization methods.
Double und size_t to decrease compile time

*/

static_assert(upcxx::is_serializable<char>::value || upcxx::is_trivially_serializable<char>::value);
static_assert(upcxx::is_serializable<std::iterator_traits<char>>::value ||
              upcxx::is_trivially_serializable<std::iterator_traits<char>>::value);
static_assert(upcxx::is_serializable<std::allocator<char>>::value ||
              upcxx::is_trivially_serializable<std::allocator<char>>::value);

template <class T> struct upcxx::serialization<std::optional<T>>
{
    template <typename Writer> static void serialize(Writer &writer, const std::optional<T> &object)
    {
        if (object.has_value())
        {
            writer.write(true);
            writer.write(*object);
        }
        else
        {
            writer.write(false);
        }
    }

    template <typename Reader> static std::optional<T> *deserialize(Reader &reader, void *storage)
    {
        bool has = reader.template read<bool>();
        if (has)
        {
            T val = reader.template read<T>();
            std::optional<T> *rt = ::new (storage) std::optional<T>(val);
            return rt;
        }
        else
        {
            std::optional<T> *rt = ::new (storage) std::optional<T>();
            return rt;
        }
    }
};
