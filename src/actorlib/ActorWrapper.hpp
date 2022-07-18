#pragma once

#include "ActorImpl.hpp"
#include <typeinfo>
#include <variant>

/*
    ActorWrapper<A,As...> allows the Actors of type A,As... to be migrated by DynamicActorGraph

*/

template <typename Act> constexpr bool checkActor() { return std::is_base_of<ActorImpl, Act>::value; }

template <typename Act1, class Act2, class... Acts>
constexpr bool checkActor() // check whetherAct,Acts... are derived from ActorImpl
{
    return std::is_base_of<ActorImpl, Act1>::value && checkActor<Act2, Acts...>();
}

template <int N, class I> constexpr bool isSameType() { return false; } // class I is not an argument of Arg,Args...

template <int N, class I, class A> constexpr bool isSameType() // class return true if I<=>A
{
    if (N >= 0)
    {
        return std::is_same<I, A>::value;
    }
    else
    {
        return false;
    }
}

template <int N, class I, class A, class B, class... As>
constexpr bool isSameType() // class return true if I<=>A or B or As...
{
    if (N == 0)
    {
        return std::is_same<I, A>::value;
    }
    else if (N == 1)
    {
        return std::is_same<I, B>::value;
    }
    else
    {
        return isSameType<(N - 2), I, As...>();
    }
}

template <typename C, class A> constexpr bool has()
{
    return std::is_same<C, A>::value;
} // return true if C<=>A, means AW can hold C

template <typename C, class A, class B, class... As>
constexpr bool has() // return true if C<=>A or B or As..., means AW can hold C
{
    if (std::is_same<C, A>::value)
    {
        return true;
    }
    else
    {
        return has<C, B, As...>();
    }
}

// N is the search limit
// we have iterated all elements but no class remains
template <int R, class I> constexpr int typeIndex() { return -1; }

// only one class to compare, if cannot hold retuns -1
template <int R, class I, class A> constexpr int typeIndex()
{
    if (isSameType<0, I, A>())
    {
        return R;
    }
    else
    {
        throw std::invalid_argument("Input type cant be hold by ActorWrapper");
    }
}

// check the type index of the class I in the Arguments List
template <int R, class I, class A, class B, class... As> constexpr int typeIndex()
{
    if (isSameType<0, I, A>())
    {
        return 0;
    }
    else if (isSameType<0, I, B>())
    {
        return 1;
    }
    else
    {
        return typeIndex<R + 2, I, As...>();
    }
}

template <typename Act, class... Acts> class ActorWrapper
{
  private:
    // serialized the std::variant<A,As..> but only the current class that is being hold within
    template <int I, class Writer, class X>
    static void serializeHelper(Writer &writer, ActorWrapper<Act, Acts...> const &object)
    {
        try
        {
            const X &x = std::get<X>(object.var);
            writer.write(I);
            // std::cout << "index of aw: " << I << std::endl;
            writer.write(x);
        }
        catch (...)
        {
            throw std::runtime_error("Cannot write anything?");
        }
    }

    // serialized the std::variant<A,As..> but only the current class that is being hold within
    template <int I, class Writer, class X, class Y, class... Xs>
    static void serializeHelper(Writer &writer, ActorWrapper<Act, Acts...> const &object)
    {
        try
        {
            const X &x = std::get<X>(object.var);
            writer.write(I);
            // std::cout << "index of aw: " << I << std::endl;
            writer.write(x);
        }
        catch (...)
        {
            serializeHelper<(I + 1), Writer, Y, Xs...>(writer, object);
        }
    }

    // reads the class X and creates an ActorWrapper
    template <typename Reader, class X>
    static ActorWrapper<Act, Acts...> *deserializeHelper(int ind, Reader &reader, void *storage)
    {
        if (ind == 0)
        {
            auto tt = new typename std::aligned_storage<sizeof(X), alignof(X)>::type;
            reader.template read_into<X>(tt);
            X *x = reinterpret_cast<X *>(tt);
            ActorWrapper<Act, Acts...> *ap = ::new ActorWrapper<Act, Acts...>(std::move(*x));
            delete x;
            return ap;
        }
        else
        {
            throw std::runtime_error("Cannot read anything?, Index wrong");
        }
    }

    // reads the class X and creates an ActorWrapper from possible classes
    template <typename Reader, class X, class Y, class... Xs>
    static ActorWrapper<Act, Acts...> *deserializeHelper(int ind, Reader &reader, void *storage)
    {
        if (ind == 0)
        {
            auto tt = new typename std::aligned_storage<sizeof(X), alignof(X)>::type;
            reader.template read_into<X>(tt);
            X *x = reinterpret_cast<X *>(tt);
            ActorWrapper<Act, Acts...> *ap = ::new ActorWrapper<Act, Acts...>(std::move(*x));
            delete x;
            return ap;
        }
        else
        {
            return deserializeHelper<Reader, Y, Xs...>(ind - 1, reader, storage);
        }
    }

  public:
    std::variant<Act, Acts...> var; // an std::variant that hols one of the declared actors

    // indexing from 1
    template <typename T> constexpr bool canHave()
    {
        return typeIndex<0, T, Act, Acts...>() >= 0;
    } // return true of AW can hold that Actor of type T

    template <typename T> constexpr int getInd()
    {
        return typeIndex<0, T, Act, Acts...>();
    } // return the type index of T if AW can hold that Actor of type T

    ActorWrapper() : var() {} // constructor of AW,  needed for Intel compiler of smth i forgot

    ActorWrapper(const ActorWrapper<Act, Acts...> &other) : var(other.var) {}

    ActorWrapper(ActorWrapper<Act, Acts...> &&other) : var(std::move(other.var)) {}

    ActorWrapper(std::variant<Act, Acts...> &&ivar) : var(std::move(ivar)) {}

    ActorWrapper(const std::variant<Act, Acts...> &ivar) : var(ivar) {}

    ActorWrapper &operator=(const ActorWrapper &other)
    {
        var = other.var;
        return *this;
    }

    ActorWrapper &operator=(ActorWrapper &&other)
    {
        var = std::move(other.var);
        return *this;
    }

    ActorWrapper &operator=(const std::variant<Act, Acts...> &other)
    {
        var = other;
        return *this;
    }

    ActorWrapper &operator=(std::variant<Act, Acts...> &&other)
    {
        var = std::move(other);
        return *this;
    }

    ~ActorWrapper(){};

    /*
        Serialize the Actor held within AW
    */

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, ActorWrapper<Act, Acts...> const &object)
        {
            serializeHelper<0, Writer, Act, Acts...>(writer, object);
            return;
        }

        template <typename Reader> static ActorWrapper<Act, Acts...> *deserialize(Reader &reader, void *storage)
        {
            int ind = reader.template read<int>();
            return deserializeHelper<Reader, Act, Acts...>(ind, reader, storage);
        }
    };
};