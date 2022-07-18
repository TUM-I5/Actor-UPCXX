/*
encapsulates information needed to encode/decode a channel on wire
*/
#pragma once

#include "PortInformationBase.hpp"
#include "SerializeOptional.hpp"
#include "Utility.hpp"
#include <array>
#include <memory>
#include <string>
#include <upcxx/upcxx.hpp>
#include <vector>

/*
    Classes to save the type of port informations

    These classes have the fields that define the state of an inport or an outport
    defined in the inport.hpp and outport.hpp headers

    these classes try to define cosntructors that can move elements as much as they can,
    and provide methods to serialzie and deserialzie them

    since user defines inport and outport types in implementation user needs to serialize
    them by hand but there are convenience writer and reader that does this in
    PortInfoConvey.hpp
*/

template <typename type, int capacity> class InPortInformation final : public InPortInformationBase
{
    static_assert(capacity > 0);
    static_assert(upcxx::is_serializable<type>::value || upcxx::is_trivially_serializable<type>::value);
    static_assert(std::is_trivially_copy_constructible<type>::value || std::is_copy_constructible<type>::value);
    static_assert(std::is_trivially_move_constructible<type>::value || std::is_move_constructible<type>::value);

  public:
    int lastEl;
    int firstEl;
    bool full;
    std::array<type, capacity> data;
    int to_notify;

    std::unique_ptr<InPortInformationBase> generateCopy() const override final
    {
        return std::make_unique<InPortInformation<type, capacity>>(name, lastEl, firstEl, full, data, to_notify);
    }

    InPortInformation() : InPortInformationBase("none"), lastEl(0), firstEl(0), full(false), data{}, to_notify(-1) {}

    InPortInformation(std::string &&name, int last, int first, bool full, std::array<type, capacity> &&inp,
                      int to_notify)
        : InPortInformationBase(std::move(name)), lastEl(last), firstEl(first), full(full), data(std::move(inp)),
          to_notify(to_notify)
    {
    }

    InPortInformation(const std::string &name, int last, int first, bool full, const std::array<type, capacity> &inp,
                      int to_notify)
        : InPortInformationBase(name), lastEl(last), firstEl(first), full(full), data(inp), to_notify(to_notify)
    {
    }

    InPortInformation(const std::string &name, int last, int first, bool full, std::array<type, capacity> &&inp,
                      int to_notify)
        : InPortInformationBase(name), lastEl(last), firstEl(first), full(full), data(std::move(inp)),
          to_notify(to_notify)
    {
    }

    InPortInformation(const InPortInformation<type, capacity> &ci) = delete;

    InPortInformation(InPortInformation<type, capacity> &&ci)
        : InPortInformationBase(std::move(ci.name)), lastEl(ci.lastEl), firstEl(ci.firstEl), full(ci.full),
          data(std::move(ci.data)), to_notify(ci.to_notify)
    {
    }

    size_t getMessageCount() const override final
    {
        int tmpLast = lastEl;
        if (lastEl < firstEl || full)
        {
            tmpLast += capacity;
        }
        return tmpLast - firstEl;
    }

    InPortInformation<type, capacity> &operator=(const InPortInformation<type, capacity> &ci) = delete;

    InPortInformation<type, capacity> &operator=(InPortInformation<type, capacity> &&other)
    {
        this->name = std::move(other.name);
        this->lastEl = other.lastEl;
        this->firstEl = other.firstEl;
        this->full = other.full;
        this->data = std::move(other.data);
        this->to_notify = other.to_notify;
        return *this;
    }

    ~InPortInformation() override final{};

    struct upcxx_serialization
    {
        template <typename Writer>
        static void serialize(Writer &writer, InPortInformation<type, capacity> const &object)
        {
            if (object.name.empty())
            {
                throw std::runtime_error("InPortInfo name should not be empty");
            }

            writer.write(object.name);
            writer.write(object.lastEl);
            writer.write(object.firstEl);
            writer.write(object.full);
            writer.write(object.to_notify);

            for (int i = 0; i < capacity; i++)
            {
                writer.write(object.data[i]);
            }

            return;
        }

        template <typename Reader> static InPortInformation<type, capacity> *deserialize(Reader &reader, void *storage)
        {
            std::string name = reader.template read<std::string>();
            int le = reader.template read<int>();
            int fe = reader.template read<int>();
            bool full = reader.template read<bool>();
            int to_notify = reader.template read<int>();

            std::array<type, capacity> arr;
            for (int i = 0; i < capacity; i++)
            {
                type t = reader.template read<type>();
                arr[i] = std::move(t);
            }

            InPortInformation<type, capacity> *v = ::new (storage)
                InPortInformation<type, capacity>(std::move(name), le, fe, full, std::move(arr), to_notify);

            return v;
        }
    };
};

template <typename type, int capacity> class OutPortInformation final : public OutPortInformationBase
{
    static_assert(capacity > 0);
    static_assert(upcxx::is_serializable<type>::value || upcxx::is_trivially_serializable<type>::value);
    static_assert(std::is_trivially_copy_constructible<type>::value || std::is_copy_constructible<type>::value);
    static_assert(std::is_trivially_move_constructible<type>::value || std::is_move_constructible<type>::value);
    /*
    Information for OutPort
    */
  public:
    int unusedCap;
    std::vector<type> buffer;

    std::unique_ptr<OutPortInformationBase> generateCopy() const override final
    {
        return std::make_unique<OutPortInformation<type, capacity>>(name, unusedCap, buffer);
    }

    OutPortInformation() : OutPortInformationBase("none"), unusedCap(capacity), buffer() {}

    OutPortInformation(std::string &&name, int unused, std::vector<type> &&buf)
        : OutPortInformationBase(std::move(name)), unusedCap(unused), buffer(std::move(buf))
    {
    }

    OutPortInformation(const std::string &name, int unused, std::vector<type> &&buf)
        : OutPortInformationBase(name), unusedCap(unused), buffer(std::move(buf))
    {
    }

    OutPortInformation(const std::string &name, int unused, const std::vector<type> &buf)
        : OutPortInformationBase(name), unusedCap(unused), buffer(buf)
    {
    }

    OutPortInformation(const OutPortInformation<type, capacity> &ci) = delete;

    OutPortInformation<type, capacity> &operator=(const OutPortInformation &ci) = delete;

    OutPortInformation<type, capacity> &operator=(OutPortInformation &&ci)
    {
        this->name = std::move(ci.name);
        this->unusedCap = ci.unusedCap;
        this->buffer = std::move(ci.buffer);
        return *this;
    }

    OutPortInformation(OutPortInformation<type, capacity> &&ci)
        : OutPortInformationBase(std::move(ci.name)), unusedCap(ci.unusedCap), buffer(std::move(ci.buffer))
    {
    }

    ~OutPortInformation() override final{};

    struct upcxx_serialization
    {
        template <typename Writer>
        static void serialize(Writer &writer, OutPortInformation<type, capacity> const &object)
        {
            if (object.name.empty())
            {
                throw std::runtime_error("OutPortInfo name should not be empty");
            }
            writer.write(object.name);
            writer.write(object.unusedCap);

            size_t t = object.buffer.size();
            writer.write(t);

            for (auto &el : object.buffer)
            {
                writer.write(el);
            }

            return;
        }

        template <typename Reader> static OutPortInformation<type, capacity> *deserialize(Reader &reader, void *storage)
        {
            std::string name = reader.template read<std::string>();
            int cap = reader.template read<int>();

            size_t elcount = reader.template read<size_t>();
            std::vector<type> vec;
            for (size_t i = 0; i < elcount; i++)
            {
                type t = reader.template read<type>();
                vec.push_back(std::move(t));
            }

            OutPortInformation<type, capacity> *v =
                ::new (storage) OutPortInformation<type, capacity>(std::move(name), cap, std::move(vec));
            return v;
        }
    };
};

/*
    Serializing empty vectors where vec = {} results in nullptr trying to be dereferenced which results a runtime error,
   but does not cause malfunction still it is overriden so that there is no runtime error

*/

template <int capacity> class InPortInformation<std::vector<float>, capacity> final : public InPortInformationBase
{
    static_assert(capacity > 0);
    static_assert(upcxx::is_serializable<std::vector<float>>::value ||
                  upcxx::is_trivially_serializable<std::vector<float>>::value);
    static_assert(std::is_trivially_copy_constructible<std::vector<float>>::value ||
                  std::is_copy_constructible<std::vector<float>>::value);
    static_assert(std::is_trivially_move_constructible<std::vector<float>>::value ||
                  std::is_move_constructible<std::vector<float>>::value);

  public:
    int lastEl;
    int firstEl;
    bool full;
    std::array<std::vector<float>, capacity> data;
    int to_notify;

    std::unique_ptr<InPortInformationBase> generateCopy() const override final
    {
        return std::make_unique<InPortInformation<std::vector<float>, capacity>>(name, lastEl, firstEl, full, data,
                                                                                 to_notify);
    }

    InPortInformation() : InPortInformationBase("none"), lastEl(0), firstEl(0), full(false), data{}, to_notify(0) {}

    InPortInformation(std::string &&name, int last, int first, bool full,
                      std::array<std::vector<float>, capacity> &&inp, int to_notify)
        : InPortInformationBase(std::move(name)), lastEl(last), firstEl(first), full(full), data(std::move(inp)),
          to_notify(to_notify)
    {
    }

    InPortInformation(const std::string &name, int last, int first, bool full,
                      std::array<std::vector<float>, capacity> &&inp, int to_notify)
        : InPortInformationBase(name), lastEl(last), firstEl(first), full(full), data(std::move(inp)),
          to_notify(to_notify)
    {
    }

    InPortInformation(const std::string &name, int last, int first, bool full,
                      const std::array<std::vector<float>, capacity> &inp, int to_notify)
        : InPortInformationBase(name), lastEl(last), firstEl(first), full(full), data(inp), to_notify(to_notify)
    {
    }

    InPortInformation(const InPortInformation<std::vector<float>, capacity> &ci) = delete;

    InPortInformation(InPortInformation<std::vector<float>, capacity> &&ci)
        : InPortInformationBase(std::move(ci.name)), lastEl(ci.lastEl), firstEl(ci.firstEl), full(ci.full),
          data(std::move(ci.data)), to_notify(ci.to_notify)
    {
    }

    InPortInformation<std::vector<float>, capacity> &
    operator=(const InPortInformation<std::vector<float>, capacity> &ci) = delete;

    InPortInformation<std::vector<float>, capacity> &operator=(InPortInformation<std::vector<float>, capacity> &&other)
    {
        this->name = std::move(other.name);
        this->lastEl = other.lastEl;
        this->firstEl = other.firstEl;
        this->full = other.full;
        this->data = std::move(other.data);
        this->to_notify = other.to_notify;
        return *this;
    }

    ~InPortInformation() override final{};

    size_t getMessageCount() const override final
    {
        int tmpLast = lastEl;
        if (lastEl < firstEl || full)
        {
            tmpLast += capacity;
        }
        return tmpLast - firstEl;
    }

    struct upcxx_serialization
    {
        template <typename Writer>
        static void serialize(Writer &writer, InPortInformation<std::vector<float>, capacity> const &object)
        {
            if (object.name.empty())
            {
                throw std::runtime_error("InPortInfo name should not be empty");
            }
            writer.write(object.name);
            writer.write(object.lastEl);
            writer.write(object.firstEl);
            writer.write(object.full);
            writer.write(object.to_notify);

            size_t size = object.getMessageCount();
            writer.write(size);
            size_t sent_size = 0;

            if (size != 0)
            {
                for (int i = 0; i < capacity; i++)
                {
                    if (!object.data[i].empty())
                    {
                        writer.write(i);
                        writer.write(object.data[i]);
                        sent_size += 1;
                    }
                }
            }

            if (sent_size != size)
            {
                throw std::runtime_error("Amount of serialized messages and messages inside are not the same");
            }

            return;
        }

        template <typename Reader>
        static InPortInformation<std::vector<float>, capacity> *deserialize(Reader &reader, void *storage)
        {
            std::string name = reader.template read<std::string>();
            int le = reader.template read<int>();
            int fe = reader.template read<int>();
            bool full = reader.template read<bool>();
            int to_notify = reader.template read<int>();

            size_t message_count = reader.template read<size_t>();
            std::array<std::vector<float>, capacity> arr;

            for (size_t i = 0; i < message_count; i++)
            {
                int offset = reader.template read<int>();
                std::vector<float> tmp = reader.template read<std::vector<float>>();
                if (offset >= capacity)
                {
                    throw std::runtime_error("Message offset should not be higher than the capacity!");
                }
                arr[offset] = std::move(tmp);
            }

            InPortInformation<std::vector<float>, capacity> *v =
                ::new (storage) InPortInformation<std::vector<float>, capacity>(std::move(name), le, fe, full,
                                                                                std::move(arr), to_notify);

            return v;
        }
    };
};

template <int capacity> class OutPortInformation<std::vector<float>, capacity> final : public OutPortInformationBase
{
    static_assert(capacity > 0);
    static_assert(upcxx::is_serializable<std::vector<float>>::value ||
                  upcxx::is_trivially_serializable<std::vector<float>>::value);
    static_assert(std::is_trivially_copy_constructible<std::vector<float>>::value ||
                  std::is_copy_constructible<std::vector<float>>::value);
    static_assert(std::is_trivially_move_constructible<std::vector<float>>::value ||
                  std::is_move_constructible<std::vector<float>>::value);

    // Information for OutPort
  public:
    int unusedCap;
    std::vector<std::vector<float>> buffer;

    std::unique_ptr<OutPortInformationBase> generateCopy() const override final
    {
        return std::make_unique<OutPortInformation<std::vector<float>, capacity>>(name, unusedCap, buffer);
    }

    OutPortInformation() : OutPortInformationBase("none"), unusedCap(capacity), buffer() {}

    OutPortInformation(const std::string &name, int unused, const std::vector<std::vector<float>> &buf)
        : OutPortInformationBase(name), unusedCap(unused), buffer(buf)
    {
    }

    OutPortInformation(std::string &&name, int unused, std::vector<std::vector<float>> &&buf)
        : OutPortInformationBase(std::move(name)), unusedCap(unused), buffer(std::move(buf))
    {
    }

    OutPortInformation(const std::string &name, int unused, std::vector<std::vector<float>> &&buf)
        : OutPortInformationBase(name), unusedCap(unused), buffer(std::move(buf))
    {
    }

    OutPortInformation(const OutPortInformation<std::vector<float>, capacity> &ci) = delete;

    OutPortInformation<std::vector<float>, capacity> &operator=(const OutPortInformation &ci) = delete;

    OutPortInformation<std::vector<float>, capacity> &operator=(OutPortInformation &&ci)
    {
        this->name = std::move(ci.name);
        this->unusedCap = ci.unusedCap;
        this->buffer = std::move(ci.buffer);
        return *this;
    }

    OutPortInformation(OutPortInformation<std::vector<float>, capacity> &&ci)
        : OutPortInformationBase(std::move(ci.name)), unusedCap(ci.unusedCap), buffer(std::move(ci.buffer))
    {
    }

    ~OutPortInformation() override final{};

    struct upcxx_serialization
    {
        template <typename Writer>
        static void serialize(Writer &writer, OutPortInformation<std::vector<float>, capacity> const &object)
        {
            if (object.name.empty())
            {
                throw std::runtime_error("OutPortInfo name should not be empty");
            }
            writer.write(object.name);
            writer.write(object.unusedCap);
            writer.write(object.buffer);

            /*
            size_t t = object.buffer.size();
            writer.write(t);

            for (size_t i = 0; i < t; i++)
            {
                if (object.buffer[i].empty())
                {
                    size_t size = 0;
                    writer.write(size);
                }
                else
                {
                    // write the size of elements
                    size_t size = object.buffer[i].size();
                    writer.write(size);
                    // write the sequence of elements
                    // writer.write_sequence(object.data[i].begin(), object.data[i].end(), size);
                    writer.write(object.buffer[i]);
                }
            }
            */

            return;
        }

        template <typename Reader>
        static OutPortInformation<std::vector<float>, capacity> *deserialize(Reader &reader, void *storage)
        {
            std::string name = reader.template read<std::string>();
            int cap = reader.template read<int>();
            std::vector<std::vector<float>> *vec = util::read<Reader, std::vector<std::vector<float>>>(reader);

            /*
            size_t elcount = reader.template read<size_t>();
            std::vector<std::vector<float>> vec;
            vec.resize(elcount);
            for (size_t i = 0; i < elcount; i++)
            {
                size_t data = reader.template read<size_t>();
                if (data > 0)
                {
                    // allocate vector and read into the saved space
                    std::vector<float> tmp = reader.template read<std::vector<float>>();
                    vec[i] = std::move(tmp);
                }
            }
            */

            OutPortInformation<std::vector<float>, capacity> *v =
                ::new (storage) OutPortInformation<std::vector<float>, capacity>(std::move(name), cap, std::move(*vec));

            delete vec;
            return v;
        }
    };
};
