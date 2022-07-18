/**
 * @file
 * This file is part of actorlib.
 *
 * @author Alexander Pöppl (poeppl AT in.tum.de,
 * https://www5.in.tum.de/wiki/index.php/Alexander_P%C3%B6ppl,_M.Sc.)
 *
 * @section LICENSE
 *
 * actorlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * actorlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with actorlib.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * @section DESCRIPTION
 *
 * TODO
 */
#pragma once
#include <iostream>
#include <memory>
#include <mutex>

#include "AbstractInPort.hpp"
#include "AbstractOutPort.hpp"
#include "ActorImpl.hpp"
#include "Channel.hpp"
#include "PortInformation.hpp"
#include "SerializeOptional.hpp"
#include "config.hpp"
#include <functional>
#include <optional>
#include <upcxx/upcxx.hpp>

#ifdef TRACE
#include "VT.h"
#endif

class ActorGraph;

template <typename type, int capacity> class OutPort : public AbstractOutPort
{

    friend class ActorImpl;
    friend class ActorGraph;
    friend class Tracker;

  private:
    upcxx::global_ptr<Channel<type, capacity>> remoteChannel; // connected channel
    mutable int unusedCapacity;                               // possible writes left
#ifdef PARALLEL
    std::mutex lock;
#endif
#ifdef TRACE
    static int event_write;
#endif
#ifdef PARALLEL
    std::atomic<unsigned int> messages_on_wire;
#else
    unsigned int messages_on_wire;
#endif
    mutable std::vector<type> buffer;

  public:
    void write(type &&element); // write to channel
    size_t freeCapacity();      // left capacity
    void updateCapacity(size_t newVal);
    std::string toString() final override;
    upcxx::future<> registerWithChannel(GlobalChannelRef ref) final override;         // connect to a remote channel
    upcxx::future<> deregister() final override;                                      // disconnect from remote channel
    void setChannel(GlobalChannelRef newChannel) final override;                      // change channel pointers
    std::unique_ptr<OutPortInformation<type, capacity>> generatePortInfoTyped();      // generate outport information
    void applyPortInfo(int cap, std::vector<std::vector<float>> &&buf);               // apply op information
    void applyPortInfo(std::unique_ptr<OutPortInformation<type, capacity>> &&opo);    // apply OutPortInformation typed
    void applyPortInfo(std::unique_ptr<OutPortInformationBase> &&opo) final override; // downcast and call the foo above
    std::unique_ptr<OutPortInformationBase> generatePortInfo() final override;        // generate ports information
    bool messageOnWire() const final override;
    bool severeConnection() final override;
    bool resurrectConnection(GlobalChannelRef rptr) final override;
    size_t bufferSize() const final override;
    void writeBuffer() const final override;
    ~OutPort<type, capacity>();
    bool connected() const final override;

  private:
    OutPort<type, capacity>(std::string &&name);
    OutPort<type, capacity>(const std::string &name);
    OutPort<type, capacity>(const OutPort<type, capacity> &other) = delete;
    OutPort<type, capacity>(OutPort<type, capacity> &&other) = delete;
    void writeInternalTasks(type &&element);
    void writeInternalSeq(type &&element) const;

  public:
};

#ifdef TRACE
template <typename type, int capacity> int OutPort<type, capacity>::event_write = -1;
#endif

template <typename type, int capacity>
OutPort<type, capacity>::OutPort(std::string &&name)
    : AbstractOutPort(std::move(name)), remoteChannel(nullptr), unusedCapacity(capacity), messages_on_wire(0), buffer()
{
#ifdef TRACE
    std::string event_write_name = "write";
    if (OutPort<type, capacity>::event_write == -1)
        int ierr = VT_funcdef(event_write_name.c_str(), VT_NOCLASS, &OutPort<type, capacity>::event_write);
#endif
}

template <typename type, int capacity>
OutPort<type, capacity>::OutPort(const std::string &name)
    : AbstractOutPort(name), remoteChannel(nullptr), unusedCapacity(capacity), messages_on_wire(0), buffer()
{
#ifdef TRACE
    std::string event_write_name = "write";
    if (OutPort<type, capacity>::event_write == -1)
        int ierr = VT_funcdef(event_write_name.c_str(), VT_NOCLASS, &OutPort<type, capacity>::event_write);
#endif
}

template <typename type, int capacity> size_t OutPort<type, capacity>::freeCapacity()
{
#ifdef PARALLEL
    std::lock_guard<std::mutex> writeLock(lock);
#endif

    return this->unusedCapacity;
}

template <typename type, int capacity> void OutPort<type, capacity>::updateCapacity(size_t newVal)
{
#ifdef PARALLEL
    std::lock_guard<std::mutex> writeLock(lock);
#endif

    unusedCapacity = newVal;
    messages_on_wire--;
}

template <typename type, int capacity> void OutPort<type, capacity>::setChannel(GlobalChannelRef newChannel)
{
    if (newChannel != nullptr)
    {
        this->remoteChannel = upcxx::reinterpret_pointer_cast<Channel<type, capacity>>(newChannel);
    }
    else
    {
        this->remoteChannel = nullptr;
    }
}

template <typename type, int capacity> void OutPort<type, capacity>::write(type &&element)
{
#ifdef TRACE
    VT_begin(OutPort<type, capacity>::event_write);
#endif

#ifdef PARALLEL
    std::lock_guard<std::mutex> writeLock(lock);
#endif

    /*
    this is not an error anymore we will just write to buffer

    if (remoteChannel == nullptr)
    {
        throw std::runtime_error(std::string("Unable to write to channel, channel not connected. "
                                             "Channel: ") +
                                 name + " " + std::to_string(remoteChannel.where()) + " " +
    std::to_string((size_t)remoteChannel.local()) + " of actor: " + this->connectedActor->getName());
    }
    */
    if (remoteChannel == nullptr)
    {
        buffer.emplace_back(std::move(element));
    }
    else
    {

        if (unusedCapacity == 0)
        {
            throw std::runtime_error("No free space in channel!");
        }

        writeBuffer();

        if (unusedCapacity > 0)
        {
            this->unusedCapacity--;
            writeInternalSeq(std::move(element));
        }
        else
        {
            buffer.push_back(std::move(element));
        }
    }

#ifdef TRACE
    VT_end(OutPort<type, capacity>::event_write);
#endif
}

template <typename type, int capacity> void OutPort<type, capacity>::writeInternalTasks(type &&element)
{

    if (remoteChannel.where() == upcxx::rank_me())
    {
        remoteChannel.local()->enqueue(std::move(element));
    }
    else
    {
        upcxx::rpc_ff(
            remoteChannel.where(),
            [](upcxx::global_ptr<Channel<type, capacity>> remoteChannel, type data)
            { remoteChannel.local()->enqueue(std::move(data)); },
            remoteChannel, std::move(element));
    }
}

template <typename type, int capacity> void OutPort<type, capacity>::writeInternalSeq(type &&element) const
{
    if (remoteChannel.where() == upcxx::rank_me())
    {
        remoteChannel.local()->enqueue(std::move(element));
    }
    else
    {
        upcxx::rpc_ff(
            remoteChannel.where(),
            [](upcxx::global_ptr<Channel<type, capacity>> remoteChannel, type data)
            { remoteChannel.local()->enqueue(std::move(data)); },
            remoteChannel, std::move(element));
    }
}

template <typename type, int capacity> std::string OutPort<type, capacity>::toString()
{
    std::stringstream ss;
    ss << "[OP-" << capacity << " ID: " << name << " C: " << this->remoteChannel
       << "] Buffer size: " << this->buffer.size();
    return ss.str();
}

// update remtoe actors name as a side effect
template <typename type, int capacity>
upcxx::future<> OutPort<type, capacity>::registerWithChannel(GlobalChannelRef ref)
{
    upcxx::intrank_t opRank = upcxx::rank_me();
    AbstractOutPort *cop = this;

    if (this->remoteChannel == nullptr)
    {
        this->remoteChannel = upcxx::reinterpret_pointer_cast<Channel<type, capacity>>(ref);

        if (this->remoteChannel.where() != upcxx::rank_me())
        {
            return upcxx::rpc(
                this->remoteChannel.where(),
                [](upcxx::global_ptr<Channel<type, capacity>> c, upcxx::intrank_t opRank, AbstractOutPort *cop)
                {
                    if (c.local()->connectedOutPort.first == -1 && c.local()->connectedOutPort.second == nullptr)
                    {
                        c.local()->connectedOutPort.first = opRank;
                        c.local()->connectedOutPort.second = cop;
                    }
                    else
                    {
                        throw std::runtime_error("remote channel was nullptr but channel had information");
                    }
                },
                this->remoteChannel, opRank, cop);
        }
        else
        {
            upcxx::global_ptr<Channel<type, capacity>> c = this->remoteChannel;
            if (c.local()->connectedOutPort.first == -1 && c.local()->connectedOutPort.second == nullptr)
            {
                c.local()->connectedOutPort.first = opRank;
                c.local()->connectedOutPort.second = cop;
            }
            else
            {
                throw std::runtime_error("remote channel was nullptr but channel had information");
            }

            return upcxx::make_future();
        }
    }
    else
    {
        throw std::runtime_error("If you call register With Channel remote channel should not be nullptr");
        return upcxx::make_future();
    }
}

template <typename type, int capacity> upcxx::future<> OutPort<type, capacity>::deregister()
{
    if (this->remoteChannel != nullptr)
    {
        if (this->remoteChannel.where() != upcxx::rank_me())
        {
            auto fut = upcxx::rpc(
                this->remoteChannel.where(),
                [](upcxx::global_ptr<Channel<type, capacity>> c)
                {
                    if (c.local()->connectedOutPort.first != -1 && c.local()->connectedOutPort.second != nullptr)
                    {
                        c.local()->connectedOutPort.first = -1;
                        c.local()->connectedOutPort.second = nullptr;
                    }
                },
                this->remoteChannel);
            this->remoteChannel = nullptr;
            return fut;
        }
        else
        {
            upcxx::global_ptr<Channel<type, capacity>> c = this->remoteChannel;
            if (c.local()->connectedOutPort.first != -1 && c.local()->connectedOutPort.second != nullptr)
            {
                c.local()->connectedOutPort.first = -1;
                c.local()->connectedOutPort.second = nullptr;
            }
            this->remoteChannel = nullptr;
            return upcxx::make_future();
        }
    }
    else
    {
        throw std::runtime_error("If you call deregister With Channel remote channel should not be nullptr");
        return upcxx::make_future();
    }
}

template <typename type, int capacity>
void OutPort<type, capacity>::applyPortInfo(std::unique_ptr<OutPortInformation<type, capacity>> &&opo)
{
    if (opo == nullptr)
    {
        throw std::runtime_error("Nullpointer in bad state");
    }
    if (name.compare(opo->name))
    {
        throw std::runtime_error("apply port info name mismatch");
    }
    unusedCapacity = opo->unusedCap;
    this->buffer = std::move(opo->buffer);
}

template <typename type, int capacity>
void OutPort<type, capacity>::applyPortInfo(int cap, std::vector<std::vector<float>> &&buf)
{
    unusedCapacity = cap;
    this->buffer = std::move(buf);
}

template <typename type, int capacity>
void OutPort<type, capacity>::applyPortInfo(std::unique_ptr<OutPortInformationBase> &&opo)
{
    OutPortInformation<type, capacity> *ptr = dynamic_cast<OutPortInformation<type, capacity> *>(opo.release());
    std::unique_ptr<OutPortInformation<type, capacity>> ptr2(ptr);
    this->applyPortInfo(std::move(ptr2));
}

template <typename type, int capacity>
std::unique_ptr<OutPortInformation<type, capacity>> OutPort<type, capacity>::generatePortInfoTyped()
{
    auto l = std::make_unique<OutPortInformation<type, capacity>>(std::move(this->name), this->unusedCapacity,
                                                                  std::move(this->buffer));
    if (!buffer.empty())
    {
        throw std::runtime_error("by the time of generating port info it buffer should be empty!");
    }
    unusedCapacity = 0;
    return l;
}

template <typename type, int capacity>
std::unique_ptr<OutPortInformationBase> OutPort<type, capacity>::generatePortInfo()
{
    auto l = std::make_unique<OutPortInformation<type, capacity>>(std::move(this->name), this->unusedCapacity,
                                                                  std::move(this->buffer));
    if (!buffer.empty())
    {
        throw std::runtime_error("by the time of generating port info it buffer should be empty!");
    }
    unusedCapacity = 0;
    return l;
}

template <typename type, int capacity> bool OutPort<type, capacity>::messageOnWire() const
{
#ifdef PARALLEL
    return messages_on_wire.load() != 0;
#else
    return messages_on_wire != 0;
#endif
}

template <typename type, int capacity> bool OutPort<type, capacity>::severeConnection()
{
    if (remoteChannel == nullptr)
    {
        return false;
    }
    else
    {
        remoteChannel = nullptr;
        return true;
    }
}

template <typename type, int capacity> bool OutPort<type, capacity>::resurrectConnection(GlobalChannelRef rptr)
{
    if (this->remoteChannel == nullptr)
    {
        this->remoteChannel = upcxx::reinterpret_pointer_cast<Channel<type, capacity>>(rptr);
        return true;
    }
    else
    {
        return false;
    }
}

template <typename type, int capacity> size_t OutPort<type, capacity>::bufferSize() const
{
    return this->buffer.size();
}

template <typename type, int capacity> void OutPort<type, capacity>::writeBuffer() const
{
    if (remoteChannel != nullptr)
    {
        for (size_t i = 0; i < buffer.size(); i++)
        {
            if (this->unusedCapacity == 0)
            {
                std::vector<type> t(std::make_move_iterator(buffer.begin() + i), std::make_move_iterator(buffer.end()));
                buffer = std::move(t);
                return;
            }
            this->unusedCapacity--;
            writeInternalSeq(std::move(buffer[i]));
        }
        buffer.clear();
    }
}
template <typename type, int capacity> OutPort<type, capacity>::~OutPort<type, capacity>()
{
    if (buffer.size() != 0)
    {
        std::cerr << upcxx::rank_me() << ": No messages should be left when destructor for outport is called"
                  << std::endl;
    }
}

template <typename type, int capacity> bool OutPort<type, capacity>::connected() const
{
    return this->remoteChannel != nullptr;
}