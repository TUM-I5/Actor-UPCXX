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

#include "PortInformation.hpp"
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <upcxx/upcxx.hpp>

#include "AbstractInPort.hpp"
#include "AbstractOutPort.hpp"
#include "ActorImpl.hpp"
#include "Channel.hpp"
#include "OutPort.hpp"
#include "config.hpp"

#ifdef TRACE
#include "VT.h"
#endif

/*

    Warning the out port capacity is often updated with an rpc this means if the actors read many in a call with no time
   to call for upc++ progress then there will be an error in the ports for not having enough space etc.

*/

class TaskDeque;
class AbstractOutPort;

template <typename type, int capacity> class InPort : public AbstractInPort
{

    friend class ActorImpl;
    friend class Tracker;
    friend class ActorGraph;

  private:
    upcxx::global_ptr<Channel<type, capacity>> globalConnectedChannel; // global ptr of created channel
    Channel<type, capacity> *connectedChannel;                         // .local() ptr of chanel
    GlobalChannelRef opaqueChannelRef; // type_casted (global void*) of channel for the library
#ifdef PARALLEL
    std::mutex lock;
#endif

#ifdef TRACE
    static int event_read;
#endif
    int size_to_notify = -1;

  public:
    static const int channelSize = capacity;
    type read();                             // pop first el
    type peek();                             // get copy of first el
    size_t available() const final override; // remaining buffer size
    std::string toString() final override;
    void deregister() final override;          // disconnect cannel
    void registerWithChannel() final override; // connect channel
    void applyPortInfo(int last, int first, bool full,
                       std::array<type, capacity> &&arr); // apply port info (got from InPortInformation arguments)
    void applyPortInfo(std::unique_ptr<InPortInformation<type, capacity>> &&ipo); // apply port info got from inportinfo
    void applyPortInfo(std::unique_ptr<InPortInformationBase> &&ipo)
        final override; // cast ipo to the require type and call applyPortInfo
    std::unique_ptr<InPortInformationBase>
    generatePortInfo() final override; // call generatePortInfoTyped and castit to base clas
    ~InPort<type, capacity>();
    void setTaskDequeOfChannel(TaskDeque *tdq) final override;
    bool severeConnection() final override;
    bool resurrectConnection(upcxx::intrank_t rank, AbstractOutPort *outptr) final override;
    GlobalChannelRef getChannelPtr() const final override;
    void notify() final override;
    bool noNeedToNotify() final override;
    bool connected() final override;
    bool empty() final override;
    void print() const final override;
    // AbstractInPort* copy()  final override;
    std::vector<type> extract_messages();
    void reinsert_messages(std::vector<type> &&msgs);

  private:
    InPort<type, capacity>(const std::string &name, const std::string &actorName);
    InPort<type, capacity>(const InPort<type, capacity> &other) = delete;
    InPort<type, capacity>(InPort<type, capacity> &&other) = delete;
    void readInternalSeq();   // read with seq (upcxxranks) strategy
    void readInternalTasks(); // read with omp

  public:
};

#ifdef TRACE
template <typename type, int capacity> int InPort<type, capacity>::event_read = -1;
#endif

template <typename type, int capacity>
InPort<type, capacity>::InPort(const std::string &name, const std::string &actorName)
    : AbstractInPort(name), globalConnectedChannel(upcxx::new_<Channel<type, capacity>>(
                                actorName, upcxx::rank_me(), dynamic_cast<AbstractInPort *>(this))),
      connectedChannel(globalConnectedChannel.local()),
      opaqueChannelRef(upcxx::reinterpret_pointer_cast<void>(globalConnectedChannel))
{
#ifdef TRACE
    std::string event_read_name = "read";
    if (event_read == -1)
        VT_funcdef(event_read_name.c_str(), VT_NOCLASS, &InPort<T, capacity>::event_read);
#endif
}

template <typename type, int capacity> InPort<type, capacity>::~InPort() { upcxx::delete_(globalConnectedChannel); }

template <typename type, int capacity> size_t InPort<type, capacity>::available() const
{
#ifdef PARALLEL
    std::lock_guard<std::mutex> readLock(lock);
#endif

    if (connectedChannel != nullptr)
    {
        return connectedChannel->size();
    }
    else
    {
        throw std::runtime_error("Unable to get size, channel not connected.");
    }
}

template <typename type, int capacity> type InPort<type, capacity>::peek()
{
#ifdef PARALLEL
    std::lock_guard<std::mutex> readLock(lock);
#endif

    return this->connectedChannel->peek();
}

template <typename type, int capacity> type InPort<type, capacity>::read()
{
#ifdef PARALLEL
    std::lock_guard<std::mutex> readLock(lock);
#endif

#ifdef TRACE
    VT_begin(InPort<type, capacity>::event_read);
#endif

    auto el = this->connectedChannel->dequeue();

    if constexpr (config::parallelization == config::ParallelizationType::UPCXX_RANKS)
    {
        readInternalSeq();
    }
    else
    {
        readInternalTasks();
    }

#ifdef TRACE
    VT_end(InPort<type, capacity>::event_read);
#endif

    return el;
}

template <typename type, int capacity> void InPort<type, capacity>::notify()
{
    if (size_to_notify != -1)
    {
        auto cop = this->connectedChannel->connectedOutPort;
        if (cop.second == nullptr)
        {
            // just notify later
        }
        else
        {

            if (cop.first == upcxx::rank_me())
            {
                AbstractOutPort *op = cop.second;
                dynamic_cast<OutPort<type, capacity> *>(op)->updateCapacity(size_to_notify);
            }
            else
            {
                upcxx::rpc_ff(
                    cop.first,
                    [](AbstractOutPort *op, size_t newCapacity)
                    {
                        // if (!op->checkParentActorIsActive())
                        //{
                        //    throw std::runtime_error("The actor is during act, precondition is not satisfied.");
                        //}

                        // op->setParentActorActive();
                        dynamic_cast<OutPort<type, capacity> *>(op)->updateCapacity(newCapacity);
                        // op->releaseParentActorActive();
                    },
                    cop.second, size_to_notify);
            }

            size_to_notify = -1;
        }
    }
}

template <typename type, int capacity> bool InPort<type, capacity>::noNeedToNotify() { return size_to_notify == -1; }

template <typename type, int capacity> void InPort<type, capacity>::readInternalTasks()
{
    auto cop = this->connectedChannel->connectedOutPort;
    if (cop.second == nullptr)
    {
        size_to_notify = capacity - (connectedChannel->size() + 1);
    }
    else
    {
        {
            if (cop.first == upcxx::rank_me())
            {
                AbstractOutPort *op = cop.second;
                size_t newCapacity = capacity - (connectedChannel->size() + 1);
                dynamic_cast<OutPort<type, capacity> *>(op)->updateCapacity(newCapacity);
            }
            else
            {
                upcxx::rpc_ff(
                    cop.first,
                    [](AbstractOutPort *op, size_t newCapacity)
                    { dynamic_cast<OutPort<type, capacity> *>(op)->updateCapacity(newCapacity); },
                    cop.second, (capacity - (connectedChannel->size() + 1)));
            }
            size_to_notify = -1;
        }
    }
}

template <typename type, int capacity> void InPort<type, capacity>::readInternalSeq()
{
    auto cop = this->connectedChannel->connectedOutPort;
    if (cop.second == nullptr)
    {
        size_to_notify = capacity - (connectedChannel->size() + 1);
    }
    else
    {
        if (cop.first == upcxx::rank_me())
        {
            AbstractOutPort *op = cop.second;
            size_t newCapacity = capacity - (connectedChannel->size() + 1);
            dynamic_cast<OutPort<type, capacity> *>(op)->updateCapacity(newCapacity);
        }
        else
        {
            upcxx::rpc_ff(
                cop.first,
                [](AbstractOutPort *op, size_t newCapacity)
                { dynamic_cast<OutPort<type, capacity> *>(op)->updateCapacity(newCapacity); },
                cop.second, (capacity - (connectedChannel->size() + 1)));
        }
        size_to_notify = -1;
    }
}

template <typename type, int capacity> std::string InPort<type, capacity>::toString()
{
    std::stringstream ss;
    ss << "[IP-" << capacity << " ID: " << name << " C: " << opaqueChannelRef << "]";
    return ss.str();
}

template <typename type, int capacity> void InPort<type, capacity>::registerWithChannel()
{

    /*
    if (this->connectedChannel->connectedInPort.first == -1 &&
        this->connectedChannel->connectedInPort.second == nullptr)
    {
        this->connectedChannel->connectedInPort = std::make_pair(upcxx::rank_me(), this);
    }
    */
}

template <typename type, int capacity> void InPort<type, capacity>::deregister()
{

    /*
    if (this->connectedChannel->connectedInPort.first != -1 &&
        this->connectedChannel->connectedInPort.second != nullptr)
    {
        this->connectedChannel->connectedInPort.first = -1;
        this->connectedChannel->connectedInPort.second = nullptr;
    }
    */
}

template <typename type, int capacity>
void InPort<type, capacity>::applyPortInfo(int last, int first, bool full, std::array<type, capacity> &&arr)
{
    this->connectedChannel->applyChannelInfo(last, first, full, std::move(arr));
}

template <typename type, int capacity>
void InPort<type, capacity>::applyPortInfo(std::unique_ptr<InPortInformation<type, capacity>> &&ipo)
{
    if (this->connectedChannel == nullptr)
    {
        throw std::runtime_error("Apply port information only after reconnecting");
    }
    else
    {
        this->connectedChannel->applyChannelInfo(std::move(ipo));
    }
}

template <typename type, int capacity>
void InPort<type, capacity>::applyPortInfo(std::unique_ptr<InPortInformationBase> &&ipo)
{
    InPortInformation<type, capacity> *ptr = dynamic_cast<InPortInformation<type, capacity> *>(ipo.release());
    std::unique_ptr<InPortInformation<type, capacity>> ptr2(ptr);
    this->applyPortInfo(std::move(ptr2));
}

template <typename type, int capacity> std::unique_ptr<InPortInformationBase> InPort<type, capacity>::generatePortInfo()
{
    if (this->connectedChannel == nullptr)
    {
        throw std::runtime_error("Connected channel is nullptr!");
    }
    else
    {
        std::string s = AbstractInPort::str();
        std::unique_ptr<InPortInformation<type, capacity>> ptr =
            this->connectedChannel->generatePortInfo(std::move(s), size_to_notify);
        return ptr;
    }
}

template <typename type, int capacity> void InPort<type, capacity>::setTaskDequeOfChannel(TaskDeque *tdq)
{
    this->connectedChannel->setTaskDeque(tdq);
}

template <typename type, int capacity> bool InPort<type, capacity>::severeConnection()
{
    return this->connectedChannel->severeConnection();
}

template <typename type, int capacity>
bool InPort<type, capacity>::resurrectConnection(upcxx::intrank_t rank, AbstractOutPort *outptr)
{
    return this->connectedChannel->resurrectConnection(rank, outptr);
}

template <typename type, int capacity> GlobalChannelRef InPort<type, capacity>::getChannelPtr() const
{
    return static_cast<GlobalChannelRef>(opaqueChannelRef);
}

template <typename type, int capacity> bool InPort<type, capacity>::connected()
{
    return this->connectedChannel->connectedOutPort.second != nullptr;
}

template <typename type, int capacity> bool InPort<type, capacity>::empty() { return this->connectedChannel->empty(); }

template <typename type, int capacity> void InPort<type, capacity>::print() const
{
    return this->connectedChannel->print(name);
}

template <typename type, int capacity> std::vector<type> InPort<type, capacity>::extract_messages()
{
    return this->connectedChannel->extract_messages();
}

template <typename type, int capacity> void InPort<type, capacity>::reinsert_messages(std::vector<type> &&msgs)
{
    for (auto &&el : msgs)
    {
        this->connectedChannel->enqueue(std::move(el));
    }
}