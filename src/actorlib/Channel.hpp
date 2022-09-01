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
#include "TaskDeque.hpp"
#include "Utility.hpp"
#include <array>
#include <iostream>
#include <memory>
#include <shared_mutex>
#include <sstream>
#include <upcxx/upcxx.hpp>
#include <utility>

class AbstractInPort;
class AbstractOutPort;
class ActorGraph;

class AbstractChannel
{
    // Dummy class to be able to hold a reference
};

class TaskDeque;

template <typename type, int capacity> class Channel : public AbstractChannel
{
    /*
        If type is not serializible like a weird user defined class user has to define serialization methodsfor its type
    */
    static_assert(std::is_trivially_copy_constructible<type>::value || std::is_copy_constructible<type>::value);
    static_assert(std::is_trivially_move_constructible<type>::value || std::is_move_constructible<type>::value);
    static_assert(capacity > 0);

  private:
    /*
        A ring of elements so you have first, last element and isFull, and an Array of type elements to buffer them and
       a mutex
    */
    int lastElement;
    int firstElement;
    bool isFull;
    std::array<type, capacity> queue;
    mutable std::shared_mutex lock;

  public:
    // if the channel is not connected then the pair is (-1,nullptr)
    const std::pair<upcxx::intrank_t, AbstractInPort *> connectedInPort; // the inPort that is connected to this channel
    std::pair<upcxx::intrank_t, AbstractOutPort *> connectedOutPort; // the outPort that is connected to this channel
    std::string inActorName;
    TaskDeque *tdq;

  public:
    Channel(const std::string &inActorName, upcxx::intrank_t rank, AbstractInPort *aip);
    // int enqueue(const type &element);
    int enqueue(type &&element);                          // move-push an element
    type dequeue();                                       // get (removes) first element
    type peek() const;                                    // copy the first element
    const std::array<type, capacity> &getContent() const; // see the content
    std::tuple<int, int, bool> getInfo() const;           // return state of ring but not the elemtns
    size_t size() const;                                  // capcity of buffer
    void applyChannelInfo(int last, int first, bool full,
                          std::array<type, capacity> &&arr); // chane the state of channel with the arguments
    void applyChannelInfo(
        std::unique_ptr<InPortInformation<type, capacity>> &&ipo); // chane the state with the inportinformation
    void applyChannelInfo(
        std::unique_ptr<InPortInformationBase> &&ipo); // change the state with an pointer to informationbase
                                                       // (must be downcasted to the right type)
    std::unique_ptr<InPortInformation<type, capacity>> generatePortInfo(std::string &&portname,
                                                                        int size_to_notify); // same but rvalue
    void triggerInPortActor();
    void setTaskDeque(TaskDeque *tdq);
    bool severeConnection();
    bool resurrectConnection(upcxx::intrank_t rank, AbstractOutPort *outptr);
    bool empty() const;
    ~Channel();
    void print(const std::string &name) const;
    std::vector<type> extract_messages();

  private:
    size_t sizeInt() const; // return size

  public:
};

#include "AbstractInPort.hpp"
#include "AbstractOutPort.hpp"
// BEGIN IMPLEMENTATION

template <typename type, int capacity>
Channel<type, capacity>::Channel(const std::string &inActorName, upcxx::intrank_t rank, AbstractInPort *aip)
    : lastElement(0), firstElement(0), isFull(false), queue(),
      lock(), connectedInPort{rank, aip}, connectedOutPort{-1, nullptr}, inActorName(inActorName), tdq(nullptr)
{
}

template <typename type, int capacity> int Channel<type, capacity>::enqueue(type &&element)
{
    // std::unique_lock<std::shared_mutex> writeLock(lock);

    if ((lastElement == firstElement) % capacity == 0 && isFull)
    {
        std::string err = "Channel is full";
        throw std::runtime_error(err);
    }

    if constexpr (std::is_same<type, std::vector<float>>::value)
    {
        if (!queue[lastElement].empty())
        {
            throw std::runtime_error("overwriting an old message?!");
        }
    }

    queue[lastElement] = std::move(element);
    lastElement = (lastElement + 1) % capacity;
    isFull = (lastElement == firstElement);
    triggerInPortActor();
    return this->sizeInt();
}

template <typename type, int capacity> const std::array<type, capacity> &Channel<type, capacity>::getContent() const
{
    return queue;
}

template <typename type, int capacity> std::tuple<int, int, bool> Channel<type, capacity>::getInfo() const
{
    return {lastElement, firstElement, isFull};
}

template <typename type, int capacity> type Channel<type, capacity>::peek() const
{
    // std::shared_lock<std::shared_mutex> writeLock(lock);

    if (lastElement == firstElement && !isFull)
    {
        throw std::runtime_error("Peek - Channel is empty");
    }
    else
    {
        size_t elemPos = (firstElement) % capacity;
        type element = queue[elemPos];
        return element;
    }
}

template <typename type, int capacity> type Channel<type, capacity>::dequeue()
{
    // std::unique_lock<std::shared_mutex> writeLock(lock);

    if (lastElement == firstElement && !isFull)
    {
        throw std::runtime_error("Dequeue - Channel is empty");
    }
    else
    {
        type element = std::move(queue[firstElement]);

        if constexpr (std::is_same<type, std::vector<float>>::value)
        {
            if (element.empty())
            {
                throw std::runtime_error("We have dequeued an empty message?");
            }
        }

        firstElement = (firstElement + 1) % capacity;
        isFull = false;
        return element;
    }
}

template <typename type, int capacity> size_t Channel<type, capacity>::sizeInt() const
{
    int tmpLast = lastElement;
    if (lastElement < firstElement || isFull)
    {
        tmpLast += capacity;
    }
    return tmpLast - firstElement;
}

template <typename type, int capacity> bool Channel<type, capacity>::empty() const { return size() == 0; }

template <typename type, int capacity> size_t Channel<type, capacity>::size() const
{
    // std::shared_lock<std::shared_mutex> readLock(lock);

    int tmpLast = lastElement;
    if (lastElement < firstElement || isFull)
    {
        tmpLast += capacity;
    }
    return tmpLast - firstElement;
}

template <typename type, int capacity>
void Channel<type, capacity>::applyChannelInfo(int last, int first, bool full, std::array<type, capacity> &&arr)
{
    throw std::runtime_error("dont call this");

    lastElement = last;
    firstElement = first;
    isFull = full;
    queue = std::move(arr);
}

template <typename type, int capacity>
void Channel<type, capacity>::applyChannelInfo(std::unique_ptr<InPortInformation<type, capacity>> &&ipo)
{
    if (lastElement != 0 || firstElement != 0 || isFull != false)
    {
        throw std::runtime_error("Double channel info, or apply channel info not applied to a crisp channel!");
    }

    lastElement = ipo->lastEl;
    firstElement = ipo->firstEl;
    isFull = ipo->full;

    if (ipo->data.size() != capacity)
    {
        throw std::runtime_error("Inportsinformation was serialized wrong?");
    }

    queue = std::move(ipo->data);

#ifdef REPORT_MAIN_ACTIONS
    std::string namecpy = this->connectedInPort.second->name;
    std::string actorname = this->connectedInPort.second->inActorName;
    size_t t = this->size();
    std::cout << actorname << "'s " << namecpy << " has restored " << t << " messages" << std::endl;
    std::cerr << actorname << "'s " << namecpy << " has restored " << t << " messages" << std::endl;
#endif
}

template <typename type, int capacity>
void Channel<type, capacity>::applyChannelInfo(std::unique_ptr<InPortInformationBase> &&ipo)
{
    InPortInformation<type, capacity> *ptr = dynamic_cast<InPortInformation<type, capacity> *>(ipo.release());
    std::unique_ptr<InPortInformation<type, capacity>> p2(ptr);
    this->applyChannelInfo(std::move(p2));
}

template <typename type, int capacity>
std::unique_ptr<InPortInformation<type, capacity>> Channel<type, capacity>::generatePortInfo(std::string &&portname,
                                                                                             int size_to_notify)
{
    std::string namecpy = portname;
    size_t t = this->size();
    std::string actorname = this->connectedInPort.second->inActorName;

#ifdef REPORT_MAIN_ACTIONS
    std::cout << actorname << "'s " << namecpy << " has " << t << " messages" << std::endl;
    std::cerr << actorname << "'s " << namecpy << " has " << t << " messages" << std::endl;
#endif

    std::unique_ptr<InPortInformation<type, capacity>> tret =
        std::make_unique<InPortInformation<type, capacity>>(std::move(namecpy), this->lastElement, this->firstElement,
                                                            this->isFull, std::move(this->queue), size_to_notify);

    lastElement = 0;
    firstElement = 0;
    isFull = false;
    size_to_notify = 0;
    queue = {};
    return tret;
}

template <typename type, int capacity> void Channel<type, capacity>::setTaskDeque(TaskDeque *tdq) { this->tdq = tdq; }

template <typename type, int capacity> void Channel<type, capacity>::triggerInPortActor()
{
#ifdef USE_ACTOR_TRIGGERS
    if (this->tdq == nullptr)
    {
        throw std::runtime_error("The taskdeque should have been set!");
    }

    if (this->inActorName.empty())
    {
        throw std::runtime_error("name empty");
    }

    this->tdq->addTrigger(this->inActorName, this->connectedInPort.second->name);
#endif

    std::string inPortName = this->connectedInPort.second->getName();
    this->connectedInPort.second->connectedActor->addTask(inPortName);
}

template <typename type, int capacity> bool Channel<type, capacity>::severeConnection()
{
    if (std::get<0>(this->connectedOutPort) >= 0 && std::get<1>(this->connectedOutPort) != nullptr)
    {
        this->connectedOutPort = {-1, nullptr};

        return true;
    }
    else
    {
        return false;
    }
}

template <typename type, int capacity>
bool Channel<type, capacity>::resurrectConnection(upcxx::intrank_t rank, AbstractOutPort *outptr)
{
    if (std::get<0>(this->connectedOutPort) < 0 && std::get<1>(this->connectedOutPort) == nullptr)
    {
        this->connectedOutPort = {rank, outptr};

        return true;
    }
    else
    {
        return false;
    }
}

template <typename type, int capacity> Channel<type, capacity>::~Channel() {}

template <typename type, int capacity> void Channel<type, capacity>::print(const std::string &name) const
{
    std::stringstream ss;
    ss << "State of: " << name << ":\n";
    ss << "last element: " << lastElement << ", first element: " << firstElement << ", is full: " << isFull << "\n";
    ss << "size: " << size() << ", size int: " << sizeInt() << "\n";
    std::cout << ss.str();
}

template <typename type, int capacity> std::vector<type> Channel<type, capacity>::extract_messages()
{
    size_t available = size();
    std::vector<type> v;

    if (available == 0)
    {
        return v;
    }
    else
    {
        while (size() > 0)
        {
            v.push_back(dequeue());
        }

        return v;
    }
}