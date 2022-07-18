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
#include "Utility.hpp"
#include <memory>
#include <string>
#include <upcxx/global_ptr.hpp>

class ActorImpl;
class InPortInformationBase;

class TaskDeque;
class AbstractOutPort;

class AbstractInPort
{

  public:
    ActorImpl *connectedActor;
    std::string inActorName;
    const std::string name;
    unsigned int total = 0;

  public:
    virtual std::string toString() = 0; // same as str()
    virtual void deregister() = 0;      // deregisters connected actor
    virtual void registerWithChannel() = 0;
    virtual void applyPortInfo(std::unique_ptr<InPortInformationBase> &&inPortInformation) = 0;
    virtual std::unique_ptr<InPortInformationBase> generatePortInfo() = 0; // generates port info for serialization

    AbstractInPort(const std::string &name);
    virtual ~AbstractInPort();

    virtual std::string str() const; // returns name but can be overriden
    std::string getName() const;
    void triggerActor(const std::string &portName) const;
    std::string getActorName() const;

    virtual void setTaskDequeOfChannel(TaskDeque *tdq) = 0;
    virtual bool severeConnection() = 0;
    virtual bool resurrectConnection(upcxx::intrank_t rank, AbstractOutPort *outptr) = 0;
    virtual GlobalChannelRef getChannelPtr() const = 0;
    virtual void notify() = 0;
    virtual bool noNeedToNotify() = 0;
    virtual bool connected() = 0;
    virtual bool empty() = 0;
    virtual void print() const = 0;
    virtual size_t available() const = 0;
};
