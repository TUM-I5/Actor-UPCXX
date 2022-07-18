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
#include <optional>
#include <string>
#include <upcxx/upcxx.hpp>

class ActorImpl;
class OutPortInformationBase;

class AbstractOutPort
{
  public:
    ActorImpl *connectedActor;
    const std::string name;

  public:
    virtual void setChannel(GlobalChannelRef channel) = 0; // set connected channel (Channel is a part of inport)
    virtual std::string toString() = 0;                    // get name
    virtual upcxx::future<> registerWithChannel(GlobalChannelRef ref) = 0; // sets channel
    virtual upcxx::future<> deregister() = 0; // removes channel ptr (Still it is constructed with InPort)
    // assumes that the input is OutPortInformation<T,cap>
    // for an OutPort<T,cap>
    virtual void applyPortInfo(std::unique_ptr<OutPortInformationBase>
                                   &&outPortInformation) = 0; // changes the OutPort state, to describe in the ptr
    virtual std::unique_ptr<OutPortInformationBase> generatePortInfo() = 0; // generated state of the OutPort

    AbstractOutPort(const std::string &name);
    virtual ~AbstractOutPort();
    std::string getName() const;                        // return base name
    virtual std::string str() const;                    // returns string, similar to getName but constant
    virtual bool messageOnWire() const = 0;
    void triggerActor() const;
    virtual bool severeConnection() = 0;
    virtual bool resurrectConnection(GlobalChannelRef rptr) = 0;
    virtual size_t bufferSize() const = 0;
    virtual void writeBuffer() const = 0;
    virtual bool connected() const = 0;
};
