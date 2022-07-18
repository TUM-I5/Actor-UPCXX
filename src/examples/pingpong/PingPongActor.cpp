/**
 * @file
 * This file is part of the actorlib sample collection.
 *
 * @author Alexander Pöppl (poeppl AT in.tum.de,
 * https://www5.in.tum.de/wiki/index.php/Alexander_P%C3%B6ppl,_M.Sc.)
 *
 * @section LICENSE
 *
 * The actorlib sample collection is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * The actorlib sample collection is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The actorlib sample collection.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 *
 * @section DESCRIPTION
 *
 * TODO
 */

#include "PingPongActor.hpp"

#include "actorlib/InPort.hpp"
#include "actorlib/OutPort.hpp"

std::string PingPongActor::IN_PORT_NAME = "IN";
std::string PingPongActor::OUT_PORT_NAME = "OUT";

/*
PingPongActor::PingPongActor(const PingPongActor &ppa)
    : ActorImpl(*dynamic_cast<const ActorImpl *>(&ppa)), ip(makeInPort<size_t, 10>(IN_PORT_NAME)),
      op(makeOutPort<size_t, 10>(OUT_PORT_NAME)), begin(ppa.begin)
{
}
*/

PingPongActor::PingPongActor(PingPongActor &&ppa)
    : ActorImpl(std::move(*dynamic_cast<ActorImpl *>(&ppa))), ip(makeInPort<size_t, 10>(IN_PORT_NAME)),
      op(makeOutPort<size_t, 10>(OUT_PORT_NAME)), begin(ppa.begin)
{
}

PingPongActor::PingPongActor(std::string _name)
    : ActorImpl(_name), ip(makeInPort<size_t, 10>(IN_PORT_NAME)), op(makeOutPort<size_t, 10>(OUT_PORT_NAME)),
      begin(true)
{
}

PingPongActor::PingPongActor(ActorData &&ad)
    : ActorImpl(std::move(ad)), ip(makeInPort<size_t, 10>(IN_PORT_NAME)), op(makeOutPort<size_t, 10>(OUT_PORT_NAME)),
      begin(true)
{
}

PingPongActor::PingPongActor()
    : ActorImpl("stubname"), ip(makeInPort<size_t, 10>(IN_PORT_NAME)), op(makeOutPort<size_t, 10>(OUT_PORT_NAME)),
      begin(true)
{
}

void PingPongActor::act()
{
    // std::cout << name << " act() has been called." << " OP cap: " <<
    // op->freeCapacity() << " IP avail: " << ip->available() << std::endl;
    if (ip->available() > 0 && op->freeCapacity() > 0)
    {
        auto data = ip->read();
        // std::cout << name << " received " << data << std::endl;
        data++;
        op->write(std::move(data));
        // std::cout << name << " sent " << data << std::endl;
    }
    else if (this->getName().compare("A-0-0") == 0 && this->begin)
    {
        size_t data = 0;
        this->begin = false;
        op->write(std::move(data));
        // std::cout << name << " sent " << data << std::endl;
    }
}

PingPongActor::PingPongActor(std::string &&name, std::variant<std::monostate> &&emptyVar)
    : ActorImpl(std::move(name)), ip(makeInPort<size_t, 10>(IN_PORT_NAME)), op(makeOutPort<size_t, 10>(OUT_PORT_NAME)),
      begin(true)
{
}