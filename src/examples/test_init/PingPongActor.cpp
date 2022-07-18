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

PingPongActor::PingPongActor(PingPongActor &&ppa)
    : ActorImpl(dynamic_cast<ActorImpl &&>(std::move(ppa))), nomadiness(ppa.nomadiness + 1),
      ip(makeInPort<std::vector<size_t>, 10>("IN")), op(makeOutPort<std::vector<size_t>, 10>("OUT")), begin(ppa.begin)
{
}

PingPongActor::PingPongActor(std::string &&name)
    : ActorImpl(std::move(name)), nomadiness(0), ip(makeInPort<std::vector<size_t>, 10>("IN")),
      op(makeOutPort<std::vector<size_t>, 10>("OUT")), begin(true)
{
}

PingPongActor::PingPongActor(ActorData &&ad)
    : ActorImpl(std::move(ad)), ip(makeInPort<std::vector<size_t>, 10>("IN")),
      op(makeOutPort<std::vector<size_t>, 10>("OUT")), begin(true)
{
}

PingPongActor::PingPongActor(std::string &&name, int o, int s, int t)
    : ActorImpl(std::move(name)), ip(makeInPort<std::vector<size_t>, 10>("IN")),
      op(makeOutPort<std::vector<size_t>, 10>("OUT")), begin(true)
{
}

PingPongActor::PingPongActor(std::string &&name, std::variant<std::monostate> &&emtpyvar)
    : ActorImpl(std::move(name)), nomadiness(0), ip(makeInPort<std::vector<size_t>, 10>("IN")),
      op(makeOutPort<std::vector<size_t>, 10>("OUT")), begin(true)
{
}

PingPongActor::PingPongActor(const std::string &name, const std::variant<std::monostate> &emtpyvar)
    : ActorImpl(name), nomadiness(0), ip(makeInPort<std::vector<size_t>, 10>("IN")),
      op(makeOutPort<std::vector<size_t>, 10>("OUT")), begin(true)
{
}

PingPongActor::~PingPongActor() {}

void PingPongActor::act()
{
    std::cout << getName() << " act() has been called."
              << " OP cap: " << op->freeCapacity() << " IP avail: " << ip->available() << std::endl;
    if (ip->available() > 0 && op->freeCapacity() > 0)
    {
        auto data = ip->read();
        std::cout << getName() << " received " << data[0] << std::endl;
        data[0]++;
        op->write(std::move(data));
        std::cout << getName() << " sent " << data[0] << std::endl;
    }
    else if (getName().compare("A-0-0") == 0 && this->begin)
    {
        std::vector<size_t> data{0};
        this->begin = false;
        op->write(std::move(data));
        std::cout << getName() << " sent " << data[0] << std::endl;
    }
}