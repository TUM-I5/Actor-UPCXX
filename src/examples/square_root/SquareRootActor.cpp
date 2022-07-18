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

#include "SquareRootActor.hpp"

#include "actorlib/InPort.hpp"
#include "actorlib/OutPort.hpp"

SquareRootActor::SquareRootActor() : ActorImpl("SquareRoot"), currentState(SquareRootActorState::DEFAULT)
{
    sourceIn = makeInPort<double, 5>("sourceIn");
    selfIn = makeInPort<double, 5>("selfIn");
    sinkOut = makeOutPort<double, 5>("sinkOut");
    selfOut = makeOutPort<double, 5>("selfOut");
}

/*
SquareRootActor::SquareRootActor(const SquareRootActor &other)
    : ActorImpl("SquareRoot"), currentState(other.currentState)
{
    sourceIn = makeInPort<double, 5>("sourceIn");
    selfIn = makeInPort<double, 5>("selfIn");
    sinkOut = makeOutPort<double, 5>("sinkOut");
    selfOut = makeOutPort<double, 5>("selfOut");
}
*/

SquareRootActor::SquareRootActor(SquareRootActor &&other) : ActorImpl("SquareRoot"), currentState(other.currentState)
{
    sourceIn = makeInPort<double, 5>("sourceIn");
    selfIn = makeInPort<double, 5>("selfIn");
    sinkOut = makeOutPort<double, 5>("sinkOut");
    selfOut = makeOutPort<double, 5>("selfOut");
}

void SquareRootActor::act()
{
    std::cout << "SquareRootActor::act() called with\n\t- |selfIn| = " << selfIn->available()
              << "\n\t- |sourceIn| = " << sourceIn->available() << "\n\t- |selfOut| = " << selfOut->freeCapacity()
              << "\n\t- |sinkOut| = " << sinkOut->freeCapacity();
    switch (this->currentState)
    {
    case SquareRootActorState::DEFAULT:
        if (sourceIn->available() > 0 && sourceIn->peek() >= 0.0 && sinkOut->freeCapacity() > 0)
        {
            std::cout << "\n\t- Peek = " << sourceIn->peek() << "(positive)";
            computeSquareRootFromSource();
        }
        else if (sourceIn->available() > 0 && sourceIn->peek() < 0.0 && selfOut->freeCapacity() > 0)
        {
            std::cout << "\n\t- Peek = " << sourceIn->peek() << "(negative)";
            computeAbsolute();
        }
        break;
    case SquareRootActorState::RECEIVED_NEGATIVE:
        if (selfIn->available() > 0 && sinkOut->freeCapacity() > 0)
        {
            std::cout << "\n\t- Peek = " << selfIn->peek();
            computeSquareRootFromSelf();
        }
        break;
    }
}

void SquareRootActor::computeAbsolute()
{
    std::cout << "\n\t- Perform action computeAbsolute" << std::endl;
    auto val = sourceIn->read();
    auto absVal = std::fabs(val);
    currentState = SquareRootActorState::RECEIVED_NEGATIVE;
    selfOut->write(std::move(absVal));
}

void SquareRootActor::computeSquareRootFromSelf()
{
    std::cout << "\n\t- Perform action computeSquareRootFromSelf" << std::endl;
    auto val = selfIn->read();
    auto sqrtVal = std::sqrt(val);
    currentState = SquareRootActorState::DEFAULT;
    sinkOut->write(std::move(sqrtVal));
}

void SquareRootActor::computeSquareRootFromSource()
{
    std::cout << "\n\t- Perform action computeSquareRootFromSource" << std::endl;
    auto val = sourceIn->read();
    auto sqrtVal = std::sqrt(val);
    sinkOut->write(std::move(sqrtVal));
}
