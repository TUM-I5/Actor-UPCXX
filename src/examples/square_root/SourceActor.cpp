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

#include "SourceActor.hpp"

#include "actorlib/OutPort.hpp"

#include <iostream>

SourceActor::SourceActor(double minNumber, double maxNumber)
    : ActorImpl("Source"), minNumber(minNumber), maxNumber(maxNumber), rd(), generator(rd()), dist(minNumber, maxNumber)
{
    op = makeOutPort<double, 5>("out");
}

/*
SourceActor::SourceActor(const SourceActor &other)
    : ActorImpl("Source"), minNumber(other.minNumber), maxNumber(other.maxNumber), rd(), generator(rd()),
      dist(minNumber, maxNumber)
{
}
*/

SourceActor::SourceActor(SourceActor &&other)
    : ActorImpl("Source"), minNumber(other.minNumber), maxNumber(other.maxNumber), rd(), generator(rd()),
      dist(minNumber, maxNumber)
{
}

double SourceActor::getNext() { return dist(generator); }

void SourceActor::act()
{
    if (op->freeCapacity() > 0)
    {
        auto randomNumber = getNext();
        std::cout << "Generated number: <<" << randomNumber << ">>" << std::endl;
        op->write(std::move(randomNumber));
    }
}
