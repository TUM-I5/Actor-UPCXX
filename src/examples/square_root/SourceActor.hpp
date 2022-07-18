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

#include "actorlib/ActorImpl.hpp"

#include <random>

#pragma once

class SourceActor : public ActorImpl
{
  public:
    const double minNumber;
    const double maxNumber;
    OutPort<double, 5> *op;

  private:
    std::random_device rd;
    std::mt19937_64 generator;
    std::uniform_real_distribution<double> dist;

  private:
    double getNext();

  public:
    SourceActor(double minNumber, double maxNumber);
    // SourceActor(const SourceActor &other);
    SourceActor(SourceActor &&other);
    void act();

    // this will not be a valid serialization, just to compile
    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, SourceActor const &object)
        {
            throw std::runtime_error("This example is not to be tested with migration enabled!");
            writer.write(true);
        }

        template <typename Reader> static SourceActor *deserialize(Reader &reader, void *storage)
        {
            throw std::runtime_error("This example is not to be tested with migration enabled!");
            bool tmp = reader.template read<bool>();
            SourceActor *v = ::new (storage) SourceActor(0.0, 1.0);
            return v;
        }
    };
};
