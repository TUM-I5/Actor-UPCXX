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

#pragma once

class SinkActor : public ActorImpl
{
  private:
    InPort<double, 5> *ip;

  public:
    SinkActor();
    SinkActor(const SinkActor &other) = delete;
    SinkActor(SinkActor &&other);
    void act();

    // this will not be a valid serialization, just to compile
    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, SinkActor const &object)
        {
            throw std::runtime_error("This example is not to be tested with migration enabled!");
            writer.write(true);
        }

        template <typename Reader> static SinkActor *deserialize(Reader &reader, void *storage)
        {
            throw std::runtime_error("This example is not to be tested with migration enabled!");

            bool tmp = reader.template read<bool>();
            SinkActor *v = ::new (storage) SinkActor();
            return v;
        }
    };
};
