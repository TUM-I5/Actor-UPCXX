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

enum class SquareRootActorState : bool
{
    DEFAULT = false,
    RECEIVED_NEGATIVE = true
};

class SquareRootActor : public ActorImpl
{
  public:
    InPort<double, 5> *sourceIn;
    InPort<double, 5> *selfIn;
    OutPort<double, 5> *sinkOut;
    OutPort<double, 5> *selfOut;

  private:
    SquareRootActorState currentState;

    void computeSquareRootFromSource();
    void computeSquareRootFromSelf();
    void computeAbsolute();

  public:
    SquareRootActor();
    // SquareRootActor(const SquareRootActor &other);
    SquareRootActor(SquareRootActor &&other);
    void act();

    // this will not be a valid serialization, just to compile
    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, SquareRootActor const &object)
        {
            throw std::runtime_error("This example is not to be tested with migration enabled!");
            writer.write(true);
        }

        template <typename Reader> static SquareRootActor *deserialize(Reader &reader, void *storage)
        {
            throw std::runtime_error("This example is not to be tested with migration enabled!");
            bool tmp = reader.template read<bool>();
            SquareRootActor *v = ::new (storage) SquareRootActor();
            return v;
        }
    };
};
