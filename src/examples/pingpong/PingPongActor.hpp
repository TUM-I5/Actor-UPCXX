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

#include "actorlib/ActorData.hpp"
#include "actorlib/ActorImpl.hpp"
#include <string>
#include <variant>

class PingPongActor : public ActorImpl
{

  public:
    constexpr static int type = 1;
    static std::string IN_PORT_NAME;
    static std::string OUT_PORT_NAME;
    // add back const

  private:
    InPort<size_t, 10> *ip;
    OutPort<size_t, 10> *op;
    bool begin;
    PingPongActor(

        bool isRunning,
        // std::atomic<int> triggerCount, -> to be reconstructed
        std::string name,
        // std::string IN_PORT_NAME, is static
        // std::string OUT_PORT_NAME, is static
        // std::string ActorTypeName, is static
        std::string DynamicActorTypeName,
        // InPort<size_t, 10> *ip; -> must be reconstructed
        // OutPort<size_t, 10> *op; -> must be reconstructed
        bool begin, int triggercountval);

  public:
    PingPongActor();
    PingPongActor(std::string &&name, std::variant<std::monostate> &&emptyVar);
    PingPongActor(std::string name);
    PingPongActor(const PingPongActor &other) = delete;
    PingPongActor(PingPongActor &&other);
    PingPongActor(ActorData &&ad);
    //<- dont use rvalue reference
    // right now the implementation does not work
    void act() final;

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, PingPongActor const &object)
        {
            std::cout << "serializing pingpongactor" << std::endl;
            writer.write(static_cast<ActorImpl const &>(object));
            writer.write(object.IN_PORT_NAME);
            writer.write(object.OUT_PORT_NAME);
            writer.write(object.begin);
            return;
        }

        template <typename Reader> static PingPongActor *deserialize(Reader &reader, void *storage)
        {
            std::cout << "deserializing pingpongactor" << std::endl;
            ActorData adata = reader.template read<ActorImpl>();
            std::string inportname = reader.template read<std::string>();
            std::string outportname = reader.template read<std::string>();
            bool begun = reader.template read<bool>();
            PingPongActor *v = ::new (storage) PingPongActor(std::move(adata));
            v->IN_PORT_NAME = inportname;
            v->OUT_PORT_NAME = outportname;
            v->begin = begun;
            return v;
        }
    };
};
