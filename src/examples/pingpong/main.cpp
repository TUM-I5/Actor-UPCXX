/***
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

#include <string>

#include <upcxx/upcxx.hpp>

#include "PingPongActor.hpp"

#include "actorlib/ActorGraph.hpp"
#include "actorlib/ActorImpl.hpp"
#include <typeinfo>

#include "actorlib/DynamicActorGraph.hpp"
#include "actorlib/SingleActorAGraph.hpp"

using namespace std::string_literals;

std::string getNext(int myActorIdx, int actorsPerPlace)
{
    if (myActorIdx == actorsPerPlace - 1)
    {
        return "A-"s + std::to_string((upcxx::rank_me() + 1) % upcxx::rank_n()) + "-0"s;
    }
    else
    {
        return "A-"s + std::to_string(upcxx::rank_me()) + "-"s + std::to_string(myActorIdx + 1);
    }
}

std::string getName(int actorIdx) { return "A-"s + std::to_string(upcxx::rank_me()) + "-"s + std::to_string(actorIdx); }

int main(int argc, const char **argv)
{
    int actorsPerPlace = 6;
    /*if (argc == 2) {
        actorsPerPlace = std::stoi(argv[1]);
        if (actorsPerPlace < 1) {
            actorsPerPlace = 1;
        }
    }*/
    upcxx::init();
    SingleActorAGraph<PingPongActor> ag;
    for (int i = 0; i < actorsPerPlace; i++)
    {
        std::string name = "A-"s + std::to_string(upcxx::rank_me()) + "-"s + std::to_string(i);
        PingPongActor *a = ::new PingPongActor(name);
        ag.addActor(a);
        // std::cout << ag.getNumActors() << std::endl;
    }
    upcxx::barrier();
    // std::cout << ag.getNumActors() << " and my rank: " << upcxx::rank_me() << std::endl;
    for (int i = 0; i < actorsPerPlace; i++)
    {
        auto actorName = getName(i);
        auto nextActorName = getNext(i, actorsPerPlace);
        std::cout << "Connect " << actorName << " -> " << nextActorName << std::endl;
        auto actor = ag.getActor(actorName);
        auto nextActor = ag.getActor(nextActorName);
        ag.connectPorts(actor, PingPongActor::OUT_PORT_NAME, nextActor, PingPongActor::IN_PORT_NAME);
    }
    upcxx::barrier();

    for (int i = 0; i < actorsPerPlace; i++)
    {
        std::string name = "A-"s + std::to_string(upcxx::rank_me()) + "-"s + std::to_string(i);
        GlobalActorRef x = ag.getActor(name);
        ActorImpl *local_actor = *x.local();
        std::cout << local_actor->toString() << std::endl;
    }

    upcxx::barrier();

    if (upcxx::rank_me() == 1)
    {
        std::string str = ag.prettyPrint();
        std::cout << str << std::endl;
        // std::list<std::string> tprint = ag.inConnectedActors("A-0-0");
        std::cout << "ownernames for A-0-0 inports:";
        std::cout << std::endl;
        std::list<std::string> tpr;
        tpr.push_front("invalid");
        // tprint = ag.outConnectedActors("A-0-0").value_or(tpr);
        std::cout << "ownernames for A-0-0 outports:";

        std::cout << std::endl;

        // it works with rpc_ff
        // or no return value with rpc
        // or only outer rpc returning a val
        // broken when the inner thing retuns a value
        std::cout << "from rank0" << std::endl;
        bool x = upcxx::rpc(1,
                            []()
                            {
                                std::cout << "from rank1" << std::endl;
                                auto forget = upcxx::rpc(2, []() { std::cout << "to rank2" << std::endl; });
                                return false;
                            })
                     .wait();
        std::cout << x << std::endl;
    }

    upcxx::barrier();

    for (int i = 0; i < actorsPerPlace; i++)
    {
        std::string name = "A-"s + std::to_string(upcxx::rank_me()) + "-"s + std::to_string(i);
        GlobalActorRef x = ag.getActor(name);
        ActorImpl *local_actor = *x.local();
        std::cout << local_actor->toString() << std::endl;
    }
    upcxx::barrier();
    ag.run();
    upcxx::barrier();

    std::cout << "done!" << std::endl;
    upcxx::finalize();
}
