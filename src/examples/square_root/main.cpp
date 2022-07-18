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

#include <string>

#include <upcxx/upcxx.hpp>

#include "actorlib/DynamicActorGraph.hpp"
#include "actorlib/MultipleActorAGraph.hpp"

#include "SinkActor.hpp"
#include "SourceActor.hpp"
#include "SquareRootActor.hpp"

using namespace std::string_literals;

int main(int argc, const char **argv)
{
    upcxx::init();

    if (upcxx::rank_n() != 3)
    {
        std::cout << "This example requires 3 ranks!" << std::endl;
    }
    else
    {
        MultipleActorAGraph<SinkActor, SourceActor, SquareRootActor> ag;
        ActorImpl *a;
        switch (upcxx::rank_me())
        {
        case 0:
            a = new SourceActor(-100.0, 100.0);
            break;
        case 1:
            a = new SquareRootActor;
            break;
        case 2:
            a = new SinkActor;
            break;
        default:
            a = nullptr;
            break;
        }

        ag.addActor(a);
        upcxx::barrier();
        if (!upcxx::rank_me())
        {
            auto source = ag.getActor("Source");
            auto sqrt = ag.getActor("SquareRoot");
            auto sink = ag.getActor("Sink");
            ag.connectPorts(source, "out", sqrt, "sourceIn");
            ag.connectPorts(sqrt, "selfOut", sqrt, "selfIn");
            ag.connectPorts(sqrt, "sinkOut", sink, "in");
        }
        ag.run();
    }

    upcxx::finalize();
}
