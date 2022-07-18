/**
 * @file
 * This file is part of Pond.
 *
 * @author Alexander Pöppl (poeppl AT in.tum.de,
 * https://www5.in.tum.de/wiki/index.php/Alexander_P%C3%B6ppl,_M.Sc.)
 *
 * @section LICENSE
 *
 * Pond is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Pond is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Pond.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * @section DESCRIPTION
 *
 * TODO
 */

#include "orchestration/ActorOrchestrator.hpp"

#include "util/Configuration.hpp"
#include "util/Logger.hh"
#include <upcxx/upcxx.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <vector>

static tools::Logger &l = tools::Logger::logger;

ActorOrchestrator::ActorOrchestrator(const Configuration &config)
    : ag(), config(config), localActorCoords(), localActors(), remoteAO(this)
{
    // if (this->config==nullptr){
    //    throw std::runtime_error("ActorOrch config is nullptr");
    //}
    SimulationActor::configuration = &config;
}

std::string ActorOrchestrator::generateName(size_t x, size_t y)
{
    SimulationActor::configuration = &config;
    return std::to_string(x) /*+ "-" + std::to_string(x + 1)*/ + "-" +
           std::to_string(y) /*+ "-" + std::to_string(y + 1)+*/;
}

std::vector<std::string> ActorOrchestrator::generateNames()
{

    size_t xActors = config.xSize / config.patchSize;
    size_t yActors = config.ySize / config.patchSize;

    std::vector<std::string> actorNames;
    actorNames.reserve(xActors * yActors);

    for (size_t y = 0; y < yActors; y++)
    {
        for (size_t x = 0; x < xActors; x++)
        {
            std::string s = generateName(x, y);
            // if (!upcxx::rank_me())
            //  std::cout << s << std::endl;
            actorNames.push_back(std::move(s));
        }
    }

    return actorNames;
}

std::vector<std::tuple<size_t, size_t>> ActorOrchestrator::generateConstructorArguments()
{
    std::vector<std::tuple<size_t, size_t>> args;
    size_t xActors = config.xSize / config.patchSize;
    size_t yActors = config.ySize / config.patchSize;
    args.reserve(xActors * yActors);
    for (size_t y = 0; y < yActors; y++)
    {
        for (size_t x = 0; x < xActors; x++)
        {
            args.push_back({x, y});
        }
    }
    return args;
}

/*
    X--------X
    |        |
    |        |
    |        |
    X--------X

    X 4 special case
    | 2 special cases
    --- 2 speical cases
*/
std::vector<std::tuple<std::string, std::string, std::string, std::string>> ActorOrchestrator::generateConnections()
{
    SimulationActor::configuration = &config;
    size_t xActors = config.xSize / config.patchSize;
    size_t yActors = config.ySize / config.patchSize;

    std::vector<std::tuple<std::string, std::string, std::string, std::string>> connections;
    size_t connectionCount = 0;
    connectionCount += 8;
    connectionCount += (xActors + yActors - 4) * 2 * 3;
    connectionCount += (xActors * yActors - (xActors + yActors - 4)) * 4;
    connections.reserve(connectionCount);

    for (size_t y = 0; y < yActors; y++)
    {
        for (size_t x = 0; x < xActors; x++)
        {
            std::string self = generateName(x, y);
            if (x > 0)
            {
                std::string left = generateName((x - 1), y);

                std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_LEFT", left,
                                                                                      "BND_RIGHT"};
                connections.push_back(tup);
            }

            if (x < xActors - 1)
            {
                std::string right = generateName((x + 1), y);

                std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_RIGHT", right,
                                                                                      "BND_LEFT"};
                connections.push_back(tup);
            }

            if (y > 0)
            {
                std::string bottom = generateName(x, (y - 1));

                std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_BOTTOM", bottom,
                                                                                      "BND_TOP"};
                connections.push_back(tup);
            }

            if (y < yActors - 1)
            {
                std::string top = generateName(x, (y + 1));

                std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_TOP", top,
                                                                                      "BND_BOTTOM"};
                connections.push_back(tup);
            }
        }
    }
    return connections;
}

std::tuple<std::vector<idx_t>, std::vector<idx_t>> ActorOrchestrator::createSparseColRow()
{
    std::vector<idx_t> vertexAdjacencyListStarts;
    std::vector<idx_t> vertexAdjacencies;

    idx_t numAdjacencies = 0;
    SimulationActor::configuration = &config;
    size_t xSize = config.xSize / config.patchSize;
    size_t ySize = config.ySize / config.patchSize;

    for (size_t y = 0; y < ySize; y++)
    {
        for (size_t x = 0; x < xSize; x++)
        {
            vertexAdjacencyListStarts.push_back(numAdjacencies);
            int numNeighbors = 0;
            if (x > 0)
            {
                numNeighbors++;
                vertexAdjacencies.push_back(y * xSize + (x - 1));
            }

            if (x < xSize - 1)
            {
                numNeighbors++;
                vertexAdjacencies.push_back(y * xSize + (x + 1));
            }

            if (y > 0)
            {
                numNeighbors++;
                vertexAdjacencies.push_back((y - 1) * xSize + x);
            }

            if (y < ySize - 1)
            {
                numNeighbors++;
                vertexAdjacencies.push_back((y + 1) * xSize + x);
            }
            numAdjacencies += numNeighbors;
        }
    }
    vertexAdjacencyListStarts.push_back(numAdjacencies);

    return {vertexAdjacencyListStarts, vertexAdjacencies};
}

void ActorOrchestrator::initFromList()
{
    auto names = generateNames();
    auto conns = generateConnections();
    auto args = generateConstructorArguments();
    auto a = createSparseColRow();

    ag.initFromList<size_t, size_t>(std::move(names), std::move(conns), std::move(args));
    upcxx::barrier();

    auto acts = ag.getActors();
    int myrank = upcxx::rank_me();
    bool first = true;
    for (const std::pair<std::string, GlobalActorRef> pr : *acts)
    {
        GlobalActorRef aref = pr.second;
        if (aref.where() == myrank)
        {
            ActorImpl *a = *aref.local();
            auto pp = dynamic_cast<SimulationActor *>(a);
            if (pp)
            {
                localActors.push_back(pp);
            }
        }
    }
    upcxx::barrier();

    initializeActors();
    upcxx::barrier();
}

void ActorOrchestrator::initializeActors()
{
    float safeTimestep{std::numeric_limits<float>::infinity()};
    for (SimulationActor *a : localActors)
    {
        a->initializeBlock();
        auto blockSafeTs = a->getMaxBlockTimestepSize();
        safeTimestep = std::min(safeTimestep, blockSafeTs);
#ifndef NDEBUG
#if defined(REPORT_MAIN_ACTIONS)
        std::cout << "Safe timestep on actor " << a->getName() << " is " << blockSafeTs
                  << " current min: " << safeTimestep << std::endl;
#endif
#endif
    }
    float globalSafeTs = upcxx::reduce_all(safeTimestep, upcxx::experimental::op_min).wait();
#ifndef NDEBUG
#if defined(REPORT_MAIN_ACTIONS)
    l.cout() << "Received safe timestep: " << globalSafeTs << " local was " << safeTimestep << std::endl;
#endif
#endif
    for (SimulationActor *a : localActors)
    {
        a->setTimestepBaseline(globalSafeTs);
    }
}

void ActorOrchestrator::endSimulation(float runtime)
{
    uint64_t localPatchUpdates = 0;
    uint64_t totalPatchUpdates = 0;
    const std::unordered_map<std::string, GlobalActorRef> *map = ag.getActors();
    for (const auto &ref : *map)
    {
        if (ref.second.where() == upcxx::rank_me())
        {
            const ActorImpl *act = *(ref.second.local());
            const SimulationActor *sa = dynamic_cast<const SimulationActor *>(act);
            if (sa != nullptr)
            {
                localPatchUpdates += sa->getNumberOfPatchUpdates();
            }
            if (sa == nullptr)
            {
                throw std::runtime_error("No null pointers should be left by now!");
            }
        }
    }

    totalPatchUpdates = upcxx::reduce_all(localPatchUpdates, upcxx::experimental::op_add).wait();

    if (!upcxx::rank_me())
    {
        l.cout() << "Performed " << totalPatchUpdates << " patch updates in " << runtime << " seconds." << std::endl;
        l.cout() << "Performed " << (totalPatchUpdates * config.patchSize * config.patchSize) << " cell updates in "
                 << runtime << " seconds." << std::endl;
        l.cout() << "=> " << (static_cast<double>(totalPatchUpdates * config.patchSize * config.patchSize) / runtime)
                 << " CellUpdates/s" << std::endl;
        l.cout() << "=> "
                 << (static_cast<double>(135 * 2 * totalPatchUpdates * config.patchSize * config.patchSize) / runtime)
                 << " FlOps/s" << std::endl;
    }
}

inline void printbegin()
{ /*l.printString("********************* Start Simulation **********************", false);*/
}

inline void printend()
{ /*l.printString("********************** End Simulation ***********************", false);*/
}

void ActorOrchestrator::printInfo()
{
    std::cout << "From rank: " << upcxx::rank_me() << std::endl;
    std::cout << ag.prettyPrint() << std::endl;
}

void ActorOrchestrator::simulate()
{
    printbegin();
    auto runTime = ag.run();
    printend();
    endSimulation(runTime);
}

ActorOrchestrator::~ActorOrchestrator() {}

void ActorOrchestrator::gitterPrint()
{
    if (upcxx::rank_me())
    {
        return;
    }

    size_t actorcount = 100;
    // ag.getActorCount();
    size_t dimsize = std::sqrt(actorcount);

    std::stringstream ss;
    ss << "  X →\n";
    for (size_t y = 0; y < dimsize; y++)
    {
        if (y == 0)
        {
            ss << "Y ";
        }
        else if (y == 1)
        {
            ss << "\u2193 ";
        }
        else
        {
            ss << "  ";
        }

        for (size_t x = 0; x < dimsize; x++)
        {
            std::string s = std::to_string(x) + "-" + std::to_string(y);
            GlobalActorRef ref = ag.getActorNoThrow(s);

            int rank = -1;
            if (ref != nullptr)
            {
                rank = ref.where();
            }

            if (rank >= 100)
            {
                ss << " " << rank;
            }
            else if (rank >= 10)
            {
                ss << "  " << rank;
            }
            else
            {
                ss << "   " << rank;
            }
        }
        ss << "\n";
    }
    std::cout << ss.str() << std::endl;
}