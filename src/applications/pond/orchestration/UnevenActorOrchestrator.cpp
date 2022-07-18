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

#include "orchestration/UnevenActorOrchestrator.hpp"

#include "util/Configuration.hpp"
#include "util/Logger.hh"
#include <algorithm>
#include <chrono>
#include <limits>
#include <upcxx/upcxx.hpp>
#include <vector>

static tools::Logger &l = tools::Logger::logger;

UnevenActorOrchestrator::UnevenActorOrchestrator(const Configuration &config)
    : ag(), config(config), localActorCoords(), localActors(), dhlocalActors(), remoteAO(this)
{
    // if (this->config==nullptr){
    //    throw std::runtime_error("ActorOrch config is nullptr");
    //}
    SimulationActor::configuration = &config;
    DoubleHeightSimulationActor::configuration = &config;
}

std::string UnevenActorOrchestrator::generateName(size_t x, size_t y)
{
    SimulationActor::configuration = &config;
    DoubleHeightSimulationActor::configuration = &config;

    return std::to_string(x) /*+ "-" + std::to_string(x + 1)*/ + "-" +
           std::to_string(y) /*+ "-" + std::to_string(y + 1)+*/;
}

std::vector<size_t> UnevenActorOrchestrator::generateTypeIndices()
{
    std::vector<size_t> types;
    auto sain = *util::typeIndex<0, SimulationActor, SimulationActor, DoubleHeightSimulationActor>();
    auto dhasin = *util::typeIndex<0, DoubleHeightSimulationActor, SimulationActor, DoubleHeightSimulationActor>();

    size_t xActorsNormal = config.xSize / config.patchSize;
    size_t xActorsDouble = ((config.xSize / (2 * config.patchSize)));
    size_t yActors = config.ySize / config.patchSize;
    for (size_t y = 0; y < yActors; y++)
    {
        if (y % 2 == 0)
        {
            for (size_t x = 0; x < xActorsNormal; x++)
            {
                types.push_back(sain);
            }
        }
        else
        {
            for (size_t x = 0; x < xActorsDouble; x++)
            {
                types.push_back(dhasin);
            }
        }
    }

    return types;
}

std::vector<std::string> UnevenActorOrchestrator::generateNames()
{

    size_t xActorsNormal = config.xSize / config.patchSize;

    size_t xActorsDouble = ((config.xSize / (2 * config.patchSize)));

    size_t yActors = config.ySize / config.patchSize;

    std::vector<std::string> actorNames;

    for (size_t y = 0; y < yActors; y++)
    {
        if (y % 2 == 0)
        {
            for (size_t x = 0; x < xActorsNormal; x++)
            {
                std::string s = generateName(x, y);
                // if (!upcxx::rank_me())
                //  std::cout << s << std::endl;
                actorNames.push_back(std::move(s));
            }
        }
        else
        {
            for (size_t x = 0; x < xActorsDouble; x++)
            {
                std::string s = generateName(x, y);
                // if (!upcxx::rank_me())
                //  std::cout << s << std::endl;
                actorNames.push_back(std::move(s));
            }
        }
    }

    return actorNames;
}

std::vector<std::tuple<size_t, size_t>> UnevenActorOrchestrator::generateConstructorArguments()
{
    std::vector<std::tuple<size_t, size_t>> args;
    size_t xActorsNormal = config.xSize / config.patchSize;

    size_t xActorsDouble = ((config.xSize / (2 * config.patchSize)));

    size_t yActors = config.ySize / config.patchSize;

    for (size_t y = 0; y < yActors; y++)
    {
        if (y % 2 == 0)
        {
            for (size_t x = 0; x < xActorsNormal; x++)
            {
                args.push_back({x, y});
            }
        }
        else
        {
            for (size_t x = 0; x < xActorsDouble; x++)
            {
                args.push_back({x, y});
            }
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
std::vector<std::tuple<std::string, std::string, std::string, std::string>>
UnevenActorOrchestrator::generateConnections()
{
    SimulationActor::configuration = &config;
    size_t yActors = config.ySize / config.patchSize;

    size_t xActorsNormal = config.xSize / config.patchSize;
    size_t xActorsDouble = config.xSize / (2 * config.patchSize);

    std::vector<std::tuple<std::string, std::string, std::string, std::string>> connections;

    for (size_t y = 0; y < yActors; y++)
    {
        if (y % 2 == 0)
        {

            for (size_t x = 0; x < xActorsNormal; x++)
            {
                std::string self = generateName(x, y);
                if (x > 0)
                {
                    std::string left = generateName((x - 1), y);

                    std::string tmp;
                    std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_LEFT", left,
                                                                                          "BND_RIGHT"};
                    connections.push_back(tup);
                }

                if (x < xActorsNormal - 1)
                {
                    std::string right = generateName((x + 1), y);

                    std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_RIGHT", right,
                                                                                          "BND_LEFT"};
                    connections.push_back(tup);
                }

                if (y > 0)
                {
                    std::string bottom = generateName(x / 2, (y - 1));
                    // even numbers get left uneven ones get right

                    std::string toptmp = (x % 2 == 0) ? "BND_TOP_LEFT" : "BND_TOP_RIGHT";
                    std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_BOTTOM", bottom,
                                                                                          toptmp};
                    connections.push_back(tup);
                }

                if (y < yActors - 1)
                {
                    std::string top = generateName(x / 2, (y + 1));

                    std::string bottmp = (x % 2 == 0) ? "BND_BOTTOM_LEFT" : "BND_BOTTOM_RIGHT";
                    std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_TOP", top, bottmp};
                    connections.push_back(tup);
                }
            }
        }
        else
        {
            // the actors in this line have double height
            for (size_t x = 0; x < xActorsDouble; x++)
            {
                std::string self = generateName(x, y);
                if (x > 0)
                {
                    std::string left = generateName((x - 1), y);

                    std::string tmp;
                    std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_LEFT", left,
                                                                                          "BND_RIGHT"};
                    connections.push_back(tup);
                }

                if (x < xActorsDouble - 1)
                {
                    std::string right = generateName((x + 1), y);

                    std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_RIGHT", right,
                                                                                          "BND_LEFT"};
                    connections.push_back(tup);
                }

                if (y > 0)
                {
                    std::string bottomleft = generateName(x * 2, y - 1);
                    std::string bottomright = generateName(x * 2 + 1, y - 1);
                    // even numbers get left uneven ones get right

                    std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_BOTTOM_LEFT",
                                                                                          bottomleft, "BND_TOP"};
                    connections.push_back(tup);
                    tup = {self, "BND_BOTTOM_RIGHT", bottomright, "BND_TOP"};
                    connections.push_back(tup);
                }

                if (y < yActors - 1)
                {
                    std::string topleft = generateName(x * 2, y + 1);
                    std::string topright = generateName(x * 2 + 1, y + 1);

                    std::tuple<std::string, std::string, std::string, std::string> tup = {self, "BND_TOP_LEFT", topleft,
                                                                                          "BND_BOTTOM"};
                    connections.push_back(tup);
                    tup = {self, "BND_TOP_RIGHT", topright, "BND_BOTTOM"};
                    connections.push_back(tup);
                }
            }
        }
    }
    return connections;
}

size_t UnevenActorOrchestrator::getOffset(size_t x, size_t y, size_t normal, size_t twice)
{
    size_t offset = 0;
    for (size_t iy = 0; iy < y; iy++)
    {
        if (iy % 2 == 0)
        {
            offset += normal;
        }
        else
        {
            offset += twice;
        }
    }
    offset += x;
    return offset;
}

std::tuple<std::vector<idx_t>, std::vector<idx_t>> UnevenActorOrchestrator::createSparseColRow()
{
    std::vector<idx_t> vertexAdjacencyListStarts;
    std::vector<idx_t> vertexAdjacencies;

    idx_t numAdjacencies = 0;
    SimulationActor::configuration = &config;
    size_t xActorsNormal = config.xSize / config.patchSize;
    size_t xActorsDouble = config.xSize / (2 * config.patchSize);
    size_t ySize = config.ySize / config.patchSize;

    for (size_t y = 0; y < ySize; y++)
    {
        if (y % 2 == 0)
        {
            for (size_t x = 0; x < xActorsNormal; x++)
            {
                vertexAdjacencyListStarts.push_back(numAdjacencies);
                int numNeighbors = 0;
                size_t self = getOffset(x, y, xActorsNormal, xActorsDouble);
                if (x > 0)
                {
                    numNeighbors++;
                    vertexAdjacencies.push_back(self - 1);
                }

                if (x < xActorsNormal - 1)
                {
                    numNeighbors++;
                    vertexAdjacencies.push_back(self + 1);
                }

                if (y > 0)
                {
                    numNeighbors++;
                    vertexAdjacencies.push_back(getOffset(y - 1, x / 2, xActorsNormal, xActorsDouble));
                }

                if (y < ySize - 1)
                {
                    numNeighbors++;
                    vertexAdjacencies.push_back(getOffset(y + 1, x / 2, xActorsNormal, xActorsDouble));
                }
                numAdjacencies += numNeighbors;
            }
        }
        else
        {
            for (size_t x = 0; x < xActorsDouble; x++)
            {
                vertexAdjacencyListStarts.push_back(numAdjacencies);
                int numNeighbors = 0;
                size_t self = getOffset(x, y, xActorsNormal, xActorsDouble);
                if (x > 0)
                {
                    numNeighbors++;
                    vertexAdjacencies.push_back(self - 1);
                }

                if (x < xActorsDouble - 1)
                {
                    numNeighbors++;
                    vertexAdjacencies.push_back(self + 1);
                }

                if (y > 0)
                {
                    numNeighbors += 2;
                    size_t ll = getOffset(y - 1, x * 2, xActorsNormal, xActorsDouble);
                    vertexAdjacencies.push_back(ll);
                    vertexAdjacencies.push_back(ll + 1);
                }

                if (y < ySize - 1)
                {
                    numNeighbors += 2;
                    size_t rl = getOffset(y + 1, x * 2, xActorsNormal, xActorsDouble);
                    vertexAdjacencies.push_back(rl);
                    vertexAdjacencies.push_back(rl + 1);
                }
                numAdjacencies += numNeighbors;
            }
        }
    }
    vertexAdjacencyListStarts.push_back(numAdjacencies);

    return {vertexAdjacencyListStarts, vertexAdjacencies};
}

void UnevenActorOrchestrator::initFromList()
{
    auto names = generateNames();
    auto conns = generateConnections();
    auto args = generateConstructorArguments();
    auto types = generateTypeIndices();
    auto a = createSparseColRow();

    ag.initFromList<size_t, size_t>(std::move(names), std::move(conns), std::move(types), std::move(args));
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
            auto dhpp = dynamic_cast<DoubleHeightSimulationActor *>(a);
            if (dhpp)
            {
                dhlocalActors.push_back(dhpp);
            }
        }
    }
    upcxx::barrier();

    initializeActors();
    upcxx::barrier();

    // broadcastActiveToNeighbors();
    // upcxx::barrier();
}

void UnevenActorOrchestrator::initializeActors()
{
    float safeTimestep{std::numeric_limits<float>::infinity()};
    for (SimulationActor *a : localActors)
    {
        a->initializeBlock();
        auto blockSafeTs = a->getMaxBlockTimestepSize();
        safeTimestep = std::min(safeTimestep, blockSafeTs);
#ifndef NDEBUG
        std::cout << "Safe timestep on actor " << a->getName() << " is " << blockSafeTs
                  << " current min: " << safeTimestep << std::endl;
#endif
    }
    for (DoubleHeightSimulationActor *a : dhlocalActors)
    {
        a->initializeBlock();
        auto blockSafeTs = a->getMaxBlockTimestepSize();
        safeTimestep = std::min(safeTimestep, blockSafeTs);
#ifndef NDEBUG
        std::cout << "Safe timestep on actor " << a->getName() << " is " << blockSafeTs
                  << " current min: " << safeTimestep << std::endl;
#endif
    }
    float globalSafeTs = upcxx::reduce_all(safeTimestep, upcxx::experimental::op_min).wait();
#ifndef NDEBUG
    l.cout() << "Received safe timestep: " << globalSafeTs << " local was " << safeTimestep << std::endl;
#endif
    for (DoubleHeightSimulationActor *a : dhlocalActors)
    {
        a->setTimestepBaseline(globalSafeTs);
    }
    for (SimulationActor *a : localActors)
    {
        a->setTimestepBaseline(globalSafeTs);
    }
}

void UnevenActorOrchestrator::endSimulation(float runtime)
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
            if (sa)
            {
                localPatchUpdates += sa->getNumberOfPatchUpdates();
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

inline void printbegin() { l.printString("********************* Start Simulation **********************", false); }

inline void printend() { l.printString("********************** End Simulation ***********************", false); }

void UnevenActorOrchestrator::printInfo()
{
    std::cout << "From rank: " << upcxx::rank_me() << std::endl;
    std::cout << ag.prettyPrint() << std::endl;
}

void UnevenActorOrchestrator::simulate()
{
    printbegin();
    auto runTime = ag.run();
    printend();
    endSimulation(runTime);
}

UnevenActorOrchestrator::~UnevenActorOrchestrator() {}
