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

#include "OrchestrationUtility.hpp"
#include "actor/SimulationActor.hpp"
#include "actorlib/SingleActorAGraph.hpp"
#include "actorlib/Utility.hpp"
#include "util/Configuration.hpp"
#include <upcxx/upcxx.hpp>

#pragma once

class ActorOrchestrator
{
  private:
    SingleActorAGraph<SimulationActor> ag;
    const Configuration &config;
    std::vector<std::pair<size_t, size_t>> localActorCoords;
    std::vector<SimulationActor *> localActors; // this variable is not updated when an actor is migrated
    upcxx::dist_object<ActorOrchestrator *> remoteAO;

  public:
    ActorOrchestrator(const Configuration &config);
    ~ActorOrchestrator();
    void simulate();
    void printInfo();
    // void broadcastActiveToNeighbors();
    std::string generateName(size_t x, size_t y);
    std::vector<std::string> generateNames();
    std::vector<std::tuple<size_t, size_t>> generateConstructorArguments();
    std::vector<std::tuple<std::string, std::string, std::string, std::string>> generateConnections();
    void initFromList();
    std::tuple<std::vector<idx_t>, std::vector<idx_t>> createSparseColRow();
    void gitterPrint();

  private:
    void initializeActors();
    void endSimulation(float runtime);
};
