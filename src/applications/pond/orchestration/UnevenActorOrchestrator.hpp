#include "OrchestrationUtility.hpp"
#include "actor/DoubleHeightSimulationActor.hpp"
#include "actor/SimulationActor.hpp"
#include "actorlib/ActorGraph.hpp"
#include "actorlib/MultipleActorAGraph.hpp"
#include "actorlib/Utility.hpp"
#include "util/Configuration.hpp"
#include <upcxx/upcxx.hpp>

#pragma once

class UnevenActorOrchestrator
{
  private:
    MultipleActorAGraph<SimulationActor, DoubleHeightSimulationActor> ag;
    const Configuration &config;
    std::vector<std::pair<size_t, size_t>> localActorCoords;
    std::vector<SimulationActor *> localActors;               // this variable is not updated when an actor is migrated
    std::vector<DoubleHeightSimulationActor *> dhlocalActors; // this variable is not updated when an actor is migrated
    upcxx::dist_object<UnevenActorOrchestrator *> remoteAO;

  public:
    UnevenActorOrchestrator(const Configuration &config);
    ~UnevenActorOrchestrator();
    void simulate();
    void printInfo();
    // void broadcastActiveToNeighbors();
    std::string generateName(size_t x, size_t y);
    std::vector<std::string> generateNames();
    std::vector<size_t> generateTypeIndices();
    std::vector<std::tuple<size_t, size_t>> generateConstructorArguments();
    std::vector<std::tuple<std::string, std::string, std::string, std::string>> generateConnections();
    void initFromList();
    std::tuple<std::vector<idx_t>, std::vector<idx_t>> createSparseColRow();

  private:
    void initializeActors();
    void endSimulation(float runtime);
    size_t getOffset(size_t x, size_t y, size_t normal, size_t twice);
};