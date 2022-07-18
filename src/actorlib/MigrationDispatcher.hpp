#pragma once

#include "AcquisitionResult.hpp"
#include "Utility.hpp"
#include <optional>
#include <string>
#include <upcxx/upcxx.hpp>

// This an empty to give access
// A Base class for Dynamic ActorGraph so that
class MigrationDispatcher
{
  public:
    virtual upcxx::future<GlobalActorRef> stealActorAsync(const std::string name) = 0;
    virtual upcxx::future<GlobalActorRef> offloadActorAsync(const std::string name, upcxx::intrank_t to) = 0;
    virtual upcxx::future<std::string> findAnActorToStealAndMark() = 0;
    virtual upcxx::future<std::vector<std::string>> findActorsToSteal() = 0;
    virtual upcxx::future<std::string, upcxx::intrank_t> findActorsToOffload() = 0;
    virtual upcxx::future<bool> tryToMarkActorForMigration(const std::string name) = 0;
    virtual upcxx::future<bool> tryStopSelfAndPinNeighborhood(const std::string name) = 0;
    virtual void print_gitter_active_status() = 0;
    MigrationDispatcher() = default;
    virtual ~MigrationDispatcher() = default;
};