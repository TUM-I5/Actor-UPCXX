#pragma once
#include "AcquisitionResult.hpp"
#include "ActorImpl.hpp"
#include "ActorState.hpp"
#include "ActorTriggers.hpp"
#include "TerminationWave.hpp"
#include "Utility.hpp"
#include "config.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <ctime>
#include <deque>
#include <functional>
#include <iomanip>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <shared_mutex>
#include <stdlib.h>
#include <string>
#include <thread>
#include <upcxx/upcxx.hpp>

class ActorGraph;
class AbstractInPort;

constexpr size_t buflen = 64;

class TaskDeque
{
  public:
    // needed to access some information of the parent class
    ActorGraph *parentActorGraph;

    // dist object
    upcxx::dist_object<TaskDeque *> remoteTaskque;

    // members regarding termination
    TerminationWave tw;
    bool try_to_terminate = false;

    // members regarding number tracking and statistics
    double timeInAct = 0.0;
    double timeInProgress = 0.0;
    double timeInDischarge = 0.0;
    double timeInBarrier = 0.0;
    double timeCheckingTimeout = 0.0;
    double timeCheckingTermination = 0.0;
    double timeForcedTermination = 0.0;
    size_t tries = 0;
    size_t stole = 0;
    size_t tried_to_steal_remotely_rejected = 0;
    size_t tried_to_steal_locally_rejected = 0;
    size_t termtrial = 0;
    size_t execs = 0;
    std::chrono::_V2::steady_clock::time_point computationBegin;
    bool computationBeginSet = false;
    double timeOutside = 0.0;
    std::chrono::_V2::steady_clock::time_point goingOutside;
    std::chrono::_V2::steady_clock::time_point lastWorkloadExchange;

    // rng
    std::mt19937 randomEngine;

    bool during_migration = false;
    bool during_victim_search = false;
    bool cant_terminate = false;

    size_t consecutiveTerminationChecks = 0;
    size_t consecutiveStealTries = 0;
    bool forcedTermination = false;
    bool unmarked_and_unpinned = false;

    std::vector<std::string> buffered_victims{};
    bool find_new_victims = true;

    bool computing_temrination_result = false;
    bool termination_result = false;
    bool steal_cooldown = false;
    bool checkpoint_output = false;
    bool checkpoint_barrier = false;
    double timeout = -1.0;

    bool rma_taskcount = false;
    upcxx::future<bool> _stole;

    // Member variables regarding node slowdown
    bool with_slowdown = false;
    int slowdown_begin_at_rank = -1;
    int slowdown_end_at_rank = -1;
    double slowdown_from = -1.0;
    double slowdown_to = -1.0;

#ifdef USE_ACTOR_TRIGGERS
    // actor triggers
    std::list<std::pair<std::string, std::unique_ptr<ActorTriggers>>> actorTriggers;
    std::list<std::pair<std::string, std::unique_ptr<ActorTriggers>>>::iterator atIterator;
    bool actorTriggersChanged = false;
    size_t taskCount = 0;
#endif

    bool print_gitter = false;
    bool steal_during_termination = false;

    std::chrono::_V2::steady_clock::time_point lastStealTime;
    std::chrono::_V2::steady_clock::time_point lastStealTryTime;

#ifdef MORE_LOCAL_VICTIM_CHOICE
    bool refresh_victim_list = true;
    std::list<std::string> victim_list{};
#endif
  public:
    TaskDeque(ActorGraph *ag);
    ~TaskDeque() = default;
    bool advance(double timeout = -1.0);
    bool nothingIsComing() const;
    bool canExecute(const std::string &name) const;
    bool canWork() const;

#ifdef USE_ACTOR_TRIGGERS
    ActorTriggers moveTriggers(const std::string &name);
    void addTrigger(ActorTriggers &&att);
    void addTrigger(const std::string &actorName, const std::string &portName);
    upcxx::future<> sendTriggers(const std::unordered_map<std::string, upcxx::intrank_t> &migList);
#endif

    unsigned int getTaskCount() const;

  private:
    upcxx::future<> steal(const std::string nameOfActorToSteal);
    bool checkTermination();
    bool checkTimeout(std::chrono::_V2::steady_clock::time_point &begin, double timeout);
    void executeActor(const std::string &name);
    upcxx::future<std::string> findVictim();
    upcxx::future<bool> tryToLockVictimAndItsNeighbors(const std::string name);
    upcxx::future<bool> tryToSteal(upcxx::future<bool> stopped, const std::string name);
    void try_steal();
    void checkForcedTermination();
    void try_offload();
    upcxx::future<std::string, upcxx::intrank_t> findActorToOffload();
    upcxx::future<bool> tryToOffload(upcxx::future<bool> canSend, std::string name, upcxx::intrank_t to);
    upcxx::future<GlobalActorRef> offload(const std::string nameOfActorToOffload, const upcxx::intrank_t to);
    bool abort();
    void check_abort(std::chrono::_V2::steady_clock::time_point &begin);
};
