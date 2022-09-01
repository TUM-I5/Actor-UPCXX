/**
 * @file
 * This file is part of actorlib.
 *
 * @author Alexander Pöppl (poeppl AT in.tum.de,
 * https://www5.in.tum.de/wiki/index.php/Alexander_P%C3%B6ppl,_M.Sc.)
 *
 * @section LICENSE
 *
 * actorlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * actorlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with actorlib.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * @section DESCRIPTION
 *
 * TODO
 */

#pragma once

#include "Actor.hpp"
#include "ActorImpl.hpp"
#include "MigrationDispatcher.hpp"
#include "Utility.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <upcxx/upcxx.hpp>
// this needs to be included so that it compiles and it is used to force
// intel compiler to use specification seriaization for optional class
#include "ActorState.hpp"
#include "Channel.hpp"
#include "PortGraph.hpp"
#include "SerializeOptional.hpp"
#include "TaskDeque.hpp"
#include <set>

#ifdef TRACE
#include <VT.h>
#endif

/*
  ActorGraph regulates and executes actors, which is the part of the initial Actorlib with Migration
  DynamicActorGraph is an wrapper around ACtorGraph with additional functionality, where the connections
  between actors are saved etc. etc. (use DynamicActorGraph)
*/

class ActorGraph
{
    // friend class DynamicActorGraph;

    /*
      upcxx rpc and lpc's are required to be registered as the run method must wait for all of them
      to be processed before exiting the run method, readiness functions are needed for pthread
      strategy and since it is the worst performing one is not regarded for migration
    */

    friend class ActorImpl;
    friend class TaskDeque;

  public:
    bool migrationphase = false;

  private:
    std::unordered_map<std::string, GlobalActorRef> actors; // global map of string->actor
    MigrationDispatcher *migcaller; // the pointer to DAG Base to be able to call migration with run()

  public:
    size_t terminated = 0;
    TaskDeque taskDeque;

  private:
    upcxx::dist_object<ActorGraph *> remoteGraphComponents; // dist obj to actorgraphs of other ranks
    uint64_t totalTime;   // total active Runtime of this rank (if it is being tracked)
    uint64_t totalTokens; // total amount of tokens gathered by this rank
    PortGraph *pg;

  public:
    std::set<ActorImpl *> localActors; // the map of local actors to their triggers
    std::unordered_set<ActorImpl *> unorderedLocalActors;
    bool has_a_terminated_actor = false;

#ifdef PARALLEL
    std::mutex actorLock;                       // lock in OMP strategy so that no conflicts occure
    std::atomic<unsigned int> activeActorCount; // amount of active actors in this rank, for run() termination
    std::atomic<unsigned int> rpcsInFlight;     // amount of rpcs that needs to be processed
    std::atomic<unsigned int> lpcsInFlight;     // amount of lps that needs to be processed
#else
    unsigned int activeActorCount; // amount of active actors in this rank, for run() termination
    unsigned int rpcsInFlight;     // amount of rpcs that needs to be processed
    unsigned int lpcsInFlight;     // amount of lps that needs to be processed
#endif
    size_t actorCount = 0; // amount of active actors in this rank, for run() termination
    double workDone = 0.0;

  public:
    std::mutex tc_mut;
    upcxx::dist_object<upcxx::global_ptr<unsigned int>> taskCount;
    upcxx::dist_object<upcxx::global_ptr<double>> gatheredCost;
    upcxx::dist_object<upcxx::global_ptr<double>> approxTaskCost;
    std::vector<upcxx::global_ptr<unsigned int>> gptrsToTaskCounts{};
    std::vector<upcxx::global_ptr<double>> gptrsToGatheredCosts{};
    std::vector<upcxx::global_ptr<double>> gptrsToApproxTaskCosts{};

    std::vector<uint64_t> messages_sent;
    std::vector<uint64_t> migration_succeeded;
    bool reject_migration = false;

    void register_message(upcxx::intrank_t to);
    void register_migration(upcxx::intrank_t endpoint);

    ActorGraph(MigrationDispatcher *dag, PortGraph *pg);
    ~ActorGraph();
    ActorGraph(const ActorGraph &other) = delete;                    // actor graph should not be copied
    ActorGraph(ActorGraph &&other) = delete;                         // or moved
    ActorGraph &operator=(ActorGraph &other) = delete;               // or assigned
    void addActor(ActorImpl *a);                                     // adds an Actor to Graph
    void addActor(Actor *a);                                         // with the user fassed class
    upcxx::future<> addActorAsync(ActorImpl *a);                     // adds an Actor to Graph
    upcxx::future<> addActorAsync(std::vector<ActorImpl *> &actors); // adds an Actor to Graph
    upcxx::future<> addActorAsync(Actor *a);                         // with the user fassed class
    upcxx::future<> addActorToAnotherAsync(GlobalActorRef a, upcxx::intrank_t rank);

    // calls to connectPortsAsync and waits for the result via upcxx::wait
    void connectPorts(GlobalActorRef sourceActor, const std::string &sourcePortName, GlobalActorRef destinationActor,
                      const std::string &destinationPortName);
    // connects ports of 2 actors, one out port form an actor to on in port,a connection is from actor1 (outport) ->
    // (inprot) actor2
    upcxx::future<> connectPortsAsync(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                      GlobalActorRef destinationActor, const std::string &destinationPortName);
    // calls disconnectPortsAsync and waits for the result with ::wait()
    void disconnectPorts(GlobalActorRef sourceActor, const std::string &sourcePortName, GlobalActorRef destinationActor,
                         const std::string &destinationPortName);
    // disconnects ports of 2 actors, a connection is from actor1 (outport) -> (inprot) actor2
    upcxx::future<> disconnectPortsAsync(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                         GlobalActorRef destinationActor, const std::string &destinationPortName);
    size_t getTotalActorCount() const;                      // get total number of actors
    GlobalActorRef getActor(const std::string &name) const; // return the global to an actor with the given name
    bool has(const std::string &name) const;
    GlobalActorRef getActorNoThrow(const std::string &name) const;
    std::string prettyPrint(); // string representation of actor graph
    // if given with argument >0 then runs until the end of the interval
    // otherwise unlimited run
    std::pair<double, bool> run(double opt_seconds,
                                bool firsttime); // runs the constellation of actors untill all of them terminate
    void rmActor(const std::string &name); // calls to rmActorAsync but waits for the result,does not delete the actor,
                                           // user must save its pointer before
    std::tuple<upcxx::future<>, GlobalActorRef>
    rmActorAsync(const std::string
                     &name); // removes an actor from the graph, returns triggercount of actor if it has any, does not
                             // delete the actor, user must save its pointer before, gets ref and calls rmActorAsync(a)
    std::tuple<upcxx::future<>, GlobalActorRef> rmActorAsync(
        const std::string &name,
        ActorState as); // removes an actor from the graph, returns triggercount of actor if it has any, does not delete
                        // the actor, user must save its pointer before, gets ref and calls rmActorAsync(a)
    const std::unordered_map<std::string, GlobalActorRef> *
    getActors() const; // returns the global actor map of this rank
    const std::unordered_map<std::string, GlobalActorRef> &
    getActorsRef() const;                 // returns the global actor map of this rank
    unsigned int getActiveActors() const; // get amount of active actors
    bool has(const ActorImpl *a) const;   // checks if actor is part of actorgraph
    void prepare(const std::unordered_map<std::string, upcxx::intrank_t>
                     &l); // calls the prepare method of given actors, it is a map because it is the type that is used
                          // by migration
    double run();         // runs ActorGraph until no work is left
    std::tuple<uint64_t, uint64_t> calcRankWork() const; // calculates ranks total work (sum of all actors)
    std::tuple<uint64_t, uint64_t> getTotalWork() const; // returns the total tracked work by this rank
    double getWorkDoneForMigration() const;
    std::optional<size_t> getActorTriggerCount(const std::string &name);
    TaskDeque *getTaskDeque();
    upcxx::future<> changeRef(const std::string &name, GlobalActorRef ref);
    void printActorStates() const;
#ifdef USE_ACTOR_TRIGGERS
    upcxx::future<> sendTriggers(const std::unordered_map<std::string, upcxx::intrank_t> &migList);
    ActorTriggers moveTriggers(const std::string &name);
    void addTriggers(ActorTriggers &&att);
#endif
    void setRunning(const std::unordered_map<std::string, upcxx::intrank_t> &migList);
    void addToWork(uint64_t time);
    std::set<ActorImpl *> &getLocalActorsRef();
    std::unordered_set<ActorImpl *> &getUnorderedLocalActorsRef();
    unsigned int getTaskCount() const;
    bool validPointer(ActorImpl *ai) const;
    void addTask(double cost);

    double getRandomActorCost() const;
    std::pair<double, std::string> getMaxActorCost() const;
    double getApproxTaskTimeCost() const;

    upcxx::future<> reduce_communicaiton_matrix(std::vector<std::vector<uint64_t>> &matrix);
    upcxx::future<> reduce_migration_matrix(std::vector<std::vector<uint64_t>> &matrix);

  private:
    bool checkRm(const std::string &actorName);                              // rm actor from map
    void checkInsert(const std::string &actorName, GlobalActorRef actorRef); // insert actor to map

    /*
      Helper functions to connect and disconnect ports
      a -> b since a and b maybe on different ranks and the function can be called from an arbitrary rank
      connect|disconnect - Source must be called for a once and then for b
      this is done with these functions by appropriately calling

      GlobalChannelRef connectDestination(GlobalActorRef destinationActor, std::string destinationPortName);
      upcxx::future<> connectSource(GlobalActorRef sourceActor, std::string sourcePortName, GlobalChannelRef
      channelRef); GlobalChannelRef disconnectDestination(GlobalActorRef destinationActor, std::string
      destinationPortName); upcxx::future<> disconnectSource(GlobalActorRef sourceActor, std::string sourcePortName);

      which are only available intern in ActorGraph.cpp
    */
    upcxx::future<> connectFromDestination(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                           GlobalActorRef destinationActor, const std::string &destinationPortName);
    upcxx::future<> connectFromSource(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                      GlobalActorRef destinationActor, const std::string &destinationPortName);
    upcxx::future<> connectFromThird(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                     GlobalActorRef destinationActor, const std::string &destinationPortName);
    upcxx::future<> disconnectFromDestination(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                              GlobalActorRef destinationActor, const std::string &destinationPortName);
    upcxx::future<> disconnectFromSource(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                         GlobalActorRef destinationActor, const std::string &destinationPortName);
    upcxx::future<> disconnectFromThird(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                        GlobalActorRef destinationActor, const std::string &destinationPortName);

    // void finishInitialization();                       // finish init of actors
    // void finishReinitialization();                                           // reinitialze actors after migration
    std::pair<double, bool> serialRun(double seconds); // run call serialRun with upcxx ranks strategy
    std::pair<double, bool> ompRun(double seconds);    // run calls ompRun OMP strategy
    void startActors();                                // set actors to started
    void restartActors();                              // check actors and restart
    void addNoOverflow(uint64_t token, uint64_t time); // add to tracke time, throw exp if there is an overflow or warn

    void generateAndAddTasks();

    unsigned int getTaskCountActive() const;
    double getApproxTaskTimeCostActive() const;
    double getActorCostActive() const;
};
