#pragma once
#include "AbstractInPort.hpp"
#include "AbstractOutPort.hpp"
#include "AcquisitionResult.hpp"
#include "Actor.hpp"
#include "ActorGraph.hpp"
#include "ActorImpl.hpp"
#include "ActorState.hpp"
#include "Distribution.hpp"
#include "MigrationDispatcher.hpp"
#include "PortGraph.hpp"
#include "Statistics.hpp"
#include "Utility.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <numeric>
#include <ostream>
#include <random>
#include <set>
#include <string>
#include <type_traits>
#include <unistd.h>
#include <upcxx/upcxx.hpp>

constexpr double allowed_imbalance = 1.10;

/*
    DynamicGraph uses curiously recurring template pattern with 2 Templates, as 2 Strategies for implementing
   DynamicActorGraph, SingleActorAGraph and DiynamicActorGraph where SAAG is for one actor type and has less overhead,
   DAAG is not complete but it is used to support actorGraphs with multiple ActorTypes

    1-Wrapper around AG
    2-PortGraph functionality
    3-Distribution functionality
    3-Migration functionality

    Any functionality that differs when the there can be multiple actor or not are distributed to the template argumnets
   which are SAAD or DAAG
*/

class DynamicActorGraph : public MigrationDispatcher
{
    // friend A;

  private:
    unsigned int interrupts; // number of migration phases between run calls
    std::shared_mutex
        migrationMutex; // this is needed even in rank mode because other ranks can try to steal at the same time
    mutable std::mt19937 randomEngine;

    // not needed in rank strategy
    // std::shared_mutex mark_lock;
    // std::vector<bool> used;
    std::vector<std::pair<bool, std::unordered_map<GlobalActorRef, bool>>> pins;
    std::vector<double> buffer;
    bool dont_use_buffer = false;
    // bool losing = false;
    std::set<int> stealing_from_me{};

  protected:
    size_t rpc_on_wait = 0;
    bool sync_init = false;

  public:
    std::set<GlobalActorRef> received_actors;
    size_t iter = 0;

    size_t allocated_actors = 0;
    size_t deallocated_actors = 0;

    size_t going_away_limit = 9999;
    size_t going_away = 0;

    bool order_victims = false;
    bool steal_during_termination = false;
    bool time_spent_for_cost = true;
    bool contigious_migration = false;
    bool use_rma = false;
    bool timetask = false;

    bool processing_steal_request = false;

    bool borders_changed = true;
    std::vector<std::string> forlorn_actors{};

  protected:
    PortGraph pg; // port graph to save connections
    ActorGraph ag;
    upcxx::dist_object<DynamicActorGraph *> remoteComponents; // remote components for sending shit
    std::mt19937 rng;

  public:
    std::unordered_map<std::string, upcxx::intrank_t> l_migList;
    Distribution actorDistribution; // for actor distribution, pointer because

  public:
    DynamicActorGraph()
        : MigrationDispatcher(), interrupts(0), migrationMutex(), randomEngine{std::random_device{}()}, pins(), pg(),
          ag(dynamic_cast<MigrationDispatcher *>(this), &pg), remoteComponents(this), actorDistribution()
    {
#ifdef REPORT_MAIN_ACTIONS
        constexpr size_t buflen = 1000;
        char name[buflen];
        gethostname(name, buflen);
        std::cout << upcxx::rank_me() << ": host name -> " << name << std::endl;
#endif

        if (upcxx::rank_me() == 0)
        {
#ifdef GLOBAL_MIGRATION
            // std::cout << "Config: steal actors from any rank (global)" << std::endl;
            std::cerr << "Config: Steal actors from any rank (global)" << std::endl;
#else
            // std::cout << "Config: steal only neighboring actors" << std::endl;
            std::cerr << "Config: Steal only neighboring actors (local)" << std::endl;
#endif

#ifdef STEAL_FROM_BUSY_RANK
            // std::cout << "Config: steal from the busiest of the allowed ranks" << std::endl;
            std::cerr << "Config: Steal from the busiest of the allowed ranks (busy)" << std::endl;
#else
            // std::cout << "Config: steal randomly" << std::endl;
            std::cerr << "Config: Steal randomly (random)" << std::endl;
#endif
        }

        constexpr char glimit[] = "GOING_AWAY_LIMIT";
        constexpr char order[] = "ORDER_VICTIMS";
        constexpr char stealdurterm[] = "STEAL_DURING_TERMINATION";
        constexpr char costmode[] = "USE_TIME_SPENT";
        constexpr char continousmig[] = "CONTIGIOUS_MIGRATION";
        constexpr char userma[] = "RMA_TASKCOUNT";
        constexpr char syncinit[] = "SLOW_INIT";
        constexpr char codemode[] = "UPCXX_CODEMODE";
        constexpr char usetimetask[] = "USE_BOTH";

        util::parseBooleanEnvVar(order, order_victims);
        util::parseBooleanEnvVar(stealdurterm, steal_during_termination);
        util::parseBooleanEnvVar(costmode, time_spent_for_cost);
        util::parseBooleanEnvVar(continousmig, contigious_migration);
        util::parseBooleanEnvVar(userma, use_rma);
        util::parseBooleanEnvVar(syncinit, sync_init);
        util::parseBooleanEnvVar(usetimetask, timetask);

        char *val = getenv(codemode);
        if (val != nullptr)
        {
            if (upcxx::rank_me() == 0)
            {
                std::cerr << "Config: UPCXX codemode: " << val << std::endl;
            }
        }
        else
        {
            if (upcxx::rank_me() == 0)
            {
                std::cerr << "Config: UPCXX codemode: undefined" << std::endl;
            }
        }

        if (const char *env_p = std::getenv(glimit))
        {
            going_away_limit = std::stoi(env_p);
            if (upcxx::rank_me() == 0)
            {
                std::cerr << "Config: Going away limit is: " << going_away_limit << std::endl;
            }
        }
        else
        {
            going_away_limit = std::numeric_limits<size_t>::max();
            if (upcxx::rank_me() == 0)
            {
                std::cerr << "Config: Going away limit is unlimited" << std::endl;
            }
        }
        if (upcxx::rank_me() == 0)
        {
            std::cerr << "Config: Order victims: " << order_victims << std::endl;
            std::cerr << "Config: Contigious migration: " << contigious_migration << std::endl;

            if (!timetask)
            {
                if (time_spent_for_cost)
                {
                    std::cerr << "Config: Using time spent in act as the base for migration." << std::endl;
                }
                else
                {
                    std::cerr << "Config: Using task count as the base for migration." << std::endl;
                }
            }
            else
            {
                std::cerr << "Config: Using task count combined with actor cost as the base for migration."
                          << std::endl;
            }

#ifdef USE_ACTOR_TRIGGERS
            std::cerr << "Config: Using actor triggers" << std::endl;
#else
            std::cerr << "Config: Not using actor triggers" << std::endl;
#endif
            if (use_rma)
            {
                std::cerr << "Config: Using rma to exchange task info." << std::endl;
            }
            else
            {
                std::cerr << "Config: Using rpc to exchange task info." << std::endl;
            }

            std::cerr << "Config: Use synchronized calls during init: " << sync_init << std::endl;
        }
    }

    /*
    /dss/dsshome1/lxc05/ge69xij2/actor-upcxx/src/actorlib/DynamicActorGraph.hpp(100): warning #1011: missing return
    statement at end of non-void function "DynamicActorGraph<A>::upcastActor(Act *) [with
    A=MultipleActorAGraph<SimulationActor, DoubleHeightSimulationActor>, Act=SimulationActor]"
          }
    */
    template <typename Act> ActorImpl *upcastActor(Act *a) // upcast actor regardles it inherits from Actor or ActorImpl
    {
        if constexpr (std::is_base_of<Actor, Act>::value)
        {
            Actor *tmpp = static_cast<Actor *>(a);
            return &(tmpp->asBase());
        }
        else if constexpr (std::is_base_of<ActorImpl, Act>::value)
        {
            ActorImpl *ap = static_cast<ActorImpl *>(a);
            return ap;
        }

        return nullptr;
    }

    void addActor(ActorImpl *a); // calls ag::addactor

    void addActor(Actor *a); // calls ag::addactor

    upcxx::future<> addActorAsync(ActorImpl *a); // calls ag::addactor

    upcxx::future<> addActorAsync(Actor *a); // calls ag::addactor

    upcxx::future<> addActorToAnotherAsync(GlobalActorRef a, upcxx::intrank_t rank); // calls ag::addActorTo..

    upcxx::future<> addActorsToAnotherAsync(const std::vector<GlobalActorRef> &refs);

    size_t getTotalActorCount() { return this->ag.getTotalActorCount(); }

    std::string prettyPrint(); // calls ag::

    upcxx::future<> connectPortsAsync(const std::string &sourceActorName, const std::string &sourcePortName,
                                      const std::string &destinationActorName,
                                      const std::string &destinationPortName); // calls::

    upcxx::future<> connectPortsAsync(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                      GlobalActorRef destinationActor,
                                      const std::string &destinationPortName); // calls ag::connectPorts

    upcxx::future<> disconnectPortsAsync(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                         GlobalActorRef destinationActor, const std::string &destinationPortName);

    void connectPorts(const std::string &sourceActorname, const std::string &sourcePortName,
                      const std::string &destinationActorName,
                      const std::string &destinationPortName); // calls::

    void connectPorts(GlobalActorRef sourceActor, const std::string &sourcePortName, GlobalActorRef destinationActor,
                      const std::string &destinationPortName); // calls ag::connectPorts

    GlobalActorRef getActor(const std::string &name) { return ag.getActor(name); }               // calls ag::get
    GlobalActorRef getActorNoThrow(const std::string &name) { return ag.getActorNoThrow(name); } // calls ag::get

    double run(); // calls ag::run

    upcxx::future<> initConnections(const std::vector<std::tuple<std::string, std::string, std::string, std::string>>
                                        &connections); // initializes connections given in the connevtion vector
    void initConnectionsSync(const std::vector<std::tuple<std::string, std::string, std::string, std::string>>
                                 &connections); // initializes connections given in the connevtion vector
    // inits actors given with a name list, connection vector, and input argument U for a solo actor
    // initializes actors after distribution and connect thems
    // constructors must look like A(std::string name,U u)

    // migrates actor with name from its rank to rank rank but async and consist of a chain of callbacks
    // std::optional<upcxx::future<GlobalActorRef>> migrateActorAsync(int rank, const std::string &name, size_t
    // onHoldCalls) override final;

    // steals actor with name from its rank to rank rank but async and consist of a chain of callbacks
    // it is like migrateActor but it is not called from the rank that has the actor but from the rank that wants to
    // actors is not totally async, it blocks until the lock is acquired

    upcxx::future<GlobalActorRef> stealActorAsync(const std::string name) override final;
    upcxx::future<GlobalActorRef> offloadActorAsync(const std::string name, upcxx::intrank_t to) override final;

    virtual ~DynamicActorGraph(){};

    void stopActors(const std::unordered_map<std::string, upcxx::intrank_t> &l);

    upcxx::future<> restartActors(const std::unordered_map<std::string, upcxx::intrank_t> &l);
    upcxx::future<> restartActors(const std::set<GlobalActorRef> &l);

    const std::unordered_map<std::string, GlobalActorRef> *getActors() const { return this->ag.getActors(); }

    const std::unordered_map<std::string, GlobalActorRef> &getActorsRef() const { return this->ag.getActorsRef(); }

    void userEnforcedLocalMigrateActorsDiscretePhases(const std::unordered_map<std::string, upcxx::intrank_t> &migList);

    void print_gitter(const std::unordered_map<std::string, GlobalActorRef> &actormap);
    void print_gitter_active_status() override final;
    void print_gitter_mark_pin();

  private:
    upcxx::future<> disconnectFromNeighboursAsync(const std::unordered_map<std::string, upcxx::intrank_t> &migList);

    upcxx::future<> disconnectFromNeighboursAsync(const std::string &name);

    upcxx::future<> reconnectToNeighboursAsync(const std::unordered_map<std::string, upcxx::intrank_t> &migList);
    upcxx::future<> reconnectToNeighboursAsync(const std::set<GlobalActorRef> &migList);

    upcxx::future<> reconnectToNeighboursAsync(const std::string &name);

    void phases(); // fires the migration phase, it redistributed actors and calls the migration::migrateDiscretePhases
                   // for bulk migration

    upcxx::future<std::vector<double>> getWorkOfOtherRanks(const std::set<int> &ranks);

    upcxx::future<std::vector<double>> getWorkOfOtherRanks();

  public:
    virtual upcxx::future<GlobalActorRef> sendActorToAsync(int rank, std::string const &name) = 0;

    virtual upcxx::future<GlobalActorRef> sendActorToAsync(int rank, GlobalActorRef name) = 0;

  private:
    void rmActor(const std::string &name) { return this->ag.rmActor(name); } // calls ag::rmAct

    std::tuple<upcxx::future<>, GlobalActorRef> rmActorAsync(const std::string &name)
    {
        return this->ag.rmActorAsync(name);
    } // calls ag::rmActAsync

    std::tuple<upcxx::future<>, std::vector<GlobalActorRef>>
    rmActorsAsync(const std::unordered_map<std::string, upcxx::intrank_t> &migList);

    void prepareActors(const std::unordered_map<std::string, upcxx::intrank_t> &l)
    {
        this->ag.prepare(l);
    } // prepare all actors that need migration

    void disconnectActors(const std::unordered_map<std::string, upcxx::intrank_t>
                              &migList); // disconnect actors in map from all neighbors

    upcxx::future<std::vector<std::string>> findActorsToSteal() override final;

    upcxx::future<std::string> findAnActorToStealAndMark() override final;

    upcxx::future<std::string, upcxx::intrank_t> findActorsToOffload() override final;

    bool tryToAcquireLockForActor(const std::string name, const upcxx::intrank_t marker, double workload);

  public:
    upcxx::future<bool> tryToMarkActorForMigration(const std::string name) override final;

    upcxx::future<bool> tryStopSelfAndPinNeighborhood(const std::string name) override final;

    double getWork();

  private:
    upcxx::future<> severeConnections(const std::string &name);

    upcxx::future<> resurrectConnections(const std::string &name, GlobalActorRef ref);

    upcxx::future<> severeConnections(const std::string &name, const std::vector<GlobalActorRef> &neighbors);

    upcxx::future<> resurrectConnections(const std::string &name, GlobalActorRef ref,
                                         const std::vector<GlobalActorRef> &neighbors);

    // rm actors from the graph and save their global pointers as well as triggers,
    // wait in bulk
    std::tuple<upcxx::future<>, std::vector<GlobalActorRef>>
    rmOldActors(const std::unordered_map<std::string, upcxx::intrank_t> &migList);

    // del remains after sending
    // this is needed because deleting them before the send actions are completed ( they return futures )
    // then there will be segmentation faults
    void delRemains(const std::vector<GlobalActorRef> &l);

    // send actors in the migration list to their destination ranks,
    // returns the new actors on the destination and their triggercount
    void sendActorsAsync(const std::vector<GlobalActorRef> &trigs,
                         const std::unordered_map<std::string, upcxx::intrank_t> &migList,
                         std::vector<GlobalActorRef> &buffer);

    // reinserts the actors in the vector to the actorgraph of the other rank
    upcxx::future<> reinsert(const std::vector<GlobalActorRef> &refs);

    // this is different than asynchron one, in the asynchron one you are not allowed to migrate if there is a
    // pinned/migrating neighbor here we severe the connection regardless of that
    upcxx::future<> severeConnections(const std::unordered_map<std::string, upcxx::intrank_t> &migList);

    // reconnects the actors that are migrated to their neighbors
    upcxx::future<> resurrectConnections(const std::unordered_map<std::string, upcxx::intrank_t> &migList);

    // same as void refillPorts(const std::vector<std::pair<GlobalActorRef, std::optional<size_t>>> &news)
    // but uses names and need 1 look up more per actor
    void refillPorts(const std::unordered_map<std::string, upcxx::intrank_t> &migList);
    void refillPorts(const std::set<GlobalActorRef> &migList);

    std::unordered_map<std::string, upcxx::intrank_t>
    cleanseMigrationList(const std::unordered_map<std::string, upcxx::intrank_t> &migList);

    std::unordered_map<std::string, upcxx::intrank_t>
    createLocalMigrationList(const std::unordered_map<std::string, upcxx::intrank_t> &migList);

    size_t getMarkOffset();

    void releaseMarkOffset(size_t);

    void migrateActorsDiscretePhases(std::unordered_map<std::string, upcxx::intrank_t> &&migList);

    upcxx::future<> sendTasks(const std::unordered_map<std::string, upcxx::intrank_t> &migList);

    void clear_portinformations(const std::unordered_map<std::string, upcxx::intrank_t> &migList);

    upcxx::future<> unpin_neighbors(const std::vector<std::string> &neighbors, size_t pin_array_offset);

    upcxx::future<> unpin_neighbors(const std::vector<std::string> &neighbors);

    upcxx::future<> unmark_actor(const std::string &name);

    std::tuple<upcxx::future<bool>, size_t> pin_neighbors(const std::vector<std::string> &neighbors);

    // upcxx::future<bool> mark_actor(const std::string& name);

#ifdef TIME
    void printTimeInfo();
#endif

    std::string getMeAnActor(int marker, int from, int must_border = -1);
};
