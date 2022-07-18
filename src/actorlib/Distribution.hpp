#pragma once

#include "ActorGraph.hpp"
#include "PortGraph.hpp"
#include <atomic>
#include <cassert>
#include <cmath>
#include <functional>
#include <iostream>
#include <metis.h>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <upcxx/upcxx.hpp>
#include <vector>

extern "C"
{
#include <metis.h>
}

/*
  Distribution is used to distribute actors that are given as a list to the Actorlib to nodes
  Overriden classes can be used to play around with metis and distribution or simulate power outages in specific nodes
  etc
*/

class Distribution
{
  private:
    int seed;

    upcxx::dist_object<Distribution *>
        remoteDistribution;             // used to send the distributed actors to other ranks (we sue serial metis)
    idx_t metisOptions[METIS_NOPTIONS]; // array for metis obtions
    unsigned int vertexcount;           // amount of vertices in the graph
    volatile unsigned int numToGet;
    std::unordered_map<std::string, upcxx::intrank_t> migs; // the migration map, set in case of a global redistribution
    std::unordered_map<std::string, size_t> orderings;      // maps actor names to an id for use in metis
    std::unordered_map<std::string, size_t> costs;
    std::vector<idx_t> colIndex;               // adjncy argument of metis
    std::vector<idx_t> rowIndex;               // xadj argument of metis
    std::vector<upcxx::intrank_t> partitioned; // returned partitions, same order in the actornames vector
    std::vector<idx_t> vertexWeights;

    std::vector<upcxx::intrank_t> distribute(
        double percentage = 1.0); // a helper function that distributes actors to the first percentage% of total ranks

    void generateOrdering(const std::vector<std::string> &actornames); // gives size_t indexes to actors

    void generateWeights(const std::vector<std::string> &actornames,
                         const std::unordered_map<std::string, GlobalActorRef> &actor_map);

    void generateMigList();

    void redistributeActors(const std::unordered_map<std::string, GlobalActorRef> &actor_map,
                            const std::vector<std::tuple<std::string, std::string>>
                                &connections); // redistribute actors on a subset of total ranks

    void clearData();
    void clearDataKeepPartitioningAndMigList();

    void acceptWeights(std::unordered_map<std::string, size_t> &&remoteCosts);

    std::unordered_map<std::string, size_t>
    generateLocalWeights(const std::unordered_map<std::string, GlobalActorRef> &actor_map);

    void gatherActorCosts(const std::unordered_map<std::string, GlobalActorRef> &actor_map);

  public:
    Distribution();
    ~Distribution(); // distribute and assigned to partitioned (with side affects not visible ot outside)

    void accept(std::vector<upcxx::intrank_t> &&other);                     // assigns another distribution vector
    void accept(std::unordered_map<std::string, upcxx::intrank_t> &&other); // assigns another mgiration list

    void
    distributeActors(const std::vector<std::string> &actornames,
                     const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections);
    // distribute actors if name list and connection list

    void redistributeActors(const PortGraph &pg, const ActorGraph &ag);

  private:
    std::string compressedValsStr() const; // print de xadjs and the other of the actor and migration list

    // creates the compressedsparserow from the user provided input
    // ordering is the list of actor names
    // connections are from Actor A -> B. with A|a -> b|B
    // actor name, outport name of actor a, inport name of b, name of Actor B
    void createCompressedSparseRow(
        const std::vector<std::string> &actornames,
        const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections);

    // some as above but uses not the initial connection vector (see singleactoractorgraph for connection vector),
    // but uses the connection vector retrieved from port graph (without ports only with actors)
    void createCompressedSparseRow(const std::vector<std::string> &nodes,
                                   const std::vector<std::tuple<std::string, std::string>> &edges);

    // string output but uses names to display it better
    std::string dist2Str(const std::vector<std::string> &actornames) const;

  public:
    std::vector<upcxx::intrank_t> movePartitioning();                      // return the partitioned vector
    std::unordered_map<std::string, upcxx::intrank_t> moveMigrationList(); // get a copy
    // const std::unordered_map<std::string, upcxx::intrank_t> &getMigrationListRef() const;

    void clearMigList();
};