#pragma once
#include "Utility.hpp"
#include "config.hpp"
#include <algorithm>
#include <fstream>
#include <mutex>
#include <ostream>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <upcxx/upcxx.hpp>
#include <utility>
#include <vector>
/*
  saves all connections A|op -> ip|B
  but the connection is saved on both node A and B
  in order to have less look-up time
*/

class Node
{
  private:
  public:
    std::vector<std::tuple<std::string, std::string, std::string>>
        outgoing; // connection of type op -> ip|B (saved as op,B,ip)
    std::vector<std::tuple<std::string, std::string, std::string>>
        incoming; // connection of type C|op -> ip (saved as ip,C,op)

    Node();

    ~Node() = default;

    std::string toStr() const;

    std::string fromVecToStr(const std::vector<std::tuple<std::string, std::string, std::string>> &vec)
        const; // get all actors and ports that sent to

    std::string toVecToStr(const std::vector<std::tuple<std::string, std::string, std::string>> &vec)
        const; // get all actors and ports that receive

    std::vector<std::string> getReceivers() const; // actors that receive messages from this actor

    std::vector<std::string> getSenders() const; // actor that send to this actor

    std::vector<std::tuple<std::string, std::string>> getSendersWithPortName() const;

    size_t connectionCount() const; // number of actors that this actor send messages to
};

class PortGraph
{

  private:
    std::mutex mut;
    std::unordered_map<std::string, Node> nodes; // string to nodes
    std::unordered_map<std::string, int> ranks;  // the ranks that the nodes are on
    std::vector<int> actors_on_rank;
    upcxx::dist_object<PortGraph *> remoteGraphComponents;
    mutable std::mt19937 randomEngine;

    void insertEdgeBothDir(const std::string &from, const std::string &outport, const std::string &to,
                           const std::string &inport); // insert the connection of thype A|op->ip|B to both

    upcxx::future<>
    insertEdgeBothDirAsync(const std::string &from, const std::string &outport, const std::string &to,
                           const std::string &inport); // insert the connection of thype A|op->ip|B to both

    std::vector<std::string> actorsOnRank(int rank) const;

    void propagateRankChange(const std::string &name, int rank);

  public:
    PortGraph();

    bool insertNode(const std::string &name, int rank); // insert edge on rank rank to a graph
    void insertNodes(const std::vector<std::string> &names, const std::vector<int> &ranks);

    bool insertEdge(const std::string &fromnode, const std::string &outport, const std::string &tonode,
                    const std::string &inport);

    upcxx::future<> insertNodeAsync(const std::string &name, int rank);

    upcxx::future<> insertEdgeAsync(const std::string &fromnode, const std::string &outport, const std::string &tonode,
                                    const std::string &inport);

    void insertEdges(const std::vector<std::string> &fromnodes, const std::vector<std::string> &outports,
                     const std::vector<std::string> &tonodes, const std::vector<std::string> &inports);

    void insertEdges(const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections);

    std::string toStr() const; // print portgraph as string

    std::vector<std::tuple<std::string, std::string, std::string>>
    getOutgoing(const std::string &name) const; // get outgoing connections from actor with anem name

    std::vector<std::tuple<std::string, std::string, std::string>>
    getIncoming(const std::string &name) const; // get ingoing connections to actor with anem name

    std::vector<std::string> getNodes() const; // return all actors in protGraph

    std::vector<std::tuple<std::string, std::string>> getEdges() const; // get all edges in the PortGraph

    std::tuple<std::vector<std::string>, std::vector<std::tuple<std::string, std::string>>>
    getGraph() const; // get the Graph as vertexlist and edge lsit

    std::vector<std::string>
    getReceivers(const std::string &name) const; // actors that receive messages from this actor

    std::vector<std::string> getSenders(const std::string &name) const; // actor that send to this actor

    std::vector<std::tuple<std::string, std::string>>
    getSendersWithPortName(const std::string &name) const; // actor that send to this actor

    std::set<std::string> getNeighbors(const std::string &name) const;

    upcxx::future<> changeRankAsyncFF(const std::string &name);

    std::vector<std::string> getRandomActors(size_t count, upcxx::intrank_t not_on) const;

    std::vector<std::string> getRandomActors(size_t count, upcxx::intrank_t not_on, upcxx::intrank_t residing_on) const;

    std::vector<std::string> getRandomActors(size_t count, upcxx::intrank_t not_on, std::set<int> residing_on) const;

    std::set<int> borderingRanks(const std::string &name) const; // bordering ranks for an actor

    std::set<int> borderingRanks(upcxx::intrank_t rank) const; // returns the ranks that border the rank rank

    const Node &getNode(const std::string &name) const;

    std::tuple<std::string, std::string, std::string, std::string> getConnection(const std::string &from,
                                                                                 const std::string &to);

    bool is_neighbor_of(const std::string &name, const int rank);

    std::vector<std::string> neighborsOf(const upcxx::intrank_t rank) const;

    std::vector<std::string> neighborsOf(const upcxx::intrank_t rank, const upcxx::intrank_t residing_on) const;

    bool communicates_with(const Node &nd, const upcxx::intrank_t rank) const;

    bool communicates_with_outside(const Node &nd, const int myrank) const;

    std::vector<std::string> getForlornActors(const upcxx::intrank_t rank) const;
};