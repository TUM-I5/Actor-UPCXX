#include "PortGraph.hpp"
#include <algorithm>
#include <string>

Node::Node() {}

std::string Node::toStr() const
{
    std::string s;
    s += "From: "; // vec_to_str(edgesFrom)
    s += fromVecToStr(incoming);
    s += ", To: "; //+vec_to_str(edgesTo);
    s += toVecToStr(outgoing);
    return s;
}

std::string Node::fromVecToStr(const std::vector<std::tuple<std::string, std::string, std::string>> &vec) const
{
    std::string s = "{";
    if (vec.size() != 0)
    {
        for (unsigned i = 0; i < vec.size() - 1; i++)
        {
            s += std::get<1>(vec[i]);
            s += "'s ";
            s += std::get<2>(vec[i]);
            s += "->";
            s += std::get<0>(vec[i]);
            s += ", ";
        }

        s += std::get<1>(vec[vec.size() - 1]);
        s += "'s ";
        s += std::get<2>(vec[vec.size() - 1]);
        s += "->";
        s += std::get<0>(vec[vec.size() - 1]);
        s += "}";
    }
    else
    {
        s += " }";
    }
    return s;
}

std::vector<std::string> Node::getReceivers() const
{
    std::vector<std::string> rt;
    rt.reserve(outgoing.size());
    for (const auto &pr : outgoing)
    {
        rt.push_back(std::get<1>(pr));
    }
    return rt;
}

std::vector<std::string> Node::getSenders() const
{
    std::vector<std::string> rt;
    rt.reserve(incoming.size());
    for (const auto &pr : incoming)
    {
        rt.push_back(std::get<1>(pr));
    }
    return rt;
}

std::vector<std::tuple<std::string, std::string>> Node::getSendersWithPortName() const
{
    std::vector<std::tuple<std::string, std::string>> rt;
    rt.reserve(incoming.size());
    for (const auto &pr : incoming)
    {
        rt.emplace_back(std::get<1>(pr), std::get<2>(pr));
    }
    return rt;
}

std::string Node::toVecToStr(const std::vector<std::tuple<std::string, std::string, std::string>> &vec) const
{
    std::string s = "{";
    if (!vec.empty())
    {
        for (unsigned i = 0; i < vec.size() - 1; i++)
        {
            s += std::get<0>(vec[i]);
            s += "->";
            s += std::get<1>(vec[i]);
            s += "'s ";
            s += std::get<2>(vec[i]);
            s += ", ";
        }

        s += std::get<0>(vec[vec.size() - 1]);
        s += "->";
        s += std::get<1>(vec[vec.size() - 1]);
        s += "'s ";
        s += std::get<2>(vec[vec.size() - 1]);
        s += "}";
    }
    else
    {
        s += " }";
    }
    return s;
}

void PortGraph::insertEdgeBothDir(const std::string &from, const std::string &outport, const std::string &to,
                                  const std::string &inport)
{
    throw std::runtime_error("call async of this method");
    insertEdgeBothDirAsync(from, outport, to, inport).wait();
}

void PortGraph::insertEdges(const std::vector<std::string> &fromnodes, const std::vector<std::string> &outports,
                            const std::vector<std::string> &tonodes, const std::vector<std::string> &inports)
{
    for (size_t i = 0; i < fromnodes.size(); i++)
    {
        const std::string &from = fromnodes[i];
        const std::string &outport = outports[i];
        const std::string &to = tonodes[i];
        const std::string &inport = inports[i];

        Node &fromnode = (nodes.find(from)->second);
        Node &tonode = (nodes.find(to)->second);

        // connection of type op -> ip|B (saved as op,B,ip)
        // connection of type C|op -> ip (saved as ip,C,op)
        fromnode.outgoing.emplace_back(outport, to, inport);
        tonode.incoming.emplace_back(inport, from, outport);
    }
}

void PortGraph::insertEdges(
    const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections)
{
    for (size_t i = 0; i < connections.size(); i++)
    {
        const std::string &from = std::get<0>(connections[i]);
        const std::string &outport = std::get<1>(connections[i]);
        const std::string &to = std::get<2>(connections[i]);
        const std::string &inport = std::get<3>(connections[i]);

        Node &fromnode = (nodes.find(from)->second);
        Node &tonode = (nodes.find(to)->second);

        // connection of type op -> ip|B (saved as op,B,ip)
        // connection of type C|op -> ip (saved as ip,C,op)
        fromnode.outgoing.emplace_back(outport, to, inport);
        tonode.incoming.emplace_back(inport, from, outport);
    }
}

upcxx::future<> PortGraph::insertEdgeBothDirAsync(const std::string &from, const std::string &outport,
                                                  const std::string &to, const std::string &inport)
{
    Node &fromnode = (nodes.find(from)->second);
    Node &tonode = (nodes.find(to)->second);

    // connection of type op -> ip|B (saved as op,B,ip)
    // connection of type C|op -> ip (saved as ip,C,op)
    std::tuple<std::string, std::string, std::string> fromtup{outport, to, inport};
    std::tuple<std::string, std::string, std::string> totup{inport, from, outport};

    if (std::find(fromnode.outgoing.begin(), fromnode.outgoing.end(), fromtup) == fromnode.outgoing.end())
    {
        fromnode.outgoing.push_back(std::move(fromtup));
    }

    if (std::find(tonode.incoming.begin(), tonode.incoming.end(), totup) == tonode.incoming.end())
    {
        tonode.incoming.push_back(std::move(totup));
    }

    upcxx::promise<> allDone;
    for (int i = 0; i < upcxx::rank_n(); i++)
    {
        if (i == upcxx::rank_me())
        {
            continue;
        }
        auto cx = upcxx::operation_cx::as_promise(allDone);
        upcxx::rpc(
            i, cx,
            [](upcxx::dist_object<PortGraph *> &rmp, std::string from, std::string to, std::string inport,
               std::string outport)
            {
                // maybe should check for the rank instead of returning?
                Node &fromnode = ((*rmp)->nodes.find(from)->second);
                Node &tonode = ((*rmp)->nodes.find(to)->second);

                // connection of type op -> ip|B (saved as op,B,ip)
                // connection of type C|op -> ip (saved as ip,C,op)
                std::tuple<std::string, std::string, std::string> fromtup{outport, std::move(to), inport};
                std::tuple<std::string, std::string, std::string> totup{std::move(inport), std::move(from),
                                                                        std::move(outport)};

                if (std::find(fromnode.outgoing.begin(), fromnode.outgoing.end(), fromtup) == fromnode.outgoing.end())
                {
                    fromnode.outgoing.push_back(std::move(fromtup));
                }

                if (std::find(tonode.incoming.begin(), tonode.incoming.end(), totup) == tonode.incoming.end())
                {
                    tonode.incoming.push_back(std::move(totup));
                }
            },
            remoteGraphComponents, from, to, inport, outport);
    }
    return allDone.finalize();
}

upcxx::future<> PortGraph::insertEdgeAsync(const std::string &from, const std::string &outportname,
                                           const std::string &to, const std::string &inportname)
{
    return insertEdgeBothDirAsync(from, outportname, to, inportname);
}

void PortGraph::insertNodes(const std::vector<std::string> &names, const std::vector<int> &ranks)
{

    for (size_t i = 0; i < names.size(); i++)
    {
        // std::cout << "t1" << std::endl;
        Node nd;
        const std::string &name = names[i];
        const int rank = ranks[i];

        this->nodes.emplace(name, std::move(nd));
        this->ranks.emplace(name, rank);
        actors_on_rank[rank] += 1;
    }
}

upcxx::future<> PortGraph::insertNodeAsync(const std::string &name, int rank)
{
    // std::cout << "t1" << std::endl;
    Node nd;

    nodes.emplace(name, std::move(nd));
    ranks.emplace(name, rank);
    actors_on_rank[rank] += 1;

    upcxx::promise<> allDone;
    for (int i = 0; i < upcxx::rank_n(); i++)
    {
        if (i == upcxx::rank_me())
        {
            continue;
        }
        auto cx = upcxx::operation_cx::as_promise(allDone);
        upcxx::rpc(
            i, cx,
            [](upcxx::dist_object<PortGraph *> &rmp, std::string name, int rank)
            {
                // maybe should check for the rank instead of returning?
                Node nd;
                (*rmp)->nodes.emplace(name, std::move(nd));
                (*rmp)->ranks.emplace(std::move(name), rank);
                (*rmp)->actors_on_rank[rank] += 1;
            },
            remoteGraphComponents, name, rank);
    }
    auto fut = allDone.finalize();
    return fut;
}

PortGraph::PortGraph()
    : nodes{}, ranks{}, actors_on_rank{}, remoteGraphComponents(this), randomEngine{std::random_device{}()}
{

    actors_on_rank.resize(util::rank_n());
    std::fill(actors_on_rank.begin(), actors_on_rank.end(), 0);
}

bool PortGraph::insertNode(const std::string &name, int rank)
{
    throw std::runtime_error("call async of this method");
    insertNodeAsync(name, rank).wait();
    return true;
}

bool PortGraph::insertEdge(const std::string &from, const std::string &outportname, const std::string &to,
                           const std::string &inportname)
{
    throw std::runtime_error("call async of this method");
    insertEdgeBothDirAsync(from, outportname, to, inportname).wait();
    return true;
}

std::string PortGraph::toStr() const
{
    std::string s = "{\n";
    for (std::pair<std::string, Node> el : nodes)
    {
        s += el.first;
        s += ":= ";
        s += (el.second).toStr();
        s += "\n";
    }
    s += "}\n";

    return s;
}

// return order outport of self -> other ActorImpl name, other ActorImpl inport name
std::vector<std::tuple<std::string, std::string, std::string>> PortGraph::getOutgoing(const std::string &name) const
{
    auto indit = nodes.find(name);
    if (indit == nodes.end())
    {
        throw std::runtime_error("Actor not in Graph");
    }
    else
    {
        const Node *nd = &(indit->second);
        if (nd->outgoing.empty())
        {
            // depends on whether we accept actors without connections
            // throw std::runtime_error("getOugoing malfunctioned");
            return {};
        }
        std::vector<std::tuple<std::string, std::string, std::string>> vec = nd->outgoing;
        return vec;
    }
}

// return order: inportname, otheractor name, other actors ourport name
std::vector<std::tuple<std::string, std::string, std::string>> PortGraph::getIncoming(const std::string &name) const
{
    auto indit = nodes.find(name);
    if (indit == nodes.end())
    {
        throw std::runtime_error("Actor not in Graph");
    }
    else
    {
        const Node *nd = &(indit->second);
        if (nd->incoming.size() == 0)
        {
            // depends on whether we accept actors without connections
            // throw std::runtime_error("getIncoming malfunctioned");
            return {};
        }
        std::vector<std::tuple<std::string, std::string, std::string>> vec = nd->incoming;
        return vec;
    }
}

std::vector<std::string> PortGraph::getNodes() const
{
    std::vector<std::string> tr;
    tr.reserve(nodes.size());
    for (const auto &pr : nodes)
    {
        tr.push_back(pr.first);
    }
    return tr;
}

std::vector<std::tuple<std::string, std::string>> PortGraph::getEdges() const
{
    std::vector<std::tuple<std::string, std::string>> tr;
    for (const auto &pr : nodes)
    {
        const Node *nd = &(pr.second);
        std::vector<std::string> rcvs = nd->getReceivers();
        std::string name = pr.first;
        for (size_t i = 0; i < rcvs.size(); i++)
        {

            tr.emplace_back(std::move(name), std::move(rcvs[i]));
        }
    }

    return tr;
}

std::tuple<std::vector<std::string>, std::vector<std::tuple<std::string, std::string>>> PortGraph::getGraph() const
{

    std::vector<std::string> nds;
    std::vector<std::tuple<std::string, std::string>> edgs;
    for (auto &pr : nodes)
    {
        const Node *nd = &(pr.second);
        nds.emplace_back(pr.first);

        std::vector<std::string> rcvs = nd->getReceivers();

        for (size_t i = 0; i < rcvs.size(); i++)
        {
            std::string name = pr.first;
            std::tuple<std::string, std::string> edge(std::move(name), std::move(rcvs[i]));
            edgs.emplace_back(std::move(edge));
        }

        std::vector<std::string> sndrs = nd->getSenders();
        for (size_t i = 0; i < sndrs.size(); i++)
        {
            std::string name = pr.first;
            std::tuple<std::string, std::string> edge(std::move(sndrs[i]), std::move(name));
            edgs.emplace_back(std::move(edge));
        }
    }

    return {nds, edgs};
}

size_t Node::connectionCount() const { return outgoing.size(); }

std::vector<std::string> PortGraph::getReceivers(const std::string &name) const
{
    auto it = nodes.find(name);
    if (it != nodes.end())
    {
        return it->second.getReceivers();
    }
    else
    {
        throw std::runtime_error("could not find");
    }
}

std::vector<std::string> PortGraph::getSenders(const std::string &name) const
{
    auto it = nodes.find(name);
    if (it != nodes.end())
    {
        return it->second.getSenders();
    }
    else
    {
        throw std::runtime_error("could not find");
    }
}

std::vector<std::tuple<std::string, std::string>> PortGraph::getSendersWithPortName(const std::string &name) const
{
    auto it = nodes.find(name);
    if (it != nodes.end())
    {
        return it->second.getSendersWithPortName();
    }
    else
    {
        throw std::runtime_error("could not find");
    }
}

std::set<std::string> PortGraph::getNeighbors(const std::string &name) const
{
    auto it = nodes.find(name);
    if (it != nodes.end())
    {
        auto recv = it->second.getReceivers();
        std::set<std::string> set =
            std::set<std::string>(std::make_move_iterator(recv.begin()), std::make_move_iterator(recv.end()));
        auto send = it->second.getSenders();
        for (auto &&s : send)
        {
            set.insert(std::move(s));
        }

        return set;
    }
    else
    {
        throw std::runtime_error("could not find" + name);
    }
}

bool PortGraph::communicates_with(const Node &nd, const upcxx::intrank_t rank) const
{
    auto &outs = nd.outgoing;
    auto &ins = nd.incoming;

    for (auto &tup : outs)
    {
        if (this->ranks.find(std::get<1>(tup))->second == rank)
        {
            return true;
        }
    }

    for (auto &tup : ins)
    {
        if (this->ranks.find(std::get<1>(tup))->second == rank)
        {
            return true;
        }
    }

    return false;
}

bool PortGraph::communicates_with_outside(const Node &nd, const int myrank) const
{
    auto &outs = nd.outgoing;
    auto &ins = nd.incoming;

    for (auto &tup : outs)
    {
        if (this->ranks.find(std::get<1>(tup))->second != myrank)
        {
            return true;
        }
    }

    for (auto &tup : ins)
    {
        if (this->ranks.find(std::get<1>(tup))->second != myrank)
        {
            return true;
        }
    }

    return false;
}

std::vector<std::string> PortGraph::neighborsOf(const upcxx::intrank_t rank) const
{
    std::vector<std::string> neighbors;
    for (const auto &pr : this->ranks)
    {
        if (pr.second != rank)
        {
            const Node &nd = this->nodes.find(pr.first)->second;
            if (communicates_with(nd, rank))
            {
                neighbors.push_back(pr.first);
            }
        }
    }
    return neighbors;
}

std::vector<std::string> PortGraph::neighborsOf(const upcxx::intrank_t rank, const upcxx::intrank_t residing_on) const
{
    std::vector<std::string> neighbors;
    for (const auto &pr : this->ranks)
    {
        if (pr.second != rank && pr.second == residing_on)
        {
            const Node &nd = this->nodes.find(pr.first)->second;
            if (communicates_with(nd, rank))
            {
                neighbors.push_back(pr.first);
            }
        }
    }
    return neighbors;
}

std::vector<std::string> PortGraph::getForlornActors(const upcxx::intrank_t rank) const
{
    std::vector<std::string> forlorn;
    for (const auto &pr : this->ranks)
    {
        if (pr.second == rank)
        {
            const Node &nd = this->nodes.find(pr.first)->second;
            if (communicates_with_outside(nd, rank))
            {
                forlorn.push_back(pr.first);
            }
        }
    }
    return forlorn;
}

void PortGraph::propagateRankChange(const std::string &name, int rank)
{
#ifdef PARALLEL
    mut.lock();
#endif

    auto it = ranks.find(name);
    actors_on_rank[it->second] -= 1;
    ranks[name] = rank;
    actors_on_rank[rank] += 1;

#ifdef PARALLEL
    mut.unlock();
#endif
}

upcxx::future<> PortGraph::changeRankAsyncFF(const std::string &name)
{
    upcxx::promise<> allDone;
    for (int i = 0; i < upcxx::rank_n(); i++)
    {
        if (i == upcxx::rank_me())
        {
            propagateRankChange(name, upcxx::rank_me());
        }
        else
        {
            auto cx = upcxx::operation_cx::as_promise(allDone);
            upcxx::rpc(
                i, cx,
                [](upcxx::dist_object<PortGraph *> &remoteGraphComponents, int rank, std::string name)
                { (*remoteGraphComponents)->propagateRankChange(name, rank); },
                remoteGraphComponents, upcxx::rank_me(), name);
        }
    }
    return allDone.finalize();
}

std::vector<std::string> PortGraph::actorsOnRank(int rank) const
{
    std::vector<std::string> ret;
    for (auto &el : ranks)
    {
        if (el.second == rank)
        {
            ret.push_back(el.first);
        }
    }
    return ret;
}

std::vector<std::string> PortGraph::getRandomActors(size_t count, upcxx::intrank_t not_on) const
{
    static std::default_random_engine generator;
    static std::uniform_int_distribution<int> distribution(0, std::numeric_limits<int>::max());

    if (nodes.empty())
    {
        return {};
    }

    std::set<std::string> out;
    std::vector<std::string> ret;
    std::vector<std::string> ret2;

    for (const auto &el : nodes)
    {
        if (this->ranks.find(el.first)->second != not_on)
        {
            ret.push_back(el.first);
        }
    }

    if (ranks.size() < count)
    {
        return ret;
    }

    unsigned int limit = ret.size() * 2;
    unsigned int cur = 0;

    while (out.size() < count)
    {
        size_t num = static_cast<size_t>(distribution(generator)) % ret.size();
        out.insert(std::move(ret[num]));

        cur += 1;

        if (cur >= limit)
        {
            break;
        }
    }

    ret2.reserve(out.size());
    for (auto &&el : out)
    {
        ret2.push_back(std::move(el));
    }
    return ret;
}

std::vector<std::string> PortGraph::getRandomActors(size_t count, upcxx::intrank_t not_on,
                                                    upcxx::intrank_t residing_on) const
{
    static std::default_random_engine generator;
    static std::uniform_int_distribution<int> distribution(0, std::numeric_limits<int>::max());

    if (nodes.empty())
    {
        return {};
    }

    std::set<std::string> out;
    std::vector<std::string> ret;
    std::vector<std::string> ret2;

    for (const auto &el : nodes)
    {
        if (this->ranks.find(el.first)->second == residing_on)
        {
            ret.push_back(el.first);
        }
    }

    if (ranks.size() < count)
    {
        return ret;
    }

    unsigned int limit = ret.size() * 2;
    unsigned int cur = 0;

    if (ret.empty())
    {
        return ret2;
    }

    while (out.size() < count)
    {
        size_t num = static_cast<size_t>(distribution(generator)) % ret.size();
        out.insert(std::move(ret[num]));

        cur += 1;

        if (cur >= limit)
        {
            break;
        }
    }

    ret2.reserve(out.size());
    for (auto &&el : out)
    {
        ret2.push_back(std::move(el));
    }

    return ret2;
}

std::vector<std::string> PortGraph::getRandomActors(size_t count, upcxx::intrank_t not_on,
                                                    std::set<int> residing_on) const
{
    static std::default_random_engine generator;
    static std::uniform_int_distribution<int> distribution(0, std::numeric_limits<int>::max());

    if (nodes.empty())
    {
        return {};
    }

    std::set<std::string> out;
    std::vector<std::string> ret;
    std::vector<std::string> ret2;

    for (const auto &el : nodes)
    {
        if (residing_on.find(this->ranks.find(el.first)->second) != residing_on.end())
        {
            ret.push_back(el.first);
        }
    }

    if (ranks.size() < count)
    {
        return ret;
    }

    unsigned int limit = ret.size() * 2;
    unsigned int cur = 0;

    if (ret.empty())
    {
        return ret2;
    }

    while (out.size() < count)
    {
        size_t num = static_cast<size_t>(distribution(generator)) % ret.size();
        out.insert(std::move(ret[num]));

        cur += 1;

        if (cur >= limit)
        {
            break;
        }
    }

    ret2.reserve(out.size());
    for (auto &&el : out)
    {
        ret2.push_back(std::move(el));
    }
    return ret2;
}

const Node &PortGraph::getNode(const std::string &name) const
{
    auto it = nodes.find(name);
    return it->second;
}

std::set<int> PortGraph::borderingRanks(const std::string &name) const
{
    auto it = nodes.find(name);
    if (it == nodes.end())
    {
        return {};
    }
    const Node &nd = it->second;
    int rank = ranks.find(name)->second;
    std::set<int> ret;
    auto snds = nd.getSenders();
    for (auto &snd : snds)
    {
        int other = ranks.find(snd)->second;
        if (other != rank)
        {
            ret.insert(other);
        }
    }
    auto rcvs = nd.getReceivers();
    for (auto &rcv : rcvs)
    {
        int other = ranks.find(rcv)->second;
        if (other != rank)
        {
            ret.insert(other);
        }
    }

    return ret;
}

std::set<int> PortGraph::borderingRanks(int rank) const
{
    std::set<int> ret;
    for (auto &pr : ranks)
    {
        if (pr.second == rank)
        {
            ret.merge(borderingRanks(pr.first));
        }
    }

    return ret;
}

std::tuple<std::string, std::string, std::string, std::string> PortGraph::getConnection(const std::string &from,
                                                                                        const std::string &to)
{
    auto it = nodes.find(from);
    if (it == nodes.end())
    {
        throw std::runtime_error("nonexistent connection asked!");
    }
    const Node &nd = it->second;

    const auto &outs = nd.outgoing;

    // connection of type op -> ip|B (saved as op,B,ip)
    for (const auto &contup : outs)
    {
        if (std::get<1>(contup) == to)
        {
            return std::make_tuple(from, std::get<0>(contup), std::get<1>(contup), std::get<2>(contup));
        }
    }

    throw std::runtime_error("nonexistent connection asked!");
}

bool PortGraph::is_neighbor_of(const std::string &name, const int rank)
{
    std::set<std::string> nbs = getNeighbors(name);

    for (const std::string &el : nbs)
    {
        if (this->ranks.find(el)->second == rank)
        {
            return true;
        }
    }

    return false;
}