#include "Distribution.hpp"

extern "C"
{
#include <metis.h>
}

Distribution::Distribution()
    : seed(42), remoteDistribution(this), vertexcount(0),
      numToGet(0), migs{}, orderings{}, costs{}, colIndex{}, rowIndex{}, partitioned{}, vertexWeights{}
{
    if (const char *env_p = std::getenv("SEED"))
    {
        try
        {
            seed = std::atoi(env_p);
        }
        catch (std::exception &e)
        {
            std::cerr << "SEED environment variable is not a valid integer!" << std::endl;
            seed = 42;
        }
    }

    METIS_SetDefaultOptions(metisOptions);
    metisOptions[METIS_OPTION_PTYPE] = METIS_PTYPE_KWAY;
    metisOptions[METIS_OPTION_OBJTYPE] = METIS_OBJTYPE_CUT;
    metisOptions[METIS_OPTION_NUMBERING] = 0;
    metisOptions[METIS_OPTION_IPTYPE] = METIS_IPTYPE_RANDOM;
    metisOptions[METIS_OPTION_CONTIG] = 1;
    metisOptions[METIS_OPTION_NCUTS] = 1;
    metisOptions[METIS_OPTION_NSEPS] = 2;
    metisOptions[METIS_OPTION_NITER] = 10;
    metisOptions[METIS_OPTION_UFACTOR] = 30;
    metisOptions[METIS_OPTION_SEED] = seed;
    // metisOptions[METIS_OPTION_MINCONN] = 1;
}

template <typename MetisIdx, typename UpcxxIdx> void moveFrom(std::vector<MetisIdx> &from, std::vector<UpcxxIdx> &to)
{
    // std::cout << "Size difference in the size of idx_t and upcxx::intrank_t. Copying and casting." << std::endl;
    to.reserve(from.size());
    for (MetisIdx i : from)
    {
        to.push_back(static_cast<UpcxxIdx>(i));
    }
    from.clear();
}

template <typename Idx> void moveFrom(std::vector<Idx> &from, std::vector<Idx> &to)
{
    // std::cout << "Directly moving the data into the destination, as the size is the same" << std::endl;
    to = std::move(from);
}

std::vector<upcxx::intrank_t> Distribution::distribute(double percentage)
{
    double arc = util::rank_n() * percentage;
    arc += 0.5;
    int rankcount = std::min(static_cast<int>(arc), util::rank_n());
    if (rankcount <= 0 || vertexcount <= 0)
    {
        throw std::runtime_error("Illegal input in distrbuted");
    }
    idx_t numberOfVertices = vertexcount;
    idx_t numberOfConstraints = 1;
    idx_t numberOfPartitions = (percentage == 1.0) ? util::rank_n() : rankcount;
    idx_t finalEdgeCut = 0;
    std::vector<idx_t> partitions(vertexcount, 0);
    int metisResult;

    if (vertexWeights.empty() || numberOfVertices <= 0 ||
        vertexWeights.size() != static_cast<unsigned long>(numberOfVertices))
    {

        throw std::runtime_error("Illegal input in distrbute (check 2)");
    }

    if (rankcount > 1)
    {
        if (vertexcount == static_cast<unsigned int>(util::rank_n()) && percentage == 1.0)
        {
            std::vector<upcxx::intrank_t> res;
            for (unsigned int i = 0; i < static_cast<unsigned int>(util::rank_n()); i++)
            {
                res.push_back(i);
            }
            metisResult = METIS_OK;
            return res;
        }
        else
        {
            metisResult = METIS_PartGraphKway(&numberOfVertices, &numberOfConstraints, rowIndex.data(), colIndex.data(),
                                              vertexWeights.data(), nullptr, nullptr, &numberOfPartitions, nullptr,
                                              nullptr, metisOptions, &finalEdgeCut, partitions.data());

            if (metisResult != METIS_OK)
            {
                throw std::runtime_error("Metis could not distribute graph?");
            }
        }
    }
    else
    {
        std::fill(partitions.begin(), partitions.end(), 0);
        metisResult = METIS_OK;
    }

    if (metisResult != METIS_OK)
    {
        if (metisResult == METIS_ERROR_INPUT)
        {
            throw std::runtime_error("METIS_ERROR_INPUT occurred.");
        }
        if (metisResult == METIS_ERROR_MEMORY)
        {
            throw std::runtime_error("METIS_ERROR_MEMORY occurred.");
        }
        if (metisResult == METIS_ERROR)
        {
            throw std::runtime_error("METIS_ERROR occurred.");
        }

        throw std::runtime_error("An undocumented METIS error occurred.");
    }
    std::vector<upcxx::intrank_t> res;
    moveFrom(partitions, res);
    return res;
}

std::string Distribution::dist2Str(const std::vector<std::string> &actornames) const
{
    if (partitioned.empty())
    {
        return "Empty";
    }
    else
    {
        std::string s;
        for (size_t i = 0; i < partitioned.size(); i++)
        {
            s += actornames[i] + " to rank: " + std::to_string(partitioned[i]) + "\n";
        }
        return s;
    }
}

std::string Distribution::compressedValsStr() const
{
    if (colIndex.empty() || rowIndex.empty())
    {
        return "";
    }

    std::stringstream ss;
    ss << "Col-Index: [";
    for (size_t i = 0; i < colIndex.size() - 1; i++)
    {
        ss << std::to_string(colIndex[i]);
        ss << ", ";
    }
    ss << std::to_string(colIndex[colIndex.size() - 1]);
    ss << "]\n";
    ss << "Row-Index: [";
    for (size_t i = 0; i < rowIndex.size() - 1; i++)
    {
        ss << std::to_string(rowIndex[i]);
        ss << ", ";
    }
    ss << std::to_string(rowIndex[rowIndex.size() - 1]);
    ss << "]\n";

    ss << "VertexWeights: [";
    for (auto i : vertexWeights)
    {
        ss << i << ", ";
    }
    ss << "end]\n";

    // ss << std::endl;
    return ss.str();
}

void Distribution::generateOrdering(const std::vector<std::string> &actornames)
{
    size_t count = actornames.size();
    for (size_t i = 0; i < count; i++)
    {
        const std::string &nm = actornames[i];
        // std::pair<std::string, size_t> pr(nm, i);
        orderings.emplace(nm, i);
    }
}

// creates the compressedsparserow from the user provided input
// ordering is the list of actor names
// connections are from Actor A -> B. with A|a -> B|b
// actor name, outport name of actor a, inport name of b, name of Actor B
void Distribution::createCompressedSparseRow(
    const std::vector<std::string> &actornames,
    const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections)
{
    std::vector<idx_t> rowStarts;
    std::vector<idx_t> colIndices;

    size_t currentrow = 0;
    size_t elcount = 0;
    size_t startoffset = 0;
    rowStarts.push_back(0);
    for (auto &name : actornames)
    {
        auto el = this->orderings.find(name);
        if (el != this->orderings.end())
        {
            if (el->second == currentrow)
            {
                size_t localelcount = 0;
                for (size_t i = 0; i < connections.size(); i++)
                // for (auto &conn : connections)
                {
                    if (std::get<0>(connections[i]) == name)
                    {
                        auto t = std::get<2>(connections[i]);
                        for (auto &l : this->orderings)
                        {
                            if (l.first == t)
                            {
                                localelcount += 1;
                                colIndices.push_back(l.second);
                            }
                        }
                        startoffset += 1;
                    }
                    else
                    {
                    }
                }

                elcount += localelcount;
                rowStarts.push_back(elcount);
                currentrow += 1;
            }
        }
        else
        {
            throw std::runtime_error("illegal input");
        }
    }

    this->rowIndex = rowStarts;
    this->colIndex = colIndices;
}

void Distribution::createCompressedSparseRow(const std::vector<std::string> &nodes,
                                             const std::vector<std::tuple<std::string, std::string>> &edges)
{

    std::vector<idx_t> rowStarts;
    std::vector<idx_t> colIndices;

    size_t currentrow = 0;
    size_t elcount = 0;
    rowStarts.push_back(0);
    for (const auto &node : nodes)
    {
        auto el = this->orderings.find(node);
        if (el != this->orderings.end())
        {
            if (el->second == currentrow)
            {
                size_t localelcount = 0;
                for (size_t i = 0; i < edges.size(); i++)
                // for (auto &edge : edges)
                {
                    if (std::get<0>(edges[i]) == node)
                    {
                        localelcount += 1;
                        auto pr = this->orderings.find(std::get<1>(edges[i]));
                        if (pr != this->orderings.end())
                        {
                            colIndices.push_back(pr->second);
                        }
                        else
                        {
                            throw std::runtime_error("This should not happen.");
                        }
                    }
                }

                elcount += localelcount;
                rowStarts.push_back(elcount);
                currentrow += 1;
            }
        }
        else
        {
            throw std::runtime_error("illegal input");
        }
    }

    this->rowIndex = rowStarts;
    this->colIndex = colIndices;
}

const auto key_selector = [](const auto &pair) { return pair.first; };

void Distribution::generateMigList()
{
    std::unordered_map<std::string, upcxx::intrank_t> migListInternal;

    for (auto &el : orderings)
    {
        size_t ind = el.second;
        size_t at = partitioned[ind];
        migListInternal.emplace(el.first, at);
    }

    migs = std::move(migListInternal);
    partitioned.clear();
}

std::unordered_map<std::string, size_t>
Distribution::generateLocalWeights(const std::unordered_map<std::string, GlobalActorRef> &actor_map)
{
    std::unordered_map<std::string, size_t> costs;

    for (const auto &pr : actor_map)
    {
        if (pr.second.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *pr.second.local();
            size_t cost = ai->calc_cost();
            costs.emplace(pr.first, cost);

            // std::cout << pr.first << " has costs: " << cost << std::endl;
        }
    }

    return costs;
}

void Distribution::acceptWeights(std::unordered_map<std::string, size_t> &&remoteCosts)
{
    costs.insert(std::make_move_iterator(remoteCosts.begin()), std::make_move_iterator(remoteCosts.end()));
    numToGet += 1;
}

void Distribution::gatherActorCosts(const std::unordered_map<std::string, GlobalActorRef> &actor_map)
{
    assert(costs.empty());
    numToGet = 0;

    upcxx::barrier();

    if (upcxx::rank_me() == 0)
    {
        auto map = generateLocalWeights(actor_map);
        // std::cout << "Rank 0: generated " << map.size() << "weights" << std::endl;
        acceptWeights(std::move(map));

        assert(upcxx::rank_n() >= 0);
        while (numToGet < static_cast<unsigned int>(util::rank_n()))
        {
            // std::cout << numToGet << ", " << util::rank_n() << std::endl;
            upcxx::progress();
        }

#ifdef REPORT_MAIN_ACTIONS
        std::cout << "waiting finished" << std::endl;
#endif
    }
    else
    {
        std::unordered_map<std::string, size_t> l_costs = generateLocalWeights(actor_map);

        upcxx::future<> sent = upcxx::rpc(
            0,
            [](upcxx::dist_object<Distribution *> &remoteDist, std::unordered_map<std::string, size_t> &&remoteCosts)
            {
                // std::cout << "Rank 0: got " << remoteCosts.size() << " weights" << std::endl;
                (*remoteDist)->acceptWeights(std::move(remoteCosts));
                // std::cout << (*remoteDist)->numToGet << ", " << upcxx::rank_n() << std::endl;
            },
            remoteDistribution, std::move(l_costs));

        upcxx::discharge();
        sent.wait();
    }

#ifdef REPORT_MAIN_ACTIONS
    std::cout << upcxx::rank_me() << " return from gatherActorCosts" << std::endl;
#endif
}

void Distribution::redistributeActors(const std::unordered_map<std::string, GlobalActorRef> &actor_map,
                                      const std::vector<std::tuple<std::string, std::string>> &connections)
{
    assert(migs.empty());
    assert(vertexWeights.empty());
    assert(migs.empty());      // the migration map, set in case of a global redistribution
    assert(orderings.empty()); // maps actor names to an id for use in metis
    assert(costs.empty());
    assert(colIndex.empty()); // adjncy argument of metis
    assert(rowIndex.empty());

    gatherActorCosts(actor_map);

    upcxx::barrier();

    if (upcxx::rank_me() == 0)
    {
        assert(!actor_map.empty());
        std::vector<std::string> keys(actor_map.size());
        std::transform(actor_map.begin(), actor_map.end(), keys.begin(), key_selector);

        vertexcount = keys.size();
        assert(vertexcount > 0);

        generateOrdering(keys);
        generateWeights(keys, actor_map);
        createCompressedSparseRow(keys, connections);

        partitioned = distribute();

        generateMigList();

        upcxx::promise<> allDone;

        for (int i = 1; i < upcxx::rank_n(); i++)
        {
            if (i == upcxx::rank_me())
            {
                continue;
            }
            auto cx = upcxx::operation_cx::as_promise(allDone);
            upcxx::rpc(
                i, cx,
                [](upcxx::dist_object<Distribution *> &rd, std::unordered_map<std::string, upcxx::intrank_t> &&migList)
                { (*rd)->accept(std::move(migList)); },
                remoteDistribution, migs);
        }

        upcxx::discharge();

        allDone.finalize().wait();
    }
}

void Distribution::generateWeights(const std::vector<std::string> &actornames,
                                   const std::unordered_map<std::string, GlobalActorRef> &actor_map)
{
    if (actor_map.size() != costs.size())
    {
        throw std::runtime_error(std::to_string(actor_map.size()) + " != " + std::to_string(costs.size()));
    }
    // set the weights array to right size
    assert(vertexWeights.empty());
    vertexWeights.resize(actornames.size());
    std::fill_n(vertexWeights.begin(), actornames.size(), 1);

    for (size_t i = 0; i < actornames.size(); i++)
    {
        const std::string &an = actornames[i];

        size_t cost = costs.find(an)->second;

        size_t index = this->orderings.find(an)->second;

        this->vertexWeights[index] = cost;
    }
}

void Distribution::distributeActors(
    const std::vector<std::string> &actornames,
    const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections)
{
    vertexcount = actornames.size();

    if (upcxx::rank_me() == 0)
    {

        generateOrdering(actornames);
        std::fill_n(std::back_inserter(vertexWeights), actornames.size(), 1);
        createCompressedSparseRow(actornames, connections);

#ifdef INVASION
        partitioned = distribute(0.5);
#else
        partitioned = distribute(1.0);
#endif

        upcxx::promise<> allDone;
        for (int i = 1; i < upcxx::rank_n(); i++)
        {
            if (upcxx::rank_me() == i)
            {
                continue;
            }

            auto cx = upcxx::operation_cx::as_promise(allDone);

            upcxx::rpc(
                i, cx,
                [](upcxx::dist_object<Distribution *> &rd, std::vector<upcxx::intrank_t> partitioned)
                { (*rd)->accept(std::move(partitioned)); },
                remoteDistribution, partitioned);
        }

        upcxx::discharge();
        allDone.finalize().wait();
    }

    clearDataKeepPartitioningAndMigList();
}

std::vector<upcxx::intrank_t> Distribution::movePartitioning() { return std::move(partitioned); }

Distribution::~Distribution() = default;

void Distribution::clearData()
{
    vertexcount = 0;
    orderings.clear();
    colIndex.clear();
    rowIndex.clear();
    vertexWeights.clear();
    costs.clear();
    numToGet = 0;

    partitioned.clear();
    migs.clear();
}

void Distribution::clearDataKeepPartitioningAndMigList()
{
    vertexcount = 0;
    orderings.clear();
    colIndex.clear();
    rowIndex.clear();
    vertexWeights.clear();
    costs.clear();
    numToGet = 0;
}

void Distribution::redistributeActors(const PortGraph &pg, const ActorGraph &ag)
{
#ifdef REPORT_MAIN_ACTIONS
    std::cout << upcxx::rank_me() << ": redistribute actor graph, repartitioning phase" << std::endl;
#endif

    auto start = std::chrono::high_resolution_clock::now();

    migs.clear();
    partitioned.clear();

    std::vector<std::tuple<std::string, std::string>> edges = pg.getEdges();

    const std::vector<std::tuple<std::string, std::string>> &edgesRef = edges;
    const std::unordered_map<std::string, GlobalActorRef> &vertices = ag.getActorsRef();

    const std::vector<std::string> &nodes = pg.getNodes();

    size_t v1 = vertices.size();
    size_t v2 = nodes.size();
    int iter = 0;

    while (v1 != v2)
    {
        upcxx::progress();
        v1 = vertices.size();
        v2 = nodes.size();

        ++iter;
        if (iter > util::too_long)
        {
            throw std::runtime_error("Loop too long! Distribution");
        }
    }

    upcxx::barrier();

    redistributeActors(vertices, edgesRef);

    upcxx::barrier();

    clearDataKeepPartitioningAndMigList();

    auto end = std::chrono::high_resolution_clock::now();
    size_t elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

    if (upcxx::rank_me() == 0)
    {
        std::cout << upcxx::rank_me() << ": redistribution took " << util::nanoToSec(elapsed) << " seconds "
                  << std::endl;
    }
}

std::unordered_map<std::string, upcxx::intrank_t> Distribution::moveMigrationList() { return std::move(migs); }

void Distribution::clearMigList() { migs.clear(); }

// const std::unordered_map<std::string, upcxx::intrank_t> &Distribution::getMigrationListRef() const { return migs; }

void Distribution::accept(std::vector<upcxx::intrank_t> &&other) { this->partitioned = std::move(other); }

void Distribution::accept(std::unordered_map<std::string, upcxx::intrank_t> &&other) { this->migs = std::move(other); }