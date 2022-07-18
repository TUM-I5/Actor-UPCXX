#include "DynamicActorGraph.hpp"
#include "InPort.hpp"

std::vector<std::pair<upcxx::intrank_t, std::vector<GlobalActorRef>>>
pack_into_ranks(const std::vector<GlobalActorRef> &neighbors)
{
    std::vector<std::pair<upcxx::intrank_t, std::vector<GlobalActorRef>>> v;

    for (GlobalActorRef n : neighbors)
    {
        auto r = n.where();
        bool inserted = false;
        for (auto &pr : v)
        {
            if (pr.first == r)
            {
                pr.second.push_back(n);
                inserted = true;
            }
        }
        if (!inserted)
        {
            std::vector<GlobalActorRef> v2;
            v2.push_back(n);
            v.push_back(std::make_pair(r, std::move(v2)));
        }
    }

    return v;
}

std::pair<size_t, size_t> get_index(const std::string &name)
{
    std::string x;
    std::string y;
    size_t t = name.find('-');
    x = name.substr(0, t);
    y = name.substr(t + 1, name.size());

    std::stringstream ss1;
    std::stringstream ss2;
    unsigned int a;
    unsigned int b;

    ss1 << x;
    ss2 << y;
    ss1 >> a;
    ss2 >> b;

    return std::make_pair(a, b);
}

void print_matrix(const std::vector<std::vector<unsigned int>> &vec)
{
    std::vector<unsigned int> row_max;
    for (const auto &row : vec)
    {
        row_max.push_back(*std::max_element(row.begin(), row.end()));
    }

    unsigned int glob_max = *std::max_element(row_max.begin(), row_max.end());

    size_t digits = std::to_string(glob_max).size();

    std::stringstream ss;

    ss << "{" << std::endl;
    for (size_t i = 0; i < vec.size(); i++)
    {
        ss << "  {";
        for (size_t j = 0; j < vec[i].size() - 1; j++)
        {
            unsigned int el = vec[i][j];
            size_t local_digits = std::to_string(el).size();

            size_t padding = digits - local_digits;

            for (size_t k = 0; k < padding + 1; k++)
            {
                ss << " ";
            }

            ss << el;
            ss << ",";
        }

        unsigned int el = vec[i][vec[i].size() - 1];
        size_t local_digits = std::to_string(el).size();

        size_t padding = digits - local_digits;

        for (size_t k = 0; k < padding + 1; k++)
        {
            ss << " ";
        }

        ss << el;
        ss << "}" << std::endl;
    }
    ss << "}";
    std::cout << ss.str() << std::endl;
    std::cerr << ss.str() << std::endl;
}

void DynamicActorGraph::print_gitter(const std::unordered_map<std::string, GlobalActorRef> &actormap)
{
    std::vector<std::vector<unsigned int>> gitter;
    std::vector<unsigned int> xs;
    std::vector<unsigned int> ys;

    for (const auto &pr : actormap)
    {
        const std::string &name = pr.first;
        std::pair<unsigned int, unsigned int> location = get_index(name);
        xs.push_back(location.first);
        ys.push_back(location.second);
    }

    unsigned int xmax = *std::max_element(xs.begin(), xs.end());
    unsigned int ymax = *std::max_element(ys.begin(), ys.end());
    // unsigned int xmin = *std::min_element(xs.begin(), xs.end());
    // unsigned int ymin = *std::min_element(ys.begin(), ys.end());

    // std::cout << "x: " << xmax << " " << " y:" << ymax << std::endl;
    // std::cout << "x: " << xmin << " " << " y:" << ymin << std::endl;

    for (unsigned int i = 0; i < ymax + 1; i++)
    {
        gitter.push_back(std::vector<unsigned int>(xmax + 1, 0));
    }

    unsigned int offset = 0;
    for (const auto &pr : actormap)
    {
        unsigned int r = pr.second.where();
        gitter[ys[offset]][xs[offset]] = r;
        offset += 1;
    }

    print_matrix(gitter);
}

void DynamicActorGraph::print_gitter_mark_pin()
{
    std::vector<std::vector<unsigned int>> gitter;
    std::vector<std::vector<unsigned int>> gitter_of_active_status;
    std::vector<unsigned int> xs;
    std::vector<unsigned int> ys;

    const auto &actormap = this->ag.getActorsRef();
    for (const auto &pr : actormap)
    {
        const std::string &name = pr.first;
        std::pair<unsigned int, unsigned int> location = get_index(name);
        xs.push_back(location.first);
        ys.push_back(location.second);
    }

    unsigned int xmax = *std::max_element(xs.begin(), xs.end());
    unsigned int ymax = *std::max_element(ys.begin(), ys.end());
    // unsigned int xmin = *std::min_element(xs.begin(), xs.end());
    // unsigned int ymin = *std::min_element(ys.begin(), ys.end());

    // std::cout << "x: " << xmax << " " << " y:" << ymax << std::endl;
    // std::cout << "x: " << xmin << " " << " y:" << ymin << std::endl;

    for (unsigned int i = 0; i < ymax + 1; i++)
    {
        gitter.push_back(std::vector<unsigned int>(xmax + 1, 4));
    }

    unsigned int offset = 0;
    for (const auto &pr : actormap)
    {
        int active = 0;
        if (pr.second.where() == upcxx::rank_me())
        {
            auto marked = (*pr.second.local())->isMarked();
            auto pinned = (*pr.second.local())->isPinned();

            if (marked && pinned)
            {
                active = 3;
            }
            else if (marked && !pinned)
            {
                active = 1;
            }
            else if (pinned && !marked)
            {
                active = 2;
                std::cerr << ys[offset] << "-" << xs[offset] << ": 2" << std::endl;
            }
            else
            {
                active = 0;
            }
        }
        else
        {
            auto el = upcxx::rpc(
                          pr.second.where(),
                          [](GlobalActorRef ref)
                          { return std::make_pair((*ref.local())->isMarked(), (*ref.local())->isPinned()); },
                          pr.second)
                          .wait();
            auto marked = el.first;
            auto pinned = el.second;
            if (marked && pinned)
            {
                active = 3;
            }
            else if (marked && !pinned)
            {
                active = 1;
            }
            else if (pinned && !marked)
            {
                active = 2;
                std::cerr << ys[offset] << "-" << xs[offset] << ": 2" << std::endl;
            }
            else
            {
                active = 0;
            }
        }

        gitter[ys[offset]][xs[offset]] = active;
        offset += 1;
    }

    std::cout << upcxx::rank_me() << ": mark and pin status\n";
    print_matrix(gitter);
}

void DynamicActorGraph::print_gitter_active_status()
{
    std::vector<std::vector<unsigned int>> gitter;
    std::vector<std::vector<unsigned int>> gitter_of_active_status;
    std::vector<unsigned int> xs;
    std::vector<unsigned int> ys;

    const auto &actormap = this->ag.getActorsRef();
    for (const auto &pr : actormap)
    {
        const std::string &name = pr.first;
        std::pair<unsigned int, unsigned int> location = get_index(name);
        xs.push_back(location.first);
        ys.push_back(location.second);
    }

    unsigned int xmax = *std::max_element(xs.begin(), xs.end());
    unsigned int ymax = *std::max_element(ys.begin(), ys.end());
    // unsigned int xmin = *std::min_element(xs.begin(), xs.end());
    // unsigned int ymin = *std::min_element(ys.begin(), ys.end());

    // std::cout << "x: " << xmax << " " << " y:" << ymax << std::endl;
    // std::cout << "x: " << xmin << " " << " y:" << ymin << std::endl;

    for (unsigned int i = 0; i < ymax + 1; i++)
    {
        gitter.push_back(std::vector<unsigned int>(xmax + 1, 0));
    }

    unsigned int offset = 0;
    for (const auto &pr : actormap)
    {
        int active = 0;
        if (pr.second.where() == upcxx::rank_me())
        {
            auto state = (*pr.second.local())->getRunningState();
            if (state == ActorState::Running)
            {
                active = 1;
            }
            if (state == ActorState::TemporaryStoppedForMigration)
            {
                active = 3;
            }
            if (state == ActorState::Transitioning)
            {
                active = 4;
            }
            if (state == ActorState::NeedsActivation)
            {
                active = 5;
            }
            if (state == ActorState::Terminated)
            {
                active = 6;
            }
        }
        else
        {
            auto state =
                upcxx::rpc(
                    pr.second.where(), [](GlobalActorRef ref) { return (*ref.local())->getRunningState(); }, pr.second)
                    .wait();
            if (state == ActorState::Running)
            {
                active = 1;
            }
            if (state == ActorState::TemporaryStoppedForMigration)
            {
                active = 3;
            }
            if (state == ActorState::Transitioning)
            {
                active = 4;
            }
            if (state == ActorState::NeedsActivation)
            {
                active = 5;
            }
            if (state == ActorState::Terminated)
            {
                active = 6;
            }
        }

        gitter[ys[offset]][xs[offset]] = active;
        offset += 1;
    }

    if (upcxx::rank_me() == 0)
    {
        std::cout << "Printing gitter running status:" << std::endl;
    }

    std::cout << upcxx::rank_me() << ": active status\n";
    print_matrix(gitter);

    print_gitter_mark_pin();
}

#ifdef TIME
void DynamicActorGraph::printTimeInfo()
{
    std::stringstream ss;
    auto aps = ag.getActors();
    for (auto pr : *aps)
    {
        GlobalActorRef ref = pr.second;
        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *aimpl = *ref.local();
            ss << aimpl->getTimeInfo();
        }
    }
    std::cout << ss.str() << std::endl;
}
#endif

size_t DynamicActorGraph::getMarkOffset()
{
    for (size_t i = 0; i < pins.size(); i++)
    {
        if (pins[i].first == false)
        {
            pins[i].first = true;
            pins[i].second.clear();
            return i;
        }
    }

    std::unordered_map<GlobalActorRef, bool> attmps;

    pins.push_back(std::make_pair(true, std::move(attmps)));
    // used.push_back(true);

    return pins.size() - 1;
}

void DynamicActorGraph::releaseMarkOffset(size_t offset)
{
    pins[offset].second.clear();
    pins[offset].first = false;
}

upcxx::future<> DynamicActorGraph::initConnections(
    const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections)
{
    // initialize connections
    std::vector<upcxx::future<>> futs;
    int myrank = upcxx::rank_me();

    for (const auto &tup : connections)
    {
        std::string from = std::get<0>(tup);
        std::string op = std::get<1>(tup);
        std::string to = std::get<2>(tup);
        std::string ip = std::get<3>(tup);

        GlobalActorRef ref = this->ag.getActorNoThrow(from);
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during init connecitons");
        }
        if (ref.where() == myrank)
        {
            GlobalActorRef otherref = this->ag.getActorNoThrow(to);
            if (otherref == nullptr)
            {
                throw std::runtime_error("Actor should be present in graph during init connections (2)");
            }
            rpc_on_wait += 2;
            upcxx::future<> con = this->ag.connectPortsAsync(ref, op, otherref, ip);
            upcxx::future<> con_w = con.then([this]() { rpc_on_wait -= 1; });
            // this->ag.connectPortsAsync(ref, op, otherref, ip);
            // upcxx::future<> con2 = this->pg.insertEdgeAsync(from, op, to, ip);
            // upcxx::future<> con2_w = con2.then([this]() { rpc_on_wait -= 1; });

            futs.push_back(std::move(con_w));
            // futs.push_back(std::move(con2_w));
        }
    }

    // if (upcxx::rank_me() % 4 == 0)
    //{
    //    std::cout << upcxx::rank_me() << ": created asyns now combine future" << std::endl;
    //}

    upcxx::future<> combined = util::combineFutures(std::move(futs));

    // if (upcxx::rank_me() % 4 == 0)
    //{
    //    std::cout << upcxx::rank_me() << ": combined futures" << std::endl;
    //}

    return combined;
}

void DynamicActorGraph::initConnectionsSync(
    const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections)
{
    // initialize connections
    int myrank = upcxx::rank_me();

    for (const auto &tup : connections)
    {
        std::string from = std::get<0>(tup);
        std::string op = std::get<1>(tup);
        std::string to = std::get<2>(tup);
        std::string ip = std::get<3>(tup);

        GlobalActorRef ref = this->ag.getActorNoThrow(from);
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during init connections");
        }
        if (ref.where() == myrank)
        {
            GlobalActorRef otherref = this->ag.getActorNoThrow(to);
            if (otherref == nullptr)
            {
                throw std::runtime_error("Actor should be present in graph during init connections");
            }
            rpc_on_wait += 2;
            this->ag.connectPortsAsync(ref, op, otherref, ip).wait();
            this->pg.insertEdgeAsync(from, op, to, ip).wait();
        }
    }
}

double DynamicActorGraph::run()
{
    double interval = std::numeric_limits<float>::max();
    size_t iter = 0;
    double timeInMigrationPhase = 0.0;
    auto computationBegin = util::timepoint();

    if constexpr (config::migrationtype == config::MigrationType::BULK || config::interrupt)
    {
        try
        {
            const char *env_p = std::getenv("REPARTITIONING_INTERVAL");
            if (env_p != nullptr)
            {
                float num_float = std::stof(env_p);
                interval = num_float;
            }
            else
            {
                interval = 50.f;
            }
        }
        catch (...)
        {
            interval = 50.f;
        }
    }

    if constexpr (config::migrationtype == config::MigrationType::ASYNC)
    {
        interval = std::numeric_limits<double>::max();
    }

    if (upcxx::rank_me() == 0)
    {
        if constexpr (config::migrationtype == config::MigrationType::BULK)
        {
            std::cout << "Bulk Migration only" << std::endl;
            std::cout << "Repartitioning time interval: " << interval << std::endl;
        }
        else if constexpr (config::migrationtype == config::MigrationType::ASYNC)
        {
            std::cout << "Async Migration only" << std::endl;
        }
        else if constexpr (config::migrationtype == config::MigrationType::NO)
        {
            std::cout << "No Migration" << std::endl;
        }

#ifdef REPORT_MAIN_ACTIONS
        print_gitter(this->ag.getActorsRef());
#endif
    }

    bool finished = false;
    double totalRunTime = 0.0;
    int runcalls = 0;
    bool firsttime = true;

    while (!finished)
    {
#ifdef REPORT_MAIN_ACTIONS
        std::cout << upcxx::rank_me() << ": call run()" << std::endl;
#endif

        auto x = this->ag.run(interval, firsttime);
        interrupts += 1;
        firsttime = false;
        runcalls += 1;
        // upcxx::progress();
        finished = upcxx::reduce_all(x.second, upcxx::experimental::op_bit_and).wait();
        totalRunTime += x.first;

        std::function<void()> migPhase = [this]() { this->phases(); };

        if constexpr (config::migrationtype == config::MigrationType::BULK)
        {
            auto tbegin = std::chrono::steady_clock::now();

            if (!finished)
            {
                upcxx::barrier();

                if (upcxx::rank_me() == 0)
                {
                    std::cout << upcxx::rank_me() << ": global repartition phase begins" << std::endl;
                }

                util::runWithTimer(migPhase, timeInMigrationPhase);
            }

            auto tend = std::chrono::steady_clock::now();
            size_t elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(tend - tbegin).count();
            double secs = util::nanoToSec(elapsed);

            if (upcxx::rank_me() == 0)
            {
                std::cout << upcxx::rank_me() << ": global repartition " << iter << " phase took " << secs << " seconds"
                          << std::endl;
            }
            iter += 1;
        }
    }

#ifdef TIME
    printTimeInfo();
#endif

    return totalRunTime;
}

void DynamicActorGraph::phases()
{
    if (upcxx::rank_me() == 0)
    {
        std::cout << upcxx::rank_me() << ": "
                  << "graph repartitioning begins" << std::endl;
    }

    static int counter = 0;
    auto start = std::chrono::high_resolution_clock::now();
    actorDistribution.redistributeActors(pg, ag);
    upcxx::barrier();
    std::unordered_map<std::string, upcxx::intrank_t> migs = actorDistribution.moveMigrationList();
#ifdef REPORT_MAIN_ACTIONS
    std::cout << util::migrationList2Str(upcxx::rank_me(), migs) << std::endl;
#endif

    /*
        Migration sends actors to the ranks found by the partitioner, of course if they are marked or pinned due to
       asynchronous migration they should not be migrated and removed from the list, also if their old rank and new rank
       are the same then we do not migrate
    */

    if (!migs.empty())
    {
        if (upcxx::rank_me() == 0)
        {
            std::cout << upcxx::rank_me() << ": "
                      << "migration begins" << std::endl;
        }

        migrateActorsDiscretePhases(std::move(migs));
    }
#ifndef NDEBUG
    else
    {
        if (!upcxx::rank_me())
        {
            std::cout << "Migration List Empty." << std::endl;
        }
    }
#endif

    actorDistribution.clearMigList();

    auto end = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double, std::milli>(end - start).count();
#ifdef REPORT_MAIN_ACTIONS
    std::string s = "Migration phase " + std::to_string(counter) + " for rank: " + std::to_string(upcxx::rank_me()) +
                    " took " + std::to_string(elapsed) +
                    " milliseconds, has: " + std::to_string(this->ag.getActiveActors()) + +" | " +
                    std::to_string(this->ag.localActors.size()) + " active actors.";
    std::cout << s << std::endl;
#endif
    counter += 1;
}

upcxx::future<>
DynamicActorGraph::disconnectFromNeighboursAsync(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    std::vector<upcxx::future<>> futs;
    for (const auto &pr : migList)
    {
        GlobalActorRef actorre = this->ag.getActorNoThrow(pr.first);
        if (actorre == nullptr)
        {
            throw std::runtime_error("Actor should be inside map in disconnect from neighbors async");
        }

        if (actorre.where() == upcxx::rank_me())
        {
            upcxx::future<> f = disconnectFromNeighboursAsync(pr.first);
            futs.emplace_back(std::move(f));
        }
    }
    return util::combineFutures(std::move(futs));
}

upcxx::future<> DynamicActorGraph::disconnectFromNeighboursAsync(const std::string &name)
{
    GlobalActorRef addedactor = this->ag.getActorNoThrow(name);
    if (addedactor == nullptr)
    {
        throw std::runtime_error("Actor should be inside map in disconnect from neighbors async");
    }
    // outportname,otheractor,inportname
    auto out = this->pg.getOutgoing(name);
    // return order: inportname, otheractor name, other actors ourport name
    auto in = this->pg.getIncoming(name);

    std::vector<upcxx::future<>> futs;
    futs.reserve(out.size() + in.size());

    for (size_t i = 0; i < out.size(); i++)
    {
        GlobalActorRef arec = this->ag.getActorNoThrow(std::get<1>(out[i]));
        if (arec == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during disconnectfromneighbors");
        }
        upcxx::future<> dc = this->ag.disconnectPortsAsync(addedactor, std::get<0>(out[i]), arec, std::get<2>(out[i]));
        futs.push_back(std::move(dc));
#ifdef REPORT_MAIN_ACTIONS
        // std::cout << "(this->other) disconnect: " << name << "'s " << std::get<0>(out[i]) << " to "
        //           << std::get<1>(out[i]) << "'s " << std::get<2>(out[i]) << std::endl;
#endif
    }

    for (size_t i = 0; i < in.size(); i++)
    {
        GlobalActorRef asend = this->ag.getActorNoThrow(std::get<1>(in[i]));
        if (asend == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during disconnectfromneighbors");
        }
        upcxx::future<> dc = this->ag.disconnectPortsAsync(asend, std::get<2>(in[i]), addedactor, std::get<0>(in[i]));
        futs.push_back(std::move(dc));
#ifdef REPORT_MAIN_ACTIONS
        // std::cout << "(other->this) disconnect: " << std::get<1>(in[i]) << "'s " << std::get<2>(in[i]) << " to " <<
        // name
        //           << "'s " << std::get<0>(in[i]) << std::endl;
#endif
    }

    upcxx::future<> all = util::combineFutures(std::move(futs));
    return all;
}

upcxx::future<>
DynamicActorGraph::reconnectToNeighboursAsync(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    std::vector<upcxx::future<>> futs;

    for (const auto &pr : migList)
    {
        GlobalActorRef actorre = this->ag.getActorNoThrow(pr.first);
        if (actorre == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during init connections");
        }
        if (actorre.where() == upcxx::rank_me())
        {
            upcxx::future<> f = reconnectToNeighboursAsync(pr.first);
            futs.push_back(std::move(f));
        }
    }
    upcxx::future<> f = util::combineFutures(std::move(futs));
    return f;
}

upcxx::future<> DynamicActorGraph::reconnectToNeighboursAsync(const std::set<GlobalActorRef> &migList)
{
    std::vector<upcxx::future<>> futs;

    for (const auto &el : migList)
    {
        GlobalActorRef actorre = el;

        if (actorre.where() == upcxx::rank_me())
        {
            std::string name = (*actorre.local())->getName();
            upcxx::future<> f = reconnectToNeighboursAsync(name);
            futs.push_back(std::move(f));
        }
    }
    upcxx::future<> f = util::combineFutures(std::move(futs));
    return f;
}

upcxx::future<> DynamicActorGraph::reconnectToNeighboursAsync(const std::string &name)
{
    // std::cout << "reconnect " << name << " to its neighbours" << std::endl;

    GlobalActorRef addedactor = this->ag.getActorNoThrow(name);
    if (addedactor == nullptr)
    {
        throw std::runtime_error("Actor should be present in graph during reconnectToNeighbors");
    }
    // outportname,otheractor,inportname
    auto out = this->pg.getOutgoing(name);
    // return order: inportname, otheractor name, other actors ourport name
    auto in = this->pg.getIncoming(name);

    std::vector<upcxx::future<>> futs;
    futs.reserve(out.size() + in.size());

    for (size_t i = 0; i < out.size(); i++)
    {
        GlobalActorRef arec = this->ag.getActorNoThrow(std::get<1>(out[i]));
        if (arec == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during reconnectneighbors");
        }
        upcxx::future<> fut = this->ag.connectPortsAsync(addedactor, std::get<0>(out[i]), arec, std::get<2>(out[i]));
        futs.push_back(fut);
        // std::cout << "(this->other) reconnect: " << name << "'s " << std::get<0>(out[i]) << " to " <<
        // std::get<1>(out[i]) << "'s " << std::get<2>(out[i])
        //          << std::endl;
    }

    for (size_t i = 0; i < in.size(); i++)
    {
        GlobalActorRef asend = this->ag.getActorNoThrow(std::get<1>(in[i]));
        if (asend == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during reconnectneighbors (2)");
        }
        upcxx::future<> fut = this->ag.connectPortsAsync(asend, std::get<2>(in[i]), addedactor, std::get<0>(in[i]));
        futs.push_back(fut);

        // std::cout << "(other->this) reconnect: " << std::get<1>(in[i]) << "'s " << std::get<2>(in[i]) << " to " <<
        // name << "'s " << std::get<0>(in[i])
        //         << std::endl;
    }

    upcxx::future<> all = util::combineFutures(futs);
    return all;
}

void DynamicActorGraph::connectPorts(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                     GlobalActorRef destinationActor, const std::string &destinationPortName)
{
    throw std::runtime_error("Call async variant of this function!");
    this->connectPortsAsync(sourceActor, sourcePortName, destinationActor, destinationPortName).wait();
}

upcxx::future<> DynamicActorGraph::connectPortsAsync(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                                     GlobalActorRef destinationActor,
                                                     const std::string &destinationPortName)
{
    auto connectFut = this->ag.connectPortsAsync(sourceActor, sourcePortName, destinationActor, destinationPortName);
    return connectFut;
}

upcxx::future<> DynamicActorGraph::disconnectPortsAsync(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                                        GlobalActorRef destinationActor,
                                                        const std::string &destinationPortName)
{
    upcxx::future<> connectFut =
        this->ag.disconnectPortsAsync(sourceActor, sourcePortName, destinationActor, destinationPortName);

    return connectFut;
}

void DynamicActorGraph::connectPorts(const std::string &sourceActorName, const std::string &sourcePortName,
                                     const std::string &destinationActorName, const std::string &destinationPortName)
{
    throw std::runtime_error("Call async variant of this function!");
    connectPortsAsync(sourceActorName, sourcePortName, destinationActorName, destinationPortName).wait();
}

upcxx::future<> DynamicActorGraph::connectPortsAsync(const std::string &sourceActorName,
                                                     const std::string &sourcePortName,
                                                     const std::string &destinationActorName,
                                                     const std::string &destinationPortName)
{
    GlobalActorRef sa = ag.getActorNoThrow(sourceActorName);
    GlobalActorRef da = ag.getActorNoThrow(destinationActorName);
    if (sa == nullptr || da == nullptr)
    {
        throw std::runtime_error("Actor should be present in graph during connectPorts");
    }

    return this->connectPortsAsync(sa, sourcePortName, da, destinationPortName);
}

std::string DynamicActorGraph::prettyPrint()
{
    auto s = ag.prettyPrint();
    s += pg.toStr();
    s += ("Count of active actors on rank: " + std::to_string(upcxx::rank_me()) + " = " +
          std::to_string(ag.getActiveActors()));
    return s;
}

upcxx::future<> DynamicActorGraph::addActorToAnotherAsync(GlobalActorRef a, upcxx::intrank_t rank)
{
    return this->ag.addActorToAnotherAsync(a, rank);
}

void DynamicActorGraph::addActor(ActorImpl *a)
{

    throw std::runtime_error("Call async variant of this function!");
    ag.addActor(a);
    pg.insertNode(a->getName(), upcxx::rank_me());
    // std::cout << "add " << a->getName() << std::endl;
}

void DynamicActorGraph::addActor(Actor *a)
{
    throw std::runtime_error("Call async variant of this function!");
    ag.addActor(a);
    pg.insertNode(a->getName(), upcxx::rank_me());
    // std::cout << "add " << a->getName() << std::endl;
}

upcxx::future<> DynamicActorGraph::addActorAsync(ActorImpl *a)
{
    rpc_on_wait += 2;
    upcxx::future<> fut1 = this->ag.addActorAsync(a);
    upcxx::future<> fut1_w = fut1.then([this]() { rpc_on_wait -= 1; });
    upcxx::future<> fut2 = pg.insertNodeAsync(a->getName(), upcxx::rank_me());
    upcxx::future<> fut2_w = fut2.then([this]() { rpc_on_wait -= 1; });
    // std::cout << "add " << a->getName() << std::endl;
    return upcxx::when_all(std::move(fut1_w), std::move(fut2_w));
}

upcxx::future<> DynamicActorGraph::addActorAsync(Actor *a)
{
    rpc_on_wait += 2;
    upcxx::future<> fut1 = this->ag.addActorAsync(a);
    upcxx::future<> fut1_w = fut1.then([this]() { rpc_on_wait -= 1; });
    upcxx::future<> fut2 = pg.insertNodeAsync(a->getName(), upcxx::rank_me());
    upcxx::future<> fut2_w = fut2.then([this]() { rpc_on_wait -= 1; });
    // std::cout << "add " << a->getName() << std::endl;
    return upcxx::when_all(std::move(fut1_w), std::move(fut2_w));
}

void DynamicActorGraph::disconnectActors(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    for (const auto &pr : migList)
    {
        GlobalActorRef ref = ag.getActorNoThrow(pr.first);
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during disconnect actors");
        }
        if (ref.where() == upcxx::rank_me())
        {
            disconnectFromNeighboursAsync(pr.first).wait();
        }
    }
}

bool DynamicActorGraph::tryToAcquireLockForActor(const std::string name, const upcxx::intrank_t marker,
                                                 float workofother)
{
    float work = (time_spent_for_cost) ? static_cast<float>(this->ag.getWorkDoneForMigration())
                                       : static_cast<float>(this->ag.getTaskCount());

    if (work < allowed_imbalance * workofother)
    {
        return false;
    }

    if (this->ag.getLocalActorsRef().size() <= 1)
    {
        return false;
    }

    if (!steal_during_termination && this->ag.has_a_terminated_actor)
    {
        return false;
    }

    if (going_away_limit != std::numeric_limits<size_t>::max() && going_away >= going_away_limit)
    {
        return false;
    }

    GlobalActorRef ref = this->ag.getActorNoThrow(name);

    if (ref == nullptr || ref.where() == marker || ref.where() != upcxx::rank_me())
    {
        return false;
    }

    ActorImpl *ai = *ref.local();

    if (ai->getRunningState() == ActorState::Terminated || ai->isMarked() || ai->isPinned() || !ai->actable())
    {
        return false;
    }

    /*
Running,
NeedsActivation,
TemporaryStoppedForMigration,
Terminated,
UnmigratableAndStopped,
Transitioning
*/

    if (!this->ag.taskDeque.canWork())
    {
        return false;
    }

    bool could = ai->mark(marker);

    if (!could)
    {
        return false;
    }

    going_away += 1;
    stealing_from_me.insert(marker);
    return true;
}

upcxx::future<bool> DynamicActorGraph::tryToMarkActorForMigration(const std::string name)
{
    if (name.empty())
    {
        throw std::runtime_error("tryToMarkActorForMigration input name empty");
    }

    GlobalActorRef ref = this->ag.getActorNoThrow(name);

    if (ref == nullptr)
    {
        return upcxx::make_future(false);
    }

    int at = ref.where();

    if (at == upcxx::rank_me())
    {
        return upcxx::make_future(false);
        // throw std::runtime_error("Don't steal an actor that is already on your rank!");
    }

    float work = (time_spent_for_cost) ? static_cast<float>(this->ag.getWorkDoneForMigration())
                                       : static_cast<float>(this->ag.getTaskCount());

    upcxx::future<bool> locked = upcxx::rpc(
        at,
        [](upcxx::dist_object<DynamicActorGraph *> &rdag, std::string name, upcxx::intrank_t marker, float work)
        { return (*rdag)->tryToAcquireLockForActor(name, marker, work); },
        remoteComponents, std::move(name), upcxx::rank_me(), work);

    return locked;
}

// Temporary Termination with state
upcxx::future<bool> DynamicActorGraph::tryStopSelfAndPinNeighborhood(const std::string name)
{

    GlobalActorRef ref = this->ag.getActorNoThrow(name);
    if (ref == nullptr)
    {
        throw std::runtime_error("Actor should be present in graph during tryStop");
    }

    // stop all neighbors
    std::set<std::string> neighbors_ = this->pg.getNeighbors(name);
    std::vector<std::string> neighbors;
    std::copy(neighbors_.begin(), neighbors_.end(), std::back_inserter(neighbors));

    if (neighbors.size() != neighbors_.size())
    {
        throw std::runtime_error("copying from set to vector changed the size??");
    }

    if (neighbors.size() == 0)
    {
        throw std::runtime_error("No no no, no actors without neighbors please!");
    }

// else we try to pin neighbors and then try to stop the main actor
#ifdef REPORT_MAIN_ACTIONS
    std::cerr << "pin neighbors of " << name << std::endl;
#endif
    std::tuple<upcxx::future<bool>, size_t> tup = pin_neighbors(neighbors);
    upcxx::future<bool> pinned = std::get<0>(tup);
    size_t pins_at_offset = std::get<1>(tup);

    // try to stop the main actor now, since neighbors are pinned (or not pinned)
    // return pinned | stopped | actorstate of the actor we want to terminate
    upcxx::future<std::tuple<bool, bool>> to_ret = pinned.then(
        [this, ref, neighbors, pins_at_offset](bool _pinned) -> upcxx::future<std::tuple<bool, bool>>
        {
            if (!_pinned)
            {
                std::tuple<bool, bool> tup = std::make_tuple(false, false);
                upcxx::future<std::tuple<bool, bool>> ret = upcxx::make_future(tup);
                return ret;
            }
            else
            {
                if (ref.where() != upcxx::rank_me())
                {
                    upcxx::future<std::tuple<bool, bool>> mainstopped = upcxx::rpc(
                        ref.where(),
                        [](GlobalActorRef ref, upcxx::intrank_t rank) -> std::tuple<bool, bool>
                        {
                            ActorImpl *ai = *ref.local();
                            return std::make_tuple(ai->stopWithCaller(ActorState::TemporaryStoppedForMigration, rank),
                                                   true);
                        },
                        ref, upcxx::rank_me());

                    return mainstopped;
                }
                else
                {
                    ActorImpl *ai = *ref.local();
                    bool mainstopped = ai->stopWithCaller(ActorState::TemporaryStoppedForMigration, upcxx::rank_me());
                    std::tuple<bool, bool> tup = std::make_tuple(mainstopped, true);
                    upcxx::future<std::tuple<bool, bool>> ret = upcxx::make_future(tup);
                    return ret;
                }
            }

            throw std::runtime_error("return lambda std::tuple<bool,bool> should not reach here!");
        });

    upcxx::future<bool> may_steal = to_ret.then(
        [neighbors, ref, name, pins_at_offset, this](std::tuple<bool, bool> tup) -> upcxx::future<bool>
        {
            bool stopped = std::get<0>(tup);
            bool pinned = std::get<1>(tup);

            // stopped -> the actor we want to terminate is stopped
            // pinned -> neighbors are pinned

            if (stopped && !pinned)
            {
                throw std::runtime_error(
                    "Stopping should not return true if we have not pinned! Error in implementation!");
            }

            if (!pinned)
            {
                if (stopped)
                {
                    throw std::runtime_error("How could we stop when we could not pin?");
                }
#ifdef REPORT_MAIN_ACTIONS
                std::cerr << "unpin neighbors of " << name << std::endl;
#endif
                // upcxx::future<> unpinned = unpin_neighbors(neighbors, pins_at_offset);
                upcxx::future<> unmarked = unmark_actor(name);
                upcxx::future<bool> failed = upcxx::make_future(false);
                return upcxx::when_all(unmarked, failed);
            }
            else
            {
                if (!stopped)
                {
                    upcxx::future<> unpinned = unpin_neighbors(neighbors);
                    upcxx::future<> unmarked = unpinned.then([this, name]() { return unmark_actor(name); });
                    upcxx::future<bool> failed = unmarked.then([]() -> bool { return false; });
                    return failed;
                }
            }

            return upcxx::make_future(true);
        });

    return may_steal;
}

upcxx::future<> DynamicActorGraph::severeConnections(const std::string &name)
{
    const Node &middle = this->pg.getNode(name);
    const std::vector<std::tuple<std::string, std::string, std::string>> &outgoing =
        middle.outgoing; // connection of type op -> ip|B (saved as op,B,ip)
    const std::vector<std::tuple<std::string, std::string, std::string>> &incoming =
        middle.incoming; // connection of type C|op -> ip (saved as ip,C,op)

    upcxx::promise<> allDone;

    for (const auto &el : outgoing)
    {

        GlobalActorRef ref = this->ag.getActorNoThrow(std::get<1>(el));
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during severeconnections");
        }

        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *ref.local();
            ai->severeConnectionIn(std::get<2>(el));
        }
        else
        {
            auto cx = upcxx::operation_cx::as_promise(allDone);
            upcxx::rpc(
                ref.where(), cx,
                [](GlobalActorRef ref, std::string portName)
                {
                    ActorImpl *ai = *ref.local();
                    ai->severeConnectionIn(portName);
                },
                ref, std::get<2>(el));
        }
    }

    for (const auto &el : incoming)
    {
        GlobalActorRef ref = this->ag.getActorNoThrow(std::get<1>(el));
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during severeconnections (2)");
        }
        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *ref.local();
            ai->severeConnectionOut(std::get<2>(el));
        }
        else
        {
            auto cx = upcxx::operation_cx::as_promise(allDone);
            upcxx::rpc(
                ref.where(), cx,
                [](GlobalActorRef ref, std::string portName)
                {
                    ActorImpl *ai = *ref.local();
                    ai->severeConnectionOut(portName);
                },
                ref, std::get<2>(el));
        }
    }

    upcxx::future<> fut = allDone.finalize();
    return fut;
}

// the difference between the single severeConnections is that
// this is called during the migration phase, so if we have a neighbor that is not in the graph due to any possible
// reason, we just continue

upcxx::future<> DynamicActorGraph::severeConnections(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    std::vector<upcxx::future<>> futs;
    futs.reserve(migList.size());

    for (const auto &el : migList)
    {
        upcxx::future<> f = severeConnections(el.first);
        futs.emplace_back(std::move(f));
    }

    upcxx::future<> combined = util::combineFutures(futs);
    return combined;
}

upcxx::future<>
DynamicActorGraph::resurrectConnections(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    std::vector<upcxx::future<>> futs;
    futs.reserve(migList.size());

    for (const auto &el : migList)
    {
        const std::string &name = el.first;

        GlobalActorRef r = this->ag.getActorNoThrow(name);
        if (r == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during resurrectconnections milists");
        }
        upcxx::future<> f = upcxx::rpc(
            r.where(),
            [](upcxx::dist_object<DynamicActorGraph *> &rdag, GlobalActorRef ref, std::string name) -> upcxx::future<>
            { return (*rdag)->resurrectConnections(name, ref); },
            remoteComponents, r, el.first);

        futs.emplace_back(std::move(f));
    }

    upcxx::future<> combined = util::combineFutures(futs);
    return combined;
}

upcxx::future<> DynamicActorGraph::resurrectConnections(const std::string &name, GlobalActorRef ref)
{
    const Node &middle = this->pg.getNode(name);
    const std::vector<std::tuple<std::string, std::string, std::string>> &outgoing =
        middle.outgoing; // connection of type op -> ip|B (saved as op,B,ip)
    const std::vector<std::tuple<std::string, std::string, std::string>> &incoming =
        middle.incoming; // connection of type C|op -> ip (saved as ip,C,op)

    GlobalActorRef mainref = this->ag.getActorNoThrow(name);
    if (mainref == nullptr)
    {
        throw std::runtime_error("Actor should be present in graph during resurrectconnections");
    }
    if (mainref.where() != upcxx::rank_me())
    {
        throw std::runtime_error("The main actor should be on my rank right now! (my rank,actors rank)" +
                                 std::to_string(upcxx::rank_me()) + " " + std::to_string(mainref.where()));
    }

    ActorImpl *main = *mainref.local();

    main->set_port_size();

    upcxx::promise<> allDone;

    // bind the ports from other actor to the main actor
    for (const auto &el : outgoing)
    {
        // connection of type op -> ip|B (saved as op,B,ip)
        GlobalActorRef ref = this->ag.getActorNoThrow(std::get<1>(el));
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during resurrectconnections (2)");
        }
        AbstractOutPort *out = main->getOutPort(std::get<0>(el));
        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *ref.local();
            ai->resurrectConnection(std::get<2>(el), upcxx::rank_me(), out);
        }
        else
        {
            auto cx = upcxx::operation_cx::as_promise(allDone);
            upcxx::rpc(
                ref.where(), cx,
                [](GlobalActorRef ref, std::string portName, upcxx::intrank_t rank, AbstractOutPort *outptr)
                {
                    ActorImpl *ai = *ref.local();
                    ai->resurrectConnection(portName, rank, outptr);
                },
                ref, std::get<2>(el), upcxx::rank_me(), out);
        }
    }
    for (const auto &el : incoming)
    {
        // C|op -> ip (saved as ip,C,op)
        GlobalActorRef ref = this->ag.getActorNoThrow(std::get<1>(el));
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during resurrectconnections");
        }
        AbstractInPort *ip = main->getInPort(std::get<0>(el));
        GlobalChannelRef channelptr = ip->getChannelPtr();

        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *other = *ref.local();
            AbstractOutPort *out = other->getOutPort(std::get<2>(el));

            other->resurrectConnection(std::get<2>(el), channelptr);
        }
        else
        {
            auto cx = upcxx::operation_cx::as_promise(allDone);
            upcxx::rpc(
                ref.where(), cx,
                [](GlobalActorRef ref, std::string portName, GlobalChannelRef channelptr)
                {
                    ActorImpl *other = *ref.local();
                    AbstractOutPort *out = other->getOutPort(portName);
                    other->resurrectConnection(portName, channelptr);
                },
                ref, std::get<2>(el), channelptr);
        }
    }

    std::vector<upcxx::future<>> futs;
    for (const auto &el : outgoing)
    {
        // op -> ip|B (saved as op,B,ip)
        GlobalActorRef ref = this->ag.getActorNoThrow(std::get<1>(el));
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during resurrectconnections (3)");
        }
        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *ref.local();
            GlobalChannelRef chptr = ai->getInPort(std::get<2>(el))->getChannelPtr();

            main->resurrectConnection(std::get<0>(el), chptr);
        }
        else
        {
            upcxx::future<GlobalChannelRef> lm = upcxx::rpc(
                ref.where(),
                [](GlobalActorRef ref, std::string name)
                {
                    ActorImpl *ai = *ref.local();
                    GlobalChannelRef chptr = ai->getInPort(name)->getChannelPtr();
                    return chptr;
                },
                ref, std::get<2>(el));

            upcxx::future<> a = lm.then([this, main, el](GlobalChannelRef chptr)
                                        { main->resurrectConnection(std::get<0>(el), chptr); });
            futs.push_back(a);
        }
    }

    for (const auto &el : incoming)
    {
        //// C|op -> ip (saved as ip,C,op)
        GlobalActorRef ref = this->ag.getActorNoThrow(std::get<1>(el));
        if (mainref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during resurrectconnections (4)");
        }
        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *ref.local();
            AbstractOutPort *out = ai->getOutPort(std::get<2>(el));

            main->resurrectConnection(std::get<0>(el), upcxx::rank_me(), out);
        }
        else
        {
            upcxx::future<AbstractOutPort *> fut = upcxx::rpc(
                ref.where(),
                [](GlobalActorRef ref, upcxx::intrank_t rank, std::string name)
                {
                    ActorImpl *ai = *ref.local();
                    AbstractOutPort *out = ai->getOutPort(name);
                    return out;
                },
                ref, upcxx::rank_me(), std::get<2>(el));

            upcxx::future<> a = fut.then([this, main, el, ref](AbstractOutPort *aptr)
                                         { main->resurrectConnection(std::get<0>(el), ref.where(), aptr); });
            futs.push_back(a);
        }
    }

    upcxx::future<> fut = allDone.finalize();
    upcxx::future<> tmp = util::combineFutures(futs);
    upcxx::future<> resurrected = upcxx::when_all(fut, tmp);

    return resurrected;
}

upcxx::future<GlobalActorRef> DynamicActorGraph::stealActorAsync(const std::string name)
{
    if (name.empty())
    {
        throw std::runtime_error("cant steal actor with empty name");
    }

    GlobalActorRef ref = this->ag.getActorNoThrow(name);
    if (ref == nullptr)
    {
        throw std::runtime_error("Actor should be present in graph during steal");
    }
    upcxx::intrank_t from = ref.where();

#ifdef REPORT_MAIN_ACTIONS
    std::cout << "Stealing begins " << name << " from rank: " << from << " to: " << upcxx::rank_me() << std::endl;
#endif

    if (ref.where() != upcxx::rank_me())
    {

        std::set<std::string> acts = this->pg.getNeighbors(name);
        std::vector<GlobalActorRef> actrefs;
        std::vector<std::string> neighbor_names;
        actrefs.reserve(acts.size());
        // save globalactorrefs for furter use
        std::set<int> neighbors;

        for (const std::string &s : acts)
        {
            neighbor_names.push_back(s);
            GlobalActorRef neighbor = this->ag.getActorNoThrow(s);
            if (neighbor == nullptr)
            {
                throw std::runtime_error("Neighbor Actor should be present in graph during steal");
            }
            actrefs.push_back(neighbor);
            neighbors.insert(neighbor.where());
        }

        upcxx::future<> connections_severed = this->severeConnections(name);

        upcxx::future<> prepared = std::move(connections_severed);

        upcxx::future<> rmed =
            prepared.then([name, this]() -> upcxx::future<> { return std::get<0>(this->ag.rmActorAsync(name)); });

        upcxx::future<> rmed_from_map = rmed;
        /*
        rmed.then(
            [neighbors]()
            {
                upcxx::promise<> allDone;
                for (int i : neighbors)
                {
                    if (i == upcxx::rank_me())
                    {
                        continue;
                    }
                    auto cx = upcxx::operation_cx::as_promise(allDone);
                    upcxx::rpc(i, cx, []() { upcxx::discharge(); });
                }
                upcxx::discharge();
                upcxx::future<> discharged = allDone.finalize();
                upcxx::future<> self_progged = discharged.then([]() { upcxx::progress(); });
                return self_progged;
            });
            */

#ifdef USE_ACTOR_TRIGGERS
        upcxx::future<ActorTriggers> got_triggers = rmed_from_map.then(
            [name, this, from]() -> upcxx::future<ActorTriggers>
            {
                upcxx::future<ActorTriggers> rt = upcxx::rpc(
                    from,
                    [](std::string name, upcxx::dist_object<DynamicActorGraph *> &remoteComponents)
                    {
                        ActorTriggers rt = (*remoteComponents)->ag.moveTriggers(name);
                        return rt;
                    },
                    name, this->remoteComponents);

                return rt;
            });

        upcxx::future<GlobalActorRef> sent =
            got_triggers.then([ref, this](ActorTriggers at) -> upcxx::future<GlobalActorRef>
                              { return sendActorToAsync(upcxx::rank_me(), ref); });
#else
        upcxx::future<GlobalActorRef> sent = rmed_from_map.then([ref, this]() -> upcxx::future<GlobalActorRef>
                                                                { return sendActorToAsync(upcxx::rank_me(), ref); });
#endif

        upcxx::future<GlobalActorRef> rdy_to_reinsert = sent;

        upcxx::future<> changed_rank =
            rdy_to_reinsert.then([this, name](GlobalActorRef _r) { this->pg.changeRankAsyncFF(name); });

        upcxx::future<GlobalActorRef> reinserted = rdy_to_reinsert.then(
            [name, this](GlobalActorRef ref) -> upcxx::future<GlobalActorRef>
            {
                if (ref.where() != upcxx::rank_me())
                {
                    throw std::runtime_error("Didnt we just steal?");
                }
                upcxx::future<> added = this->ag.addActorToAnotherAsync(ref, upcxx::rank_me());
                upcxx::future<GlobalActorRef> r = upcxx::make_future(ref);
                upcxx::future<GlobalActorRef> rr = upcxx::when_all(r, added);
                return rr;
                // return this->ag.changeRef(name, ref);
            });

        // upcxx::future<GlobalActorRef> old2 = upcxx::when_all(reinserted, old);

        // delete old actor
        upcxx::future<std::vector<std::pair<std::string, std::vector<std::vector<float>>>>> deleted =
            reinserted.then( // old2.then(
                [ref, this](GlobalActorRef sent)
                {
                    if (ref == sent)
                    {
                        throw std::runtime_error("Old ref should not be the same as the new ref after stealing!");
                    }

                    GlobalActorRef old = ref;
                    upcxx::future<std::vector<std::pair<std::string, std::vector<std::vector<float>>>>> deld =
                        upcxx::rpc(
                            old.where(),
                            [](GlobalActorRef old, upcxx::dist_object<DynamicActorGraph *> &remoteComponents)
                            {
                                std::vector<std::pair<std::string, std::vector<std::vector<float>>>> v;
                                ActorImpl *aimpl = *old.local();

                                if (!aimpl->hasNoMessages())
                                {
                                    v = aimpl->extract_messages<std::vector<float>, 64>();
                                    std::cerr << upcxx::rank_me() << " had to extract messages" << std::endl;

                                    // throw std::runtime_error("During deletion of old actor inports must be empty!");
                                }

                                if (!aimpl->buffersEmpty())
                                {
                                    aimpl->flushBuffers();
                                    if (!aimpl->buffersEmpty())
                                    {
                                        throw std::runtime_error("During deletion of old actor buffers must be empty!");
                                    }
                                }

                                delete aimpl;
                                (*remoteComponents)->deallocated_actors += 1;

                                upcxx::delete_(old);

                                return v;
                            },
                            old, this->remoteComponents);
                    return deld;
                });

        upcxx::future<GlobalActorRef, std::vector<std::pair<std::string, std::vector<std::vector<float>>>>> rrr =
            upcxx::when_all(reinserted, deleted);

        // refill ports of the actor
        upcxx::future<GlobalActorRef> refilled = rrr.then(
            [name, this](GlobalActorRef sameref,
                         std::vector<std::pair<std::string, std::vector<std::vector<float>>>> msgs) -> GlobalActorRef
            {
                ActorImpl *a = *sameref.local();
                a->refillPorts();
                a->broadcastTaskDeque(this->ag.getTaskDeque());

                for (auto &&pr : msgs)
                {
                    auto *ainp = a->getInPort(pr.first);
                    InPort<std::vector<float>, 64> *inp = dynamic_cast<InPort<std::vector<float>, 64> *>(ainp);
                    inp->reinsert_messages(std::move(pr.second));
                }

                return sameref;
            });

        upcxx::future<GlobalActorRef> reconn = refilled.then(
            [name, this](GlobalActorRef a)
            {
                upcxx::future<> res = this->resurrectConnections(name, a);
                return res.then([a]() { return a; });
            });

#ifdef USE_ACTOR_TRIGGERS
        upcxx::future<GlobalActorRef, ActorTriggers> rdy_to_reinsert_tasks = upcxx::when_all(reconn, got_triggers);

        upcxx::future<GlobalActorRef> reinserted_tasks = rdy_to_reinsert_tasks.then(
            [this](GlobalActorRef ref, ActorTriggers triggers)
            {
                this->ag.addTriggers(std::move(triggers));
                return ref;
            });
#else
        upcxx::future<GlobalActorRef> reinserted_tasks = reconn;
#endif

        // unmark neighbors after reconnection
        upcxx::future<> neighbors_back = reinserted_tasks.then(
            [neighbor_names, name, this](GlobalActorRef _a) -> upcxx::future<>
            {
#ifdef REPORT_MAIN_ACTIONS
                std::cerr << "unpin neighbors of " << name << " (after stealing)" << std::endl;
#endif

                return unpin_neighbors(neighbor_names);
            });

        upcxx::future<GlobalActorRef> ret2 = neighbors_back.then(
            [this, name, from]() -> upcxx::future<GlobalActorRef>
            {
                // losing = false;
                GlobalActorRef ref = this->getActorNoThrow(name);
                if (ref == nullptr)
                {
                    throw std::runtime_error("Actor should be present in graph during steal (2)");
                }
                ActorImpl *aimpl = *ref.local();
                bool started = aimpl->startWithCaller(upcxx::rank_me());
                if (!started)
                {
                    throw std::runtime_error("strating after stealing should not fail");
                }

                if (aimpl->isMarked())
                {
                    throw std::runtime_error("Actor should be unmarked after stealing ends!");
                }

                upcxx::future<> f = upcxx::rpc(
                    from,
                    [](upcxx::dist_object<DynamicActorGraph *> &rmt, int marker)
                    {
                        (*rmt)->going_away -= 1;
                        (*rmt)->stealing_from_me.erase(marker);
                    },
                    remoteComponents, upcxx::rank_me());
                upcxx::future<GlobalActorRef> rfut = upcxx::make_future(ref);

                return upcxx::when_all(f, rfut);
            });

        return upcxx::when_all(ret2, changed_rank);
    }
    else
    {
        throw std::runtime_error("no reason to steal an Actor that is on the same rank, stealActorAsyncImpl");
    }
}

upcxx::future<GlobalActorRef> DynamicActorGraph::offloadActorAsync(const std::string name, upcxx::intrank_t to)
{

    if (name.empty())
    {
        throw std::runtime_error("Cant offload actor with empty name");
    }
    upcxx::intrank_t myrank = upcxx::rank_me();

    GlobalActorRef ref = this->ag.getActorNoThrow(name);
    if (ref == nullptr)
    {
        throw std::runtime_error("Actor should be present in graph during offloading (1)");
    }
    upcxx::intrank_t from = ref.where();

#ifdef REPORT_MAIN_ACTIONS
    std::cout << "Offload begins " << name << " from rank: " << from << " to: " << upcxx::rank_me() << std::endl;
#endif

    if (ref.where() == upcxx::rank_me())
    {

        std::set<std::string> acts = this->pg.getNeighbors(name);
        std::set<int> neighbors;
        std::vector<GlobalActorRef> actrefs;
        std::vector<std::string> neighbor_names;
        actrefs.reserve(acts.size());
        // save globalactorrefs for furter use
        for (const std::string &s : acts)
        {
            neighbor_names.push_back(s);
            GlobalActorRef neighbor = this->ag.getActorNoThrow(s);
            if (neighbor == nullptr)
            {
                throw std::runtime_error("Actor should be present in graph during offloading (1)");
            }
            actrefs.push_back(neighbor);
            neighbors.insert(neighbor.where());
        }
        // wait, such that no messages are on the wire

        // stop and prepare actor for migration, stop the actor to be migrated after the others actor stopped
        // because no act method is called when actor is stopped but non-stopped actors can send it data (if user
        // defines in such a way, that is the case in simulation actor) stop act, wait later

        upcxx::future<> connections_severed = this->severeConnections(name);

        upcxx::future<> prepared = std::move(connections_severed);

        upcxx::future<> rmed_from_map = std::get<0>(this->ag.rmActorAsync(name));

        upcxx::future<> rdy_to_send = rmed_from_map.then(
            [neighbors]()
            {
                upcxx::promise<> allDone;
                upcxx::discharge();

                for (int i : neighbors)
                {
                    if (i == upcxx::rank_me())
                    {
                        continue;
                    }
                    auto cx = upcxx::operation_cx::as_promise(allDone);
                    upcxx::rpc(i, cx, []() { upcxx::discharge(); });
                }

                upcxx::future<> discharged = allDone.finalize();
                upcxx::future<> self_progged = discharged.then([]() { upcxx::progress(); });
                return self_progged;
            });

        upcxx::future<GlobalActorRef> sent =
            rdy_to_send.then([ref, this, to]() -> upcxx::future<GlobalActorRef> { return sendActorToAsync(to, ref); });

#ifdef USE_ACTOR_TRIGGERS
        upcxx::future<ActorTriggers> got_triggers = sent.then(
            [name, this](GlobalActorRef rrr) -> ActorTriggers
            {
                ActorTriggers rt = (*remoteComponents)->ag.moveTriggers(name);
                return rt;
            });
#endif

        upcxx::future<> uwu = sent.then(
            [name, this, from](GlobalActorRef r)
            {
                upcxx::rpc_ff(
                    from,
                    [](upcxx::dist_object<DynamicActorGraph *> &rmt, std::string name)
                    { (*rmt)->pg.changeRankAsyncFF(name); },
                    remoteComponents, name);
                this->going_away -= 1;
            });

        upcxx::future<GlobalActorRef> rdy_to_reinsert = upcxx::when_all(rmed_from_map, sent);

        upcxx::future<GlobalActorRef> reinserted = rdy_to_reinsert.then(
            [name, this, to](GlobalActorRef ref) -> upcxx::future<GlobalActorRef>
            {
                upcxx::future<> added = this->ag.addActorToAnotherAsync(ref, to);
                upcxx::future<GlobalActorRef> r = upcxx::make_future(ref);
                upcxx::future<GlobalActorRef> rr = upcxx::when_all(std::move(r), std::move(added));
                return rr;
                // return this->ag.changeRef(name, ref);
            });

        // upcxx::future<GlobalActorRef> old2 = upcxx::when_all(reinserted, old);

        // delete old actor
        upcxx::future<> deleted = sent.then( // old2.then(
            [ref, this](GlobalActorRef sent) -> upcxx::future<>
            {
                GlobalActorRef old = ref;
                return upcxx::rpc(
                    old.where(),
                    [](GlobalActorRef old, upcxx::dist_object<DynamicActorGraph *> &remoteComponents)
                    {
                        ActorImpl *aimpl = *old.local();

                        if (!aimpl->hasNoMessages())
                        {
                            throw std::runtime_error("During deletion of old actor inports must be empty!");
                        }

                        if (!aimpl->buffersEmpty())
                        {
                            aimpl->flushBuffers();
                            if (!aimpl->buffersEmpty())
                            {
                                throw std::runtime_error("During deletion of old actor buffers must be empty!");
                            }
                        }

                        delete aimpl;
                        (*remoteComponents)->deallocated_actors += 1;

                        upcxx::delete_(old);
                    },
                    old, this->remoteComponents);
            });

        // refill ports of the actor
        upcxx::future<GlobalActorRef> refilled = reinserted.then(
            [name, to, this](GlobalActorRef ref) -> upcxx::future<GlobalActorRef>
            {
                // upcxx::future<GlobalActorRef> u;
                upcxx::future<GlobalActorRef> u = upcxx::rpc(
                    to,
                    [](GlobalActorRef ref, upcxx::dist_object<DynamicActorGraph *> &rmt,
                       std::string name) -> GlobalActorRef
                    {
                        ActorImpl *a = *ref.local();
                        a->refillPorts();
                        a->broadcastTaskDeque((*rmt)->ag.getTaskDeque());
                        GlobalActorRef o = ref;
                        return o;
                    },
                    ref, remoteComponents, name);
                return u;
            });

#ifdef USE_ACTOR_TRIGGERS
        upcxx::future<GlobalActorRef, ActorTriggers> rdy_to_reinsert_tasks = upcxx::when_all(refilled, got_triggers);

        upcxx::future<GlobalActorRef> reinserted_tasks = rdy_to_reinsert_tasks.then(
            [to, this](GlobalActorRef ref, ActorTriggers triggers)
            {
                upcxx::future<> u = upcxx::rpc(
                    to,
                    [](ActorTriggers att, upcxx::dist_object<DynamicActorGraph *> &rmt)
                    { (*rmt)->ag.addTriggers(std::move(att)); },
                    std::move(triggers), remoteComponents);
                return u.then([ref]() { return ref; });
            });
#else
        upcxx::future<GlobalActorRef> reinserted_tasks = refilled;
#endif

        upcxx::future<GlobalActorRef> reconn = reinserted_tasks.then(
            [name, to, this](GlobalActorRef uwu)
            {
                upcxx::future<> u = upcxx::rpc(
                    to,
                    [](GlobalActorRef ref, upcxx::dist_object<DynamicActorGraph *> &rmt,
                       std::string name) -> upcxx::future<> { return (*rmt)->resurrectConnections(name, ref); },
                    uwu, remoteComponents, name);
                return u.then([uwu]() { return uwu; });
            });

        upcxx::future<> sds = reconn.then(
            [neighbor_names, name, to, this](GlobalActorRef _a) -> upcxx::future<>
            {
                GlobalActorRef stolen = this->ag.getActorNoThrow(name);
                if (stolen == nullptr)
                {
                    throw std::runtime_error("Actor should be present in graph during offloading (2)");
                }

                std::vector<upcxx::future<>> fut_flush;

                for (const std::string &nname : neighbor_names)
                {
                    auto conn = this->pg.getConnection(nname, name);
                    const std::string outp = std::get<1>(conn);

                    GlobalActorRef n_ref = this->ag.getActorNoThrow(nname);
                    if (n_ref == nullptr)
                    {
                        throw std::runtime_error("Actor should be present in graph during offloading (3)");
                    }
                    if (n_ref.where() == upcxx::rank_me())
                    {
                        ActorImpl *n_ai = *n_ref.local();
                        n_ai->flushBuffer(outp);
                    }
                    else
                    {
                        upcxx::future<> ff = upcxx::rpc(
                            n_ref.where(),
                            [](GlobalActorRef neighbor, std::string outp_name)
                            {
                                ActorImpl *n_ai = *neighbor.local();
                                n_ai->flushBuffer(outp_name);
                            },
                            n_ref, outp);
                        fut_flush.push_back(std::move(ff));
                    }
                }

                upcxx::future<> e = upcxx::rpc(
                    to,
                    [](GlobalActorRef ref)
                    {
                        ActorImpl *ai = (*ref.local());
                        ai->flushBuffers();
                        ai->sendNotifications();
                    },
                    stolen);

                fut_flush.push_back(std::move(e));

                return util::combineFutures(std::move(fut_flush));
            });

        // unmark neighbors after reconnection
        upcxx::future<> neighbors_back = sds.then(
            [neighbor_names, name, this]() -> upcxx::future<>
            {
#ifdef REPORT_MAIN_ACTIONS
                std::cerr << "unpin neighbors of " << name << " (after stealing)" << std::endl;
#endif
                return unpin_neighbors(neighbor_names);
            });

        // restart the migrated actor
        upcxx::future<GlobalActorRef> started_main = neighbors_back.then(
            [name, myrank, to, this]()
            {
                GlobalActorRef ref = this->getActorNoThrow(name);
                if (ref == nullptr)
                {
                    throw std::runtime_error("Actor should be present in graph during offloading (4)");
                }
                upcxx::future<> u = upcxx::rpc(
                    to,
                    [](GlobalActorRef offloaded, int rank)
                    {
                        ActorImpl *ai = *offloaded.local();
                        bool started = ai->startWithCaller(rank);
                        if (!started)
                        {
                            throw std::runtime_error("Starting after stealing should not fail");
                        }

                        if (ai->isMarked())
                        {
                            throw std::runtime_error("Actor should be unmarked after stealing ends!");
                        }
                    },
                    ref, myrank);

                return ref;
            });

        return started_main;
    }
    else
    {

        throw std::runtime_error("no reason to steal an Actor that is on the same rank, stealActorAsyncImpl");
    }
}

upcxx::future<std::vector<float>> DynamicActorGraph::getWorkOfOtherRanks(const std::set<int> &ranks)
{
    if (dont_use_buffer)
    {
        throw std::runtime_error("getWork called for a second time before the first one was completed!");
    }

    dont_use_buffer = true;
    buffer.resize(upcxx::rank_n());
    std::fill(buffer.begin(), buffer.end(), 0.0);

    // buffer[upcxx::rank_me()] = (time_spent_for_cost) ? static_cast<float>(this->ag.getWorkDoneForMigration())
    //                                                  : static_cast<float>(this->ag.getTaskCount());

    std::vector<upcxx::future<>> f;
    f.reserve(ranks.size());

    for (auto r : ranks)
    {

        if (r == upcxx::rank_me())
        {
            continue;
        }
        else
        {
            if (!use_rma)
            {

                upcxx::future<float> df = upcxx::rpc(
                    r,
                    [](upcxx::dist_object<DynamicActorGraph *> &remoteComponents)
                    {
                        float t = ((*remoteComponents)->time_spent_for_cost)
                                      ? static_cast<float>((*remoteComponents)->ag.getWorkDoneForMigration())
                                      : static_cast<float>((*remoteComponents)->ag.getTaskCount());
                        return t;
                    },
                    remoteComponents);

                upcxx::future<> dif = df.then([r, this](float d) { this->buffer[r] = d; });
                f.push_back(dif);
            }
            else
            {
                upcxx::future<unsigned int> wk = upcxx::rget(this->ag.gptrsToTaskCounts[r]);
                upcxx::future<> dif = wk.then([r, this](unsigned int d) { this->buffer[r] = static_cast<float>(d); });
                f.push_back(dif);
            }
        }
    }
    upcxx::future<> all = util::combineFutures(f);

    upcxx::future<std::vector<float>> done = all.then(
        [this]()
        {
            if (!dont_use_buffer)
            {
                throw std::runtime_error("getWork called for a second time before the first one was completed!");
            }

            std::vector<float> trt = std::move(this->buffer);
            buffer.clear();
            dont_use_buffer = false;
            // Communication takes some time so we take (comm beg + comm end) / 2 as a guess

            if (!use_rma)
            {
                trt[upcxx::rank_me()] = (time_spent_for_cost) ? static_cast<float>(this->ag.getWorkDoneForMigration())
                                                              : static_cast<float>(this->ag.getTaskCount());
            }
            else
            {
                trt[upcxx::rank_me()] = static_cast<float>(*this->ag.taskCount->local());
            }

            // trt[upcxx::rank_me()] /= 2.0;
            return trt;
        });

    return done;
}

upcxx::future<std::vector<float>> DynamicActorGraph::getWorkOfOtherRanks()
{
    std::vector<upcxx::future<>> f;
    f.reserve(upcxx::rank_n());

    if (dont_use_buffer)
    {
        throw std::runtime_error("getWork called for a second time before the first one was completed!");
    }

    dont_use_buffer = true;
    buffer.resize(upcxx::rank_n());
    std::fill(buffer.begin(), buffer.end(), 0.0);

    for (int r = 0; r < upcxx::rank_n(); r++)
    {
        if (r == upcxx::rank_me())
        {
            // buffer[r] = (time_spent_for_cost) ? static_cast<float>(this->ag.getWorkDoneForMigration())
            //                                   : static_cast<float>(this->ag.getTaskCount());
            continue;
        }

        if (!use_rma)
        {

            upcxx::future<float> df = upcxx::rpc(
                r,
                [](upcxx::dist_object<DynamicActorGraph *> &remoteComponents)
                {
                    float t = ((*remoteComponents)->time_spent_for_cost)
                                  ? static_cast<float>((*remoteComponents)->ag.getWorkDoneForMigration())
                                  : static_cast<float>((*remoteComponents)->ag.getTaskCount());
                    return t;
                },
                remoteComponents);

            upcxx::future<> dif = df.then([r, this](float d) { this->buffer[r] = d; });
            f.push_back(dif);
        }
        else
        {
            upcxx::future<unsigned int> wk = upcxx::rget(this->ag.gptrsToTaskCounts[r]);
            upcxx::future<> dif = wk.then([r, this](unsigned int d) { this->buffer[r] = static_cast<float>(d); });
            f.push_back(dif);
        }
    }
    upcxx::future<> all = util::combineFutures((f));

    upcxx::future<std::vector<float>> done = all.then(
        [this]() -> std::vector<float>
        {
            if (!dont_use_buffer)
            {
                throw std::runtime_error("getWork called for a second time before the first one was completed!");
            }

            std::vector<float> trt = std::move(buffer);
            buffer.clear();
            dont_use_buffer = false;
            // Communication takes some time so we take (comm beg + comm end) / 2 as a guess
            if (!use_rma)
            {
                trt[upcxx::rank_me()] = (time_spent_for_cost) ? static_cast<float>(this->ag.getWorkDoneForMigration())
                                                              : static_cast<float>(this->ag.getTaskCount());
            }
            else
            {
                trt[upcxx::rank_me()] = static_cast<float>(*this->ag.taskCount->local());
            }
            // trt[upcxx::rank_me()] /= 2.0;

            return trt;
        });

    return done;
}

upcxx::future<std::vector<std::string>> DynamicActorGraph::findActorsToSteal()
{

    /*

    CONSTEXPR IF ELSE CAUSES LINKAGE ERRORS WITH INTEL COMPILER!!!!!!!

    */
    /*
    if (losing)
    {
        std::vector<std::string> v;
        upcxx::future<std::vector<std::string>> fnd = upcxx::make_future(std::move(v));
        return fnd;
    }
    */

#ifdef GLOBAL_MIGRATION
#ifdef STEAL_FROM_BUSY_RANK
    upcxx::future<std::vector<float>> work = getWorkOfOtherRanks();

    upcxx::future<std::vector<std::string>> fnd = work.then(
        [this](std::vector<float> ds) -> std::vector<std::string>
        {
            int maxElementIndex = std::max_element(std::begin(ds), std::end(ds)) - std::begin(ds);
            float maxElement = ds[maxElementIndex];
            if (maxElementIndex != upcxx::rank_me() && maxElement > allowed_imbalance * ds[upcxx::rank_me()])
            {
                if (stealing_from_me.find(maxElementIndex) == stealing_from_me.end())
                {
                    auto v = this->pg.getRandomActors(1, upcxx::rank_me(), maxElementIndex);
                    return v;
                }
            }

            std::vector<std::string> s;
            return s;
        });

    return fnd;
#else
    auto v = this->pg.getRandomActors(config::actor_steal_list_size, upcxx::rank_me());
    return upcxx::make_future(std::move(v));

#endif
#else
#ifdef STEAL_FROM_BUSY_RANK
    std::set<int> bordering_ranks = this->pg.borderingRanks(upcxx::rank_me());
    upcxx::future<std::vector<float>> work;
    if (bordering_ranks.empty())
    {
        // rank is empty so we steal from anz rank
        work = getWorkOfOtherRanks();
    }
    else
    {
        work = getWorkOfOtherRanks(bordering_ranks);
    }

    upcxx::future<std::vector<std::string>> fnd = work.then(
        [this](std::vector<float> ds) -> std::vector<std::string>
        {
            int maxElementIndex = std::max_element(std::begin(ds), std::end(ds)) - std::begin(ds);
            float maxElement = ds[maxElementIndex];
            if (maxElementIndex != upcxx::rank_me() && maxElement > allowed_imbalance * ds[upcxx::rank_me()])
            {
                if (stealing_from_me.find(maxElementIndex) == stealing_from_me.end())
                {
                    if (contigious_migration)
                    {
                        auto v = this->pg.neighborsOf(upcxx::rank_me(), maxElementIndex);
                        return v;
                    }
                    else
                    {
                        auto v = this->pg.getRandomActors(1, upcxx::rank_me(), maxElementIndex);
                        return v;
                    }
                }
            }

            std::vector<std::string> s;
            return s;
        });

    return fnd;
#else
    std::set<int> bordering_ranks_stealable = this->pg.borderingRanks(upcxx::rank_me());
    std::vector<int> r;

    if (bordering_ranks_stealable.empty())
    {
        auto v = this->pg.getRandomActors(config::actor_steal_list_size, upcxx::rank_me());
        return upcxx::make_future(std::move(v));
    }
    else
    {
        if (contigious_migration)
        {
            auto v = this->pg.neighborsOf(upcxx::rank_me());
            return upcxx::make_future(std::move(v));
        }
        else
        {
            auto v =
                this->pg.getRandomActors(config::actor_steal_list_size, upcxx::rank_me(), bordering_ranks_stealable);
            return upcxx::make_future(std::move(v));
        }
    }

#endif
#endif

    throw std::runtime_error("Misconfiguration!");
}

upcxx::future<std::string> DynamicActorGraph::findAnActorToStealAndMark()
{

    /*

    CONSTEXPR IF ELSE CAUSES LINKAGE ERRORS WITH INTEL COMPILER!!!!!!!

    */
    /*
    if (losing)
    {
        std::vector<std::string> v;
        upcxx::future<std::vector<std::string>> fnd = upcxx::make_future(std::move(v));
        return fnd;
    }
    */

#ifdef GLOBAL_MIGRATION
#ifdef STEAL_FROM_BUSY_RANK
    upcxx::future<std::vector<float>> work = getWorkOfOtherRanks();

    upcxx::future<std::string> fnd = work.then(
        [this](std::vector<float> ds) -> upcxx::future<std::string>
        {
            std::string s;
            upcxx::future<std::string> vs = upcxx::make_future(s);

            int maxElementIndex = std::max_element(std::begin(ds), std::end(ds)) - std::begin(ds);
            float maxElement = ds[maxElementIndex];
            if (maxElementIndex != upcxx::rank_me() && maxElement > allowed_imbalance * ds[upcxx::rank_me()])
            {
                if (stealing_from_me.find(maxElementIndex) == stealing_from_me.end())
                {
                    return upcxx::rpc(
                        maxElementIndex,
                        [](upcxx::dist_object<DynamicActorGraph *> &rmt, int marker) -> std::string
                        { return (*rmt)->getMeAnActor(marker, upcxx::rank_me()); },
                        remoteComponents, upcxx::rank_me());
                }
            }

            return vs;
        });

    return fnd;
#else
    std::uniform_int_distribution<int> gen(0, upcxx::rank_n() - 1);
    int r = gen(rng);
    size_t tries = 0;
    while ((r == upcxx::rank_me() || stealing_from_me.find(r) != stealing_from_me.end()) && tries <= 3)
    {
        tries += 1;
        r = gen(rng);
    }
    float t = (time_spent_for_cost) ? static_cast<float>(ag.getWorkDoneForMigration())
                                    : static_cast<float>(ag.getTaskCount());

    if (r != upcxx::rank_me())
    {
        upcxx::future<std::string> vs = upcxx::rpc(
            r,
            [](upcxx::dist_object<DynamicActorGraph *> &rmt, int marker, float work) -> std::string
            {
                float t = ((*rmt)->time_spent_for_cost) ? static_cast<float>((*rmt)->ag.getWorkDoneForMigration())
                                                        : static_cast<float>((*rmt)->ag.getTaskCount());
                if (t >= allowed_imbalance * work)
                {
                    return (*rmt)->getMeAnActor(marker, upcxx::rank_me());
                }
                std::string s;
                return s;
            },
            remoteComponents, upcxx::rank_me(), t);
        return vs;
    }

#endif
#else
#ifdef STEAL_FROM_BUSY_RANK
    std::set<int> bordering_ranks = this->pg.borderingRanks(upcxx::rank_me());
    upcxx::future<std::vector<float>> work;
    if (bordering_ranks.empty())
    {
        // rank is empty so we steal from anz rank
        work = getWorkOfOtherRanks();
    }
    else
    {
        work = getWorkOfOtherRanks(bordering_ranks);
    }

    upcxx::future<std::string> fnd = work.then(
        [this](std::vector<float> ds) -> upcxx::future<std::string>
        {
            std::string s;
            upcxx::future<std::string> victims = upcxx::make_future(s);

            int maxElementIndex = std::distance(std::begin(ds), std::max_element(std::begin(ds), std::end(ds)));
            if (maxElementIndex < 0)
            {
                throw std::runtime_error("HOW?");
            }

            float maxElement = ds[maxElementIndex];
            if (maxElementIndex != upcxx::rank_me() && maxElement > allowed_imbalance * ds[upcxx::rank_me()])
            {
                if (stealing_from_me.find(maxElementIndex) == stealing_from_me.end())
                {
                    return upcxx::rpc(
                        maxElementIndex,
                        [](upcxx::dist_object<DynamicActorGraph *> &rmt, int rank) -> std::string
                        {
                            if ((*rmt)->contigious_migration)
                            {
                                return (*rmt)->getMeAnActor(rank, upcxx::rank_me());
                            }
                            else
                            {
                                return (*rmt)->getMeAnActor(rank, upcxx::rank_me());
                            }
                        },
                        remoteComponents, upcxx::rank_me());
                }
            }

            return victims;
        });

    return fnd;
#else
    std::set<int> bordering_ranks = this->pg.borderingRanks(upcxx::rank_me());
    std::set<int> bordering_ranks_stealable;
    std::set_difference(bordering_ranks.begin(), bordering_ranks.end(), stealing_from_me.begin(),
                        stealing_from_me.end(),
                        std::inserter(bordering_ranks_stealable, bordering_ranks_stealable.begin()));
    std::vector<int> r;

    std::string s;
    upcxx::future<std::string> victims = upcxx::make_future(s);

    if (!bordering_ranks_stealable.empty())
    {
        std::sample(bordering_ranks_stealable.begin(), bordering_ranks_stealable.end(), std::back_inserter(r), 1,
                    std::mt19937{std::random_device{}()});

        if (!r.empty() && r[0] != upcxx::rank_me())
        {
            float t = (time_spent_for_cost) ? static_cast<float>(ag.getWorkDoneForMigration())
                                            : static_cast<float>(ag.getTaskCount());
            return upcxx::rpc(
                r[0],
                [](upcxx::dist_object<DynamicActorGraph *> &rmt, int rank, float work) -> std::string
                {
                    float t = ((*rmt)->time_spent_for_cost) ? static_cast<float>((*rmt)->ag.getWorkDoneForMigration())
                                                            : static_cast<float>((*rmt)->ag.getTaskCount());
                    if (t >= allowed_imbalance * work)
                    {
                        if ((*rmt)->contigious_migration)
                        {
                            return (*rmt)->getMeAnActor(rank, upcxx::rank_me(), rank);
                        }
                        else
                        {
                            return (*rmt)->getMeAnActor(rank, upcxx::rank_me());
                        }
                    }
                    std::string s;
                    return s;
                },
                remoteComponents, upcxx::rank_me(), t);
        }
    }
    else
    {
        // if the rank is empty and has neighbors at all we should steal like global random from any rank
        std::uniform_int_distribution<int> gen(0, upcxx::rank_n() - 1);
        int r = gen(rng);
        size_t tries = 0;
        while (r == upcxx::rank_me() && tries <= 3)
        {
            tries += 1;
            r = gen(rng);
        }
        if (r != upcxx::rank_me())
        {
            upcxx::future<std::string> vs = upcxx::rpc(
                r,
                [](upcxx::dist_object<DynamicActorGraph *> &rmt, int marker) -> std::string
                { return (*rmt)->getMeAnActor(marker, upcxx::rank_me()); },
                remoteComponents, upcxx::rank_me());
            return vs;
        }
    }

    return victims;
#endif
#endif

    throw std::runtime_error("Misconfiguration!");
}

upcxx::future<std::string, upcxx::intrank_t> DynamicActorGraph::findActorsToOffload()
{
#ifndef GLOBAL_MIGRATION
#ifndef ORDERED_OFFLOAD
    std::set<int> bordering_ranks = this->pg.borderingRanks(upcxx::rank_me());
    upcxx::future<std::vector<float>> work;
    if (bordering_ranks.empty())
    {
        // throw std::runtime_error("Offloading won't work with empty initial ranks!");
        std::string s{};
        upcxx::intrank_t t = -1;
        return upcxx::make_future(std::move(s), t);
    }
    else
    {
        work = getWorkOfOtherRanks(bordering_ranks);
    }

    upcxx::future<std::string, upcxx::intrank_t> fnd = work.then(
        [this, bordering_ranks](std::vector<float> ds) -> upcxx::future<std::string, int>
        {
            std::string victims;

            std::vector<std::pair<upcxx::intrank_t, float>> filtered;
            filtered.reserve(bordering_ranks.size() + 1);

            for (auto el : bordering_ranks)
            {
                filtered.emplace_back(el, ds[el]);
            }
            filtered.emplace_back(upcxx::rank_me(), ds[upcxx::rank_me()]);

            auto maxelit = std::max_element(std::begin(filtered), std::end(filtered),
                                            [](const auto &l, const auto &r) { return l.second < r.second; });
            upcxx::intrank_t maxElementIndex = maxelit->first;
            float max = maxelit->second;

            auto minelit = std::min_element(std::begin(filtered), std::end(filtered),
                                            [](const auto &l, const auto &r) { return l.second < r.second; });
            upcxx::intrank_t minElementIndex = minelit->first;
            double min = minelit->second;

#ifdef REPORT_MAION_ACTIONS
            {
                std::cout << "{";
                for (auto &el : filtered)
                {
                    std::cout << "(" << el.first << ", " << el.second << ") ";
                }
                std::cout << "end }" << std::endl;
            }
#endif

            if (maxElementIndex == upcxx::rank_me() && max >= allowed_imbalance * min)
            {
#ifdef REPORT_MAIN_ACTIONS
// std::cout << "I am the max: " << upcxx::rank_me() << std::endl;
#endif
                if (contigious_migration)
                {
                    victims = getMeAnActor(upcxx::rank_me(), upcxx::rank_me(), minElementIndex);
                }
                else
                {
                    victims = getMeAnActor(upcxx::rank_me(), upcxx::rank_me());
                }

                return upcxx::make_future(std::move(victims), minElementIndex);
            }

            return upcxx::make_future(std::move(victims), -1);
        });

    return fnd;
#else
    std::set<int> bordering_ranks = this->pg.borderingRanks(upcxx::rank_me());
    upcxx::future<std::vector<float>> work;
    if (bordering_ranks.empty())
    {
        // throw std::runtime_error("Offloading won't work with empty initial ranks!");
        std::string s{};
        upcxx::intrank_t t = -1;
        return upcxx::make_future(std::move(s), t);
    }
    else
    {
        work = getWorkOfOtherRanks(bordering_ranks);
    }

    upcxx::future<std::string, upcxx::intrank_t> fnd = work.then(
        [this, bordering_ranks](std::vector<float> ds) -> upcxx::future<std::string, int>
        {
            std::string victims;

            std::vector<std::pair<upcxx::intrank_t, float>> zipped;
            zipped.reserve(bordering_ranks.size() + 1);

            for (auto el : bordering_ranks)
            {
                zipped.emplace_back(el, ds[el]);
            }
            zipped.emplace_back(upcxx::rank_me(), ds[upcxx::rank_me()]);

            // Sort descending
            std::sort(zipped.begin(), zipped.end(), [](const auto &l, const auto &r) { return l.second > r.second; });

            size_t middle = (zipped.size() + 1) > 1;

            size_t my_offset =
                std::find_if(zipped.begin(), zipped.end(), [](const auto &l) { return l.first == upcxx::rank_me(); }) -
                zipped.begin();

            if (my_offset < middle)
            {
                size_t my_offset_inverse = (zipped.size() - 1) - my_offset;
                upcxx::intrank_t other_rank = zipped[my_offset_inverse].first;
                if (my_offset_inverse > middle && ds[upcxx::rank_me()] >= allowed_imbalance * ds[other_rank])
                {
                    victims = getMeAnActor(upcxx::rank_me(), upcxx::rank_me());
                    return upcxx::make_future(std::move(victims), other_rank);
                }
            }

            return upcxx::make_future(std::move(victims), -1);
        });

    return fnd;
#endif
#else
#ifndef ORDERED_OFFLOAD
    upcxx::future<std::vector<float>> work = getWorkOfOtherRanks();

    upcxx::future<std::string, upcxx::intrank_t> fnd = work.then(
        [this](std::vector<float> ds) -> upcxx::future<std::string, int>
        {
            std::string victims;

            int maxElementIndex = std::max_element(std::begin(ds), std::end(ds)) - std::begin(ds);
            float max = ds[maxElementIndex];

            int minElementIndex = std::min_element(std::begin(ds), std::end(ds)) - std::begin(ds);
            float min = ds[minElementIndex];

            if (maxElementIndex == upcxx::rank_me() && max >= allowed_imbalance * min)
            {
                victims = getMeAnActor(upcxx::rank_me(), upcxx::rank_me());
            }

            return upcxx::make_future(std::move(victims), minElementIndex);
        });

    return fnd;
#else
    upcxx::future<std::vector<float>> work = getWorkOfOtherRanks();

    upcxx::future<std::string, upcxx::intrank_t> fnd = work.then(
        [this](std::vector<float> ds) -> upcxx::future<std::string, int>
        {
            std::string victims;

            std::vector<std::pair<upcxx::intrank_t, float>> zipped;
            zipped.reserve(ds.size() + 1);

            for (size_t i = 0; i < ds.size(); i++)
            {
                zipped.emplace_back(i, ds[i]);
            }
            zipped.emplace_back(upcxx::rank_me(), ds[upcxx::rank_me()]);

            // Sort descending
            std::sort(zipped.begin(), zipped.end(), [](const auto &l, const auto &r) { return l.second > r.second; });

            size_t middle = (ds.size() + 1) > 1;

            size_t my_offset =
                std::find_if(zipped.begin(), zipped.end(), [](const auto &l) { return l.first == upcxx::rank_me(); }) -
                zipped.begin();

            if (my_offset < middle)
            {
                size_t my_offset_inverse = (zipped.size() - 1) - my_offset;
                upcxx::intrank_t other_rank = zipped[my_offset_inverse].first;
                if (my_offset_inverse > middle && ds[upcxx::rank_me()] >= allowed_imbalance * ds[other_rank])
                {
                    victims = getMeAnActor(upcxx::rank_me(), upcxx::rank_me());
                    return upcxx::make_future(std::move(victims), other_rank);
                }
            }

            return upcxx::make_future(std::move(victims), -1);
        });

    return fnd;
#endif
#endif
}

void DynamicActorGraph::stopActors(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    for (auto &pr : migList)
    {
        GlobalActorRef ref = ag.getActorNoThrow(pr.first);
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during stop actors");
        }
        ActorImpl *ai = *ref.local();

        ai->stopWithCaller(ActorState::TemporaryStoppedForMigration, util::rank_n() + 4);
    }
}

upcxx::future<> DynamicActorGraph::restartActors(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    std::vector<upcxx::future<>> futs;

    for (auto &pr : migList)
    {
        GlobalActorRef ref = ag.getActorNoThrow(pr.first);
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during restart");
        }
        if (ref.where() == upcxx::rank_me())
        {

            ActorImpl *ai = *ref.local();

            if (ai->getRunningState() != ActorState::Terminated)
            {
                ai->startWithCaller(upcxx::rank_n() + 4);
            }
        }
    }

    return util::combineFutures(futs);
}

upcxx::future<> DynamicActorGraph::restartActors(const std::set<GlobalActorRef> &migList)
{
    std::vector<upcxx::future<>> futs;

    for (auto &pr : migList)
    {
        GlobalActorRef ref = pr;

        if (ref.where() == upcxx::rank_me())
        {

            ActorImpl *ai = *ref.local();

            if (ai->getRunningState() != ActorState::Terminated)
            {

                ai->startWithCaller(upcxx::rank_n() + 4);
            }
        }
    }

    return util::combineFutures(futs);
}

std::tuple<upcxx::future<>, std::vector<GlobalActorRef>>
DynamicActorGraph::rmOldActors(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    std::vector<upcxx::future<>> futs;
    std::vector<GlobalActorRef> v;
    for (auto &pr : migList)
    {
        GlobalActorRef r = this->ag.getActorNoThrow(pr.first);
        if (r == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during rmold actors");
        }
        ActorImpl *act = *r.local();
        auto tup = this->ag.rmActorAsync(act->getName());
        futs.push_back(std::get<0>(tup));
        v.push_back(std::get<1>(tup));
    }

    if (!futs.empty())
    {
        return {util::combineFutures(futs), v};
    }

    return {upcxx::make_future(), v};
}

void DynamicActorGraph::delRemains(const std::vector<GlobalActorRef> &l)
{
    for (auto &el : l)
    {

        ActorImpl *a = *(el.local());
        delete a;
        upcxx::delete_(el);
    }
}

void DynamicActorGraph::sendActorsAsync(const std::vector<GlobalActorRef> &trigs,
                                        const std::unordered_map<std::string, upcxx::intrank_t> &migList,
                                        std::vector<GlobalActorRef> &buffer)
{
    throw std::runtime_error("Need to reimplement this");

    std::vector<upcxx::future<>> futs;

    buffer.resize(trigs.size());
    std::fill(buffer.begin(), buffer.end(), nullptr);

    std::vector<ActorImpl *> local_actors;

    for (size_t i = 0; i < trigs.size(); i++)
    {
        auto el = trigs[i];

        ActorImpl *ai = *el.local();
        local_actors.push_back(ai);
        const std::string &name = ai->getNameRef();
        const std::string name_cpy = ai->getNameRef();
        auto pr = *migList.find(name);

        // std::cout << "send " << ai->getNameRef() << " to " << pr.second << std::endl;
        upcxx::future<GlobalActorRef> nref = this->sendActorToAsync(pr.second, el);

        upcxx::future<> nref_into_buffer = nref.then([i, &buffer](GlobalActorRef sentref) { buffer[i] = sentref; });

        upcxx::future<> del_old = nref_into_buffer.then(
            [el, name_cpy, this]()
            {
                delete (*el.local());
                this->deallocated_actors += 1;
                upcxx::delete_(el);
            });

        futs.push_back(del_old);
    }

    // upcxx::future<> all = util::combineFutures(futs);
    unsigned int i = 0;
    while (i < futs.size())
    {
        unsigned int end = std::min(static_cast<unsigned int>(i + 15), static_cast<unsigned int>(futs.size()));
        std::vector<upcxx::future<>> tmp;
        std::copy(futs.begin() + i, futs.begin() + end, std::back_inserter(tmp));
        upcxx::future<> f = util::combineFutures(tmp);
        i = end;
        f.wait();
    }
}

void DynamicActorGraph::refillPorts(const std::unordered_map<std::string, upcxx::intrank_t> &migs)
{
    for (auto &pr : migs)
    {
        GlobalActorRef ref = this->ag.getActorNoThrow(pr.first);
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during refill miglist");
        }
        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *ref.local();

            if (ai->getRunningState() != ActorState::Terminated)
            {
                ai->refillPorts();
            }
        }
    }
}

void DynamicActorGraph::clear_portinformations(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    for (auto &pr : migList)
    {
        GlobalActorRef ref = this->ag.getActorNoThrow(pr.first);
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in clear portsinfo");
        }
        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *ref.local();

            ai->clearPortInformation();
        }
    }
}

void DynamicActorGraph::refillPorts(const std::set<GlobalActorRef> &migs)
{
    for (auto &el : migs)
    {
        GlobalActorRef ref = el;

        {
            ActorImpl *ai = *ref.local();

            if (ai->getRunningState() != ActorState::Terminated)
            {
                ai->refillPorts();
            }
        }
    }
}

// reinserts the actors in the vector to the actorgraph of the other rank
upcxx::future<> DynamicActorGraph::reinsert(const std::vector<GlobalActorRef> &refs)
{
    std::vector<upcxx::future<>> futs;
    for (auto &el : refs)
    {
        if (el.where() == upcxx::rank_me())
        {
            throw std::runtime_error("Actor was at rank:" + std::to_string(el.where()) +
                                     " my rank: " + std::to_string(upcxx::rank_me()) + ", sent to your own rank wut?");
        }
        else
        {
#ifndef NDEBUG
            std::cout << "reinsert to " << el.where() << std::endl;
#endif
            futs.push_back(this->ag.addActorToAnotherAsync(el, el.where()));
        }
    }

    return util::combineFutures(futs);
}

std::unordered_map<std::string, upcxx::intrank_t>
DynamicActorGraph::cleanseMigrationList(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    std::unordered_map<std::string, upcxx::intrank_t> cleansed;

    for (const auto &pr : migList)
    {
        GlobalActorRef ar = this->ag.getActorNoThrow(pr.first);
        if (ar == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during cleanse");
        }
        if (ar.where() != pr.second)
        {
            cleansed.insert(pr);
        }
    }

    return cleansed;
}

std::unordered_map<std::string, upcxx::intrank_t>
DynamicActorGraph::createLocalMigrationList(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    std::unordered_map<std::string, upcxx::intrank_t> cleansed;

    for (const auto &pr : migList)
    {

        GlobalActorRef ar = this->ag.getActorNoThrow(pr.first);
        if (ar == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during create local miglist");
        }
        if (ar.where() != pr.second && ar.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *ar.local();

            if (!ai->isMarked() && !ai->isPinned() && ai->noNeedToNotify() && ai->buffersEmpty() &&
                (ai->getRunningState() == ActorState::Running))
            {
#ifdef REPORT_MAIN_ACTIONS
                std::cout << ai->getName() << " can be migrated to " << pr.second << std::endl;
#endif

                cleansed.insert(pr);
            }
        }
    }

    return cleansed;
}

void DynamicActorGraph::userEnforcedLocalMigrateActorsDiscretePhases(
    const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    l_migList = migList;

    if (!l_migList.empty())
    {
        std::vector<upcxx::future<>> v;
        v.reserve(upcxx::rank_n() - 1);
        for (int i = 0; i < upcxx::rank_n(); i++)
        {
            if (i == upcxx::rank_me())
            {
                continue;
            }

            upcxx::future<> u = upcxx::rpc(
                i,
                [](upcxx::dist_object<DynamicActorGraph *> &remoteComponents,
                   std::unordered_map<std::string, upcxx::intrank_t> migList)
                { (*remoteComponents)->l_migList = std::move(migList); },
                remoteComponents, l_migList);
            v.emplace_back(u);
        }

        upcxx::future<> all = util::combineFutures(v);

        upcxx::discharge();
        all.wait();
    }
    else
    {
        upcxx::progress();
    }

    migrateActorsDiscretePhases(std::move(l_migList));

    l_migList.clear();
}

void DynamicActorGraph::migrateActorsDiscretePhases(std::unordered_map<std::string, upcxx::intrank_t> &&migList)
{
    this->ag.migrationphase = true;

    // make sure no messages are left
    upcxx::discharge();
    upcxx::progress();
    upcxx::barrier();

    const std::unordered_map<std::string, upcxx::intrank_t> &migListConstRef =
        const_cast<std::unordered_map<std::string, upcxx::intrank_t> &>(migList);

    auto allActorsToMigrate = cleanseMigrationList(migListConstRef);

    /*
    std::cout << "{";
    for (const auto &pr : allActorsToMigrate)
    {
        std::cout << "[" << pr.first << " -> " << pr.second << "], ";
    }
    std::cout << "}" << std::endl;
    */

    auto localActorsToMigrate = createLocalMigrationList(allActorsToMigrate);
    allActorsToMigrate.clear();

    upcxx::barrier();

    stopActors(localActorsToMigrate);

    upcxx::discharge();
    upcxx::progress();
    upcxx::barrier();

    // std::cout << upcxx::rank_me() << " disconnectFromNeighboursAsync" << std::endl;
    upcxx::future<> l = disconnectFromNeighboursAsync(localActorsToMigrate);
    l.wait();

    upcxx::discharge();
    upcxx::progress();
    upcxx::barrier();

    // std::cout << upcxx::rank_me() << " rmActorsAsync" << std::endl;
    std::tuple<upcxx::future<>, std::vector<GlobalActorRef>> rmedTup = rmActorsAsync(localActorsToMigrate);

    upcxx::future<> rmed = std::get<0>(rmedTup);
    std::vector<GlobalActorRef> refs = std::move(std::get<1>(rmedTup));
    rmed.wait();

    upcxx::discharge();
    upcxx::progress();
    upcxx::barrier();

    std::vector<GlobalActorRef> sendbuf;
#ifdef USE_ACTOR_TRIGGERS
    this->ag.sendTriggers(localActorsToMigrate).wait();
#endif
    sendActorsAsync(refs, localActorsToMigrate, sendbuf /*, messagesInInPorts*/);

    upcxx::discharge();
    upcxx::progress();
    upcxx::barrier();

    // std::cout << upcxx::rank_me() << " addActorsToAnotherAsync" << std::endl;

    upcxx::future<> added = addActorsToAnotherAsync(sendbuf);

    added.wait();
    localActorsToMigrate.clear();
    sendbuf.clear();

    std::vector<upcxx::future<>> pgraphrankschanged;

    for (auto a : received_actors)
    {
        ActorImpl *ai = *a.local();
        upcxx::future<> f = this->pg.changeRankAsyncFF(ai->getNameRef());
        pgraphrankschanged.push_back(std::move(f));
    }

    upcxx::future<> rc = util::combineFutures(pgraphrankschanged);

    upcxx::discharge();
    upcxx::progress();
    upcxx::barrier();

    refillPorts(received_actors);

    upcxx::barrier();

    // std::cout << upcxx::rank_me() << " reconnectToNeighboursAsync" << std::endl;
    auto resurrected = reconnectToNeighboursAsync(received_actors);
    resurrected.wait();

    upcxx::discharge();
    upcxx::progress();
    upcxx::barrier();

    auto fk = restartActors(received_actors);
    fk.wait();

    sendbuf.clear();
    received_actors.clear();

    // clear_portinformations(localActorsToMigrate);

#ifdef REPORT_MAIN_ACTIONS
    std::cout << "Rank-" << upcxx::rank_me() << " allocated: " << this->allocated_actors
              << ", deallocated: " << this->deallocated_actors << " actors, " << this->ag.localActors.size()
              << " active actors"
              << " globally " << this->ag.getActors()->size() << std::endl;
#endif

    rc.wait();

    upcxx::barrier();

#ifdef REPORT_MAIN_ACTIONS
    if (upcxx::rank_me() == 0)
    {
        print_gitter(this->ag.getActorsRef());
    }
#endif

    this->ag.migrationphase = false;
}

std::tuple<upcxx::future<>, std::vector<GlobalActorRef>>
DynamicActorGraph::rmActorsAsync(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    std::vector<upcxx::future<>> futs;
    futs.reserve(migList.size());
    std::vector<GlobalActorRef> refs;
    refs.reserve(migList.size());

    for (const auto &el : migList)
    {
        GlobalActorRef ref = this->ag.getActorNoThrow(el.first);
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during rm actor miglist");
        }
        if (ref.where() == upcxx::rank_me())
        {
            std::tuple<upcxx::future<>, GlobalActorRef> tup = rmActorAsync(el.first);
            futs.emplace_back(std::move(std::get<0>(tup)));
            refs.emplace_back(std::move(std::get<1>(tup)));
        }
    }

    upcxx::future<> all = util::combineFutures(futs);

    return {all, refs};
}

upcxx::future<> DynamicActorGraph::addActorsToAnotherAsync(const std::vector<GlobalActorRef> &refs)
{
    std::vector<upcxx::future<>> l;
    l.reserve(refs.size());

    for (const auto &el : refs)
    {
        upcxx::future<> f = addActorToAnotherAsync(el, el.where());

        upcxx::future<> ff = f.then(
            [this, el]() -> upcxx::future<>
            {
                return upcxx::rpc(
                    el.where(),
                    [](upcxx::dist_object<DynamicActorGraph *> &rmt, GlobalActorRef ref)
                    { (*rmt)->received_actors.insert(ref); },
                    remoteComponents, el);
            });

        l.push_back(ff);
    }

    upcxx::future<> all = util::combineFutures(l);
    return all;
}

upcxx::future<> DynamicActorGraph::unmark_actor(const std::string &name)
{

    GlobalActorRef ref = this->ag.getActorNoThrow(name);
    if (ref == nullptr)
    {
        throw std::runtime_error("Actor should be present in graph during unmark actor");
    }
    // since we have failed we need to unmark our original actor too
    if (ref.where() != upcxx::rank_me())
    {
        upcxx::promise<> allDone;

        auto cx = upcxx::operation_cx::as_promise(allDone);
        upcxx::rpc(
            ref.where(), cx,
            [this](upcxx::dist_object<DynamicActorGraph *> &rmt, GlobalActorRef ref, upcxx::intrank_t rank)
            {
                ActorImpl *ai = *ref.local();
                bool could = ai->unmark(rank);
                (*rmt)->going_away -= 1;

                if (!could)
                {
                    throw std::runtime_error("I stopped it, I should be able to start? 2 " + ai->getName() + " state " +
                                             ASPrinter::as2str(ai->getRunningState()));
                }
            },
            remoteComponents, ref, upcxx::rank_me());

        return allDone.finalize();
    }
    else
    {
        ActorImpl *ai = *ref.local();
        bool could = ai->unmark(upcxx::rank_me());
        going_away -= 1;

        if (!could)
        {
            throw std::runtime_error("I stopped it, I should be able to start? 2 " + ai->getName() + " state " +
                                     ASPrinter::as2str(ai->getRunningState()));
        }

        return upcxx::make_future();
    }
}

upcxx::future<> DynamicActorGraph::unpin_neighbors(const std::vector<std::string> &neighbors, size_t pin_array_offset)
{
    // could not pin at least one neighbor, so we unmark the ones we stopped
    upcxx::promise<> allDone;

    size_t t = pin_array_offset;

    std::vector<GlobalActorRef> refs;
    for (size_t i = 0; i < neighbors.size(); i++)
    {
        GlobalActorRef neighbor = this->ag.getActorNoThrow(neighbors[i]);
        if (neighbor == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during unpin neighbor");
        }
        refs.push_back(neighbor);
    }

    for (auto &pr : pins[t].second)
    {
        if (pr.second)
        {
            if (pr.first.where() == upcxx::rank_me())
            {
                ActorImpl *ai = *pr.first.local();
                bool could = ai->unpin(upcxx::rank_me());

                if (!could)
                {
                    throw std::runtime_error("I stopped it, I should be able to start? 1 " + ai->getName() + " state " +
                                             ASPrinter::as2str(ai->getRunningState()));
                }
            }
            else
            {
                auto cx = upcxx::operation_cx::as_promise(allDone);
                upcxx::rpc(
                    pr.first.where(), cx,
                    [this](GlobalActorRef m, upcxx::intrank_t rank)
                    {
                        ActorImpl *ai = *m.local();
                        bool could = ai->unpin(rank);

                        if (!could)
                        {
                            throw std::runtime_error("I stopped it, I should be able to start? 1 " + ai->getName() +
                                                     " state " + ASPrinter::as2str(ai->getRunningState()));
                        }
                    },
                    pr.first, upcxx::rank_me());
            }
        }
    }

    upcxx::future<> done = allDone.finalize();
    upcxx::future<> resource_freed = done.then([t, this]() { this->releaseMarkOffset(t); });
    return resource_freed;
}

upcxx::future<> DynamicActorGraph::unpin_neighbors(const std::vector<std::string> &neighbors)
{
    // could not pin at least one neighbor, so we unmark the ones we stopped
    upcxx::promise<> allDone;

    std::vector<GlobalActorRef> refs;
    for (size_t i = 0; i < neighbors.size(); i++)
    {
        GlobalActorRef neighbor = this->ag.getActorNoThrow(neighbors[i]);
        if (neighbor == nullptr)
        {
            throw std::runtime_error(
                "Actor should be present in graph during unpin neighbors (unpin failed neighborhood failed)");
        }
        refs.push_back(neighbor);
    }

    for (auto ref : refs)
    {
        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *ref.local();
            bool could = ai->unpin(upcxx::rank_me());

            if (!could)
            {
                throw std::runtime_error("I pinned it, I should be able to unpin? 1 " + ai->getName() + " state " +
                                         ASPrinter::as2str(ai->getRunningState()));
            }
        }
        else
        {
            auto cx = upcxx::operation_cx::as_promise(allDone);
            upcxx::rpc(
                ref.where(), cx,
                [this](GlobalActorRef m, upcxx::intrank_t rank)
                {
                    ActorImpl *ai = *m.local();
                    bool could = ai->unpin(rank);

                    if (!could)
                    {
                        throw std::runtime_error("I pinned it, I should be able to unpin? 1 " + ai->getName() +
                                                 " state " + ASPrinter::as2str(ai->getRunningState()));
                    }
                },
                ref, upcxx::rank_me());
        }
    }

    upcxx::future<> done = allDone.finalize();
    return done;
}

std::tuple<upcxx::future<bool>, size_t> DynamicActorGraph::pin_neighbors(const std::vector<std::string> &neighbors)
{
    size_t t = getMarkOffset();

    std::vector<upcxx::future<>> pinned;
    bool only_local = true;

    // this->pins[t].resize(neighbors.size());
    // std::fill(pins[t].begin(), pins[t].end(), false);

    std::vector<GlobalActorRef> neighbor_refs;

    for (size_t i = 0; i < neighbors.size(); i++)
    {
        GlobalActorRef neighbor = this->ag.getActorNoThrow(neighbors[i]);

        if (neighbor == nullptr)
        {
            upcxx::future<bool> u = upcxx::make_future(false);
            return std::make_tuple(std::move(u), std::move(t));
        }
        neighbor_refs.push_back(neighbor);
    }

    bool do_not_send_msgs = false;
    for (auto neighbor : neighbor_refs)
    {
        if (neighbor.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *neighbor.local();
            bool could = ai->pin(upcxx::rank_me());
            pins[t].second.insert(std::make_pair(neighbor, could));
            if (!could)
            {
                do_not_send_msgs = true;
                break;
            }
        }
    }

    if (!do_not_send_msgs)
    {
        for (auto neighbor : neighbor_refs)
        {
            if (neighbor.where() != upcxx::rank_me())
            {
                only_local = false;

                upcxx::future<std::pair<GlobalActorRef, bool>> could = upcxx::rpc(
                    neighbor.where(),
                    [](GlobalActorRef m, upcxx::intrank_t rank)
                    {
                        ActorImpl *ai = *m.local();
                        bool could = ai->pin(rank);
                        return std::make_pair(m, could);
                    },
                    neighbor, upcxx::rank_me());

                upcxx::future<> d = could.then([this, t](std::pair<GlobalActorRef, bool> is_pinned)
                                               { this->pins[t].second.insert(std::move(is_pinned)); });

                pinned.push_back(d);
            }
        }
    }

    if (only_local || do_not_send_msgs)
    {
        bool tmp = true;

        for (auto &pr : pins[t].second)
        {
            tmp &= pr.second;
        }

        if (tmp)
        {
            this->releaseMarkOffset(t);
        }

        if (!tmp)
        {
            for (auto &pr : pins[t].second)
            {
                if (pr.second)
                {
                    GlobalActorRef neighbor = pr.first;
                    ActorImpl *ai = *neighbor.local();
                    bool up = ai->unpin(upcxx::rank_me());
                    if (!up)
                    {
                        throw std::runtime_error("I had pineed I should be able to unpin later?!");
                    }
                }
            }
        }

        upcxx::future<bool> f = upcxx::make_future(tmp);
        return std::make_tuple(f, t);
    }
    else
    {
        upcxx::future<> fut = util::combineFutures(pinned);

        upcxx::future<bool> pinned_all = fut.then(
            [this, t, neighbors]() -> upcxx::future<bool>
            {
                bool tmp = true;

                for (auto &pr : pins[t].second)
                {
                    tmp &= pr.second;
                }

                if (tmp)
                {
                    this->releaseMarkOffset(t);
                    return upcxx::make_future(true);
                }
                else
                {
                    upcxx::promise<> allDone;

                    for (auto &pr : pins[t].second)
                    {
                        if (pr.second)
                        {
                            if (pr.first.where() == upcxx::rank_me())
                            {

                                ActorImpl *ai = *pr.first.local();
                                bool up = ai->unpin(upcxx::rank_me());
                                if (!up)
                                {
                                    throw std::runtime_error("I had pineed I should be able to unpin later?!");
                                }
                            }
                            else
                            {
                                auto cx = upcxx::operation_cx::as_promise(allDone);
                                upcxx::rpc(
                                    pr.first.where(), cx,
                                    [](GlobalActorRef ref, int pinner)
                                    {
                                        ActorImpl *ai = *ref.local();
                                        bool up = ai->unpin(pinner);
                                        if (!up)
                                        {
                                            throw std::runtime_error("I had pineed I should be able to unpin later?!");
                                        }
                                    },
                                    pr.first, upcxx::rank_me());
                            }
                        }
                    }

                    upcxx::future<> f = allDone.finalize();
                    upcxx::future<bool> ff = upcxx::make_future(tmp);
                    return upcxx::when_all(f, ff);
                }
            });

        return std::make_tuple(std::move(pinned_all), t);
    }
}

std::string DynamicActorGraph::getMeAnActor(int marker, int from, int must_border)
{
    // Do not go into actor locking if we have global reasons to not send them a task
    if (this->ag.getActiveActors() <= 1)
    {
        return "";
    }

    if (this->ag.getLocalActorsRef().size() <= 1)
    {
        return "";
    }

    if (this->going_away_limit != std::numeric_limits<size_t>::max() && this->going_away >= this->going_away_limit)
    {
        return "";
    }

    if ((!this->steal_during_termination && this->ag.has_a_terminated_actor))
    {
        return "";
    }

// If we are stealing from the rank that has worked the most, then this check is not necessary
#ifndef STEAL_FROM_BUSY_RANK
    /*
    if (!this->ag.taskDeque.canWork())
    {
        return "";
    }
    */
#endif

    if (order_victims)
    {
        std::set<ActorImpl *> &lacts = this->ag.getLocalActorsRef();
        std::vector<std::pair<ActorImpl *, float>> zipped;
        zipped.reserve(lacts.size());
        size_t i = 0;
        for (auto *aref : lacts)
        {
            zipped.emplace_back(aref, aref->getCost());
            i += 1;
        }

        std::sort(zipped.begin(), zipped.end(), [](const auto &l, const auto &r) { return l.second > r.second; });

        for (auto &pr : zipped)
        {
            ActorImpl *ai = pr.first;
            bool marked = ai->mark(marker); // std::get<1>(tryToAcquireLockForActor(ai->getNameRef(), marker));
            if (marked)
            {
                going_away += 1;
                stealing_from_me.insert(marker);
                return ai->getName();
            }
        }
    }
    else
    {
        if (must_border == -1)
        {
            auto &set = this->ag.getUnorderedLocalActorsRef();
            int tries = 0;
            for (auto *ai : set)
            {
                if (tries > 5)
                {
                    std::string s;
                    return s;
                }

                // if (ai->actable())
                {
                    bool marked = ai->mark(marker); // std::get<1>(tryToAcquireLockForActor(ai->getNameRef(), marker));
                    if (marked)
                    {
                        going_away += 1;
                        stealing_from_me.insert(marker);
                        return ai->getName();
                    }
                }
                tries += 1;
            }
        }
        else
        {
            auto v = this->pg.neighborsOf(must_border, from);
            // int tries = 0;
            for (auto &ainame : v)
            {
                GlobalActorRef r = this->ag.getActorNoThrow(ainame);
                if (r == nullptr || r.where() != upcxx::rank_me())
                {
                    continue;
                }
                ActorImpl *ai = *r.local();

                // if (ai->actable())
                {
                    bool marked = ai->mark(marker); // std::get<1>(tryToAcquireLockForActor(ai->getNameRef(), marker));
                    if (marked)
                    {
                        going_away += 1;
                        stealing_from_me.insert(marker);
                        return ai->getName();
                    }
                }
                // tries += 1;
            }
        }
    }
    std::string s;
    return s;
}
