/*
    X       x
        X
    X       X

*/

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

#include "ActorGraph.hpp"
#include "AbstractInPort.hpp"
#include "AbstractOutPort.hpp"
#include "ActorImpl.hpp"
#include "Channel.hpp"
#include "InPort.hpp"
#include "OutPort.hpp"
#include "SerializeOptional.hpp"
#include "config.hpp"
#include <chrono>

//#define DEBUG_ACTOR_TERMINATION
#define ACTOR_PARALLEL_TERMINATION_HACK

using namespace std::string_literals;

GlobalChannelRef connectDestination(GlobalActorRef destinationActor, std::string destinationPortName);
upcxx::future<> connectSource(GlobalActorRef sourceActor, std::string sourcePortName, GlobalChannelRef channelRef);
GlobalChannelRef disconnectDestination(GlobalActorRef destinationActor, std::string destinationPortName);
upcxx::future<> disconnectSource(GlobalActorRef sourceActor, std::string sourcePortName);

// ActorGraph instance methods

ActorGraph::ActorGraph(MigrationDispatcher *dag, PortGraph *pg)
    : actors(), migcaller(dag), taskDeque(this), remoteGraphComponents(this), totalTime(0), totalTokens(0),
      pg(pg), localActors{},

#ifdef PARALLEL
      actorLock(), activeActorCount(0), rpcsInFlight(0), lpcsInFlight(0),
#else
      activeActorCount(0), rpcsInFlight(0), lpcsInFlight(0),
#endif
      taskCount(upcxx::new_<unsigned int>(0))
{
    if (!upcxx::rank_me())
    {
        // std::cout << config::configToString();
    }

    auto a = util::timepoint();

    gptrsToTaskCounts.resize(upcxx::rank_n());
    std::vector<upcxx::future<>> v;
    v.reserve(upcxx::rank_n() - 1);
    for (int i = 0; i < upcxx::rank_n(); i++)
    {
        if (i == upcxx::rank_me())
        {
            continue;
        }
        upcxx::future<upcxx::global_ptr<unsigned int>> tfut = taskCount.fetch(i);
        upcxx::future<> f = tfut.then([this, i](upcxx::global_ptr<unsigned int> gptr) { gptrsToTaskCounts[i] = gptr; });
        v.push_back(std::move(f));
    }

    upcxx::discharge();
    upcxx::future<> ptrexchanged = util::combineFutures(std::move(v));
    ptrexchanged.wait();

    upcxx::barrier();
    auto b = util::timerDifference(a);
    // if (upcxx::rank_me() == 0)
    //{
    //     std::cout << "Exchanging global pointers took: " << b << " seconds" << std::endl;
    // }
}

void ActorGraph::addActor(Actor *a) { addActor(&(a->asBase())); }

void ActorGraph::addActor(ActorImpl *a) { addActorAsync(a).wait(); }

upcxx::future<> ActorGraph::addActorAsync(Actor *a) { return addActorAsync(&(a->asBase())); }

upcxx::future<> ActorGraph::addActorAsync(ActorImpl *a)
{
    auto aGPtr = upcxx::new_<ActorImpl *>(a);
    a->parentActorGraph = this;
    a->broadcastTaskDeque(&(this->taskDeque));
    localActors.insert(a);
    unorderedLocalActors.insert(a);
    actorCount += 1;
    checkInsert(a->name, aGPtr);
    this->workDone += std::get<1>(a->getWork());

    unsigned int *tc = this->taskCount->local();
    *tc += a->canActNTimes();

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
            [](upcxx::dist_object<ActorGraph *> &rag, GlobalActorRef newActorRef, std::string newActorName)
            { (*rag)->checkInsert(newActorName, newActorRef); },
            remoteGraphComponents, aGPtr, a->name);
    }
    upcxx::future<> fut = allDone.finalize();
    return fut;
}

upcxx::future<> ActorGraph::addActorAsync(std::vector<ActorImpl *> &actors)
{
    std::vector<GlobalActorRef> grefs;
    grefs.reserve(actors.size());
    std::vector<std::string> gnames;
    gnames.reserve(actors.size());

    for (auto *a : actors)
    {
        auto aGPtr = upcxx::new_<ActorImpl *>(a);
        a->parentActorGraph = this;
        a->broadcastTaskDeque(&(this->taskDeque));
        localActors.insert(a);
        unorderedLocalActors.insert(a);
        actorCount += 1;
        checkInsert(a->name, aGPtr);
        this->workDone += std::get<1>(a->getWork());

        unsigned int *tc = this->taskCount->local();
        *tc += a->canActNTimes();
        grefs.push_back(aGPtr);
        gnames.push_back(a->getName());
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
            [](upcxx::dist_object<ActorGraph *> &rag, std::vector<GlobalActorRef> newActorRefs,
               std::vector<std::string> newActorNames)
            {
                for (size_t i = 0; i < newActorRefs.size(); i++)
                {
                    (*rag)->checkInsert(newActorNames[i], newActorRefs[i]);
                }
            },
            remoteGraphComponents, grefs, gnames);
    }
    upcxx::future<> fut = allDone.finalize();
    return fut;
}

// ActorImpl must be initialized at the rank "rank"
upcxx::future<> ActorGraph::addActorToAnotherAsync(GlobalActorRef a, upcxx::intrank_t rank)
{
    if (rank != a.where())
    {
        throw std::runtime_error("precondition: ActorImpl must be initialized at "
                                 "destination rank no fulfilled");
    }

    if (rank == upcxx::rank_me())
    {
        ActorImpl *act = *a.local();
        act->parentActorGraph = this;
        act->broadcastTaskDeque(&(this->taskDeque));
        auto pr = localActors.insert(act);
        unorderedLocalActors.insert(act);
        if (!pr.second)
        {
            throw std::runtime_error("inserting a new actor to local actors should not fail!");
        }
        this->checkInsert(act->getName(), a);
        actorCount += 1;
        this->workDone += std::get<1>(act->getWork());

        unsigned int *tc = this->taskCount->local();
        *tc += act->canActNTimes();

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
                [](upcxx::dist_object<ActorGraph *> &rag, GlobalActorRef newActorRef, std::string newActorName)
                { (*rag)->checkInsert(newActorName, newActorRef); },
                remoteGraphComponents, a, act->getNameRef());
        }
        return allDone.finalize();
    }
    else
    {
        upcxx::future<std::string> nm = upcxx::rpc(
            a.where(),
            [](GlobalActorRef ref) -> std::string
            {
                ActorImpl *ai = *ref.local();
                return ai->getName();
            },
            a);

        upcxx::future<std::string> lk = nm.then(
            [this, a](const std::string name) -> std::string
            {
                this->checkInsert(name, a);
                return name;
            });

        upcxx::future<> xcc = lk.then(
            [this, a](const std::string name) -> upcxx::future<>
            {
                upcxx::promise<> allDone;
                auto cx1 = upcxx::operation_cx::as_promise(allDone);
                upcxx::rpc(
                    a.where(), cx1,
                    [](upcxx::dist_object<ActorGraph *> &rag, GlobalActorRef ref)
                    {
                        ActorImpl *a = *ref.local();
                        a->parentActorGraph = (*rag);
                        auto pr = (*rag)->localActors.insert(a);
                        (*rag)->unorderedLocalActors.insert(a);
                        if (!pr.second)
                        {
                            throw std::runtime_error("inserting a new actor to local actors should not fail!");
                        }
                        a->broadcastTaskDeque(&(*rag)->taskDeque);
                        (*rag)->checkInsert(a->getName(), ref);
                        (*rag)->actorCount += 1;
                        (*rag)->workDone += std::get<1>(a->getWork());

                        unsigned int *tc = (*rag)->taskCount->local();
                        *tc += a->canActNTimes();
                    },
                    remoteGraphComponents, a);

                for (int i = 0; i < upcxx::rank_n(); i++)
                {
                    if (i == upcxx::rank_me() || i == a.where())
                    {
                        continue;
                    }
                    auto cxi = upcxx::operation_cx::as_promise(allDone);
                    upcxx::rpc(
                        i, cxi,
                        [](upcxx::dist_object<ActorGraph *> &rag, GlobalActorRef newActorRef, std::string newActorName)
                        { (*rag)->checkInsert(newActorName, newActorRef); },
                        remoteGraphComponents, a, name);
                }
                return allDone.finalize();
            });

        // upcxx::future<> rt = upcxx::when_all(std::move(lk), std::move(xcc));
        return xcc;
    }
}

std::tuple<upcxx::future<>, GlobalActorRef> ActorGraph::rmActorAsync(const std::string &name)
{
    return rmActorAsync(name, ActorState::Terminated);
}

std::tuple<upcxx::future<>, GlobalActorRef> ActorGraph::rmActorAsync(const std::string &name, ActorState as)
{
    GlobalActorRef a = actors.find(name)->second;

    if (a.where() == upcxx::rank_me())
    {
        unsigned int *tc = this->taskCount->local();
        ActorImpl *x = *a.local();
        unorderedLocalActors.erase(x);
        // should already be stopped
        // x->stopWithCaller(x, as);
        size_t erased = localActors.erase(x);

        if (erased != 1)
        {
            throw std::runtime_error("Could not erase actor from localActors");
        }

        bool couldrm = checkRm(x->getName());
        actorCount -= 1;
        this->workDone -= std::get<1>(x->getWork());

        if (*tc < x->canActNTimes())
        {
            // throw std::runtime_error("TaskCount underflow at rm");
            *tc = 0;
        }
        else
        {
            *tc -= x->canActNTimes();
        }

        if (!couldrm)
        {
            throw std::runtime_error("ActorImpl not in actorgraph, same rank, rmActor");
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
                i, cx, [](upcxx::dist_object<ActorGraph *> &rag, std::string actorName) { (*rag)->checkRm(actorName); },
                remoteGraphComponents, x->getName());
        }
        upcxx::future<> fut = allDone.finalize();
        return {fut, a};
    }
    else
    {
        upcxx::promise<> allDone;
        bool couldrm = checkRm(name);
        if (!couldrm)
        {
            throw std::runtime_error("ActorImpl not in actorgraph, diff rank, rmActor");
        }

        for (int i = 0; i < upcxx::rank_n(); i++)
        {
            if (i == upcxx::rank_me())
            {
                continue;
            }

            if (i == a.where())
            {
                auto cx = upcxx::operation_cx::as_promise(allDone);
                upcxx::rpc(
                    i, cx,
                    [](upcxx::dist_object<ActorGraph *> &rag, std::string actorName)
                    {
                        GlobalActorRef aref = (*rag)->getActorNoThrow(actorName);
                        if (aref == nullptr)
                        {
                            throw std::runtime_error("Actor should be present in graph during rm");
                        }

                        ActorImpl *a = *aref.local();

                        unsigned int *tc = ((*rag)->taskCount->local());

                        size_t erased = (*rag)->localActors.erase(a);
                        (*rag)->unorderedLocalActors.erase(a);

                        if (erased != 1)
                        {
                            throw std::runtime_error("Could not erase actor from localActors");
                        }

                        (*rag)->checkRm(actorName);
                        (*rag)->actorCount -= 1;
                        (*rag)->workDone -= std::get<1>(a->getWork());

                        if (*tc < a->canActNTimes())
                        {
                            // throw std::runtime_error("TaskCount underflow at rm");
                            *tc = 0;
                        }
                        else
                        {
                            *tc -= a->canActNTimes();
                        }
                    },
                    remoteGraphComponents, name);
            }
            else
            {
                auto cx = upcxx::operation_cx::as_promise(allDone);
                upcxx::rpc(
                    i, cx,
                    [](upcxx::dist_object<ActorGraph *> &rag, std::string actorName) { (*rag)->checkRm(actorName); },
                    remoteGraphComponents, name);
            }
        }
        auto fut = allDone.finalize();
        return std::make_tuple(std::move(fut), std::move(a));
    }
}

void ActorGraph::rmActor(const std::string &name)
{
    auto tup = rmActorAsync(name);
    std::get<0>(tup).wait();
}

bool ActorGraph::checkRm(const std::string &actorName)
{
    if (this->actors.find(actorName) == this->actors.end())
    {
        throw std::runtime_error("Cannot rm what does not exist");
    }
#ifdef PARALLEL
    this->actorLock.lock();
#endif

    this->actors.erase(actorName);

#ifdef PARALLEL
    this->actorLock.unlock();
#endif

    return true;
}

void ActorGraph::checkInsert(const std::string &actorName, GlobalActorRef actorRef)
{
    if (this->actors.find(actorName) != this->actors.end())
    {
        throw std::runtime_error("Cannot add what does already exist");
    }
#ifdef PARALLEL
    this->actorLock.lock();
#endif

    this->actors.emplace(actorName, actorRef);

#ifdef PARALLEL
    this->actorLock.unlock();
#endif
}

void ActorGraph::connectPorts(GlobalActorRef sourceActor, const std::string &sourcePortName,
                              GlobalActorRef destinationActor, const std::string &destinationPortName)
{
    throw std::runtime_error("Call the async of this procedure!");

    /*
    if (sourcePortName.empty() or destinationPortName.empty())
    {
        throw std::runtime_error("connectPorts port names have empty strings (one or both)");
    }

    if (destinationActor.where() == upcxx::rank_me())
    {
        connectFromDestination(sourceActor, sourcePortName, destinationActor, destinationPortName).wait();
    }
    else if (sourceActor.where() == upcxx::rank_me())
    {
        connectFromSource(sourceActor, sourcePortName, destinationActor, destinationPortName).wait();
    }
    else
    {
        connectFromThird(sourceActor, sourcePortName, destinationActor, destinationPortName).wait();
    }
    */
}

void ActorGraph::disconnectPorts(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                 GlobalActorRef destinationActor, const std::string &destinationPortName)
{

    disconnectPortsAsync(sourceActor, sourcePortName, destinationActor, destinationPortName).wait();
}

upcxx::future<> ActorGraph::disconnectPortsAsync(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                                 GlobalActorRef destinationActor,
                                                 const std::string &destinationPortName)
{
    if (sourcePortName.empty() or destinationPortName.empty())
    {
        throw std::runtime_error("connectPorts port names have empty strings (one or both)");
    }

    if (sourceActor.where() == upcxx::rank_me())
    {
        return disconnectFromSource(sourceActor, sourcePortName, destinationActor, destinationPortName);
    }
    else if (destinationActor.where() == upcxx::rank_me())
    {
        throw std::runtime_error("Don't use until I improve this, make sure sourceActor is at your rank, ty "
                                 "(disconnectFromDestination was going to be called)");
        // return disconnectFromDestination(sourceActor, sourcePortName, destinationActor, destinationPortName);
    }
    else
    {
        throw std::runtime_error("Don't use until I improve this, make sure sourceActor is at your rank, ty "
                                 "(disconnectFromDestination was going to be called)");
        // return disconnectFromThird(sourceActor, sourcePortName, destinationActor, destinationPortName);
    }
}

upcxx::future<> ActorGraph::connectPortsAsync(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                              GlobalActorRef destinationActor, const std::string &destinationPortName)
{
    if (sourcePortName.empty() or destinationPortName.empty())
    {
        throw std::runtime_error("connectPorts port names have empty strings (one or both)");
    }

    if (sourceActor.where() == upcxx::rank_me())
    {
        return connectFromSource(sourceActor, sourcePortName, destinationActor, destinationPortName);
    }
    else if (destinationActor.where() == upcxx::rank_me())
    {

        throw std::runtime_error("Don't use until I improve this, make sure sourceActor is at your rank, ty "
                                 "(connectFromDestination was going to be called)");
        // return connectFromDestination(sourceActor, sourcePortName, destinationActor, destinationPortName);
    }
    else
    {
        throw std::runtime_error("Don't use until I improve this, make sure sourceActor is at your rank, ty "
                                 "(connectFromThird was going to be called");
        // return connectFromThird(sourceActor, sourcePortName, destinationActor, destinationPortName);
    }
}

upcxx::future<> ActorGraph::connectFromDestination(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                                   GlobalActorRef destinationActor,
                                                   const std::string &destinationPortName)
{
    GlobalChannelRef c = connectDestination(destinationActor, destinationPortName);
    auto srcFut = upcxx::rpc(sourceActor.where(), connectSource, sourceActor, sourcePortName, c);
    return srcFut;
}

upcxx::future<> ActorGraph::connectFromSource(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                              GlobalActorRef destinationActor, const std::string &destinationPortName)
{
    if (destinationActor.where() != upcxx::rank_me())
    {
        upcxx::future<GlobalChannelRef> channelFut =
            upcxx::rpc(destinationActor.where(), connectDestination, destinationActor, destinationPortName);
        upcxx::future<> sourceDone = channelFut.then(
            [sourceActor, sourcePortName](GlobalChannelRef c)
            {
                if (sourceActor.where() != upcxx::rank_me())
                {
                    throw std::runtime_error("sourceActor should be at my rank if we call connectFromSource");
                }

                return connectSource(sourceActor, sourcePortName, c);
            });
        return sourceDone;
    }
    else
    {
        GlobalChannelRef c = connectDestination(destinationActor, destinationPortName);
        return connectSource(sourceActor, sourcePortName, c);
    }
}

upcxx::future<> ActorGraph::connectFromThird(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                             GlobalActorRef destinationActor, const std::string &destinationPortName)
{
    upcxx::future<> allDone = upcxx::rpc(
        sourceActor.where(),
        [](upcxx::dist_object<ActorGraph *> &rag, GlobalActorRef sourceActor, const std::string &sourcePortName,
           GlobalActorRef destinationActor, std::string destinationPortName)
        { return (*rag)->connectFromSource(sourceActor, sourcePortName, destinationActor, destinationPortName); },
        this->remoteGraphComponents, sourceActor, sourcePortName, destinationActor, destinationPortName);
    return allDone;
}

upcxx::future<> ActorGraph::disconnectFromDestination(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                                      GlobalActorRef destinationActor,
                                                      const std::string &destinationPortName)
{
    auto srcFut = upcxx::rpc(sourceActor.where(), disconnectSource, sourceActor, sourcePortName);
    return srcFut;
}

upcxx::future<> ActorGraph::disconnectFromSource(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                                 GlobalActorRef destinationActor,
                                                 const std::string &destinationPortName)
{
    if (destinationActor.where() != upcxx::rank_me())
    {
        upcxx::future<GlobalChannelRef> channelFut =
            upcxx::rpc(destinationActor.where(), disconnectDestination, destinationActor, destinationPortName);
        upcxx::future<> sourceDone = channelFut.then(
            [sourceActor, sourcePortName](GlobalChannelRef _c)
            {
                upcxx::future<> dced = disconnectSource(sourceActor, sourcePortName);
                return dced;
            });
        return sourceDone;
    }
    else
    {
        disconnectDestination(destinationActor, destinationPortName);
        upcxx::future<> dced = disconnectSource(sourceActor, sourcePortName);
        return dced;
    }
}

upcxx::future<> ActorGraph::disconnectFromThird(GlobalActorRef sourceActor, const std::string &sourcePortName,
                                                GlobalActorRef destinationActor, const std::string &destinationPortName)
{
    upcxx::future<> allDone = upcxx::rpc(
        sourceActor.where(),
        [](upcxx::dist_object<ActorGraph *> &rag, GlobalActorRef sourceActor, const std::string &sourcePortName,
           GlobalActorRef destinationActor, std::string destinationPortName)
        { return (*rag)->disconnectFromSource(sourceActor, sourcePortName, destinationActor, destinationPortName); },
        this->remoteGraphComponents, sourceActor, sourcePortName, destinationActor, destinationPortName);
    return allDone;
}

size_t ActorGraph::getTotalActorCount() const { return actors.size(); }

GlobalActorRef ActorGraph::getActor(const std::string &name) const
{
    auto entry = actors.find(name);
    if (entry != actors.end())
    {
        return entry->second;
    }
    else
    {
        throw std::runtime_error(std::to_string(upcxx::rank_me()) + ": unable to find ActorImpl named "s + name);
    }
}

bool ActorGraph::has(const std::string &name) const
{
    auto entry = actors.find(name);
    if (entry != actors.end())
    {
        return true;
    }
    else
    {
        return false;
    }
}

GlobalActorRef ActorGraph::getActorNoThrow(const std::string &name) const
{
    auto entry = actors.find(name);
    if (entry != actors.end())
    {
        return entry->second;
    }
    else
    {
        return nullptr;
    }
}

std::string ActorGraph::prettyPrint()
{
    std::stringstream ss;
    ss << "ActorGraph {" << std::endl;
    ss << "  Actors {" << std::endl;

    for (auto &kv : this->actors)
    {
        ss << "    " << kv.first << "\t" << kv.second << std::endl;
    }
    ss << "  }" << std::endl;
    ss << "ActiveActors: " << activeActorCount << std::endl;
    ss << "}" << std::endl;

    return ss.str();
}

std::pair<double, bool> ActorGraph::ompRun(double seconds)
{
    throw std::runtime_error("OMP run called withput OMP Config!");
}

std::pair<double, bool> ActorGraph::serialRun(double seconds)
{
    bool finished = true;
    if (config::parallelization == config::ParallelizationType::UPCXX_RANKS)
    {
        auto start = std::chrono::steady_clock::now();

        generateAndAddTasks();
        finished = taskDeque.advance(seconds);

        auto end = std::chrono::steady_clock::now();
        double runTime = std::chrono::duration<double, std::ratio<1>>(end - start).count();

        return std::make_pair(runTime, finished);
    }
    else
    {
        throw std::runtime_error("Serial Run should not be called with parallel UPCXX / OMP Config");
    }
}

std::pair<double, bool> ActorGraph::run(double opt_seconds, bool firsttime)
{
    this->migrationphase = false;
    double runTime = 0.0;
    bool finished = false;

    if (firsttime)
    {
        this->startActors();

        upcxx::barrier();

        firsttime = false;
    }

    if constexpr (config::parallelization == config::ParallelizationType::UPCXX_RANKS)
    {
        auto pr = serialRun(opt_seconds);
        runTime = pr.first;
        finished = pr.second;
    }
    else
    {
        auto pr = ompRun(opt_seconds);
        runTime = pr.first;
        finished = pr.second;
    }

    if (!finished)
    {
        this->migrationphase = true;
    }

    return std::make_pair(runTime, finished);
}

// Non-instance methods
GlobalChannelRef connectDestination(GlobalActorRef destinationActor, std::string destinationPortName)
{
    auto *dstActorPtr = *(destinationActor.local());
    auto *dstIp = dstActorPtr->getInPort(destinationPortName);
    auto channelRef = dstIp->getChannelPtr();
    // register with channel is not needed as it is done in the constructor of the channel now on
    // dstIp->registerWithChannel();
    return channelRef;
}

upcxx::future<> connectSource(GlobalActorRef sourceActor, std::string sourcePortName, GlobalChannelRef channelRef)
{
    auto *srcActorPtr = *(sourceActor.local());
    auto *srcOp = srcActorPtr->getOutPort(sourcePortName);
    upcxx::future<> regFut = srcOp->registerWithChannel(channelRef);
    return regFut;
}

upcxx::future<> disconnectSource(GlobalActorRef sourceActor, std::string sourcePortName)
{
    auto *srcActorPtr = *(sourceActor.local());
    auto *srcOp = srcActorPtr->getOutPort(sourcePortName);
    auto regFut = srcOp->deregister();
    return regFut;
}

GlobalChannelRef disconnectDestination(GlobalActorRef destinationActor, std::string destinationPortName)
{
    auto *dstActorPtr = *(destinationActor.local());
    auto *dstIp = dstActorPtr->getInPort(destinationPortName);
    auto channelRef = dstIp->getChannelPtr();
    dstIp->deregister();
    return channelRef;
}

ActorGraph::~ActorGraph()
{
    for (std::pair<const std::string, GlobalActorRef> &kvPair : actors)
    {
        if (kvPair.second.where() == upcxx::rank_me())
        {
            ActorImpl *a = *(kvPair.second.local());

            if (localActors.find(a) == localActors.end())
            {
                std::cerr << "Local actor and global actor hash map mismatch!" << std::endl;
            }

            delete a;
            upcxx::delete_(kvPair.second);
            localActors.erase(a);
        }
    }
}

const std::unordered_map<std::string, GlobalActorRef> *ActorGraph::getActors() const { return &actors; }

const std::unordered_map<std::string, GlobalActorRef> &ActorGraph::getActorsRef() const { return actors; }

void ActorGraph::startActors()
{
    for (auto &actorPairs : actors)
    {
        if (actorPairs.second.where() == upcxx::rank_me())
        {
            auto aRef = *(actorPairs.second.local());

#ifdef USE_ACTOR_TRIGGERS
            this->taskDeque.addTrigger(aRef->createTriggers());
#endif

            bool c = aRef->start();
            if (!c)
            {
                throw std::runtime_error("Starting an actor should not fail!");
            }
        }
    }
}

void ActorGraph::restartActors()
{
    for (auto &actorPairs : actors)
    {
        if (actorPairs.second.where() == upcxx::rank_me())
        {
            auto aRef = *(actorPairs.second.local());
            if (aRef->needsReactivation())
            {
                bool c = aRef->start();
                if (!c)
                {
                    throw std::runtime_error("Starting an actor should not fail!");
                }
            }
        }
    }
}

unsigned int ActorGraph::getActiveActors() const
{
#ifdef PARALLEL
    return this->activeActorCount.load();
#else
    return this->activeActorCount;
#endif
}

bool ActorGraph::has(const ActorImpl *a) const
{
    if (actors.find(a->getName()) != actors.end())
    {
        return true;
    }
    else
    {
        return false;
    }
}

void ActorGraph::prepare(const std::unordered_map<std::string, upcxx::intrank_t> &l)
{
    for (const auto &pr : l)
    {
        GlobalActorRef ref = this->getActorNoThrow(pr.first);
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during prepare");
        }
        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *a = *ref.local();
            a->prepare();
        }
    }
}

double ActorGraph::run() { return run(std::numeric_limits<double>::max(), true).first; }

std::tuple<uint64_t, uint64_t> ActorGraph::calcRankWork() const
{

    std::tuple<uint64_t, uint64_t> work{0, 0};

    for (const auto &a : this->actors)
    {
        if (a.second.where() == upcxx::rank_me())
        {
            ActorImpl *act = *(a.second.local());
            auto tup = act->getWork();
            std::get<0>(work) += std::get<0>(tup);
            std::get<1>(work) += std::get<1>(tup);
        }
    }
    return work;
}

void ActorGraph::addToWork(uint64_t time)
{
    this->totalTime += time;
    this->totalTokens += 1;
    this->workDone += static_cast<double>(time);
}

std::tuple<uint64_t, uint64_t> ActorGraph::getTotalWork() const { return {this->totalTokens, this->totalTime}; }

uint64_t ActorGraph::getWorkDoneForMigration() const
{ // return this->workDone;
    uint64_t u = 0;
    for (auto *ai : localActors)
    {
        u += static_cast<uint64_t>(ai->getCost());
    }
    return u;
}

void ActorGraph::addNoOverflow(uint64_t token, uint64_t time)
{
    this->totalTokens = util::addNoOverflow(this->totalTokens, token);
    this->totalTime = util::addNoOverflow(this->totalTime, time);
}

void actAsTask(ActorImpl *ai) { ai->act(); }

void ActorGraph::generateAndAddTasks()
{
    for (const auto &at : actors)
    {
        if (at.second.where() == upcxx::rank_me())
        {
            GlobalActorRef r = at.second;
            ActorImpl *at = *r.local();
            at->set_port_size();
            at->broadcastTaskDeque(&(this->taskDeque));
        }
    }
}

TaskDeque *ActorGraph::getTaskDeque() { return &(this->taskDeque); }

upcxx::future<> ActorGraph::changeRef(const std::string &name, GlobalActorRef ref)
{

    upcxx::promise<> allDone;
    for (int i = 0; i < util::rank_n(); i++)
    {
        if (i == upcxx::rank_me())
        {
            this->checkInsert(name, ref);
        }
        auto cx = upcxx::operation_cx::as_promise(allDone);
        upcxx::rpc(
            i, cx,
            [](upcxx::dist_object<ActorGraph *> &rag, GlobalActorRef newActorRef, std::string &&newActorName)
            { (*rag)->checkInsert(newActorName, newActorRef); },
            remoteGraphComponents, ref, name);
    }

    return allDone.finalize();
}

// bool ActorGraph::hasActTasks(ActorImpl *ai) const { return this->taskDeque.hasActTasks(ai->getNameRef()); }

// bool ActorGraph::hasNoActTasks(ActorImpl *ai) const { return this->taskDeque.hasNoActTasks(ai->getNameRef()); }

void ActorGraph::printActorStates() const
{
    std::cout << "{";
    for (const auto *el : localActors)
    {
        std::cout << "(" << el->getName() << ", " << ASPrinter::as2str(el->getRunningState()) << ") ";
    }
    std::cout << "}" << std::endl;
}

#ifdef USE_ACTOR_TRIGGERS
upcxx::future<> ActorGraph::sendTriggers(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    return this->taskDeque.sendTriggers(migList);
}

ActorTriggers ActorGraph::moveTriggers(const std::string &name) { return this->taskDeque.moveTriggers(name); }

void ActorGraph::addTriggers(ActorTriggers &&att) { this->taskDeque.addTrigger(std::move(att)); }
#endif

void ActorGraph::setRunning(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    for (const auto &pr : migList)
    {
        GlobalActorRef ref = this->getActorNoThrow(pr.first);
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during setrunning");
        }
        if (ref.where() == upcxx::rank_me())
        {
            ActorImpl *ai = *ref.local();
            ai->setRunning();
        }
    }
}

std::set<ActorImpl *> &ActorGraph::getLocalActorsRef() { return localActors; }

std::unordered_set<ActorImpl *> &ActorGraph::getUnorderedLocalActorsRef() { return unorderedLocalActors; }

float ActorImpl::getCost() const { return actor_cost[index_for_actor_cost]; }

unsigned int ActorGraph::getTaskCount() const { return taskDeque.getTaskCount(); }

bool ActorGraph::validPointer(ActorImpl *ai) const
{
    bool in_local_map = localActors.find(ai) != localActors.end();
    return in_local_map;
}

void ActorGraph::addTask()
{
    unsigned int *tc = this->taskCount->local();
    *tc += 1;
}

unsigned int ActorGraph::getTaskCountActive() const
{
    unsigned int tc = 0;
    for (const auto *a : localActors)
    {
        tc += a->canActNTimes();
    }
    return tc;
}