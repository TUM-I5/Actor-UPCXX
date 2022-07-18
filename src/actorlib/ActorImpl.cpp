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

#include "ActorImpl.hpp"

#include "InPort.hpp"
#include "OutPort.hpp"

#include "ActorGraph.hpp"

#include <sstream>
#include <string>

#ifdef TRACE
int ActorImpl::event_act = -1;
#endif

using namespace std::string_literals;

ActorImpl::ActorImpl(const std::string &name)
    : name(name), parentActorGraph(nullptr), state(ActorState::NeedsActivation), portsInfoApplicable(false),
      workTokens(0), workTime(0), markedBy{-1}, pinnedBy{0}, index_for_actor_cost(0)
{
#ifdef TRACE
    std::string event_act_name = "act";
    if (ActorImpl::event_act == -1)
        VT_funcdef(event_act_name.c_str(), VT_NOCLASS, &ActorImpl::event_act);
#endif
    initPinCount();
}

ActorImpl::ActorImpl()
    : name(""), parentActorGraph(nullptr), state(ActorState::NeedsActivation), portsInfoApplicable(false),
      workTokens(0), workTime(0), markedBy{-1}, pinnedBy{0}, index_for_actor_cost(0)
{
#ifdef TRACE
    std::string event_act_name = "act";
    if (ActorImpl::event_act == -1)
        VT_funcdef(event_act_name.c_str(), VT_NOCLASS, &ActorImpl::event_act);
#endif
    initPinCount();
}

ActorImpl::ActorImpl(std::string &&name)
    : name(std::move(name)), parentActorGraph(nullptr), state(ActorState::NeedsActivation), portsInfoApplicable(false),
      workTokens(0), workTime(0), markedBy{-1}, pinnedBy{0}, index_for_actor_cost(0)
{
#ifdef TRACE
    std::string event_act_name = "act";
    if (ActorImpl::event_act == -1)
        VT_funcdef(event_act_name.c_str(), VT_NOCLASS, &ActorImpl::event_act);
#endif
    initPinCount();
}

ActorImpl::ActorImpl(ActorImpl &&act)
    : name(std::move(act.name)), parentActorGraph(act.parentActorGraph), inPorts(std::move(act.inPorts)),
      outPorts(std::move(act.outPorts)), state(act.state.load()), portsInformation(std::move(act.portsInformation)),
      portsInfoApplicable(act.portsInfoApplicable), workTokens(act.workTokens), workTime(act.workTime),
      markedBy(std::move(act.markedBy)), pinnedBy{0}, pin_count(std::move(act.pin_count)),
      actor_cost(std::move(act.actor_cost)), index_for_actor_cost(act.index_for_actor_cost),
      initial_act(act.initial_act), terminated(act.terminated)
{
    if (portsInfoBadState())
    {
        throw std::runtime_error("In current implementation move is only called during serialization thus the "
                                 "portsInfo should be in a valid state");
    }

    act.portsInfoApplicable = false;
    std::get<0>(act.portsInformation).clear();
    std::get<1>(act.portsInformation).clear();

    for (auto &pr : inPorts)
    {
        pr.second->connectedActor = this;
    }

    for (auto &pr : outPorts)
    {
        pr.second->connectedActor = this;
    }

    if (terminated)
    {
        state = ActorState::Terminated;
    }

    if (!portsInfoApplicable)
    {
        throw std::runtime_error("After a fresh move constructor the port information should be applicable");
    }
}

ActorImpl &ActorImpl::operator=(ActorImpl &&act)
{
    name = std::move(act.name);
    parentActorGraph = act.parentActorGraph;
    state = act.state.load();
    portsInformation = std::move(act.portsInformation);
    portsInfoApplicable = act.portsInfoApplicable;
    act.portsInfoApplicable = false;
    workTokens = act.workTokens;
    workTime = act.workTime;
    markedBy = std::move(act.markedBy);
    pinnedBy = act.pinnedBy;
    pin_count = std::move(act.pin_count);
    actor_cost = std::move(act.actor_cost);
    index_for_actor_cost = act.index_for_actor_cost;

    inPorts = std::move(act.inPorts);
    outPorts = std::move(act.outPorts);
    terminated = act.terminated;

    for (auto &pr : inPorts)
    {
        pr.second->connectedActor = this;
    }

    for (auto &pr : outPorts)
    {
        pr.second->connectedActor = this;
    }

    if (terminated)
    {
        state = ActorState::Terminated;
    }

    if (!portsInfoApplicable)
    {
        throw std::runtime_error("After a fresh move constructor the port information should be applicable");
    }
    return *this;
}

void ActorImpl::update_ports(std::vector<AbstractInPort *> ins, std::vector<AbstractOutPort *> outs)
{
    for (auto &pr : inPorts)
    {
        auto *a = pr.second;
        delete a;
    }
    for (auto &pr : outPorts)
    {
        auto *a = pr.second;
        delete a;
    }
    inPorts.clear();
    outPorts.clear();

    for (auto *el : ins)
    {
        if (el != nullptr)
        {
            inPorts.insert(std::make_pair(el->getName(), el));
        }
    }

    for (auto *el : outs)
    {
        if (el != nullptr)
        {
            outPorts.insert(std::make_pair(el->getName(), el));
        }
    }
}

ActorImpl::ActorImpl(ActorData &&data,
                     std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                std::vector<std::unique_ptr<OutPortInformationBase>>> &&portsInformation)
    : name(std::move(data.name)), parentActorGraph(nullptr), inPorts(), outPorts(), state(data.state),
      portsInformation(std::move(portsInformation)), portsInfoApplicable(true), workTokens(data.workTokens),
      workTime(data.workTime), markedBy(data.mark), actor_cost(std::move(data.actor_cost)),
      index_for_actor_cost(data.index), initial_act(data.initial_act), terminated(data.term)
{
#ifdef TRACE
    std::string event_act_name = "act";
    if (ActorImpl::event_act == -1)
        VT_funcdef(event_act_name.c_str(), VT_NOCLASS, &ActorImpl::event_act);
#endif
    // called = -1;
    initPinCount();

    if (isPinned())
    {
        throw std::runtime_error("Freshly deserialized actor should not be pinned!");
    }

    if (terminated)
    {
        state = ActorState::Terminated;
    }
}

ActorImpl::ActorImpl(ActorData &&data)
    : name(std::move(data.name)), parentActorGraph(nullptr), inPorts(), outPorts(),
      state(data.state), portsInformation{}, portsInfoApplicable(true), workTokens(data.workTokens),
      workTime(data.workTime), markedBy(data.mark), actor_cost(std::move(data.actor_cost)),
      index_for_actor_cost(data.index), initial_act(data.initial_act), terminated(data.term)
{
#ifdef TRACE
    std::string event_act_name = "act";
    if (ActorImpl::event_act == -1)
        VT_funcdef(event_act_name.c_str(), VT_NOCLASS, &ActorImpl::event_act);
#endif
    // called = -1;
    initPinCount();

    if (isPinned())
    {
        throw std::runtime_error("Freshly deserialized actor should not be pinned!");
    }

    if (terminated)
    {
        state = ActorState::Terminated;
    }
}

void ActorImpl::initPinCount()
{
    pin_count.resize(util::rank_n());
    std::fill(pin_count.begin(), pin_count.end(), 0);
    pinnedBy = 0;
}

ActorImpl::~ActorImpl()
{
    for (auto &aip : this->inPorts)
    {
        delete aip.second;
    }
    for (auto &aop : this->outPorts)
    {
        if (aop.second->bufferSize() != 0)
        {
            std::cerr << upcxx::rank_me() << ": Buffered messages should be sent before destructor is called!"
                      << std::endl;
        }
        delete aop.second;
    }
}

std::string ActorImpl::toString()
{
    std::stringstream ss;
    ss << "Actor( " << name
       << " ) is Pinned: " + std::to_string(isPinned()) + ", is Makred: " + std::to_string(isMarked()) +
              +", status:" + ASPrinter::as2str(this->state) + " { ";
    for (auto &aip : this->inPorts)
    {
        ss << aip.second->toString() << " ";
    }
    for (auto &aop : this->outPorts)
    {
        ss << aop.second->toString() << " ";
    }
    ss << "}";
    return ss.str();
}

std::pair<std::vector<std::pair<std::string, AbstractInPort *>>, std::vector<std::pair<std::string, AbstractOutPort *>>>
ActorImpl::getPorts() const
{
    std::pair<std::vector<std::pair<std::string, AbstractInPort *>>,
              std::vector<std::pair<std::string, AbstractOutPort *>>>
        x(getAllInPorts(), getAllOutPorts());
    return x;
}

// std::unordered_map<std::string, AbstractInPort *> inPorts;
// std::unordered_map<std::string, AbstractOutPort *> outPorts;
// transforms the unordered map of inports to a vector of pairs
// and returns the vector
std::vector<std::pair<std::string, AbstractInPort *>> ActorImpl::getAllInPorts() const
{
    std::vector<std::pair<std::string, AbstractInPort *>> v;
    for (auto it = inPorts.begin(); it != inPorts.end(); ++it)
    {
        v.emplace_back(std::pair<std::string, AbstractInPort *>(it->first, it->second));
    }
    return v;
}
// transforms the unordered map of outports to a vector of pairs
// and returns the vector
std::vector<std::pair<std::string, AbstractOutPort *>> ActorImpl::getAllOutPorts() const
{
    std::vector<std::pair<std::string, AbstractOutPort *>> v;
    for (auto it = outPorts.begin(); it != outPorts.end(); ++it)
    {
        v.emplace_back(it->first, it->second);
    }
    return v;
}

std::vector<std::string> ActorImpl::getAllInPortNames() const
{
    std::vector<std::string> v;
    for (const auto &pr : inPorts)
    {
        v.emplace_back(pr.first);
    }
    return v;
}
// transforms the unordered map of outports to a vector of pairs
// and returns the vector
std::vector<std::string> ActorImpl::getAllOutPortNames() const
{
    std::vector<std::string> v;
    for (const auto &pr : outPorts)
    {
        v.emplace_back(pr.first);
    }
    return v;
}

void ActorImpl::rmInPort(const std::string &name)
{
    auto res = inPorts.find(name);
    if (res != inPorts.end())
    {
        inPorts.erase(name);
    }
    else
    {
        std::cout << "no such inport" << std::endl;
    }
}

void ActorImpl::rmOutPort(const std::string &name)
{
    auto res = outPorts.find(name);
    if (res != outPorts.end())
    {
        outPorts.erase(name);
    }
    else
    {
        std::cout << "no such outport" << std::endl;
    }
}

AbstractInPort *ActorImpl::getInPort(std::string &name)
{
    auto res = inPorts.find(name);
    if (res != inPorts.end())
    {
        return res->second;
    }
    else
    {
        throw std::runtime_error("Actor "s + this->toString() + "has no InPort with id "s + name);
    }
}

AbstractOutPort *ActorImpl::getOutPort(std::string &name)
{
    auto res = outPorts.find(name);
    if (res != outPorts.end())
    {
        return res->second;
    }
    throw std::runtime_error("Actor "s + this->toString() + "has no OutPort with name "s + name);
}

void ActorImpl::clearPortInformation()
{
    this->portsInfoApplicable = false;
    std::get<0>(this->portsInformation).clear();
    std::get<1>(this->portsInformation).clear();
}

bool ActorImpl::start() { return startWithCaller(-1); }

bool ActorImpl::startWithCaller(upcxx::intrank_t caller)
{
    prepared = false;

    if (this->state.load() == ActorState::Terminated || terminated)
    {
        markedBy = -1;
        return true;
    }

    if (caller == -1)
    {
        if (!this->parentActorGraph->migrationphase)
        {
            // std::cout << asp.as2str(state) << std::endl;
            if (state != ActorState::NeedsActivation)
            {
                throw std::runtime_error("during the initial start actors need to be in needsactivation state!");
            }
        }
    }

    // try to change an actor marked with caller
    // shpould have stopped only for migration only check that

    if (caller != markedBy)
    {
        throw std::runtime_error("Do not call start with caller if you are not the marker!");
        return false;
    }

    ActorState nac = ActorState::NeedsActivation;
    ActorState tsfm = ActorState::TemporaryStoppedForMigration;

    if (this->state.compare_exchange_strong(nac, ActorState::Transitioning) ||
        (this->state.compare_exchange_strong(tsfm, ActorState::Transitioning)))
    {
#if defined(REPORT_MAIN_ACTIONS)
        // std::cout << "Starting actor " << this->name << " on rank " << std::to_string(upcxx::rank_me()) << " by "
        //           << caller << std::endl;
#endif

        this->parentActorGraph->activeActorCount += 1;

        this->state = ActorState::Running;

        this->markedBy = -1;

        set_port_size();

        clearPortInformation();
        return true;
    }

    throw std::runtime_error(std::to_string(markedBy) + " " + std::to_string(caller) + " " +
                             ASPrinter::as2str(this->state) + " Oh sdsdss!");

    return false;
}

bool ActorImpl::stop() { return this->stopFromAct(); }

bool ActorImpl::tryLock(ActorState expected, ActorState desired)
{
    return this->state.compare_exchange_strong(expected, desired);
}

bool ActorImpl::mark(upcxx::intrank_t marker)
{
    if (!acted || isMarked() || isPinned() || state != ActorState::Running || initial_act || terminated)
    {
        return false;
    }

    if (isMarked() || isPinned())
    {
        throw std::runtime_error("By here it should not be marked or pinned");
    }

    markedBy = marker;

    return true;
}

bool ActorImpl::unmark(upcxx::intrank_t marker)
{
    /*
    if (this->parentActorGraph->migrationphase)
    {
        this->markedBy = -1;
        return true;
    }
    */

    if (isPinned())
    {
        throw std::runtime_error("Unmark: Either marked after pinning or pinned after marking illegal state! (unmark)");
    }

    if (markedBy == marker || state == ActorState::Terminated || terminated)
    {
        markedBy = -1;
        return true;
    }

#ifdef REPORT_MAIN_ACTIONS
#ifndef NDEBUG
    std::cerr << "Tried to unmark " << getNameRef() << " by " << marker
              << " could not (might be terminated and thus unmarked), marked by: " << markedBy << std::endl;
#endif
#endif

    throw std::runtime_error("dont call unmark if you havent marked");
}

bool ActorImpl::pin(upcxx::intrank_t pinner)
{
    if (this->parentActorGraph->migrationphase)
    {
        return false;
    }

    if (isMarked())
    {
#ifdef REPORT_MAIN_ACTIONS
#ifndef NDEBUG
// std::cerr << "Tried to pin " << getNameRef() << " by " << pinner << " could not because is marked" << std::endl;
#endif
#endif
#ifdef REPORT_MAIN_ACTIONS
        std::cerr << pinner << "could not pin " << name << " marked" << std::endl;
#endif
        return false;
    }

    if (state != ActorState::Running || initial_act || terminated || this->parentActorGraph == nullptr)
    {
#ifdef REPORT_MAIN_ACTIONS
#ifndef NDEBUG
// std::cerr << "Tried to pin " << getNameRef() << " by " << pinner << " could not because is terminated" << std::endl;
#endif
#endif
#ifdef REPORT_MAIN_ACTIONS
        std::cerr << pinner << "could not pin " << name << " terminated" << std::endl;
#endif
        return false;
    }

    if (!isMarked())
    {
        pinnedBy += 1;
        pin_count[pinner] += 1;

#ifdef REPORT_MAIN_ACTIONS
        std::cerr << pinner << " pinned " << name << std::endl;
#endif

        return true;
    }

#ifdef REPORT_MAIN_ACTIONS
    std::cerr << pinner << "could not pin " << name << std::endl;
#endif
    return false;
}

bool ActorImpl::unpin(upcxx::intrank_t pinner)
{
    if (this->parentActorGraph->migrationphase)
    {
        return false;
    }

    //-1 unmarked, 0-(n-1) marked by a rank, n marked to be not marked again
    if (isMarked())
    {
        throw std::runtime_error("Unpin: Either marked after pinning or pinned after marking, illegal state! (unpin)");
    }

    if (!isPinned())
    {
        throw std::runtime_error("Don't call unpin if you have not pinned");
    }

    if (pinnedBy > 0 && pin_count[pinner] <= 0)
    {
        throw std::runtime_error("Again, don't call unpin if you have not pinned");
    }

    if (!(pin_count[pinner] > 0 && pinnedBy > 0))
    {
        throw std::runtime_error("Bad pin state");
    }

    pin_count[pinner] -= 1;
    pinnedBy -= 1;

#ifdef REPORT_MAIN_ACTIONS
    std::cerr << pinner << " unpinned " << name << std::endl;
#endif
    return true;
}

bool ActorImpl::isPinned() const { return pinnedBy > 0; }

bool ActorImpl::isPinnedByMe(int rank) const
{
    if (pinnedBy <= 0)
    {
        if (pin_count[rank] != 0)
        {
            throw std::runtime_error("pinnedBy pin_count mismatch");
        }

        return false;
    }

    return pin_count[rank] > 0;
}

bool ActorImpl::isMarked() const { return markedBy >= 0; }

void ActorImpl::force_unmark() {}

void ActorImpl::force_unpin() {}

int ActorImpl::isMarkedBy() const { return markedBy; }

// rank_n() stops unmarked, rank stops an actor marked by the caller
bool ActorImpl::stopWithCaller(ActorState as, upcxx::intrank_t caller)
{

    // bool was = false;
    // try to change an actor marked with caller
    // either the actor needs to be marke by the caller, or it shoul be the initial start call from the actor graph

    // we dont allow stopping if it is not marked!
    if (markedBy != caller)
    {
        throw std::runtime_error("Call stop only after markign!");
    }

    if (isPinned())
    {
        return false;
    }

    {
        // stop only if the actor is unmarked by me
        ActorState run = ActorState::Running;
        // ActorState tsfm = ActorState::NeedsActivation;

        if (this->state.compare_exchange_strong(run, ActorState::Transitioning))
        {
            this->parentActorGraph->activeActorCount -= 1;
            this->state = as;

#if defined(REPORT_MAIN_ACTIONS)
            //  std::cout << "Stopping actor:" << this->getName() << " on rank: " << upcxx::rank_me()
            //            << " by rank: " << caller << " " << this->parentActorGraph->activeActorCount << "/"
            //            << this->parentActorGraph->actorCount << std::endl;
#endif

            return true;
        }
    }

    // need to revert marking
    // the value we have is old so
    // this mean it is in wanting to terminate or terminated state
    // if (was)
    // {
    //     markedBy = -1;
    // }

    return false;
}

bool ActorImpl::stopFromAct()
{

    // if we stop from the actor then we can automatically unmark it
    // we should not see terminated since it is called only once
    // this will prevent any migration attempts
    // markedBy = -1;
    if (state != ActorState::Terminated)
    {
        unsigned int *tc = this->parentActorGraph->taskCount->local();
        if (*tc < this->canActNTimes())
        {
            // throw std::runtime_error("task count underflow during termination");
            *tc = 0;
        }
        else
        {
            *tc -= this->canActNTimes();
        }

        this->parentActorGraph->activeActorCount -= 1;
        this->state = ActorState::Terminated;
        this->parentActorGraph->has_a_terminated_actor = true;
        terminated = true;
        // if it is stopped then it should have the mark of the caller

#if defined(REPORT_MAIN_ACTIONS)
        std::cout << "Stopping actor:" << this->getName() << " on rank: " << upcxx::rank_me() << " "
                  << this->parentActorGraph->activeActorCount << "/" << this->parentActorGraph->actorCount << std::endl;
#endif
    }
    return true;
}

std::string ActorImpl::getName() const { return name; }

void ActorImpl::b_act()
{
    unsigned int *tc = this->parentActorGraph->taskCount->local();
    if (*tc == 0)
    {
        // throw std::runtime_error("TaskCount underflow " + std::to_string(*tc) + " " +
        //                          std::to_string(this->canActNTimes()));
        *tc = 0;
    }
    else
    {
        *tc -= 1;
    }

    // std::cout << upcxx::rank_me() << ": execute b_act for " << name << std::endl;
    if (this->state.load() != ActorState::Running)
    {
        throw std::runtime_error("act() should not be called is the state of the actor is not Running!");
    }

    if (!initial_act)
    {
        for (const auto &pr : inPorts)
        {
            if (pr.second->empty())
            {
                throw std::runtime_error("Act should not be called if the hinterlying ports are empty!");
                // return;
            }
        }
    }

    initial_act = false;

    workTokens += 1;
    tbegin = std::chrono::steady_clock::now();
#ifdef TRACE
    VT_begin(ActorImpl::event_act);
#endif

    act();

#ifdef TRACE
    VT_end(ActorImpl::event_act);
#endif

    tend = std::chrono::steady_clock::now();
    size_t elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(tend - tbegin).count();

    workTime += elapsed;

    this->portsInfoApplicable = false;

    actor_cost[index_for_actor_cost] = (elapsed >> 6);
    index_for_actor_cost += 1;
    if (index_for_actor_cost == history_array_length)
    {
        index_for_actor_cost = 0;
    }

    this->parentActorGraph->addToWork(elapsed);
    calls += 1;

    acted = true;
}

bool ActorImpl::needsReactivation() { return this->state == ActorState::NeedsActivation; }

bool ActorImpl::isRunningGlobal(GlobalActorRef ref)
{
    if (ref.where() == upcxx::rank_me())
    {
        ActorImpl *a = *(ref.local());
        return a->state == ActorState::Running;
    }
    else
    {
        bool isrunning = upcxx::rpc(
                             ref.where(),
                             [](GlobalActorRef ref)
                             {
                                 ActorImpl *a = *(ref.local());
                                 return a->state == ActorState::Running;
                             },
                             ref)
                             .wait();
        return isrunning;
    }
}

/*
    generates ports information from in and out ports, copies the portsinfo data
   directly if the portsInformation struct in an applicable state
*/
std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>, std::vector<std::unique_ptr<OutPortInformationBase>>>
ActorImpl::generatePortsInformation() const
{
    if (this->state != ActorState::TemporaryStoppedForMigration)
    {
        throw std::runtime_error("Portsinformation should be generated only for an actor that is stopped temporarily!");
    }

    std::vector<std::unique_ptr<InPortInformationBase>> inPortInformations;
    // int curindex = 0;
    for (auto &&pr : inPorts)
    {
        auto *ptr = pr.second;
        std::unique_ptr<InPortInformationBase> inPortInfo = ptr->generatePortInfo();
        inPortInformations.push_back(std::move(inPortInfo));
        // curindex += 1;
    }

    std::vector<std::unique_ptr<OutPortInformationBase>> outPortInformations;
    // curindex = 0;
    for (auto &&pr : outPorts)
    {
        auto *ptr = pr.second;
        std::unique_ptr<OutPortInformationBase> outPortInfo = ptr->generatePortInfo();
        outPortInformations.push_back(std::move(outPortInfo));
        // curindex += 1;
    }

    return std::make_pair(std::move(inPortInformations), std::move(outPortInformations));
}

/*
    refills ports with data, uses portsInformation member variable for that
*/
void ActorImpl::refillPorts()
{
    if (!portsInfoApplicable)
    {
        throw std::runtime_error("Refill ports called but ports information is not in a legal state!");
    }

    // std::cout << "refill ports for " << getNameRef() << std::endl;

    auto inarr = std::move(std::get<0>(portsInformation));
    for (auto &&el : inarr)
    {
        std::string name = el->getName();

        auto it = inPorts.find(name);
        if (it == inPorts.end())
        {
            throw std::runtime_error("Copied inport is not a member inport of the actor!");
        }

        AbstractInPort *ip = it->second;

        ip->applyPortInfo(std::move(el));
    }

    auto outarr = std::move(std::get<1>(portsInformation));
    for (auto &&el : outarr)
    {
        std::string name = el->getName();

        auto it = outPorts.find(name);

        if (it == outPorts.end())
        {
            throw std::runtime_error("Copied outport is not a member outport of the actor!");
        }

        AbstractOutPort *op = it->second;
        op->applyPortInfo(std::move(el));
    }

    portsInfoApplicable = false;

    // apply port info uses move operations. also we need to change to state
    // after
    clearPortInformation();
    // std::cout << "refill ports for " << this->name << std::endl;
}

/*
    moves port information from actor a to this actor
*/
void ActorImpl::movePortsInformation(ActorImpl &&a) const
{
    if (a.portsInfoBadState())
    {
        auto tup = a.generatePortsInformation();
        assignPortsInformation(std::move(tup));
    }
    else
    {
        assignPortsInformation(std::move(a.portsInformation));
    }

    a.portsInfoApplicable = false;
    std::get<0>(a.portsInformation).clear();
    std::get<1>(a.portsInformation).clear();
}

/*
    generates port information and assigns to portsInformation member variable,
    to be called before serializing actors
*/
void ActorImpl::generateAndAssignPortsInformation() const
{
    std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
               std::vector<std::unique_ptr<OutPortInformationBase>>>
        tup = this->generatePortsInformation();
    this->assignPortsInformation(std::move(tup));

    // std::cout << "generated ports information for " << getNameRef() << std::endl;
}

/*
    assigns ports information, before assigning frees the data pointer by the
   old struct
*/
void ActorImpl::assignPortsInformation(std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                                  std::vector<std::unique_ptr<OutPortInformationBase>>> &&tup) const
{
    this->portsInformation = std::move(tup);
    this->portsInfoApplicable = true;
}

std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>, std::vector<std::unique_ptr<OutPortInformationBase>>>
ActorImpl::moveTuple() const
{

    if (!this->portsInfoApplicable)
    {
        throw std::runtime_error("moveports when ports information is applicable!");
    }

    this->portsInfoApplicable = false;
    std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
               std::vector<std::unique_ptr<OutPortInformationBase>>>
        tmp(std::move(portsInformation));
    return tmp;
}

// checks if the portsInformation is in a moved state, if so you probably need
// to regenerate it also returns true if the other actors portInformation is not
// applicable, meaning the ports hav echanged their values
bool ActorImpl::portsInfoBadState() const { return !portsInfoApplicable; }

void ActorImpl::prepare() const
{
    /*
    if(prepared)
    {
        this->portsInfoApplicable = false;
        std::get<0>(this->portsInformation).clear();
        std::get<1>(this->portsInformation).clear();
    }

    this->generateAndAssignPortsInformation();
    prepared = true;

#ifndef NDEBUG
    checkMessageCountsSameAsSaved();
#endif
    */
}

void ActorImpl::prepareDerived()
{
    /*
    std::cout << "You might want to implement this function!" << std::endl;
    */
}

std::tuple<uint64_t, uint64_t> ActorImpl::getWork() const { return {this->workTokens, this->workTime}; }

bool ActorImpl::getIsRunning() const { return this->state.load() == ActorState::Running; }

ActorState ActorImpl::getRunningState() const { return this->state.load(); }

const std::string &ActorImpl::getNameRef() const { return name; }

void ActorImpl::broadcastTaskDeque(TaskDeque *tdq)
{
    for (const auto &ip : inPorts)
    {
        ip.second->setTaskDequeOfChannel(tdq);
    }
}

bool ActorImpl::messageOnWire() const
{
    bool ret = false;
    for (const auto &pr : outPorts)
    {
        auto *out = pr.second;
        ret |= out->messageOnWire();
    }
    return ret;
}

bool ActorImpl::messageOnWire(const std::string &outportName) const
{
    const auto &pr = *outPorts.find(outportName);
    return pr.second->messageOnWire();
}

void ActorImpl::severeConnectionIn(const std::string &portName)
{
    auto it = inPorts.find(portName);
    bool rt = it->second->severeConnection();
    // std::cout << upcxx::rank_me() << " severeConnectionIn " << name << " " << portName << std::endl;
    if (!rt)
    {
        throw std::runtime_error("Severe connection (In) should not fail");
    }
}

void ActorImpl::severeConnectionOut(const std::string &portName)
{
    auto qit = outPorts.find(portName);
    bool rt = qit->second->severeConnection();
    if (!rt)
    {
        throw std::runtime_error("Severe connection (Out) should not fail");
    }
}

void ActorImpl::resurrectConnection(const std::string &inPortName, upcxx::intrank_t rank, AbstractOutPort *outptr)
{
    auto it = inPorts.find(inPortName);
    bool rt = it->second->resurrectConnection(rank, outptr);
    // if two neighbors are migrated at the same time then it could be the case that we will have attempts to resurrect
    // twice, thus resulting with the value false being returned
    if (!rt)
    {
        throw std::runtime_error("Resurrect connection should not fail");
    }
}
void ActorImpl::resurrectConnection(const std::string &outPortName, GlobalChannelRef channelptr)
{
    auto it = outPorts.find(outPortName);
    bool rt = it->second->resurrectConnection(channelptr);
    // if two neighbors are migrated at the same time then it could be the case that we will have attempts to resurrect
    // twice, thus resulting with the value false being returned
    if (!rt)
    {
        throw std::runtime_error("Resurrect connection should not fail");
    }
}

AbstractInPort *ActorImpl::getInPort(const std::string &name)
{
    auto it = inPorts.find(name);
    return it->second;
}
AbstractOutPort *ActorImpl::getOutPort(const std::string &name)
{
    auto it = outPorts.find(name);
    return it->second;
}

void ActorImpl::sendNotifications()
{
    if (!noNeedToNotify())
    {
        for (const auto &it : inPorts)
        {
            AbstractInPort *in = it.second;
            in->notify();
        }
    }
}

void ActorImpl::sendNotification(const std::string &portName)
{
    const auto &it = *inPorts.find(portName);

    AbstractInPort *in = it.second;
    in->notify();
}

bool ActorImpl::noNeedToNotify() const
{
    bool empty = true;
    for (const auto &it : inPorts)
    {
        AbstractInPort *in = it.second;
        empty &= in->noNeedToNotify();
    }
    return empty;
}

bool ActorImpl::noNeedToNotify(const std::string &portName) const
{
    auto &it = *inPorts.find(portName);

    AbstractInPort *in = it.second;
    bool empty = in->noNeedToNotify();

    return empty;
}

bool ActorImpl::buffersEmpty() const
{
    bool empty = true;
    for (const auto &it : outPorts)
    {
        AbstractOutPort *out = it.second;
        empty &= out->bufferSize() == 0;
    }
    return empty;
}

bool ActorImpl::bufferEmpty(const std::string &portName) const
{
    bool empty = true;
    auto *out = outPorts.find(portName)->second;
    empty &= out->bufferSize() == 0;

    return empty;
}

void ActorImpl::flushBuffer(const std::string &portName)
{
    auto *out = outPorts.find(portName)->second;
    out->writeBuffer();
}

void ActorImpl::flushBuffers() const
{
    for (const auto &it : outPorts)
    {
        AbstractOutPort *out = it.second;
        out->writeBuffer();
    }
}

bool ActorImpl::markableForMigration() const { return pinnedBy == 0 && markedBy == -1; }

bool ActorImpl::markedForMigration() const { return markedBy >= 0; }

bool ActorImpl::centralStealable() const { return state != ActorState::Terminated && markedBy != upcxx::rank_n(); }

void ActorImpl::set_port_size()
{
    // std::cout << getNameRef() << " called to set_port_size" << std::endl;
    for (auto &ip : inPorts)
    {
        if (ip.second->total == 0)
        {
            ip.second->total = inPorts.size();
        }
    }
}

bool ActorImpl::connected()
{
    //
    if (inPorts.empty() && outPorts.empty())
    {
        return true;
    }

    bool a = true;
    for (auto &ip : inPorts)
    {
        a &= ip.second->connected();
    }
    for (auto &op : outPorts)
    {
        a &= op.second->connected();
    }
    return a;
}

uint64_t ActorImpl::calc_cost()
{
    // ind points to the current index to overwrite so we will begin
    // with i-1 th index (if ind is 0 then start at 32)
    // calculate the number a/i and create to cost of the actor

    if (index_for_actor_cost == 0 && actor_cost[history_array_length - 1] == 0)
    {
        return 1;
    }

    size_t beg = index_for_actor_cost == 0 ? (history_array_length - 1) : index_for_actor_cost - 1;
    size_t i = beg;
    uint64_t sum = 0;
    unsigned int diff = 1;

    int iter = 0;
    while (i != index_for_actor_cost)
    {
        uint64_t cost = 0;

        if (actor_cost[i] != 0)
        {
            cost = static_cast<uint64_t>((static_cast<float>(actor_cost[i])) /
                                         (static_cast<float>(util::intPow(2, diff))));
            sum += cost;
        }
        else
        {
            break;
        }

        if (i == 0)
        {
            i = history_array_length - 1;
            static_assert(history_array_length > 1);
        }
        else
        {
            i -= 1;
        }

        diff += 1;

        ++iter;
        if (iter > util::too_long)
        {
            throw std::runtime_error("Loop too long!");
        }
    }

    return sum;
}

size_t ActorImpl::getMark() const { return markedBy; }

void ActorImpl::setRunning() { this->state = ActorState::Running; }

void ActorImpl::setRunningState(ActorState as)
{
    this->state = as;
    if (as == ActorState::Terminated)
    {
        this->parentActorGraph->activeActorCount -= 1;
        this->state = ActorState::Terminated;
        this->parentActorGraph->has_a_terminated_actor = true;
    }
}

bool ActorImpl::isTerminated() { return this->state == ActorState::Terminated; }

bool ActorImpl::resetState()
{
    if (!isMarked() && !isPinned())
    {
        this->state = ActorState::NeedsActivation;
        return true;
    }

    return false;
}

void ActorImpl::setParentActorGraph(ActorGraph *ag)
{
    if (this->parentActorGraph == nullptr)
    {
        this->parentActorGraph = ag;
    }
}

ActorTriggers ActorImpl::createTriggers()
{
    std::vector<std::string> v;

    if (inPorts.empty())
    {
        throw std::runtime_error("There should be no actors without ports");
    }

    for (auto &pr : inPorts)
    {
        v.emplace_back(pr.second->getName());
    }

    ActorTriggers aa(this->getNameRef(), std::move(v));

    return aa;
}

bool ActorImpl::actable() const
{
    if (state != ActorState::Running || terminated)
    {
        // std::cout << name << " marked" << std::endl;
        return false;
    }

    if (!initial_act)
    {
        for (const auto &pr : inPorts)
        {
            if (pr.second->empty())
            {
                // std::cout << name << " cant act buffer empty" << std::endl;
                return false;
            }
        }
    }

    return true;
}

size_t ActorImpl::canActNTimes() const
{
    if (state == ActorState::Terminated || terminated)
    {
        return 0;
    }

    size_t i = 0;
    if (initial_act)
    {
        i = 1;
    }

    size_t minavail = std::numeric_limits<size_t>::max();

    if (inPorts.empty())
    {
        throw std::runtime_error("inPorts should not be empty when canActNTimes is called");
    }

    for (const auto &pr : inPorts)
    {
        if (pr.second->available() < minavail)
        {
            minavail = pr.second->available();
        }
    }

    return i + minavail;
}

std::vector<std::pair<std::string, size_t>> ActorImpl::getMessageCounts() const
{
    std::vector<std::pair<std::string, size_t>> buf;
    buf.reserve(inPorts.size());
    for (const auto &pr : inPorts)
    {
        size_t msgs = pr.second->available();
        buf.emplace_back(pr.first, msgs);
    }

    return buf;
}

bool ActorImpl::checkMessageCountsSameAsSaved() const
{
    if (!prepared)
    {
        return false;
    }

    std::vector<std::pair<std::string, size_t>> msgs_inports = getMessageCounts();

    for (const auto &el : std::get<0>(portsInformation))
    {
        auto msg1 = el->getMessageCount();

        for (const auto &pr : msgs_inports)
        {
            if (pr.first == el->getName())
            {
                auto msg2 = pr.second;

                if (msg2 != msg1)
                {
                    std::cerr << upcxx::rank_me()
                              << ": Message counts of inport information and ports are not the same: (" + pr.first +
                                     ", " + std::to_string(pr.second) + ") and (" + el->getName() + ", " +
                                     std::to_string(msg1) + ")"
                              << std::endl;
                    return false;
                }
            }
        }
    }

    return true;
}

bool ActorImpl::is_prepared() { return prepared; }

bool ActorImpl::hasNoMessages() const
{
    for (const auto &pr : inPorts)
    {
        AbstractInPort *aip = pr.second;
        if (!aip->empty())
        {
            return false;
        }
    }

    return true;
}

bool ActorImpl::checkTerminated() const { return false; }

size_t canActNTimesL(std::vector<std::pair<std::string, size_t>> &msgs)
{

    size_t minavail = std::numeric_limits<size_t>::max();

    for (const auto &pr : msgs)
    {
        if (pr.second < minavail)
        {
            minavail = pr.second;
        }
    }

    return minavail;
}

void ActorImpl::addTask(const std::string &increased_port_name)
{
    if (state == ActorState::Running)
    {

        std::vector<std::pair<std::string, size_t>> msgs_inports = getMessageCounts();
        std::vector<std::pair<std::string, size_t>> msgs_inports_no_msg = getMessageCounts();

        for (auto &el : msgs_inports_no_msg)
        {
            if (el.first == increased_port_name)
            {
                if (el.second == 0)
                {
                    throw std::runtime_error("should not be 0");
                }
                el.second -= 1;
            }
        }

        auto a = canActNTimesL(msgs_inports);
        auto b = canActNTimesL(msgs_inports_no_msg);

        if (a != b && a != b + 1)
        {
            throw std::runtime_error("This should not happen");
        }

        /*
        for (auto &pr : msgs_inports)
        {
            if (pr.first == increased_port_name)
            {
                if(pr.second == 0){
                    throw std::runtime_error("should not be 0");
                }
                if (pr.second - 1 < max_exec_if_no_msg)
                {
                    max_exec_if_no_msg = pr.second - 1;
                }
                if (pr.second < max_exec_with_msg)
                {
                    max_exec_with_msg = pr.second;
                }
            }
            else
            {
                if (pr.second < max_exec_with_msg)
                {
                    max_exec_with_msg = pr.second;
                }
                if (pr.second < max_exec_if_no_msg)
                {
                    max_exec_if_no_msg = pr.second;
                }
            }
        }
        */

        /*
        std::cout << "{";
        for (auto &pr : msgs_inports)
        {
            if (pr.first == increased_port_name)
            {
                std::cout << "(" << pr.second << "), ";
            }
            else
            {
                std::cout << pr.second << ", ";
            }
        }
        std::cout << ": " << (a != b) << "}" << std::endl;
        */

        if (a != b)
        {
            this->parentActorGraph->addTask();
        }
    }
}