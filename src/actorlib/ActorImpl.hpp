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
#include "ActorData.hpp"
#include "ActorTriggers.hpp"
#include "Definitions.hpp"
#include "PortInformationBase.hpp"
#include "Utility.hpp"
#include "config.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <upcxx/upcxx.hpp>
#include <utility>
#include <vector>

#ifdef TRACE
#include "VT.h"
#endif

#include "ActorState.hpp"

class ActorGraph;
class AbstractInPort;
class AbstractOutPort;
template <typename T, int capacity> class InPort;
template <typename T, int capacity> class OutPort;
class TaskDeque;

class ActorImpl
{
    friend class ActorGraph;
    friend class AbstractInPort;
    friend class AbstractOutPort;

  private:
    // the name to help differ initial and following run(x,y) calls
    // does the same thing as start, checks for parallization as extra -> has
    // start actor
    // stop actor
    std::string name; // name

  private:
#ifdef TRACE
    static int event_act;
#endif

  public:
    ActorGraph *parentActorGraph; // Ag that this actor is connected
    static double assumed_cost;

  protected:
    std::unordered_map<std::string, AbstractInPort *> inPorts;   // map of all inports
    std::unordered_map<std::string, AbstractOutPort *> outPorts; // map of all outports
    mutable std::atomic<ActorState> state;
    // required for reinitialization/reconnection of the actor on act rank
    mutable std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                       std::vector<std::unique_ptr<OutPortInformationBase>>>
        portsInformation;             // a struct to save port informations in heap
    mutable bool portsInfoApplicable; // save if portsInformation can be used to refill
    uint64_t workTokens;              // amount of tokens actor has gathered
    uint64_t workTime;                // amount of the time this actor spent in its act
    template <typename T, int capacity> InPort<T, capacity> *makeInPort(std::string const &name);   // makes an InPort
    template <typename T, int capacity> OutPort<T, capacity> *makeOutPort(std::string const &name); // maes an OutPort

  public:
    template <typename T, int capacity> std::vector<std::pair<std::string, std::vector<T>>> extract_messages();

  private:
    upcxx::intrank_t markedBy = -1;
    size_t pinnedBy = 0;
    std::vector<unsigned short> pin_count;
    std::array<double, history_array_length> actor_cost{0.0};
    size_t index_for_actor_cost = 0;
    mutable bool prepared = false;
    // bool force_unmarked = false;
    // bool force_unpinned = false;
    size_t calls = 0;
    bool initial_act = true;
    bool terminated = false;

  public:
    uint64_t acted = 0;
    ActorImpl(const std::string &name);
    ActorImpl(std::string &&name);
    ActorImpl();
    virtual ~ActorImpl();
    ActorImpl(const ActorImpl &act) = delete;
    ActorImpl(ActorImpl &&act);
    ActorImpl(ActorData &&data);
    ActorImpl(ActorData &&data, std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                           std::vector<std::unique_ptr<OutPortInformationBase>>> &&portsInformation);
    ActorImpl &operator=(const ActorImpl &act) = delete;
    ActorImpl &operator=(ActorImpl &&act) = delete;
    ActorImpl &operator=(ActorImpl &act) = delete;
    std::string toString();
    AbstractOutPort *getOutPort(std::string &name);
    AbstractInPort *getInPort(std::string &name);
    template <typename T, int capacity>
    std::tuple<InPort<T, capacity> *, InPort<T, capacity> *, OutPort<T, capacity> *>
    generateMultiplexerPorts(std::string &portName);
    template <typename T, int capacity>
    std::tuple<OutPort<T, capacity> *, OutPort<T, capacity> *, InPort<T, capacity> *>
    generateDemultiplexerPorts(std::string &portName);
    virtual void act() = 0;
    virtual bool checkTerminated() const;
    void b_act();                                // runs act() but tracks time
    void rmInPort(const std::string &portName);  // rm inport
    void rmOutPort(const std::string &portName); // rm outport
    std::pair<std::vector<std::pair<std::string, AbstractInPort *>>,
              std::vector<std::pair<std::string, AbstractOutPort *>>>
    getPorts() const;                                                            // get pointers and names of all ports
    std::vector<std::pair<std::string, AbstractInPort *>> getAllInPorts() const; // get only the in ports
    std::vector<std::pair<std::string, AbstractOutPort *>> getAllOutPorts() const; // get only or the out ports
    std::vector<std::string> getAllInPortNames() const;                            // get only the inport names
    std::vector<std::string> getAllOutPortNames() const;                           // get only the outport name
    std::string getName() const;
    std::string getReplicationName() const;
    std::tuple<uint64_t, uint64_t> getWork() const;  // sets both work tracking variables to 0
    bool needsReactivation();                        // checs if a reactivation is necessary
    static bool isRunningGlobal(GlobalActorRef ref); // static method to check if an actor is running
    // refills ports with data according to the information available in ports
    // information
    void refillPorts(); // changes state of ports to the state that is saved in the portsInformation struct
    // generates 2 arrays of ports informations, should be assigned to
    // portsInformation as soon as possible
    std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
               std::vector<std::unique_ptr<OutPortInformationBase>>>
    generatePortsInformation() const; // generates the portsInformation struct from the ports
    // uses info in member
    // moves from actor a and assigns ports information ans assigns to member
    // variable
    void movePortsInformation(ActorImpl &&a) const; // moves the portsinfo of anact actor
    // copies from actor a/ generates ports informatiion from ports of actor a,
    // and assigns to member var
    // void copyPortsInformation(const ActorImpl &a); // copies the port info of anact actor
    void generateAndAssignPortsInformation() const; // generates portinfo and assigns to member portsInformation
    bool portsInfoBadState() const;                 // checks if the ports information in a legal state
    void assignPortsInformation(std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                           std::vector<std::unique_ptr<OutPortInformationBase>>> &&tup)
        const; // assignes the new portsinfo frees the old

    /*
    std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
               std::vector<std::unique_ptr<OutPortInformationBase>>>
    copyPortsInformation() const;  // copies ports info and returns the portsINormation
    */

    void prepare() const;          // prepare for migration
    virtual void prepareDerived(); // overriden if extra preparation is needed for migration
    std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
               std::vector<std::unique_ptr<OutPortInformationBase>>>
    moveTuple() const;         // get portsInformation
    bool getIsRunning() const; // return true if isRunning is true
    ActorState getRunningState() const;
    virtual std::string getTimeInfo() { return ""; } // get time Information
    bool start();
    const std::string &getNameRef() const;
    bool tryLock(ActorState expected, ActorState desired);
    bool mark(upcxx::intrank_t marker);
    bool unmark(upcxx::intrank_t marker);
    bool pin(upcxx::intrank_t pinner);
    bool unpin(upcxx::intrank_t pinner);
    bool isPinned() const;
    bool isMarked() const;
    bool isPinnedByMe(int rank) const;
    void force_unmark();
    void force_unpin();
    int isMarkedBy() const;
    bool stopWithCaller(
        ActorState as,
        upcxx::intrank_t caller); // make stop private because, the graph must ensure some stuff for migration
    bool startWithCaller(upcxx::intrank_t caller);
    void broadcastTaskDeque(TaskDeque *tdq);
    bool messageOnWire() const;
    bool messageOnWire(const std::string &outportName) const;
    void severeConnectionIn(const std::string &portName);
    void severeConnectionOut(const std::string &portName);
    void resurrectConnection(const std::string &inPortName, upcxx::intrank_t rank, AbstractOutPort *outptr);
    void resurrectConnection(const std::string &outPortName, GlobalChannelRef channelptr);
    AbstractInPort *getInPort(const std::string &name);
    AbstractOutPort *getOutPort(const std::string &name);
    void flushBuffers() const;
    void flushBuffer(const std::string &portName);
    bool buffersEmpty() const;
    bool bufferEmpty(const std::string &portName) const;
    void sendNotifications();
    void sendNotification(const std::string &portName);
    bool noNeedToNotify() const;
    bool noNeedToNotify(const std::string &portName) const;
    bool markableForMigration() const;
    bool markedForMigration() const;
    bool centralStealable() const;
    void set_port_size();
    bool connected();
    size_t getMark() const;
    uint64_t calc_cost();
    void setRunning();
    void setRunningState(ActorState as);
    bool isTerminated();
    bool resetState();
    void setParentActorGraph(ActorGraph *ag);
    ActorTriggers createTriggers();
    void clearPortInformation();
    void update_ports(std::vector<AbstractInPort *> ins, std::vector<AbstractOutPort *> outs);
    bool actable() const;
    std::vector<std::pair<std::string, size_t>> getMessageCounts() const;
    bool checkMessageCountsSameAsSaved() const;
    bool is_prepared();
    double getCost() const;
    void addCost(double additional_cost);
    bool hasNoMessages() const;
    bool stopFromAct();
    size_t canActNTimes() const;
    void addTask(const std::string &increased_port_name);
    uint64_t getPastExecutionCount() const;

  protected:
    bool stop();

  private:
    void initPinCount();

  public:
    /*
      Serialized the abstract base class to an ActorData object that describes state
    */
    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, ActorImpl const &object)
        {

            if (object.state != ActorState::TemporaryStoppedForMigration && object.state != ActorState::Terminated)
            {
                throw std::runtime_error("Illegal state during serialization of an actors");
            }

            if (!object.isMarked() || object.isPinned())
            {
                throw std::runtime_error("Actor should be marked but unpinned during serialization");
            }

            object.flushBuffers();

            if (!object.buffersEmpty())
            {
                throw std::runtime_error("Buffers should be empty at this point!");
            }

            object.generateAndAssignPortsInformation();
            object.prepared = true;

            if (!object.hasNoMessages())
            {
                throw std::runtime_error("No messages should remain after preparing!");
            }

            writer.write(object.name);
            writer.write(object.workTokens);
            writer.write(object.workTime);
            writer.write(object.markedBy);
            ActorState as = object.state.load();
            writer.write(as);
            writer.write(object.actor_cost);
            writer.write(object.index_for_actor_cost);
            writer.write(object.initial_act);
            writer.write(object.terminated);
            writer.write(object.acted);
#ifdef REPORT_MAIN_ACTIONS
            // std::cout << "Serializing ActorImpl: " << object.getName() << " the state is: " <<
            // ActorImpl::asp.as2str(object.getRunningState()) << std::endl;
#endif
            return;
        }

        template <typename Reader> static ActorData *deserialize(Reader &reader, void *storage)
        {
            std::string name = reader.template read<std::string>();
            uint64_t workTokens = reader.template read<uint64_t>();
            uint64_t workTime = reader.template read<uint64_t>();
            auto rc = reader.template read<upcxx::intrank_t>();
            ActorState as = reader.template read<ActorState>();
            std::array<double, history_array_length> acost =
                reader.template read<std::array<double, history_array_length>>();
            size_t index = reader.template read<size_t>();
            bool initial_act = reader.template read<bool>();
            bool term = reader.template read<bool>();
            uint64_t acted = reader.template read<uint64_t>();

            ActorData *v = ::new (storage) ActorData(std::move(name), workTokens, workTime, std::move(rc), as,
                                                     std::move(acost), index, initial_act, term, acted);
#ifdef REPORT_MAIN_ACTIONS
            // std::cout << "Deserializing ActorImpl: " << v->name << " the state is: " <<
            // ActorImpl::asp.as2str(v->state) << std::endl;
#endif
            return v;
        }
    };
};

template <typename type, int capacity> InPort<type, capacity> *ActorImpl::makeInPort(const std::string &name)
{
    InPort<type, capacity> *ip = ::new InPort<type, capacity>(name, this->name);
    ip->connectedActor = this;
    inPorts[name] = ip;
    return ip;
}

template <typename type, int capacity> OutPort<type, capacity> *ActorImpl::makeOutPort(const std::string &name)
{
    OutPort<type, capacity> *op = ::new OutPort<type, capacity>(name);
    op->connectedActor = this;
    outPorts[name] = op;
    return op;
}

#include "AbstractInPort.hpp"

template <typename T, int capacity> std::vector<std::pair<std::string, std::vector<T>>> ActorImpl::extract_messages()
{
    std::vector<std::pair<std::string, std::vector<T>>> v;

    if (state != ActorState::Terminated)
    {
        for (auto &inppr : inPorts)
        {
            AbstractInPort *ainp = inppr.second;
            InPort<T, capacity> *inp = dynamic_cast<InPort<T, capacity> *>(ainp);
            auto msgs = inp->extract_messages();
            v.push_back(std::make_pair(ainp->getName(), std::move(msgs)));
        }
    }

    return v;
}