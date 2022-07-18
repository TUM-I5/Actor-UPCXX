#pragma once

#include "ActorData.hpp"
#include "ActorImpl.hpp"
#include "ActorState.hpp"
#include "Utility.hpp"
#include <memory>
#include <new>
#include <string>
#include <tuple>
#include <vector>

/*
  Class to be used instead of ActorImpl
  to be used as an actor fassade for the user
*/

class AbstractInPort;
class AbstractOutPort;

class Actor : private ActorImpl
{
    friend class AbstractInPort;
    friend class AbstractOutPort;

  public:
    ActorImpl &asBase();                  // cast to ActorImpl, needed bcs private
    const ActorImpl &asConstBase() const; // casts to a const ref of base class, needed sometimes

    std::string getName() const;
    const std::string &getNameRef() const;

    // creates an in port
    template <typename T, int capacity> InPort<T, capacity> *createInPort(const std::string &name)
    {
        return ActorImpl::makeInPort<T, capacity>(name);
    }
    // creates an out port
    template <typename T, int capacity> OutPort<T, capacity> *createOutPort(const std::string &name)
    {
        return ActorImpl::makeOutPort<T, capacity>(name);
    }

    void copyPortsInformation(const Actor &other); // copy ports information of another actor, needed for migration

    void movePortsInformation(Actor &&other); // move ports information of another actor, needed for migration

    ActorState getRunningState();

  protected:
    void b_act();   // calls the ActorImpl act(t) to track the time
    void prepare(); // prepare actor for migration
    void stop();    // stops actor, will not run in actorGraph::run()

  private:
    virtual void act() override = 0; // act() to be implemented by the users
    virtual bool checkTerminated() const override;

  public:
    virtual ~Actor(){};
    Actor(std::string &&name);
    Actor(std::string const &name);
    Actor(Actor &&other);
    Actor(Actor const &other) = delete;

    // convenience (?) constructor where portsInformation is also serialzied during the serialization
    Actor(ActorData &&data, std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                       std::vector<std::unique_ptr<OutPortInformationBase>>> &&portsInformation);
    Actor(ActorData &&data);
    Actor &operator=(Actor &other) = delete;

    // since there is a private inheritance the constructors that need to be used are redeclared here
    using ActorImpl::act;
    using ActorImpl::checkTerminated;
    using ActorImpl::getName;
    using ActorImpl::getNameRef;
    using ActorImpl::getRunningState;
    using ActorImpl::makeInPort;
    using ActorImpl::makeOutPort;
    using ActorImpl::movePortsInformation;
    using ActorImpl::prepare;
    using ActorImpl::stop;
    /*
      If this class is serialized then do not cast to ActorImpl
    */
    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, Actor const &object)
        {
            const ActorImpl &ai = object.asConstBase();
            // std::cout << "Serializing Actor: " << ai.getName() << " the state is: " <<
            // Actor::asp.as2str(ai.getRunningState()) << std::endl;
            writer.write(ai);
        }

        template <typename Reader> static ActorData *deserialize(Reader &reader, void *storage)
        {
            ActorData ad = reader.template read<ActorImpl>();
            // std::cout << "Deserializing Actor: " << ad.name << " the state is: " << Actor::asp.as2str(ad.state) <<
            // std::endl;
            return ::new (storage) ActorData(std::move(ad));
        }
    };
};
