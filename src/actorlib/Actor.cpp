#include "Actor.hpp"

ActorImpl &Actor::asBase() { return static_cast<ActorImpl &>(*this); }

const ActorImpl &Actor::asConstBase() const { return static_cast<const ActorImpl &>(*this); }

void Actor::b_act() { ActorImpl::b_act(); }

// bool Actor::inActableState() const { return ActorImpl::inActableState(); }

void Actor::prepare() { ActorImpl::prepare(); }

std::string Actor::getName() const { return ActorImpl::getName(); }

const std::string &Actor::getNameRef() const { return ActorImpl::getNameRef(); }

/*
void Actor::copyPortsInformation(const Actor &other)
{
    const ActorImpl *tmpref = &other.asConstBase();
    ActorImpl::copyPortsInformation(*tmpref);
}
*/

void Actor::movePortsInformation(Actor &&other) { ActorImpl::movePortsInformation(std::move(other.asBase())); }

void Actor::stop() { ActorImpl::stop(); }

Actor::Actor(std::string const &name) : ActorImpl(name) {}

Actor::Actor(std::string &&name) : ActorImpl(std::move(name)) {}

Actor::Actor(Actor &&other) : ActorImpl(std::move(other.asBase())) {}

Actor::Actor(ActorData &&data, std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                          std::vector<std::unique_ptr<OutPortInformationBase>>> &&portsInformation)
    : ActorImpl(std::move(data), std::move(portsInformation))
{
}

Actor::Actor(ActorData &&data) : ActorImpl(std::move(data)) {}

ActorState Actor::getRunningState() { return ActorImpl::getRunningState(); }

bool Actor::checkTerminated() const { return false; }