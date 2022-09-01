#pragma once
#include "ActorState.hpp"
#include "Definitions.hpp"
#include <iostream>
#include <set>
#include <string>
#include <upcxx/upcxx.hpp>
#include <utility>

/*
    Actor and ActorImpl are abstract classes so they are serialized to ActorData class
    which describes the state of an actor, which is used to initialize an ACtor on the
    new rank
*/

class ActorData
{
  public:
    std::string name;    // name
    uint64_t workTokens; // counted tokens till now
    uint64_t workTime;   // time spent in act() till now
    upcxx::intrank_t mark;
    ActorState state;
    std::array<double, history_array_length> actor_cost;
    size_t index;
    bool initial_act;
    bool term;
    uint64_t acted;

    ActorData() = delete;

    ActorData(std::string &&name, uint64_t workTokens, uint64_t workTime, upcxx::intrank_t &&mark, ActorState state,
              std::array<double, history_array_length> &&historyArray, size_t index, bool initial_act, bool term,
              uint64_t acted)
        : name(std::move(name)), workTokens(workTokens), workTime(workTime), mark(std::move(mark)), state(state),
          actor_cost(std::move(historyArray)), index(index), initial_act(initial_act), term(term), acted(acted)
    {
        if (state != ActorState::TemporaryStoppedForMigration && state != ActorState::Terminated)
        {
            throw std::runtime_error("Constructing an Actor with an illegal state for init");
        }
    }

    ActorData(const std::string &name, uint64_t workTokens, uint64_t workTime, const upcxx::intrank_t &mark,
              ActorState state, const std::array<double, history_array_length> &historyArray, size_t index,
              bool initial_act, bool term, uint64_t acted)
        : name(name), workTokens(workTokens), workTime(workTime), mark(mark), state(state), actor_cost(historyArray),
          index(index), initial_act(initial_act), term(term), acted(acted)
    {
        if (state != ActorState::TemporaryStoppedForMigration && state != ActorState::Terminated)
        {
            throw std::runtime_error("Constructing an Actor with an illegal state for init");
        }
    }

    ActorData(ActorData &&ad)
        : name(std::move(ad.name)), workTokens(ad.workTokens), workTime(ad.workTime), mark(ad.mark), state(ad.state),
          actor_cost(std::move(ad.actor_cost)), index(std::move(ad.index)), initial_act(ad.initial_act), term(ad.term),
          acted(ad.acted)
    {
        if (state != ActorState::TemporaryStoppedForMigration && state != ActorState::Terminated)
        {
            throw std::runtime_error("Constructing an Actor with an illegal state for init");
        }
    }

    ActorData(const ActorData &ad)
        : name(ad.name), workTokens(ad.workTokens), workTime(ad.workTime), mark(ad.mark), state(ad.state),
          actor_cost(ad.actor_cost), index(ad.index), initial_act(ad.initial_act), term(ad.term), acted(ad.acted)
    {
        if (state != ActorState::TemporaryStoppedForMigration && state != ActorState::Terminated)
        {
            throw std::runtime_error("Constructing an Actor with an illegal state for init");
        }
    }

    ~ActorData(){};

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, ActorData const &object)
        {
            writer.write(object.name);
            writer.write(object.workTokens);
            writer.write(object.workTime);
            writer.write(object.mark);
            writer.write(object.state);
            writer.write(object.actor_cost);
            writer.write(object.index);
            writer.write(object.initial_act);
            writer.write(object.term);
            writer.write(object.acted);
            if (object.state != ActorState::TemporaryStoppedForMigration && object.state != ActorState::Terminated)
            {
                throw std::runtime_error("Serializing an Actor with an illegal state for serializazion");
            }
            return;
        }

        template <typename Reader> static ActorData *deserialize(Reader &reader, void *storage)
        {
            std::string name = reader.template read<std::string>();
            uint64_t workTokens = reader.template read<uint64_t>();
            uint64_t workTime = reader.template read<uint64_t>();
            auto rc = reader.template read<upcxx::intrank_t>();
            ActorState as = reader.template read<ActorState>();
            std::array<unsigned int, history_array_length> acost =
                reader.template read<std::array<double, history_array_length>>();
            size_t index = reader.template read<size_t>();
            bool initial_act = reader.template read<bool>();
            uint64_t acted = reader.template read<uint64_t>();
            if (as != ActorState::TemporaryStoppedForMigration && as != ActorState::Terminated)
            {
                throw std::runtime_error("De-serializing an Actor with an illegal state for serializazion");
            }
            bool term = reader.template read<bool>();

            ActorData *v = ::new (storage) ActorData(std::move(name), workTokens, workTime, std::move(rc), as,
                                                     std::move(acost), index, initial_act, term, acted);

            return v;
        }
    };
};