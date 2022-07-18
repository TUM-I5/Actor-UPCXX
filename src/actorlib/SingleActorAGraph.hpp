#pragma once
#include "DynamicActorGraph.hpp"
#include "Utility.hpp"
#include <array>
#include <cstddef>
#include <mutex>
#include <optional>
#include <sstream>
#include <tuple>
#include <type_traits>
#include <upcxx/upcxx.hpp>
#include <utility>

template <typename ActorType> class SingleActorAGraph final : public DynamicActorGraph
{
  private:
    upcxx::future<GlobalActorRef> retrieveFromOutsideAsyncImpl(GlobalActorRef r); // insert an Actor r into the AG

  public:
    upcxx::future<GlobalActorRef>
    sendActorToAsync(int rank, std::string const &name) override final; // sent actor with name name to rank rank

    upcxx::future<GlobalActorRef>
    sendActorToAsync(int rank, GlobalActorRef ref) override final; // sent actor pointed by ref name to rank rank

    SingleActorAGraph() : DynamicActorGraph() {}

    template <typename S> constexpr std::optional<size_t> typeIndex() { return {0}; }

    // inits actors from given names vector, connection vector and input arguments
    // distributes and creates actors,constructors must have type A(name,T,Ts...)
    // connections are also init after the actors are inserted to the graph
    template <typename T, typename... Ts>
    void initFromList(const std::vector<std::string> &actorNames,
                      const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections,
                      const std::vector<std::tuple<T, Ts...>> &inputArguments)
    {

        upcxx::barrier();
        auto a = util::timepoint();

        // delegate actors for distribution
        this->actorDistribution.distributeActors(actorNames, connections);

        upcxx::progress();
        upcxx::barrier();

        auto b = util::timerDifference(a);

        if (upcxx::rank_me() == 0)
        {
            std::cout << "Distributing the actor graph took: " << b << " seconds" << std::endl;
        }

        a = util::timepoint();

        // initialize actors of your rank
        std::vector<upcxx::intrank_t> partitioning = this->actorDistribution.movePartitioning();
        if (partitioning.empty())
        {
            throw std::runtime_error("returned partitioning vector is illegal");
        }

        int myrank = upcxx::rank_me();

        size_t size = partitioning.size();

        std::vector<ActorImpl *> allocated_actors;

        allocated_actors.reserve(size / upcxx::rank_n());

        upcxx::future<> added;
        for (size_t i = 0; i < size; i++)
        {
            if (partitioning[i] == myrank)
            {
                std::tuple<std::string> tmptup = std::make_tuple(actorNames[i]);
                auto iarg = inputArguments[i];
                std::tuple<std::string, T, Ts...> tmp = std::tuple_cat(std::move(tmptup), std::move(iarg));

                ActorType *a = util::apply_tup([this](auto &&...args) -> ActorType *
                                               { return ::new ActorType(std::move(args)...); },
                                               std::move(tmp));

                ActorImpl *ab = this->upcastActor(a);
                allocated_actors.push_back(ab);

                this->allocated_actors += 1;
            }
        }

        if (sync_init)
        {
            for (auto *ab : allocated_actors)
            {
                this->addActorAsync(ab).wait();
            }
        }
        else
        {
            added = this->ag.addActorAsync(allocated_actors);
            this->pg.insertNodes(actorNames, partitioning);
        }

        if (!sync_init)
        {

            upcxx::discharge();
            upcxx::progress();
            upcxx::barrier();

            added.wait();
        }

        b = util::timerDifference(a);

        if (upcxx::rank_me() == 0)
        {
            std::cout << "Added actors in: " << b << " seconds." << std::endl;
        }

        a = util::timepoint();

        upcxx::barrier();
        upcxx::future<> initialized_connections;

        if (sync_init)
        {
            this->initConnectionsSync(connections);
        }
        else
        {
            initialized_connections = this->initConnections(connections);

            upcxx::discharge();
            upcxx::progress();
            upcxx::barrier();

            initialized_connections.wait();

            this->pg.insertEdges(connections);
        }

        b = util::timerDifference(a);

        if (upcxx::rank_me() == 0)
        {
            std::cout << "Initialized connections of actors in: " << b << " seconds." << std::endl;
        }

        upcxx::barrier();
    }

    // inits actors from given names vector, connection vector and input arguments
    // distributes and creates actors,constructors must have type A(name)
    // connections are also init after the actors are inserted to the graph
    void initFromList(const std::vector<std::string> &actorNames,
                      const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections);

    ~SingleActorAGraph() override final {}

    ActorType *downcast(ActorImpl *ai);
    const ActorType *downcast(const ActorImpl *ai);
};

/*

    Implemation begins here

*/

template <typename ActorType>
void SingleActorAGraph<ActorType>::initFromList(
    const std::vector<std::string> &actorNames,
    const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections)
{
    // delegate actors for distribution

    // auto fp = std::bind(&this->actorDistribution::equal, this->actorDistribution.colIndex,
    // this->actorDistribution.rowIndex) auto fp = std::bind(&Distribution::equal); std::cout << "1" << std::endl;

    this->actorDistribution.distributeActors(actorNames, connections);

    // initialize actors of your rank
    // std::cout << "2" << std::endl;
    // auto partitioning = this->actorDistribution.getPartitioning();
    std::vector<upcxx::intrank_t> partitioning = this->actorDistribution.movePartitioning();
    if (partitioning.empty())
    {
        throw std::runtime_error("returned partitioning vector is illegal");
    }

    // need to wait for arrival
    upcxx::barrier();
    int myrank = upcxx::rank_me();

    // std::cout << "3" << std::endl;
    for (size_t i = 0; i < actorNames.size(); i++)
    {
        // std::cout << "loop-" << i << std::endl;
        if (partitioning[i] == myrank)
        {
            // std::cout << "loop-samerank" << i << std::endl;
            ActorType *atmp = ::new ActorType(actorNames[i]);
            ActorImpl *a = this->upcastActor(atmp);
            this->addActor(a);
            this->allocated_actors += 1;
        }
    }
    // std::cout << "4" << std::endl;
    upcxx::barrier();

    this->initConnections(connections);
    upcxx::barrier();
}

template <typename ActorType>
upcxx::future<GlobalActorRef> SingleActorAGraph<ActorType>::retrieveFromOutsideAsyncImpl(GlobalActorRef r)
{
    if (r.where() == upcxx::rank_me())
    {
        upcxx::future<GlobalActorRef> ref = upcxx::make_future(r);
        return ref;
    }
    else
    {
        upcxx::future<ActorType *> a = upcxx::rpc(
            r.where(),
            [](upcxx::intrank_t callee, GlobalActorRef r) -> upcxx::future<ActorType *>
            {
                ActorImpl *a = *r.local();
                ActorType *downed = dynamic_cast<ActorType *>(a);
                return upcxx::rpc(
                    callee,
                    [](upcxx::view<ActorType> a) -> ActorType *
                    {
                        auto storage = ::new typename std::aligned_storage<sizeof(ActorType), alignof(ActorType)>::type;
                        ActorType *aptr = a.begin().deserialize_into(storage);
                        return aptr;
                    },
                    upcxx::make_view(std::make_move_iterator(downed), std::make_move_iterator(downed + 1)));
            },
            upcxx::rank_me(), r);
        upcxx::future<ActorImpl *> ar =
            a.then([this](ActorType *aptr) -> ActorImpl * { return this->upcastActor(aptr); });
        upcxx::future<GlobalActorRef> gar =
            ar.then([](ActorImpl *ai) -> GlobalActorRef { return upcxx::new_<ActorImpl *>(ai); });
        return gar;
    }
}

template <typename ActorType>
upcxx::future<GlobalActorRef> SingleActorAGraph<ActorType>::sendActorToAsync(int rank, const std::string &name)
{
    GlobalActorRef ref = this->ag.getActorNoThrow(name);
    if (ref == nullptr)
    {
        throw std::runtime_error("Actor should be present in graph during sendActor sag");
    }
    return sendActorToAsync(rank, ref);
}

template <typename ActorType> ActorType *SingleActorAGraph<ActorType>::downcast(ActorImpl *ai)
{
    ActorType *downcasted = nullptr;

    if (std::is_base_of<Actor, ActorType>::value)
    {
        bool grandparent = std::is_base_of<ActorImpl, ActorType>::value;
        bool parent = std::is_base_of<ActorImpl, Actor>::value;

        if (!parent || !grandparent)
        {
            return nullptr;
        }

        Actor *downcastedtmp = reinterpret_cast<Actor *>(ai);

        downcasted = dynamic_cast<ActorType *>(downcastedtmp);
    }
    else
    {
        bool bb = std::is_base_of<ActorImpl, ActorType>::value;

        if (!bb)
        {
            return nullptr;
        }

        downcasted = dynamic_cast<ActorType *>(ai);
    }

    return downcasted;
}

template <typename ActorType> const ActorType *SingleActorAGraph<ActorType>::downcast(const ActorImpl *ai)
{

    if (std::is_base_of<Actor, ActorType>::value)
    {
        bool grandparent = std::is_base_of<ActorImpl, ActorType>::value;
        bool parent = std::is_base_of<ActorImpl, Actor>::value;

        if (!parent || !grandparent)
        {
            return nullptr;
        }

        const Actor *downcastedtmp = reinterpret_cast<const Actor *>(ai);
        const ActorType *downcasted = dynamic_cast<const ActorType *>(downcastedtmp);

        return downcasted;
    }
    else
    {
        bool bb = std::is_base_of<ActorImpl, ActorType>::value;

        if (!bb)
        {
            return nullptr;
        }

        const ActorType *downcasted = dynamic_cast<const ActorType *>(ai);

        return downcasted;
    }
}

template <typename ActorType>
upcxx::future<GlobalActorRef> SingleActorAGraph<ActorType>::sendActorToAsync(int rank, GlobalActorRef ref)
{
    if (ref.where() == upcxx::rank_me())
    {
        if (rank == upcxx::rank_me())
        {
            throw std::runtime_error("ActorImpl already on your rank but you "
                                     "try to send it to yourself?!");
        }
        else
        {
            ActorImpl *act = *ref.local();

            if (act == nullptr)
            {
                throw std::runtime_error("Cant send away non-local actor");
            }

            ActorType *downcasted = downcast(act);

            upcxx::future<GlobalActorRef> fu = upcxx::rpc(
                rank,
                [](upcxx::view<ActorType> a, upcxx::dist_object<DynamicActorGraph *> &rsaag)
                {
                    auto storage = new typename std::aligned_storage<sizeof(ActorType), alignof(ActorType)>::type;
                    ActorType *acttmp = a.begin().deserialize_into(storage);

                    ActorImpl *act = (*rsaag)->upcastActor(acttmp);
                    GlobalActorRef r = upcxx::new_<ActorImpl *>(act);

#ifdef REPORT_MAIN_ACTIONS
                    std::cout << "Serialized and deserialized " << act->getNameRef() << " to " << upcxx::rank_me()
                              << std::endl;
#endif
                    (*rsaag)->allocated_actors += 1;
                    return r;
                },
                upcxx::make_view(std::make_move_iterator(downcasted), std::make_move_iterator(downcasted + 1)),
                this->remoteComponents);

            // Do not delete "del remains" delete them
            // and act should be deleted after the future is completed
            return fu;
        }
    }
    else
    {
        if (rank == upcxx::rank_me())
        {
            return retrieveFromOutsideAsyncImpl(ref);
        }
        else
        {
            throw std::runtime_error("Temporarily disabled!");

            upcxx::future<GlobalActorRef> aglb = retrieveFromOutsideAsyncImpl(ref);
            upcxx::future<ActorType *> act = aglb.then([](GlobalActorRef ref) -> ActorType *
                                                       { return reinterpret_cast<ActorType *>(*(ref.local())); });

            upcxx::future<GlobalActorRef> fu = act.then(
                [rank, this](ActorType *act) -> upcxx::future<GlobalActorRef>
                {
                    upcxx::future<GlobalActorRef> tmp = upcxx::rpc(
                        rank,
                        [](upcxx::view<ActorType> a, upcxx::dist_object<DynamicActorGraph *> &rsaag) -> GlobalActorRef
                        {
                            auto storage =
                                new typename std::aligned_storage<sizeof(ActorType), alignof(ActorType)>::type;
                            ActorType *acttmp = a.begin().deserialize_into(storage);

                            ActorImpl *act = (*rsaag)->upcastActor(acttmp);

                            GlobalActorRef r = upcxx::new_<ActorImpl *>(act);
                            (*rsaag)->allocated_actors += 1;
                            return r;
                        },
                        upcxx::make_view(std::make_move_iterator(act), std::make_move_iterator(act + 1)),
                        this->remoteComponents);
                    return tmp;
                });

            upcxx::future<GlobalActorRef, ActorType *> sent = upcxx::when_all(fu, act);
            upcxx::future<GlobalActorRef> all = sent.then(
                [this](GlobalActorRef ref, ActorType *act) -> GlobalActorRef
                {
                    delete act;
                    this->deallocated_actors += 1;
                    return ref;
                });
            return all;
        }
    }
}
