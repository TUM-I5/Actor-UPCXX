#pragma once
#include "ActorWrapper.hpp"
#include "DynamicActorGraph.hpp"
#include <mutex>
#include <optional>
#include <upcxx/upcxx.hpp>

template <typename A, typename... As> class MultipleActorAGraph final : public DynamicActorGraph
{
  private:
    std::vector<ActorImpl *> actorsInitializedFromList;

    template <size_t I, typename S> constexpr std::optional<size_t> typeIndexIntern()
    {
        return {};
    } // return {} because it is not in it

    template <size_t I, typename S, typename Act, typename... Acts>
    constexpr std::optional<size_t> typeIndexIntern() // return the offset of the element
    {
        if constexpr (std::is_same<expr_type<S>, expr_type<Act>>::value)
        {
            return I;
        }
        // else
        {
            return typeIndexIntern<(I + 1), S, Acts...>();
        }
    }

    // last entry for name + arg with typeindex
    // initiate the actor that has the type typeindex that takes a name and arg as arguments
    template <typename... Args, template <typename...> class T = std::tuple>
    ActorImpl *instantiateIntern(size_t typeIndex, const std::string &name, const T<Args...> &arg)
    {
        return nullptr;
    }

    // name + arg with typeindex
    // initiate the actor that has the type typeindex that takes a name and arg as arguments
    template <typename ActType, typename... ActTypes, typename... TupleTypes,
              template <typename...> class Tuple = std::tuple>
    ActorImpl *instantiateIntern(size_t typeIndex, const std::string &name, const Tuple<TupleTypes...> &arg)
    {
        if (typeIndex == 0)
        {
            std::tuple<std::string> tmptup = std::make_tuple(name);
            Tuple<TupleTypes...> iarg = arg;
            std::tuple<std::string, TupleTypes...> tmp = std::tuple_cat(std::move(tmptup), std::move(iarg));
            ActType *a = util::apply_tup([](auto &&...args) -> ActType * { return ::new ActType(std::move(args)...); },
                                         std::move(tmp));

            ActorImpl *ap = this->upcastActor(a);
            return ap;
        }
        // else
        {
            return instantiateIntern<ActTypes...>(typeIndex - 1, name, std::forward<const Tuple<TupleTypes...>>(arg));
        }
    }

    // name from typeindex
    // instantiate an actor with the typeIndex, but actor does not require input argument Arg
    // constructor of type type_of(type_index)::(name)
    template <typename Act> ActorImpl *instantiateIntern(size_t typeIndex, const std::string &name)
    {
        if (typeIndex == 0)
        {
            ActorImpl *ap = ::new Act(std::forward<const std::string>(name));
            return ap;
        }
        // else
        {
            throw std::invalid_argument("typeIndex illegal");
        }
    }

    // name from typeindex
    // instantiate an actor with the typeIndex, but actor does not require input argument Arg
    // constructor of type type_of(type_index)::(name)
    template <typename Act, typename Bact, typename... Acts>
    ActorImpl *instantiateIntern(size_t typeIndex, const std::string &name)
    {
        if (typeIndex == 0)
        {
            ActorImpl *ap = ::new Act(std::forward<const std::string>(name));
            return ap;
        }
        // else
        {
            return instantiateIntern<Bact, Acts...>(typeIndex - 1, std::forward<const std::string>(name));
        }
    }

    // instantiate an actor with the typeindex constructor of type A(name,arg)
    template <typename... Args>
    ActorImpl *instantiate(size_t typeIndex, const std::string &name, const std::tuple<Args...> &arg)
    {
        auto *p = instantiateIntern<A, As...>(typeIndex, std::forward<const std::string>(name),
                                              std::forward<const std::tuple<Args...>>(arg));
        if (p == nullptr)
            throw std::invalid_argument("Typeindex illegal");
        return p;
    }

    // instantiate an actor with the typeindex constructor of type A(name)
    ActorImpl *instantiate(size_t typeIndex, const std::string &name)
    {
        auto p = instantiateIntern<A, As...>(typeIndex, std::forward<const std::string>(name));
        if (p == nullptr)
            throw std::invalid_argument("Typeindex illegal");
        return p;
    }

    /*
     Typeless wrappers for visit functions
    WR stands for wrapper, AW is ActorWrapper
    */
    ActorWrapper<A, As...> *visitActorToAwHeapWr(ActorImpl *a)
    {
        return visitActorToAwHeap<A, As...>(a);
    } // copy actor to an AW in Heap

    ActorWrapper<A, As...> visitActorToAwStackWr(ActorImpl *a)
    {
        return visitActorToAwStack<A, As...>(a);
    } // copy actor to an AW in Stack

    ActorWrapper<A, As...> *visitToHeapWr(ActorWrapper<A, As...> &input)
    {
        return visitToHeap<A, As...>(input);
    } // copy actor wrapper to heap

    ActorWrapper<A, As...> *visitToHeapWr(ActorWrapper<A, As...> &&input)
    {
        return visitToHeap<A, As...>(std::move(input));
    } // move actor wrapper to heap

    upcxx::future<GlobalActorRef> visitSendWr(int rank, ActorWrapper<A, As...> &aw)
    {
        return visitSend<A, As...>(rank, aw);
    } // send actorwrapper to rank rank

    ActorImpl *visitDevariantWr(ActorWrapper<A, As...> &&aw)
    {
        return visitDevariant<A, As...>(std::move(aw));
    } // extract the saved within the actorwrapper
      // extract the saved within the actorwrapper

    // converts Actor* a -> variant<A,As...>(x)
    // where A,As.. are Actorimplementation types
    // copy ActorImpl to an ActorWrapper in Heap
    template <typename X> constexpr ActorWrapper<A, As...> *visitActorToAwHeap(ActorImpl *a)
    {
        X *x = dynamic_cast<X *>(a);
        if (x != nullptr)
        {
            ActorWrapper<A, As...> *awp = ::new ActorWrapper<A, As...>(std::move(*x));
            return awp;
        }
        // else
        {
            throw std::invalid_argument("Actor's impl type cannot be hold by aw_heap");
        }
    }

    template <typename X, class Y, typename... Xs>
    constexpr ActorWrapper<A, As...> *visitActorToAwHeap(ActorImpl *a) // copy ActorImpl to an ActorWrapper in Heap
    {
        X *x = dynamic_cast<X *>(a);
        if (x != nullptr)
        {
            ActorWrapper<A, As...> *awp = ::new ActorWrapper<A, As...>(std::move(*x));
            return awp;
        }
        // else
        {
            return visitActorToAwHeap<Y, Xs...>(a);
        }
    }

    template <typename X>
    constexpr ActorWrapper<A, As...> visitActorToAwStack(ActorImpl *a) // copy ActorImpl to an ActorWrapper in Stack
    {
        X *x = dynamic_cast<X *>(a);
        if (x != NULL)
        {
            ActorWrapper<A, As...> aw(*x);
            return aw;
        }
        // else
        {
            throw std::runtime_error("Actor's impl type cannot be hold by aw_stack");
        }
    }

    template <typename X, class Y, typename... Xs>
    constexpr ActorWrapper<A, As...> visitActorToAwStack(ActorImpl *a) // copy ActorImpl to an ActorWrapper in Stack
    {
        X *x = dynamic_cast<X *>(a);
        if (x != NULL)
        {
            ActorWrapper<A, As...> aw(*x);
            return aw;
        }
        // else
        {
            return visitActorToAwStack<Y, Xs...>(a);
        }
    }

    // reinits actorwrapper initialized in stack/heap
    // in heap space and returns the pointer
    // where A,As.. are Actorimplementation types
    template <typename X>
    constexpr ActorWrapper<A, As...> *
    visitToHeap(ActorWrapper<A, As...> &&input) // copy ActorWrapper to an ActorWrapper in Heap
    {
        return std::visit(
            [&input](auto &&value) -> ActorWrapper<A, As...> *
            {
                if constexpr (std::is_same<expr_type<decltype(value)>, expr_type<X>>::value)
                {
                    ActorWrapper<A, As...> *awp = ::new ActorWrapper<A, As...>(value);
                    return awp;
                }

                throw std::invalid_argument("Wished type cannot be hold by variant->toheap");
            },
            input.var);
    }

    template <typename X, class Y, typename... Zs>
    constexpr ActorWrapper<A, As...> *
    visitToHeap(ActorWrapper<A, As...> &&input) // copy ActorWrapper to an ActorWrapper in Heap
    {
        return std::visit(
            [this, &input](auto &&value) -> ActorWrapper<A, As...> *
            {
                if constexpr (std::is_same<expr_type<decltype(value)>, expr_type<X>>::value)
                {
                    ActorWrapper<A, As...> *awp = ::new ActorWrapper<A, As...>(std::move(value));
                    return awp;
                }
                // else
                {
                    return visitToHeap<Y, Zs...>(std::move(input));
                }
            },
            input.var);
    }

    // sends the Actor that is hold by the an ActorWrapper
    // pointed by awp member variable to another rank
    // returns a global pointer to the global ActorImpl ptr*
    template <typename X> upcxx::future<GlobalActorRef> visitSend(int rank, ActorWrapper<A, As...> &aw)
    {
        return std::visit(
            [this, rank](auto &&value) -> upcxx::future<GlobalActorRef>
            {
                if constexpr (std::is_same<expr_type<decltype(value)>, expr_type<X>>::value)
                {
                    // std::cout << "still sending... " << std::endl;
                    // X x(std::forward<const X>(value));
                    upcxx::future<GlobalActorRef> ref = upcxx::rpc(
                        rank,
                        [](upcxx::view<X> x, upcxx::dist_object<DynamicActorGraph *> &rtag) -> GlobalActorRef
                        {
                            auto storage = new typename std::aligned_storage<sizeof(X), alignof(X)>::type;
                            X *acttmp = x.begin().deserialize_into(storage);
                            ActorImpl *act = (*rtag)->upcastActor(acttmp);
                            GlobalActorRef ref = upcxx::new_<ActorImpl *>(act);
                            return ref;
                        },
                        upcxx::make_view(std::make_move_iterator(&value), std::make_move_iterator(&value + 1)),
                        this->remoteComponents);
                    return ref;
                }
                // else
                {
                    throw std::invalid_argument("type mismatch while sending actor!");
                }
            },
            aw.var);
    }

    // returns a global pointer to the global ActorImpl ptr*s
    template <typename X, typename Y, typename... Xs>
    upcxx::future<GlobalActorRef> visitSend(int rank, ActorWrapper<A, As...> &aw)
    {
        return std::visit(
            [this, rank, &aw](auto &&value) -> upcxx::future<GlobalActorRef>
            {
                if constexpr (std::is_same<expr_type<decltype(value)>, expr_type<X>>::value)
                {
                    // std::cout << "sending... " << std::endl;
                    // X x(std::forward<X>(value));
                    upcxx::future<GlobalActorRef> ref = upcxx::rpc(
                        rank,
                        [](upcxx::view<X> x, upcxx::dist_object<DynamicActorGraph *> &rtag) -> GlobalActorRef
                        {
                            auto storage = new typename std::aligned_storage<sizeof(X), alignof(X)>::type;
                            X *acttmp = x.begin().deserialize_into(storage);
                            ActorImpl *act = (*rtag)->upcastActor(acttmp);
                            GlobalActorRef ref = upcxx::new_<ActorImpl *>(act);
                            return ref;
                        },
                        upcxx::make_view(std::make_move_iterator(&value), std::make_move_iterator(&value + 1)),
                        this->remoteComponents);
                    return ref;
                }
                // else
                {
                    return visitSend<Y, Xs...>(rank, aw);
                }
            },
            aw.var);
    }

    // returns the copy of the Actorheld in awp
    // wrapper, Actoris allocated in the heap
    // extracts the argument hold within the ActorWrapper
    template <typename X> constexpr ActorImpl *visitDevariant(ActorWrapper<A, As...> &&aw)
    {
        return std::visit(
            [&aw](auto &&value) -> ActorImpl *
            {
                if constexpr (std::is_same<expr_type<decltype(value)>, expr_type<X>>::value)
                {
                    ActorImpl *act = ::new X(std::move(value));
                    return act;
                }
                // else
                {
                    throw std::invalid_argument("Type missmatch, visit_devariant!");
                }
            },
            aw.var);
    }

    // extracts the argument hold within the ActorWrapper
    template <typename X, typename Y, typename... Xs> constexpr ActorImpl *visitDevariant(ActorWrapper<A, As...> &&aw)
    {
        return std::visit(
            [this, &aw](auto &&value) -> ActorImpl *
            {
                if constexpr (std::is_same<expr_type<decltype(value)>, expr_type<X>>::value)
                {
                    X *acttmp = ::new X(std::move(value));
                    ActorImpl *act = this->upcastActor(acttmp);
                    return act;
                }
                // else
                {
                    return visitDevariant<Y, Xs...>(std::move(aw));
                }
            },
            aw.var);
    }

    /*
        Some of the functions that used in bulk sending are not implemented in MAAG yet
    */

    upcxx::future<GlobalActorRef>
    retrieveFromOutsideAsyncImpl(GlobalActorRef r); // add the actor that is not connected to the AG

  public:
    // return the type index of an actor
    template <typename S> constexpr std::optional<size_t> typeIndex() { return typeIndexIntern<0, S, A, As...>(); }

    upcxx::future<GlobalActorRef> sendActorToAsync(int rank, const std::string &name) override final
    {
        GlobalActorRef ref = this->ag.getActorNoThrow(name);
        if (ref == nullptr)
        {
            throw std::runtime_error("Actor should be present in graph during sendActor mag");
        }
        return sendActorToAsync(rank, ref);
    }

    upcxx::future<GlobalActorRef> sendActorToAsync(int rank, const GlobalActorRef ref) override final
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
                ActorWrapper<A, As...> *aw = this->visitActorToAwHeapWr(act);

                upcxx::future<GlobalActorRef> sent = upcxx::rpc(
                    rank,
                    [](upcxx::view<ActorWrapper<A, As...>> aw, upcxx::dist_object<DynamicActorGraph *> &rtag)
                    {
                        auto storage = new typename std::aligned_storage<sizeof(ActorWrapper<A, As...>),
                                                                         alignof(ActorWrapper<A, As...>)>::type;

                        DynamicActorGraph *dag = *rtag;
                        MultipleActorAGraph<A, As...> *maag = dynamic_cast<MultipleActorAGraph<A, As...> *>(dag);

                        ActorWrapper<A, As...> *acttmp = aw.begin().deserialize_into(storage);
                        ActorImpl *act = maag->visitDevariantWr(std::move(*acttmp));
                        GlobalActorRef ref = upcxx::new_<ActorImpl *>(act);
                        return ref;
                    },
                    upcxx::make_view(std::make_move_iterator(aw), std::make_move_iterator(aw + 1)),
                    this->remoteComponents);

                return sent;
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
                upcxx::future<GlobalActorRef> aglb = retrieveFromOutsideAsyncImpl(ref);
                upcxx::future<ActorWrapper<A, As...> *> recvd =
                    aglb.then([this](GlobalActorRef ref) -> ActorWrapper<A, As...> *
                              { return this->visitActorToAwHeapWr(*ref.local()); });

                upcxx::future<GlobalActorRef> sent = recvd.then(
                    [rank, this](ActorWrapper<A, As...> *aw) -> upcxx::future<GlobalActorRef>
                    {
                        upcxx::future<GlobalActorRef> tmp = upcxx::rpc(
                            rank,
                            [](upcxx::view<ActorWrapper<A, As...>> aw, upcxx::dist_object<DynamicActorGraph *> &rtag)
                            {
                                auto storage = new typename std::aligned_storage<sizeof(ActorWrapper<A, As...>),
                                                                                 alignof(ActorWrapper<A, As...>)>::type;

                                DynamicActorGraph *dag = *rtag;
                                MultipleActorAGraph<A, As...> *maag =
                                    dynamic_cast<MultipleActorAGraph<A, As...> *>(dag);

                                ActorWrapper<A, As...> *acttmp = aw.begin().deserialize_into(storage);
                                ActorImpl *act = maag->visitDevariantWr(std::move(*acttmp));
                                GlobalActorRef ref = upcxx::new_<ActorImpl *>(act);
                                return ref;
                            },
                            upcxx::make_view(std::make_move_iterator(aw), std::make_move_iterator(aw + 1)),
                            this->remoteComponents);
                        return tmp;
                    });

                upcxx::future<GlobalActorRef, ActorWrapper<A, As...> *> both = upcxx::when_all(sent, recvd);
                upcxx::future<GlobalActorRef> all = both.then(
                    [](GlobalActorRef ref, ActorWrapper<A, As...> *aw) -> GlobalActorRef
                    {
                        delete aw;
                        return ref;
                    });
                return all;
            }
        }
    }

    // Args <- std::variant<A,B,C...> that unified type of input arguments for any Actortype declared
    // inits the actors that have have the typeIndex currentTypeToInit from given name argument and partitioning
    // vectores typeindices must be generated for every actor by the user
    template <size_t maxTypes, typename Arg, typename... Args>
    void initAllTypesIntern(const std::vector<std::string> &actorNames, const std::vector<size_t> &typeIndices,
                            const std::vector<std::tuple<Args...>> &arg,
                            const std::vector<upcxx::intrank_t> &partitioning)
    {
        size_t currentTypeToInit = 0;
        if (currentTypeToInit >= maxTypes)
        {
            return;
        }

        int myrank = upcxx::rank_me();
        for (size_t i = 0; i < typeIndices.size(); i++)
        {
            // std::cout << i << " " << partitioning.size() << " " << typeIndices[i] << " " << partitioning[i] << " " <<
            // currentTypeToInit << std::endl;
            if (partitioning[i] == myrank)
            {
                ActorImpl *ap = instantiate<Args...>(typeIndices[i], std::forward<const std::string>(actorNames[i]),
                                                     std::forward<const std::tuple<Args...>>(arg[i]));
                this->addActor(ap);
                actorsInitializedFromList.push_back(ap);
            }
        }
        return;
    }

    // Args <- std::variant<A,B,C...> that unified type of input arguments for any Actortype declared
    // init all types of actor by iterating through all type indices and calling helper functions
    template <size_t maxTypes, typename... Args>
    void initAllTypes(const std::vector<std::string> &actorNames, const std::vector<size_t> &typeIndices,
                      const std::vector<std::tuple<Args...>> &arg, const std::vector<upcxx::intrank_t> &partitioning)
    {
        initAllTypesIntern<maxTypes, Args...>(std::forward<const std::vector<std::string>>(actorNames),
                                              std::forward<const std::vector<size_t>>(typeIndices),
                                              std::forward<const std::vector<std::tuple<Args...>>>(arg), partitioning);
    }

    // init One type
    // inits only the actors that have the type T
    // constructor must have the type T(name,arg)
    template <typename T, typename Arg>
    void initOneType(const std::vector<std::string> &actorNames, std::vector<size_t> &offsetToInit,
                     const std::vector<upcxx::intrank_t> &partitioning, const std::vector<Arg> &arg)
    {
        int myrank = upcxx::rank_me();

        for (size_t i = 0; i < offsetToInit.size(); i++)
        {
            auto typeIndexOpt = typeIndex<T>();
            if (typeIndexOpt.has_value())
            {
                if (partitioning[i] == myrank)
                {
                    std::string name = actorNames[offsetToInit[i]];
                    instantiate<Arg>(*typeIndexOpt, std::move(name), std::forward<const Arg>(arg[i]));
                }
            }
            else
            {
                throw std::invalid_argument("Undeclared type (add type to the MultiActorAGraph");
            }
        }
    }

    // init One type
    // inits only the actors that have the type T
    // constructor must have the type T(name)
    template <typename T>
    void initOneType(const std::vector<std::string> &actorNames, const std::vector<size_t> &offsetToInit)
    {
        for (size_t i = 0; i < offsetToInit.size(); i++)
        {
            auto typeIndexOpt = typeIndex<T>();
            if (typeIndexOpt.has_value())
            {
                std::string name = actorNames[offsetToInit[i]];
                instantiate(*typeIndexOpt, std::move(name));
            }
            else
            {
                throw std::invalid_argument("Undeclared type (add type to the MultiActorAGraph");
            }
        }
    }

    // actorNames -> names of the actors
    // connections -> connections between Actor(same order as connectPorts: first Actorname, outport, second Actorname,
    // in port name) typeindices -> type index for the Actor(recieve from multiple Actorgraph) args -> args needed.
    // First Arg ist for typeIndex 0, second Arg for typeIndex 1 and so on. (use monostate if that type requires no
    // extra arguments, monostate arguments are ignored, but fot this approach the actors need to be able to initialzied
    // with variants all actors require an unified inptu std::variant<std::monostate,std::string,...> for input
    // management) distributes actors and then inits them by iterating types constructor must have the type
    // Type(name,arg)
    template <typename Arg, typename... Args>
    void initFromList(const std::vector<std::string> &actorNames,
                      const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections,
                      const std::vector<size_t> &typeIndices, const std::vector<std::tuple<Arg, Args...>> &arg)
    {
        this->actorDistribution.distributeActors(actorNames, connections);
        upcxx::barrier();
        std::vector<upcxx::intrank_t> partitioning = this->actorDistribution.movePartitioning();
        upcxx::barrier();
        // int myrank = upcxx::rank_me();
        constexpr size_t maxcount = util::counter<A, As...>();

        initAllTypes<maxcount, Arg, Args...>(std::forward<const std::vector<std::string>>(actorNames),
                                             std::forward<const std::vector<size_t>>(typeIndices),
                                             std::forward<const std::vector<std::tuple<Arg, Args...>>>(arg),
                                             partitioning);

        upcxx::barrier();
        this->initConnections(
            std::forward<const std::vector<std::tuple<std::string, std::string, std::string, std::string>>>(
                connections));

        upcxx::barrier();
    }

    // some as above, to use when no Actortype requires additional arguments other than its name
    // distributes actors and then inits them by iterating types
    // constructor must have the type Type(name)
    void initFromList(const std::vector<std::string> &actorNames,
                      const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections,
                      const std::vector<size_t> &typeIndices);

    // static_cast<MultipleActorAGraph<A, As...> *>
    MultipleActorAGraph() : DynamicActorGraph(), actorsInitializedFromList() {}

    ~MultipleActorAGraph() override final {}
};

/*
template <typename A, typename... As> void MultipleActorAGraph<A,As...>::
*/

template <typename A, typename... As>
void MultipleActorAGraph<A, As...>::initFromList(
    const std::vector<std::string> &actorNames,
    const std::vector<std::tuple<std::string, std::string, std::string, std::string>> &connections,
    const std::vector<size_t> &typeIndices)
{
    this->actorDistribution.distributeActors(actorNames, connections);
    upcxx::barrier();

    int myrank = upcxx::rank_me();

    size_t size = this->actorDistribution.partitioned.size();
    for (size_t i = 0; i < size; i++)
    {
        if (this->actorDistribution.partitioned[i] == myrank)
        {
            ActorImpl *ap = instantiate(typeIndices[i], std::forward<const std::string>(actorNames[i]));
            this->addActor(ap);
            actorsInitializedFromList.push_back(ap);
        }
    }
    // std::cout << "4" << std::endl;
    upcxx::barrier();

    this->initConnections(
        std::forward<const std::vector<std::tuple<std::string, std::string, std::string, std::string>>>(connections));
    upcxx::barrier();
}

template <typename A, typename... As>
upcxx::future<GlobalActorRef> MultipleActorAGraph<A, As...>::retrieveFromOutsideAsyncImpl(GlobalActorRef r)
{
    if (r.where() == upcxx::rank_me())
    {
        return upcxx::make_future(r);
    }
    else
    {
        upcxx::future<ActorWrapper<A, As...> *> aw = upcxx::rpc(
            r.where(),
            [](upcxx::intrank_t callee, GlobalActorRef r,
               upcxx::dist_object<DynamicActorGraph *> &rtag) -> upcxx::future<ActorWrapper<A, As...> *>
            {
                ActorImpl *a = *r.local();
                DynamicActorGraph *dag = *rtag;
                MultipleActorAGraph<A, As...> *maag = dynamic_cast<MultipleActorAGraph<A, As...> *>(dag);
                ActorWrapper<A, As...> *downed = maag->visitActorToAwHeapWr(a);

                return upcxx::rpc(
                    callee,
                    [](upcxx::view<ActorWrapper<A, As...>> a) -> ActorWrapper<A, As...> *
                    {
                        auto storage = new typename std::aligned_storage<sizeof(ActorWrapper<A, As...>),
                                                                         alignof(ActorWrapper<A, As...>)>::type;
                        ActorWrapper<A, As...> *aw = a.begin().deserialize_into(storage);
                        return aw;
                    },
                    upcxx::make_view(std::make_move_iterator(downed), std::make_move_iterator(downed + 1)));
            },
            upcxx::rank_me(), r, this->remoteComponents);

        upcxx::future<ActorImpl *> ar = aw.then([this](ActorWrapper<A, As...> *awtmp) -> ActorImpl *
                                                { return this->visitDevariantWr(std::move(*awtmp)); });
        upcxx::future<GlobalActorRef> gar =
            ar.then([](ActorImpl *ai) -> GlobalActorRef { return upcxx::new_<ActorImpl *>(ai); });
        return gar;
    }
}