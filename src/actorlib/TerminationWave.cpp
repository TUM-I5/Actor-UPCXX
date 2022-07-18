#include "TerminationWave.hpp"
#include "ActorGraph.hpp"
#include "TaskDeque.hpp"

TerminationWave::TerminationWave(const ActorGraph *pag, TaskDeque *ptd)
    : checked(0), n(util::rank_n()), remoteSources(this), pag(pag), completed_count(0), total_msg_received(0),
      msg_id(0), received{}, ptd(ptd)
{
    received.resize(util::rank_n());
    std::fill(received.begin(), received.end(), std::make_pair(0, -1));
}

bool TerminationWave::selfTerminated() { return this->ptd->parentActorGraph->activeActorCount == 0; }

void TerminationWave::setTerminated()
{
    this->received[upcxx::rank_me()] = std::make_pair(~(size_t)0, 1);

    // broadcast that I am done to all other ranks
    for (int i = 0; i < util::rank_n(); i++)
    {
        if (upcxx::rank_me() == i)
        {
            continue;
        }
        else
        {
            upcxx::rpc_ff(
                i,
                [](upcxx::dist_object<TerminationWave *> &tw, size_t i)
                { (*tw)->received[i] = std::make_pair(~(size_t)0, 1); },
                remoteSources, upcxx::rank_me());
        }
    }
    upcxx::discharge();

    sent = true;
}

bool TerminationWave::globallyTerminated()
{
    int i = 0;
    for (auto &el : received)
    {
        if (el.first != ~(size_t)0 && el.second != 1)
        {
#ifdef REPORT_MAIN_ACTIONS
// std::cout << upcxx::rank_me() << " checks, rank: "<< i << " is not termianted" << std::endl;
#endif

            return false;
        }

        i++;
    }

    return true;
}

upcxx::future<bool> TerminationWave::checkTermination()
{
    checked += 1;
    call_id += 1;

    if (global_terminated)
    {
        return upcxx::make_future(true);
    }

    if (!this->selfTerminated())
    {
        return upcxx::make_future(false);
    }

    // we can indeed terminate
    answered = 1;
    terminated = 1;
    std::vector<upcxx::future<>> v;
    v.reserve(upcxx::rank_n() - 1);

    for (int i = 0; i < upcxx::rank_n(); i++)
    {
        if (i == upcxx::rank_me())
        {
            continue;
        }

        upcxx::future<std::pair<bool, size_t>> answ = upcxx::rpc(
            i,
            [](upcxx::dist_object<TerminationWave *> &rmt, size_t call_id)
            { return std::make_pair((*rmt)->selfTerminated(), call_id); },
            remoteSources, call_id);

        upcxx::future<> a = answ.then(
            [this](std::pair<bool, size_t> pr)
            {
                if (pr.second != call_id)
                {
                    throw std::runtime_error("Ah fuck");
                }

                this->answered += 1;
                if (pr.first)
                {
                    this->terminated += 1;
                }
            });

        v.push_back(std::move(a));
    }

    upcxx::future<> collected_answers = util::combineFutures(v);

    upcxx::future<bool> result = collected_answers.then(
        [this]()
        {
            if (answered != static_cast<size_t>(upcxx::rank_n()))
            {
                throw std::runtime_error("Not everybody answered yet but we stopped waiting");
            }

            if (terminated == answered)
            {
                for (int i = 0; i < upcxx::rank_n(); i++)
                {
                    if (i == upcxx::rank_me())
                    {
                        global_terminated = true;
                    }
                    else
                    {
                        upcxx::rpc_ff(
                            i,
                            [](upcxx::dist_object<TerminationWave *> &rmt) { return (*rmt)->global_terminated = true; },
                            remoteSources);
                    }
                }

                return true;
            }

            return false;
        });

    return result;
}
