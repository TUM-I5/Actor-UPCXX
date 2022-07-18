#pragma once
#include "Utility.hpp"
#include <atomic>
#include <upcxx/upcxx.hpp>
#include <utility>

/*
    Implementation sees the whole topology but other an implementation based on:
    A Termination Detection Technique Using Gossip in Cloud Computing Environments
    10.1007/978-3-642-35606-3_51

    might be necessary
*/
class ActorGraph;
class TaskDeque;

class TerminationWave
{
  public:
    size_t checked;
    bool global_terminated = false;

  private:
    size_t call_id = 0;
    volatile size_t answered = 0;
    volatile size_t terminated = 0;

    unsigned int n;
    upcxx::dist_object<TerminationWave *> remoteSources;
    const ActorGraph *pag;
#ifdef PARALLEL
    std::atomic<uint32_t> completed_count; // left 32 bits are for temrinated ranks right 32 for return calls
    std::atomic<uint32_t> total_msg_received;
#else
    uint32_t completed_count;
    uint32_t total_msg_received;
#endif
    size_t msg_id;
    bool sent = false;

  public:
    std::vector<std::pair<size_t, int>> received;
    TaskDeque *ptd;
    void setTerminated();
    bool globallyTerminated();
    bool selfTerminated();

  public:
    TerminationWave(const ActorGraph *pag, TaskDeque *td);
    ~TerminationWave() = default;
    upcxx::future<bool> checkTermination();
};
