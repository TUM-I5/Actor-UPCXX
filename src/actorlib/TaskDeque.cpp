#include "TaskDeque.hpp"
#include "ActorGraph.hpp"

int generate_seed()
{
    char *env_var = std::getenv("SEED");
    int seed = 0;
    if (env_var)
    {
        seed = atoi(env_var);
    }

    return seed + upcxx::rank_me();
}

TaskDeque::TaskDeque(ActorGraph *ag)
    : parentActorGraph(ag), remoteTaskque(this),
      tw(ag, this), randomEngine{static_cast<unsigned long int>(generate_seed())}, _stole(upcxx::make_future(true))
{
    constexpr char go[] = "GITTER_OUTPUT";
    constexpr char stealdurterm[] = "STEAL_DURING_TERMINATION";
    constexpr char cpout[] = "SAMPLE_OUTPUT";
    constexpr char cd[] = "STEAL_COOLDOWN";
    constexpr char userma[] = "RMA_TASKCOUNT";
    constexpr char cpbar[] = "SAMPLE_BARRIER";
    constexpr char timeoutmin[] = "TIMEOUT_IN_MINUTES";
    constexpr char cpinterval[] = "SAMPLE_INTERVAL";
    constexpr char stats[] = "PRINT_MORE_STATISTICS";
    constexpr char fcp[] = "EARLY_SAMPLE";

    util::parseBooleanEnvVar(go, print_gitter);
    util::parseBooleanEnvVar(stealdurterm, steal_during_termination);
    util::parseBooleanEnvVar(cpout, sample_output);
    util::parseBooleanEnvVar(cd, steal_cooldown);
    util::parseBooleanEnvVar(userma, rma_taskcount);
    util::parseBooleanEnvVar(cpbar, sample_barrier);
    util::parseBooleanEnvVar(stats, print_additional_statistics);
    util::parseBooleanEnvVar(fcp, early_sample);

    char *val = getenv(timeoutmin);

    if (val == nullptr)
    {
        timeout = -1.0;
    }
    else
    {
        int timeout_in_min = std::stoi(val);
        timeout = 60 * timeout_in_min;
    }

    val = getenv(cpinterval);
    if (val == nullptr)
    {
        cp_interval = 180.0;
    }
    else
    {
        int cp_in_min = std::stoi(val);
        cp_interval = cp_in_min;
    }

    if (upcxx::rank_me() == 0)
    {
        std::cerr << "Config: Print gitter output: " << print_gitter << std::endl;
        std::cerr << "Config: Can steal during termination (when graph partially terminated): "
                  << steal_during_termination << std::endl;
        std::cerr << "Config: Sample output: " << sample_output << std::endl;
        std::cerr << "Config: Cooldown between steal tries and attempts: " << steal_cooldown << std::endl;
        std::cerr << "Config: Additional barrier after sample output: " << sample_barrier << std::endl;

        if (timeout >= 0.0)
        {
            std::cerr << "Config: Timeout is set to " << timeout << " second" << std::endl;
        }

        std::cerr << "Config: Statistics output interval: " << cp_interval << std::endl;
    }

    constexpr char sd[] = "SLOWDOWN";
    constexpr char sdrb[] = "SLOWDOWN_RANK_BEGIN";
    constexpr char sdre[] = "SLOWDOWN_RANK_END";
    constexpr char sdtb[] = "SLOWDOWN_TIME_BEGIN";
    constexpr char sdte[] = "SLOWDOWN_TIME_END";
    util::parseBooleanEnvVar(sd, with_slowdown);
    util::parseIntEnvVar(sdrb, slowdown_begin_at_rank);
    util::parseIntEnvVar(sdre, slowdown_end_at_rank);
    int sf = 0;
    int st = 0;
    util::parseIntEnvVar(sdtb, sf);
    slowdown_from = sf * 60;
    util::parseIntEnvVar(sdte, st);
    slowdown_to = st * 60;

    if (upcxx::rank_me() == 0 && with_slowdown)
    {
        std::cerr << "Nodes [" << slowdown_begin_at_rank << ":" << slowdown_end_at_rank
                  << ") will be slowed down from seconds: "
                  << "[" << slowdown_from << ":" << slowdown_to << "]" << std::endl;
    }

    constexpr char timesync[] = "TIMESTEP_SYNCHRONIZATION";
    val = getenv(timesync);
    if (val == nullptr)
    {
        timestep_synchronization_at = 0;
        if (upcxx::rank_me() == 0)
        {
            std::cout << "Config: No synchronization points" << std::endl;
        }
    }
    else
    {
        timestep_synchronization_at = std::stoi(val);
        if (upcxx::rank_me() == 0)
        {
            std::cout << "Config: Synchronization points after every: " << timestep_synchronization_at << " timesteps."
                      << std::endl;
        }
        do_sync = true;
    }

    sync_point_difference = timestep_synchronization_at;
    next_sync_point = timestep_synchronization_at;
}

bool TaskDeque::abort() { throw std::runtime_error(std::to_string(upcxx::rank_me()) + " is aborting."); }

void TaskDeque::check_abort(std::chrono::_V2::steady_clock::time_point &begin)
{
    if (timeout > 0 && util::timerDifference(begin) >= timeout)
    {
        for (int i = 0; i < upcxx::rank_n(); i++)
        {
            if (i == upcxx::rank_me())
            {
                continue;
            }
            upcxx::rpc_ff(
                i, [](upcxx::dist_object<TaskDeque *> &rmt) { (*rmt)->abort(); }, remoteTaskque);
        }

        upcxx::discharge();

        abort();
    }
}

bool TaskDeque::checkTimeout(std::chrono::_V2::steady_clock::time_point &begin, double timeout)
{
    auto timeoutTimer = util::timepoint();

    bool is_timeout = false;

    if constexpr (config::migrationtype == config::MigrationType::BULK || config::interrupt)
    {

        double dur = util::timerDifference(begin);

        if (timeout > 0 && dur > timeout)
        {
            is_timeout = true;
        }
    }

    this->timeCheckingTimeout += util::timerDifference(timeoutTimer);

    return is_timeout;
}

bool TaskDeque::nothingIsComing() const
{
#ifdef PARALLEL
    std::shared_lock<std::shared_mutex> l(migrationLock);
#endif

    return !this->cant_terminate;
}

upcxx::future<> TaskDeque::steal(const std::string nameOfActorToSteal)
{
    upcxx::future<GlobalActorRef> got = this->parentActorGraph->migcaller->stealActorAsync(nameOfActorToSteal);

    upcxx::future<> h = got.then(
        [this, nameOfActorToSteal](GlobalActorRef _r)
        {
            this->stole += 1;
            if (!this->during_migration)
            {
                throw std::runtime_error("Setting during migration was wrong");
            }
            this->during_migration = false;
            if (!cant_terminate)
            {
                throw std::runtime_error("Setting cant terminate was wrong");
            }
            cant_terminate = false;

// if expansion, our borders has changed we need find new actors
#ifdef STEAL_FROM_BUSY_RANK
            find_new_victims = true;
#endif

#ifdef REPORT_MAIN_ACTIONS
            auto end = std::chrono::system_clock::now();
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);
            std::cout << upcxx::rank_me() << ": " << std::ctime(&end_time) << ": stole " << nameOfActorToSteal
                      << std::endl;
#endif
        });

    return h;
}

bool TaskDeque::canWork() const
{
#ifdef USE_ACTOR_TRIGGERS
    return taskCount > 0;
#else

    for (auto *a : this->parentActorGraph->localActors)
    {
        if (a->actable())
        {
            return true;
        }
    }
    return false;

#endif
}

void TaskDeque::checkTermination()
{
    auto f = this->tw.checkTermination();
    f.then([this](bool res) { this->termination_result = res; });
}

#ifdef USE_ACTOR_TRIGGERS
ActorTriggers TaskDeque::moveTriggers(const std::string &name)
{
    for (auto it = actorTriggers.begin(); it != actorTriggers.end(); ++it)
    {
        const std::string &s = it->first;

        if (s == name)
        {
            ActorTriggers *t = it->second.release();
            ActorTriggers tt = std::move(*t);

            taskCount -= tt.canExecuteNTimes();
            delete t;

            if (atIterator == it)
            {
                atIterator = actorTriggers.erase(it);
                actorTriggersChanged = true;
            }
            else
            {
                actorTriggers.erase(it);
            }

            return tt;
        }
    }

    throw std::runtime_error("ActorTrigger not found");
}
#endif

#ifdef USE_ACTOR_TRIGGERS
void TaskDeque::addTrigger(ActorTriggers &&att)
{
    std::string name = att.getNameRef();

    for (const auto &pr : actorTriggers)
    {
        if (pr.first == name)
        {
            throw std::runtime_error("Only add a full trigger object it is an actor's first time joining the rank");
        }
    }

    std::unique_ptr<ActorTriggers> attptr(new ActorTriggers(std::move(att)));

#ifdef REPORT_MAIN_ACTIONS
    std::cout << upcxx::rank_me() << ": Added triggerCount for: " << name << " with: " << std::endl;
    std::cout << "\t";

    for (auto pr : attptr.get()->getTriggersAsMap())
    {
        std::cout << "(" << pr.first << ", " << pr.second << ") ";
    }
    std::cout << std::endl;
#endif

    size_t t = attptr->canExecuteNTimes();
    taskCount += t;

    actorTriggers.push_front(std::make_pair(std::move(name), std::move(attptr)));
}

upcxx::future<> TaskDeque::sendTriggers(const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    std::vector<upcxx::future<>> futs;
    for (auto &pr : migList)
    {
        ActorTriggers att = moveTriggers(pr.first);
        upcxx::future<> d = upcxx::rpc(
            pr.second,
            [](upcxx::dist_object<TaskDeque *> &rmt, ActorTriggers att, int from)
            {
#ifdef REPORT_MAIN_ACTIONS
// std::cout << "Sent actor trigger from " << from << " to " << upcxx::rank_me() << std::endl;
#endif
                (*rmt)->addTrigger(std::move(att));
            },
            remoteTaskque, std::move(att), upcxx::rank_me());
        futs.push_back(std::move(d));
    }

    return util::combineFutures(std::move(futs));
}
#endif

upcxx::future<std::string> TaskDeque::findVictim()
{
#ifdef MORE_LOCAL_VICTIM_CHOICE
#ifdef STEAL_FROM_BUSY_RANK
    refresh_victim_list = true;
#endif

    if (!refresh_victim_list)
    {
        if (victim_list.empty())
        {
            refresh_victim_list = true;
        }
        else
        {
            std::string name = std::move(*victim_list.begin());
            victim_list.erase(victim_list.begin());
            return upcxx::make_future(std::move(name));
        }
    }
    else
    {
        refresh_victim_list = false;
        upcxx::future<std::vector<std::string>> fut_victims = this->parentActorGraph->migcaller->findActorsToSteal();

        return fut_victims.then(
            [this](std::vector<std::string> victims)
            {
                std::list<std::string> buf(std::make_move_iterator(victims.begin()),
                                           std::make_move_iterator(victims.end()));
                victim_list = std::move(buf);

                if (!victim_list.empty())
                {
                    std::string name = std::move(*victim_list.begin());
                    victim_list.erase(victim_list.begin());
                    return name;
                }

                std::string s;
                return s;
            });
    }
#else
    return this->parentActorGraph->migcaller->findAnActorToStealAndMark();
#endif

    std::string s;
    return upcxx::make_future(std::move(s));
}

upcxx::future<bool> TaskDeque::tryToLockVictimAndItsNeighbors(const std::string name)
{
    if constexpr (config::migrationtype == config::MigrationType::ASYNC)
    {
        /*
            findVictim now locks depending on the config
        */
#ifdef MORE_LOCAL_VICTIM_CHOICE
        if (!name.empty())
        {
            upcxx::future<bool> locked = this->parentActorGraph->migcaller->tryToMarkActorForMigration(name);

            upcxx::future<bool> rly_locked = locked.then(
                [this, name](bool res)
                {
                    auto it = std::find(victim_list.begin(), victim_list.end(), name);
                    if (it != victim_list.end())
                    {
                        this->victim_list.erase(it);
                    }

                    if (res)
                    {
                        return this->parentActorGraph->migcaller->tryStopSelfAndPinNeighborhood(name);
                    }

                    return upcxx::make_future(false);
                });
            return rly_locked;
        }
#else
        if (!name.empty())
        {
            return this->parentActorGraph->migcaller->tryStopSelfAndPinNeighborhood(name);
        }
#endif
        return upcxx::make_future(false);
    }
    else
    {
        if (!name.empty())
        {
            return this->parentActorGraph->migcaller->tryStopSelfAndPinNeighborhood(name);
        }

        return upcxx::make_future(false);
    }
}

upcxx::future<bool> TaskDeque::tryToSteal(upcxx::future<bool> stopped, const std::string name)
{
    std::string victim = name;
    upcxx::future<bool> stole = stopped.then(
        [this, victim](bool stopped) -> upcxx::future<bool>
        {
            if (stopped)
            {
#ifdef REPORT_MAIN_ACTIONS
                std::cout << upcxx::rank_me() << " will steal: " << victim << std::endl;
#endif

                if (cant_terminate)
                {
                    throw std::runtime_error("Second steal attempt started before the first ended");
                }
                cant_terminate = true;

                upcxx::future<> a = steal(victim);
                upcxx::future<> c = a.then([this]() { lastStealTime = util::timepoint(); });
                upcxx::future<bool> b = upcxx::make_future(true);

                consecutiveStealTries = 0;

                return upcxx::when_all(std::move(c), std::move(b));
            }
            else
            {
                if (!during_migration)
                {
                    throw std::runtime_error("Second steal attempt started before the first ended");
                }
                during_migration = false;
                upcxx::future<bool> b = upcxx::make_future(false);
                consecutiveStealTries += 1;

                return b;
            }
        });

    return stole;
}

bool TaskDeque::canExecute(const std::string &name) const { throw std::runtime_error("Dont call"); }

#ifdef PARALLEL
bool TaskDeque::advance(double timeout) { throw std::runtime_error("not yet implemented"); }
#else
bool TaskDeque::advance(double timeout)
{
    std::string runt;

    if (computationBeginSet)
    {
        timeOutside += util::timerDifference(goingOutside);
    }

    std::function<void()> discharge = []() { upcxx::discharge(); };
    std::function<void()> barrier = []() { upcxx::barrier(); };
    std::function<void()> progress = []() { upcxx::progress(); };

    auto l = std::chrono::steady_clock::now();
    std::stringstream ss;
    double totaltimediff = 0.0;

    if (!computationBeginSet)
    {
        computationBeginSet = true;
        computationBegin = util::timepoint();
    }

    size_t iter_without_act = 0;
    double sample_difference = cp_interval;
    double next_sample = cp_interval;
    int ch_time = 1;
    lastStealTime = util::timepoint();
    lastStealTryTime = util::timepoint();
    lastWorkloadExchange = util::timepoint();
    size_t exec_during_interval = 0;

    // std::set<ActorImpl *> s = this->parentActorGraph->getLocalActorsRef();

    /*
    // This is done in startActors()
    for (ActorImpl *ai : s)
    {
        std::string name = ai->getName();
        std::vector<std::string> callers = ai->getAllInPortNames();
        ActorTriggers at(std::move(name), std::move(callers));
        addTrigger(std::move(at));
    }
    */

    util::runWithTimer(barrier, timeInBarrier);
    // upcxx::barrier();

    while (true)
    {
        if (do_sync && timestep_synchronization_at != 0)
        {
            notify_termination();
        }

        util::runWithTimer(discharge, timeInDischarge);

        if (sample_output)
        {
            if (util::timerDifference(computationBegin) > next_sample || 
                ((early_sample && util::timerDifference(computationBegin) >= 5.0 && ch_time == 1) ||
                  (early_sample && util::timerDifference(computationBegin) >= 10.0 && ch_time == 2)
                ))
            {
                

                totaltimediff = util::timerDifference(computationBegin);
                auto pr = this->parentActorGraph->getMaxActorCost();
                std::stringstream sls;
                sls << "Output number: " << ch_time << ", Rank-" << upcxx::rank_me() << ": (1) " << stole << " | (2) "
                    << tries << " | (3)" << util::percentage(stole, tries) << "/100 | (4) "
                    << this->parentActorGraph->actorCount << ", " << this->parentActorGraph->localActors.size()
                    << " | (5) " << timeInAct << " | (6) " << timeInProgress << " | (7) " << timeInBarrier << " | (8) "
                    << timeCheckingTimeout << " | (9) " << timeCheckingTermination << " | (10) "
                    << timeForcedTermination << " | (11) " << termtrial << " | (12) " << timeOutside << " | (13) "
                    << totaltimediff << " | (14) "
                    << util::percentage((timeInAct + timeInProgress + timeInBarrier + timeCheckingTimeout +
                                         timeCheckingTermination + timeOutside),
                                        totaltimediff)
                    << "/100"
                    << " | (15) " << forcedTermination << " | (17) (" << this->parentActorGraph->getActiveActors()
                    << ", " << this->parentActorGraph->activeActorCount << ") | (18) "
#ifdef USE_ACTOR_TRIGGERS
                    << taskCount
#else
                    << "X"
#endif
                    << " | (19) (" << execs << ", " << exec_during_interval << ") | (20) " << this->canWork()
                    << " | (21) " << this->during_migration << " | (22) " << this->termination_result << " | (23) ("
                    << this->parentActorGraph->getTaskCountActive() << ", "
                    << *this->parentActorGraph->taskCount->local() << ") | (24) ("
                    << this->parentActorGraph->getActorCostActive() << ", "
                    << *this->parentActorGraph->gatheredCost->local() << ") | (25) (" << pr.first << ", " << pr.second
                    << ")"
                    << " | (26) " << iter_without_act << " | (27) ("
                    << this->parentActorGraph->getApproxTaskTimeCostActive() << ", "
                    << *this->parentActorGraph->approxTaskCost->local() << ")" << std::endl;

                *this->parentActorGraph->taskCount->local() = this->parentActorGraph->getTaskCountActive();
                *this->parentActorGraph->approxTaskCost->local() =
                    this->parentActorGraph->getApproxTaskTimeCostActive();
                *this->parentActorGraph->gatheredCost->local() = this->parentActorGraph->getActorCostActive();

                if (!early_sample)
                {
                    next_sample += sample_difference;
                   
                } else {
                    if (ch_time > 2)
                    {
                        next_sample += sample_difference;
                    }
                }

                ch_time += 1;
                
                std::cout << sls.str();
                exec_during_interval = 0;

                if (sample_barrier)
                {
                    upcxx::barrier();
                    if (upcxx::rank_me() == 0)
                    {
                        std::cout << "Barrier " << ch_time - 1 << " completed" << std::endl;
                    }
                }
            }
        }

        util::runWithTimer(progress, timeInProgress);
        // upcxx::progress();

        /*
        if (forcedTermination)
        {
            goto ret_true;
        }
        */

        if (termination_result)
        {
            goto ret_true;
        }

        if (print_gitter && iter_without_act > 10000000)
        {
            if (upcxx::rank_me() == 0)
            {
                this->parentActorGraph->migcaller->print_gitter_active_status();
            }

            iter_without_act = 0;

#ifdef USE_ACTOR_TRIGGERS
            for (const auto &pr : actorTriggers)
            {
                const std::string &name = pr.first;
                GlobalActorRef r = this->parentActorGraph->getActorNoThrow(name);

                if (r != nullptr)
                {
                    ActorImpl *ai = *r.local();

                    auto msgs = ai->getMessageCounts();
                    auto triggermap = pr.second->getTriggersAsMap();
                    if (ai->getRunningState() != ActorState::Running)
                    {
                        for (const auto &msgpr : msgs)
                        {
                            std::stringstream ss;
                            if (triggermap.find(msgpr.first)->second != msgpr.second)
                            {
                                ss << "Of" << ai->getNameRef()
                                   << ": message count on " + msgpr.first + " is " + std::to_string(msgpr.second) +
                                          " but the trigger count is " +
                                          std::to_string(triggermap.find(msgpr.first)->second);
                            }
                            std::cerr << ss.str() << std::endl;
                        }
                    }
                }
            }
#endif
        }

// execute them (giotine)
#ifdef USE_ACTOR_TRIGGERS
        actorTriggersChanged = false;
        size_t buffered_actorTriggers_size = actorTriggers.size();
        // std::cout << upcxx::rank_me() << ": actorTriggers.size() == " << actorTriggers.size() << std::endl;
        atIterator = actorTriggers.begin();
        size_t c = 0;
        while (atIterator != actorTriggers.end())
        {
            iter_without_act += 1;

            auto &pr = *atIterator;
            const std::string &name = pr.first;
#else
        std::set<ActorImpl *> la(this->parentActorGraph->getUnorderedLocalActorsRef().begin(),
                                 this->parentActorGraph->getUnorderedLocalActorsRef().end());
        for (ActorImpl *a : la)
        {
            if (sample_output)
            {
                if (util::timerDifference(computationBegin) > next_sample)
                {
                    break;
                }
            }

            iter_without_act += 1;

            // util::runWithTimer(discharge, timeInDischarge);
            util::runWithTimer(progress, timeInProgress);
            // upcxx::discharge();
            // upcxx::progress();

            auto ga = this->parentActorGraph->validPointer(a);
            if (!ga)
            {
                continue;
            }

            const std::string &name = a->getNameRef();
#endif

            // GlobalActorRef aref = this->parentActorGraph->getActorNoThrow(name);
            //  std::cout << upcxx::rank_me() << " is iterating: " << name << " " << aref << std::endl;

            // if (aref != nullptr)
            {
                ActorImpl *ai = a;

                if constexpr (config::migrationtype == config::MigrationType::ASYNC ||
                              config::migrationtype == config::MigrationType::OFFLOAD)
                {
                    if (ai->getRunningState() != ActorState::TemporaryStoppedForMigration)
                    {
                        ai->flushBuffers();
                        ai->sendNotifications();
                    }
                }

                if (ai->actable() && (!do_sync || (do_sync && ai->getPastExecutionCount() < next_sync_point)))
                // if (ai->getRunningState() == ActorState::Running)
                {
#ifdef USE_ACTOR_TRIGGERS
                    if (pr.second->canExecute())
                    {
                        pr.second->decreaseTriggers();
                        taskCount -= 1;

                        execs += 1;
                        exec_during_interval += 1;
                        iter_without_act = 0;
                        consecutiveTerminationChecks = 0;
                        consecutiveStealTries = 0;

                        std::function<void()> intern_b_act = [ai]() { ai->b_act(); };
                        util::runWithTimer(intern_b_act, timeInAct);
                        // ai->b_act();
                        util::runWithTimer(progress, timeInProgress);
                        // util::runWithTimer(discharge, timeInDischarge);
                        //  upcxx::discharge();
                        //  upcxx::progress();
                    }
#else

                    execs += 1;
                    exec_during_interval += 1;
                    iter_without_act = 0;
                    consecutiveTerminationChecks = 0;
                    consecutiveStealTries = 0;

                    std::function<void()> intern_b_act = [ai]() { ai->b_act(); };

                    if (with_slowdown)
                    {
                        auto abefore = util::timepoint();
                        util::runWithTimer(intern_b_act, timeInAct);
                        auto aafter = util::timerDifference(abefore);

                        if (upcxx::rank_me() >= slowdown_begin_at_rank && upcxx::rank_me() < slowdown_end_at_rank)
                        {
                            auto secs_after_start = util::timerDifference(computationBegin);
                            if (secs_after_start >= slowdown_from && secs_after_start <= slowdown_to)
                            {
                                // std::cout << upcxx::rank_me() << " wait for "
                                //           << std::chrono::milliseconds(static_cast<uint64_t>(aafter *
                                //           2000.0)).count()
                                //           << "ms";
                                uint64_t additional_time = static_cast<uint64_t>(aafter * 2000.0);

                                std::this_thread::sleep_for(std::chrono::milliseconds(additional_time));

                                ai->addCost(static_cast<float>(additional_time));
                            }
                        }
                    }
                    else
                    {
                        util::runWithTimer(intern_b_act, timeInAct);
                    }

                    util::runWithTimer(progress, timeInProgress);
                    // util::runWithTimer(discharge, timeInDischarge);
                    //  upcxx::discharge();
                    //  upcxx::progress();

                    if (do_sync && this->timestep_synchronization_at != 0)
                    {
                        notify_termination();

                        if (ai->getPastExecutionCount() == next_sync_point)
                        {
                            enter_sync_point();
                        }
                    }

#endif
                }
#ifdef USE_ACTOR_TRIGGERS
                if (ai->getRunningState() == ActorState::Terminated)
                {
                    pr.second->reset();
                }
#endif

                if constexpr (config::migrationtype == config::MigrationType::OFFLOAD)
                {
                    try_offload();
                }

                util::runWithTimer(progress, timeInProgress);
            }

            util::runWithTimer(progress, timeInProgress);
            // upcxx::progress();

            if constexpr (config::migrationtype == config::MigrationType::BULK || config::interrupt)
            {
                if (checkTimeout(l, timeout))
                {
                    goto ret_false;
                }
            }

            // util::runWithTimer(discharge, timeInDischarge);
            util::runWithTimer(progress, timeInProgress);
            // upcxx::discharge();
            // upcxx::progress();

#ifdef USE_ACTOR_TRIGGERS
            if (atIterator != actorTriggers.end())
            {
                // actor triggers changed means erase was called in the list and we were iterating
                // and were coincidentally the thing we used got deleted so we already have the next iterator
                if (!actorTriggersChanged)
                {
                    ++atIterator;
                }
                actorTriggersChanged = false;
            }

            ++c;

            if (c > actorTriggers.size() * 3)
            {
                atIterator = actorTriggers.end();
                c = 0;
            }
#endif
        }
#ifdef USE_ACTOR_TRIGGERS
        c = 0;
#endif

        if (do_sync && actors_in_sync_point == this->parentActorGraph->getActiveActors())
        {
            wait_for_sync_point();
        }

        // util::runWithTimer(discharge, timeInDischarge);
        util::runWithTimer(progress, timeInProgress);
        // upcxx::discharge();
        // upcxx::progress();

        if constexpr (config::migrationtype == config::MigrationType::BULK || config::interrupt)
        {
            if (checkTimeout(l, timeout))
            {
                goto ret_false;
            }
        }

        // util::runWithTimer(discharge, timeInDischarge);
        util::runWithTimer(progress, timeInProgress);
        // upcxx::discharge();
        // upcxx::progress();

        if constexpr (config::migrationtype == config::MigrationType::ASYNC)
        {
            try_steal();
        }

        if constexpr (config::migrationtype == config::MigrationType::OFFLOAD)
        {
            try_offload();
        }

        // util::runWithTimer(discharge, timeInDischarge);
        util::runWithTimer(progress, timeInProgress);
        // upcxx::discharge();
        // upcxx::progress();

        // try to terminate

        this->checkTermination();
        if (termination_result)
        {
            goto ret_true;
        }

        this->check_abort(computationBegin);

        // this->checkForcedTermination();
        // util::runWithTimer(discharge, timeInDischarge);
        util::runWithTimer(progress, timeInProgress);
        // upcxx::discharge();
        // upcxx::progress();

        iter_without_act += 1;
    }

ret_true:
    if (upcxx::rank_me() == 0)
    {
        ss << "Decomposition: (1) STOLE_COUNT |(2) STOLE_TRIES | (3) REJECTION | (4) ACTIVE_ACTORS | (5) TIME_ACT | "
              "(6) PROGRESS | (7) BARRIER | "
              " (8) TIMEOUT_CHECK | (9) TERMINATION_CHECK_DUR | (10) FORCE_TERMINATION_DUR | (11) "
              "TERMINATION_CHECK_COUNT | (12) OUTSIDE | "
              " (13) TOTAL | (14) PERCENTAGE | (15) FORCED | (24) "
           << std::endl;
    }

    totaltimediff = util::timerDifference(computationBegin);

    ss << "Rank-" << upcxx::rank_me() << ": (1) " << stole << " | (2) " << tries << " | (3)"
       << util::percentage(stole, tries) << "/100 | (4) " << this->parentActorGraph->actorCount << " | (5) "
       << timeInAct << " | (6) " << timeInProgress << " | (7) " << timeInBarrier << " | (8) " << timeCheckingTimeout
       << " | (9) " << timeCheckingTermination << " | (10) " << timeForcedTermination << " | (11) " << termtrial
       << " | (12) " << timeOutside << " | (13) " << totaltimediff << " | (14) "
       << util::percentage((timeInAct + timeInProgress + timeInBarrier + timeCheckingTimeout + timeCheckingTermination +
                            timeOutside),
                           totaltimediff)
       << "/100"
       << " | (15)" << forcedTermination << " | (16) " << termination_result << " | (24) "
       << *this->parentActorGraph->gatheredCost->local() << std::endl;

    std::cout << ss.str();
    goingOutside = util::timepoint();

    upcxx::discharge();
    upcxx::progress();

    totaltimediff = upcxx::reduce_all(totaltimediff, upcxx::experimental::op_max).wait();
    // upcxx::barrier();
    if (upcxx::rank_me() == 0)
    {
        runt = "time-to-solution:" + std::to_string(totaltimediff);
        std::cout << runt << std::endl;
    }
    upcxx::barrier();

    if (print_additional_statistics)
    {
        if (upcxx::rank_me() == 0)
        {
            std::vector<std::vector<uint64_t>> comm_matrix(upcxx::rank_n(), std::vector<uint64_t>());
            std::vector<std::vector<uint64_t>> migration_matrix(upcxx::rank_n(), std::vector<uint64_t>());

            upcxx::future<> commreduced = this->parentActorGraph->reduce_communicaiton_matrix(comm_matrix);
            upcxx::future<> migreduced = this->parentActorGraph->reduce_migration_matrix(migration_matrix);
            upcxx::future<> reduced = upcxx::when_all(std::move(commreduced), std::move(migreduced));

            reduced.wait();

            std::stringstream ss;
            ss << "COMM_MATRIX BEGIN\n";
            for (int i = 0; i < upcxx::rank_n(); i++)
            {
                for (int j = 0; j < upcxx::rank_n() - 1; j++)
                {
                    ss << comm_matrix[i][j];
                    ss << ",";
                }
                ss << comm_matrix[i][upcxx::rank_n() - 1];
                ss << "\n";
            }
            ss << "COMM_MATRIX END\n";

            ss << "MIGRATION_MATRIX BEGIN\n";
            for (int i = 0; i < upcxx::rank_n(); i++)
            {
                for (int j = 0; j < upcxx::rank_n() - 1; j++)
                {
                    ss << migration_matrix[i][j];
                    ss << ",";
                }
                ss << migration_matrix[i][upcxx::rank_n() - 1];
                ss << "\n";
            }
            ss << "MIGRATION_MATRIX END";
            ss << std::endl;
            std::cout << ss.str();
        }
        upcxx::barrier();
    }

    return true;

ret_false:
#ifdef REPORT_MAIN_ACTIONS
    ss << "Rank-" << upcxx::rank_me() << " executed " << execs << " actors during the phase" << std::endl;
    std::cout << ss.str();
#endif
    execs = 0;
    goingOutside = util::timepoint();
    upcxx::barrier();
    return false;
}
#endif

#ifdef USE_ACTOR_TRIGGERS
void TaskDeque::addTrigger(const std::string &actorName, const std::string &portName)
{
    if (actorName.empty() || portName.empty())
    {
        throw std::runtime_error("actor or port name should not be empy when called addTrigger");
    }

    for (const auto &pr : actorTriggers)
    {
        if (pr.first == actorName)
        {
            size_t a = pr.second->canExecuteNTimes();
            pr.second->addTrigger(portName);
            size_t b = pr.second->canExecuteNTimes();
            taskCount += (b - a);
            return;
        }
    }

    throw std::runtime_error(
        "Call addTrigger: " + std::to_string(upcxx::rank_me()) + ", " + actorName + " " + portName +
        "-> call addTrigger with a portName only if the triggers of an actor is already initialized!");
}
#endif

void TaskDeque::try_steal()
{
    if constexpr (config::migrationtype == config::MigrationType::ASYNC)
    {
        if (steal_cooldown &&
            (util::timerDifference(lastStealTime) <= 0.1 || util::timerDifference(lastStealTryTime) <= 0.1))
        {
            return;
        }

        if (util::timerDifference(computationBegin) <= 5.0)
        {
            return;
        }

        if (sync_started)
        {
            return;
        }

        if (/*(!canWork()) &&*/ !during_migration && !during_victim_search && !forcedTermination && _stole.ready())
        {
            lastStealTryTime = util::timepoint();

            if (steal_during_termination ||
                (!steal_during_termination && !this->parentActorGraph->has_a_terminated_actor))
            {
                // find victim sets coming to the name of the victim
                if (during_victim_search)
                {
                    throw std::runtime_error("Second steal attempt started before the first ended");
                }
                during_victim_search = true;

                auto opt_str_fut = findVictim();

                opt_str_fut.then(
                    [this](std::string opt_str)
                    {
                        if (!during_victim_search)
                        {
                            throw std::runtime_error("Second steal attempt started before the first ended");
                        }
                        during_victim_search = false;

#if defined(MORE_LOCAL_VICTIM_CHOICE)
                        if (!opt_str.empty() /*&& !canWork()*/)
#else
                        if (!opt_str.empty())
#endif
                        {

                            tries += 1;
                            {
                                std::string victim = opt_str;

                                if (during_migration)
                                {
                                    throw std::runtime_error("Second steal attempt started before the first ended");
                                }
                                during_migration = true;

                                upcxx::future<bool> stopped = tryToLockVictimAndItsNeighbors(victim);

                                upcxx::future<bool> stole = tryToSteal(stopped, victim);

                                _stole = stole;
                            }
                        }
                        else
                        {
                            consecutiveStealTries += 1;
                        }
                    });
            }
        }
    }
}

upcxx::future<std::string, upcxx::intrank_t> TaskDeque::findActorToOffload()
{
    return this->parentActorGraph->migcaller->findActorsToOffload();
}

upcxx::future<bool> TaskDeque::tryToOffload(upcxx::future<bool> canSend, std::string name, upcxx::intrank_t to)
{
    upcxx::future<bool> offloaded = canSend.then(
        [this, name, to](bool sendable) -> upcxx::future<bool>
        {
            if (sendable)
            {
#ifdef REPORT_MAIN_ACTIONS
                std::cout << upcxx::rank_me() << " will steal: " << name << std::endl;
#endif

                if (cant_terminate)
                {
                    throw std::runtime_error("Second offload attempt started before the first ended");
                }
                cant_terminate = true;

                upcxx::future<GlobalActorRef> a = offload(name, to);
                upcxx::future<> noret = a.then(
                    [this](GlobalActorRef self)
                    {
                        during_migration = false;
                        cant_terminate = false;
                        stole += 1;
                        return;
                    });
                upcxx::future<bool> b = upcxx::make_future(true);

                consecutiveStealTries = 0;

#ifdef REPORT_MAIN_ACTIONS
                std::cout << upcxx::rank_me() << " can offload (stop|pin successful): " << name << std::endl;
#endif
                return upcxx::when_all(noret, b);
            }
            else
            {
                if (!during_migration)
                {
                    throw std::runtime_error("Second offload attempt started before the first ended");
                }
                during_migration = false;
                upcxx::future<bool> b = upcxx::make_future(false);
                consecutiveStealTries += 1;

#ifdef REPORT_MAIN_ACTIONS
                std::cout << upcxx::rank_me() << " cant offload (stop|pin failed): " << name << std::endl;
#endif
                return b;
            }
        });

    return offloaded;
}

upcxx::future<GlobalActorRef> TaskDeque::offload(const std::string nameOfActorToOffload, const upcxx::intrank_t to)
{
    return this->parentActorGraph->migcaller->offloadActorAsync(nameOfActorToOffload, to);
}

void TaskDeque::try_offload()
{
    if constexpr (config::migrationtype == config::MigrationType::OFFLOAD)
    {

        if (util::timerDifference(lastWorkloadExchange) <= 0.1)
        {
            return;
        }

        if (util::timerDifference(computationBegin) <= 5.0)
        {
            return;
        }

        if (sync_started)
        {
            return;
        }

        lastWorkloadExchange = util::timepoint();

        if (!during_migration && !during_victim_search && !forcedTermination && _stole.ready())
        {

            if (steal_during_termination ||
                (!steal_during_termination && !this->parentActorGraph->has_a_terminated_actor))
            {
                // find victim sets coming to the name of the victim
                if (during_victim_search)
                {
                    throw std::runtime_error("Second offload started before the first ended");
                }
                during_victim_search = true;

                auto opt_str_fut = findActorToOffload();

                opt_str_fut.then(
                    [this](std::string opt_str, upcxx::intrank_t to)
                    {
                        if (!during_victim_search)
                        {
                            throw std::runtime_error("Second steal attempt started before the first ended");
                        }
                        during_victim_search = false;

                        if (!opt_str.empty())
                        {
#ifdef REPORT_MAIN_ACTIONS
                            std::cout << upcxx::rank_me() << " tries to offload: " << opt_str << std::endl;
#endif
                            tries += 1;
                            {
                                std::string victim = std::move(opt_str);

                                if (during_migration)
                                {
                                    throw std::runtime_error("Second steal attempt started before the first ended");
                                }
                                during_migration = true;

                                upcxx::future<bool> stopped = tryToLockVictimAndItsNeighbors(victim);

                                upcxx::future<bool> l_stole = tryToOffload(stopped, victim, to);

                                _stole = std::move(l_stole);
                            }
                        }
                    });
            }
        }
    }
}

void TaskDeque::checkForcedTermination()
{
    auto t1 = util::timepoint();
    if (consecutiveTerminationChecks > 10000000 ||
        (consecutiveStealTries > 10000000 && this->parentActorGraph->has_a_terminated_actor))
    {
        std::cerr << "Forced termination!" << std::endl;

        upcxx::future<unsigned int> aa =
            upcxx::reduce_all(this->parentActorGraph->activeActorCount, upcxx::op_fast_add);

        aa.then(
            [this](unsigned int a)
            {
                if (a == 0)
                {
                    forcedTermination = true;
                    consecutiveTerminationChecks = 0;
                    consecutiveStealTries = 0;
                }
            });
    }
    double time = util::timerDifference(t1);
    timeForcedTermination += time;
}

unsigned int TaskDeque::getTaskCount() const
{
#ifdef USE_ACTOR_TRIGGERS
    return taskCount;
#else
    if (rma_taskcount)
    {
        return *this->parentActorGraph->taskCount->local();
    }
    else
    {
        size_t acc = 0;
        for (auto *a : this->parentActorGraph->localActors)
        {
            acc += a->canActNTimes();
        }
        return acc;
    }

#endif
}

void TaskDeque::enter_sync_point()
{
    if (!this->parentActorGraph->has_a_terminated_actor)
    {
        if (actors_in_sync_point == 0)
        {
            this->parentActorGraph->reject_migration = true;
            sync_started = true;
            for (int i = 0; i < upcxx::rank_n(); i++)
            {
                if (upcxx::rank_me() != i)
                {
                    upcxx::rpc_ff(
                        i, [](upcxx::dist_object<TaskDeque *> &rmt) { (*rmt)->sync_started = true; }, remoteTaskque);
                }
            }
            if (upcxx::rank_me() == 0)
            {
                // std::cout << "Sync no." << sync_number << " commences" << std::endl;
            }
            upcxx::discharge();
        }
    }

    actors_in_sync_point += 1;
}

void TaskDeque::wait_for_sync_point()
{
    upcxx::barrier();
    actors_in_sync_point = 0;
    this->parentActorGraph->reject_migration = false;
    sync_started = false;
    if (upcxx::rank_me() == 0)
    {
        // std::cout << "Sync no." << sync_number << " has ended" << std::endl;
    }
    sync_number += 1;
    next_sync_point += sync_point_difference;
}

void TaskDeque::notify_termination()
{
    if (this->parentActorGraph->has_a_terminated_actor)
    {
        for (int i = 0; i < upcxx::rank_n(); i++)
        {
            if (upcxx::rank_me() != i)
            {
                upcxx::rpc_ff(
                    i, [](upcxx::dist_object<TaskDeque *> &rmt) { (*rmt)->termination_started = true; }, remoteTaskque);
            }
        }
        termination_started = true;
        upcxx::discharge();
    }
}