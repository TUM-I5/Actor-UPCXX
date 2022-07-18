#pragma once

enum class SimulationActorState : int
{
    INITIAL = 0,
#ifdef LAZY_ACTIVATION
    RESTING,
#endif
    RUNNING,
    TERMINATED
};
