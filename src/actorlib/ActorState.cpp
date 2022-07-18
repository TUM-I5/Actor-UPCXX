#include "ActorState.hpp"

std::string ASPrinter::as2str(ActorState as)
{
    switch (as)
    {
    case ActorState::Running:
    {
        return "Running";
    }
    case ActorState::NeedsActivation:
    {
        return "Needs Activation";
    }
    case ActorState::TemporaryStoppedForMigration:
    {
        return "Stopped and will be migrated";
    }
    case ActorState::Terminated:
    {
        return "Temrinated";
    }
    case ActorState::Transitioning:
    {
        return "Transitioning";
    }
    default:
    {
        return "Undefined";
    }
    }
}