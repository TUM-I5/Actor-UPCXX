#pragma once
#include <string>
#include <unordered_map>

enum class ActorState : int
{
    Transitioning = 0,
    Running,
    TemporaryStoppedForMigration,
    Terminated,
    NeedsActivation
};

namespace ASPrinter
{
std::string as2str(ActorState as);
}
