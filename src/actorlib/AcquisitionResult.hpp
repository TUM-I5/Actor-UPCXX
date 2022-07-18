#pragma once
#include <string>
#include <unordered_map>

/*d::d
    Intenal usage, needed for Migration it indicates whether the actor could be locked,
    Terminated means it was teminated because the actor reached the terminated state,
    and it was not stopped form the ActoGraph before ther Termination
*/

enum class AcquisitionResult
{
    Ok = 0,
    LostLockingRace,
    ActorTerminated,
    ActorBeingMigrated,
    UnmigrateableNeighbor,
    RankMigrationOnLimit,
    ActorHasNoTasks,
    NotEnoughActors,
    OtherRankHasNoTasks,
    ThisRankHasNoTasks,
    TerminatedActorHere,
    TerminatedActorThere
};

namespace ARPrinter
{
std::string ar2str(AcquisitionResult ar);
}
