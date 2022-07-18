#include "AcquisitionResult.hpp"

std::string ARPrinter::ar2str(AcquisitionResult ar)
{
    switch (ar)
    {
    case AcquisitionResult::Ok:
    {
        return "Ok";
    }
    case AcquisitionResult::LostLockingRace:
    {
        return "LostLockingRace";
    }
    case AcquisitionResult::ActorTerminated:
    {
        return "ActorTerminated";
    }
    case AcquisitionResult::ActorBeingMigrated:
    {
        return "ActorBeingMigrate";
    }
    case AcquisitionResult::UnmigrateableNeighbor:
    {
        return "UnmigrateableNeighbor";
    }
    case AcquisitionResult::RankMigrationOnLimit:
    {
        return "RankMigrationOnLimit";
    }
    case AcquisitionResult::ActorHasNoTasks:
    {
        return "ActorHasNoTasks";
    }
    case AcquisitionResult::NotEnoughActors:
    {
        return "NotEnoughActors";
    }
    case AcquisitionResult::OtherRankHasNoTasks:
    {
        return "OtherRankHasNoTasks";
    }
    case AcquisitionResult::ThisRankHasNoTasks:
    {
        return "ThisRankHasNoTasks";
    }
    case AcquisitionResult::TerminatedActorHere:
    {
        return "TerminatedActorHere";
    }
    case AcquisitionResult::TerminatedActorThere:
    {
        return "TerminatedActorThere";
    }
    default:
    {
        return "Undefined";
    }
    }
}