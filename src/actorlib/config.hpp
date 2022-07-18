/**
 * @file
 * This file is part of actorlib.
 *
 * @author Alexander Pöppl (poeppl AT in.tum.de,
 * https://www5.in.tum.de/wiki/index.php/Alexander_P%C3%B6ppl,_M.Sc.)
 *
 * @section LICENSE
 *
 * actorlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * actorlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with actorlib.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * @section DESCRIPTION
 *
 * TODO
 */

#pragma once

#include <string>

#define STR(...) #__VA_ARGS__
#define S(...) STR(__VA_ARGS__)

/*
    Config to choose execution strategy and print debug information about
    GASNET and UPCXX
*/

namespace config
{

enum class ParallelizationType
{
    UPCXX_RANKS
};

enum class MigrationType
{
    NO,
    BULK,
    ASYNC,
    OFFLOAD
};

enum class MigrationStrategy
{
    Global,
    Expansion,
    Undefined
};

enum class VictimChoice
{
    Random,
    Busiest,
    Undefined
};

#ifdef GASNET_PAR
constexpr bool isGasnetSequentialBackend = false;
#else
constexpr bool isGasnetSequentialBackend = true;
#endif

#if UPCXX_ASSERT_ENABLED == 1
constexpr char upcxxCodemode[] = "debug";
#else
constexpr char upcxxCodemode[] = "opt";
#endif

constexpr char gasnetConduit[] = S(ACTORLIB_UPCXX_GASNET_CONDUIT);
constexpr char upcxxInstallation[] = S(ACTORLIB_UPCXX_INSTALLATION);
constexpr char gitRevision[] = S(ACTORLIB_GIT_REVISION);
constexpr char gitCommitDate[] = S(ACTORLIB_GIT_DATE);
constexpr char gitCommitMessage[] = S(ACTORLIB_GIT_COMMIT_MSG);
constexpr bool isUnchangedFromGitRevision = true;

constexpr ParallelizationType parallelization = ParallelizationType::UPCXX_RANKS;

#if MIGRATION_TYPE == 0
constexpr MigrationType migrationtype = MigrationType::NO;
#elif MIGRATION_TYPE == 1
constexpr MigrationType migrationtype = MigrationType::BULK;
#elif MIGRATION_TYPE == 2
constexpr MigrationType migrationtype = MigrationType::ASYNC;
#elif MIGRATION_TYPE == 3
constexpr MigrationType migrationtype = MigrationType::OFFLOAD;
#else
#error "Missconfiguration between CMake migration settings and defined settings in the config.hpp"
#endif

#if defined(GLOBAL_MIGRATION)
constexpr MigrationStrategy migration_strategy = MigrationStrategy::Global;
#else
constexpr MigrationStrategy migration_strategy = MigrationStrategy::Expansion;
#endif

#if defined(STEAL_FROM_BUSY_RANK)
constexpr VictimChoice victim_choice = VictimChoice::Busiest;
#else
constexpr VictimChoice victim_choice = VictimChoice::Random;
#endif

#if defined(STEAL_ONLY_ACTABLE_ACTOR)
constexpr bool steal_only_actable_actor = true;
#else
constexpr bool steal_only_actable_actor = false;
#endif

#if defined(INTERRUPT)
constexpr bool interrupt = true;
#else
constexpr bool interrupt = false;
#endif

constexpr size_t actor_steal_list_size = 10;

std::string configToString();
} // namespace config
