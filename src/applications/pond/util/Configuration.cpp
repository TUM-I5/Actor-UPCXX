/**
 * @file
 * This file is part of Pond.
 *
 * @author Alexander Pöppl (poeppl AT in.tum.de,
 * https://www5.in.tum.de/wiki/index.php/Alexander_P%C3%B6ppl,_M.Sc.)
 *
 * @section LICENSE
 *
 * Pond is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Pond is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Pond.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * @section DESCRIPTION
 *
 * TODO
 */

#include "util/Configuration.hpp"

#include "scenario/SWE_Scenario.hh"
#include "scenario/SWE_simple_scenarios.hh"
#include "util/args.hh"
#ifdef WRITENETCDF
#include "scenario/NetCdfScenario.hpp"
#endif
#include "scenario/ScalableMultiDropScenario.hpp"
#include "scenario/ScalablePoolDropScenario.hpp"
#include "scenario/SymmetricDambreakScenario.hpp"

#include <sstream>
#include <string>

using namespace std::string_literals;

Configuration::Configuration(size_t xSize, size_t ySize, size_t patchSize, size_t numberOfCheckpoints,
                             std::string fileNameBase, Scenario *scenario, float dx,
                             float dy //, double ilim
                             )
    : xSize(xSize), ySize(ySize), patchSize(patchSize), numberOfCheckpoints(numberOfCheckpoints),
      fileNameBase(fileNameBase), scenario(scenario), // lim(ilim),
                                                      // the change is to make sure there are no nullpointer access,
                                                      // actually only should happen with option -h (probably) swap back
                                                      // to -1 when you can
      dx(dx != -1 ? dx : ((scenario != nullptr) ? scenario->getSimulationArea().getDx(xSize) : 1.0)),
      dy(dy != -1 ? dy : ((scenario != nullptr) ? scenario->getSimulationArea().getDy(ySize) : 1.0)), remoteConfig(this)
{

    if (scenario == nullptr)
        throw std::runtime_error("argument -h or scenario is nullptr");

    if (xSize % patchSize != 0)
    {
        throw std::runtime_error("Patch Size "s + std::to_string(patchSize) + " is no even divisor of x size "s +
                                 std::to_string(xSize));
    }
    else if (ySize % patchSize != 0)
    {
        throw std::runtime_error("Patch Size "s + std::to_string(patchSize) + " is no even divisor of y size "s +
                                 std::to_string(ySize));
    }

#ifdef BOTTLENECK
    if (ySize % 2 * patchSize != 0)
    {
        throw std::runtime_error("Patch Size "s + to_string(patchSize) + " is no even divisor of 2*y size "s +
                                 to_string(xSize) + " cannot create double height actors because of this");
    }
#endif
}

std::string Configuration::toString()
{
    std::stringstream ss;
    ss << "#### POND Configuration ####" << std::endl;
    ss << "Total Grid Size:          " << xSize << "x" << ySize << " = " << (xSize * ySize) << " cells. (x*y)"
       << std::endl;
    ss << "Patch Size:               " << patchSize << "x" << patchSize << std::endl;
    ss << "Number of checkpoints:    " << numberOfCheckpoints << std::endl;
    ss << "File name Prefix:         " << fileNameBase << std::endl;
    ss << "Scenario simulation area: " << scenario->getSimulationArea() << std::endl;
    ss << "Cell Size:                " << dx << "m * " << dy << "m (dx * dy)" << std::endl;
    ss << "#### POND Configuration ####" << std::endl;
    return ss.str();
}

const Configuration *Configuration::getSelf(int rank) const
{
    if (rank == upcxx::rank_me())
    {
        return this;
    }
    else
    {
        return upcxx::rpc(
                   rank, [](upcxx::dist_object<Configuration *> &remoteConfig) { return (*remoteConfig); },
                   remoteConfig)
            .wait();
    }
}

Configuration Configuration::build(int argc, char **argv, size_t rank)
{
    tools::Args args;
    args.addOption("grid-size-x", 'x', "Number of cell in x direction");
    args.addOption("grid-size-y", 'y', "Number of cell in y direction");
    args.addOption("patch-size", 'p', "Size of the patches (quadratic)");
    args.addOption("output-basepath", 'o', "Output base file name");
    args.addOption("output-steps-count", 'c', "Number of output time steps");
    args.addOption("scenario", 's',
                   "Number of the scenario: 0 = Symmetric dam break, 1 = "
                   "NetCdfScenario, 2 = ScalablePoolDropScenario,"
                   "3 = ScalableMultiDropScenario");
#ifdef WRITENETCDF
    args.addOption("bathymetry-file", 'b', "Path to a bathymetry file in GEBCO/NetCdf format", tools::Args::Required,
                   false);
    args.addOption("displacement-x", 'u', "X Position of the initial displacement", tools::Args::Required, false);
    args.addOption("displacement-y", 'v', "Y Position of the initial displacement", tools::Args::Required, false);
    args.addOption("displacement-radius", 'r', "Radius of the initial displacement", tools::Args::Required, false);
#endif
    args.addOption("end-simulation", 'e', "Time after which simulation ends", tools::Args::Required, false);

    tools::Args::Result ret = args.parse(argc, argv, rank == 0);

    switch (ret)
    {
    case tools::Args::Error:
        throw std::runtime_error("Unable to parse configuration");
    case tools::Args::Help:
        return Configuration(0, 0, 0, 0, "", nullptr);
    default:
        break;
    }

    auto xSize = args.getArgument<size_t>("grid-size-x");
    auto ySize = args.getArgument<size_t>("grid-size-y");
    auto patchSize = args.getArgument<size_t>("patch-size");
    auto fileNameBase = args.getArgument<std::string>("output-basepath");
    auto numberOfCheckpoints = args.getArgument<size_t>("output-steps-count");
    auto scenarioNumber = args.getArgument<int>("scenario");
    auto endTime = args.getArgument<float>("end-simulation");
    Scenario *scenario;
    if (scenarioNumber == 1)
    {
#ifdef WRITENETCDF
        auto bathymetryFile = args.getArgument<std::string>("bathymetry-file");
        auto displacementX = args.getArgument<double>("displacement-x");
        auto displacementY = args.getArgument<double>("displacement-y");
        auto displacementR = args.getArgument<double>("displacement-radius");
        scenario = ::new NetCdfScenario(bathymetryFile, displacementX, displacementY, displacementR, endTime);
#else
        throw std::runtime_error("NetCdfScenario is disabled, as NetCDF was "
                                 "not found on the system.");
#endif
    }
    else if (scenarioNumber == 0)
    {
        auto gcd = [](size_t a, size_t b)
        {
            size_t h = 0;
            while (b != 0)
            {
                h = a % b;
                a = b;
                b = h;
            }
            return a;
        }(xSize, ySize);

        auto endTime = args.getArgument<float>("end-simulation");
        scenario = ::new SymmetricDambreakScenario(xSize / gcd, ySize / gcd, endTime);
    }
    else if (scenarioNumber == 2)
    {
        auto gcd = [](size_t a, size_t b)
        {
            size_t h = 0;
            while (b != 0)
            {
                h = a % b;
                a = b;
                b = h;
            }
            return a;
        }(xSize, ySize);

        auto endTime = args.getArgument<float>("end-simulation");
        scenario = ::new ScalablePoolDropScenario(xSize / gcd, ySize / gcd, endTime);
    }
    else if (scenarioNumber == 3)
    {
        auto gcd = [](size_t a, size_t b)
        {
            size_t h = 0;
            while (b != 0)
            {
                h = a % b;
                a = b;
                b = h;
            }
            return a;
        }(xSize, ySize);

        auto endTime = args.getArgument<float>("end-simulation");
        scenario = ::new ScalableMultiDropScenario(xSize / gcd, ySize / gcd, endTime);
    }
    else
    {
        scenario = nullptr;
        throw std::runtime_error("Invalid scenario number.");
    }

    return Configuration(xSize, ySize, patchSize, numberOfCheckpoints, fileNameBase, scenario);
}
