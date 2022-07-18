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

#include "config.hpp"

#include <sstream>

#define STR(...) #__VA_ARGS__
#define S(...) STR(__VA_ARGS__)

namespace config
{

std::string configToString()
{
    std::stringstream ss;
    std::string parallelizationStr = "UPC++ rank-sequential";
    ss << "UPC++ Actor Library Configuration" << std::endl;
    ss << "=================================" << std::endl;
    ss << "Actorlib mode:        " << parallelizationStr << std::endl;
    ss << "Git revision:         " << gitRevision << std::endl;
    ss << "Git revision date:    " << gitCommitDate << std::endl;
    ss << "Git revision message: " << gitCommitMessage << std::endl;
    ss << "UPC++ install:        " << upcxxInstallation << std::endl;
    ss << "UPC++ codemode:       " << upcxxCodemode << std::endl;
    ss << "UPC++ Backend type:   " << (isGasnetSequentialBackend ? "Sequential" : "Parallel") << " Backend"
       << std::endl;
    ss << "UPC++ GASNET Conduit: " << gasnetConduit << std::endl;
    return ss.str();
}

} // namespace config