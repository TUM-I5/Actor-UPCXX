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

#include "scenario/SimulationArea.hpp"

#include <string>
#include <vector>

#pragma once

class NetCdfData
{
  private:
    std::string fileName;
    bool isLoaded;

    size_t xSize;
    size_t ySize;
    std::vector<double> xCoordinates;
    std::vector<double> yCoordinates;
    std::vector<float> data;

  public:
    NetCdfData(const std::string &fileName);
    ~NetCdfData() = default;
    bool isLoadedSuccessfully();
    size_t getXSize();
    size_t getYSize();
    SimulationArea getSimulationArea();
    std::vector<float> &&exportData();
};
