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
#include "util/Configuration.hpp"

#include <sstream>

SimulationArea::SimulationArea() : minX(0), maxX(0), minY(0), maxY(0) {}

SimulationArea::SimulationArea(float minX, float maxX, float minY, float maxY)
    : minX(minX), maxX(maxX), minY(minY), maxY(maxY)
{
}

SimulationArea::SimulationArea(SimulationArea &&sa) : minX(sa.minX), maxX(sa.maxX), minY(sa.minY), maxY(sa.maxY) {}

SimulationArea::SimulationArea(const SimulationArea &sa) : minX(sa.minX), maxX(sa.maxX), minY(sa.minY), maxY(sa.maxY) {}

SimulationArea &SimulationArea::operator=(const SimulationArea &other)
{
    minX = other.minX;
    maxX = other.maxX;
    minY = other.minY;
    maxY = other.maxY;
    return *this;
}

std::string SimulationArea::toString() const
{
    std::stringstream ss;
    ss.setf(ss.floatfield, std::ios::fixed);
    ss.precision(3);
    ss << "{X: " << minX << " - " << maxX << ", Y: " << minY << " - " << maxY << "}";
    return ss.str();
}

std::ostream &operator<<(std::ostream &os, SimulationArea const &sa)
{
    os << sa.toString();
    return os;
}

SimulationArea makePatchArea(const Configuration &config, size_t xPos, size_t yPos)
{
    float minX = config.dx * config.patchSize * xPos;
    float maxX = config.dx * config.patchSize * (xPos + 1);
    float minY = config.dy * config.patchSize * yPos;
    float maxY = config.dy * config.patchSize * (yPos + 1);
    return SimulationArea(minX, maxX, minY, maxY);
}

SimulationArea makePatchArea(const Configuration *config, size_t xPos, size_t yPos)
{
    float minX = config->dx * config->patchSize * xPos;
    float maxX = config->dx * config->patchSize * (xPos + 1);
    float minY = config->dy * config->patchSize * yPos;
    float maxY = config->dy * config->patchSize * (yPos + 1);
    return SimulationArea(minX, maxX, minY, maxY);
}

SimulationArea makeDoublePatchArea(const Configuration &config, size_t xPos, size_t yPos)
{
    float minX = config.dx * config.patchSize * xPos * 2;
    float maxX = config.dx * config.patchSize * (xPos * 2 + 2);
    float minY = config.dy * config.patchSize * yPos;
    float maxY = config.dy * config.patchSize * (yPos + 1);
    return SimulationArea(minX, maxX, minY, maxY);
}

SimulationArea makeDoublePatchArea(const Configuration *config, size_t xPos, size_t yPos)
{
    float minX = config->dx * config->patchSize * xPos * 2;
    float maxX = config->dx * config->patchSize * (xPos * 2 + 2);
    float minY = config->dy * config->patchSize * yPos;
    float maxY = config->dy * config->patchSize * (yPos + 1);
    return SimulationArea(minX, maxX, minY, maxY);
}

float SimulationArea::getDx(size_t xCells) { return (maxX - minX) / xCells; }

float SimulationArea::getDy(size_t yCells) { return (maxY - minY) / yCells; }
