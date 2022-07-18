/**
 * @file
 * This file is part of Pond.
 *
 * @author Michael Bader, Kaveh Rahnema, Tobias Schnabel
 * @author Sebastian Rettenberger (rettenbs AT in.tum.de,
 * http://www5.in.tum.de/wiki/index.php/Sebastian_Rettenberger,_M.Sc.)
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

#include <iostream>
#include <string>

#pragma once

struct Configuration;

struct SimulationArea
{
    float minX;
    float maxX;
    float minY;
    float maxY;

    SimulationArea();
    SimulationArea(float minX, float maxX, float minY, float maxY);
    SimulationArea(SimulationArea &&sa);
    SimulationArea(const SimulationArea &sa);
    ~SimulationArea() = default;
    SimulationArea &operator=(const SimulationArea &other);

    std::string toString() const;
    float getDx(size_t xCells);
    float getDy(size_t yCells);

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, const SimulationArea &object)
        {

            writer.write(object.minX);
            writer.write(object.maxX);
            writer.write(object.minY);
            writer.write(object.maxY);
            return;
        }

        template <typename Reader> static SimulationArea *deserialize(Reader &reader, void *storage)
        {

            int a = reader.template read<int>();
            int b = reader.template read<int>();
            int c = reader.template read<int>();
            int d = reader.template read<int>();
            SimulationArea *sa = ::new (storage) SimulationArea(a, b, c, d);
            return sa;
        }
    };
};

std::ostream &operator<<(std::ostream &os, SimulationArea const &sa);

SimulationArea makePatchArea(const Configuration &config, size_t xPos, size_t yPos);
SimulationArea makePatchArea(const Configuration *config, size_t xPos, size_t yPos);
SimulationArea makeDoublePatchArea(const Configuration *config, size_t xPos, size_t yPos);
SimulationArea makeDoublePatchArea(const Configuration &config, size_t xPos, size_t yPos);