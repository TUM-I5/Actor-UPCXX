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

#ifndef __SWE_SCENARIO_H
#define __SWE_SCENARIO_H

#include "scenario/SimulationArea.hpp"

/**
 * enum type: available types of boundary conditions
 */
// TODO: check if enum class typedef enum cuase different behaviour!
enum class BoundaryType : int
{
    OUTFLOW,
    WALL,
    INFLOW,
    CONNECT,
    PASSIVE
};

/**
 * enum type: numbering of the boundary edges
 */
enum class BoundaryEdge : int
{
    BND_LEFT,
    BND_RIGHT,
    BND_BOTTOM,
    BND_TOP
};

/**
 * Scenario defines an interface to initialise the unknowns of a
 * shallow water simulation - i.e. to initialise water height, velocities,
 * and bathymatry according to certain scenarios.
 * Scenario can act as stand-alone scenario class, providing a very
 * basic scenario (all functions are constant); however, the idea is
 * to provide derived classes that implement the SWE_Scenario interface
 * for more interesting scenarios.
 */
class Scenario
{

  public:
    virtual float getWaterHeight(float x, float y) const { return 10.0f; };
    virtual float getVeloc_u(float x, float y) const { return 0.0f; };
    virtual float getVeloc_v(float x, float y) const { return 0.0f; };
    virtual float getBathymetry(float x, float y) const { return 0.0f; };

    virtual float waterHeightAtRest() const { return 10.0f; };

    virtual float endSimulation() const { return 0.1f; };

    virtual float defWaterHeight() const { return 0.f; };

    virtual BoundaryType getBoundaryType(BoundaryEdge edge) const { return BoundaryType::WALL; };
    virtual float getBoundaryPos(BoundaryEdge edge) const
    {
        if (edge == BoundaryEdge::BND_LEFT || edge == BoundaryEdge::BND_BOTTOM)
            return 0.0f;
        else
            return 1.0f;
    };

    virtual SimulationArea getSimulationArea() const
    {
        auto minX = getBoundaryPos(BoundaryEdge::BND_LEFT);
        auto maxX = getBoundaryPos(BoundaryEdge::BND_RIGHT);
        auto minY = getBoundaryPos(BoundaryEdge::BND_BOTTOM);
        auto maxY = getBoundaryPos(BoundaryEdge::BND_TOP);
        return SimulationArea(minX, maxX, minY, maxY);
    }

    virtual ~Scenario(){};
};

#endif
