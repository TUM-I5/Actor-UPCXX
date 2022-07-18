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

#include "scenario/SWE_Scenario.hh"
#include "scenario/SimulationArea.hpp"

#include <string>
#include <vector>

#pragma once

class ScalablePoolDropScenario : public Scenario
{
    float endTime;
    float xScale;
    float yScale;

  public:
    ScalablePoolDropScenario(float xScale, float yScale, double end);

    float getBathymetry(float x, float y) const override;
    float getWaterHeight(float x, float y) const override;
    float endSimulation() const override;
    BoundaryType getBoundaryType(BoundaryEdge e) const override;
    float getBoundaryPos(BoundaryEdge e) const override;
    float defWaterHeight() const override;
};
