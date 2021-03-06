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

#ifndef __SWE_SIMPLE_SCENARIOS_H
#define __SWE_SIMPLE_SCENARIOS_H

#include <cmath>

#include "SWE_Scenario.hh"

/**
 * Scenario "Radial Dam Break":
 * elevated water in the center of the domain
 */
class RadialDamBreakScenario : public Scenario {

public:
  float getBathymetry(float x, float y) const { return 0.f; };

  float getWaterHeight(float x, float y) const {
    return (sqrt((x - 500.f) * (x - 500.f) + (y - 500.f) * (y - 500.f)) < 100.f)
               ? 15.f
               : 10.0f;
  };

  virtual float endSimulation() const { return (float)50; };

  virtual BoundaryType getBoundaryType(BoundaryEdge edge) const {
    return BoundaryType::WALL;
  };

  /** Get the boundary positions
   *
   * @param i_edge which edge
   * @return value in the corresponding dimension
   */
  float getBoundaryPos(BoundaryEdge i_edge) const {
    if (i_edge == BoundaryEdge::BND_LEFT)
      return (float)0;
    else if (i_edge == BoundaryEdge::BND_RIGHT)
      return (float)1000;
    else if (i_edge == BoundaryEdge::BND_BOTTOM)
      return (float)0;
    else
      return (float)1000;
  };
};

/**
 * Scenario "Bathymetry Dam Break":
 * uniform water depth, but elevated bathymetry in the centre of the domain
 */
class BathymetryDamBreakScenario : public Scenario {

public:
  float getBathymetry(float x, float y) const {
    return (std::sqrt((x - 500.f) * (x - 500.f) + (y - 500.f) * (y - 500.f)) <
            50.f)
               ? -255.f
               : -260.f;
  };

  virtual float endSimulation() const { return (float)15; };

  virtual BoundaryType getBoundaryType(BoundaryEdge edge) const {
    return BoundaryType::OUTFLOW;
  };

  /** Get the boundary positions
   *
   * @param i_edge which edge
   * @return value in the corresponding dimension
   */
  float getBoundaryPos(BoundaryEdge i_edge) const {
    if (i_edge == BoundaryEdge::BND_LEFT)
      return (float)0;
    else if (i_edge == BoundaryEdge::BND_RIGHT)
      return (float)1000;
    else if (i_edge == BoundaryEdge::BND_BOTTOM)
      return (float)0;
    else
      return (float)1000;
  };

  /**
   * Get the water height at a specific location.
   *
   * @param i_positionX position relative to the origin of the bathymetry grid
   * in x-direction
   * @param i_positionY position relative to the origin of the bathymetry grid
   * in y-direction
   * @return water height (before the initial displacement)
   */
  float getWaterHeight(float i_positionX, float i_positionY) const {
    return (float)260;
  }
};

/**
 * Scenario "Sea at Rest":
 * flat water surface ("sea at rest"),
 * but non-uniform bathymetry (id. to "Bathymetry Dam Break")
 * test scenario for "sea at rest"-solution
 */
class SeaAtRestScenario : public Scenario {

public:
  float getWaterHeight(float x, float y) const {
    return (sqrt((x - 0.5) * (x - 0.5) + (y - 0.5) * (y - 0.5)) < 0.1f) ? 9.9f
                                                                        : 10.0f;
  };
  float getBathymetry(float x, float y) const {
    return (sqrt((x - 0.5) * (x - 0.5) + (y - 0.5) * (y - 0.5)) < 0.1f) ? 0.1f
                                                                        : 0.0f;
  };
};

/**
 * Scenario "Splashing Pool":
 * intial water surface has a fixed slope (diagonal to x,y)
 */
class SplashingPoolScenario : public Scenario {

public:
  float getBathymetry(float x, float y) const { return -250.f; };

  float getWaterHeight(float x, float y) const {
    return 250.0f + (5.0f - (x + y) / 200);
  };

  virtual float endSimulation() const { return (float)15; };

  /** Get the boundary positions
   *
   * @param i_edge which edge
   * @return value in the corresponding dimension
   */
  float getBoundaryPos(BoundaryEdge i_edge) const {
    if (i_edge == BoundaryEdge::BND_LEFT)
      return (float)0;
    else if (i_edge == BoundaryEdge::BND_RIGHT)
      return (float)1000;
    else if (i_edge == BoundaryEdge::BND_BOTTOM)
      return (float)0;
    else
      return (float)1000;
  };
};

/**
 * Scenario "Splashing Cone":
 * bathymetry forms a circular cone
 * intial water surface designed to form "sea at rest"
 * but: elevated water region in the centre (similar to radial dam break)
 */
class SWE_SplashingConeScenario : public Scenario {

public:
  float getWaterHeight(float x, float y) const {
    float r = sqrt((x - 0.5f) * (x - 0.5f) + (y - 0.5f) * (y - 0.5f));
    float h = 4.0f - 4.5f * (r / 0.5f);

    if (r < 0.1f)
      h = h + 1.0f;

    return (h > 0.0f) ? h : 0.0f;
  };

  float getBathymetry(float x, float y) const {
    float r = sqrt((x - 0.5f) * (x - 0.5f) + (y - 0.5f) * (y - 0.5f));
    return 1.0f + 9.0f * ((r < 0.5f) ? r : 0.5f);
  };

  float waterHeightAtRest() const { return 4.0f; };
  float endSimulation() const { return 0.5f; };

  virtual BoundaryType getBoundaryType(BoundaryEdge edge) const {
    return BoundaryType::OUTFLOW;
  };
};

#endif
