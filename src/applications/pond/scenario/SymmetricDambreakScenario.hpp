#include "scenario/SWE_Scenario.hh"
#include "scenario/SimulationArea.hpp"

#include <string>
#include <vector>

#pragma once

class SymmetricDambreakScenario : public Scenario
{
    float endTime;
    float xScale;
    float yScale;

  public:
    SymmetricDambreakScenario(float xScale, float yScale, double end);
    ~SymmetricDambreakScenario() = default;

    float getBathymetry(float x, float y) const override;
    float getWaterHeight(float x, float y) const override;
    float endSimulation() const override;
    BoundaryType getBoundaryType(BoundaryEdge e) const override;
    float getBoundaryPos(BoundaryEdge e) const override;
    float defWaterHeight() const override;
};
