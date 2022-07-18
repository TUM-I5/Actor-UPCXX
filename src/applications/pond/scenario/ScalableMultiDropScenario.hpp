#include "scenario/SWE_Scenario.hh"
#include "scenario/SimulationArea.hpp"

#include <string>
#include <vector>

#pragma once

class ScalableMultiDropScenario : public Scenario
{
    float endTime;
    float xScale;
    float yScale;

  public:
    ScalableMultiDropScenario(float xScale, float yScale, double end);
    ~ScalableMultiDropScenario() = default;

    float getBathymetry(float x, float y) const override;
    float getWaterHeight(float x, float y) const override;
    float endSimulation() const override;
    BoundaryType getBoundaryType(BoundaryEdge e) const override;
    float getBoundaryPos(BoundaryEdge e) const override;
    float defWaterHeight() const override;
};
