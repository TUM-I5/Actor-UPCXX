#include "scenario/SymmetricDambreakScenario.hpp"

#include <cmath>
#include <vector>

SymmetricDambreakScenario::SymmetricDambreakScenario(float xScale, float yScale, double end)
    : endTime(end), xScale(xScale), yScale(yScale)
{
}

float SymmetricDambreakScenario::getBathymetry(float x, float y) const { return 0.0f; }

float SymmetricDambreakScenario::getWaterHeight(float x, float y) const
{
    constexpr float radius = 15.0f;
    constexpr float inverse_radius = 85.0f;
    constexpr float full = 100.0f;

    float xRangeLow = radius * xScale;
    float xRangeHigh = inverse_radius * xScale;

    if (x <= xRangeLow || x >= xRangeHigh)
    {
        return 15.0;
    }
    else
    {
        return 10.0;
    }
}

float SymmetricDambreakScenario::defWaterHeight() const { return 10.0f; }

float SymmetricDambreakScenario::endSimulation() const { return this->endTime; }

float SymmetricDambreakScenario::getBoundaryPos(BoundaryEdge e) const
{
    switch (e)
    {
    case BoundaryEdge::BND_TOP:
        return 100.0f * yScale;
    case BoundaryEdge::BND_RIGHT:
        return 100.f * xScale;
    default:
        return 0.0f;
    }
}

BoundaryType SymmetricDambreakScenario::getBoundaryType(BoundaryEdge e) const { return BoundaryType::WALL; }
