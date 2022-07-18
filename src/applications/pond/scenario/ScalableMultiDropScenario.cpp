#include "scenario/ScalableMultiDropScenario.hpp"

#include <cmath>
#include <vector>

ScalableMultiDropScenario::ScalableMultiDropScenario(float xScale, float yScale, double end)
    : endTime(end), xScale(xScale), yScale(yScale)
{
}

float ScalableMultiDropScenario::getBathymetry(float x, float y) const { return 0.0f; }

float ScalableMultiDropScenario::getWaterHeight(float x, float y) const
{
    constexpr float radius = 10.0f;
    constexpr float inverse_radius = 90.0f;
    constexpr float full = 100.0f;

    float cirX = radius * xScale;
    float cirY = radius * yScale;
    float totX = full * xScale;
    float totY = full * yScale;

    float xMid = 50.0f * xScale;
    float yMid = 50.0f * yScale;

    bool withincenter = (std::abs(x - xMid) < cirX) && (std::abs(y - yMid) < cirY);

    // quarter circle starting at 0-0 15% of the whole map
    float xRange = radius * xScale;
    float yRange = radius * yScale;
    bool within00 = (x <= xRange && y <= yRange && ((x / xRange) * (x / xRange) + (y / yRange) * (y / yRange)) <= 1);

    // 1-0
    xRange = inverse_radius * xScale;
    yRange = radius * yScale;
    bool within10 = (x >= xRange && y <= yRange &&
                     ((((totX - x) / cirX) * ((totX - x) / cirX)) + ((y / yRange) * (y / yRange))) <= 1);

    // 0-1
    xRange = radius * xScale;
    yRange = inverse_radius * yScale;
    bool within01 = (x <= xRange && y >= yRange &&
                     (((x / xRange) * (x / xRange)) + (((totY - y) / (cirY))) * ((totY - y) / (cirY))) <= 1);

    // 1-1
    xRange = radius * xScale;
    yRange = radius * yScale;
    bool within11 =
        (x >= xRange && y >= yRange &&
         (((totX - x) / cirX) * ((totX - x) / cirX) + (((totY - y) / (cirY))) * ((totY - y) / (cirY))) <= 1);

    return (within00 || within10 || within01 || within11 || withincenter) ? 15.0f : 10.0f;
}

float ScalableMultiDropScenario::defWaterHeight() const { return 10.0f; }

float ScalableMultiDropScenario::endSimulation() const { return this->endTime; }

float ScalableMultiDropScenario::getBoundaryPos(BoundaryEdge e) const
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

BoundaryType ScalableMultiDropScenario::getBoundaryType(BoundaryEdge e) const { return BoundaryType::WALL; }
