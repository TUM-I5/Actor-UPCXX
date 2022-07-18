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

#ifdef WRITENETCDF

#include "scenario/NetCdfScenario.hpp"
#include "scenario/NetCdfData.hpp"

#include <upcxx/upcxx.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include <sstream>
#include <vector>

inline float bound(float ratio) { return std::max(0.0f, std::min(1.0f, ratio)); }

inline float weightedAverage(float a, float b, float w) { return w * a + (1 - w) * b; }

inline float distanceSquared(float x1, float y1, float x2, float y2)
{
    auto xDist = x1 - x2;
    auto yDist = y1 - y2;
    return xDist * xDist + yDist * yDist;
}

NetCdfScenario::NetCdfScenario(std::string bathymetryFilename, double displacementX, double displacementY,
                               double displacementR, double end)
    : endTime(end), simulationArea(0.0f, 0.0f, 0.0f, 0.0f), displacementX(displacementX), displacementY(displacementY),
      displacementR(displacementR)
{
    upcxx::dist_object<NetCdfScenario *> distScenario(this);
    if (!upcxx::rank_me())
    {
        NetCdfData bathyData(bathymetryFilename);
        this->bathymetryData = bathyData.exportData();
        this->bathymetryXSize = bathyData.getXSize();
        this->bathymetryYSize = bathyData.getYSize();
        simulationArea = bathyData.getSimulationArea();
        upcxx::promise<> allDone;
        auto bathyView = upcxx::make_view(this->bathymetryData);
        for (int i = 0; i < upcxx::rank_n(); i++)
        {
            upcxx::rpc(
                i, upcxx::operation_cx::as_promise(allDone),
                [](upcxx::view<float> bathyView, SimulationArea s, size_t bxs, size_t bys,
                   upcxx::dist_object<NetCdfScenario *> &rThis)
                {
                    std::vector<float> &rBathymetryData = (*rThis)->bathymetryData;
                    std::copy(bathyView.begin(), bathyView.end(), back_inserter(rBathymetryData));
                    (*rThis)->simulationArea = s;
                    (*rThis)->bathymetryXSize = bxs;
                    (*rThis)->bathymetryYSize = bys;
                },
                bathyView, simulationArea, bathymetryXSize, bathymetryYSize, distScenario);
        }
        allDone.finalize().wait();
    }

    upcxx::barrier();
}

std::string NetCdfScenario::debugPrint()
{
    std::stringstream ss;
    for (size_t y = 0; y < this->bathymetryYSize; y += 5)
    {
        for (size_t x = 0; x < this->bathymetryXSize; x += 5)
        {
            auto b = bathymetryData[y * this->bathymetryXSize + x];
            ss << ((b < -1000.0f) ? '-' : (b < 0.0f) ? '~' : (b < 1000.0f) ? '#' : '*');
        }
        ss << std::endl;
    }
    return ss.str();
}

float NetCdfScenario::getResultData(const SimulationArea &targetArea, float x, float y,
                                    const std::vector<float> &targetData) const
{
    auto regXSize = targetArea.maxX - targetArea.minX;
    auto regYSize = targetArea.maxY - targetArea.minY;
    auto targetXRatio = bound((x - targetArea.minX) / regXSize);
    auto targetYRatio = bound((y - targetArea.minY) / regYSize);
    auto targetXCoordinate = bathymetryXSize * targetXRatio;
    auto targetYCoordinate = bathymetryYSize * targetYRatio;

    auto leftXArrayIdx = static_cast<size_t>(std::floor(targetXCoordinate));
    auto rightXArrayIdx = static_cast<size_t>(std::ceil(targetXCoordinate));
    auto lowerYArrayIdx = static_cast<size_t>(std::floor(targetYCoordinate));
    auto upperYArrayIdx = static_cast<size_t>(std::ceil(targetYCoordinate));

    // leftXArrayIdx += (leftXArrayIdx < 0) ? 1 : 0;
    // rightXArrayIdx += (rightXArrayIdx < 0) ? 1 : 0;
    leftXArrayIdx += (leftXArrayIdx >= bathymetryXSize) ? -1 : 0;
    rightXArrayIdx += (rightXArrayIdx >= bathymetryXSize) ? -1 : 0;
    // lowerYArrayIdx += (lowerYArrayIdx < 0) ? 1 : 0;
    // upperYArrayIdx += (upperYArrayIdx < 0) ? 1 : 0;
    lowerYArrayIdx += (lowerYArrayIdx >= bathymetryYSize) ? -1 : 0;
    upperYArrayIdx += (upperYArrayIdx >= bathymetryYSize) ? -1 : 0;

    assert(leftXArrayIdx < bathymetryXSize);
    assert(rightXArrayIdx < bathymetryXSize);
    assert(lowerYArrayIdx < bathymetryYSize);
    assert(upperYArrayIdx < bathymetryYSize);

    auto xRemainder = targetXCoordinate - static_cast<float>(leftXArrayIdx);
    auto yRemainder = targetYCoordinate - static_cast<float>(lowerYArrayIdx);

    auto tl = targetData[upperYArrayIdx * bathymetryXSize + leftXArrayIdx];
    auto tr = targetData[upperYArrayIdx * bathymetryXSize + rightXArrayIdx];
    auto bl = targetData[lowerYArrayIdx * bathymetryXSize + leftXArrayIdx];
    auto br = targetData[lowerYArrayIdx * bathymetryXSize + rightXArrayIdx];

    auto avgLeft = weightedAverage(tl, bl, yRemainder);
    auto avgRight = weightedAverage(tr, br, yRemainder);
    auto avg = weightedAverage(avgLeft, avgRight, xRemainder);
    return avg;
}

float NetCdfScenario::getBathymetry(float x, float y) const
{
    return getResultData(simulationArea, x, y, bathymetryData);
}

float NetCdfScenario::getWaterHeight(float x, float y) const
{
    auto bathymetry = getResultData(simulationArea, x, y, bathymetryData);
    auto displacement =
        (distanceSquared(x, y, displacementX, displacementY) < (displacementR * displacementR)) ? 50.f : 0.0f;
    auto stillWaterHeight = std::max(-bathymetry, 0.0f);
    if (bathymetry >= 0.0f && displacement > 0.0f)
    {
        throw std::runtime_error("displacement may not be on land");
    }
    return stillWaterHeight + displacement;
}

float NetCdfScenario::defWaterHeight() const { return 0.0f; }

BoundaryType NetCdfScenario::getBoundaryType(BoundaryEdge e) const { return BoundaryType::OUTFLOW; }

float NetCdfScenario::getBoundaryPos(BoundaryEdge e) const
{
    switch (e)
    {
    case BoundaryEdge::BND_LEFT:
        return simulationArea.minX;
    case BoundaryEdge::BND_RIGHT:
        return simulationArea.maxX;
    case BoundaryEdge::BND_BOTTOM:
        return simulationArea.minY;
    case BoundaryEdge::BND_TOP:
        return simulationArea.maxY;
    default:
        return FP_NAN;
    }
}

float NetCdfScenario::endSimulation() const { return endTime; }

#endif // WRITENETCDF
