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

#include <cstddef>
#include <memory>
#include <vector>

#pragma once

class SWE_Block1D;

class BlockCommunicator
{
  public:
    std::vector<float> sendBuffer;
    std::unique_ptr<SWE_Block1D> copyLayer;
    std::unique_ptr<SWE_Block1D> ghostLayer;
    size_t patchSize;

    BlockCommunicator();
    BlockCommunicator(size_t patchSize, std::unique_ptr<SWE_Block1D> &&copyLayer,
                      std::unique_ptr<SWE_Block1D> &&ghostLayer);
    ~BlockCommunicator(); // no need to delete SWE_Block1D

    BlockCommunicator &operator=(BlockCommunicator &&other);

    std::vector<float> packCopyLayer();
    void receiveGhostLayer(std::vector<float> &&ghostLayerBuffer);
};
