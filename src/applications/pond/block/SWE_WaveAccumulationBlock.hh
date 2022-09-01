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

#ifndef SWE_WAVEACCUMULATION_BLOCK_HH_
#define SWE_WAVEACCUMULATION_BLOCK_HH_

#include "block/SWE_Block.hh"
#include "util/help.hh"

#include <string>

// *** SWE_WaveAccumulationBlock only supports the following wave propagation
// solvers:
//  2: Approximate Augmented Riemann solver (functional implementation:
//  AugRieFun) 4: f-Wave (vectorized implementation: FWaveVec)
#if WAVE_PROPAGATION_SOLVER == 2
#include "solver/AugRieFun.hpp"
#elif WAVE_PROPAGATION_SOLVER == 3
#include "solver/HLLEFun.hpp"
#else
#include "solver/FWaveVec.hpp"
#endif

/**
 * SWE_WaveAccumulationBlock is an implementation of the SWE_Block abstract
 * class. It uses a wave propagation solver which is defined with the
 * pre-compiler flag WAVE_PROPAGATION_SOLVER (see above).
 *
 * Possible wave propagation solvers are:
 *  F-Wave, Apprximate Augmented Riemann, Hybrid (f-wave + augmented).
 *  (details can be found in the corresponding source files)
 */
class SWE_WaveAccumulationBlock : public SWE_Block
{

#if WAVE_PROPAGATION_SOLVER == 2
    //! Approximate Augmented Riemann solver
    solver::AugRieFun<float> wavePropagationSolver;
#elif WAVE_PROPAGATION_SOLVER == 3
    //! Vectorized HLLE solver
    solver::HLLEFun<float> wavePropagationSolver;
#else
    //! Vectorized FWave solver
    solver::FWaveVec<float> wavePropagationSolver;
#endif

    //! net-updates for the heights of the cells (for accumulation)
    Float2D hNetUpdates;

    //! net-updates for the x-momentums of the cells (for accumulation)
    Float2D huNetUpdates;

    //! net-updates for the y-momentums of the cells (for accumulation)
    Float2D hvNetUpdates;

  public:
    // constructor of a SWE_WaveAccumulationBlock.
    SWE_WaveAccumulationBlock(int l_nx, int l_ny, float l_dx, float l_dy);
    // destructor of a SWE_WaveAccumulationBlock.
    SWE_WaveAccumulationBlock(SWEBlockData &&bd, Float2D &&hNetUpdates, Float2D &&huNetUpdates, Float2D &&hvNetUpdates);
    SWE_WaveAccumulationBlock(SWE_WaveAccumulationBlock &&other);
    SWE_WaveAccumulationBlock(const SWE_WaveAccumulationBlock &other);
    ~SWE_WaveAccumulationBlock(){};

    SWE_Block *clone() const final override;

    // computes the net-updates for the block
    void computeNumericalFluxes() final override;

    // update the cells
    void updateUnknowns(float dt) final override;

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, const SWE_WaveAccumulationBlock &object)
        {
            writer.write(dynamic_cast<SWE_Block const &>(object));
            writer.write(object.hNetUpdates);
            writer.write(object.huNetUpdates);
            writer.write(object.hvNetUpdates);

            return;
        }

        template <typename Reader> static SWE_WaveAccumulationBlock *deserialize(Reader &reader, void *storage)
        {
            SWEBlockData bd = reader.template read<SWE_Block>();
            Float2D hNetUpdates = reader.template read<Float2D>();
            Float2D huNetUpdates = reader.template read<Float2D>();
            Float2D hvNetUpdates = reader.template read<Float2D>();

            SWE_WaveAccumulationBlock *wab = ::new (storage) SWE_WaveAccumulationBlock(
                std::move(bd), std::move(hNetUpdates), std::move(huNetUpdates), std::move(hvNetUpdates));

            return wab;
        }
    };
};

#endif /* SWE_WAVEACCUMULATION_BLOCK_HH_ */
