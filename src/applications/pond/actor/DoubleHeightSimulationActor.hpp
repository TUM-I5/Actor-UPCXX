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
#pragma once
#include "SimulationActor.hpp"
#include "SimulationActorBase.hpp"
#include "SimulationActorBaseData.hpp"
#include "SimulationActorState.hpp"
#include "actorlib/InPort.hpp"
#include "actorlib/OutPort.hpp"
#include <upcxx/upcxx.hpp>

enum class LocalPortEnum
{
    BND_TOP_LEFT,
    BND_TOP_RIGHT,
    BND_BOTTOM_LEFT,
    BND_BOTTOM_RIGHT,
    BND_LEFT,
    BND_RIGHT
};

class DoubleHeightSimulationActor final : public SimulationActorBase
{
  private:
    InPort<std::vector<float>, 64> *dataIn[6];   // inports
    OutPort<std::vector<float>, 64> *dataOut[6]; // outports

  public:
    DoubleHeightSimulationActor(std::string &&name, size_t xPos, size_t yPos);
    DoubleHeightSimulationActor(const std::string &name, size_t xPos, size_t yPos);

    // reconstructor with all the actors
    DoubleHeightSimulationActor(ActorData &&ad,
                                std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                           std::vector<std::unique_ptr<OutPortInformationBase>>> &&tup,
                                size_t xPos, size_t yPos, std::unique_ptr<SWE_WaveAccumulationBlock> &&block,
                                SimulationActorState currentState, float currentTime, float timestepBaseline,
                                float outputdelta, float nextWriteTime, float endTime, uint64_t patchUpdates,
#if defined(WRITENETCDF)
                                std::unique_ptr<io::NetCdfWriter> &&writer,
#elif defined(WRITEVTK)
                                std::unique_ptr<io::VtkWriter> &&writer,
#endif
                                SimulationArea &&sa, bool actv);

    /*
        DoubleHeightSimulationActor(ActorData &&ad, size_t xPos, size_t yPos,
                                    std::unique_ptr<SWE_WaveAccumulationBlock> &&block, SimulationActorState
    currentState, float currentTime, float timestepBaseline, float outputdelta, float nextWriteTime, float endTime,
    uint64_t patchUpdates, #if defined(WRITENETCDF) std::unique_ptr<io::NetCdfWriter> &&writer, #elif defined(WRITEVTK)
                                    std::unique_ptr<io::VtkWriter> &&writer,
    #endif
                                    SimulationArea &&sa, bool actv) ;
                                    */

    // DoubleHeightSimulationActor(const DoubleHeightSimulationActor &sa) ;
    DoubleHeightSimulationActor(DoubleHeightSimulationActor &&sa);
    DoubleHeightSimulationActor(SimulationActorBaseData &&sabd);
    DoubleHeightSimulationActor &operator=(const DoubleHeightSimulationActor &sa) = delete;
    DoubleHeightSimulationActor &operator=(DoubleHeightSimulationActor &&sa) = delete;
    ~DoubleHeightSimulationActor() override final{
        /*
        for (auto * ptr : dataIn)
        {
          delete ptr;
        }
        for(auto * ptr : dataOut)
        {
          delete ptr;
        }
        */
    };

    void initializeBlock() override final;
    void reInitializeBlock() override final;
    void initializeBoundary(BoundaryEdge edge, std::function<bool()>) override final;
    void sendData() override final;
    void receiveData() override final;
    void sendTerminationSignal() override final;
    bool hasReceivedTerminationSignal() override final;
    bool mayWrite() override final;
    bool mayRead() override final;
    void makePorts() override final;
    bool noChange() override final;

  public:
    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, const DoubleHeightSimulationActor &object)
        {
            writer.write(dynamic_cast<SimulationActorBase const &>(object));
        }
        template <typename Reader> static DoubleHeightSimulationActor *deserialize(Reader &reader, void *storage)
        {
            SimulationActorBaseData sabd = reader.template read<SimulationActorBase>();
            DoubleHeightSimulationActor *dhsa = ::new (storage) DoubleHeightSimulationActor(std::move(sabd));
            return dhsa;
        }
    };
};
