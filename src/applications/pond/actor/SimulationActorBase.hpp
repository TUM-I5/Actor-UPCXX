#include "SimulationActorState.hpp"

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
#include <upcxx/upcxx.hpp>

#include "block/BlockCommunicator.hpp"
#include "block/SWE_Block.hh"
#include "block/SWE_WaveAccumulationBlock.hh"
#include "util/Configuration.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <sstream>
#include <tuple>
#include <vector>

#include "block/SWE_Block.hh"
#include "scenario/SWE_Scenario.hh"
#include "scenario/SimulationArea.hpp"
#include "util/Configuration.hpp"
#include "writer/NetCdfWriter.hh"
#include "writer/VtkWriter.hh"
#include "writer/Writer.hh"

#ifdef WRITENETCDF
#include "writer/NetCdfWriter.hh"
#elif WRITEVTK
#include "writer/VtkWriter.hh"
#endif

#include "SimulationActorBaseData.hpp"
#include "SimulationActorState.hpp"
#include "actorlib/ActorImpl.hpp"
#include "actorlib/InPort.hpp"
#include "actorlib/OutPort.hpp"
#include "actorlib/PortInfoConvey.hpp"
#include "actorlib/PortInformation.hpp"
#include "util/Logger.hh"
#include <limits>

namespace io
{
class Writer;
class VtkWriter;
class NetCdfWriter;
class BoundarySize;
} // namespace io

class SimulationActorBase : public ActorImpl
{

  public:
    static const Configuration *configuration; // configuration used to configure the actors, gloval

  protected:
    bool slow;
    size_t position[2];                               // positions in the gitter
    std::unique_ptr<SWE_WaveAccumulationBlock> block; // done
    BlockCommunicator communicators[4];               // communicates with other actors (send boundaries)
    SimulationActorState currentState;                // state of the actor
    float currentTime;                                // current time in simulation
    float timestepBaseline;                           // +=time per iteration
    float outputDelta;                                // time interval to write to netcdf file (or vtk)
    float nextWriteTime;                              // next time to write
    float endTime;                                    // time to end the simulation
    uint64_t patchUpdates;                            // number of total patch updates

  public:
    SimulationArea patchArea; // patch area of this actor

  protected:
#if defined(WRITENETCDF)
    mutable std::unique_ptr<io::NetCdfWriter> writer; // recons//netcdf
#elif defined(WRITEVTK)
    mutable std::unique_ptr<io::VtkWriter> writer;
#else
    // nothing
#endif
    bool active; // set when initializing blocks, if the patch has displacement := true
    // std::vector<InPort<std::vector<float>, 64> *> activeNeighbors;

#ifdef TIME
  protected:
    size_t timeSpentRunning = 0;
    size_t timeSpentFinishing = 0;
    size_t timeSpentTerminated = 0;
    size_t timeSpentReading = 0;
    size_t timeSpentWriting = 0;
    size_t timeSpentReceiving = 0;
    size_t timeSpentNotReceiving = 0;
    size_t timeSpentComputing = 0;
    size_t timesWrittenToFile = 0;
#endif

  public:
    SimulationActorBase();
    SimulationActorBase(std::string &&name, std::tuple<size_t, size_t> &&coordinates, bool slow = false);
    SimulationActorBase(const std::string &name, const std::tuple<size_t, size_t> &coordinates, bool slow = false);

    // reconstructor with all the actors
    SimulationActorBase(ActorData &&ad,
                        std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                   std::vector<std::unique_ptr<OutPortInformationBase>>> &&tup,
                        bool slow, size_t xPos, size_t yPos, std::unique_ptr<SWE_WaveAccumulationBlock> &&block,
                        SimulationActorState currentState, float currentTime, float timestepBaseline, float outputdelta,
                        float nextWriteTime, float endTime, uint64_t patchUpdates,
#if defined(WRITENETCDF)
                        std::unique_ptr<io::NetCdfWriter> &&writer,
#elif defined(WRITEVTK)
                        std::unique_ptr<io::VtkWriter> &&writer,
#endif
                        SimulationArea &&sa, bool actv);

    // SimulationActorBase(const SimulationActorBase &sa) ;
    SimulationActorBase(SimulationActorBase &&sa);
    SimulationActorBase(SimulationActorBaseData &&sabd);
    SimulationActorBase &operator=(const SimulationActorBase &sa) = delete;
    SimulationActorBase &operator=(SimulationActorBase &&sa);
    virtual ~SimulationActorBase();

    virtual void initializeBlock() = 0;
    virtual void reInitializeBlock() = 0;
    float getMaxBlockTimestepSize();
    void setTimestepBaseline(float timestepBaseline);
    virtual void initializeBoundary(BoundaryEdge edge, std::function<bool()>) = 0;
    void clearActivation();
    uint64_t getNumberOfPatchUpdates() const;
    void prepareDerived();
    std::string getTimeInfo();
    std::string strConfig() const;
    bool getActive() const;
    void act();
    bool checkTerminated() const override final;

  protected:
    void computeWriteDelta();
    void performComputationStep();
    virtual void sendData() = 0;
    virtual void receiveData() = 0;
    virtual void sendTerminationSignal() = 0;
    virtual bool hasReceivedTerminationSignal() = 0;
    virtual bool mayWrite() = 0;
    virtual bool mayRead() = 0;
    bool firstBatch();
    void writeTimeStep(float currentTime);
    virtual void makePorts() = 0; // only after the init list!
    bool diffThanInitial(const std::vector<float> &data);
    std::vector<float> createDefaultData();
    virtual bool noChange() = 0;

  public:
    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, const SimulationActorBase &object)
        {
            writer.write(dynamic_cast<ActorImpl const &>(object));
            for (int i = 0; i < 2; i++)
                writer.write(object.position[i]);
            writer.write(*object.block.get());
            writer.write(object.currentState);
            writer.write(object.currentTime);
            writer.write(object.timestepBaseline);
            writer.write(object.outputDelta);
            writer.write(object.nextWriteTime);
            writer.write(object.endTime);
            writer.write(object.patchUpdates);
            writer.write(object.patchArea);
#if defined(WRITEVTK) || defined(WRITENETCDF)
            writer.write(*object.writer.get());
#endif
            writer.write(object.active);

#ifdef TIME
            writer.write(object.timeSpentRunning);
            writer.write(object.timeSpentFinishing);
            writer.write(object.timeSpentTerminated);

            writer.write(object.timeSpentReading);
            writer.write(object.timeSpentWriting);
            writer.write(object.timeSpentReceiving);
            writer.write(object.timeSpentNotReceiving);
            writer.write(object.timeSpentComputing);
            writer.write(object.timesWrittenToFile);
#endif

            writer.write(object.slow);

            if (object.portsInfoBadState())
            {
                throw std::runtime_error("ports info should be generated and "
                                         "assigned in preparation phase");
            }

            PortsInformationWriter<std::vector<float>, 64> piw(object.moveTuple());
            piw.writeData<Writer>(writer);

            return;
        }

        template <typename Reader> static SimulationActorBaseData *deserialize(Reader &reader, void *storage)
        {
            // auto adstore = new typename std::aligned_storage<sizeof(ActorData), alignof(ActorData)>::type;
            // reader.template read_into<ActorImpl>(adstore);
            // ActorData *ad = reinterpret_cast<ActorData *>(adstore);

            ActorData ad = reader.template read<ActorImpl>();

            size_t position[2];
            for (int i = 0; i < 2; i++)
                position[i] = reader.template read<size_t>();
            // SWE_WaveAccumulationBlock block = reader.template read<SWE_WaveAccumulationBlock>();
            std::unique_ptr<SWE_WaveAccumulationBlock> block =
                util::read_unique<Reader, SWE_WaveAccumulationBlock>(reader);

            SimulationActorState currentState = reader.template read<SimulationActorState>();
            float currentTime = reader.template read<float>();      //""
            float timestepBaseline = reader.template read<float>(); //""
            float outputDelta = reader.template read<float>();      //""
            float nextWriteTime = reader.template read<float>();    //""
            float endTime = reader.template read<float>();          //""
            uint64_t patchUpdates = reader.template read<uint64_t>();
            SimulationArea sa = reader.template read<SimulationArea>();
#if defined(WRITENETCDF)
            std::unique_ptr<io::NetCdfWriter> writer = util::read_unique<Reader, io::NetCdfWriter>(reader);
#elif defined(WRITEVTK)
            std::unique_ptr<io::VtkWriter> writer = util::read_unique<Reader, io::VtkWriter>(reader);
#else
            void *writer = nullptr;
#endif
            bool actv = reader.template read<bool>();
            // std::vector<std::string> v = reader.template read<std::vector<std::string>>();

#ifdef TIME
            size_t running = reader.template read<size_t>();
            size_t finishing = reader.template read<size_t>();
            size_t terminated = reader.template read<size_t>();

            size_t timeSpentReading = reader.template read<size_t>();
            size_t timeSpentWriting = reader.template read<size_t>();
            size_t timeSpentReceiving = reader.template read<size_t>();
            size_t timeSpentNotReceiving = reader.template read<size_t>();
            size_t timeSpentComputing = reader.template read<size_t>();
            size_t timesWrittenToFile = reader.template read<size_t>();
#endif

            bool slow = reader.template read<bool>();

            PortsInformationReader<std::vector<float>, 64> pir;
            std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                       std::vector<std::unique_ptr<OutPortInformationBase>>>
                tup(pir.readData<Reader>(reader));

            auto sabd = ::new (storage) SimulationActorBaseData(
                std::move(ad), position, std::move(block), currentState, currentTime, timestepBaseline, outputDelta,
                nextWriteTime, endTime, patchUpdates, sa,
#if defined(WRITENETCDF)
                std::move(writer),
#elif defined(WRITEVTK)
                std::move(writer),
#else
      writer),
#endif
                actv,
#ifdef TIME

                running, finishing, terminated, timeSpentReading, timeSpentWriting, timeSpentReceiving,
                timeSpentNotReceiving, timeSpentComputing, timesWrittenToFile,
#endif
                slow, std::move(tup));

            return sabd;
        }
    };
};