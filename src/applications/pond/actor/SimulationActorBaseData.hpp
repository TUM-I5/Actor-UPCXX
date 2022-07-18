#pragma once
#include "block/BlockCommunicator.hpp"
#include "block/SWE_Block.hh"
#include "block/SWE_WaveAccumulationBlock.hh"
#include "writer/Writer.hh"

#ifdef WRITENETCDF
#include "writer/NetCdfWriter.hh"
#else
#include "writer/VtkWriter.hh"
#endif

#include "SimulationActorState.hpp"
#include "actorlib/ActorData.hpp"

class InPortInformationBase;
class OutPortInformationBase;

#include "actorlib/PortInformationBase.hpp"
#include <memory>
#include <vector>

class SimulationActorBaseData
{
  public:
    ActorData ad;
    size_t position[2];
    std::unique_ptr<SWE_WaveAccumulationBlock> block;
    SimulationActorState currentState;
    float currentTime;
    float timestepBaseline;
    float outputDelta;
    float nextWriteTime;
    float endTime;
    uint64_t patchUpdates;
    SimulationArea sa;

#if defined(WRITENETCDF)
    std::unique_ptr<io::NetCdfWriter> writer;
#elif defined(WRITEVTK)
    std::unique_ptr<io::VtkWriter> writer;
#else
    void *writer;
#endif

    bool actv;

#ifdef TIME
    size_t running;
    size_t finishing;
    size_t terminated;

    size_t timeSpentReading;
    size_t timeSpentWriting;
    size_t timeSpentReceiving;
    size_t timeSpentNotReceiving;
    size_t timeSpentComputing;
    size_t timesWrittenToFile;
#endif

    bool slow;

    std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
               std::vector<std::unique_ptr<OutPortInformationBase>>>
        tup;

    SimulationActorBaseData(SimulationActorBaseData &&other)
        : ad(std::move(other.ad)), position{other.position[0], other.position[1]}, block(std::move(other.block)),
          currentState(other.currentState), currentTime(other.currentTime), timestepBaseline(other.timestepBaseline),
          outputDelta(other.outputDelta), nextWriteTime(other.nextWriteTime), endTime(other.endTime),
          patchUpdates(other.patchUpdates), sa(other.sa),
#if defined(WRITENETCDF) || defined(WRITEVTK)
          writer(std::move(other.writer)),
#endif
          actv(std::move(other.actv)),
#ifdef TIME
          running(other.running), finishing(other.finishing), terminated(other.terminated),
          timeSpentReading(other.timeSpentReading), timeSpentWriting(other.timeSpentWriting),
          timeSpentReceiving(other.timeSpentReceiving), timeSpentNotReceiving(other.timeSpentNotReceiving),
          timeSpentComputing(other.timeSpentComputing), timesWrittenToFile(other.timesWrittenToFile),
#endif
          slow(other.slow), tup(std::move(other.tup))
    {
#if defined(WRITENETCDF) || defined(WRITEVTK)
#else
        other.writer = nullptr;
#endif
    }

    SimulationActorBaseData(ActorData &&ad, size_t position[2], std::unique_ptr<SWE_WaveAccumulationBlock> &&block,
                            SimulationActorState currentState, float currentTime, float timestepBaseline,
                            float outputDelta, float nextWriteTime, float endTime, uint64_t patchUpdates,
                            SimulationArea sa,

#if defined(WRITENETCDF)
                            std::unique_ptr<io::NetCdfWriter> &&writer,
#elif defined(WRITEVTK)
                            std::unique_ptr<io::VtkWriter> &&writer,
#else
                            void *writer,
#endif
                            bool actv,
#ifdef TIME
                            size_t running, size_t finishing, size_t terminated, size_t timeSpentReading,
                            size_t timeSpentWriting, size_t timeSpentReceiving, size_t timeSpentNotReceiving,
                            size_t timeSpentComputing, size_t timesWrittenToFile,
#endif
                            bool slow,
                            std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                       std::vector<std::unique_ptr<OutPortInformationBase>>> &&tup)
        : ad(std::move(ad)), position{position[0], position[1]}, block(std::move(block)), currentState(currentState),
          currentTime(currentTime), timestepBaseline(timestepBaseline), outputDelta(outputDelta),
          nextWriteTime(nextWriteTime), endTime(endTime), patchUpdates(patchUpdates), sa(sa),
#if defined(WRITENETCDF)
          writer(std::move(writer)),
#elif defined(WRITEVTK)
          writer(std::move(writer)),
#else
          writer(writer),
#endif
          actv(actv),
#ifdef TIME
          running(running), finishing(finishing), terminated(terminated), timeSpentReading(timeSpentReading),
          timeSpentWriting(timeSpentWriting), timeSpentReceiving(timeSpentReceiving),
          timeSpentNotReceiving(timeSpentNotReceiving), timeSpentComputing(timeSpentComputing),
          timesWrittenToFile(timesWrittenToFile),
#endif
          slow(slow), tup(std::move(tup))
    {
    }

    ~SimulationActorBaseData(){};

    UPCXX_SERIALIZED_DELETE()
};