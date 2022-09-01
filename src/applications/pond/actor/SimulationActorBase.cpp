#include "SimulationActorBase.hpp"

const Configuration *SimulationActorBase::configuration = nullptr;
static tools::Logger &l = tools::Logger::logger;
constexpr io::BoundarySize ee = {1, 1, 1, 1};

SimulationActorBase::SimulationActorBase(std::string &&name, std::tuple<size_t, size_t> &&coordinates, bool slow)
    : ActorImpl(std::move(name)), slow(slow), position{std::get<0>(coordinates), std::get<1>(coordinates)},
      block(std::make_unique<SWE_WaveAccumulationBlock>(
          SimulationActorBase::configuration->patchSize, SimulationActorBase::configuration->patchSize,
          SimulationActorBase::configuration->dx, SimulationActorBase::configuration->dy)),
      currentState(SimulationActorState::INITIAL), currentTime(0.0f), timestepBaseline(0.0f), outputDelta(0.0f),
      nextWriteTime(0.0f), endTime(SimulationActorBase::configuration->scenario->endSimulation()), patchUpdates(0),
      patchArea(makePatchArea(SimulationActorBase::configuration, position[0], position[1])),
#if defined(WRITENETCDF)
      writer(std::make_unique<io::NetCdfWriter>(
          SimulationActorBase::configuration->fileNameBase, ee, patchArea,
          SimulationActorBase::configuration->scenario->endSimulation() /
              SimulationActorBase::configuration->numberOfCheckpoints,
          SimulationActorBase::configuration->patchSize, SimulationActorBase::configuration->patchSize,
          SimulationActorBase::configuration->dx, SimulationActorBase::configuration->dy, patchArea.minX,
          patchArea.minY, 1)),
#elif defined(WRITEVTK)
      writer(std::make_unique<io::VtkWriter>(
          configuration->fileNameBase, ee, SimulationActorBase::configuration->patchSize,
          SimulationActorBase::configuration->patchSize, SimulationActorBase::configuration->dx,
          SimulationActorBase::configuration->dy, position[0] * SimulationActorBase::configuration->patchSize,
          position[1] * SimulationActorBase::configuration->patchSize)),
#endif
      active(false) //, activeNeighbors()
{
}

SimulationActorBase::SimulationActorBase(const std::string &name, const std::tuple<size_t, size_t> &coordinates,
                                         bool slow)
    : ActorImpl(name), slow(slow), position{std::get<0>(coordinates), std::get<1>(coordinates)},
      block(std::make_unique<SWE_WaveAccumulationBlock>(
          SimulationActorBase::configuration->patchSize, SimulationActorBase::configuration->patchSize,
          SimulationActorBase::configuration->dx, SimulationActorBase::configuration->dy)),
      currentState(SimulationActorState::INITIAL), currentTime(0.0f), timestepBaseline(0.0f), outputDelta(0.0f),
      nextWriteTime(0.0f), endTime(SimulationActorBase::configuration->scenario->endSimulation()), patchUpdates(0),
      patchArea(makePatchArea(SimulationActorBase::configuration, position[0], position[1])),
#if defined(WRITENETCDF)
      writer(std::make_unique<io::NetCdfWriter>(
          SimulationActorBase::configuration->fileNameBase, ee, patchArea,
          SimulationActorBase::configuration->scenario->endSimulation() /
              SimulationActorBase::configuration->numberOfCheckpoints,
          SimulationActorBase::configuration->patchSize, SimulationActorBase::configuration->patchSize,
          SimulationActorBase::configuration->dx, SimulationActorBase::configuration->dy, patchArea.minX,
          patchArea.minY, 1)),
#elif defined(WRITEVTK)
      writer(std::make_unique<io::VtkWriter>(
          SimulationActorBase::configuration->fileNameBase, ee, SimulationActorBase::configuration->patchSize,
          SimulationActorBase::configuration->patchSize, SimulationActorBase::configuration->dx,
          SimulationActorBase::configuration->dy, position[0] * SimulationActorBase::configuration->patchSize,
          position[1] * SimulationActorBase::configuration->patchSize)),
#else
// do nothing
#endif
      active(false) //, activeNeighbors()
{
}

SimulationActorBase::SimulationActorBase(ActorData &&ad,
                                         std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                                    std::vector<std::unique_ptr<OutPortInformationBase>>> &&tup,
                                         bool slow, size_t xPos, size_t yPos,
                                         std::unique_ptr<SWE_WaveAccumulationBlock> &&block,
                                         SimulationActorState currentState, float currentTime, float timestepBaseline,
                                         float outputDelta, float nextWriteTime, float endTime, uint64_t patchUpdates,
#if defined(WRITENETCDF)
                                         std::unique_ptr<io::NetCdfWriter> &&writer,
#elif defined(WRITEVTK)
                                         std::unique_ptr<io::VtkWriter> &&writer,
#endif
                                         SimulationArea &&sa, bool actv)
    : ActorImpl(std::move(ad), std::move(tup)), slow(slow), position{xPos, yPos}, block(std::move(block)),
      currentState(currentState), currentTime(currentTime), timestepBaseline(timestepBaseline),
      outputDelta(outputDelta), nextWriteTime(nextWriteTime), endTime(endTime), patchUpdates(patchUpdates),
      patchArea(std::move(sa)),
#if defined(WRITENETCDF) || defined(WRITEVTK)
      writer(std::move(writer)),
#endif
      active(actv) //, activeNeighbors()
{
    if (SimulationActorBase::configuration->scenario->endSimulation() != endTime)
    {
        throw std::runtime_error("Endtime should not change during moves or migration");
    }
}

SimulationActorBase::SimulationActorBase(SimulationActorBaseData &&sabd)
    : ActorImpl(std::move(sabd.ad), std::move(sabd.tup)), slow(sabd.slow), position{sabd.position[0], sabd.position[1]},
      block(std::move(sabd.block)), currentState(sabd.currentState), currentTime(sabd.currentTime),
      timestepBaseline(sabd.timestepBaseline), outputDelta(sabd.outputDelta), nextWriteTime(sabd.nextWriteTime),
      endTime(sabd.endTime), patchUpdates(sabd.patchUpdates), patchArea(std::move(sabd.sa)),
#if defined(WRITENETCDF) || defined(WRITEVTK)
      writer(std::move(sabd.writer)),
#endif
      active(sabd.actv)
{
#ifdef TIME
    timeSpentFinishing = sabd.finishing;
    timeSpentRunning = sabd.running;
    timeSpentTerminated = sabd.terminated;
    timeSpentReading = sabd.timeSpentReading;
    timeSpentWriting = sabd.timeSpentWriting;
    timeSpentReceiving = sabd.timeSpentReceiving;
    timeSpentNotReceiving = sabd.timeSpentNotReceiving;
    timeSpentComputing = sabd.timeSpentComputing;
    timesWrittenToFile = sabd.timesWrittenToFile;
#endif
    if (SimulationActorBase::configuration->scenario->endSimulation() != endTime)
    {
        throw std::runtime_error("Endtime should not change during moves or migration");
    }
}

SimulationActorBase::SimulationActorBase(SimulationActorBase &&sa)
    : ActorImpl(dynamic_cast<ActorImpl &&>(std::move(sa))), slow(sa.slow), position{sa.position[0], sa.position[1]},
      block(std::move(sa.block)), currentState(sa.currentState), currentTime(sa.currentTime),
      timestepBaseline(sa.timestepBaseline), outputDelta(sa.outputDelta), nextWriteTime(sa.nextWriteTime),
      endTime(sa.endTime), patchUpdates(sa.patchUpdates), patchArea(std::move(sa.patchArea)),
#if defined(WRITENETCDF) || defined(WRITEVTK)
      writer(std::move(sa.writer)),
#endif
      active(sa.active)
#ifdef TIME
      ,
      timeSpentRunning(sa.timeSpentRunning), timeSpentFinishing(sa.timeSpentFinishing),
      timeSpentTerminated(sa.timeSpentTerminated), timeSpentReading(sa.timeSpentReading),
      timeSpentWriting(sa.timeSpentWriting), timeSpentReceiving(sa.timeSpentReceiving),
      timeSpentNotReceiving(sa.timeSpentNotReceiving), timeSpentComputing(sa.timeSpentComputing),
      timesWrittenToFile(sa.timesWrittenToFile)
#endif
{
    if (SimulationActorBase::configuration->scenario->endSimulation() != endTime)
    {
        throw std::runtime_error("Endtime should not change during moves or migration");
    }
}

float SimulationActorBase::getMaxBlockTimestepSize()
{
    block->computeMaxTimestep();
    return block->getMaxTimestep();
}

void SimulationActorBase::setTimestepBaseline(float timestepBaseline) { this->timestepBaseline = timestepBaseline; }

void SimulationActorBase::act()
{
    if (currentState == SimulationActorState::TERMINATED && this->getRunningState() != ActorState::Terminated)
    {
        stop();
        return;
    }

    if (currentTime >= endTime)
    {
        stop();
        currentState = SimulationActorState::TERMINATED;
        return;
    }

    // If we are triggered but can still act lets act as if normal?
    // if (currentState == SimulationActorState::TERMINATED)
    // {
    //     return;
    // }

    auto start = std::chrono::high_resolution_clock::now();

#ifdef LAZY_ACTIVATION
    if (currentState == SimulationActorState::RESTING)
    {
        if (!noChange())
        {
#ifdef REPORT_MAIN_ACTIONS
            std::cout << this->getNameRef() << " is activating!" << std::endl;
#endif
            currentState = SimulationActorState::RUNNING;
        }
    }
#endif

    // it is active send activation signal and change state to
    if (currentState == SimulationActorState::INITIAL)
    {

#if defined(REPORT_MAIN_ACTIONS) || defined(REPORT_MAIN_ACTIONS)
        l.cout() << getName() << " sending initial data to neighbors." << std::endl;
#endif

#ifndef NOWRITE
#ifdef REPORT_MAIN_ACTIONS
        std::cout << this->getName() << ", cur time: " << currentTime << ", timeStepBaseline: " << timestepBaseline
                  << ", outputDelta " << outputDelta << ", nextWriteTime " << nextWriteTime << ", endyTime " << endTime
                  << std::endl;
#endif
        writeTimeStep(0.0f);
#ifdef TIME
        timesWrittenToFile += 1;
#endif
#endif

#ifdef TIME
        auto writestart = std::chrono::high_resolution_clock::now();
        sendData();
        auto writeend = std::chrono::high_resolution_clock::now();
        auto writeelapsed = std::chrono::duration_cast<std::chrono::microseconds>(writeend - writestart);
        timeSpentWriting += writeelapsed.count();
#else
        sendData();
#endif

#ifdef LAZY_ACTIVATION
        if (active)
        {
            currentState = SimulationActorState::RUNNING;
            // std::cout << this->getNameRef() << " starting as active" << std::endl;
        }
        else
        {
            currentState = SimulationActorState::RESTING;
            // std::cout << this->getNameRef() << " starting as resting" << std::endl;
        }
#else
        currentState = SimulationActorState::RUNNING;
#endif
    }
#ifdef LAZY_ACTIVATION
    else if (currentState == SimulationActorState::RESTING && currentTime < endTime)
    {

#if defined(REPORT_MAIN_ACTIONS) || defined(REPORT_MAIN_ACTIONS)
        // l.cout() << getName() << " iteration at " << currentTime << std::endl;
#endif

#ifdef TIME
        auto readstart = std::chrono::high_resolution_clock::now();
        receiveData();
        auto readend = std::chrono::high_resolution_clock::now();
        auto readelapsed = std::chrono::duration_cast<std::chrono::microseconds>(readend - readstart);
        timeSpentReading += readelapsed.count();
#else
        receiveData();
#endif

#ifdef TIME
        auto computestart = std::chrono::high_resolution_clock::now();
        block->setGhostLayer();
        // block->computeNumericalFluxes();
        // block->updateUnknowns(timestepBaseline);
        auto computeend = std::chrono::high_resolution_clock::now();
        auto computeelapsed = std::chrono::duration_cast<std::chrono::microseconds>(computeend - computestart);
        timeSpentComputing += computeelapsed.count();
#else
        block->setGhostLayer();
        // block->computeNumericalFluxes();
        // block->updateUnknowns(timestepBaseline);
#endif

#ifdef TIME
        auto writestart = std::chrono::high_resolution_clock::now();
        sendData();
        auto writeend = std::chrono::high_resolution_clock::now();
        auto writeelapsed = std::chrono::duration_cast<std::chrono::microseconds>(writeend - writestart);
        timeSpentWriting += writeelapsed.count();
#else
        sendData();
#endif
        currentTime += timestepBaseline;
#ifndef NOWRITE
        if (currentTime >= nextWriteTime && currentTime < endTime)
        {
            writeTimeStep(currentTime);
            nextWriteTime += outputDelta;
#ifdef TIME
            timesWrittenToFile += 1;
#endif
#ifdef REPORT_MAIN_ACTIONS
            std::cout << this->getName() << ", cur time: " << currentTime << ", timeStepBaseline: " << timestepBaseline
                      << ", outputDelta " << outputDelta << ", nextWriteTime " << nextWriteTime << ", endTime "
                      << endTime << std::endl;
#endif
        }
#endif
        // we dont update the patch in lazy activation patchUpdates++;
        if (currentTime >= endTime)
        {
#ifdef REPORT_MAIN_ACTIONS
            l.cout() << getName() << "\treached endTime." << std::endl;
#endif
            currentState = SimulationActorState::TERMINATED;
        }
    }
#endif
    else if (currentState == SimulationActorState::RUNNING && currentTime < endTime)
    {
#if defined(REPORT_MAIN_ACTIONS)
        std::cout << "Rank-" << upcxx::rank_me() << ": " << getName() << " iteration at " << currentTime << std::endl;
#endif

#ifdef TIME
        auto readstart = std::chrono::high_resolution_clock::now();
        receiveData();
        auto readend = std::chrono::high_resolution_clock::now();
        auto readelapsed = std::chrono::duration_cast<std::chrono::microseconds>(readend - readstart);
        timeSpentReading += readelapsed.count();
#else
        receiveData();
#endif

#ifdef TIME
        auto computestart = std::chrono::high_resolution_clock::now();

#endif
        block->setGhostLayer();
        auto v1 = std::chrono::steady_clock::now();
        block->computeNumericalFluxes();
        auto v2 = std::chrono::steady_clock::now();
        auto calc = std::chrono::duration_cast<std::chrono::microseconds>(v2 - v1);

        block->updateUnknowns(timestepBaseline);
#ifdef TIME
        auto computeend = std::chrono::high_resolution_clock::now();
        auto computeelapsed = std::chrono::duration_cast<std::chrono::microseconds>(computeend - computestart);
        timeSpentComputing += computeelapsed.count();
#endif

#ifdef TIME
        auto writestart = std::chrono::high_resolution_clock::now();
        sendData();
        auto writeend = std::chrono::high_resolution_clock::now();
        auto writeelapsed = std::chrono::duration_cast<std::chrono::microseconds>(writeend - writestart);
        timeSpentWriting += writeelapsed.count();
#else
        sendData();
#endif
        currentTime += timestepBaseline;
#ifndef NOWRITE
        if (currentTime >= nextWriteTime && currentTime < endTime)
        {
            writeTimeStep(currentTime);
            nextWriteTime += outputDelta;
#ifdef TIME
            timesWrittenToFile += 1;
#endif
#ifdef REPORT_MAIN_ACTIONS
            std::cout << this->getName() << ", cur time: " << currentTime << ", timeStepBaseline: " << timestepBaseline
                      << ", outputDelta " << outputDelta << ", nextWriteTime " << nextWriteTime << ", endTime "
                      << endTime << std::endl;
#endif
        }
#endif
        patchUpdates++;
        if (currentTime >= endTime)
        {
#ifdef REPORT_MAIN_ACTIONS
            l.cout() << getName() << "\treached endTime." << std::endl;
#endif
            currentState = SimulationActorState::TERMINATED;
        }
    }
    else if (currentTime >= endTime)
    {
#ifdef REPORT_MAIN_ACTIONS
        l.cout() << getName() << " terminating at " << currentTime << std::endl;
        std::cout << getNameRef() << " has calculated " << patchUpdates << " updates" << std::endl;
#endif

        // sendTerminationSignal();
        currentState = SimulationActorState::TERMINATED;

        stop();
    }

    auto end = std::chrono::high_resolution_clock::now();

    // std::chrono::duration<double> elapsed = end - start;
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    size_t add = elapsed.count();

#ifdef TIME
    if (currentState == SimulationActorState::FINISHED)
    {
        timeSpentFinishing += add;
    }
    else if (currentState == SimulationActorState::RUNNING)
    {
        timeSpentRunning += add;
    }
    else if (currentState == SimulationActorState::TERMINATED)
    {
        timeSpentTerminated += add;
    }
#endif
}

SimulationActorBase::~SimulationActorBase()
{
#ifdef WRITENETCDF
    if (writer.get() != nullptr && writer->open)
        writer->closeFile();
#endif
}

void SimulationActorBase::writeTimeStep(float currentTime)
{
#if defined(WRITENETCDF) || defined(WRITEVTK)
    writer->writeTimeStep(block->getBathymetry(), block->getWaterHeight(), block->getDischargeHu(),
                          block->getDischargeHv(), upcxx::rank_me(), currentTime);
#endif
}

std::vector<float> SimulationActorBase::createDefaultData()
{
    std::vector<float> data;
    float psize = communicators->patchSize;
    data.resize(communicators->patchSize * 3);

    float val = configuration->scenario->defWaterHeight();
    std::fill(data.begin(), data.begin() + psize, val);
    std::fill(data.begin() + psize, data.end(), 0.0);
    if (data.size() != psize * 3)
    {
        throw std::runtime_error("bug in create default data");
    }
    return data;
}

bool SimulationActorBase::diffThanInitial(const std::vector<float> &packedData)
{
    float def = configuration->scenario->defWaterHeight();
    int patchSize = packedData.size() / 3;
    for (int i = 0; i < 3 * patchSize; i++)
    {

        if (i < patchSize)
        {
            float diff = def - packedData[i];
            if (diff < -0.05 || diff > 0.05)
            {
                // std::cout << diff << " " << def << " " << packedData[i] << std::endl;
                return true;
            }
        }
        else
        {
            float val = packedData[i];
            if (val < -0.05 || val > 0.05)
            {
                // std::cout << val << " " << def << std::endl;
                return true;
            }
        }
    }
    return false;
}

std::string SimulationActorBase::strConfig() const
{
    std::stringstream ss;
    ss << configuration->xSize << "\n";
    ss << configuration->ySize << "\n";
    ss << configuration->patchSize << "\n";
    ss << configuration->fileNameBase << "\n";
    ss << configuration->dx << "\n";
    ss << configuration->dy << "\n";
    ss << configuration->scenario << "\n";
    return ss.str();
}

void SimulationActorBase::computeWriteDelta()
{
    auto numCheckpoints = configuration->numberOfCheckpoints;
    auto endTime = configuration->scenario->endSimulation();
    this->outputDelta = endTime / static_cast<float>(numCheckpoints);
    this->nextWriteTime = this->outputDelta;
}

uint64_t SimulationActorBase::getNumberOfPatchUpdates() const { return patchUpdates; }

void SimulationActorBase::prepareDerived()
{
#ifdef WRITENETCDF
    this->writer->closeFile();
#endif
}

std::string SimulationActorBase::getTimeInfo()
{
    std::stringstream ss;
    ss << upcxx::rank_me() << ": Actor " << this->getName() << " worked for";
#ifdef TIME
    ss << ": Running " << (timeSpentRunning >> 6) << " -> Computing: " << (timeSpentComputing >> 6) << ", Finishing "
       << (timeSpentFinishing >> 6) << ", Terminated " << (timeSpentTerminated >> 6)
       << " read for: " << (timeSpentReading >> 6) << " written for: " << (timeSpentWriting >> 6)
       << " received signal for: " << (timeSpentReceiving >> 6)
       << " checked for hasnt received: " << (timeSpentNotReceiving >> 6)
       << " written to file for times: " << (timesWrittenToFile) << "\n";
#else
    ss << "it was not compiled with TIME option \n";
#endif

    return ss.str();
}

bool SimulationActorBase::getActive() const { return active; }

bool SimulationActorBase::checkTerminated() const
{
    return currentTime >= endTime || currentState == SimulationActorState::TERMINATED;
}