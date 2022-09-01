#include "actor/SimulationActor.hpp"

SimulationActor::SimulationActor(std::string &&name, size_t xPos, size_t yPos)
    : SimulationActorBase(std::move(name), {xPos, yPos})
{
    makePorts();
    initializeBlock();
    if (nextWriteTime == 0.0 || outputDelta == 0.0)
    {
        throw std::runtime_error("Unallowed input params");
    }
}

SimulationActor::SimulationActor(const std::string &name, size_t xPos, size_t yPos)
    : SimulationActorBase(name, {xPos, yPos})
{
    makePorts();
    initializeBlock();
    if (nextWriteTime == 0.0 || outputDelta == 0.0)
    {
        throw std::runtime_error("Unallowed input params");
    }
}

SimulationActor::SimulationActor(ActorData &&ad,
                                 std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                            std::vector<std::unique_ptr<OutPortInformationBase>>> &&tup,
                                 size_t xPos, size_t yPos, std::unique_ptr<SWE_WaveAccumulationBlock> &&block,
                                 SimulationActorState currentState, float currentTime, float timestepBaseline,
                                 float outputDelta, float nextWriteTime, float endTime, uint64_t patchUpdates,
#if defined(WRITENETCDF)
                                 std::unique_ptr<io::NetCdfWriter> &&writer,
#elif defined(WRITEVTK)
                                 std::unique_ptr<io::VtkWriter> &&writer,
#endif
                                 SimulationArea &&sa, bool actv /*, std::vector<std::string> &&activeNeighbors*/)
    : SimulationActorBase(std::move(ad), std::move(tup), false, xPos, yPos, std::move(block), currentState, currentTime,
                          timestepBaseline, outputDelta, nextWriteTime, endTime, patchUpdates,
#if defined(WRITEVTK) || defined(WRITENETCDF)
                          std::move(writer),
#endif
                          std::move(sa), actv)
{

    makePorts();
    reInitializeBlock();
    if (nextWriteTime == 0.0 || outputDelta == 0.0)
    {
        throw std::runtime_error("Unallowed input params");
    }
}

SimulationActor::SimulationActor(SimulationActor &&sa)
    : SimulationActorBase(std::move(dynamic_cast<SimulationActorBase &&>(sa)))
{

    reInitializeBlock();
    for (int i = 0; i < 4; i++)
    {
        dataIn[i] = sa.dataIn[i];
        dataOut[i] = sa.dataOut[i];
        sa.dataIn[i] = nullptr;
        sa.dataOut[i] = nullptr;
    }
    if (nextWriteTime == 0.0 || outputDelta == 0.0)
    {
        throw std::runtime_error("Unallowed input params");
    }
}

SimulationActor::SimulationActor(SimulationActorBaseData &&sabd) : SimulationActorBase(std::move(sabd))
{
    makePorts();
    reInitializeBlock();
    if (nextWriteTime == 0.0 || outputDelta == 0.0)
    {
        throw std::runtime_error("Unallowed input params");
    }
}

void SimulationActor::makePorts()
{
    if (configuration == nullptr)
    {
        throw std::runtime_error("Make ports should be called after configuration is set");
    }

    auto totalX = configuration->xSize / configuration->patchSize;
    auto totalY = configuration->ySize / configuration->patchSize;
    auto xPos = position[0];
    auto yPos = position[1];
    dataIn[(int)BoundaryEdge::BND_LEFT] = (xPos != 0) ? this->makeInPort<std::vector<float>, 64>("BND_LEFT") : nullptr;
    dataOut[(int)BoundaryEdge::BND_LEFT] =
        (xPos != 0) ? this->makeOutPort<std::vector<float>, 64>("BND_LEFT") : nullptr;
    dataIn[(int)BoundaryEdge::BND_RIGHT] =
        (xPos != totalX - 1) ? this->makeInPort<std::vector<float>, 64>("BND_RIGHT") : nullptr;
    dataOut[(int)BoundaryEdge::BND_RIGHT] =
        (xPos != totalX - 1) ? this->makeOutPort<std::vector<float>, 64>("BND_RIGHT") : nullptr;
    dataIn[(int)BoundaryEdge::BND_BOTTOM] =
        (yPos != 0) ? this->makeInPort<std::vector<float>, 64>("BND_BOTTOM") : nullptr;
    dataOut[(int)BoundaryEdge::BND_BOTTOM] =
        (yPos != 0) ? this->makeOutPort<std::vector<float>, 64>("BND_BOTTOM") : nullptr;
    dataIn[(int)BoundaryEdge::BND_TOP] =
        (yPos != totalY - 1) ? this->makeInPort<std::vector<float>, 64>("BND_TOP") : nullptr;
    dataOut[(int)BoundaryEdge::BND_TOP] =
        (yPos != totalY - 1) ? this->makeOutPort<std::vector<float>, 64>("BND_TOP") : nullptr;
}

void SimulationActor::initializeBlock()
{
    size_t xPos = position[0];
    size_t yPos = position[1];
    auto totalX = configuration->xSize / configuration->patchSize;
    auto totalY = configuration->ySize / configuration->patchSize;
    block->initScenario(patchArea.minX, patchArea.minY, *(configuration->scenario), true);
    initializeBoundary(BoundaryEdge::BND_LEFT, [xPos]() { return xPos == 0; });
    initializeBoundary(BoundaryEdge::BND_RIGHT, [xPos, totalX]() { return xPos == totalX - 1; });
    initializeBoundary(BoundaryEdge::BND_BOTTOM, [yPos]() { return yPos == 0; });
    initializeBoundary(BoundaryEdge::BND_TOP, [yPos, totalY]() { return yPos == totalY - 1; });
    this->computeWriteDelta();
    this->active = block->displacement(configuration->scenario->defWaterHeight());
}

void SimulationActor::reInitializeBlock()
{
    size_t xPos = position[0];
    size_t yPos = position[1];
    auto totalX = configuration->xSize / configuration->patchSize;
    auto totalY = configuration->ySize / configuration->patchSize;
    block->reinitScenario(*(configuration->scenario), true);
    initializeBoundary(BoundaryEdge::BND_LEFT, [xPos]() { return xPos == 0; });
    initializeBoundary(BoundaryEdge::BND_RIGHT, [xPos, totalX]() { return xPos == totalX - 1; });
    initializeBoundary(BoundaryEdge::BND_BOTTOM, [yPos]() { return yPos == 0; });
    initializeBoundary(BoundaryEdge::BND_TOP, [yPos, totalY]() { return yPos == totalY - 1; });
}

void SimulationActor::initializeBoundary(BoundaryEdge edge, std::function<bool()> isBoundary)
{
    const Scenario *sc = configuration->scenario;
    if (isBoundary())
    {
        block->setBoundaryType(edge, sc->getBoundaryType(edge));
    }
    else
    {
        block->setBoundaryType(edge, BoundaryType::PASSIVE);
        communicators[(int)edge] =
            BlockCommunicator(configuration->patchSize, block->registerCopyLayer(edge), block->grabGhostLayer(edge));
    }
}

bool SimulationActor::mayRead()
{
    bool res = true;
    // std::stringstream ss;
    for (int i = 0; i < 4; i++)
    {
        if (this->dataIn[i])
        {
            if (this->dataIn[i]->available() <= 0)
            {
                // ss << dataIn[i]->getName() << " is empty | ";
                // res = false;
                return false;
            }
            else
            {
                // ss << dataIn[i]->getName() << " has messages | ";
            }
        }
    }
    // std::cout << this->getName() << " " << ss.str() << std::endl;
    return res;
}

bool SimulationActor::mayWrite()
{
    bool res = true;
    // std::stringstream ss;
    // ss << this->getName() << " can write to: ";
    for (int i = 0; i < 4; i++)
    {
        res &= (!this->dataOut[i] || this->dataOut[i]->freeCapacity() > 0);
        // if (this->dataOut[i] && this->dataOut[i]->freeCapacity() > 0)
        //{
        //    ss << this->dataOut[i]->getName() << " | ";
        //}
    }
    return res;
}

bool SimulationActor::hasReceivedTerminationSignal()
{
#ifdef TIME
    auto start = std::chrono::high_resolution_clock::now();
#endif

    bool res = false;
    for (int i = 0; i < 4; i++)
    {
        if (this->dataIn[i] && this->dataIn[i]->available() > 0)
        {
            auto v = dataIn[i]->peek();
            if (v.size() == 1 && v[0] == std::numeric_limits<float>::min())
            {
                res |= true;
                dataIn[i]->read();
            }
        }
    }

#ifdef TIME
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    timeSpentReceiving += elapsed.count();
#endif

    return res;
}

void SimulationActor::sendData()
{
    // std::cout << this->getName() << " sending data" << std::endl;
    for (int i = 0; i < 4; i++)
    {
        if (this->dataOut[i])
        {
            auto packedData = communicators[i].packCopyLayer();
            if (packedData.empty())
            {
                throw std::runtime_error("message taken from inport is empty (should not occur)");
            }
            this->dataOut[i]->write(std::move(packedData));
        }
    }
}

void SimulationActor::sendTerminationSignal()
{
    for (int i = 0; i < 4; i++)
    {
        if (this->dataOut[i] && this->dataOut[i]->freeCapacity() > 0)
        {
            std::vector<float> v;
            v.push_back(std::numeric_limits<float>::min());
            dataOut[i]->write(std::move(v));
        }
    }
}

void SimulationActor::receiveData()
{
    for (int i = 0; i < 4; i++)
    {
        if (this->dataIn[i])
        {
            auto packedData = dataIn[i]->read();
            communicators[i].receiveGhostLayer(std::move(packedData));
        }
    }
}

bool SimulationActor::noChange()
{
    for (int i = 0; i < 4; i++)
    {
        if (this->dataIn[i])
        {
            if (this->dataIn[i]->available() > 0)
            {
                auto packedData = dataIn[i]->peek();
                // communicators[i].receiveGhostLayer(packedData);
                if (diffThanInitial(packedData))
                {
                    return false;
                }
            }
        }
    }
    return true;
}
