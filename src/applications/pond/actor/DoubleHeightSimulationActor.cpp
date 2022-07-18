#include "DoubleHeightSimulationActor.hpp"

/*
    X X  X X
     A    A
    x X  X X
*/

DoubleHeightSimulationActor::DoubleHeightSimulationActor(std::string &&name, size_t xPos, size_t yPos)
    : SimulationActorBase(std::move(name), {xPos, yPos})
{
    makePorts();
    if (nextWriteTime == 0.0 || outputDelta == 0.0)
    {
        throw std::runtime_error("Unallowed input params");
    }
}

DoubleHeightSimulationActor::DoubleHeightSimulationActor(const std::string &name, size_t xPos, size_t yPos)
    : SimulationActorBase(name, {xPos, yPos})
{
    makePorts();
    if (nextWriteTime == 0.0 || outputDelta == 0.0)
    {
        throw std::runtime_error("Unallowed input params");
    }
}

DoubleHeightSimulationActor::DoubleHeightSimulationActor(
    ActorData &&ad,
    std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
               std::vector<std::unique_ptr<OutPortInformationBase>>> &&tup,
    size_t xPos, size_t yPos, std::unique_ptr<SWE_WaveAccumulationBlock> &&block, SimulationActorState currentState,
    float currentTime, float timestepBaseline, float outputDelta, float nextWriteTime, float endTime,
    uint64_t patchUpdates,
#if defined(WRITENETCDF)
    std::unique_ptr<io::NetCdfWriter> &&writer,
#elif defined(WRITEVTK)
    std::unique_ptr<io::VtkWriter> &&writer,
#endif
    SimulationArea &&sa, bool actv)
    : SimulationActorBase(std::move(ad), std::move(tup), false, xPos, yPos, std::move(block), currentState, currentTime,
                          timestepBaseline, outputDelta, nextWriteTime, endTime, patchUpdates,
#if defined(WRITENETCDF) || defined(WRITEVTK)
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

DoubleHeightSimulationActor::DoubleHeightSimulationActor(DoubleHeightSimulationActor &&sa)
    : SimulationActorBase(std::move(sa))
{
    if (nextWriteTime == 0.0 || outputDelta == 0.0)
    {
        throw std::runtime_error("Unallowed input params");
    }
}

DoubleHeightSimulationActor::DoubleHeightSimulationActor(SimulationActorBaseData &&sabd)
    : SimulationActorBase(std::move(sabd))
{
    if (nextWriteTime == 0.0 || outputDelta == 0.0)
    {
        throw std::runtime_error("Unallowed input params");
    }
}

void DoubleHeightSimulationActor::makePorts()
{
    if (configuration != nullptr)
    {
        throw std::runtime_error("config needs to be set when makeports is called");
    }
    auto totalX = configuration->xSize / (2 * configuration->patchSize);
    auto totalY = configuration->ySize / configuration->patchSize;
    auto xPos = position[0];
    auto yPos = position[1];
    dataIn[(int)LocalPortEnum::BND_TOP_LEFT] =
        (yPos != totalY - 1) ? this->makeInPort<std::vector<float>, 64>("BND_TOP_LEFT") : nullptr;
    dataIn[(int)LocalPortEnum::BND_TOP_RIGHT] =
        (yPos != totalY - 1) ? this->makeInPort<std::vector<float>, 64>("BND_TOP_RIGHT") : nullptr;
    dataIn[(int)LocalPortEnum::BND_BOTTOM_LEFT] =
        (yPos != 0) ? this->makeInPort<std::vector<float>, 64>("BND_BOTTOM_LEFT") : nullptr;
    dataIn[(int)LocalPortEnum::BND_BOTTOM_RIGHT] =
        (yPos != 0) ? this->makeInPort<std::vector<float>, 64>("BND_BOTTOM_RIGHT") : nullptr;

    dataOut[(int)LocalPortEnum::BND_TOP_LEFT] =
        (yPos != totalY - 1) ? this->makeOutPort<std::vector<float>, 64>("BND_TOP_LEFT") : nullptr;
    dataOut[(int)LocalPortEnum::BND_TOP_RIGHT] =
        (yPos != totalY - 1) ? this->makeOutPort<std::vector<float>, 64>("BND_TOP_RIGHT") : nullptr;
    dataOut[(int)LocalPortEnum::BND_BOTTOM_LEFT] =
        (yPos != 0) ? this->makeOutPort<std::vector<float>, 64>("BND_BOTTOM_LEFT") : nullptr;
    dataOut[(int)LocalPortEnum::BND_BOTTOM_RIGHT] =
        (yPos != 0) ? this->makeOutPort<std::vector<float>, 64>("BND_BOTTOM_RIGHT") : nullptr;

    dataIn[(int)LocalPortEnum::BND_LEFT] = (xPos != 0) ? this->makeInPort<std::vector<float>, 64>("BND_LEFT") : nullptr;
    dataOut[(int)LocalPortEnum::BND_LEFT] =
        (xPos != 0) ? this->makeOutPort<std::vector<float>, 64>("BND_LEFT") : nullptr;
    dataIn[(int)LocalPortEnum::BND_RIGHT] =
        (xPos != totalX - 1) ? this->makeInPort<std::vector<float>, 64>("BND_RIGHT") : nullptr;
    dataOut[(int)LocalPortEnum::BND_RIGHT] =
        (xPos != totalX - 1) ? this->makeOutPort<std::vector<float>, 64>("BND_RIGHT") : nullptr;
}

void DoubleHeightSimulationActor::initializeBlock()
{
    size_t xPos = position[0];
    size_t yPos = position[1];
    auto totalX = configuration->xSize / (configuration->patchSize * 2);
    auto totalY = configuration->ySize / configuration->patchSize;
    block->initScenario(patchArea.minX, patchArea.minY, *(configuration->scenario), true);
    initializeBoundary(BoundaryEdge::BND_LEFT, [xPos]() { return xPos == 0; });
    initializeBoundary(BoundaryEdge::BND_RIGHT, [xPos, totalX]() { return xPos == totalX - 1; });
    initializeBoundary(BoundaryEdge::BND_BOTTOM, [yPos]() { return yPos == 0; });
    initializeBoundary(BoundaryEdge::BND_TOP, [yPos, totalY]() { return yPos == totalY - 1; });
    this->computeWriteDelta();
    this->active = block->displacement(configuration->scenario->defWaterHeight());
}

void DoubleHeightSimulationActor::reInitializeBlock()
{
    size_t xPos = position[0];
    size_t yPos = position[1];
    auto totalX = configuration->xSize / (configuration->patchSize * 2);
    auto totalY = configuration->ySize / configuration->patchSize;
    block->reinitScenario(patchArea.minX, patchArea.minY, *(configuration->scenario), true);
    initializeBoundary(BoundaryEdge::BND_LEFT, [xPos]() { return xPos == 0; });
    initializeBoundary(BoundaryEdge::BND_RIGHT, [xPos, totalX]() { return xPos == totalX - 1; });
    initializeBoundary(BoundaryEdge::BND_BOTTOM, [yPos]() { return yPos == 0; });
    initializeBoundary(BoundaryEdge::BND_TOP, [yPos, totalY]() { return yPos == totalY - 1; });
}

void DoubleHeightSimulationActor::initializeBoundary(BoundaryEdge edge, std::function<bool()> isBoundary)
{
    const Scenario *sc = configuration->scenario;
    if (isBoundary())
    {
        block->setBoundaryType(edge, sc->getBoundaryType(edge));
    }
    else
    {
        block->setBoundaryType(edge, BoundaryType::PASSIVE);
        if (edge == BoundaryEdge::BND_BOTTOM || edge == BoundaryEdge::BND_TOP)
        {
            communicators[(int)edge] = BlockCommunicator(configuration->patchSize * 2, block->registerCopyLayer(edge),
                                                         block->grabGhostLayer(edge));
        }
        else
        {
            communicators[(int)edge] = BlockCommunicator(configuration->patchSize, block->registerCopyLayer(edge),
                                                         block->grabGhostLayer(edge));
        }
    }
}

bool DoubleHeightSimulationActor::mayWrite()
{
    for (int i = 0; i < 6; i++)
    {
        if (this->dataOut[i])
        {
            if (this->dataOut[i]->freeCapacity() <= 0)
            {
                return false;
            }
        }
    }
    return true;
}

bool DoubleHeightSimulationActor::mayRead()
{
    bool res = true;
    for (int i = 0; i < 6; i++)
    {
        if (this->dataIn[i])
        {
            if (this->dataIn[i]->available() <= 0)
            {
                return false;
            }
        }
    }
    return true;
}

bool DoubleHeightSimulationActor::hasReceivedTerminationSignal()
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

std::tuple<std::vector<float>, std::vector<float>> split(std::vector<float> &v)
{

    std::vector<float> a;
    std::vector<float> b;

    size_t s = v.size() / 6;
    if (v.size() % 6 != 0)
    {
        throw std::runtime_error("Ghost layer size is wrong");
    }

    a.insert(a.end(), v.begin(), v.begin() + s);
    a.insert(a.end(), v.begin() + 2 * s, v.begin() + 3 * s);
    a.insert(a.end(), v.begin() + 4 * s, v.begin() + 5 * s);

    b.insert(b.end(), v.begin() + s, v.begin() + 2 * s);
    b.insert(b.end(), v.begin() + 3 * s, v.begin() + 4 * s);
    b.insert(b.end(), v.begin() + 5 * s, v.begin() + 6 * s);

    return {a, b};
}

std::vector<float> combine(std::vector<float> &v1, std::vector<float> &v2)
{
    std::vector<float> a;

    if (v1.empty() || v2.empty())
    {
        return a;
    }

    if (v1.empty() || v2.empty() || v1.size() % 3 != 0 || v1.size() != v2.size())
    {
        throw std::runtime_error("Copy layer size is wrong");
    }

    size_t s = v1.size() / 3;

    a.insert(a.end(), v1.begin(), v1.begin() + s);
    a.insert(a.end(), v2.begin(), v2.begin() + s);
    a.insert(a.end(), v1.begin() + s, v1.begin() + 2 * s);
    a.insert(a.end(), v2.begin() + s, v2.begin() + 2 * s);
    a.insert(a.end(), v1.begin() + 2 * s, v1.begin() + 3 * s);
    a.insert(a.end(), v2.begin() + 2 * s, v2.begin() + 3 * s);

    return a;
}

void DoubleHeightSimulationActor::sendData()
{
    for (int i = 0; i < 4; i++)
    {

        if (i == (int)BoundaryEdge::BND_BOTTOM)
        {
            if (this->dataOut[(int)LocalPortEnum::BND_BOTTOM_LEFT] &&
                this->dataOut[(int)LocalPortEnum::BND_BOTTOM_RIGHT])
            {
                auto packedData = communicators[i].packCopyLayer();
                auto splits = split(packedData);
                dataOut[(int)LocalPortEnum::BND_BOTTOM_LEFT]->write(std::move(std::get<0>(splits)));
                dataOut[(int)LocalPortEnum::BND_BOTTOM_RIGHT]->write(std::move(std::get<1>(splits)));
            }
        }
        else if (i == (int)BoundaryEdge::BND_TOP)
        {
            if (this->dataOut[(int)LocalPortEnum::BND_TOP_LEFT] && this->dataOut[(int)LocalPortEnum::BND_TOP_RIGHT])
            {
                auto packedData = communicators[i].packCopyLayer();
                auto splits = split(packedData);
                dataOut[(int)LocalPortEnum::BND_TOP_LEFT]->write(std::move(std::get<0>(splits)));
                dataOut[(int)LocalPortEnum::BND_TOP_RIGHT]->write(std::move(std::get<1>(splits)));
            }
        }
        else if (i == (int)BoundaryEdge::BND_RIGHT)
        {
            if (this->dataOut[(int)LocalPortEnum::BND_RIGHT])
            {
                auto packedData = communicators[i].packCopyLayer();
                dataOut[(int)LocalPortEnum::BND_RIGHT]->write(std::move(packedData));
            }
        }
        else if (i == (int)BoundaryEdge::BND_LEFT)
        {
            if (this->dataOut[(int)LocalPortEnum::BND_LEFT])
            {
                auto packedData = communicators[i].packCopyLayer();
                dataOut[(int)LocalPortEnum::BND_LEFT]->write(std::move(packedData));
            }
        }
        else
        {
            throw std::runtime_error("should have matched");
        }
    }
}

void DoubleHeightSimulationActor::sendTerminationSignal()
{
    for (int i = 0; i < 6; i++)
    {
        if (this->dataOut[i] && this->dataOut[i]->freeCapacity() > 0)
        {
            std::vector<float> v;
            v.push_back(std::numeric_limits<float>::min());
            dataOut[i]->write(std::move(v));
        }
    }
}

void DoubleHeightSimulationActor::receiveData()
{
    std::vector<float> bottoms1;
    std::vector<float> bottoms2;
    std::vector<float> tops1;
    std::vector<float> tops2;

    for (int i = 0; i < 6; i++)
    {
        if (this->dataIn[i])
        {
            auto packedData = dataIn[i]->read();
            switch (i)
            {
            case ((int)LocalPortEnum::BND_BOTTOM_LEFT):
            {
                bottoms1 = packedData;
            }
            break;
            case ((int)LocalPortEnum::BND_BOTTOM_RIGHT):
            {
                bottoms2 = packedData;
            }
            break;
            case ((int)LocalPortEnum::BND_TOP_LEFT):
            {
                tops1 = packedData;
            }
            break;
            case ((int)LocalPortEnum::BND_TOP_RIGHT):
            {
                tops2 = packedData;
            }
            break;
            case ((int)LocalPortEnum::BND_LEFT):
            {
                communicators[(int)BoundaryEdge::BND_LEFT].receiveGhostLayer(std::move(packedData));
            }
            break;
            case ((int)LocalPortEnum::BND_RIGHT):
            {
                communicators[(int)BoundaryEdge::BND_RIGHT].receiveGhostLayer(std::move(packedData));
            }
            break;
            default:
                throw std::runtime_error("should have amtched");
            }
        }
    }

    auto l = combine(bottoms1, bottoms2);
    auto h = combine(tops1, tops2);
    if (l.size() != 0)
        communicators[(int)BoundaryEdge::BND_BOTTOM].receiveGhostLayer(std::move(l));
    if (h.size() != 0)
        communicators[(int)BoundaryEdge::BND_TOP].receiveGhostLayer(std::move(h));
}

bool DoubleHeightSimulationActor::noChange()
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
