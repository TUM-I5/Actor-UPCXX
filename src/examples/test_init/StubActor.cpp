#include "StubActor.hpp"

int StubActor::max = 100000;

void StubActor::makePorts(int out, int in)
{
    for (int i = 0; i < out; i++)
    {
        outs[i] = this->makeOutPort<std::vector<float>, 32>(std::to_string(i));
    }
    for (int i = 0; i < in; i++)
    {
        ins[i] = this->makeInPort<std::vector<float>, 32>(std::to_string(i));
    }
}

StubActor::StubActor(std::string name, int outgoingPortCount, int incomingPortCount)
    : Actor(name), inpcount(incomingPortCount), outpcount(outgoingPortCount), sends(0)
{
    makePorts(outgoingPortCount, incomingPortCount);
}

StubActor::StubActor(std::string &&name, int stub1, int stub2, int stub3) : Actor(name), inpcount(1), outpcount(1)
{
    makePorts(1, 1);
}

StubActor::StubActor(std::string &&name, std::variant<std::monostate> &&emptyVar)
    : Actor(name), inpcount(1), outpcount(1), sends(0)
{
    makePorts(1, 1);
}

StubActor::StubActor(const std::string &name, const std::variant<std::monostate> &emptyVar)
    : Actor(name), inpcount(1), outpcount(1), sends(0)
{
    makePorts(1, 1);
}

StubActor::StubActor(StubActor &&other)
    : Actor(dynamic_cast<Actor &&>(other)), inpcount(other.inpcount), outpcount(other.outpcount), sends(other.sends)
{
    makePorts(other.outpcount, other.inpcount);
    movePortsInformation(dynamic_cast<Actor &&>(other));
}

StubActor::StubActor(ActorData &&data) : Actor(std::move(data)) {}

void StubActor::act()
{
    if (sends < StubActor::max)
    {
    }
    else
    {
        this->stop();
    }
}
