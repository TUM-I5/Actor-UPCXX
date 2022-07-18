#pragma once
#include <set>

class PortMakerBase;

class PortConnectorBase
{
  public:
    bool finished = false;
    virtual void forwardMessage() = 0;
};
