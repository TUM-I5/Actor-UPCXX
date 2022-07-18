#pragma once

#include <iostream>
#include <memory>
#include <string>

/*
  Base classes of portInformations, these are required to have names so that actor can assign
  right information to right ports, also generateCopy functions must be virtual so that the
  base class Actor can assign them without user needing to manually do it
*/

class InPortInformationBase
{
  public:
    std::string name;

    InPortInformationBase(std::string &&name) : name(std::move(name)) {}
    InPortInformationBase(const std::string &name) : name(name) {}
    InPortInformationBase(InPortInformationBase &&other) : name(std::move(other.name)) {}
    InPortInformationBase(const InPortInformationBase &other) = delete;
    InPortInformationBase() : name(){};
    InPortInformationBase &operator=(const InPortInformationBase &other) = delete;
    InPortInformationBase &operator=(InPortInformationBase &&other)
    {
        name = std::move(other.name);
        return *this;
    }
    std::string getName() const { return name; }
    virtual ~InPortInformationBase() {}
    virtual std::unique_ptr<InPortInformationBase> generateCopy() const = 0;
    virtual size_t getMessageCount() const = 0;
};

class OutPortInformationBase
{
  public:
    std::string name;

    OutPortInformationBase(std::string &&name) : name(std::move(name)) {}
    OutPortInformationBase(OutPortInformationBase &&other) : name(std::move(other.name)) {}
    OutPortInformationBase(const OutPortInformationBase &other) = delete;
    OutPortInformationBase(const std::string &name) : name(name) {}
    OutPortInformationBase() : name(){};
    OutPortInformationBase &operator=(const OutPortInformationBase &other) = delete;
    OutPortInformationBase &operator=(OutPortInformationBase &&other)
    {
        name = std::move(other.name);
        return *this;
    }
    std::string getName() const { return name; }
    virtual ~OutPortInformationBase() {}
    virtual std::unique_ptr<OutPortInformationBase> generateCopy() const = 0;
};