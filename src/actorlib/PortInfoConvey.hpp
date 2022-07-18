#pragma once

#include <tuple>
#include <upcxx/upcxx.hpp>

/*
    PortsInformationWriter and PortsInformationReader convenience classes
    that can be used by user to easily write and read port informations
*/
#include "AbstractInPort.hpp"
#include "AbstractOutPort.hpp"
#include "PortInformation.hpp"
#include "PortInformationBase.hpp"
#include "Utility.hpp"

template <class type, int capacity> class PortsInformationWriter
{
    static_assert(upcxx::is_serializable<type>::value || upcxx::is_trivially_serializable<type>::value);

  public:
    std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
               std::vector<std::unique_ptr<OutPortInformationBase>>>
        portsInformationToWrite;

    PortsInformationWriter(std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                                      std::vector<std::unique_ptr<OutPortInformationBase>>> &&dataToWrite)
        : portsInformationToWrite(std::move(dataToWrite))
    {
    } // construct with the portsinformation tuple to directly write

    // PortsInformationWriter() : portsInformationToWrite{0, nullptr, 0, nullptr} {} // user needs to assign it by
    // itself

    void assign(std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
                           std::vector<std::unique_ptr<OutPortInformationBase>>> &&dataToWrite)
    {
        portsInformationToWrite = std::move(dataToWrite);
    }

    /*
        call write for every inportinformationbase object where the call is forwarred to the right PortInformation type
        same holds for the OutPort too
    */

    template <class Writer> void writeData(Writer &writer) const
    {
        writeLimits<Writer>(writer);
        writeInports<Writer>(writer);
        writeOutports<Writer>(writer);
    }

    // write the amount of in and outports
    template <class Writer> void writeLimits(Writer &writer) const
    {

        int limit1 = std::get<0>(portsInformationToWrite).size();
        int limit2 = std::get<1>(portsInformationToWrite).size();
        writer.write(limit1);
        writer.write(limit2);
    }

    // write the inports, downcast inportinformationbase to the declared type in the portinfoconvey
    template <class Writer> void writeInports(Writer &writer) const
    {
        for (auto &el : std::get<0>(portsInformationToWrite))
        {
            const InPortInformation<type, capacity> *ptr =
                dynamic_cast<const InPortInformation<type, capacity> *>(el.get());
            writer.write(*ptr);
        }
    }

    // write the outports, downcast outportinformationbase to the declared type in the portinfoconvey
    template <class Writer> void writeOutports(Writer &writer) const
    {
        for (auto &el : std::get<1>(portsInformationToWrite))
        {

            const OutPortInformation<type, capacity> *downcasted =
                dynamic_cast<const OutPortInformation<type, capacity> *>(el.get());
            writer.write(*downcasted);
        }
    }

    ~PortsInformationWriter(){};
};

/*
    Same as writer but this time we read
*/

template <class type, int capacity> class PortsInformationReader
{
  public:
    int inpcount;  // need to know how much inports tehre was
    int outpcount; // out ports too

    PortsInformationReader(int inPortCount, int outPortCount) : inpcount(inPortCount), outpcount(outPortCount)
    {
        if (inPortCount <= 0 || outPortCount <= 0)
        {
            throw std::runtime_error("Illegal inport or outport count for ports information");
        }
    }

    PortsInformationReader() : inpcount(0), outpcount(0) {}

    // read amount of ports to read
    template <class Reader> std::tuple<int, int> readLimits(Reader &reader)
    {
        int l1 = reader.template read<int>();
        int l2 = reader.template read<int>();
        return {l1, l2};
    }

    // inspector-xe
    // ddt ->

    // read inports until the limit, read it and then upcast and add to an array so that it can be delegated to the
    // struct for the cosntruction of the actor
    template <class Reader> std::vector<std::unique_ptr<InPortInformationBase>> readInports(Reader &reader)
    {
        std::vector<std::unique_ptr<InPortInformationBase>> arr;
        for (int i = 0; i < inpcount; i++)
        {
            std::unique_ptr<InPortInformationBase> upp(util::read<Reader, InPortInformation<type, capacity>>(reader));
            // upp = std::move(up);
            arr.emplace_back(std::move(upp));
        }
        return arr;
    }

    // the same read strategy applied to the outports
    template <class Reader> std::vector<std::unique_ptr<OutPortInformationBase>> readOutports(Reader &reader)
    {
        std::vector<std::unique_ptr<OutPortInformationBase>> arr;
        for (int i = 0; i < outpcount; i++)
        {
            std::unique_ptr<OutPortInformationBase> upp(util::read<Reader, OutPortInformation<type, capacity>>(reader));
            // upp = std::move(up);
            arr.emplace_back(std::move(upp));
        }
        return arr;
    }

    // read limits and then read given amount of inports and outpurts return the portsinformation struct
    template <class Reader>
    std::tuple<std::vector<std::unique_ptr<InPortInformationBase>>,
               std::vector<std::unique_ptr<OutPortInformationBase>>>
    readData(Reader &reader)
    {
        auto lims = readLimits(reader);

        inpcount = std::get<0>(lims);

        outpcount = std::get<1>(lims);

        std::vector<std::unique_ptr<InPortInformationBase>> inarr = readInports(reader);

        std::vector<std::unique_ptr<OutPortInformationBase>> outarr = readOutports(reader);

        return std::make_tuple(std::move(inarr), std::move(outarr));
    }

    ~PortsInformationReader(){};
};