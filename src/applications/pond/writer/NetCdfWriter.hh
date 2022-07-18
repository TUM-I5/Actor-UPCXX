/**
 * @file
 * This file is part of Pond.
 *
 * @author Alexander Breuer
 * @author Sebastian Rettenberger (rettenbs AT in.tum.de,
 * http://www5.in.tum.de/wiki/index.php/Sebastian_Rettenberger,_M.Sc.)
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

#ifndef NETCDFWRITER_HH_
#define NETCDFWRITER_HH_

#include <cstring>
#include <string>
#include <vector>

#ifdef USEMPI
#include <mpi.h>
#ifndef MPI_INCLUDED
#define MPI_INCLUDED
#define MPI_INCLUDED_NETCDF
#endif
#endif

#ifdef WRITENETCDF
#include <netcdf.h>
#endif

#ifdef MPI_INCLUDED_NETCDF
#undef MPI_INCLUDED
#undef MPI_INCLUDED_NETCDF
#endif

#include "scenario/SimulationArea.hpp"
#include "writer/Writer.hh"

namespace io
{
class NetCdfWriter;
}

class io::NetCdfWriter final : public io::Writer
{
  private:
    /** netCDF file id*/
    int dataFile;

    /** Variable ids */
    int timeVar, hVar, huVar, hvVar, bVar, rankVar;

    /** Part of the simulation domain covered by the writer */
    const SimulationArea patchArea;

    /** time to pass between writes */
    float writeDelta;

    /** Flush after every x write operation? */
    unsigned int flush;

    // writer time dependent variables.
    void writeVarTimeDependent(const Float2D &i_matrix, int i_ncVariable);

    // writes time independent variables.
    void writeVarTimeIndependent(const Float2D &i_matrix, int i_ncVariable);

    // write rank similar to water height
    void writeRank(const int i_rank, int i_ncVariable);

  public:
    // bool for open-status of file
    // do not change only to be changed by open_ and close_file functions
    mutable bool open;

    NetCdfWriter(const std::string &i_fileName, BoundarySize i_boundarySize, const SimulationArea &sa, float writeDelta,
                 int i_nX, int i_nY, float i_dX, float i_dY, float i_originX = 0., float i_originY = 0.,
                 unsigned int i_flush = 0);

    NetCdfWriter(WriterData &&wd, int dataFile, int timeVar, int hVar, int huVar, int hvVar, int bVar, int rankVar,
                 SimulationArea pA, float writeDelta, unsigned int flush);

    ~NetCdfWriter() override final;

    int closeFile() const;
    int openFile();

    // both copy operations move the ownership of the file the next,
    // use with caution
    NetCdfWriter(NetCdfWriter &&other);
    NetCdfWriter(const NetCdfWriter &other) = delete;

    void writeTimeStep(const Float2D &i_b, const Float2D &i_h, const Float2D &i_hu, const Float2D &i_hv, int i_rank,
                       float time) override final;
    // void writeTimeStep(const Float2D &i_b, const Float2D &i_h, const Float2D &i_hu, const Float2D &i_hv, const
    // Float2D &i_r, float i_time) override final; writes the unknowns at a given time step to the netCDF-file.
    inline void writeTimeStepInt(const Float2D &i_b, const Float2D &i_h, const Float2D &i_hu, const Float2D &i_hv,
                                 const int i_rank, float i_time);
    // inline void writeTimeStepInt(const Float2D &i_b, const Float2D &i_h, const Float2D &i_hu, const Float2D &i_hv,
    // const Float2D &i_r, float i_time);

    /**
     * This is a small wrapper for `nc_put_att_text` which automatically sets
     * the length.
     */
    void ncPutAttText(int varid, const char *name, const char *value)
    {
#ifdef WRITENETCDF
        nc_put_att_text(dataFile, varid, name, strlen(value), value);
#endif
    }

  public:
    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, const NetCdfWriter &object)
        {
#ifndef NOWRITE
            object.closeFile();
            assert(!object.open);
#endif

            writer.write(static_cast<io::Writer const &>(object));
            writer.write(object.dataFile);
            writer.write(object.timeVar);
            writer.write(object.hVar);
            writer.write(object.huVar);
            writer.write(object.hvVar);
            writer.write(object.bVar);
            writer.write(object.rankVar);
            writer.write(object.patchArea);
            writer.write(object.writeDelta);
            writer.write(object.flush);

            return;
        }

        template <typename Reader> static NetCdfWriter *deserialize(Reader &reader, void *storage)
        {
            WriterData wd = (reader.template read<WriterData>());
            int dataFileNum = reader.template read<int>();
            int timeVar = reader.template read<int>();
            int hVar = reader.template read<int>();
            int huVar = reader.template read<int>();
            int hvVar = reader.template read<int>();
            int bVar = reader.template read<int>();
            int rankVar = reader.template read<int>();
            SimulationArea patchArea = reader.template read<SimulationArea>();
            float writeDelta = reader.template read<float>();
            unsigned int flush = reader.template read<unsigned int>();
            NetCdfWriter *netcdfw = ::new (storage) NetCdfWriter(std::move(wd), dataFileNum, timeVar, hVar, huVar,
                                                                 hvVar, bVar, rankVar, patchArea, writeDelta, flush);
#ifndef NOWRITE
            netcdfw->openFile();
            assert(netcdfw->open);
#endif
            return netcdfw;
        }
    };
};

#endif /* NETCDFWRITER_HH_ */
