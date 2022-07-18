/**
 * @file
 * This file is part of Pond.
 *
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

#ifndef WRITER_HH_
#define WRITER_HH_

#include "util/help.hh"

namespace io
{
class BoundarySize;
class Writer;
class WriterData;
} // namespace io

/**
 * This struct is used so we can initialize this array
 * in the constructor.
 */
class io::BoundarySize
{
  public:
    /**
     * boundarySize[0] == left
     * boundarySize[1] == right
     * boundarySize[2] == bottom
     * boundarySize[3] == top
     */
    int boundarySize[4];

    int &operator[](unsigned int i) { return boundarySize[i]; }

    int operator[](unsigned int i) const { return boundarySize[i]; }

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, const io::BoundarySize &object)
        {
            for (int i = 0; i < 4; i++)
                writer.write(object.boundarySize[i]);

            return;
        }

        template <typename Reader> static io::BoundarySize *deserialize(Reader &reader, void *storage)
        {
            io::BoundarySize *bs = ::new (storage) io::BoundarySize();
            for (int i = 0; i < 4; i++)
                bs->boundarySize[i] = reader.template read<int>();
            return bs;
        }
    };
};

class io::WriterData
{
  public:
    std::string fileName;
    io::BoundarySize boundarySize;
    unsigned int nX, nY;
    size_t timeStep;

    WriterData(std::string &&name, io::BoundarySize &&bs, unsigned int nX, unsigned int nY, size_t timeStep)
        : fileName(std::move(name)), boundarySize(std::move(bs)), nX(nX), nY(nY), timeStep(timeStep)
    {
    }

    WriterData(const std::string &name, const io::BoundarySize &bs, const unsigned int nX, const unsigned int nY,
               const size_t timeStep)
        : fileName(name), boundarySize(bs), nX(nX), nY(nY), timeStep(timeStep)
    {
    }

    // wd.fileName.empty() ? "emptiah!!!-move": wd.fileName
    WriterData(WriterData &&wd)
        : fileName(std::move(wd.fileName)), boundarySize(wd.boundarySize), nX(wd.nX), nY(wd.nY), timeStep(wd.timeStep)
    {
    }

    WriterData(WriterData &wd)
        : fileName(wd.fileName), boundarySize(wd.boundarySize), nX(wd.nX), nY(wd.nY), timeStep(wd.timeStep)
    {
    }

    WriterData() : fileName("-1"), boundarySize({-1, -1, -1, -1}), nX(-1), nY(-1), timeStep(-1) {}

    ~WriterData(){};

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, io::WriterData const &object)
        {

            if (object.fileName.empty())
            {
                throw std::runtime_error("writer in illegal state");
            }

            writer.write(object.fileName);
            writer.write(object.boundarySize);
            writer.write(object.nX);
            writer.write(object.nY);
            writer.write(object.timeStep);
            return;
        }

        template <typename Reader> static io::WriterData *deserialize(Reader &reader, void *storage)
        {

            std::string s = reader.template read<std::string>();
            io::BoundarySize bs = reader.template read<io::BoundarySize>();
            unsigned int nX = reader.template read<unsigned int>();
            unsigned int nY = reader.template read<unsigned int>();
            size_t ts = reader.template read<size_t>();
            io::WriterData *wd = ::new (storage) io::WriterData(s, bs, nX, nY, ts);

            return wd;
        }
    };
};

class io::Writer
{
  protected:
    //! file name of the data file
    const std::string fileName;

    //! Boundary layer size
    const BoundarySize boundarySize;

    //! dimensions of the grid in x- and y-direction.
    const unsigned int nX, nY;

    //! current time step
    size_t timeStep;

  public:
    /**
     * @param i_boundarySize size of the boundaries.
     */
    Writer(const std::string &i_fileName, const BoundarySize &i_boundarySize, int i_nX, int i_nY)
        : fileName(i_fileName), boundarySize(i_boundarySize), nX(i_nX), nY(i_nY), timeStep(0)
    {
    }

    Writer(WriterData &&wd)
        : fileName(std::move(wd.fileName)), boundarySize(wd.boundarySize), nX(wd.nX), nY(wd.nY), timeStep(wd.timeStep)
    {
    }

    Writer(WriterData &wd)
        : fileName(wd.fileName), boundarySize(wd.boundarySize), nX(wd.nX), nY(wd.nY), timeStep(wd.timeStep)
    {
    }

    virtual ~Writer(){};

    /**
     * Writes one time step
     *
     * @param i_h water heights at a given time step.
     * @param i_hu momentums in x-direction at a given time step.
     * @param i_hv momentums in y-direction at a given time step.
     * @param i_time simulation time of the time step.
     */
    virtual void writeTimeStep(const Float2D &i_b, const Float2D &i_h, const Float2D &i_hu, const Float2D &i_hv,
                               const int i_rank, float i_time) = 0;

    /* virtual void writeTimeStep(const Float2D &i_b, const Float2D &i_h,
                            const Float2D &i_hu, const Float2D &i_hv,
                            const Float2D &i_r, float i_time){
                                throw new std::runtime_error("implement this in vtk writer too");
                            }*/

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, const io::Writer &object)
        {

            writer.write(object.fileName);
            writer.write(object.boundarySize);
            writer.write(object.nX);
            writer.write(object.nY);
            writer.write(object.timeStep);
            // writer.write();
            return;
        }

        template <typename Reader> static WriterData *deserialize(Reader &reader, void *storage)
        {

            std::string fileName = reader.template read<std::string>();
            BoundarySize boundarySize = reader.template read<BoundarySize>();
            unsigned int nX = reader.template read<unsigned int>();
            unsigned int nY = reader.template read<unsigned int>();
            size_t timeStep = reader.template read<size_t>();

            WriterData *w = ::new (storage) WriterData(std::move(fileName), std::move(boundarySize), nX, nY, timeStep);
            return w;
        }
    };
};

#endif // WRITER_HH_
