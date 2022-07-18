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

#ifndef VTKWRITER_HH_
#define VTKWRITER_HH_

#include "writer/Writer.hh"
#include <sstream>

namespace io
{
class VtkWriter;
}

class io::VtkWriter final : public io::Writer
{
  private:
    //! cell size
    float dX, dY;

    float offsetX, offsetY;

  public:
    VtkWriter(const std::string &i_fileName, const BoundarySize &i_boundarySize, int i_nX, int i_nY, float i_dX,
              float i_dY, int i_offsetX = 0, int i_offsetY = 0);

    VtkWriter(WriterData &&wd, float i_dX, float i_dY, int i_offsetX = 0, int i_offsetY = 0);

    // VtkWriter( const io::VtkWriter& other);
    VtkWriter(io::VtkWriter &&other);
    VtkWriter(const io::VtkWriter &other);
    ~VtkWriter() override final;

    // writes the unknowns at a given time step to a vtk file
    void writeTimeStep(const Float2D &i_b, const Float2D &i_h, const Float2D &i_hu, const Float2D &i_hv,
                       const int i_rank, float i_time) override final;

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, const VtkWriter &object)
        {
            writer.write(dynamic_cast<io::Writer const &>(object));
            writer.write(object.dX);
            writer.write(object.dY);
            writer.write(object.offsetX);
            writer.write(object.offsetY);

            return;
        }

        template <typename Reader> static VtkWriter *deserialize(Reader &reader, void *storage)
        {
            WriterData wd = reader.template read<WriterData>();
            float dX = reader.template read<float>();
            float dY = reader.template read<float>();
            float offsetX = reader.template read<float>();
            float offsetY = reader.template read<float>();
            VtkWriter *vtkw = ::new (storage) VtkWriter(std::move(wd), dX, dY, offsetX, offsetY);
            return vtkw;
        }
    };

  private:
    std::string generateFileName()
    {
        std::ostringstream name;

        name << fileName << '.' << offsetX << '-' << offsetY << '.' << timeStep << ".vts";
        return name.str();
    }
};

#endif // VTKWRITER_HH_
