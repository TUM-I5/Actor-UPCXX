/**
 * @file
 * This file is part of Pond.
 *
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

#include "scenario/NetCdfData.hpp"

#ifdef WRITENETCDF

#include "util/Logger.hh"

#include <netcdf.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>

NetCdfData::NetCdfData(const std::string &fileName) : fileName(fileName), isLoaded(true)
{
    int fileHandle = 0;
    int returnCode = nc_open(fileName.c_str(), NC_WRITE, &fileHandle);
    bool isValid = returnCode == NC_NOERR;

#ifndef NDEBUG
    std::cout << "NetCDF data file " << fileName << " is " << ((isValid) ? "" : "in") << "valid." << std::endl;
#endif
    int numVars;
    nc_inq(fileHandle, nullptr, &numVars, nullptr, nullptr);

#ifndef NDEBUG
    std::cout << "NetCDF data file has " << numVars << " variables." << std::endl;

#endif

    int numberOfDimensions;
    int numberOfVariables;
    int numberOfGlobalAttributes;
    int unlimitedDimensionNum;
    returnCode =
        nc_inq(fileHandle, &numberOfDimensions, &numberOfVariables, &numberOfGlobalAttributes, &unlimitedDimensionNum);

    isLoaded = isValid;

#ifndef NDEBUG
    for (int i = 0; i < numberOfVariables; i++)
    {
        char varName[32];
        returnCode = nc_inq_varname(fileHandle, i, varName);
        std::cout << "Variable " << i << ": " << varName << std::endl;
    }
#endif

    char varName[32];
    returnCode = nc_inq_varname(fileHandle, 0, varName);
    if (std::strcmp(varName, "elevation"))
    {

#ifndef NDEBUG
        std::cout << "Found old grid-based format used in the TUM Tsunami scenarios" << std::endl;
#endif

        returnCode = nc_inq_dimlen(fileHandle, 0, &xSize);
        returnCode = nc_inq_dimlen(fileHandle, 1, &ySize);

        this->xCoordinates.resize(xSize);
        this->yCoordinates.resize(ySize);
        size_t totalSize = xSize * ySize;
        this->data.resize(totalSize);

#ifndef NDEBUG
        std::cout << "Loading X coords: ";
#endif
        int xLoaded = nc_get_var_double(fileHandle, 0, xCoordinates.data());
        isLoaded &= (xLoaded == NC_NOERR);
#ifndef NDEBUG
        std::cout << ((xLoaded == NC_NOERR) ? "OK" : "FAIL") << " Loading Y coords: ";
#endif
        int yLoaded = nc_get_var_double(fileHandle, 1, yCoordinates.data());
        isLoaded &= (yLoaded == NC_NOERR);
#ifndef NDEBUG
        std::cout << ((yLoaded == NC_NOERR) ? "OK" : "FAIL") << " Loading Bathymetry information: ";
#endif
        int bathyLoaded = nc_get_var_float(fileHandle, 2, data.data());
        isLoaded &= (bathyLoaded == NC_NOERR);
#ifndef NDEBUG
        std::cout << ((bathyLoaded == NC_NOERR) ? "OK" : "FAIL") << std::endl;
#endif
    }
    else
    {
#ifndef NDEBUG
        std::cout << "Found GEBCO Style NetCDF format." << std::endl;
#endif
        int elevationID;
        int elevationDims;
        nc_type varType;
        int numElevationAttributes;
        int returnCode = nc_inq_varid(fileHandle, "elevation", &elevationID);
        if (returnCode == NC_EBADID)
        {
#ifndef PRINT_LEVEL_ERROR
            std::cout << "Found bad format. Variable \"elevation\" not found." << std::endl;
#endif
            exit(NC_EBADID);
        }
        returnCode = nc_inq_varndims(fileHandle, elevationID, &elevationDims);
        if (returnCode == NC_EBADID)
        {
#ifndef PRINT_LEVEL_ERROR
            std::cout << "Error. Unable to enquire the Variable dimensionality." << std::endl;
#endif
            exit(returnCode);
        }
        auto dimensions = std::make_unique<int[]>(elevationDims);
        returnCode =
            nc_inq_var(fileHandle, elevationID, nullptr, &varType, nullptr, dimensions.get(), &numElevationAttributes);
        // char names[elevationDims][32];
        auto names = std::make_unique<char[][32]>(elevationDims);
        // size_t dimSizes[elevationDims];
        auto dimSizes = std::make_unique<size_t[]>(elevationDims);
        for (int i = 0; i < elevationDims; i++)
        {
            returnCode = nc_inq_dim(fileHandle, dimensions[i], names[i], (dimSizes.get() + i));
        }

#ifndef NDEBUG
        std::cout << "Variable information for variable \"elevation\":" << std::endl;
        std::cout << " - VarID: " << elevationID << std::endl;
        std::cout << " - Variable dimensions: " << elevationDims << std::endl;
        std::cout << " - Variable data type: "
                  << "float" << std::endl;
        std::cout << " - Dimension contents:" << std::endl;
        for (int i = 0; i < elevationDims; i++)
        {
            std::cout << "\t[" << i << "]: " << dimensions[i] << "\tName: " << names[i] << "\tSize: " << dimSizes[i]
                      << std::endl;
        }
        std::cout << std::endl;
#endif

        if (elevationDims != 2)
        {
#ifndef PRINT_LEVEL_ERROR
            std::cout << "Error. Variable \"elevation\" must be 2D." << std::endl;
#endif
            exit(1);
        }

        xSize = dimSizes[1];
        ySize = dimSizes[0];

        this->xCoordinates.resize(xSize);
        this->yCoordinates.resize(ySize);
        this->data.resize(xSize * ySize);

#ifndef NDEBUG
        std::cout << "Loading X coords: ";
#endif
        int xLoaded = nc_get_var_double(fileHandle, 2, xCoordinates.data());
        isLoaded &= (xLoaded == NC_NOERR);
#ifndef NDEBUG
        std::cout << ((xLoaded == NC_NOERR) ? "OK" : "FAIL") << " Loading Y coords: ";
#endif
        int yLoaded = nc_get_var_double(fileHandle, 1, yCoordinates.data());
        isLoaded &= (yLoaded == NC_NOERR);
#ifndef NDEBUG
        std::cout << ((yLoaded == NC_NOERR) ? "OK" : "FAIL") << " Loading Bathymetry information: ";
#endif
        int bathyLoaded = nc_get_var_float(fileHandle, elevationID, data.data());
        isLoaded &= (bathyLoaded == NC_NOERR);
#ifndef NDEBUG
        std::cout << ((bathyLoaded == NC_NOERR) ? "OK" : "FAIL") << std::endl;
#endif
        // Next, convert them to actual distances. The GEBCO format downloadable
        // from the Website stores latitude/longitude parameters. From that, we
        // can calculate approximate distances. We have to project the
        // coordinates. I will choose a simple projection for now. Distance
        // between degrees of longitude is assumed to remain constant 111km.
        // (This implies a mercator-globe-like projection) Distance between
        // degrees of latitude is assumed to remain constant at 111km

        double minLatitude = yCoordinates[0];
        double minLongitude = xCoordinates[0];

#ifndef NDEBUG
        double maxLongitude = xCoordinates[xSize - 1];
        double maxLatitude = yCoordinates[ySize - 1];
        double minDistance = cos(minLatitude) * 111000;
        double maxDistance = cos(maxLatitude) * 111000;
        double avgDistance = ((minDistance + maxDistance) / 2.0);
        std::cout << "Read Geo Coordinates." << std::endl;
        std::cout << " - Latitude:  " << minLatitude << " - " << maxLatitude << std::endl;
        std::cout << " - Longitude: " << minLongitude << " - " << maxLongitude << std::endl;
        std::cout << " - Scenario Y Size: " << ((maxLatitude - minLatitude) * 111000) / 1000.0 << "km" << std::endl;
        std::cout << " - Scenario X Size: " << ((maxLongitude - minLongitude) * avgDistance) / 1000.0 << "km"
                  << std::endl;
#endif

        for (size_t i = 0; i < xSize; i++)
        {
            xCoordinates[i] = (xCoordinates[i] - minLongitude) * /*avgDistance*/ 111000;
        }

        for (size_t i = 0; i < ySize; i++)
        {
            yCoordinates[i] = (yCoordinates[i] - minLatitude) * 111000;
        }
    }
#ifndef NDEBUG
    std::cout << "Finished loading NetCdf data file " << fileName << std::endl;
#endif
}

bool NetCdfData::isLoadedSuccessfully() { return isLoaded; }

size_t NetCdfData::getXSize() { return xSize; }

size_t NetCdfData::getYSize() { return ySize; }

SimulationArea NetCdfData::getSimulationArea()
{
    return SimulationArea(0.0f, xCoordinates[xCoordinates.size() - 1] - xCoordinates[0], 0.0f,
                          yCoordinates[yCoordinates.size() - 1] - yCoordinates[0]);
}

std::vector<float> &&NetCdfData::exportData() { return std::move(this->data); }

#endif // WRITENETCDF
