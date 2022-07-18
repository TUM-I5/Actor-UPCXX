#pragma once

#include <string>
#include <vector>
extern "C"
{
#include <metis.h>
}
#include <sstream>

namespace orchutil
{

std::string compressedValsStr(const std::vector<idx_t> &rowIndex, const std::vector<idx_t> &colIndex);

} // namespace orchutil