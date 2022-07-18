#include "OrchestrationUtility.hpp"

std::string orchutil::compressedValsStr(const std::vector<idx_t> &rowIndex, const std::vector<idx_t> &colIndex)
{
    if (colIndex.size() == 0 || rowIndex.size() == 0)
    {
        return "";
    }

    std::stringstream ss;
    ss << "Col-Index: [";
    for (size_t i = 0; i < colIndex.size() - 1; i++)
    {
        ss << std::to_string(colIndex[i]);
        ss << ", ";
    }
    ss << std::to_string(colIndex[colIndex.size() - 1]);
    ss << "]";
    ss << std::endl;
    ss << "Row-Index: [";
    for (size_t i = 0; i < rowIndex.size() - 1; i++)
    {
        ss << std::to_string(rowIndex[i]);
        ss << ", ";
    }
    ss << std::to_string(rowIndex[rowIndex.size() - 1]);
    ss << "]";
    // ss << std::endl;
    return ss.str();
}