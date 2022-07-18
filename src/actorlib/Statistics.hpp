#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <optional>

/*
    Functions that calculate median. mean, variance
*/

namespace statistics
{
template <class T> bool checkNan(T t) { return std::isnan(t); }

// calculate median for a container of numerics, return {} if none
template <class Forwarditerator>
std::optional<double> calcMedian(Forwarditerator beg, Forwarditerator end, bool sorted = false)
{
    if (beg == end)
    {
        return {};
    }

    int elcount = 0;
    std::vector<double> vec;
    while (beg != end)
    {
        double x = (double)(*beg);
        vec.push_back(x);
        elcount += 1;
        ++beg;
    }

    if (sorted)
        std::sort(vec.begin(), vec.end());

    return vec[elcount >> 1];
}

// calculate meean for a container of numerics, return {} if none
template <class Forwarditerator> std::optional<double> calcMean(Forwarditerator beg, Forwarditerator end)
{
    if (beg == end)
    {
        return {};
    }

    int elcount = 0;
    double sum = 0.0;
    while (beg != end)
    {
        sum += (double)(*beg);
        elcount += 1;
        ++beg;
    }
    return sum / (double)elcount;
}

// calculate variance for a container of numerics, return {} if none
template <class Forwarditerator>
std::optional<double> calcVariance(Forwarditerator beg, Forwarditerator end, double mean)
{
    if (beg == end)
    {
        return {};
    }

    int elcount = 0;
    double sum = 0.0;

    while (beg != end)
    {
        elcount += 1;
        double diff = ((double)(*beg)) - mean;
        sum += diff * diff;
        ++beg;
    }

    return sum / (double)elcount;
}

// calculate relative variance for a container of numerics, return {} if none
template <class Forwarditerator> std::optional<double> calcRelativeVariance(Forwarditerator beg, Forwarditerator end)
{
    if (beg == end)
    {
        return {};
    }

    std::optional<double> meanopt = calcMean(beg, end);
    double mean = 0.0;

    if (meanopt.has_value())
    {
        mean = *meanopt;
    }
    else
    {
        return {};
    }

    std::optional<double> varianceopt = calcVariance(beg, end, mean);
    double variance = 0.0;
    if (varianceopt.has_value())
    {
        variance = *varianceopt;
    }
    else
    {
        return {};
    }

    if (mean == 0.0 && variance == 0.0)
    {
        return 0.0;
    }
    else
    {
        double rt = variance / (double)std::abs(mean);
        return variance / (double)std::abs(mean);
    }
}

// return true if |diff(val1,val2)| <= limit, checks if they are almost equal
template <class T> bool fuzzyEquality(T val1, T val2, T limit)
{
    if (limit < 0)
    {
        return val1 == val2;
    }
    else
    {
        return std::abs(val1 - val2) <= limit;
    }
}

} // namespace statistics