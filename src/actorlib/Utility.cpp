#include "Utility.hpp"

double util::nanoToSec(uint64_t val) { return static_cast<double>(val) / 1000000000.0; }

std::pair<bool, uint64_t> util::addWOverflow(uint64_t a, uint64_t b)
{
    if (a <= std::numeric_limits<uint64_t>::max() - b)
    {
        return std::make_pair(true, a + b);
    }
    else
    {
        long long neg = b - std::numeric_limits<uint64_t>::max();
        neg += a;
        uint64_t uval = neg;
        return std::make_pair(true, uval);
    }
}

upcxx::future<> util::combineFutures(const std::vector<upcxx::future<>> &futs)
{
    if (futs.size() == 0)
    {
        return upcxx::make_future();
    }
    else if (futs.size() == 1)
    {
        return futs[0];
    }
    else if (futs.size() == 2)
    {
        upcxx::future<> f = upcxx::when_all(futs[0], futs[1]);
        return f;
    }
    else
    {
        upcxx::future<> f = upcxx::when_all(futs[0], futs[1]);
        for (unsigned int i = 2; i < futs.size(); i++)
        {
            f = upcxx::when_all(std::move(f), futs[i]);
        }

        return f;
    }
}

upcxx::future<> util::combineFutures(std::vector<upcxx::future<>> &&futs)
{
    switch (futs.size())
    {
    case 0:
    {
        return upcxx::make_future();
    }
    case 1:
    {
        return std::move(futs[0]);
    }
    case 2:
    {
        upcxx::future<> f = upcxx::when_all(std::move(futs[0]), std::move(futs[1]));
        return f;
    }
    case 3:
    {
        upcxx::future<> f = upcxx::when_all(std::move(futs[0]), std::move(futs[1]), std::move(futs[2]));
        return f;
    }
    case 4:
    {
        upcxx::future<> f =
            upcxx::when_all(std::move(futs[0]), std::move(futs[1]), std::move(futs[2]), std::move(futs[3]));
        return f;
    }
    default:
    {

        unsigned int b = 4;
        unsigned int p = 2;

        while (b < futs.size())
        {
            b *= 2;
            p += 1;
        }

        // unsigned int lenbefore = futs.size();
        unsigned int pad = b - futs.size();

        if (pad != 0)
        {
            futs.reserve(pad);
            for (unsigned int i = 0; i < pad; i++)
            {
                futs.push_back(upcxx::make_future());
            }
        }

        // For example b = 64, p = 6
        // Then we have to iterate += 2, then += 4 all the way to += 32, this means we iterate in 2^0 to 2^(p-1)
        for (unsigned int jump = 2; jump <= b; jump *= 2)
        {
            // std::cout << jump << " @ " << b << " @ " << futs.size() << " @ " << lenbefore;
            for (unsigned int i = 0; i < b; i += jump)
            {
                upcxx::future<> a = std::move(futs[i]);
                upcxx::future<> b = std::move(futs[i + (jump >> 1)]);
                // std::cout << " | " << i << " + " << i + (jump >> 1);
                futs[i] = upcxx::when_all(std::move(a), std::move(b));
            }
            // std::cout << " | " << std::endl;
        }

        return std::move(futs[0]);
    }
    }
}

void util::printConnectionTuple(const std::tuple<std::string, std::string, std::string, std::string> &tup)
{
    std::string s = std::get<0>(tup) + " : " + std::get<1>(tup) + " -> " + std::get<2>(tup) + " : " + std::get<3>(tup);
    std::cout << s << std::endl;
}

void util::printConnectionTuple(std::tuple<std::string, std::string, std::string, std::string> &&tup)
{
    std::string s = std::move(std::get<0>(tup)) + " : " + std::move(std::get<1>(tup)) + " -> " +
                    std::move(std::get<2>(tup)) + " : " + std::move(std::get<3>(tup));
    std::cout << s << std::endl;
}

std::string util::migrationList2Str(int rank, const std::unordered_map<std::string, upcxx::intrank_t> &migList)
{
    std::stringstream ss;
    ss << "Rank-" << rank << ": ";
    for (auto &el : migList)
    {
        ss << "( " << el.first << " -> " << el.second << " ) ";
    }

    return ss.str();
}

upcxx::intrank_t util::rank_n() { return upcxx::rank_n(); }

int64_t util::intPow(int base, unsigned int exp)
{
    if (base == 2)
    {
        int64_t tmp = static_cast<int64_t>(base) << exp;
        return tmp;
    }

    int64_t result = 1;
    int iter = 0;
    while (true)
    {
        ++iter;
        if (iter > too_long)
        {
            throw std::runtime_error("Loop too long!");
        }

        if (exp & 1)
        {
            result *= base;
        }

        exp >>= 1;

        if (!exp)
        {
            break;
        }

        base *= base;
    }

    return result;
}

double util::percentage(unsigned int part, unsigned int total)
{
    double fpart = static_cast<double>(part);
    double ftotal = static_cast<double>(total);

    if (ftotal == 0.0)
    {
        return 0.0;
    }

    double perc = 100.0 * (fpart / ftotal);

    return perc;
}

std::chrono::_V2::steady_clock::time_point util::timepoint() { return std::chrono::steady_clock::now(); }

double util::timerDifference(std::chrono::_V2::steady_clock::time_point &beg)
{
    auto nw = std::chrono::steady_clock::now();
    size_t dur = std::chrono::duration_cast<std::chrono::nanoseconds>(nw - beg).count();
    double durd = util::nanoToSec(dur);
    return durd;
}

void util::runWithTimer(std::function<void()> &foo, double &collector)
{
    auto t1 = util::timepoint();
    foo();
    double time = util::timerDifference(t1);
    collector += time;
}

void util::parseBooleanEnvVar(const std::string &key, bool &val)
{
    char *cp = std::getenv(key.c_str());

    if (cp != nullptr)
    {
        val = false;
        std::string s(cp);
        if (s == "1")
        {
            val = true;
        }
        // make everything else map to false
    }
}

void util::parseBooleanEnvVar(const char *key, bool &val)
{
    char *cp = std::getenv(key);

    if (cp != nullptr)
    {
        val = false;
        std::string s(cp);
        if (s == "1")
        {
            val = true;
        }
        // make everything else map to false
    }
}

void util::parseIntEnvVar(const char *key, int &val)
{
    char *internal_val = getenv(key);

    if (internal_val == nullptr)
    {
        val = -1.0;
    }
    else
    {
        int timeout_in_min = std::stoi(internal_val);
        val = timeout_in_min;
    }
}