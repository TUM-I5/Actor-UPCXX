#include "ActorTriggers.hpp"

std::ostream &operator<<(std::ostream &os, const ActorTriggers &obj)
{
    std::string mapstr = util::map2Str(obj.getTriggersAsMap());
    os << obj.name << "has triggers: " << mapstr;
    return os;
}