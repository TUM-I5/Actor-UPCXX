target_sources(
    actorlib
    
    PUBLIC
    AbstractInPort.hpp
    AbstractOutPort.hpp
    Actor.hpp
    ActorGraph.hpp
    ActorRegistration.hpp
    Channel.hpp
    InPort.hpp
    OutPort.hpp
    config.hpp
    
    PRIVATE
    AbstractInPort.cpp
    AbstractOutPort.cpp
    Actor.cpp
    ActorGraph.cpp
    ActorRegistration.cpp
    config.cpp
)
