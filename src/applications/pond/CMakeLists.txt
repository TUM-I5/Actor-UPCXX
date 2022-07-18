add_subdirectory(actor)
add_subdirectory(block)
add_subdirectory(orchestration)
add_subdirectory(scenario)
add_subdirectory(solver)
add_subdirectory(util)
add_subdirectory(writer)

target_sources(pond
    PRIVATE
    main.cpp
)
