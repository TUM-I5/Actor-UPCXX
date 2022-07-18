cmake_minimum_required(VERSION 3.14)

project(ActorUpcxx)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
  "${CMAKE_SOURCE_DIR}/CMake-Aux/Modules/")

file(GLOB SQUARE_ROOT_EXAMPLE_SOURCES "src/examples/square_root/*.cpp")
file(GLOB PINGPONG_EXAMPLE_SOURCES "src/examples/pingpong/*.cpp")
file(GLOB_RECURSE TASK_TEST_SOURCES "src/examples/task_test/*.cpp")
file(GLOB_RECURSE ACTORLIB_SOURCES "src/actorlib/*.cpp")
file(GLOB_RECURSE POND_SOURCES "src/applications/pond/*.cpp")
file(GLOB TEST_INIT_SOURCES "src/examples/test_init/*.cpp")
file(GLOB SERIALIZATION_SOURCES "src/examples/serialization/*.cpp")

option(ENABLE_FILE_OUTPUT "ENABLE file output for Pond." OFF)
option(RANK_OUTPUT "Enables output for on which rank the actor is" OFF)
option(BUILD_RELEASE "Builds with optimizations enabled." ON)
option(ENABLE_O3_UPCXX_BACKEND "Builds with optimized UPC++ Library." ON)
option(UB_SANITIZER "sanitize undefined behaviour" OFF)

# option(ENABLE_PARALLEL_UPCXX_BACKEND "Links to parallel UPC++ Library backend."
# OFF)
option(ENABLE_MEMORY_SANITATION "Enable Address sanitation." OFF)
option(IS_CROSS_COMPILING
  "Specific settings for cross compilation. Skip tests where necessary."
  OFF)
option(
  REPORT_MAIN_ACTIONS
  "Prints verbose output when an actor is stopped,started, migrated, when an act call completes etc."
  OFF)
option(INVASION "Links the invasion distribution scenario" OFF)
option(TIME "Print runtimes of pond" OFF)
option(ANALYZE "Run with static analysis GNU/Clang" OFF)

option(THREAD_SANITIZER "Thread sanitizer you know" OFF)
option(LAZY_ACTIVATION "Lazy activation of pond actors" ON)
option(INTERRUPT "Interrupt the run in Intervals" OFF)

option(GLOBAL_MIGRATION "Migrate random actors and not only neighbors" OFF)

option(USE_ACTOR_TRIGGERS "Uses actor triggers" OFF)

option(STEAL_ONLY_ACTABLE_ACTOR "Steal an actor if it has tasks available" OFF)
option(STEAL_FROM_BUSY_RANK "Choose actor from the rank that has worked the most" ON)
option(MORE_LOCAL_VICTIM_CHOICE "Uses less recent data to have less computation on the victim rank" OFF)

option(ORDERED_OFFLOAD "Offloads from (n/2) overloaded ranks to (n/2) less overloaded ranks instead of max (1) to min (1)" OFF)

set(MIGRATION
  0
  CACHE
  STRING
  "0=No migration, 1=Allow bulk migration, 2=Allow asynchronous migration only, 3=Offloading"
)

message(STATUS "${MIGRATION}")
set_property(CACHE MIGRATION PROPERTY STRINGS "0" "1" "2" "3")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/CMake-Aux/Modules)
message(STATUS "${CMAKE_MODULE_PATH}")

if(ENABLE_FILE_OUTPUT)
  find_package(NetCDF)
endif()

find_package(OpenMP REQUIRED)
find_package(Git REQUIRED)
find_package(Metis REQUIRED)

if(THREAD_SANITIZER)
  set(CMAKE_CXX_FLAGS_DEBUG
    "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
  set(CMAKE_LINKER_FLAGS_DEBUG
    "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif()

add_compile_definitions(MIGRATION_TYPE=${MIGRATION})

if(LAZY_ACTIVATION)
  add_compile_definitions(LAZY_ACTIVATION)
endif()

if(USE_ACTOR_TRIGGERS)
  add_compile_definitions(USE_ACTOR_TRIGGERS)
endif()

if(MORE_LOCAL_VICTIM_CHOICE)
  add_compile_definitions(MORE_LOCAL_VICTIM_CHOICE)
endif()

if(GLOBAL_MIGRATION)
  add_compile_definitions(GLOBAL_MIGRATION)
endif()

if(STEAL_ONLY_ACTABLE_ACTOR)
  add_compile_definitions(STEAL_ONLY_ACTABLE_ACTOR)
endif()

if(STEAL_FROM_BUSY_RANK)
  add_compile_definitions(STEAL_FROM_BUSY_RANK)
endif()

if(BUILD_RELEASE)
  add_compile_definitions(NDEBUG)
endif()

if(TIME)
  add_compile_definitions(TIME)
endif()

if(REPORT_MAIN_ACTIONS)
  add_compile_definitions(REPORT_MAIN_ACTIONS)
endif()

if(INTERRUPT)
  add_compile_definitions(INTERRUPT)
endif()

if(ORDERED_OFFLOAD)
  add_compile_definitions(ORDERED_OFFLOAD)
endif()

if(IS_CROSS_COMPILING)
  set(METIS_TEST_RUNS "XXX")
endif()

# if(ENABLE_PARALLEL_UPCXX_BACKEND)
# message(STATUS "parallel backend")
# set(ENV{UPCXX_THREADMODE} "par")
# add_compile_definitions(PARALLEL)
# else()
message(STATUS "seq backend")
set(ENV{UPCXX_THREADMODE} "seq")
add_compile_definitions(SEQUENTIAL)

# endif()
if(ENABLE_O3_UPCXX_BACKEND)
  set(ENV{UPCXX_CODEMODE} "opt")
else()
  set(ENV{UPCXX_CODEMODE} "debug")
endif()

if(INVASION)
  add_compile_definitions(INVASION)
endif()

if(ANALYZE)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 10.0)
      set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} \
            -Wanalyzer-double-free -Wanalyzer-free-of-non-heap -Wanalyzer-malloc-leak -Wanalyzer-possible-null-argument \
            -Wanalyzer-possible-null-dereference \
            -Wanalyzer-null-argument \
            -Wanalyzer-null-dereference \
            -Wanalyzer-stale-setjmp-buffer \
            -Wanalyzer-tainted-array-index \
            -Wanalyzer-unsafe-call-within-signal-handler \
            -Wanalyzer-use-after-free \
            -Wanalyzer-use-of-pointer-in-stale-stack-frame")
    else()
      message(WARN "gcc version is less than 10, cannot add fanalyzer")
    endif()
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wthread-safety")
    set(CMAKE_CXX_CLANG_TIDY
      "clang-tidy;-checks=*,no-llvmlibc-*,-llvmlibc-callee-namespace,-llvmlibc-implementation-in-namespace \
            -fuchsia-*,-llvmlibc-*,-modernize-*,-fuchsia-default-arguments-calls, \
            -cppcoreguidelines-pro-bounds-array-to-pointer-decay,-hicpp-no-array-decay,unused-function, \
            -llvm-else-after-return,-readability-else-after-return")
  endif()
endif()

find_package(UPCXX)

# Get information about the git revision. the commit's SHA1, and whether the
# building workspace was dirty or not
execute_process(
  COMMAND "${GIT_EXECUTABLE}" describe --match=NeVeRmAtCh --always --abbrev=40
  --dirty
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  OUTPUT_VARIABLE GIT_SHA1
  ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

# the date of the commit
execute_process(
  COMMAND "${GIT_EXECUTABLE}" log -1 --format=%ad --date=local
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  OUTPUT_VARIABLE GIT_DATE
  ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

# the subject of the commit
execute_process(
  COMMAND "${GIT_EXECUTABLE}" log -1 --format=%s
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  OUTPUT_VARIABLE GIT_COMMIT_SUBJECT
  ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

add_library(actorlib ${ACTORLIB_SOURCES})
set(CMAKE_CXX_FLAGS "-Wno-unused-function ${CMAKE_CXX_FLAGS} ${UPCXX_CXXFLAGS}")

add_executable(task_test ${TASK_TEST_SOURCES})
add_executable(square_root_example ${SQUARE_ROOT_EXAMPLE_SOURCES})
add_executable(pingpong_example ${PINGPONG_EXAMPLE_SOURCES})
add_executable(pond ${POND_SOURCES})
add_executable(test_init ${TEST_INIT_SOURCES})
add_executable(serialization ${SERIALIZATION_SOURCES})

if(TRACE)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
    message(STATUS "Compiling with tracing")
    add_definitions(-DTRACE)

    include_directories($ENV{VT_ROOT}/include)
    link_directories($ENV{VT_ROOT}/slib)

    target_compile_options(actorlib PUBLIC -g)
    target_compile_options(pond PUBLIC -g)
    target_link_libraries(pond PUBLIC libVT.so)
    target_link_libraries(actorlib PUBLIC libVT.so)

    target_include_directories(actorlib PUBLIC $ENV{VT_ROOT}/include)
    target_link_directories(actorlib PUBLIC $ENV{VT_ROOT}/slib)
    target_include_directories(pond PUBLIC $ENV{VT_ROOT}/include)
    target_link_directories(pond PUBLIC $ENV{VT_ROOT}/slib)
    target_include_directories(task_test PUBLIC $ENV{VT_ROOT}/include)
    target_link_directories(task_test PUBLIC $ENV{VT_ROOT}/slib)
    target_include_directories(square_root_example PUBLIC $ENV{VT_ROOT}/include)
    target_link_directories(square_root_example PUBLIC $ENV{VT_ROOT}/slib)
    target_include_directories(pingpong_example PUBLIC $ENV{VT_ROOT}/include)
    target_link_directories(pingpong_example PUBLIC $ENV{VT_ROOT}/slib)
    target_include_directories(serialization PUBLIC $ENV{VT_ROOT}/include)
    target_link_directories(serialization PUBLIC $ENV{VT_ROOT}/slib)
    target_include_directories(test_init PUBLIC $ENV{VT_ROOT}/include)
    target_link_directories(test_init PUBLIC $ENV{VT_ROOT}/slib)
  else()
    message(WARN "You need intel compiler for tracing")
  endif()
else()
  message(STATUS "Compiling without tracing")
endif()

# if(ENABLE_PARALLEL_UPCXX_BACKEND)
# target_compile_definitions(actorlib PRIVATE ACTORLIB_USE_OPENMP_TASKS)
# target_link_libraries(actorlib PRIVATE OpenMP::OpenMP_CXX)
# message(STATUS "Actorlib Parallelization set to OpenMP Tasks")
# else()
target_compile_definitions(actorlib PRIVATE ACTORLIB_EXECUTION_STRATEGY_RANK)
message(STATUS "Actorlib Parallelization set to UPC++ Ranks")

# endif()
target_compile_definitions(actorlib PRIVATE ACTORLIB_GIT_REVISION=${GIT_SHA1})
target_compile_definitions(actorlib PRIVATE ACTORLIB_GIT_DATE=${GIT_DATE})
target_compile_definitions(
  actorlib PRIVATE ACTORLIB_UPCXX_GASNET_CONDUIT=${UPCXX_GASNET_CONDUIT_NAME})
target_compile_definitions(
  actorlib
  PRIVATE ACTORLIB_UPCXX_INSTALLATION=${UPCXX_GASNET_INSTALLATION_PATH})
target_compile_definitions(
  actorlib PRIVATE ACTORLIB_GIT_COMMIT_MSG=${GIT_COMMIT_SUBJECT})


if(UB_SANITIZER AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
target_compile_options(
  actorlib
  PUBLIC 
  -g
  -fno-omit-frame-pointer
  -fsanitize=undefined
  -fsanitize-address-use-after-scope
  -Wuninitialized
  -Winit-self
  -pedantic
  -Wall
  -Wextra
  -pedantic
  -fsanitize=bounds
  -fsanitize=alignment
  -fsanitize=object-size
  -fsanitize=vptr
  )
target_link_options(
    actorlib
    PUBLIC 
    -g
    -fno-omit-frame-pointer
    -fsanitize=undefined
    -fsanitize-address-use-after-scope
    -Wuninitialized
    -Winit-self
    -pedantic
    -Wall
    -Wextra
    -pedantic
    -fsanitize=bounds
    -fsanitize=alignment
    -fsanitize=object-size
    -fsanitize=vptr
    )
target_compile_options(
  pond
  PUBLIC
  -g
  -fno-omit-frame-pointer
  -fsanitize=undefined
  -fsanitize-address-use-after-scope
  -Wuninitialized
  -Winit-self
  -pedantic
  -Wall
  -Wextra
  -pedantic
  -fsanitize=bounds
  -fsanitize=alignment
  -fsanitize=object-size
  -fsanitize=vptr
  )
target_link_options(
  pond
  PUBLIC
  -g
  -fno-omit-frame-pointer
  -fsanitize=undefined
  -fsanitize-address-use-after-scope
  -Wuninitialized
  -Winit-self
  -pedantic
  -Wall
  -Wextra
  -pedantic
  -fsanitize=bounds
  -fsanitize=alignment
  -fsanitize=object-size
  -fsanitize=vptr
  )
endif()

if(ENABLE_MEMORY_SANITATION AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
  message(STATUS "Memory sanitation enabled. Linking appropriate libraries.")
  target_compile_options(
    actorlib
    PUBLIC -fno-omit-frame-pointer
    -fsanitize=address
    -fsanitize=undefined
    -fsanitize-address-use-after-scope
    -Wuninitialized
    -Winit-self
    -pedantic
    -Wall
    -Wextra
    -pedantic
    -g)
  target_link_libraries(
    actorlib PUBLIC -fno-omit-frame-pointer -fsanitize=address
    -fsanitize=undefined -g)

# For some old Ubuntu machines, you may need to use a different linker. Set
# the following variable: CMAKE_EXE_LINKER_FLAGS=-fuse-ld=gold
elseif(ENABLE_MEMORY_SANITATION AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
  message(
    STATUS
    "Full Memory sanitation only supported on GNU Environment. Trying to use intel fallback."
  )
  target_compile_options(actorlib PUBLIC -g -traceback -check-uninit)
  target_link_libraries(actorlib PUBLIC -g -traceback -check-uninit)
elseif(ENABLE_MEMORY_SANITATION AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  message(STATUS "Memotry sanitation enabled, with Clang")
  target_compile_options(
    actorlib
    PUBLIC -g
    -fno-omit-frame-pointer
    -fsanitize=leak
    -fsanitize-address-use-after-scope
    -fsanitize=address
    -fsanitize=undefined
    -fsanitize=float-divide-by-zero
    -fsanitize=float-cast-overflow
    -fno-sanitize=null
    -fno-sanitize=alignment)
  target_link_libraries(
    actorlib
    PUBLIC -fno-omit-frame-pointer
    -fsanitize=leak
    -fsanitize-address-use-after-scope
    -fsanitize=address
    -fsanitize=undefined
    -fsanitize=float-divide-by-zero
    -fsanitize=float-cast-overflow
    -fno-sanitize=null
    -fno-sanitize=alignment)
  target_compile_options(
    pond
    PUBLIC -g
    -fno-omit-frame-pointer
    -fsanitize=leak
    -fsanitize-address-use-after-scope
    -fsanitize=address
    -fsanitize=undefined
    -fsanitize=float-divide-by-zero
    -fsanitize=float-cast-overflow
    -fno-sanitize=null
    -fno-sanitize=alignment)
  target_link_libraries(
    pond
    PUBLIC -fno-omit-frame-pointer
    -fsanitize=leak
    -fsanitize-address-use-after-scope
    -fsanitize=address
    -fsanitize=undefined
    -fsanitize=float-divide-by-zero
    -fsanitize=float-cast-overflow
    -fno-sanitize=null
    -fno-sanitize=alignment)
endif()

target_compile_options(
  actorlib
  PUBLIC $<$<CXX_COMPILER_ID:AppleClang>:
  -Wall
  -Wextra>
  $<$<CXX_COMPILER_ID:Clang>:
  -Wall
  -Wextra>
  $<$<CXX_COMPILER_ID:Intel>:
  -w3
  -diag-disable:remark
  -Wall
  -Warray-bounds
  -Wchar-subscripts
  -Wcomment
  -Wenum-compare
  -Wformat
  -Wuninitialized
  -Wmaybe-uninitialized
  -Wmain
  -Wnarrowing
  -Wnonnull
  -Wparentheses
  -Wpointer-sign
  -Wreorder
  -Wreturn-type
  -Wsign-compare
  -Wsequence-point
  -Wtrigraphs
  -Wunused-function
  -Wunused-but-set-variable
  -Wunused-variable
  -Wwrite-strings>
  $<$<CXX_COMPILER_ID:GNU>:
  -Wall
  -Wextra
  -Wno-unused-parameter
  -Wcast-align
  -Wpointer-arith
  -Wnon-virtual-dtor
  -Woverloaded-virtual
  -g
  -pedantic>)

target_include_directories(task_test PRIVATE ${UPCXX_INCLUDE_DIRS})
target_compile_options(task_test PUBLIC ${UPCXX_CXXFLAGS})
target_compile_definitions(task_test PUBLIC ${UPCXX_DEFINITIONS})
target_include_directories(task_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_definitions(task_test PRIVATE USE_OPENMP)

target_include_directories(square_root_example
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(pingpong_example
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(test_init PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(serialization
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_include_directories(pond PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(
  pond PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/applications/pond)
target_compile_definitions(pond PRIVATE WAVE_PROPAGATION_SOLVER=3)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(pond PRIVATE -fstrict-aliasing -fargument-noalias)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
  target_compile_options(pond PRIVATE -fstrict-aliasing -fargument-noalias -std=c++17)
else(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(pond PRIVATE -fstrict-aliasing)
endif()

if(BUILD_RELEASE)
  message(STATUS "Configure Pond for Release mode")

  if(NOT IS_CROSS_COMPILING)
    target_compile_options(pond PRIVATE -march=native)
  else()
    target_compile_options(pond PRIVATE -xMIC-AVX512)
  endif()

  target_compile_options(pond PRIVATE -O3)
  target_compile_definitions(pond PRIVATE NDEBUG VECTORIZE)
  target_compile_options(actorlib PRIVATE -O3)

  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    target_compile_options(pond PRIVATE -qopt-report-phase=vec -qopt-report=2)
  endif()
else()
  message(STATUS "Configure Pond for Debug mode")
  target_compile_options(pond PRIVATE -g)
  target_compile_definitions(pond PRIVATE NDEBUG VECTORIZE)
  target_compile_options(actorlib PRIVATE -g)
endif()

if(NETCDF_FOUND)
  message(STATUS "Enabled NetCDF IO in Pond")
  target_compile_definitions(pond PRIVATE WRITENETCDF)
  target_include_directories(pond PRIVATE ${NETCDF_INCLUDE_DIRS})
  target_link_libraries(pond PRIVATE ${NETCDF_LIBRARIES})
else()
  message(WARNING "Enabled VTK IO (Slow, fall-back, as NetCDF is unavailable)")
  target_compile_definitions(pond PRIVATE WRITEVTK)
endif()

target_include_directories(actorlib PUBLIC ${METIS_INCLUDE_DIRS})
include_directories(actorlib PRIVATE /usr/lib/x86_64-linux-gnu/)
target_link_libraries(actorlib PUBLIC ${METIS_LIBRARIES})
target_include_directories(pond PRIVATE ${METIS_INCLUDE_DIRS})
target_link_libraries(pond PRIVATE ${METIS_LIBRARIES})
target_compile_definitions(pond PRIVATE METIS_PARTITIONING)

if(NOT ENABLE_FILE_OUTPUT)
  message(STATUS "Disable writing of time step files in Pond")
  target_compile_definitions(pond PRIVATE NOWRITE)
endif()

target_link_libraries(actorlib PUBLIC UPCXX::upcxx)
target_link_libraries(actorlib PRIVATE OpenMP::OpenMP_CXX)
target_link_libraries(task_test PRIVATE OpenMP::OpenMP_CXX)
target_link_libraries(task_test PUBLIC UPCXX::upcxx)
target_link_libraries(square_root_example PUBLIC actorlib)
target_link_libraries(square_root_example PUBLIC UPCXX::upcxx)
target_link_libraries(pingpong_example PUBLIC actorlib)
target_link_libraries(pingpong_example PUBLIC UPCXX::upcxx)
target_link_libraries(pond PRIVATE OpenMP::OpenMP_CXX)
target_link_libraries(pond PRIVATE actorlib)
target_link_libraries(test_init PUBLIC actorlib)
target_link_libraries(test_init PUBLIC UPCXX::upcxx)
target_link_libraries(serialization PUBLIC actorlib)
target_link_libraries(serialization PUBLIC UPCXX::upcxx)

set_property(TARGET actorlib PROPERTY POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
