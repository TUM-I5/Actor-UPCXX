<h1 align="center">Actor-UPCXX</h1>

<p align="center">
  <img src="doc/animation_sc0_radial_dambreak.gif" width="250"/>
  <img src="doc/animation_sc2_pooldrop.gif" width="250"/>
  <img src="doc/animation_sc3_multidrop.gif" width="250"/>
</p>

## Table of Contents
- [Table of Contents](#table-of-contents)
- [General Information](#general-information)
- [Dependencies](#dependencies)
- [Setup](#setup)
- [Compile Options](#compile-options)
- [Runtime Options](#runtime-options)
- [Usage](#usage)

## General Information
Actor-UPCXX is a high performance computing library based on the actor model to enable the use of the actor model for HPC simulations.

## Dependencies
Actor library is written on [UPC++](https://bitbucket.org/berkeleylab/upcxx), a C++ template library that implements the asynchronous partial global address space (APGAS) model, which is built on top of [GASNet-EX](https://gasnet.lbl.gov/).

For the partitioning of the actors [METIS](http://glaros.dtc.umn.edu/gkhome/metis/metis/overview) graph partitioner is used.

## Setup
Actor-UPCXX library requires METIS and UPC++.

To compile UPC++, follow the UPC++ install instructions on this [link](https://bitbucket.org/berkeleylab/upcxx/wiki/INSTALL).

The Pond proxy application which is used to solve the Shallow Water Equations (SWE) additionally requires [NetCDF](https://www.unidata.ucar.edu/software/netcdf/) for file output if compiled with the option ENABLE_FILE_OUTPUT.

## Compile Options
Possible compile-time options to influence the behaviour of Actor-UPCXX. Experimental features are per default disabled.

- ENABLE_FILE_OUTPUT // Enables file output with netcdf 
- ENABLE_BUILD_RELEASE // Enables optimization for pond and actorlib
- ENABLE_O3_UPCXX_BACKEND // Enables optimized UPC++ backend
- REPORT_MAIN_ACTIONS // Prints the outputs of important steps (might be useful for debugging)
- LAZY_ACTIVATION // Enables lazy activation in pond application
- MIGRATION // "0=No migration, 1=Global Repartitioning, (work in progress) global synchronized partitioning through METIS with using runtime of actors as a cost model, 2=Stealing, allows asynchronous stealing where underworking ranks steal actors from other overlaoded ranks, 3=Offloading, allows overlaoded ranks to asynchronously offload actors to underloaded ranks"
- GLOBAL_MIGRATION // Enables ranks can steal from any rank, otherwise only from neighboring ranks (actors to which they directly communicate with)
- STEAL_FROM_BUSY_RANK // Steals from the rank that has worked the most at the time of execution (from set of allowed ranks for the migration)
- TRACE // Enables user defined regions of tracing with intel VTUNE
- INVASION // Partitions the actors on the ranks [0;n/2] instead of [0;n] to create a static imbalance
- TIME // Enables tracing of the time spent in various actions of pond actors 
- ANALYZE // Enables static analysis tools of Clang and GCC
- USE_ACTOR_TRIGGER // Uses additional structs called actor triggers to track the amount of tasks each actor has (experimental)
- STEAL_ONLY_ACTABLE_ACTOR // Denies steal request when the actors do not have tasks (experimental)
- MORE_LOCAL_VICTIM_CHOICE // Changes the victim choice procedure to use less communication, but might increase the number of failed steal attempts, enabling it is suggested
- ORDERED OFFLOAD // Changes offloading strcuture from sending actors from the most overloaded rank to most underloaded rank to an approach where every overloaded rank (higher load than average) to every underloaded rank (lower load than average) in a ordered and paired fashion. (experimental)

## Runtime Options
Possible runtime options to influence the behaviour of Actor-UPCXX. Default values are shown next to variable. Experimental features are per default disabled. All environment variables are integers. Some accept only 0 or 1 as an argument. 1 enables the given feature, 0 disables it.

- GOING_AWAY_LIMIT=Unlimited // Any integer, governs the amount of actors a rank can lose (stolen, or given for offloading) simultaneously. Although default is undefined, setting it a small integer (1) is suggested
- ORDER_VICTIMS=0 // 0 or 1, Order victims depending on their cost if MORE_LOCAL_VICTIM choice is disabled. (Experimental)
- STEAL_DURING_TERMINATION=0 // O or 1, Enables stealing if the set of ranks available for stealing or offloading have a temrinated actor. Suggested to enable when actors will terminate at the same time.
- USE_TIME_SPENT=1 // 0 or 1, Enables using the time spent by actors as the workload
- CONTIGIOUS_MIGRATION=0 // 0 or 1, If GLOBAL_MIGRATION is disable, enforces the actors to be offloaded or stolen to neighbor to recipient rank. (They need to directly communicate with the recipient rank) (Experimental)
- RMA_TASKCOUNT=0 // 0 or 1, Only relevant if USE_TIME_SPENT is disabled. The amount of task are read from a variable in the global address space, if disable they are computed by iterating over the local actors task counts. (Experimental)
- SLOW_INIT=0 // 0 or 1, Enables synchronous calls during initialization
- STEAL_COOLDOWN=1 // 0 or 1, Enforces a cooldown after each successful or unsuccessful steal attempt
- SAMPLE_OUTPUT=1 // 0 or 1, Print output in 2 minute intervals regarding the statistics of each rank. Does not involve communication.
- SAMPLE_BARRIER=0 // 0 or 1, Employs a barrier after the sampling output if enables
- SAMPLE_INTERVAL=SECS // Any positive integer, changes the interval between two samples
- EARLY_SAMPLE=0 // 0 or 1, If set two one, two additional samples are done at time points 5 and 10.
- TIMEOUT_IN_MINUTES=-1 // Any integer, employs a timeout in form a runtime error injected into every rank after n minutes. Disregarded if it is smaller than 1
- SLOWDOWN=0 // 0 or 1, Enables node slowdown, SLOWDOWN_* variables are disregarded if SLOWDOWN is disabled. Slowdown, artificially slows down the execution speed of actors. During the time interval [SLOWDOWN_TIME_BEGIN;SLOWDOWN_TIME_END] the ranks with Id inside the interval  [SLOWDOWN_RANK_BEGIN;SLOWDOWN_RANK_END) require triple the time to execute actors
- SLOWDOWN_RANK_BEGIN=1 // Any integer, First rank that will be slowed down
- SLOWDOWN_RANK_END=-1 // Any integer, Last rank+1 that will be slowed down
- SLOWDOWN_TIME_BEGIN=-1 // Any integer, Begin time of the slow down in minutes
- SLOWDOWN_TIME_END=-1 // Any integer, End time of the slow down in minutes


## Usage
To compile pond and actorlib and pond, with example options:

```bash
export UPCXX_DIR=/path/to/upcxx

mkdir build
cd build 
# Compiler should be the same compiler used to compile UPC++.
# MPI is used only for the job launch, due to convenience but it is not necessary.
cmake \
-DCMAKE_C_COMPILER=mpicc \
-DCMAKE_CXX_COMPILER=mpicxx \
-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
-DBUILD_RELEASE=ON \
-DENABLE_O3_UPCXX_BACKEND=ON \
-DMIGRATION=2 \
-DLAZY_ACTIVATION=ON \
-DSTEAL_FROM_BUSY_RANK=ON \
-DGLOBAL_MIGRATION=OFF \
..
```

To run pond and example configuration may look like:

```bash
export UPCXX_DIR=/path/to/upcxx
export PATH=$UPCXX_DIR:$PATH

# Check pond -h for details.
# Pond asks for -c and -o options also when file output is disabled.
# If file output is disabled, -c and -o options are disregarded.
upcxx-run -n 6 -shared-heap 128MB ./build/pond -x 6000 -y 6000 -p 250 -c 10 --scenario 3 -o /tmp/o -e 0.5
```
