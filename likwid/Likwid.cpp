#include "Likwid.h"

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <vector>

#if defined(HAVE_LIKWID)
#include <likwid.h>

// Likwid provides an ENERGY performance group (likwid-perfctr -g ENERGY -H)
// for CPUs. We use this group instead of hard-coded performance counters.
#define EVENTSET "ENERGY"

#endif

namespace pmt {
namespace likwid {

class Likwid_ : public Likwid {
 public:
  Likwid_();
  ~Likwid_();

  State GetState() override;

  virtual const char *GetDumpFilename() override {
    return "/tmp/pmt_likwid.out";
  }

  virtual int GetMeasurementInterval() override {
    return 100;  // milliseconds
  }

 private:
  std::vector<double> GetMeasurements();

  double previous_time;
  int nr_groups, nr_events;
  int nr_threads_group;
  std::vector<int> relevant_event_ids;
  double joulesTotal = 0;
};

std::unique_ptr<Likwid> Likwid::Create() { return std::make_unique<Likwid_>(); }

Likwid_::Likwid_() {
#if defined(HAVE_LIKWID)
  // Load the topology module
  if (topology_init() < 0) {
    std::cerr << "Failed to initialize LIKWID's topology module" << std::endl;
    exit(EXIT_FAILURE);
  }

  // CpuInfo_t contains global information like name, CPU family, ...
  CpuInfo_t info = get_cpuInfo();

  // CpuTopology_t contains information about the topology of the CPUs
  CpuTopology_t topo = get_cpuTopology();

  // Get number of sockets
  int nr_sockets = topo->numSockets;

  // Get number of hardware threads
  int nr_threads = topo->numHWThreads;

  // Number of threads per socket
  int nr_threads_socket = nr_threads / nr_sockets;

  // Measure only first cpu in socket
  nr_threads_socket = 1;

  // Initialize performance monitoring per socket
  nr_groups = nr_sockets;
  nr_threads_group = nr_threads_socket;
  for (int socket = 0; socket < nr_sockets; socket++) {
    // Fill cpu array with acpi id's of threads
    int cpus[nr_threads_socket];
    cpus[0] = -1;
    int count = 0;
    for (int i = 0; i < nr_threads / nr_sockets; i++) {
      if (socket == topo->threadPool[i].packageId &&
          topo->threadPool[i].inCpuSet) {
        cpus[count++] = topo->threadPool[i].apicId;
        break;
      }
    }
    if (cpus[0] == -1) {
      // We don't have any active CPUs in the current socket, so we skip it
      nr_groups--;
      continue;
    }

    // Initialize the perfmon module
    if (perfmon_init(nr_threads_socket, cpus) < 0) {
      std::cerr << "Failed to initialize LIKWID's performance monitoring module"
                << std::endl;
      topology_finalize();
      exit(EXIT_FAILURE);
    }

    // Add eventset string to the perfmon module
    // Assumption: groupId is equal to socket number
    int groupId = perfmon_addEventSet((char *)EVENTSET);
    if (groupId < 0) {
      std::cerr << "Failed to add event string " << EVENTSET
                << " to LIKWID's performance monitoring module" << std::endl;
      perfmon_finalize();
      topology_finalize();
      exit(EXIT_FAILURE);
    }

    // Setup the eventset identified by group ID (groupId)
    if (perfmon_setupCounters(groupId) < 0) {
      std::cerr << "Failed to setup group " << groupId
                << " in LIKWID's performance monitoring module" << std::endl;
      perfmon_finalize();
      topology_finalize();
      exit(EXIT_FAILURE);
    }

    // Start all counters in the previously set up event set
    if (perfmon_startCounters() < 0) {
      std::cerr << "Failed to start counters for group " << groupId
                << std::endl;
      perfmon_finalize();
      topology_finalize();
      exit(EXIT_FAILURE);
    }
  }  // end for socket

  if (nr_groups > 0) {
    nr_events = perfmon_getNumberOfEvents(0);
    for (int i = 0; i < nr_events; i++) {
      char *event_name = perfmon_getEventName(0, i);
      // Energy counters are different across CPU vendors and generations, but
      // they do contain the word "ENERGY". We store the event IDs to only
      // query those in GetMeasurements()
      if (event_name != nullptr && strstr(event_name, "ENERGY") != nullptr) {
        relevant_event_ids.push_back(i);
      }
    }
  }

  previous_time = GetTime();
#endif
}  // end constructor

Likwid_::~Likwid_() {
#if defined(HAVE_LIKWID)
  perfmon_stopCounters();
  perfmon_finalize();
  topology_finalize();
#endif
}  // end destructor

State Likwid_::GetState() {
  State state(nr_groups + 1);
  state.timestamp_ = GetTime();

  double duration = state.timestamp_ - previous_time;
  double total_watts = 0;
  std::vector<double> current_measurements = GetMeasurements();
  for (int i = 0; i < nr_groups; i++) {
    joulesTotal += current_measurements[i];
    state.joules_[i + 1] = current_measurements[i];
    state.watt_[i + 1] = current_measurements[i] / duration;
    total_watts += state.watt_[i + 1];
  }

  state.joules_[0] = static_cast<float>(joulesTotal);
  state.watt_[0] = total_watts;
  previous_time = state.timestamp_;

  return state;
}

std::vector<double> Likwid_::GetMeasurements() {
  std::vector<double> measurements(nr_groups);

#if defined(HAVE_LIKWID)
  for (int groupId = 0; groupId < nr_groups; groupId++) {
    // Read performance counters
    if (perfmon_readGroupCounters(groupId) != 0) {
      std::cerr << "perfmon_readCounters() fails" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Accumulate results
    // Only for first thread in group
    int threadId = 0;
    // Here we only read the energy counters. The event ids are identical
    // across all groups.
    for (int eventId : relevant_event_ids) {
      double result = perfmon_getLastResult(groupId, eventId, threadId);
      measurements[groupId] += result;
    }
  }
#endif

  return measurements;
}

}  // end namespace likwid
}  // end namespace pmt
