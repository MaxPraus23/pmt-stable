#ifndef LIKWIDIMPL_PMT_H
#define LIKWIDIMPL_PMT_H

#include <string>
#include <vector>

#include "Likwid.h"
#include "common/pmt.h"

namespace pmt::likwid {

class LikwidImpl : public Likwid {
 public:
  LikwidImpl(std::string event_group_name);
  ~LikwidImpl();

  State GetState() override;

  virtual const char *GetDumpFilename() override {
    return "/tmp/pmt_likwid.out";
  }

  virtual int GetMeasurementInterval() override {
    return 100;  // milliseconds
  }

 private:
  std::string event_group;
  std::vector<double> GetMeasurements();

  double previous_time;
  int nr_groups, nr_events;
  int nr_threads_group;
  std::vector<int> relevant_event_ids;
  double joulesTotal = 0;
};
}  // namespace pmt::likwid

#endif  // LIKWIDIMPL_PMT_H
