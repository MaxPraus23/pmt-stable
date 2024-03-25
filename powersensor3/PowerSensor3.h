#ifndef PMT_POWERSENSOR3_H_
#define PMT_POWERSENSOR3_H_

#include <memory>
#include <string>

#include "common/PMT.h"

namespace pmt::powersensor3 {

class PowerSensor3 : public PMT {
 public:
  inline static std::string name = "powersensor3";
  static std::unique_ptr<PowerSensor3> Create(
      const char *device = default_device().c_str());
  static std::string default_device() { return "/dev/ttyACM0"; }
  virtual std::string GetPairName(int pairID) { return ""; };
  virtual bool IsInUse(int pairID) { return false; };
  virtual State Read(int pairID) { return PMT::Read(); };
};

}  // end namespace pmt::powersensor3

#endif  // PMT_POWERSENSOR3_H_
