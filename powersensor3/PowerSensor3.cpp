#include "PowerSensor3.h"

#include POWERSENSOR3_HEADER

namespace {
double Seconds(const PowerSensor3::State &first,
               const PowerSensor3::State &second) {
  return PowerSensor3::seconds(first, second);
}

double Joules(const PowerSensor3::State &first,
              const PowerSensor3::State &second, int sensor_id) {
  return PowerSensor3::Joules(first, second, sensor_id);
}

double Watt(const PowerSensor3::State &first, const PowerSensor3::State &second,
            int sensor_id) {
  return PowerSensor3::Watt(first, second, sensor_id);
}
}  // namespace

namespace pmt::powersensor3 {

template <typename PowerSensor, typename PowerSensorState>
class PowerSensor3Impl : public PowerSensor3 {
 public:
  PowerSensor3Impl(const char *device)
      : powersensor_(std::make_unique<PowerSensor>(device)),
        first_state_(powersensor_->read()) {}

  State GetState() override {
    const PowerSensorState powersensor_state = powersensor_->read();
    State state;
    state.timestamp_ = ::Seconds(first_state_, powersensor_state);
    state.name_[0] = "device";
    state.joules_[0] = ::Joules(first_state_, powersensor_state, -1);
    state.watt_[0] = ::Watt(first_state_, powersensor_state, -1);
    return state;
  }

 private:
  virtual const char *GetDumpFilename() { return "/tmp/pmt_powersensor3.out"; }

  virtual int GetMeasurementInterval() {
    return 1;  // milliseconds
  }

  std::unique_ptr<PowerSensor> powersensor_{};
  PowerSensorState first_state_{};
};

std::unique_ptr<PowerSensor3> PowerSensor3::Create(const char *device) {
  return std::make_unique<
      PowerSensor3Impl<::PowerSensor3::PowerSensor, ::PowerSensor3::State>>(
      device);
}

}  // end namespace pmt::powersensor3
