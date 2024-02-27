#include <pybind11/pybind11.h>
#include <bytearrayobject.h>  // must be included after pybind11.h
#include <pybind11/detail/common.h>
#include <pybind11/detail/descr.h>
#include <pybind11/pytypes.h>

#include <../common/pmt.h>

#ifdef BUILD_CRAY
#include <../cray/Cray.h>
#endif

#ifdef BUILD_POWERSENSOR2
#include <../powersensor2/PowerSensor2.h>
#endif

#ifdef BUILD_POWERSENSOR3
#include <../powersensor3/PowerSensor3.h>
#endif

#include <../dummy/Dummy.h>
#ifdef BUILD_TEGRA
#include <../tegra/Tegra.h>
#endif

#ifdef BUILD_NVML
#include <../nvml/NVML.h>
#endif

#ifdef BUILD_RAPL
#include <../rapl/Rapl.h>
#endif

#ifdef BUILD_ROCM
#include <../rocm/ROCM.h>
#endif

#ifdef BUILD_XILINX
#include <../xilinx/Xilinx.h>
#endif

namespace py = pybind11;

PYBIND11_MODULE(pypmt, m) {
  m.doc() = "libpmt python bindings";

  py::class_<pmt::PMT>(m, "PMT")
      .def("seconds", &pmt::PMT::seconds, "Get elapsed time")
      .def("joules", &pmt::PMT::joules, "Get energy consumption")
      .def("watts", &pmt::PMT::watts, "Get average power consumption");

  py::class_<pmt::State>(m, "State");

#ifdef BUILD_CRAY
  py::class_<pmt::cray::Cray>(m, "Cray")
      .def("create", &pmt::cray::Cray::Create)
      .def("read", &pmt::cray::Cray::Read)
      .def("startDump", &pmt::cray::Cray::StartDump)
      .def("stopDump", &pmt::cray::Cray::StopDump);
#endif

#ifdef BUILD_POWERSENSOR2
  py::class_<pmt::powersensor2::PowerSensor2>(m, "PowerSensor2")
      .def("create", &pmt::powersensor2::PowerSensor2::Create)
      .def("read", &pmt::powersensor2::PowerSensor2::Read)
      .def("startDump", &pmt::powersensor2::PowerSensor2::StartDump)
      .def("stopDump", &pmt::powersensor2::PowerSensor2::StopDump);
#endif

#ifdef BUILD_POWERSENSOR3
  py::class_<pmt::powersensor3::PowerSensor3>(m, "PowerSensor3")
      .def("create", &pmt::powersensor3::PowerSensor3::Create)
      .def("read", &pmt::powersensor3::PowerSensor3::Read)
      .def("startDump", &pmt::powersensor3::PowerSensor3::StartDump)
      .def("stopDump", &pmt::powersensor3::PowerSensor3::StopDump)
      .def("getPairName", &pmt::powersensor3::PowerSensor3::GetPairName)
      .def("isInUse", &pmt::powersensor3::PowerSensor3::IsInUse);
#endif

  py::class_<pmt::Dummy>(m, "Dummy")
      .def("create", &pmt::Dummy::Create)
      .def("read", &pmt::Dummy::Read)
      .def("startDump", &pmt::Dummy::StartDump)
      .def("stopDump", &pmt::Dummy::StopDump);

#ifdef BUILD_XILINX
  py::class_<pmt::tegra::Tegra>(m, "tegra")
      .def("create", &pmt::tegra::Tegra::Create)
      .def("read", &pmt::tegra::Tegra::Read)
      .def("startDump", &pmt::tegra::Tegra::StartDump)
      .def("stopDump", &pmt::tegra::Tegra::StopDump);
#endif

#ifdef BUILD_NVML
  py::class_<pmt::nvml::NVML>(m, "NVML")
      .def("create", &pmt::nvml::NVML::Create)
      .def("read", &pmt::nvml::NVML::Read)
      .def("startDump", &pmt::nvml::NVML::StartDump)
      .def("StopDump", &pmt::nvml::NVML::StopDump);
#endif

#ifdef BUILD_RAPL
  py::class_<pmt::rapl::Rapl>(m, "Rapl")
      .def("create", &pmt::rapl::Rapl::Create)
      .def("read", &pmt::rapl::Rapl::Read)
      .def("startDump", &pmt::rapl::Rapl::StartDump)
      .def("stopDump", &pmt::rapl::Rapl::StopDump);
#endif

#ifdef BUILD_ROCM
  py::class_<pmt::rocm::ROCM>(m, "ROCM")
      .def("create", &pmt::rocm::ROCM::Create)
      .def("read", &pmt::rocm::ROCM::Read)
      .def("startDump", &pmt::rocm::ROCM::StartDump)
      .def("stopDump", &pmt::rocm::ROCM::StopDump);
#endif

#ifdef BUILD_XILINX
  py::class_<pmt::xilinx::Xilinx>(m, "Xilinx")
      .def("create", &pmt::xilinx::Xilinx::Create)
      .def("read", &pmt::xilinx::Xilinx::Read)
      .def("startDump", &pmt::xilinx::Xilinx::StartDump)
      .def("stopDump", &pmt::xilinx::Xilinx::StopDump);
#endif
}
