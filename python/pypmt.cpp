#include <pybind11/pybind11.h>

#include <pmt.h>

namespace py = pybind11;

PYBIND11_MODULE(pmt, m) {
  m.doc() = "libpmt python bindings";

  m.def("create", py::overload_cast<const std::string &, int>(&pmt::Create),
        py::arg("name"), py::arg("argument"));
  m.def("create",
        py::overload_cast<const std::string &, const char *>(&pmt::Create),
        py::arg("name"), py::arg("argument"));

  py::class_<pmt::PMT>(m, "PMT")
      .def("seconds", &pmt::PMT::seconds, "Get elapsed time")
      .def("joules", &pmt::PMT::joules, "Get energy consumption")
      .def("watts", &pmt::PMT::watts, "Get average power consumption")
      .def("read", &pmt::PMT::Read)
      .def("startDump", &pmt::PMT::StartDump)
      .def("stopDump", &pmt::PMT::StopDump);

  py::class_<pmt::State>(m, "State")
      .def("timestamp", &pmt::State::timestamp, "Get timestamp")
      .def("watts", &pmt::State::watts, py::arg("index"),
           "Get instantenous power consumption for the specified measurement")
      .def("name", &pmt::State::name, py::arg("index"),
           "Get name for the specified measurement")
      .def("nr_measurements", &pmt::State::NrMeasurements,
           "Get number of distinct measurements");
}
