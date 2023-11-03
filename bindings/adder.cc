#include <pybind11/pybind11.h>

namespace {

  int add(int x, int y) { return x + y; }

}

PYBIND11_MODULE(adder, m) {
  m.def("add", &add, "adds two numbers");
}

