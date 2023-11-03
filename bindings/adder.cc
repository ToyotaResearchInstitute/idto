#include <pybind11/pybind11.h>

int add(int x, int y) {
  return x + y;
}

int subtract(int x, int y) {
  return x - y;
}

PYBIND11_MODULE(adder, m) {
  m.def("add", &add, "adds two numbers");
  m.def("subtract", &subtract, "subtracts two numbers");
}
