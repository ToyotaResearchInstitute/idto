#include <pybind11/pybind11.h>

namespace py = pybind11;

int add(int x, int y) {
  return x + y;
}

int subtract(int x, int y) {
  return x - y;
}

struct Pet {
  explicit Pet(const std::string& name) : name(name) {}
  void setName(const std::string& name_) { name = name_; }
  const std::string& getName() const { return name; }

  std::string name;
};

PYBIND11_MODULE(example, m) {
  m.doc() = "pybind11 example plugin";

  m.def("add", &add, "A function which adds two numbers");
  m.def("subtract", &subtract, "A function which subtracts two numbers");

  py::class_<Pet>(m, "Pet")
      .def(py::init<const std::string&>())
      .def("setName", &Pet::setName)
      .def("getName", &Pet::getName);
}
