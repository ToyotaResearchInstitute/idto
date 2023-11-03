#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "optimizer/problem_definition.h"

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

class MyEigenClass {
 public:
  Eigen::MatrixXd& getMatrix() { return big_mat_; }
  const Eigen::MatrixXd& viewMatrix() { return big_mat_; }

 private:
  Eigen::MatrixXd big_mat_ = Eigen::MatrixXd::Zero(10000, 10000);
};

PYBIND11_MODULE(example, m) {
  m.doc() = "pybind11 example plugin";

  m.def("add", &add, "A function which adds two numbers");
  m.def("subtract", &subtract, "A function which subtracts two numbers");

  py::class_<Pet>(m, "Pet")
      .def(py::init<const std::string&>())
      .def("setName", &Pet::setName)
      .def("getName", &Pet::getName);

  py::class_<MyEigenClass>(m, "MyEigenClass")
      .def(py::init<>())
      .def("getMatrix", &MyEigenClass::getMatrix, py::return_value_policy::reference_internal)
      .def("viewMatrix", &MyEigenClass::viewMatrix, py::return_value_policy::reference_internal);

  py::class_<idto::optimizer::ProblemDefinition>(m, "ProblemDefinition")
      .def(py::init<>())
      .def_readwrite("num_steps",
                     &idto::optimizer::ProblemDefinition::num_steps)
      .def_property(
          "q_init",
          [](idto::optimizer::ProblemDefinition& self) {
            return self.q_init;
          },
          [](idto::optimizer::ProblemDefinition& self,
             const Eigen::VectorXd& q_init) {
            self.q_init = q_init;
          });
}
