#include "optimizer/problem_definition.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using idto::optimizer::ProblemDefinition;

PYBIND11_MODULE(problem_definition, m) {
  py::class_<ProblemDefinition>(m, "ProblemDefinition")
      .def(py::init<>())
      .def_readwrite("num_steps", &ProblemDefinition::num_steps)
      .def_property(
          "q_init",
          [](ProblemDefinition& self) {
            return self.q_init;
          },
          [](ProblemDefinition& self, const Eigen::VectorXd& q_init) {
            self.q_init = q_init;
          })
      .def_property(
          "v_init",
          [](ProblemDefinition& self) {
            return self.v_init;
          },
          [](ProblemDefinition& self, const Eigen::VectorXd& v_init) {
            self.v_init = v_init;
          })
      .def_property(
          "Qq",
          [](ProblemDefinition& self) {
            return self.Qq;
          },
          [](ProblemDefinition& self, const Eigen::MatrixXd& Qq) {
            self.Qq = Qq;
          })
      .def_property(
          "Qv",
          [](ProblemDefinition& self) {
            return self.Qv;
          },
          [](ProblemDefinition& self, const Eigen::MatrixXd& Qv) {
            self.Qv = Qv;
          })
      .def_property(
          "Qf_q",
          [](ProblemDefinition& self) {
            return self.Qf_q;
          },
          [](ProblemDefinition& self, const Eigen::MatrixXd& Qf_q) {
            self.Qf_q = Qf_q;
          })
      .def_property(
          "Qf_v",
          [](ProblemDefinition& self) {
            return self.Qf_v;
          },
          [](ProblemDefinition& self, const Eigen::MatrixXd& Qf_v) {
            self.Qf_v = Qf_v;
          })
      .def_property(
          "R",
          [](ProblemDefinition& self) {
            return self.R;
          },
          [](ProblemDefinition& self, const Eigen::MatrixXd& R) {
            self.R = R;
          })
      .def_property(
          "q_nom",
          [](ProblemDefinition& self) {
            return self.q_nom;
          },
          [](ProblemDefinition& self,
             const std::vector<Eigen::VectorXd>& q_nom) {
            self.q_nom = q_nom;
          })
      .def_property(
          "v_nom",
          [](ProblemDefinition& self) {
            return self.v_nom;
          },
          [](ProblemDefinition& self,
             const std::vector<Eigen::VectorXd>& v_nom) {
            self.v_nom = v_nom;
          });
}
