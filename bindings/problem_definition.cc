#include "optimizer/problem_definition.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(problem_definition, m) {
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
          })
      .def_property(
          "v_init",
          [](idto::optimizer::ProblemDefinition& self) {
            return self.v_init;
          },
          [](idto::optimizer::ProblemDefinition& self,
             const Eigen::VectorXd& v_init) {
            self.v_init = v_init;
          })
      .def_property(
          "Qq",
          [](idto::optimizer::ProblemDefinition& self) {
            return self.Qq;
          },
          [](idto::optimizer::ProblemDefinition& self,
             const Eigen::MatrixXd& Qq) {
            self.Qq = Qq;
          })
      .def_property(
          "Qv",
          [](idto::optimizer::ProblemDefinition& self) {
            return self.Qv;
          },
          [](idto::optimizer::ProblemDefinition& self,
             const Eigen::MatrixXd& Qv) {
            self.Qv = Qv;
          })
      .def_property(
          "Qf_q",
          [](idto::optimizer::ProblemDefinition& self) {
            return self.Qf_q;
          },
          [](idto::optimizer::ProblemDefinition& self,
             const Eigen::MatrixXd& Qf_q) {
            self.Qf_q = Qf_q;
          })
      .def_property(
          "Qf_v",
          [](idto::optimizer::ProblemDefinition& self) {
            return self.Qf_v;
          },
          [](idto::optimizer::ProblemDefinition& self,
             const Eigen::MatrixXd& Qf_v) {
            self.Qf_v = Qf_v;
          })
      .def_property(
          "R",
          [](idto::optimizer::ProblemDefinition& self) {
            return self.R;
          },
          [](idto::optimizer::ProblemDefinition& self,
             const Eigen::MatrixXd& R) {
            self.R = R;
          })
      .def_property(
          "q_nom",
          [](idto::optimizer::ProblemDefinition& self) {
            return self.q_nom;
          },
          [](idto::optimizer::ProblemDefinition& self,
             const std::vector<Eigen::VectorXd>& q_nom) {
            self.q_nom = q_nom;
          })
      .def_property(
          "v_nom",
          [](idto::optimizer::ProblemDefinition& self) {
            return self.v_nom;
          },
          [](idto::optimizer::ProblemDefinition& self,
             const std::vector<Eigen::VectorXd>& v_nom) {
            self.v_nom = v_nom;
          });
}
