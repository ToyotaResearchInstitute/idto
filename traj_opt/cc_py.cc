#include <string>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include "traj_opt/convergence_criteria_tolerances.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace py = pybind11;

namespace idto {
namespace traj_opt {
namespace internal {

void DefineTrajOptConvergenceCriteriaTolerances(py::module m) {
  // TODO(anrp): This used to use the generated pybind documentation header
  // machinery in drake. Would be good to still have that...
  using Class = ConvergenceCriteriaTolerances;
  py::class_<Class> cls(m, "ConvergenceCriteriaTolerances");
  cls.def(drake::pydrake::ParamInit<Class>());
  drake::pydrake::DefAttributesUsingSerialize(&cls);
  drake::pydrake::DefReprUsingSerialize(&cls);
  drake::pydrake::DefCopyAndDeepCopy(&cls);
}

}  // namespace internal

PYBIND11_MODULE(cc, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = "A collection of trajectory optimization methods.";

  internal::DefineTrajOptConvergenceCriteriaTolerances(m);
}

}  // namespace traj_opt
}  // namespace idto
