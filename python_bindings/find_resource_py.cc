#include "utils/find_resource.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;
using idto::utils::FindIdtoResource;

void bind_find_resource(py::module_& m) {
   m.def("FindIdtoResource", &FindIdtoResource);
}