#include "utils/find_resource.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;
using idto::FindIdtoResourceOrThrow;

PYBIND11_MODULE(find_idto_resource, m) {
  m.def("FindIdtoResourceOrThrow", &FindIdtoResourceOrThrow);
}
