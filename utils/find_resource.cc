#include "utils/find_resource.h"

#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <stdexcept>

#include <fmt/format.h>

#include <drake/common/drake_assert.h>

using drake::FindRunfile;

namespace idto {

drake::RlocationOrError
FindIdtoResource(const std::string& idto_resource_path) {
  // Check the user input.
  if (idto_resource_path.empty()) {
    throw std::runtime_error("Resource path must not be empty");
  }
  if (idto_resource_path.substr(0, 5) != "idto/") {
    throw std::runtime_error(fmt::format(
        "Resource path '{}' must start with 'idto/' ", idto_resource_path));
  }
  // Workspace name, and subdir name.
  const std::string resource_path = idto_resource_path;

  // Use Drake's Rlocation helper.
  return FindRunfile(resource_path);
}

std::string FindIdtoResourceOrThrow(const std::string& idto_resource_path) {
  auto rlocation_or_error = FindIdtoResource(idto_resource_path);
  if (!rlocation_or_error.error.empty()) {
    throw std::runtime_error(rlocation_or_error.error);
  }
  DRAKE_DEMAND(!rlocation_or_error.abspath.empty());
  return rlocation_or_error.abspath;
}

}  // namespace idto
