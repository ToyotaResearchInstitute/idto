#include "utils/find_resource.h"

#include <stdexcept>

#include <fmt/format.h>

namespace idto {
namespace utils {

std::string FindIdtoResource(const std::string& idto_resource_path) {
  if (idto_resource_path.substr(0, 5) != "idto/") {
    throw std::runtime_error(fmt::format(
        "Resource path '{}' must start with 'idto/' ", idto_resource_path));
  }
  // N.B. IDTO_BINARY_DIR set in top-level CMakeLists.txt
  return std::string(IDTO_BINARY_DIR) + "/" + idto_resource_path;
}

}  // namespace utils
}  // namespace idto