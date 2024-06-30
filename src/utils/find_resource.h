#pragma once

#include <string>

namespace idto {
namespace utils {

/**
 * Find the absolute path of a resource (e.g. urdf file) given its name.
 *
 * Resources are copied to the idto/ folder in the binary directory.
 *
 * @param idto_resource_path The name of the file to find, must start with
 * "idto/".
 * @return std::string The absolute path to the resource.
 */
std::string FindIdtoResource(const std::string& idto_resource_path);

}  // namespace utils
}  // namespace idto
