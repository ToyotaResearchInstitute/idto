#pragma once

#include <string>

#include <drake/common/find_runfiles.h>

namespace idto {

/// Attempts to locate an IDTO resource named by the given `idto_resource_path`.
/// The path refers to the relative path within the source repository, e.g.,
/// `apps/home/foo.yaml`. If there is an Rlocation error, it is indicated in
/// the return value.
/// If there is a user error (i.e. empty string or absolute path), an exception
/// is thrown.
drake::RlocationOrError FindIDTOResource(
    const std::string& idto_resource_path);

/// Same as `FindIDTOResource`, but throws if there is an error, otherwise
/// returns a string to the resource's absolute path.
std::string FindIDTOResourceOrThrow(const std::string& idto_resource_path);

}  // namespace idto
