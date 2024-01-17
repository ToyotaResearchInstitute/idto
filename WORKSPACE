# SPDX-License-Identifier: MIT-0

workspace(name = "idto")

# Set the version of Drake that we'll use
DRAKE_COMMIT = "v1.25.0"
DRAKE_CHECKSUM = "ebebd812c4f3644cf2fefbbf72835060cbd26e8896a0959ad0fcd2f3428a0547"

# Or choose a specific revision of Drake to use.
# DRAKE_COMMIT = "251bd6982d48343cbd727d0a46d887461b2d13cc"
# DRAKE_CHECKSUM = "72d106c3a61383170f564916cfbae0c25285cc3cb946fca4785beea8c342b8f6"
#
# You can also use DRAKE_COMMIT to choose a Drake release; eg:
# DRAKE_COMMIT = "v0.15.0"
#
# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# DRAKE_CHECKSUM = "0" * 64

# Or to temporarily build against a local checkout of Drake, at the bash prompt
# set an environment variable before building:
#  export IDTO_LOCAL_DRAKE_PATH=/home/user/stuff/drake

# Load an environment variable.
load("//:environ.bzl", "environ_repository")
environ_repository(name = "environ", vars = ["IDTO_LOCAL_DRAKE_PATH"])
load("@environ//:environ.bzl", IDTO_LOCAL_DRAKE_PATH = "IDTO_LOCAL_DRAKE_PATH")

# This declares the `@drake` repository as an http_archive from github,
# iff IDTO_LOCAL_DRAKE_PATH is unset.  When it is set, this declares a
# `@drake_ignored` package which is never referenced, and thus is ignored.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
http_archive(
    name = "drake" if not IDTO_LOCAL_DRAKE_PATH else "drake_ignored",
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake-{}".format(DRAKE_COMMIT.lstrip("v")),
)

# This declares the `@drake` repository as a local directory,
# iff IDTO_LOCAL_DRAKE_PATH is set.  When it is unset, this declares a
# `@drake_ignored` package which is never referenced, and thus is ignored.
local_repository(
    name = "drake" if IDTO_LOCAL_DRAKE_PATH else "drake_ignored",
    path = IDTO_LOCAL_DRAKE_PATH,
)
print("Using IDTO_LOCAL_DRAKE_PATH={}".format(IDTO_LOCAL_DRAKE_PATH)) if IDTO_LOCAL_DRAKE_PATH else None  # noqa

# Reference external software libraries, tools, and toolchains per Drake's
# defaults.  Some software will come from the host system (Ubuntu or macOS);
# other software will be downloaded in source or binary form from GitHub or
# other sites.
load("@drake//tools/workspace:default.bzl", "add_default_workspace")
add_default_workspace()

# Load pybind11 for python bindings
http_archive(
  name = "pybind11_bazel",
  strip_prefix = "pybind11_bazel-2.11.1",
  urls = ["https://github.com/pybind/pybind11_bazel/archive/v2.11.1.zip"],
)
http_archive(
  name = "pybind11",
  build_file = "@pybind11_bazel//:pybind11.BUILD",
  strip_prefix = "pybind11-2.11.1",
  urls = ["https://github.com/pybind/pybind11/archive/v2.11.1.tar.gz"],
)
load("@pybind11_bazel//:python_configure.bzl", "python_configure")
python_configure(name = "local_config_python")
