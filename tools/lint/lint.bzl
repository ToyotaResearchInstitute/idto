load("//tools/lint:cpplint.bzl", "cpplint")

# This is a pared-down version of Drake's linter, see
# https://github.com/RobotLocomotion/drake/blob/master/tools/lint/

def add_lint_tests(
        cpplint_data = None,
        cpplint_extra_srcs = None,
        enable_clang_format_lint = False):
    """For every rule in the BUILD file so far, and for all Bazel files in this
    directory, adds test rules that run Drake's standard lint suite over the
    sources.  Thus, BUILD file authors should call this function at the *end*
    of every BUILD file.

    Refer to the specific linters for their semantics and argument details:
    - bazel_lint.bzl
    - cpplint.bzl
    - python_lint.bzl

    """
    existing_rules = native.existing_rules().values()
    cpplint(
        existing_rules = existing_rules,
        data = cpplint_data,
        extra_srcs = cpplint_extra_srcs,
        enable_clang_format_lint = enable_clang_format_lint,
    )
