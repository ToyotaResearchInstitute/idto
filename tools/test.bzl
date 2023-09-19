def cc_googletest(name, deps = [], **kwargs):
  native.cc_test(name=name, deps=deps + ["//common:googletest_main"], **kwargs)
