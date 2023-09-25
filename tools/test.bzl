def cc_googletest(name, deps = [], **kwargs):
  native.cc_test(name=name, deps=deps + ["//idto/common:googletest_main"], **kwargs)
