def cc_googletest(name, deps = [], **kwargs):
  native.cc_test(name=name, deps=deps + ["//utils:googletest_main"], **kwargs)
