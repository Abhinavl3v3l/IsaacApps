load("//bzl:module.bzl", "isaac_pkg")


py_binary(
    name = "xm430_tilt",
    srcs = [
        "__init__.py",
        "xm430_tilt.py",
    ],
    data = [
        "xm430_tilt.app.json",
        "//apps:py_init",
        "//packages/sight:libsight_module.so",
        "//packages/dynamixel:libdynamixel_module.so",
    ],
    deps = ["//packages/pyalice"],
)

isaac_pkg(
    name = "xm430_tilt-pkg",
    srcs = ["xm430_tilt"],
)