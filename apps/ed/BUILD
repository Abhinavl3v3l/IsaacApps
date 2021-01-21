load("//bzl:module.bzl", "isaac_pkg")


py_binary(
    name = "ed",
    srcs = [
        "__init__.py",
        "ed.py",
    ],
    data = [
        "ed.app.json",
        "//apps:py_init",
        "//packages/sensors:libv4l2_camera_module.so",
        "//packages/sight:libsight_module.so",
        "//packages/viewers:libviewers_module.so"
    ],
    deps = ["//packages/pyalice"],
)

isaac_pkg(
    name = "ed-pkg",
    srcs = ["ed"],
)