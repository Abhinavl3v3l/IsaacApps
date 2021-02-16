# load("//bzl:module.bzl", "isaac_app")

# isaac_app(
#     name = "xm430",
#     app_json_file = "xm430.app.json",
#     modules = [
#         "dynamixel",
#         "sight"
#     ],
# )

load("//bzl:module.bzl", "isaac_pkg")


py_binary(
    name = "xm430_mg",
    srcs = [
        "__init__.py",
        "xm430_mg.py",
    ],
    data = [
        "xm430_mg.app.json",
        "//apps:py_init",
        "//packages/sight:libsight_module.so",
        "//packages/dynamixel:libdynamixel_module.so"
    ],
    deps = ["//packages/pyalice"],
)

isaac_pkg(
    name = "xm430_mg-pkg",
    srcs = ["xm430_mg"],
)