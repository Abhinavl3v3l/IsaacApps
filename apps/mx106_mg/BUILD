# load("//bzl:module.bzl", "isaac_app")

# isaac_app(
#     name = "mx106_mg",
#     app_json_file = "mx106_mg.app.json",
#     modules = [
#         "dynamixel",
#         "sight"
#     ],
# )

load("//bzl:module.bzl", "isaac_pkg")


py_binary(
    name = "mx106_mg",
    srcs = [
        "__init__.py",
        "mx106_mg.py",
    ],
    data = [
        "mx106_mg.app.json",
        "//apps:py_init",
        "//packages/sight:libsight_module.so",
        "//packages/dynamixel:libdynamixel_module.so"
    ],
    deps = ["//packages/pyalice"],
)

isaac_pkg(
    name = "mx106_mg-pkg",
    srcs = ["mx106_mg"],
)