# load("//bzl:module.bzl", "isaac_app")

# isaac_app(
#     name = "mx64_mg",
#     app_json_file = "mx64_mg.app.json",
#     modules = [
#         "dynamixel",
#         "sight"
#     ],
# )

load("//bzl:module.bzl", "isaac_pkg")


py_binary(
    name = "mx64_mg",
    srcs = [
        "__init__.py",
        "mx64_mg.py",
    ],
    data = [
        "mx64_mg.app.json",
        "//apps:py_init",
        "//packages/sight:libsight_module.so",
        "//packages/dynamixel:libdynamixel_module.so"
    ],
    deps = ["//packages/pyalice"],
)

isaac_pkg(
    name = "mx64_mg-pkg",
    srcs = ["mx64_mg"],
)