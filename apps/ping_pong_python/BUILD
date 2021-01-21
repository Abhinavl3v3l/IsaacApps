
# load("//bzl:py.bzl", "isaac_py_app")

# isaac_py_app(
#     name = "ping_pong_python",
#     srcs = [
#         "ping_pong_python.py",
#     ],
#     data = [
#         "ping_pong_python.app.json",
#         "//apps:py_init",
#         "//apps/tutorials:py_init",
#     ],
#     modules = [
#     ],
# )


load("//bzl:module.bzl", "isaac_pkg")


py_binary(
    name = "ping_pong_python",
    srcs = [
        "__init__.py",
        "ping_pong_python.py",
    ],
    data = [
        "ping_pong_python.app.json",
        "//apps:py_init",
        "//packages/sight:libsight_module.so",
    ],
    deps = ["//packages/pyalice"],
)

isaac_pkg(
    name = "ping_pong_python-pkg",
    srcs = ["ping_pong_python"],
)