# load("//bzl:module.bzl", "isaac_app","isaac_pkg")

# isaac_app(
#     name = "edge_detector",
#     modules = [
#         "sensors:v4l2_camera",
#         "sight",
#         "viewers",
#     ],
# )

# py_binary(
#     name = "edge_detector",
#     srcs = [
#         "__init__.py",
#         "edge_detector.py",
#     ],
#     data = [
#         "edge_detector.app.json",
#         "//apps:py_init",
#         "//apps/samples:py_init",
#         "//messages:image_proto",
#         "//packages/sensors:libv4l2_camera_module.so",
#         "//packages/sight:libsight_module.so",
#         "//packages/viewers:libviewers_module.so"
#     ],
#     deps = ["//packages/pyalice"],
# )


load("//bzl:module.bzl", "isaac_pkg")


py_binary(
name = "edge_detector",
srcs = [
"__init__.py",
"edge_detector.py",
],
data = [
"edge_detector.app.json",
"//apps:py_init",
"//packages/sensors:libv4l2_camera_module.so",
"//packages/sight:libsight_module.so",
"//packages/viewers:libviewers_module.so"
],
deps = ["//packages/pyalice"],
)

isaac_pkg(
name = "edge_detector-pkg",
srcs = ["edge_detector"],
)
