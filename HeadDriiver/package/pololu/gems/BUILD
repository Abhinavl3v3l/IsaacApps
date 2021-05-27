
load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "maestro",
    srcs = [
        "maestro.cpp",
    ],
    hdrs = [
        "maestro.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":config",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "config",
    srcs = [
    ],
    hdrs = [
        "maestroconfig.hpp",
    ],
    tags = ["nolint"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)
