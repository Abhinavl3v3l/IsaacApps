load("@com_nvidia_isaac_sdk//bzl:module.bzl", "isaac_component")

isaac_component(
    name = "pololu_head_driver",
    visibility = ["//visibility:public"],
    deps = [
        "//genesis/pololu/gems:maestro",
        "@com_nvidia_isaac_sdk//packages/engine_gems/state:io",
        "@com_nvidia_isaac_engine//engine/core/tensor:universal_tensor",
        "@com_nvidia_isaac_engine//engine/gems/tensor",
        "//genesis/dynamixel/gems:dynamixel",
        "@com_nvidia_isaac_sdk//packages/composite/gems:parser",
        "@com_nvidia_isaac_sdk//packages/map:kinematic_tree",
        "@com_nvidia_isaac_sdk//packages/math/gems/kinematic_tree",
        "@com_nvidia_isaac_engine//engine/gems/serialization:json",
        "@com_nvidia_isaac_engine//third_party/nlohmann"
    ],
)