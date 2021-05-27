load("//bzl:py.bzl", "genesis_py_app")
load("@com_nvidia_isaac_sdk//bzl:module.bzl", "isaac_subgraph")


filegroup(
    name = "codelets",
    srcs = glob(["codelets/**"]),
)

filegroup(
    name = "graphs",
    srcs = glob(["graphs/*"]),
)

filegroup(
    name = "data",
    srcs = glob(["data/*"]),
)

genesis_py_app(
    name = "hdriver",
    srcs = [
        ":codelets",
    ],
    data = [
        ":graphs",
        ":data",
    ],
    main = "codelets/mpololu.py",
    modules = [
        "//genesis/genesis_sight",
        "//genesis/pololu"
    ],
)



