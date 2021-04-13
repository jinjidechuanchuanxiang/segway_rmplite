load("//bzl:module.bzl",  "isaac_cc_module", "isaac_app")

isaac_cc_module(
    name = "segway_rmplite",
    srcs = [
        "segwayRmp.cpp",
    ],
    hdrs = [
        "segwayRmp.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//apps/segway_rmplite/segwayrmp/lib:segway_chassis",
        "//messages/state:differential_base",
        "//messages/state:common",
        "//packages/engine_gems/state:io",
        "@com_nvidia_isaac_engine//engine/core/math:math",
    ],
)


