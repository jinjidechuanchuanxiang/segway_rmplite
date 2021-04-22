load("//bzl:module.bzl", "isaac_app", "isaac_cc_module", "isaac_subgraph")

isaac_subgraph(
    name = "segway_rmplite_subgraph",
    modules = [
        "//apps/segway_rmplite/segwayrmp:segway_rmplite",
    ],
    subgraph = "segway_rmplite.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_cc_module(
    name = "DriveSegway_component",
    srcs = [
        "//apps/segway_rmplite/drive_segway_sample:drive_segway_sample.cpp",
    ],
    hdrs = [
        "//apps/segway_rmplite/drive_segway_sample:drive_segway_sample.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//messages/state:differential_base",
        "//packages/engine_gems/state:io",
        # "//apps/segway_rmplite/drive_segway_sample:drive_segway_sample",
    ],
)

isaac_app(
    name = "drive_segway_sample",
    app_json_file = "drive_segway_sample.app.json",
    modules = [
        "//apps/segway_rmplite/drive_segway_sample:DriveSegway_component",
    ],
    data = [
        ":segway_rmplite_subgraph",
    ],
)

