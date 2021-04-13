"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl",  "isaac_cc_library")

isaac_cc_library(
    name = "segway_chassis",
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["libctrl_x86_64.so"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["libctrl_arm64-v8a.so"],
        }),
    hdrs = [
        ":comm_ctrl_navigation.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        
    ],
)
