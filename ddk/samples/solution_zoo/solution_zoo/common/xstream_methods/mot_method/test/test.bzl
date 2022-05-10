def motmethod_test(name, case):
    native.cc_test(
        name = name,
        deps = [":motmethod", "@googletest//:gtest", "//xstream/vision_type:vision_type", "@opencv_aarch64//:opencv_aarch64"],
        srcs = [":test_src"] + case,
        includes = ["test"],
)