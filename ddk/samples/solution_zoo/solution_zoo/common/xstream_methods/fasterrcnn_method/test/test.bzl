#def fasterrcnnmethod_test(name, case):
#    native.cc_test(
#        name = name,
#        deps = [":fasterrcnnmethod", "@googletest//:gtest", "//xstream/vision_type:vision_type", "@opencv_aarch64//:opencv_aarch64"],
#        srcs = [":test_src"] + case,
#        includes = ["test", "external/vision_type/include"],
#)

def fasterrcnnmethod_test_defines():
    return select({
    "@hr_bazel_tools//rules_toolchain:x2j2-aarch64_linux": ["HR_POSIX", "HR_LINUX", "X2"],
    "@hr_bazel_tools//rules_toolchain:x86_32_linux": ["HR_POSIX","HR_LINUX"],
    "@hr_bazel_tools//rules_toolchain:x86_64_linux": ["HR_POSIX","HR_LINUX"],
    "//conditions:default": [],
    })