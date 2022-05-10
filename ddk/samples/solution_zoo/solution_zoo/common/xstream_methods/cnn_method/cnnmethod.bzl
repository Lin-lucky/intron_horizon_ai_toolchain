def cnnmethod_defines():
    return select({
    "@hr_bazel_tools//rules_toolchain:x2j2-aarch64_linux": ["HR_POSIX","HR_LINUX"],
    "//conditions:default": [],
    })

