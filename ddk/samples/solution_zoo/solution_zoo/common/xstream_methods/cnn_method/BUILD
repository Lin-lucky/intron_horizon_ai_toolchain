load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("cnnmethod.bzl", "cnnmethod_defines")
load("@hr_bazel_tools//rules_cc:deps.bzl", "cc_library_pkg")
load(":test/test.bzl", "cnnmethod_test")
cc_library_pkg(
    name = "cnnmethod",
    srcs = glob(["src/**/*.cpp", "include/**/*.h"]),
    deps = ["@jsoncpp//:jsoncpp", "@hobotlog//:hobotlog",
            "//xstream/imagetools:xstream-imagetools",
            "@bpu_predict//:bpu_predict",
            "@x2_prebuilt_aarch64//:x2_prebuilt_aarch64",
            "//xstream/framework:xstream",
            "//xstream/methods/fasterrcnnmethod:fasterrcnnmethod"],
    linkopts = [],
    copts = [],
    defines = cnnmethod_defines(),
    hdrs = glob(["include/CNNMethod/*.h"]),
    includes = ["include", "include/PostPredictor",
                 "include/Predictor", "include/util",
                 "external/vision_type/include",
                 "external/bpu_predict/include/bpu_predict"],
    visibility = ["//visibility:public"],
)

filegroup(
     name = "test_src",
     srcs = glob([
            "test/gtest_main.cc"]),
)

cc_binary(
    name = "cnnmethod_unit_test",
    deps = ["@jsoncpp//:jsoncpp", "@hobotlog//:hobotlog",
             "@googletest//:gtest",
             "@opencv_aarch64//:opencv_aarch64",
             "@libjpeg-turbo_aarch64//:libjpeg-turbo_aarch64",
             "//xstream/vision_type:vision_type",
             "@x2_prebuilt_aarch64//:x2_prebuilt_aarch64",
             "//xstream/imagetools:xstream-imagetools",
             "@bpu_predict//:bpu_predict",
             "//xstream/framework:xstream",
             "//xstream/methods/fasterrcnnmethod:fasterrcnnmethod",
             "//xstream/methods/cnnmethod:cnnmethod"
             ],
    linkopts = ["-Lexternal/x2_prebuilt_aarch64/x2_prebuilt/lib",
                "-lhbrt_bernoulli_aarch64",
                "-lcam", "-lcnn_intf", "-lvio", "-lfb", "-ldl",
                "-Lexternal/opencv_aarch64/opencv_ext/lib", "-lopencv_world"],
    copts = [],
    defines = [],
    srcs = ["test/gtest_main.cc", "test/cnn_test.cpp"],
    includes = ["include", "include/util", "include/Predictor", "include/PostPredictor",
                "example/include",
                "external/vision_type/include",
                "external/vision_type/include/horizon/visibility",
                "external/bpu_predict/include/",
                "external/bpu_predict/include/bpu_predict", "example/",
                "external/x2_prebuilt_aarch64/x2_prebuilt/include"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "cnnmethod_example",
    deps = ["@jsoncpp//:jsoncpp", "@hobotlog//:hobotlog",
             "@googletest//:gtest",
             "@opencv_aarch64//:opencv_aarch64",
             "@libjpeg-turbo_aarch64//:libjpeg-turbo_aarch64",
             "//xstream/vision_type:vision_type",
             "@x2_prebuilt_aarch64//:x2_prebuilt_aarch64",
             "//xstream/imagetools:xstream-imagetools",
             "@bpu_predict//:bpu_predict",
             "//xstream/framework:xstream",
              "//xstream/methods/fasterrcnnmethod:fasterrcnnmethod",
              "//xstream/methods/cnnmethod:cnnmethod"
             ],
    linkopts = ["-Lexternal/x2_prebuilt_aarch64/x2_prebuilt/lib",
                 "-lhbrt_bernoulli_aarch64", "-lcam", "-lcnn_intf", "-lvio", "-lfb", "-ldl", 
                 "-Lexternal/opencv_aarch64/opencv_ext/lib", "-lopencv_world",
                 "-Lexternal/libjpeg-turbo_aarch64/libjpeg-turbo/lib", "-lturbojpeg", "-ljpeg"
                 ],
    copts = [],
    defines = [],
    srcs = glob(["example/*.cpp", "example/conformance/*.cpp"]),
    includes = ["include", "include/util", "include/Predictor", "include/PostPredictor",
                "example/include",
                "external/vision_type/include",
                "external/vision_type/include/horizon/visibility",
                "external/bpu_predict/include/",
                "external/bpu_predict/include/bpu_predict", "example/",
                "external/x2_prebuilt_aarch64/x2_prebuilt/include"],
    visibility = ["//visibility:public"],
)