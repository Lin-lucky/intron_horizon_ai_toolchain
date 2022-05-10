/*
 * Copyright (c) 2018 Horizon Robotics
 * @brief     provides xstream framework C interface
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @date      2018.12.14
 */

#ifndef XSTREAM_VERSION_H_
#define XSTREAM_VERSION_H_

#define XSTREAM_FRAMEWORK_VERSION "1.1.10"
#define XSTREAM_FRAMEWORK_VERSION_TIME \
        XSTREAM_FRAMEWORK_VERSION " " __TIMESTAMP__

#if defined(_MSC_VER)
#define XSTREAM_EXPORT __declspec(dllexport)
#elif defined(__GNUC__)
#define XSTREAM_EXPORT __attribute__((__visibility__("default")))
#else
#define XSTREAM_EXPORT
#endif

#endif  // XSTREAM_VERSION_H_
