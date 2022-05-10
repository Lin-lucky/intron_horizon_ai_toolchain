/*
 * Copyright (c) 2021 Horizon Robotics
 * @brief     provides xproto version
 * @author    hangjun.yang
 * @email     hangjun.yang@horizon.ai
 * @date      2021.04.20
 */

#ifndef XPROTO_VERSION_H_
#define XPROTO_VERSION_H_

#define XPROTO_FRAMEWORK_VERSION "1.1.10"
#define XPROTO_FRAMEWORK_VERSION_TIME \
        XPROTO_FRAMEWORK_VERSION " " __TIMESTAMP__

#if defined(_MSC_VER)
#define XPROTO_EXPORT __declspec(dllexport)
#elif defined(__GNUC__)
#define XPROTO_EXPORT __attribute__((__visibility__("default")))
#else
#define XPROTO_EXPORT
#endif

#endif  // XPROTO_VERSION_H_
