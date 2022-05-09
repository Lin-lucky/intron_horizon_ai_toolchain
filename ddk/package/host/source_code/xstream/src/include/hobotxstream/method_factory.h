/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     method factory
 * @file      method_fatory.h
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 * @update    2021.01.27 by hangjun.yang
 */
#ifndef HOBOTXSTREAM_METHOD_FACTORY_H_
#define HOBOTXSTREAM_METHOD_FACTORY_H_

#include <string>
#include <memory>
#include "hobotlog/hobotlog.hpp"
#include "xstream/method.h"
#include "xstream/xstream_data.h"
#include "xstream/user_method_factory.h"

namespace xstream {

class MethodFactory {
 public:
  /// Method 工厂方法
  virtual MethodPtr CreateMethod(const std::string &method_name) = 0;
};

typedef std::shared_ptr<MethodFactory> MethodFactoryPtr;

// 默认MethodFactory
class DefaultMethodFactory final : public MethodFactory {
 public:
    DefaultMethodFactory() {
        // 构造默认Method Factory的时候,检查这个函数是否链接进来了.
        HOBOT_CHECK(method_factory::CreateMethod)
            << "Missing global default Method Factory";
    }
    ~DefaultMethodFactory() = default;

    // Method 工厂
    MethodPtr CreateMethod(const std::string &method_name) {
        return method_factory::CreateMethod(method_name);
    }
};

}  // namespace xstream

#endif  // HOBOTXSTREAM_METHOD_FACTORY_H_
