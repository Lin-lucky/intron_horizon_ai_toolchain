/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     default method factory
 * @author    zhuoran.rong
 * @email     zhuoran.rong@horizon.ai
 * @version   0.0.0.1
 * @date      2020.04.15
 * @update    2021.01.27 by hangjun.yang
 */

#ifndef XSTREAM_USER_METHOD_FACTORY_H_
#define XSTREAM_USER_METHOD_FACTORY_H_
#include <string>

#include "xstream/version.h"
#include "xstream/method.h"

namespace xstream {

namespace method_factory {
/**
 * @brief The declaration of the default global Method Factory factory function.
 * This symbol is a weak symbol, that is to say: if this function is not
 * defined, the compilation and linking will still succeed, but an error will be
 * reported when used.The advantage of this is that if the user does not use the
 * Default Method Factory, there is no need to define an empty function to
 * prevent link errors and put it in the code. object is destroyed, including
 * the release of key variables, etc.
 * @param[in] method_name The name of method
 * @return[out] Return MethodPtr
 */
__attribute__((weak))  // NOLINT
XSTREAM_EXPORT MethodPtr
CreateMethod(const std::string &method_name);

}  // namespace method_factory
}  // namespace xstream
#endif  // XSTREAM_USER_METHOD_FACTORY_H_
