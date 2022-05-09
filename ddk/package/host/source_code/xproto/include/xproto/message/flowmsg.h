/*!
 * -------------------------------------------
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     flowmsg.h
 * \Author   Yingmin Li
 * \Mail     yingmin.li@horizon.ai
 * \Version  1.0.0.0
 * \Date     2019-04-17
 * \Brief    implement of flowmsg.h
 * \DO NOT MODIFY THIS COMMENT, \
 * \WHICH IS AUTO GENERATED BY EDITOR
 * -------------------------------------------
 */

#ifndef XPROTO_INCLUDE_XPROTO_MESSAGE_FLOWMSG_H_
#define XPROTO_INCLUDE_XPROTO_MESSAGE_FLOWMSG_H_

#include <memory>
#include <string>

#include "xproto/version.h"

namespace xproto {
/**
 * @brief error code
 */
enum XPROTO_EXPORT XPluginErrorCode {
  ERROR_CODE_OK = 0,
  ERROR_CODE_HBIPC_UNINIT,
  ERROR_CODE_XIC_UNINIT
};

/**
 * @brief flow message: the base message. The message bus distribution mechanism
 * of the interaction between plugins requires a unified message interface to be
 * defined, and messages that realize the unified interface are distributed to
 * the corresponding plugin by the bus. Description: The new Message needs to
 * inherit XProtoMessage and define its own message class. XProtoMessage
 * describes message properties and functions, including: type_: message type
 * param_: message additional parameter information
 */
struct XPROTO_EXPORT
       XProtoMessage : public std::enable_shared_from_this<XProtoMessage> {
  XProtoMessage &operator=(const XProtoMessage &) = delete;
  virtual ~XProtoMessage() = default;

  std::string type_ = "NONE";

  std::string param_ = "";
  /**
   * @description: Get the type of message
   * @param {*}
   * @return {*}
   */
  std::string type() const { return type_; }

  /**
   * @description: New message types need to implement this interface to
   * implement serialization operations, such as serialization of protobuf
   * @param {*}
   * @return {*}
   */
  virtual std::string Serialize() = 0;

  /**
   * @description: New message types need to implement this interface to
   * implement deserialization operations, such as deserialization of protobuf
   * @param {std::string} data to DeSerialize
   * @return {*} 
   */
  virtual bool DeSerialize(const std::string &data) { return false; }
  /**
   * @description: The message is converted into a specified data structure,
   * which is often used in special needs.
   * @param {*}
   * @return {*}
   */
  virtual void *ConvertData() {
    // need call delete() to release the output
    return nullptr;
  }
};

using XProtoMessagePtr = std::shared_ptr<XProtoMessage>;
}  // namespace xproto

#endif  // XPROTO_INCLUDE_XPROTO_MESSAGE_FLOWMSG_H_
