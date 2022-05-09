//
// Created by xudong.du@hobot.cc on 14/15/2021.
// Copyright (c) 2018 horizon robotics. All rights reserved.
//
#include "xproto/session/xsession_inner.h"

#include "hobotlog/hobotlog.hpp"
#include "xproto/msg_manager.h"
#include "xproto/msg_type/message_factory.hpp"
#include "xproto/msg_type/protobuf/x3.pb.h"
#include "xproto/session/zmq_manager.h"
#include "xproto/utils/time_helper.h"

namespace xproto {
using xproto::message::CreateXprotoMsg;
XSessionInner::XSessionInner() {
  stop_ = false;
  session_info_.id_ = 0;
  session_info_.type_ = TYPE_LOCAL;  //  default Local
  session_info_.state_ = STATE_OFFLINE;
  recv_handle_.CreatThread(1);
  send_handle_.CreatThread(1);
  socket_ = std::make_shared<ZmqManager>();
  if (nullptr == socket_) {
    LOGE << "Alloc socket failed.";
  }
}

int32_t XSessionInner::AsMaster(const std::string& host_ip,
                                uint16_t host_port) {
  if (is_inited_) {
    return -1;
  }
  int Log_level = GetLogLevel();
  std::cout << "log_level = " << Log_level << std::endl;
  if (Log_level == 2) {
    SetLogLevel(HOBOT_LOG_DEBUG);
  }
  LOGD << "AsMaster start.";
  if (nullptr == socket_) {
    LOGE << "socket is invalid.";
    return -1;
  }
  auto ret = socket_->Init(true, this);
  if (0 != ret) {
    LOGE << "socket::Init failed, ret = " << ret;
    return -1;
  }
  ret = socket_->Bind(host_ip, host_port);  // default localhost
  if (0 != ret) {
    LOGE << "socket::Bind failed, ret = " << ret << ", Ip = " << host_ip
         << ", host_port = " << host_port;
    socket_->Close();
    socket_->UnInit();
    socket_ = nullptr;
    return -1;
  }
  session_info_.type_ = TYPE_MASTER;
  session_info_.state_ = STATE_ONLINE;
  recv_handle_.PostTask(std::bind(&XSessionInner::RecvThread, this));
  is_inited_ = true;
  LOGD << "AsMaster suc.";
  return 0;
}

int32_t XSessionInner::ConnectTo(const std::string& ip, uint16_t host_port) {
  if (is_inited_) {
    return -1;
  }
  int Log_level = GetLogLevel();
  std::cout << "log_level = " << Log_level << std::endl;
  if (Log_level == 2) {
    SetLogLevel(HOBOT_LOG_DEBUG);
  }
  LOGD << "ConnectTo, IP = " << ip << ", port = " << host_port;
  if (nullptr == socket_) {
    LOGE << "socket is invalid.";
    return -1;
  }
  auto ret = socket_->Init(false, this);
  if (0 != ret) {
    LOGE << "socket::Init failed, ret = " << ret;
    return -1;
  }
  ret = socket_->ConnectTo(ip, host_port, session_version_);
  LOGD << "ConnectTo ret = " << ret;
  if (0 == ret) {
    session_info_.type_ = TYPE_SLAVE;
    session_info_.state_ = STATE_ONLINE;
    recv_handle_.PostTask(std::bind(&XSessionInner::RecvThread, this));
    is_inited_ = true;
  } else {
    socket_->Close();
    socket_->UnInit();
    socket_ = nullptr;
  }
  return ret;
}

std::vector<SessionInfo> XSessionInner::Info() {
  std::vector<SessionInfo> ret;
  return ret;
}

int XSessionInner::OnMessage(ActionType action_type, std::string& msg_type,
                             std::string& data) {
  LOGD << "Enter XSessionInner::OnMessage, action_type = " << action_type
       << ", msg_type = " << msg_type << ", version = " << data;
  if (action_type == ACTION_CONNECT_TO_SERVER) {
    if (data == session_version_) {
      return 1;
    }
  }
  return 0;
}

void XSessionInner::Reset() {
  Close();
  session_info_.type_ = TYPE_LOCAL;
  session_info_.state_ = STATE_OFFLINE;
}

void XSessionInner::Dispatch(XProtoMessagePtr msg) {
  // msg must not null
  if (stop_ || session_info_.state_ != STATE_ONLINE) {
    LOGD << "XSession is offline, msg_type = " << msg->type_;
    return;
  }
  if (session_info_.type_ == TYPE_MASTER) {
    std::lock_guard<std::mutex> l(sub_msg_types_mtx_);
    if (sub_msg_types_.find(msg->type_) == sub_msg_types_.end()) {
      LOGD << "Not found in sub_msg_types : " << msg->type_;
      return;
    }
  }
  send_handle_.PostTask(std::bind(&XSessionInner::SendMsgProc, this, msg));
}

void XSessionInner::SendMsgProc(XProtoMessagePtr msg) {
  // msg must not null
  std::string msg_str;
  msg_str = msg->Serialize();
  if (msg_str.length() == 0) {
    LOGD << "msg: " << msg->type_ << ", Serialize return len = 0!";
    return;
  }
  if (socket_) {
    socket_->ProcessOutputMsg(ACTION_SEND_MSG, msg->type_, msg_str, NULL);
  }
}

void XSessionInner::RecvThread() {
  while (!stop_) {
    if (socket_) {
      ActionType action_type = ACTION_NONE;
      std::string recv_data;
      std::string msg_type;
      auto ret = socket_->GetInputMsg(action_type, msg_type, recv_data);
      if (ret != 0) {
        continue;
      }

      if (action_type == ACTION_RECV_MSG) {
        /*
        1 如果是Master, 收到消息后需要再转发给sub
        2 如果是Slave, 收到消息后不需要转发, 只push给当前Session即可
        */
        if (session_info_.type_ == TYPE_MASTER) {
          /* MASTER收到的消息是SLAVE发来的中转消息,
             需要判断当前MASTER是否有消费者 这里不走XPROTO框架dispatch,
             节省序列化和反序列化的开销
          */
          {
            std::lock_guard<std::mutex> l(sub_msg_types_mtx_);
            if (sub_msg_types_.find(msg_type) == sub_msg_types_.end()) {
              LOGD << "Not found in sub_msg_types : " << msg_type;
              continue;
            }
          }
          // 在分发给其他Slave前创建msg, ProcessOutputMsg会把msg move掉
          auto msg = CreateXprotoMsg(msg_type, recv_data);
          if (msg) {
            XMsgQueue::Instance().TryPushMsg(msg, 1);
          }
          if (socket_) {
            LOGD << "transfer msg: " << msg_type;
            socket_->ProcessOutputMsg(ACTION_SEND_MSG, msg_type, recv_data,
                                      NULL);
          }
        } else if (session_info_.type_ == TYPE_SLAVE) {
          // TYPE_SLAVE
          /* SLAVE收到的消息一定是通过订阅收到的, 所以肯定有消费者 */
          auto msg = CreateXprotoMsg(msg_type, recv_data);
          if (msg) {
            XMsgQueue::Instance().TryPushMsg(msg, 1);
          }
        }
      } else if (action_type == ACTION_SUBSCRIBE_MSG) {
        /*
        1 如果是client接收到消息, 只更新消息表即可
        2 如果是Master节点接收到消息, 除了更新消息表, 还需要广播出去
        */
        LOGD << "Action Subscribe msg type = " << msg_type;
        if (session_info_.type_ == TYPE_SLAVE) {
          if (msg_type.size() > 0) {
            {
              std::lock_guard<std::mutex> l(sub_msg_types_mtx_);
              sub_msg_types_[msg_type] = Timer::current_time_stamp();
            }
          }
        } else if (session_info_.type_ == TYPE_MASTER) {
          RegisterMsg(msg_type);
        }
      } else if (action_type == ACTION_UNSUBSCRIBE_MSG) {
        LOGD << "Action UnSubscribe msg type = " << msg_type;
        if (session_info_.type_ == TYPE_SLAVE) {
          if (msg_type.size() > 0) {
            std::lock_guard<std::mutex> l(sub_msg_types_mtx_);
            if (sub_msg_types_.find(msg_type) != sub_msg_types_.end()) {
              sub_msg_types_.erase(msg_type);
            }
          }
        } else if (session_info_.type_ == TYPE_MASTER) {
          UnRegisterMsg(msg_type);
        }
      }
    } else {
      LOGE << "socket is null.";
    }
  }
}

/*
1 Master/Slave的xproto框架会调用此函数向网络中订阅消息
2 Master收到Slave订阅的消息,也会调用此函数向网络中其他Slave广播消息
3 Slave接收到消息, 只更新消息表
 */
void XSessionInner::RegisterMsg(const std::string& msg_type) {
  if (stop_ || session_info_.state_ != STATE_ONLINE) {
    LOGD << "XSession is offline, msg_type = " << msg_type;
    return;
  }
  if (msg_type.size() > 0) {
    {
      std::lock_guard<std::mutex> l(sub_msg_types_mtx_);
      sub_msg_types_[msg_type] = Timer::current_time_stamp();
    }
    if (socket_) {
      LOGD << "RegisterMsg to sub : msgtype = " << msg_type;
      socket_->ProcessOutputMsg(ACTION_SUBSCRIBE_MSG,
                                g_session_register_msg_subject, msg_type, NULL);
    }
  }
}

void XSessionInner::UnRegisterMsg(const std::string& msg_type) {
  if (stop_ || session_info_.state_ != STATE_ONLINE) {
    LOGD << "XSession is offline, msg_type = " << msg_type;
    return;
  }
  if (msg_type.size() > 0) {
    bool bFound = false;
    {
      std::lock_guard<std::mutex> l(sub_msg_types_mtx_);
      if (sub_msg_types_.find(msg_type) != sub_msg_types_.end()) {
        bFound = true;
        sub_msg_types_.erase(msg_type);
      }
    }
    if (bFound && socket_) {
      socket_->ProcessOutputMsg(ACTION_UNSUBSCRIBE_MSG,
                                g_session_unregister_msg_subject, msg_type,
                                NULL);
    }
  }
}

void XSessionInner::Close() {
  stop_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  recv_handle_.Stop();
  send_handle_.Stop();
  if (socket_) {
    socket_->Close();
    socket_->UnInit();
    socket_ = nullptr;
  }
  sub_msg_types_.clear();
}

}  // namespace xproto
