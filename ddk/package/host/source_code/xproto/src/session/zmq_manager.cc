//
// Created by xudong.du@hobot.cc on 14/15/2021.
// Copyright (c) 2018 horizon robotics. All rights reserved.
//
#include "xproto/session/zmq_manager.h"

#include <memory>
#include <sstream>
#include <thread>

#include "hobotlog/hobotlog.hpp"

namespace xproto {

ZmqManager::ZmqManager() {
  stop_ = false;
  is_online_ = false;
  is_server_ = false;
  zmq_context_ = std::make_shared<zmq::context_t>(1);
  server_pub_handle_.CreatThread(1);
  server_router_handle_.CreatThread(1);
  client_sub_handle_.CreatThread(1);
  client_dealer_handle_.CreatThread(1);
  command_pub_handle_.CreatThread(1);
  wait_send_datas_.Init(ring_queue_max_, nullptr);
  recv_datas_.Init(ring_queue_max_, nullptr);
}

ZmqManager::~ZmqManager() { Close(); }

int ZmqManager::Init(bool is_server, MessageCallBack* call_back) {
  if (is_inited_) {
    return 0;
  }
  call_back_ = call_back;
  is_server_ = is_server;
  is_inited_ = true;
  return 0;
}

int ZmqManager::UnInit() {
  call_back_ = nullptr;
  is_inited_ = false;
  is_server_ = false;
  return 0;
}

int ZmqManager::Bind(const std::string& host_ip, uint16_t host_port) {
  if (!is_inited_ || !is_server_) {
    LOGD << "Not Master!";
    return -1;
  }
  if (is_online_) {
    LOGD << "ZmqManager already bind.";
    return -1;
  }
  ip_ = host_ip;
  port_ = host_port;

  std::ostringstream buffer;
  buffer << "tcp://" << host_ip << ":" << host_port;
  std::string tcpaddr = buffer.str();

  server_pub_handle_.PostTask(
      std::bind(&ZmqManager::ServerPubProc, this, tcpaddr));

  buffer.str("");
  buffer << "tcp://" << host_ip << ":" << (host_port + 1);
  tcpaddr = buffer.str();
  server_router_handle_.PostTask(
      std::bind(&ZmqManager::ServerRouterProc, this, tcpaddr));

  is_online_ = true;
  return 0;
}

void ZmqManager::ServerPubProc(const std::string& tcpaddr) {
  // start zmq server
  auto server_pub = std::make_shared<zmq::socket_t>(*zmq_context_, ZMQ_PUB);
  try {
    server_pub->bind(tcpaddr);
    server_pub->setsockopt(ZMQ_LINGER, &zero_, sizeof(zero_));
  } catch (std::exception& e) {
    LOGE << "ZmqManager bind: " << tcpaddr << " failed, " << e.what();
    return;
  }
  auto command_pub = std::make_shared<zmq::socket_t>(*zmq_context_, ZMQ_PUB);
  try {
    command_pub->bind(command_address_);
    command_pub->setsockopt(ZMQ_LINGER, &zero_, sizeof(zero_));
  } catch (std::exception& e) {
    LOGE << "ZmqManager bind: " << tcpaddr << " failed, " << e.what();
    return;
  }

  while (!stop_) {
    MsgData send_data;
    auto ret = wait_send_datas_.Pop(send_data);
    if (!ret) {
      break;
    }
    zmq::message_t address(send_data.msg_type_.data(),
                           send_data.msg_type_.size());
    server_pub->send(address, zmq::send_flags::sndmore);
    zmq::const_buffer buffer(send_data.msg_data_.data(),
                             send_data.msg_data_.size());
    server_pub->send(buffer, zmq::send_flags::dontwait);
  }
  // 退出时向其他线程发送退出消息
  LOGD << "start to send kill subject" << std::endl;
  zmq::message_t kill_subject(g_killself_subject, strlen(g_killself_subject));
  command_pub->send(kill_subject, zmq::send_flags::sndmore);
  zmq::message_t kill_msg("", 0);  // 随意发送
  command_pub->send(kill_msg, zmq::send_flags::none);

  server_pub->close();
  command_pub->close();
  server_pub = nullptr;
  command_pub = nullptr;
  LOGD << "ServerPubProc end" << std::endl;
}

void ZmqManager::ServerRouterProc(const std::string& tcpaddr) {
  // start zmq server
  auto server_router =
      std::make_shared<zmq::socket_t>(*zmq_context_, ZMQ_ROUTER);
  try {
    server_router->bind(tcpaddr);
    server_router->setsockopt(ZMQ_LINGER, &zero_, sizeof(zero_));
  } catch (std::exception& e) {
    LOGE << "ZmqManager bind : " << tcpaddr << " failed, " << e.what();
    server_router->close();
    server_router = nullptr;
    return;
  }
  auto command_sub = std::make_shared<zmq::socket_t>(*zmq_context_, ZMQ_SUB);
  command_sub->connect(command_address_);
  command_sub->setsockopt(ZMQ_SUBSCRIBE, g_killself_subject,
                          strlen(g_killself_subject));
  zmq::pollitem_t items[] = {
      {server_router->handle(), 0, ZMQ_POLLIN, 0},
      {command_sub->handle(), 0, ZMQ_POLLIN, 0},
  };
  while (!stop_) {
    zmq::message_t address;
    zmq::message_t msg_type;
    zmq::message_t msg_request;

    zmq::poll(items, 2, -1);
    if (items[0].revents & ZMQ_POLLIN) {
      server_router->recv(address);
      server_router->recv(msg_type);
      server_router->recv(msg_request);
      LOGD << "msg_type = " << msg_type.to_string();
      if (msg_type.to_string() == g_first_request_subject) {
        if (call_back_) {
          int ret = 0;
          auto type = msg_type.to_string();
          auto data = std::move(msg_request.to_string());
          ret = call_back_->OnMessage(ACTION_CONNECT_TO_SERVER, type, data);
          if (ret != 0) {
            server_router->send(address, zmq::send_flags::sndmore);
            server_router->send(msg_type, zmq::send_flags::sndmore);
            zmq::message_t return_msg;
            if (1 == ret) {
              return_msg.rebuild(2);
              memcpy(return_msg.data(), "ok", 2);
            } else {
              return_msg.rebuild(5);
              memcpy(return_msg.data(), "error", 5);
            }
            server_router->send(return_msg, zmq::send_flags::none);
          }
        }
      } else {
        MsgData msg_data;
        LOGD << "router recv data msgtype : " << msg_type.to_string();
        if (msg_type.to_string() == g_heart_update_subject) {
          msg_data.action_type_ = ACTION_HEART_UPDATE;
          msg_data.msg_type_ = std::move(msg_request.to_string());
          msg_data.msg_data_ = "";
          LOGD << "router recv data request : " << msg_data.msg_type_;
        } else if (msg_type.to_string() == g_session_register_msg_subject) {
          msg_data.action_type_ = ACTION_SUBSCRIBE_MSG;
          msg_data.msg_type_ = std::move(msg_request.to_string());
          msg_data.msg_data_ = "";
          LOGD << "router recv data request : " << msg_data.msg_type_;
        } else if (msg_type.to_string() == g_session_register_msg_subject) {
          msg_data.action_type_ = ACTION_UNSUBSCRIBE_MSG;
          msg_data.msg_type_ = std::move(msg_request.to_string());
          msg_data.msg_data_ = "";
          LOGD << "router recv data request : " << msg_data.msg_type_;
        } else {
          LOGD << "recv msg";
          // 这里肯定是Master, 在这里转发效率更高
          msg_data.action_type_ = ACTION_RECV_MSG;
          msg_data.msg_type_ = std::move(msg_type.to_string());
          msg_data.msg_data_ = std::move(msg_request.to_string());
        }
        recv_datas_.PushMove(msg_data);
      }
    }
    if (items[1].revents & ZMQ_POLLIN) {
      command_sub->recv(address);
      command_sub->recv(msg_type);
      break;
    }
  }
  server_router->close();
  command_sub->close();
  server_router = nullptr;
  command_sub = nullptr;
  LOGD << "ServerRouterProc end.";
}

void ZmqManager::ClientSubProc(const std::string& tcpaddr) {
  auto client_subscribe =
      std::make_shared<zmq::socket_t>(*zmq_context_, ZMQ_SUB);
  {
    client_subscribe->connect(tcpaddr);
    client_subscribe->setsockopt(ZMQ_LINGER, &zero_, sizeof(zero_));
    /* client_subscribe接收到的是Master广播的订阅消息,
     * 不需要更新client_subscribe订阅subject,
     * 但是需要更新Session的消息表sub_msg_types_ */
    client_subscribe->setsockopt(ZMQ_SUBSCRIBE,
                                 g_session_register_msg_subject,
                                 strlen(g_session_register_msg_subject));
    client_subscribe->setsockopt(ZMQ_SUBSCRIBE,
                                 g_session_unregister_msg_subject,
                                 strlen(g_session_unregister_msg_subject));
  }
  auto command_sub = std::make_shared<zmq::socket_t>(*zmq_context_, ZMQ_SUB);
  {
    command_sub->connect(command_address_);
    command_sub->setsockopt(ZMQ_SUBSCRIBE, g_killself_subject,
                            strlen(g_killself_subject));
    /* command_sub接收到的是当前Client订阅的消息,
     * 当前节点只会真正订阅当前client节点需要订阅的消息
     * 需要更新client_subscribe订阅subject*/
    command_sub->setsockopt(ZMQ_SUBSCRIBE,
                            g_session_register_msg_subject,
                            strlen(g_session_register_msg_subject));
    command_sub->setsockopt(ZMQ_SUBSCRIBE,
                            g_session_unregister_msg_subject,
                            strlen(g_session_unregister_msg_subject));
  }
  zmq::pollitem_t items[] = {
      {client_subscribe->handle(), 0, ZMQ_POLLIN, 0},
      {command_sub->handle(), 0, ZMQ_POLLIN, 0},
  };

  while (!stop_) {
    zmq::message_t address;
    zmq::message_t request;
    MsgData msg_data;
    zmq::poll(items, 2, -1);
    /* sub msg */
    if (items[0].revents & ZMQ_POLLIN) {
      auto ret = client_subscribe->recv(address);
      ret = client_subscribe->recv(request);
      if (address.to_string() == g_session_register_msg_subject) {
        msg_data.action_type_ = ACTION_SUBSCRIBE_MSG;
        msg_data.msg_type_ = std::move(request.to_string());
        msg_data.msg_data_ = "";
      } else if (address.to_string() == g_session_unregister_msg_subject) {
        msg_data.action_type_ = ACTION_UNSUBSCRIBE_MSG;
        msg_data.msg_type_ = std::move(request.to_string());
        msg_data.msg_data_ = "";
      } else {
        msg_data.action_type_ = ACTION_RECV_MSG;
        msg_data.msg_type_ = std::move(address.to_string());
        msg_data.msg_data_ = std::move(request.to_string());
      }
      recv_datas_.PushMove(msg_data);
      LOGD << "recv data: "
           << std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();
    }
    /* sub command msg */
    if (items[1].revents & ZMQ_POLLIN) {
      command_sub->recv(address);
      command_sub->recv(request);
      if (address.to_string() == g_session_register_msg_subject) {
        client_subscribe->setsockopt(ZMQ_SUBSCRIBE, request.to_string().data(),
                                     request.size());
        LOGD << "sub client set subject: " << request.to_string();
      } else if (address.to_string() == g_session_unregister_msg_subject) {
        client_subscribe->setsockopt(
            ZMQ_UNSUBSCRIBE, request.to_string().data(), request.size());
        LOGD << "sub client unsubscribe subject: " << request.to_string();
      } else if (address.to_string() == g_killself_subject) {
        LOGD << "sub client recv kill. " << request.to_string();
        break;
      }
    }
  }
  client_subscribe->close();
  command_sub->close();
  client_subscribe = nullptr;
  command_sub = nullptr;
  LOGD << "ClientSubProc end.";
}

void ZmqManager::ClientDealerProc(const std::string& tcpaddr,
                                  const std::string& version_data) {
  auto client_dealer =
      std::make_shared<zmq::socket_t>(*zmq_context_, ZMQ_DEALER);
  client_dealer->setsockopt(ZMQ_LINGER, &zero_, sizeof(zero_));
  try {
    client_dealer->connect(tcpaddr);
    client_dealer->setsockopt(ZMQ_LINGER, &zero_, sizeof(zero_));
  } catch (std::exception& e) {
    LOGE << "ZmqManager connect to : " << tcpaddr << " failed, " << e.what();
    client_dealer->close();
    return;
  }
  auto command_sub = std::make_shared<zmq::socket_t>(*zmq_context_, ZMQ_SUB);
  command_sub->connect(command_address_);
  command_sub->setsockopt(ZMQ_SUBSCRIBE, "", 0);
  zmq::pollitem_t items[] = {
      {client_dealer->handle(), 0, ZMQ_POLLIN, 0},
      {command_sub->handle(), 0, ZMQ_POLLIN, 0},
  };
  {
    /* send first request */
    zmq::message_t subject(g_first_request_subject,
                           strlen(g_first_request_subject));
    client_dealer->send(subject, zmq::send_flags::sndmore);
    zmq::message_t request(version_data.data(), version_data.size());
    client_dealer->send(request, zmq::send_flags::none);
  }
  while (!stop_) {
    zmq::message_t address;
    zmq::message_t request;
    zmq::poll(items, 2, -1);
    if (items[0].revents & ZMQ_POLLIN) {
      LOGD << "ClientDealerProc start poll 0.";
      client_dealer->recv(address);
      client_dealer->recv(request);
      if (address.to_string() == g_first_request_subject &&
          request.to_string() == "ok") {
        std::unique_lock<std::mutex> lock(first_request_mtx_);
        firtst_reply_ = true;
        first_request_cv_.notify_all();
      }
    }
    if (items[1].revents & ZMQ_POLLIN) {
      command_sub->recv(address);
      command_sub->recv(request);
      if (address.to_string() == g_killself_subject) {
        LOGD << "ClientDealerProc recv kill command." << std::endl;
        break;
      } else if (address.to_string() == g_session_register_msg_subject ||
                 address.to_string() == g_session_unregister_msg_subject) {
        /* 收到client订阅的信息，需要广播出去 */
        LOGD << "ClientDealerProc recv:  " << address.to_string() << std::endl;
        client_dealer->send(address, zmq::send_flags::sndmore);
        client_dealer->send(request, zmq::send_flags::none);
      } else {
        /* 收到本client的消息，需要广播出去 */
        LOGD << "ClientDealerProc recv:  " << address.to_string() << std::endl;
        client_dealer->send(address, zmq::send_flags::sndmore);
        client_dealer->send(request, zmq::send_flags::none);
      }
    }
  }
  client_dealer->close();
  command_sub->close();
  client_dealer = nullptr;
  command_sub = nullptr;
  LOGD << "ClientDealerProc end." << std::endl;
}

void ZmqManager::CommandProc() {
  auto command_pub = std::make_shared<zmq::socket_t>(*zmq_context_, ZMQ_PUB);
  command_pub->bind(command_address_);
  command_pub->setsockopt(ZMQ_LINGER, &zero_, sizeof(zero_));

  while (!stop_) {
    MsgData send_data;
    /*
    1 kill消息, 直接kill
    2 register/unregister消息,
    3 msg
    */
    auto ret = wait_send_datas_.Pop(send_data);
    if (!ret) {
      break;
    }
    zmq::message_t address(send_data.msg_type_.data(),
                           send_data.msg_type_.size());
    auto r = command_pub->send(address, zmq::send_flags::sndmore);
    zmq::const_buffer buffer(send_data.msg_data_.data(),
                             send_data.msg_data_.size());
    r = command_pub->send(buffer, zmq::send_flags::dontwait);
  }
  // 退出时向其他线程发送退出消息
  zmq::message_t kill_subject(g_killself_subject,
                              strlen(g_killself_subject));
  command_pub->send(kill_subject, zmq::send_flags::sndmore);
  zmq::message_t kill_msg("kill", 4);  // 随意发送
  command_pub->send(kill_msg, zmq::send_flags::none);

  command_pub->close();
  command_pub = nullptr;
  LOGD << "CommandProc end." << std::endl;
}

int ZmqManager::ConnectTo(const std::string& ip, uint16_t port,
                          std::string& version) {
  if (!is_inited_ || is_online_) {
    LOGD << "ZmqManager already connected: " << ip << ":" << port_
         << ", or not Call Inited";
    return -1;
  }
  ip_ = ip;
  port_ = port;

  command_pub_handle_.PostTask(std::bind(&ZmqManager::CommandProc, this));
  std::ostringstream buffer;
  buffer << "tcp://" << ip << ":" << (port + 1);
  std::string tcpaddr = buffer.str();
  client_dealer_handle_.PostTask(
      std::bind(&ZmqManager::ClientDealerProc, this, tcpaddr, version));
  bool request_ok = false;
  {
    std::chrono::milliseconds timeout(1000);
    std::unique_lock<std::mutex> lock(first_request_mtx_);
    request_ok = first_request_cv_.wait_for(
        lock, timeout, [this] { return (firtst_reply_ == true); });
  }
  if (request_ok) {
    LOGD << "connect suc.";
  } else {
    LOGD << "session not match.";
    // return -1;
  }
  buffer.str("");
  buffer << "tcp://" << ip << ":" << port;
  tcpaddr = buffer.str();
  client_sub_handle_.PostTask(
      std::bind(&ZmqManager::ClientSubProc, this, tcpaddr));
  is_online_ = true;
  return 0;
}

int ZmqManager::ProcessOutputMsg(const ActionType action_type,
                                 const std::string& msg_type,
                                 const std::string& data, void* extra_data) {
  if (!is_online_ || stop_) {
    LOGE << "error: is_online = " << is_online_ << ", stop = " << stop_;
    return -1;
  }
  /*
  1 如果是server端, 只需要把此消息广播给client即可,
  client需要提前订阅两个subject.
  2 如果是client端, 先订阅这个subject,
  再发送给server 订阅subject, 通过socket先发送给.
  */
  if (action_type == ACTION_SEND_MSG || action_type == ACTION_SUBSCRIBE_MSG ||
      action_type == ACTION_UNSUBSCRIBE_MSG) {
    MsgData send_data(action_type, msg_type, data);
    wait_send_datas_.PushMove(send_data);
  }
  return 0;
}

int ZmqManager::GetInputMsg(ActionType& action_type, std::string& msg_type,
                            std::string& data) {
  if (!is_online_) {
    return -1;
  }
  while (!stop_) {
    MsgData msg_data;
    auto ret = recv_datas_.Pop(msg_data);
    if (false == ret) {
      LOGD << "GetInputMsg Stop.";
      return -1;
    }
    action_type = msg_data.action_type_;
    msg_type = std::move(msg_data.msg_type_);
    data = std::move(msg_data.msg_data_);
    return 0;
  }
  return -1;
}

void ZmqManager::Close() {
  if (!is_inited_) {
    return;
  }
  stop_ = true;
  LOGD << "ZmqManager::Close enter.";
  wait_send_datas_.UnInit();
  recv_datas_.UnInit();
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  if (is_online_) {
    is_online_ = false;
    zmq_context_->close();
    zmq_context_ = nullptr;
  }
  is_inited_ = false;
  LOGD << "ZmqManager::Close end.";
}

}  // namespace xproto
