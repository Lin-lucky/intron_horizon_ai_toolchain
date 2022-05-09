## XProto示例4
本节将介绍使用Xproto框架订阅跨进程的智能消息以及互发消息示例。
stage4_master_plugin.cc负责生成SmartLegibleMessage并push到总线，同时订阅了ControlMessage。
stage4_slave_plugin.cc订阅了SmartLegibleMessage，收到消息后生成一个ControlMessage并push到xproto数据总线。
### 说明
* xproto框架定义了智能消息结构，消息的生成者需要填充xproto定义好的智能消息。
* 本示例中生成master/slave两个测试程序，可以在两个板子同时用测试程序测试跨设备订阅/发布消息
* 如果slave测试程序想订阅solution中的SmartLegibleMessage，请参考multitask_perception_solution，详细说明参考其README中的第五小节

### 编译
使用build_xxx.sh脚本
例如gcc 4.8.5 ubuntu 18.04.2
```c++
build_ubuntu.sh
```
会生成stage4_ipc_subscriber_master/stage4_ipc_subscriber_slave两个测试程序

## stage4_ipc_subscriber_master
### 向框架注册消息类型
首先向框架注册消息类型
```c++
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SMART_LEGIBLE_MESSAGE);
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_CONTROL_MESSAGE);
```

### 定义插件
Xproto中插件的管理都基于`xproto::XPluginAsync`，因此自定义插件需继承于XPluginAsync。本示例中，我们需要定义一个插件：ExampleMasterPlugin获取其他设备生成的智能消息并打印结果

自定义插件需要根据功能重写`XPluginAsync::Init()`, `XPluginAsync::Start()`, `XPluginAsync::Stop()`, `XPluginAsync::DeInit()`几个核心函数。此外，可以根据需要扩充插件的成员函数，如订阅消息的插件，可以扩充消息的回调函数。

 - 其中Init()做插件的初始化工作，如在Init()中订阅消息，订阅消息使用`XPluginAsync::RegisterMsg()`接口，在订阅消息的同时指定该消息的回调函数。**注意：订阅消息后，务必调用基类的XPluginAsync::Init()接口，该插件才可接收到消息并处理。**
 - Start()主要是开始插件的运行，比如发布消息，发布消息使用`PushMsg()`接口。
 - Stop()主要是对插件做清理善后工作，如reset成员变量、内存释放等。
 - DeInit()主要负责插件的重置，**注意：若重写该函数，需要在函数内调用基类的`XPluginAsync::DeInit()`接口，取消已订阅消息类型。**

 下面看下ExampleMasterPlugin插件的定义：
 * CreateSmartLegibleMessage：用于创建SmartLegibleMessage并Push到xproto总线上
 * FeedControl：用于接收总线上的TYPE_CONTROL_MESSAGE消息
```c++
class ExampleMasterPlugin : public xproto::XPluginAsync {
 public:
  explicit ExampleMasterPlugin(std::string config_path) {
    LOGD << "ExampleMasterPlugin config_path = " << config_path;
  }
  ~ExampleMasterPlugin() {}
  int Init() override {
    LOGI << "ExampleMasterPlugin Init.";
    RegisterMsg(TYPE_CONTROL_MESSAGE,
                std::bind(&ExampleMasterPlugin::FeedControl, this,
                          std::placeholders::_1));
    return 0;
  }
  int DeInit() override {
    LOGD << "ExampleMasterPlugin::DeInit.";
    return 0;
  }
  int Start() override {
    LOGD << "ExampleMasterPlugin::Start.";
    int time = 20 * 1000;
    while (time > 0) {
      CreateSmartLegibleMessage();
      std::this_thread::sleep_for(milliseconds(30));
      time -= 33;
    }
    LOGD << "ExampleMasterPlugin::start exit.";
    return 0;
  }
  int Stop() override {
    LOGD << "ExampleMasterPlugin::Stop.";
    return 0;
  }

  std::string desc() const { return "ExampleMasterPlugin"; }

 private:
  int FeedControl(XProtoMessagePtr msg) {
    if (nullptr == msg) {
      return -1;
    }
    auto control_data = std::static_pointer_cast<ControlMessage>(msg);
    LOGD << "******ExampleMasterPlugin::FeedControl******";
    LOGD << "control timestamp = " << control_data->time_stamp_;
    LOGD << "control cmd_id = " << control_data->message_.cmd_id_;
    LOGD << "control type = " << control_data->message_.type_;
    LOGD << "control value = " << control_data->message_.value_;
    LOGD << "******ExampleMasterPlugin::FeedControl End ******";
  }
  int CreateSmartLegibleMessage() {
    LOGD << "******ExampleMasterPlugin::CreateSmartLegibleMessage******";
    auto smart_data = std::make_shared<SmartLegibleMessage>();

    auto cur_time = std::chrono::system_clock::now();
    smart_data->time_stamp_ = cur_time.time_since_epoch().count();
    smart_data->frame_id_ = frame_id_++;
    LOGD << "Create SmartLegibleMessage: time_stamp = "
         << smart_data->time_stamp_ << ", frame_id = " << smart_data->frame_id_;
    for (int index = 1; index <= 2; index++) {
      auto dst_target = std::make_shared<xproto::message::Target>();

      smart_data->smart_data_.error_code_ = 0;
      smart_data->smart_data_.targets_.push_back(dst_target);

      dst_target->type_ = "test_person";
      dst_target->track_id_ = index;
      for (int box_index = 1; box_index < 3; box_index++) {
        auto dst_box = std::make_shared<xstream::BBox>();
        dst_box->name_ = "test_box";
        dst_box->score_ = 0.999;
        dst_box->x1_ = 1.0 + box_index * 6;
        dst_box->y1_ = 100.0 + box_index * 6;
        dst_box->x2_ = 1.0 + box_index * 6;
        dst_box->y2_ = 100.0 + box_index * 6;

        dst_target->boxs_.push_back(dst_box);
      }
    }
    PushMsg(smart_data);
    LOGD << "******ExampleMasterPlugin::CreateSmartLegibleMessage end******";
    return 0;
  }
  int frame_id_ = 0;
};
```
### 使能订阅跨进程的消息
main函数中使能跨进程功能，并设置为Master节点。
xproto::XSession::Instance().AsMaster(port);
port指定Master节点使用的端口号。
```c++
int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout
        << "Usage: stage4_ipc_subscriber_master port"
        << std::endl;
    return 0;
  }
  SetLogLevel(HOBOT_LOG_DEBUG);
  uint16_t port = atoi(argv[1]);

  std::cout << "prot = " << port << std::endl;

  xproto::XSession::Instance().AsMaster(port);

  std::string example_config = "example.json";

  auto example_plg = std::make_shared<ExampleMasterPlugin>(example_config);

  example_plg->Init();

  example_plg->Start();

  // std::this_thread::sleep_for(seconds(20));

  example_plg->Stop();

  example_plg->DeInit();
  xproto::XSession::Instance().Reset();

  return 0;
}
```

## stage4_ipc_subscriber_slave
### 向框架注册消息类型
首先向框架注册消息类型
```c++
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SMART_LEGIBLE_MESSAGE);
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_CONTROL_MESSAGE);
```

### 定义插件
Xproto中插件的管理都基于`xproto::XPluginAsync`，因此自定义插件需继承于XPluginAsync。本示例中，我们需要定义一个插件：ExampleSlavePlugin获取其他设备生成的智能消息并打印结果

自定义插件需要根据功能重写`XPluginAsync::Init()`, `XPluginAsync::Start()`, `XPluginAsync::Stop()`, `XPluginAsync::DeInit()`几个核心函数。此外，可以根据需要扩充插件的成员函数，如订阅消息的插件，可以扩充消息的回调函数。

 - 其中Init()做插件的初始化工作，如在Init()中订阅消息，订阅消息使用`XPluginAsync::RegisterMsg()`接口，在订阅消息的同时指定该消息的回调函数。**注意：订阅消息后，务必调用基类的XPluginAsync::Init()接口，该插件才可接收到消息并处理。**
 - Start()主要是开始插件的运行，比如发布消息，发布消息使用`PushMsg()`接口。
 - Stop()主要是对插件做清理善后工作，如reset成员变量、内存释放等。
 - DeInit()主要负责插件的重置，**注意：若重写该函数，需要在函数内调用基类的`XPluginAsync::DeInit()`接口，取消已订阅消息类型。**

 下面看下ExampleSlavePlugin插件的定义：
 * CreateControlMessage：用于创建ControlMessage并Push到xproto总线上
 * FeedSmart：用于接收总线上的TYPE_SMART_LEGIBLE_MESSAGE消息，收到消息后创建一个ControlMessage推到总线上
```c++
class ExampleSlavePlugin : public xproto::XPluginAsync {
 public:
  explicit ExampleSlavePlugin(std::string config_path) {
    LOGD << "ExampleSlavePlugin config_path = " << config_path;
  }
  ~ExampleSlavePlugin() {}
  int Init() override {
    LOGI << "ExampleSlavePlugin Init.";
    RegisterMsg(
        TYPE_SMART_LEGIBLE_MESSAGE,
        std::bind(&ExampleSlavePlugin::FeedSmart, this, std::placeholders::_1));
    return 0;
  }
  int DeInit() override {
    LOGD << "ExampleSlavePlugin::DeInit.";
    return 0;
  }
  int Start() override {
    LOGD << "ExampleSlavePlugin::Start.";
    return 0;
  }
  int Stop() override {
    LOGD << "ExampleSlavePlugin::Stop.";
    return 0;
  }

  std::string desc() const { return "ExampleSlavePlugin"; }

 private:
  void CreateControlMessage(uint64_t id) {
    static int cc = 0;
    auto control_msg = std::make_shared<ControlMessage>();
    //  构造一个ControlMessage
    control_msg->time_stamp_ = cc++;
    control_msg->message_.cmd_id_ = id;
    control_msg->message_.type_ = "reply";
    control_msg->message_.value_ = "i have got one SmartLegibleMessage";
    PushMsg(control_msg);
    LOGD << "Slave push control msg to.";
  }
  int FeedSmart(XProtoMessagePtr msg) {
    LOGD << "ExampleSlavePlugin::FeedSmart.";
    if (nullptr == msg) {
      return -1;
    }
    auto smart_data = std::static_pointer_cast<SmartLegibleMessage>(msg);
    LOGD << "******ExampleSlavePlugin::FeedSmart******";
    LOGD << "time_stamp: " << smart_data->time_stamp_;
    std::cout << "frame_id： " << smart_data->frame_id_ << std::endl;
    if (smart_data->background_img_) {
      LOGD << "img: size = " << smart_data->background_img_->DataSize()
           << ", uv size = " << smart_data->background_img_->DataUVSize()
           << ", width = " << smart_data->background_img_->Width()
           << ", hegith = " << smart_data->background_img_->Height()
           << ", type = " << smart_data->img_serialize_type_;
      if (dump_count_++ < 5) {
        std::string dump_filename = "test_pic_";
        if (smart_data->img_serialize_type_ ==
            xproto::message::kSmartImageTypeNv12) {
          dump_filename = dump_filename + std::to_string(dump_count_) + ".yuv";
        } else if (smart_data->img_serialize_type_ ==
                   xproto::message::kSmartImageTypeJpg) {
          dump_filename = dump_filename + std::to_string(dump_count_) + ".jpg";
        }
        SaveImageFile(
            reinterpret_cast<uint8_t *>(smart_data->background_img_->Data()),
            smart_data->background_img_->DataSize(), dump_filename);
      }
    }
    for (auto t : smart_data->smart_data_.targets_) {
      LOGD << "track_id: " << t->track_id_ << ", ";
      for (auto b : t->boxs_) {
        LOGW << *b;
      }
      for (auto attr : t->attributes_) {
        LOGW << *attr;
      }
      for (auto lmk : t->lmks_) {
        LOGW << *lmk;
      }
    }
    //  收到一帧有效message, 向数据总线push一个ControlMessage
    CreateControlMessage(smart_data->frame_id_);
    LOGD << "******ExampleSlavePlugin::FeedSmart end******";
    return 0;
  }

  int SaveImageFile(uint8_t *addr, int len, std::string name) {
    FILE *fd = fopen(name.c_str(), "wb+");
    if (!fd) {
      LOGE << "open file name:%s failure!";
      return -1;
    }
    LOGD << "filename = " << name << ", len = " << len
         << ", addr = " << reinterpret_cast<int64_t>(addr);

    fwrite(addr, sizeof(char), len, fd);
    fflush(fd);
    fclose(fd);
    return 0;
  }
  int dump_count_ = 0;
};
```
### 使能订阅跨进程的消息
main函数中使能跨进程功能，并设置为Slave节点。
xproto::XSession::Instance().ConnectTo(ip, port);
ip/port指定需要链接的Master节点ip和端口号
```c++
int main(int argc, char **argv) {
  if (argc < 3) {
    std::cout
        << "Usage: stage4_ipc_subscriber ip port"
        << std::endl;
    return 0;
  }
  SetLogLevel(HOBOT_LOG_DEBUG);
  std::string ip = std::string(argv[1]);
  uint16_t port = atoi(argv[2]);
  std::cout << "ip = " << ip << ", prot = " << port << std::endl;

  xproto::XSession::Instance().ConnectTo(ip, port);

  std::string example_config = "example.json";

  auto example_plg = std::make_shared<ExampleSlavePlugin>(example_config);

  example_plg->Init();

  example_plg->Start();

  std::this_thread::sleep_for(seconds(20));

  example_plg->Stop();

  example_plg->DeInit();
  xproto::XSession::Instance().Reset();

  return 0;
}
```

### 运行程序
#### 准备工作
编译完成后，生成的目标文件在：'xproto/build/tutorials/stage4_ipc_subscriber'目录下
依赖库libxproto.so在xproto/build目录，拷贝到执行目录'xproto/build/tutorials/stage4_ipc_subscriber'
#### 运行
在Masger节点上运行：./stage4_ipc_subscriber_Master port
在Slave节点上运行：./stage4_ipc_subscriber_Slave ip port

来看下运行程序的输出结果（gcc 7.5.0 ubuntu 16.04）：
##### Masger节点输出
./stage4_ipc_subscriber_Master 6688

```
(stage4_master_plugin.cc:115): ******ExampleMasterPlugin::CreateSmartLegibleMessage******
(stage4_master_plugin.cc:121): Create SmartLegibleMessage: time_stamp = 1621590539942288085, frame_id = 598
(stage4_master_plugin.cc:144): ******ExampleMasterPlugin::CreateSmartLegibleMessage end******
(smart_legible_message.cc:58): time_stamp: 1621590539942288085
(smart_legible_message.cc:59): targets_ num = 2
(smart_legible_message.cc:61): track_id: 1, type = test_person
(smart_legible_message.cc:72): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(smart_legible_message.cc:72): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(smart_legible_message.cc:61): track_id: 2, type = test_person
(smart_legible_message.cc:72): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(smart_legible_message.cc:72): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(zmq_manager.cc:154): msg_type = XPLUGIN_CONTROL_MESSAGE
(zmq_manager.cc:177): router recv data msgtype : XPLUGIN_CONTROL_MESSAGE
(zmq_manager.cc:194): recv msg
(xsession_inner.cc:186): transfer msg: XPLUGIN_CONTROL_MESSAGE
(stage4_master_plugin.cc:107): ******ExampleMasterPlugin::FeedControl******
(stage4_master_plugin.cc:108): control timestamp = 522
(stage4_master_plugin.cc:109): control cmd_id = 598
(stage4_master_plugin.cc:110): control type = reply
(stage4_master_plugin.cc:111): control value = i have got one SmartLegibleMessage
(stage4_master_plugin.cc:112): ******ExampleMasterPlugin::FeedControl End ******
(stage4_master_plugin.cc:115): ******ExampleMasterPlugin::CreateSmartLegibleMessage******
(stage4_master_plugin.cc:121): Create SmartLegibleMessage: time_stamp = 1621590539972491529, frame_id = 599
(stage4_master_plugin.cc:144): ******ExampleMasterPlugin::CreateSmartLegibleMessage end******
(smart_legible_message.cc:58): time_stamp: 1621590539972491529
(smart_legible_message.cc:59): targets_ num = 2
(smart_legible_message.cc:61): track_id: 1, type = test_person
(smart_legible_message.cc:72): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(smart_legible_message.cc:72): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(smart_legible_message.cc:61): track_id: 2, type = test_person
(smart_legible_message.cc:72): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(smart_legible_message.cc:72): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(zmq_manager.cc:154): msg_type = XPLUGIN_CONTROL_MESSAGE
(zmq_manager.cc:177): router recv data msgtype : XPLUGIN_CONTROL_MESSAGE
(zmq_manager.cc:194): recv msg
(xsession_inner.cc:186): transfer msg: XPLUGIN_CONTROL_MESSAGE
(stage4_master_plugin.cc:107): ******ExampleMasterPlugin::FeedControl******
(stage4_master_plugin.cc:108): control timestamp = 523
(stage4_master_plugin.cc:109): control cmd_id = 599
(stage4_master_plugin.cc:110): control type = reply
(stage4_master_plugin.cc:111): control value = i have got one SmartLegibleMessage
(stage4_master_plugin.cc:112): ******ExampleMasterPlugin::FeedControl End ******
(stage4_master_plugin.cc:115): ******ExampleMasterPlugin::CreateSmartLegibleMessage******
(stage4_master_plugin.cc:121): Create SmartLegibleMessage: time_stamp = 1621590540002743439, frame_id = 600
(stage4_master_plugin.cc:144): ******ExampleMasterPlugin::CreateSmartLegibleMessage end******
(smart_legible_message.cc:58): time_stamp: 1621590540002743439
(smart_legible_message.cc:59): targets_ num = 2
(smart_legible_message.cc:61): track_id: 1, type = test_person
(smart_legible_message.cc:72): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(smart_legible_message.cc:72): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(smart_legible_message.cc:61): track_id: 2, type = test_person
(smart_legible_message.cc:72): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(smart_legible_message.cc:72): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(zmq_manager.cc:154): msg_type = XPLUGIN_CONTROL_MESSAGE
(zmq_manager.cc:177): router recv data msgtype : XPLUGIN_CONTROL_MESSAGE
(zmq_manager.cc:194): recv msg
(xsession_inner.cc:186): transfer msg: XPLUGIN_CONTROL_MESSAGE
(stage4_master_plugin.cc:107): ******ExampleMasterPlugin::FeedControl******
(stage4_master_plugin.cc:108): control timestamp = 524
(stage4_master_plugin.cc:109): control cmd_id = 600
(stage4_master_plugin.cc:110): control type = reply
(stage4_master_plugin.cc:111): control value = i have got one SmartLegibleMessage
(stage4_master_plugin.cc:112): ******ExampleMasterPlugin::FeedControl End ******
```

##### Slave节点输出
./stage4_ipc_subscriber_Slave xxx.xxx.xxx.xxx 6688

```
(stage4_slave_plugin.cc:106): ExampleSlavePlugin::FeedSmart.
(stage4_slave_plugin.cc:111): ******ExampleSlavePlugin::FeedSmart******
(stage4_slave_plugin.cc:112): time_stamp: 1621590540123554771
frame_id： 604
(stage4_slave_plugin.cc:135): track_id: 1, 
(stage4_slave_plugin.cc:137): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(stage4_slave_plugin.cc:137): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(stage4_slave_plugin.cc:135): track_id: 2, 
(stage4_slave_plugin.cc:137): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(stage4_slave_plugin.cc:137): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(stage4_slave_plugin.cc:103): Slave push control msg to.
(stage4_slave_plugin.cc:148): ******ExampleSlavePlugin::FeedSmart end******
(zmq_manager.cc:362): ClientDealerProc recv:  XPLUGIN_CONTROL_MESSAGE

(zmq_manager.cc:275): recv data: 1621590540154
(stage4_slave_plugin.cc:106): ExampleSlavePlugin::FeedSmart.
(stage4_slave_plugin.cc:111): ******ExampleSlavePlugin::FeedSmart******
(stage4_slave_plugin.cc:112): time_stamp: 1621590540153833415
frame_id： 605
(stage4_slave_plugin.cc:135): track_id: 1, 
(stage4_slave_plugin.cc:137): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(stage4_slave_plugin.cc:137): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(stage4_slave_plugin.cc:135): track_id: 2, 
(stage4_slave_plugin.cc:137): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(stage4_slave_plugin.cc:137): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(stage4_slave_plugin.cc:103): Slave push control msg to.
(stage4_slave_plugin.cc:148): ******ExampleSlavePlugin::FeedSmart end******
(zmq_manager.cc:362): ClientDealerProc recv:  XPLUGIN_CONTROL_MESSAGE

(zmq_manager.cc:275): recv data: 1621590540184
(stage4_slave_plugin.cc:106): ExampleSlavePlugin::FeedSmart.
(stage4_slave_plugin.cc:111): ******ExampleSlavePlugin::FeedSmart******
(stage4_slave_plugin.cc:112): time_stamp: 1621590540184095231
frame_id： 606
(stage4_slave_plugin.cc:135): track_id: 1, 
(stage4_slave_plugin.cc:137): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(stage4_slave_plugin.cc:137): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(stage4_slave_plugin.cc:135): track_id: 2, 
(stage4_slave_plugin.cc:137): ( x1: 7 y1: 106 x2: 7 y2: 106 score: 0.999 )
(stage4_slave_plugin.cc:137): ( x1: 13 y1: 112 x2: 13 y2: 112 score: 0.999 )
(stage4_slave_plugin.cc:103): Slave push control msg to.
(stage4_slave_plugin.cc:148): ******ExampleSlavePlugin::FeedSmart end******
```
