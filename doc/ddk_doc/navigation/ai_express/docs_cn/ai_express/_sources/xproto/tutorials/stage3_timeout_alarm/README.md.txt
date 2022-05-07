## XProto示例3
本节将介绍使用Xproto框架对消息处理的耗时进行监控。我们来实现一个简单的示例，示例中包括两个插件：一个插件负责产生数据并发布，另个插件订阅数据并将接收到的数据进行累加，在执行完累加的动作后，休眠1005毫秒，模拟plugin处理消息超时。
### 超时监控说明
* 框架默认消息处理超时时间为1000毫秒，也可以通过环境变量`export msg_timeout_monitor="200"`设置新的超时时间，对所有plugin生效
* 框架在RegisterPlugin接口中设置默认超时时间，plugin也可以调用SetMsgMonitorTime接口设置自己的超时时间，在注册消息后调用才能生效
* GetMsgMonitorTime接口用于获取plugin的超时时间
### 定义消息类型
首先需要定义不同插件间传递消息的类型，Xproto框架中的消息都是基于`xproto::XProtoMessage`，因此需要构建XProtoMessage的派生消息结构，根据需要扩充成员变量。**注意：不同插件间通过XProtoMessage::type_来区分消息类型，因此自定义消息时需要指定唯一`type_`。同时需要使用接口`XPLUGIN_REGISTER_MSG_TYPE`，向总线注册该消息类型。**

```c++
#define TYPE_SAMPLE_MESSAGE "XPLUGIN_SAMPLE_MESSAGE"    // 消息类型
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SAMPLE_MESSAGE)       // 注册消息

struct NumberProdMessage : XProtoMessage {              // 自定义消息结构
  float num_;
  explicit NumberProdMessage(float num) :num_(num) {
    type_ = TYPE_SAMPLE_MESSAGE;
  }
  std::string Serialize() override {
    std::ostringstream ss;
    ss << num_;
    return std::string(ss.str());
  }
};
```

### 定义插件
Xproto中插件的管理都基于`xproto::XPluginAsync`，因此自定义插件需继承于XPluginAsync。本示例中，我们需要定义两个插件：NumberProducerPlugin产生浮点数，SumConsumerPlugin实现数据累加。

自定义插件需要根据功能重写`XPluginAsync::Init()`, `XPluginAsync::Start()`, `XPluginAsync::Stop()`, `XPluginAsync::DeInit()`几个核心函数。此外，可以根据需要扩充插件的成员函数，如订阅消息的插件，可以扩充消息的回调函数。

 - 其中Init()做插件的初始化工作，如在Init()中订阅消息，订阅消息使用`XPluginAsync::RegisterMsg()`接口，在订阅消息的同时指定该消息的回调函数。**注意：订阅消息后，务必调用基类的XPluginAsync::Init()接口，该插件才可接收到消息并处理。**
 - Start()主要是开始插件的运行，比如发布消息，发布消息使用`PushMsg()`接口。
 - Stop()主要是对插件做清理善后工作，如reset成员变量、内存释放等。
 - DeInit()主要负责插件的重置，**注意：若重写该函数，需要在函数内调用基类的`XPluginAsync::DeInit()`接口，取消已订阅消息类型。**

 下面看下NumberProducerPlugin和SumConsumerPlugin两个插件的定义：
```c++
class NumberProducerPlugin : public XPluginAsync {
 public:
  std::string desc() const {
    return "NumberProducerPlugin";
  }
  int Init() {
    total_cnt_ = 5;
    prd_thread_ = nullptr;
    return XPluginAsync::Init();
  }
  int Start() {
    prd_thread_ = new std::thread([&] (){
      for (uint32_t i = 0; i < total_cnt_ && !prd_stop_; i++) {
        auto np_msg = std::make_shared<NumberProdMessage>(5);
        PushMsg(np_msg);
        std::this_thread::sleep_for(milliseconds(40));
      }
    });
    return 0;
  }
  int Stop() {
    prd_stop_ = true;
    prd_thread_->join();
    if (prd_thread_) {
      delete prd_thread_;
    }
    return 0;
  }
  int DeInit() {
    return XPluginAsync::DeInit();
  }

 private:
  uint32_t total_cnt_;
  std::thread *prd_thread_;
  bool prd_stop_{false};
};

class SumConsumerPlugin : public XPluginAsync {
 public:
  int Init() override {
    sum_ = 0.f;
    RegisterMsg(TYPE_SAMPLE_MESSAGE, std::bind(&SumConsumerPlugin::Sum,
                                               this, std::placeholders::_1));
    return XPluginAsync::Init();
  }
  int Sum(XProtoMessagePtr msg) {
    auto np_msg = std::static_pointer_cast<NumberProdMessage>(msg);
    sum_ += np_msg->num_;
    std::this_thread::sleep_for(milliseconds(1005));
    return sum_;
  }

  int Start() {
    return 0;
  }
  int Stop() {
    return 0;
  }
  int DeInit() {
    return XPluginAsync::DeInit();
  }
  std::string desc() const {
    return "SumConsumerPlugin";
  }

 private:
  float sum_;
};
```

### 运行程序
消息和插件都定义完成后，现在需要将插件运作起来。只需实例化两个插件，并顺序执行Init()、Start()、Stop()、DeInit()即可。

```c++
int main() {
  auto np_plg = std::make_shared<NumberProducerPlugin>();
  auto sc_plg = std::make_shared<SumConsumerPlugin>();

  np_plg->Init();
  sc_plg->Init();

  np_plg->Start();
  sc_plg->Start();

  std::this_thread::sleep_for(seconds(6));

  np_plg->Stop();
  sc_plg->Stop();

  np_plg->DeInit();
  sc_plg->DeInit();

  return 0;
}
```

来看下运行程序的输出结果，Consumer插件处理msg耗时1005ms，框架提示消息处理超时信息，信息中包括插件名称和消息名称等信息：
```
(xpluginasync.cpp:107): XPluginAsync() cons
(xpluginasync.cpp:107): XPluginAsync() cons
(stage3_sample_plugin.cpp:81): total_cnt=5
(stage3_sample_plugin.cpp:82): NumberProducerPlugin Start
(stage3_sample_plugin.cpp:128): SumConsumerPlugin Start
(stage3_sample_plugin.cpp:123): curr sum:5
(xpluginasync.cpp:100): Plugin: SumConsumerPlugin, MsgType = XPLUGIN_SAMPLE_MESSAGE, cost time = 1005(ms)
(stage3_sample_plugin.cpp:123): curr sum:10
(xpluginasync.cpp:100): Plugin: SumConsumerPlugin, MsgType = XPLUGIN_SAMPLE_MESSAGE, cost time = 1005(ms)
(stage3_sample_plugin.cpp:123): curr sum:15
(xpluginasync.cpp:100): Plugin: SumConsumerPlugin, MsgType = XPLUGIN_SAMPLE_MESSAGE, cost time = 1005(ms)
(stage3_sample_plugin.cpp:123): curr sum:20
(xpluginasync.cpp:100): Plugin: SumConsumerPlugin, MsgType = XPLUGIN_SAMPLE_MESSAGE, cost time = 1005(ms)
(stage3_sample_plugin.cpp:123): curr sum:25
(xpluginasync.cpp:100): Plugin: SumConsumerPlugin, MsgType = XPLUGIN_SAMPLE_MESSAGE, cost time = 1005(ms)
(stage3_sample_plugin.cpp:98): NumberProducerPlugin Stop
(stage3_sample_plugin.cpp:132): SumConsumerPlugin Stop
(msg_manager.h:73): to erase plugin: SumConsumerPlugin
```