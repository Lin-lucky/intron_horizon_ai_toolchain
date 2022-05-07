# Hello World
## 1. 编译
进入stage1_hello_world目录，执行如下步骤

```shell
  mkdir build
  cd build
  cmake .. -DINDEPENDENT_BUILD=ON
  make
  make install
```

## 2. 运行
将stage1_hello_world目录下产生的output目录拷贝至XJ3系统，即可运行。运行使用如下命令
```shell
  ./stage1_hello_world
```

## 3. XProto简单示例
本节将介绍如何使用Xproto框架实现不同插件之间消息的发布和订阅。我们来实现一个简单的示例，示例中包括两个插件：一个插件负责产生数据并发布，另个插件订阅数据并将接收到的数据进行累加。

### 3.1. 定义消息类型
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

### 3.2. 定义插件
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
  // 初始化
  int Init() {
    total_cnt_ = 50;
    prd_thread_ = nullptr;
    return XPluginAsync::Init();
  }
  // 发布消息
  int Start() {
    prd_thread_ = new std::thread([&] (){
      // 累计发布50次消息，数值为5
      for (uint32_t i = 0; i < total_cnt_ && !prd_stop_; i++) {
        auto np_msg = std::make_shared<NumberProdMessage>(5);
        PushMsg(np_msg);
        std::this_thread::sleep_for(milliseconds(40));
      }
    });
    return 0;
  }
  // 内存释放
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
  // 初始化，订阅消息
  int Init() override {
    sum_ = 0.f;
    RegisterMsg(TYPE_SAMPLE_MESSAGE, std::bind(&SumConsumerPlugin::Sum,
                                               this, std::placeholders::_1));
    return XPluginAsync::Init();
  }
  // 消息回调函数，对收到的消息做累加
  int Sum(XProtoMessagePtr msg) {
    auto np_msg = std::static_pointer_cast<NumberProdMessage>(msg);
    sum_ += np_msg->num_;
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

### 3.3. 运行程序
消息和插件都定义完成后，现在需要将插件运作起来。只需实例化两个插件，并顺序执行Init()、Start()、Stop()、DeInit()即可。

```c++
int main() {
  auto np_plg = std::make_shared<NumberProducerPlugin>();
  auto sc_plg = std::make_shared<SumConsumerPlugin>();

  np_plg->Init();
  sc_plg->Init();

  np_plg->Start();
  sc_plg->Start();

  std::this_thread::sleep_for(seconds(3));

  np_plg->Stop();
  sc_plg->Stop();

  np_plg->DeInit();
  sc_plg->DeInit();

  return 0;
}
```

来看下运行程序的输出结果，可见两个插件配合完成了50次求和操作：
```
(stage1_sample_plugin.cpp:81): total_cnt=50
(stage1_sample_plugin.cpp:82): NumberProducerPlugin Start
(stage1_sample_plugin.cpp:127): SumConsumerPlugin Start
(stage1_sample_plugin.cpp:122): curr sum:5
(stage1_sample_plugin.cpp:122): curr sum:10
(stage1_sample_plugin.cpp:122): curr sum:15
(stage1_sample_plugin.cpp:122): curr sum:20
(stage1_sample_plugin.cpp:122): curr sum:25
(stage1_sample_plugin.cpp:122): curr sum:30
(stage1_sample_plugin.cpp:122): curr sum:35
(stage1_sample_plugin.cpp:122): curr sum:40
(stage1_sample_plugin.cpp:122): curr sum:45
(stage1_sample_plugin.cpp:122): curr sum:50
(stage1_sample_plugin.cpp:122): curr sum:55
(stage1_sample_plugin.cpp:122): curr sum:60
(stage1_sample_plugin.cpp:122): curr sum:65
(stage1_sample_plugin.cpp:122): curr sum:70
(stage1_sample_plugin.cpp:122): curr sum:75
(stage1_sample_plugin.cpp:122): curr sum:80
(stage1_sample_plugin.cpp:122): curr sum:85
(stage1_sample_plugin.cpp:122): curr sum:90
(stage1_sample_plugin.cpp:122): curr sum:95
(stage1_sample_plugin.cpp:122): curr sum:100
(stage1_sample_plugin.cpp:122): curr sum:105
(stage1_sample_plugin.cpp:122): curr sum:110
(stage1_sample_plugin.cpp:122): curr sum:115
(stage1_sample_plugin.cpp:122): curr sum:120
(stage1_sample_plugin.cpp:122): curr sum:125
(stage1_sample_plugin.cpp:122): curr sum:130
(stage1_sample_plugin.cpp:122): curr sum:135
(stage1_sample_plugin.cpp:122): curr sum:140
(stage1_sample_plugin.cpp:122): curr sum:145
(stage1_sample_plugin.cpp:122): curr sum:150
(stage1_sample_plugin.cpp:122): curr sum:155
(stage1_sample_plugin.cpp:122): curr sum:160
(stage1_sample_plugin.cpp:122): curr sum:165
(stage1_sample_plugin.cpp:122): curr sum:170
(stage1_sample_plugin.cpp:122): curr sum:175
(stage1_sample_plugin.cpp:122): curr sum:180
(stage1_sample_plugin.cpp:122): curr sum:185
(stage1_sample_plugin.cpp:122): curr sum:190
(stage1_sample_plugin.cpp:122): curr sum:195
(stage1_sample_plugin.cpp:122): curr sum:200
(stage1_sample_plugin.cpp:122): curr sum:205
(stage1_sample_plugin.cpp:122): curr sum:210
(stage1_sample_plugin.cpp:122): curr sum:215
(stage1_sample_plugin.cpp:122): curr sum:220
(stage1_sample_plugin.cpp:122): curr sum:225
(stage1_sample_plugin.cpp:122): curr sum:230
(stage1_sample_plugin.cpp:122): curr sum:235
(stage1_sample_plugin.cpp:122): curr sum:240
(stage1_sample_plugin.cpp:122): curr sum:245
(stage1_sample_plugin.cpp:122): curr sum:250
(stage1_sample_plugin.cpp:98): NumberProducerPlugin Stop
(stage1_sample_plugin.cpp:130): SumConsumerPlugin Stop
(msg_manager.h:68): to erase plugin: SumConsumerPlugin
```