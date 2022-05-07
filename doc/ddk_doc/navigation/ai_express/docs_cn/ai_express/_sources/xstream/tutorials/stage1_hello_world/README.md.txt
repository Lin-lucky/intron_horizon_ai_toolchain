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
将stage1_hello_world目录下生产的output目录拷贝至XJ3系统，即可运行。运行使用如下命令
```shell
  // 异步方式
  ./stage1_hello_world  bbox_workflow.json async
  // 同步方式
  ./stage1_hello_world  bbox_workflow.json  sync
```

## 3. 构造简单workflow
本节将介绍如何搭建一个简单的workflow，并运行起来输出结果。

首先我们来实现一个简单的检测框过滤的workflow，具体如下。workflow输入数据是face_head_box, 输入数据经过BBoxFilter节点的运算，最终输出数据是face_head_box_filter。其中BBoxFilter作用是过滤掉置信度小于阈值的框。

```json
{
    "inputs": ["face_head_box"],   // workflow输入数据
    "outputs": ["face_head_box_filter"],   // workflow输出数据
    "workflow": [
      {
        "method_type": "BBoxFilter",   // Method类型
        "unique_name": "bbox_filter",  // Node的唯一标志
        "inputs": [
          "face_head_box"              // Node的输入
        ],
        "outputs": [
          "face_head_box_filter"       // Node的输出
        ],
        "method_config_file": "null"   // Node对应的Method的配置文件
      }
    ]
}
```

### 3.1. 定义XStream框架BBox数据结构
确定好workflow后，我们还需要定义workflow中用到的数据结构。由于XStream Framework中的数据都是基于BaseData，因此需要构建BaseData的派生数据结构。

```c++
// method/bbox.h
struct BBox : public BaseData {
  inline BBox() {}
  inline BBox(float x1_, float y1_, float x2_, float y2_,
              float score_ = 0) {
    x1 = x1_;
    y1 = y1_;
    x2 = x2_;
    y2 = y2_;
    score = score_;
  }

  float x1 = 0;
  float y1 = 0;
  float x2 = 0;
  float y2 = 0;
  float score = 0;
};
typedef std::shared_ptr<BBox> BBoxPtr;
```

### 3.2. 定义BBoxFilter Method
现在需要实现BBoxFilter Method的核心功能。XStream中定义了SimpleMethod基础类，我们需要在此基础上实现自定义的BBoxFilter，并根据需要实现`DoProcess`这个核心处理函数，以及扩充成员变量，如置信度阈值`score_threshold_`。

```c++
class BBoxFilter : public SimpleMethod {
 private:
  float score_threshold_ = 0.5;

 public:
  int Init(const std::string &config_file_path) override;

  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override;

  void Finalize() override;


  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override;
};
```

`DoProcess`的实现，过滤掉小于score_threshold_的BBox：
```c++
  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override {
    std::cout << "BBoxScoreFilter::DoProcess " << input.size() << std::endl;
    std::vector<BaseDataPtr> output;

    // one frame
    for (size_t j = 0; j < input.size(); j++) {
      output.push_back(std::make_shared<BaseDataVector>());
      if (input[j]->state_ == DataState::INVALID) {
        std::cout << "input slot " << j << " is invalid" << std::endl;
        continue;
      }
      auto in_rects = std::static_pointer_cast<BaseDataVector>(input[j]);
      auto out_rects = std::static_pointer_cast<BaseDataVector>(output[j]);
      for (auto &in_rect : in_rects->datas_) {
        auto bbox = std::static_pointer_cast<BBox>(in_rect);
        if (bbox->score > score_threshold_) {
          out_rects->datas_.push_back(in_rect);
        } else {
          std::cout << "filter out: " << bbox->x1 << "," << bbox->y1 << ","
                    << bbox->x2 << "," << bbox->y2 << ", score: " << bbox->score
                    << std::endl;
        }
      }
    }
    return output;
  }
```

### 3.3. 注册BBoxFilter Method到MethodFactory
XStream框架构建workflow时，调用全局MethodFactory创建对应Method实例，在使用BBoxFilter之前需要注册到MethodFactory中。也就是说程序需要根据workflow中的`"method_type": "BBoxFilter"`，对应程序中具体的自定义Method。做法如下：
```c++
namespace method_factory {
MethodPtr CreateMethod(const std::string &method_name) {
  if ("BBoxFilter" == method_name) {
    return MethodPtr(new BBoxFilter());
  } else {
    return MethodPtr();
  }
}
}  // namespace method_factory
```

### 3.4. 运行workflow
首先需要创建XStream SDK并初始化，SDK是运行workflow的对外接口对象。通过`SetConfig()`接口配置workflow json文件，然后调用`Init()`进行初始化。如果运行在异步模式下，还需要通过`SetCallback()`配置输出结果的回调函数。

```c++
  // Create xstream sdk object
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  // Set config_file
  flow->SetConfig("config_file", config);

  // Set CallBack Func For Async Mode
  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  // Init
  flow->Init();
```

然后创建BBox输入数据，包括4个BBox框(0, 0, 10, 20, 0.1)，(0, 0, 4, 5, 0.3)，(0, 0, 6, 8, 0.7)，(0, 0, 8, 8, 0.9)。这里需要注意的是，由于框架需要根据workflow把输入数据送入对应的Node，所以**输入数据的`BaseData::name_`需要和workflow中的对应**，否则框架会找不到对应的输入数据而抛出错误。

```c++
  InputDataPtr inputdata(new InputData());
  BaseDataVector *data(new BaseDataVector);
  xstream::BBoxPtr bbox1 = std::make_shared<xstream::BBox>(0, 0, 10, 20, 0.1);
  xstream::BBoxPtr bbox2 = std::make_shared<xstream::BBox>(0, 0, 4, 5, 0.3);
  xstream::BBoxPtr bbox3 = std::make_shared<xstream::BBox>(0, 0, 6, 8, 0.7);
  xstream::BBoxPtr bbox4 = std::make_shared<xstream::BBox>(0, 0, 8, 8, 0.9);
  data->name_ = "face_head_box";   // corresponding the inputs in workflow
  data->datas_.push_back(bbox1);
  data->datas_.push_back(bbox2);
  data->datas_.push_back(bbox3);
  data->datas_.push_back(bbox4);
  inputdata->datas_.push_back(BaseDataPtr(data));
```

下面可以将输入数据送入框架进行运算，运算分为同步和异步两种模式，`SyncPredict()`是同步运行接口，`AsyncPredict()`是异步运行接口。同步模式下，接口返回输出数据；异步模式下，接口直接返回，运行结束后回调函数会自动处理输出数据。

```c++
  // sync mode
    auto out = flow->SyncPredict(inputdata);
    callback.OnCallback(out);

  // async mode
  flow->AsyncPredict(inputdata);
```

来看下经过上述workflow后的结果，BBoxFilter中默认置信度阈值score_threshold_ = 0.5。输出结果包括BBox(0, 0, 6, 8, 0.7)和(0, 0, 8, 8, 0.9)，具体如下：

```
============Output Call Back============
—seq: 0
—output_type: __NODE_WHOLE_OUTPUT__
—error_code: 0
—error_detail_: 
—datas_ size: 1
——output data face_head_box_filter state:0
——output data: ( x1: 0 y1: 0 x2: 6 y2: 8 score: 0.7 )
——output data: ( x1: 0 y1: 0 x2: 8 y2: 8 score: 0.9 )
============Output Call Back End============
```