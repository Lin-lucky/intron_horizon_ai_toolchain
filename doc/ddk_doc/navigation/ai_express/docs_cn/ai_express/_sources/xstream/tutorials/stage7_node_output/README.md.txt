# Node输出回调

# 简介

xstream内部提供了灵活的回调功能，当使用xstream时，当整个workflow还在处理数据，没有运行结束，但是中间某个method结点已经执行结束的时候，可直接通过回调函数获取中间某个已经结束的结点的运行结果。

本教程将展示串联2个method，并在BBoxFilterA中sleep 10ms，在BBoxFilter中Sleep 100ms，当BBoxFilterA结点运行结束后，会直接调用BBoxFilterA的回调函数OnCallbackBBoxFilterA，当BBoxFilterB运行结束后，会自动调用整个workflow的回调函数OnCallback。

# 开始

## 第1步： 构建workflow

我们构造一个workflow，这个workflow中什么也不做，只是数据流动和一些普通计算。

```json
// xstream/tutorial/stage7_node_output/config/filter.json
 1    {
 2      "inputs": [
 3        "head_box"
 4      ],
 5      "outputs": [
 6        "head_box_filter2"
 7      ],
 8      "workflow": [
 9        {
10          "method_type": "BBoxFilterA",
11          "unique_name": "BBoxFilter_1",
12          "inputs": [
13            "head_box"
14          ],
15          "outputs": [
16            "head_box_filter"
17          ],
18          "method_config_file": "null"
19        },
20        {
21          "method_type": "BBoxFilterB",
22          "unique_name": "BBoxFilter_2",
23          "inputs": [
24            "head_box_filter"
25          ],
26          "outputs": [
27            "head_box_filter2"
28          ],
29          "method_config_file": "null"
30        }
31      ]
32    }
```

xstream的配置文件是以json格式为主，filter.json每个字段的解释如下:

line 2: inputs字段，用来表示整个workflow的输入，这里输入的是head_box(人头框)。

line 5: outputs字段，用来表示整个workflow的输出，这里输出的是head_box_filter2(经过filter2过滤后人头框)。

line 8: workflow字段，用来表示整个workflow的开始，方括号[ ]中的内容包含整个workflow中的method结点。

line 10: method_type字段，用来表示method的类型，对应代码里的BBoxFilterA类(bbox_filter_a.h)。

line 11: unique_name字段，用于xstream框架内部node的标识（这里可暂时不关注）。

line 12: inputs字段，用来标识workflow这个json数组第1个元素的输入，即用来标识workflow中第1个method结点的输入，这里value是head_box，也就是整个workflow的输入作为第1个method结点的输入，从这个method结点开始被处理。

line 15: outputs字段，用来表示workflow中第一个method结点的输出，即，head_box_filter。

line 18: method_config_file字段，用来表示每个method结点对应的配置文件。（在stage7中没有用到）。

line 21: method_type字段，workflow的第二个元素的method类型，对应代码中的BBoxFilterB类(bbox_filter_b.h)。

line 22: unique_name字段，用于xstream框架内部node的标识（这里可暂时不关注）。

line 23: inputs字段，用来标识workflow这个json数组第2个元素的输入，即用来标识workflow中第2个method结点的输入，这里value是head_box_filter，也就是workflow中第1个method结点的输出作为第2个method结点的输入。

line 26: outputs字段，用来表示workflow中第2个method结点的输出，即，head_box_filter2，同时因为BBoxFilterB也是workkflow的最后一个结点，这里它也是整个workflow中输出的一部分，对应line6中的head_box_filter2。

line 29: method_config_file字段，用来表示每个method结点对应的配置文件。（在stage7中没有用到）。


## 第2步：定义xstream框架的数据结构

为了方便地对数据进行统一的处理，xstream模块内部流动的数据有统一的格式，都继承自BaseData类，也就是每个流动的数据中的都有BaseData类中的成员。

BaseData类定义如下：

```C++
// xstream/framework/include/hobotxstream/xstream_data.h
struct BaseData {
  BaseData();
  virtual ~BaseData();
  // type
  std::string type_ = "";
  // name
  std::string name_ = "";
  // error code
  int error_code_ = 0;
  // error detail info
  std::string error_detail_ = "";
  // context of C structure
  std::shared_ptr<CContext> c_data_;
  // data status
  DataState state_ = DataState::VALID;
};

```

继承自BaseData类的BBox类定义如下：

```C++
// xstream/tutorial/stage7_node_output/include/method/bbox.h
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
  inline float Width() const { return (x2 - x1); }
  inline float Height() const { return (y2 - y1); }
  inline float CenterX() const { return (x1 + (x2 - x1) / 2); }
  inline float CenterY() const { return (y1 + (y2 - y1) / 2); }

  inline friend std::ostream &operator<<(std::ostream &out, BBox &bbox) {
    out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2
        << " y2: " << bbox.y2 << " score: " << bbox.score << " )";
    return out;
  }

  inline friend std::ostream &operator<<(std::ostream &out, const BBox &bbox) {
    out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2
        << " y2: " << bbox.y2 << " score: " << bbox.score << " )";
    return out;
  }

  float x1 = 0;
  float y1 = 0;
  float x2 = 0;
  float y2 = 0;
  float score = 0;
};

typedef std::shared_ptr<BBox> BBoxPtr;

```

## 第3步：根据具体需求，定义Method

根据具体需求，定义具体的Method, 以BBoxFilterA为例：

```C++
// xstream/tutorial/stage7_node_output/include/method/bbox_filter_a.h
class BBoxFilterA : public SimpleMethod {
 public:
  int Init(const std::string &config_file_path) override;

  virtual std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param);

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override;

 private:
  std::atomic<float> area_threshold_;
};

class BBoxFilterAParam : public xstream::InputParam {
 public:
  explicit BBoxFilterAParam(const std::string &module_name) :
           xstream::InputParam(module_name) {}
  std::string Format() override {
    return "";
  }
};
```

BBoxFilterA类中，DoProcess为需要使用者自己实现的成员函数，xstream模块内部会通过回调，调用DoProcess成员函数，通过DoProcess，对输入的数据(head_box)进行过滤处理。

```C++
// xstream/tutorial/stage7_node_output/include/method/callback.h
std::vector<BaseDataPtr> BBoxFilterA::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr &param) {

  std::cout << "BBoxFilterA::DoProcess begin " << input.size() << std::endl;
  std::vector<BaseDataPtr> output;
  // one batch
  for (size_t j = 0; j < input.size(); j++) {
    output.push_back(std::make_shared<BaseDataVector>());
    if (input[j]->state_ == DataState::INVALID) {
      // std::cout << "input slot " << j << " is invalid" << std::endl;
      continue;
    }
    auto in_rects = std::static_pointer_cast<BaseDataVector>(input[j]);
    auto out_rects = std::static_pointer_cast<BaseDataVector>(output[j]);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::cout << "BBoxFilterA::DoProcessing end " << std::endl;
  return output;
}
```

## 第4步：在Callback类中，编写自己的回调函数

`void OnCallbackBBoxFilterA(xstream::OutputDataPtr output)`为BBoxFilterA的回调函数

`void OnCallback(xstream::OutputDataPtr output)` 为整个workflow的回调函数

```C++
// xstream/tutorial/stage7_node_output/include/method/callback.h
class Callback {
 public:
  void OnCallbackBBoxFilterA(xstream::OutputDataPtr output) { ParseOutputA(output); }
  void OnCallback(xstream::OutputDataPtr output) { ParseOutput(output); }

 private:

  void ParseOutputA(xstream::OutputDataPtr output) {
    using xstream::BaseDataVector;

    std::cout << "========BBoxFilterA CallBack begin==============" << std::endl;
    std::cout << "seq: " << output->sequence_id_ << std::endl;
    std::cout << "output_type: " << output->output_type_ << std::endl;
    std::cout << "method_unique_name: " << output->unique_name_ << std::endl;
    std::cout << "error_code: " << output->error_code_ << std::endl;
    std::cout << "error_detail_: " << output->error_detail_ << std::endl;
    std::cout << "datas_ size: " << output->datas_.size() << std::endl;

    for (auto data : output->datas_) {
      if (data->error_code_ < 0) {
        std::cout << "data error: " << data->error_code_ << std::endl;
        continue;
      }
      std::cout << "data type_name : " << data->type_ << " " << data->name_
                << std::endl;
      BaseDataVector *pdata = reinterpret_cast<BaseDataVector *>(data.get());
      std::cout << "pdata size: " << pdata->datas_.size() << std::endl;
      std::cout << "Output BBox " << pdata->name_ << ":" << std::endl;
      for (size_t i = 0; i < pdata->datas_.size(); ++i) {
        auto xstream_box =
            std::static_pointer_cast<xstream::BBox>(
                pdata->datas_[i]);
        if (xstream_box->state_ == xstream::DataState::VALID) {
          std::cout << "[" << xstream_box->value.x1 << ","
                    << xstream_box->value.y1 << "," << xstream_box->value.x2
                    << "," << xstream_box->value.y2 << "]" << std::endl;
        } else {
          std::cout << "pdata->datas_[ " << i
                    << " ]: state_:" << static_cast<int>(xstream_box->state_)
                    << std::endl;
        }
      }
    }
    std::cout << "========BBoxFilterA CallBack end==============" << std::endl;
  }

  void ParseOutput(xstream::OutputDataPtr output) {
    using xstream::BaseDataVector;

    std::cout << "========WorkFlow CallBack begin==============" << std::endl;
    std::cout << "seq: " << output->sequence_id_ << std::endl;
    std::cout << "output_type: " << output->output_type_ << std::endl;
    std::cout << "method_unique_name: " << output->unique_name_ << std::endl;
    std::cout << "error_code: " << output->error_code_ << std::endl;
    std::cout << "error_detail_: " << output->error_detail_ << std::endl;
    std::cout << "datas_ size: " << output->datas_.size() << std::endl;

    for (auto data : output->datas_) {
      if (data->error_code_ < 0) {
        std::cout << "data error: " << data->error_code_ << std::endl;
        continue;
      }
      std::cout << "data type_name : " << data->type_ << " " << data->name_
                << std::endl;
      BaseDataVector *pdata = reinterpret_cast<BaseDataVector *>(data.get());
      std::cout << "pdata size: " << pdata->datas_.size() << std::endl;
      std::cout << "Output BBox " << pdata->name_ << ":" << std::endl;
      for (size_t i = 0; i < pdata->datas_.size(); ++i) {
        auto xstream_box =
            std::static_pointer_cast<xstream::BBox>(
                pdata->datas_[i]);
        if (xstream_box->state_ == xstream::DataState::VALID) {
          std::cout << "[" << xstream_box->value.x1 << ","
                    << xstream_box->value.y1 << "," << xstream_box->value.x2
                    << "," << xstream_box->value.y2 << "]" << std::endl;
        } else {
          std::cout << "pdata->datas_[ " << i
                    << " ]: state_:" << static_cast<int>(xstream_box->state_)
                    << std::endl;
        }
      }
    }
    std::cout << "========WorkFlow CallBack end==============" << std::endl;
  }
};

```


## 第5步：注册Method到MehtodFactory中

在使用BBoxFilterA和BBoxFilterB类之前，要注册这两个类到xstream的MethodFactory中。对应xstream/framework/tutorial/stage7/config/filter.json中的method_type字段。

```C++
// xstream/tutorial/stage7_node_output/include/method/method_factory.cc
MethodPtr CreateMethod(const std::string &method_name) {
  if (method_name == "BBoxFilterA") {
    return MethodPtr(new BBoxFilterA());
  } else if (method_name == "BBoxFilterB"){
    return MethodPtr(new BBoxFilterB());
  } else {
    return MethodPtr();
  }
}
```

## 第6步：编写main函数，串联整个workflow
```C++
// xstream/tutorial/stage7_node_output/src/main.cc
int main(int argc, char const *argv[]) {
  using Stage7Async::Callback;
  using xstream::BaseData;
  using xstream::BaseDataPtr;
  using xstream::BaseDataVector;
  using xstream::InputData;
  using xstream::InputDataPtr;

  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();

  int num = 10;
  flow->SetConfig("config_file", "./config/filter.json");

  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->Init();
  std::cout << "========Init Finish==============" << std::endl;
  flow->SetCallback(
      std::bind(&Callback::OnCallbackBBoxFilterA, &callback, std::placeholders::_1),
      "BBoxFilter_1");

  float x1{0};   // BBox(框)的左上角横坐标
  float y1{20};  // BBox(框)的左上角纵坐标
  float x2{0};   // BBox(框)的右上角横坐标
  float y2{50};  // BBox(框)的右上角纵坐标

  std::shared_ptr<xstream::BBox> bbox = std::make_shared<xstream::BBox>(
    x1, y1, x2, y2);
  bbox->type_ = "BBox";
  std::shared_ptr<BaseDataVector> data = std::make_shared<BaseDataVector>();
  data->datas_.push_back(BaseDataPtr(bbox));
  data->name_ = "head_box";

  InputDataPtr inputdata = std::make_shared<InputData>();
  inputdata->datas_.push_back(BaseDataPtr(data));

  flow->AsyncPredict(inputdata);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  delete flow;

  return 0;
}
```

在main函数中，首先为整个workflow注册回调函数OnCallback，然后再为BBoxFilterA结点注册回调函数OnCallbackBBoxFilterA。

在BBoxFilterA的DoProcess函数中sleep 10ms，在BBoxFilterB的DoProcess函数中sleep 100ms。

预期结果，数据经过workflow的第一个结点BBoxFilterA处理(sleep 10ms)后，进入BBoxFilterB(继续sleep 100ms)，当数据还在BBoxFilterB中被处理时，会触发BBoxFilterA的回调函数OnCallbackBBoxFilterA，此时回调函数OnCallbackBBoxFilterA可以获取到BBoxFilterA处理后的数据(即，head_box_filter)，即，xstream支持直接获取workflow中间某个结点的计算结果。


## 第7步：编译

进入xstream/framework/目录下，执行如下命令，进行编译

```bash
cd xstream/framework/
mkdir build
cmake ..
make
```

## 第8步：运行

进入xstream/framework/目录下，执行如下命令，运行stage7_node_output

```bash
cd xstream/framework/tutorial/stage7_node_output
cp -rf ../../../tutorials/stage7_node_output/config/ .
./stage7_node_output
```

## 第9步：运行结果展示与查看

输出结果如下：

```bash
  1 BBoxFilterA::Init ./config/null
  2 BBoxFilterA::Init area_thres:100
  3 BBoxFilterB::Init ./config/null
  4 BBoxFilterB::Init area_thres:100
  5 ========Init Finish==============
  6 BBoxFilterA::DoProcess begin 1
  7 filter out: 0,20,0,50, score: 0
  8 BBoxFilterA::DoProcessing end 
  9 ========BBoxFilterA CallBack begin==============
 10 seq: 0
 11 output_type: 
 12 method_unique_name: BBoxFilter_1
 13 error_code: 0
 14 error_detail_: 
 15 datas_ size: 1
 16 data type_name : BaseDataVector head_box_filter
 17 pdata size: 0
 18 Output BBox head_box_filter:
 19 ========BBoxFilterA CallBack end==============
 20 BBoxFilterB::DoProcess begin1
 21 BBoxFilterB::DoProcessing end
 22 ========WorkFlow CallBack begin==============
 23 seq: 0
 24 output_type: __NODE_WHOLE_OUTPUT__
 25 method_unique_name: 
 26 error_code: 0
 27 error_detail_: 
 28 datas_ size: 1
 29 data type_name : BaseDataVector head_box_filter2
 30 pdata size: 0
 31 Output BBox head_box_filter2:
 32 ========WorkFlow CallBack end==============
 33 BBoxFilterB::Finalize
 34 BBoxFilterA::Finalize
```

通过上述结果可以看到回调函数的运行顺序，即：
BBoxFilterA运行结束后(line 8)，即会调用BBoxFilterA的回调函数 OnCallbackBBoxFilterA(line 9)，对BBoxFilterA的输出数据(head_box_filter)进行处理。

然后直到BBoxFilterB运行结束后(line 21)，再运行整个workflow的回调函数OnCallback(line 22)
