# 多线程支持
## 1. 编译
进入stage2_multithread目录，执行如下步骤

```shell
  mkdir build
  cd build
  cmake .. -DINDEPENDENT_BUILD=ON
  make
  make install
```

## 2. 运行
将stage2_multithread目录下生产的output目录拷贝至XJ3系统，即可运行。运行使用如下命令
```shell
  ./stage2_multithread_example  multithread_workflow.json
```

## 3. 多线程示例
本节将介绍在workflow中启动多线程。

首先我们来实现一个计算输入数组`平均值`、以及`标准差`的workflow。workflow输入数据是一个float数组, 先后经过Average以及StandardDeviation两个节点运算，最终输出平均值average和标准差standard_deviation。注意到下面的workflow中，节点中的关键字`"thread_count"`，该字段表示节点的线程数，默认为1。这里我们设置两个节点的线程数都为2。
```json
{
    "inputs": ["input_array"],
    "outputs": ["average", "standard_deviation"],
    "workflow": [
      {
        "thread_count": 2,  // 节点的线程数
        "method_type": "Average",
        "unique_name": "average",
        "inputs": [
          "input_array"
        ],
        "outputs": [
          "average"
        ]
      },
      {
        "thread_count": 2,     // 节点的线程数
        "method_type": "StandardDeviation",
        "unique_name": "standard_deviation",
        "inputs": [
          "input_array"
        ],
        "outputs": [
          "standard_deviation"
        ]
      }
    ]
}
```

### 3.1. 定义XStream框架FloatValue数据结构
确定好workflow后，我们还需要定义workflow中用到的数据结构。由于XStream Framework中的数据都是基于BaseData，因此需要构建BaseData的派生数据结构。

```c++
// method/value.h
struct FloatValue : public BaseData {
  FloatValue() {}
  FloatValue(float x) {
    value_ = x;
  }

  float value_ = 0;
};
```
另外，框架中也内置了`BaseDataVector`的数据结构表示数组，本示例中workflow的输入数据是float数组，我们可以通过该结构表示FloatValue Array。
```c++
struct BaseDataVector : public BaseData {
  BaseDataVector();

  std::vector<BaseDataPtr> datas_;
};
```
### 3.2. 自定义Method以及属性MethodInfo
现在需要实现Average、StandardDeviation Method的核心功能。XStream中定义了Method基础类，我们需要在此基础上实现自定义Method，并根据需要实现`DoProcess`这个核心处理函数。同时对基类Method中的成员函数`GetMethodInfo()`进行重写，对实现的Method设置属性。

#### **设置MethodInfo**
首先来看一下Method属性(MethodInfo)的定义，包括is_thread_safe_、is_need_reorder_、is_src_ctx_dept_三种属性。本节主要介绍`is_thread_safe_`字段，该字段默认值为false。如果is_thread_safe_ = false，即非线程安全，表示一个Method的实例只能与一个thread绑定。若设置一个节点为多线程，则框架中会创建多个Method实例，每个实例跑在不同的线程；如果is_thread_safe_ = true，即线程安全，表示一个Method实例可以同时运行在多个线程上，即使设置一个节点为多线程，框架也仅创建一个Method实例。但需要注意，使用线程安全的属性，要求Method本身在业务上是可重入的。

```c++
struct MethodInfo {
  /// is thread safe
  bool is_thread_safe_ = false;
  /// is need reorder, the order of outputdata must be same as the inputdata
  bool is_need_reorder_ = false;
  /// is dependent on inputdata source
  bool is_src_ctx_dept_ = false;
};
```

本节中Average和StandardDeviation两个Method在本质上都是线程安全的，为了演示两种不同的属性特点，我们设置Average为非线程安全(is_thread_safe_ = false)，StandardDeviation为线程安全(is_thread_safe_ = true).

```c++
MethodInfo Average::GetMethodInfo() {
  MethodInfo method_info = MethodInfo();
  method_info.is_thread_safe_ = false;  // 设置该Method非线程安全
  return method_info;
}

MethodInfo StandardDeviation::GetMethodInfo() {
  MethodInfo method_info = MethodInfo();
  method_info.is_thread_safe_ = true;  // 设置该Method线程安全
  return method_info;
}
```

`DoProcess`的实现，计算数组的平均值、标准差；为了说明is_thread_safe_属性，这里我们在两个DoProcess函数中加入打印，分别打印对应Method实例的this对象、以及本次运行的thread id：
```c++
// Average::DoProcess
std::vector<BaseDataPtr> Average::DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) {
  std::cout << "Start Average::DoProcess..." << std::endl;
  std::cout << "Average Instance id: " << this << std::endl;
  std::cout << "Average Thread id: " << std::this_thread::get_id() << std::endl;

  std::vector<BaseDataPtr> output;
  // one batch
  for (size_t j = 0; j < input.size(); j++) {
    output.push_back(std::make_shared<FloatValue>());
    if (input[j]->state_ == DataState::INVALID) {
      std::cout << "input slot " << j << " is invalid" << std::endl;
      continue;
    }
    // 计算输入数组的平均值
    auto in_datas = std::static_pointer_cast<BaseDataVector>(input[j]);
    auto out_data = std::static_pointer_cast<FloatValue>(output[j]);
    float sum = 0;
    int count = 0;
    for (auto &in_data : in_datas->datas_) {
      auto data = std::static_pointer_cast<FloatValue>(in_data);
      sum += data->value_;
      count++;
    }
    out_data->value_ = sum / count;
  }
  return output;
}

// StandardDeviation::DoProcess
std::vector<BaseDataPtr> StandardDeviation::DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) {
  std::cout << "Start StandardDeviation::DoProcess..." << std::endl;
  std::cout << "StandardDeviation Instance id: " << this << std::endl;
  std::cout << "StandardDeviation Thread id: " << std::this_thread::get_id()
            << std::endl;

  std::vector<BaseDataPtr> output;
  // one batch
  for (size_t j = 0; j < input.size(); j++) {
    output.push_back(std::make_shared<FloatValue>());
    if (input[j]->state_ == DataState::INVALID) {
      std::cout << "input slot " << j << " is invalid" << std::endl;
      continue;
    }
    // 计算输入数组的标准差
    auto in_datas = std::static_pointer_cast<BaseDataVector>(input[j]);
    auto out_data = std::static_pointer_cast<FloatValue>(output[j]);
    float average = 0, standard_deviation = 0;
    int count = 0;
    // 平均值
    for (auto &in_data : in_datas->datas_) {
      auto data = std::static_pointer_cast<FloatValue>(in_data);
      average += data->value_;
      count++;
    }
    average /= count;
    for (auto &in_data : in_datas->datas_) {
      auto data = std::static_pointer_cast<FloatValue>(in_data);
      standard_deviation += (data->value_ - average) * (data->value_ - average);
    }
    standard_deviation /= count;
    standard_deviation = sqrt(standard_deviation);
    out_data->value_ = standard_deviation;
  }
  return output;
}
```

### 3.3. 运行workflow
首先需要创建XStream SDK并初始化，SDK是运行workflow的对外接口对象。通过`SetConfig()`接口配置workflow json文件，然后调用`Init()`进行初始化。由于设置了多线程，所以我们这里需要运行在异步模式下，需要通过`SetCallback()`配置输出结果的回调函数。

```c++
  // Create xstream sdk object
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  // Set config_file
  flow->SetConfig("config_file", config);
  // Init
  flow->Init();
  // Set CallBack Func For Async Mode
  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
```

创建输入数组，随机生成一组大小为10的float数组，并封装成框架数据InputData；并将输入数据送入框架进行两次异步运算：
```c++
  for (int i = 0; i < 2; i++) {
    // Prepare input data
    InputDataPtr inputdata(new InputData());
    BaseDataVector *data(new BaseDataVector);
    for (int i = 0; i < 10; i++) {
      std::random_device rd;
      std::default_random_engine eng(rd());
      std::uniform_real_distribution<float> distr(0, 10);

      FloatValuePtr float_value = std::make_shared<FloatValue>(distr(eng));
      data->datas_.push_back(float_value);
    }
    data->name_ = "input_array";   // corresponding the inputs in workflow
    inputdata->datas_.push_back(BaseDataPtr(data));

    flow->AsyncPredict(inputdata);
  }
```

来看下经过上述workflow后的结果：
```
Average::Init
Average::Init
StandardDeviation::Init

Start Average::DoProcess...
Average Instance id: 0xaf03f10
Average Thread id: 548091961792

Start Average::DoProcess...
Average Instance id: 0xaf04190
Average Thread id: 548083569088

Start StandardDeviation::DoProcess...
StandardDeviation Instance id: 0xaf053c0
StandardDeviation Thread id: 548075176384

Start StandardDeviation::DoProcess...
StandardDeviation Instance id: 0xaf053c0
StandardDeviation Thread id: 548066783680
```

可以看到虽然workflow中设置了两个节点的线程都为2，但Average非线程安全，实际上注册了两个实例(0xaf03f10,0xaf04190)，而StandardDeviation线程安全，实际注册了一个实例(0xaf053c0)。
