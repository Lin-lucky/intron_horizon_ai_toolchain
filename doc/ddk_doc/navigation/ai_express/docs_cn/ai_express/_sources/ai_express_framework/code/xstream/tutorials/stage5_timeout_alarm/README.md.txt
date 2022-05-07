## 超时告警说明
本节将介绍在workflow中设置单个Node、以及整个workflow的超时时间，当一帧数据未在规定时间完成单个Node或整个workflow的计算，程序会输出超时告警日志。

本节中我们基于下面的workflow来介绍超时告警功能，其中TimeoutAlarm节点实现检测框的透传功能，并在节点内随机休眠3~10秒。
```json
{
  "inputs": [
      "in_bbox"
  ],
  "outputs": [
      "out_bbox"
  ],
  "workflow": [
      {
          "method_type": "TimeoutAlarm",
          "unique_name": "TimeoutAlarm",
          "inputs": [
              "in_bbox"
          ],
          "outputs": [
              "out_bbox"
          ],
          "method_config_file": null
      }
  ]
}
```

### 定义XStream框架BBox数据结构
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
```

### 自定义Method
现在需要实现TimeoutAlarm Method的核心功能。XStream中定义了Method基础类，我们需要在此基础上实现自定义Method，并根据需要实现`DoProcess`这个核心处理函数。注意，为了演示`超时告警`功能，在DoProcess函数会随机休眠一段时间。
```c++
std::vector<BaseDataPtr> TimeoutAlarm::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr &param) {
  std::cout << "TimeoutAlarm::DoProcess" << std::endl;
  unsigned int seed = time(0);
  int cost = rand_r(&seed) % (MAX_VALUE - MIN_VALUE + 1) + MIN_VALUE;
  std::vector<BaseDataPtr> output;
  // one batch
  for (size_t j = 0; j < input.size(); j++) {
    output.push_back(std::make_shared<BaseDataVector>());
    if (input[j]->state_ == DataState::INVALID) {
      std::cout << "input slot " << j << " is invalid" << std::endl;
      continue;
    }
    auto in_rects = std::static_pointer_cast<BaseDataVector>(input[j]);
    auto out_rects = std::static_pointer_cast<BaseDataVector>(output[j]);
    for (auto &in_rect : in_rects->datas_) {
      // passthrough data
      out_rects->datas_.push_back(in_rect);
    }
  }
  std::cout << "sleep " << cost << " seconds" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(cost));
  return output;
}
```

### 指定workflow超时时间
workflow的超时时间是通过`SetConfig`接口来设置的。函数原型是`SetConfig(const std::string &key, const std::string &value)`，当设置workflow的超时时间时，参数key = `"time_monitor"`,参数value表示整数的字符串表示。

具体的使用方法如下：
```c++
xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();

int monitor_interval = 5;  // 超时时间, 单位秒
flow->SetConfig("time_monitor", std::to_string(monitor_interval));  // 设置workflow超时时间为5秒
```

运行workflow：
```c++
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();

  int monitor_interval = 5;
  flow->SetConfig("time_monitor", std::to_string(monitor_interval));  // set monitor_interval

  flow->SetConfig("config_file", workflow_config);
  flow->Init();

  std::cout << "***********************" << std::endl
            << "testing synchronous function" << std::endl
            << "***********************" << std::endl;
  for (int i = 0; i < 10; i++) {
    InputDataPtr inputdata(new InputData());
    BaseDataVector *data(new BaseDataVector);
    xstream::BBoxPtr bbox1 =
    std::make_shared<xstream::BBox>(0, 0, 10, 20, 0.1);
    xstream::BBoxPtr bbox2 = std::make_shared<xstream::BBox>(0, 0, 7, 7, 0.8);
    data->name_ = "in_bbox";   // corresponding the inputs in workflow
    data->datas_.push_back(bbox1);
    data->datas_.push_back(bbox2);
    inputdata->datas_.push_back(BaseDataPtr(data));

    auto out = flow->SyncPredict(inputdata);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
```

设置超时时间为5秒后，10帧数据经过上述workflow后的输出如下，可以看到当sleep时间超过5秒后，会有对应帧的告警日志。
```
TimeoutAlarm::DoProcess
sleep 5 seconds

TimeoutAlarm::DoProcess
sleep 10 seconds
(scheduler.cpp:541): source_id: 0, sequence_id: 1, not finished in 5 seconds!!

TimeoutAlarm::DoProcess
sleep 3 seconds

TimeoutAlarm::DoProcess
sleep 4 seconds

TimeoutAlarm::DoProcess
sleep 3 seconds

TimeoutAlarm::DoProcess
sleep 4 seconds

TimeoutAlarm::DoProcess
sleep 3 seconds

TimeoutAlarm::DoProcess
sleep 5 seconds

TimeoutAlarm::DoProcess
sleep 9 seconds
(scheduler.cpp:541): source_id: 0, sequence_id: 8, not finished in 5 seconds!!

TimeoutAlarm::DoProcess
sleep 5 seconds
```


### 指定Node超时时间
设置Node的超时时间与上述有所不同，是在workflow的配置文件中进行设置。注意到下面的workflow中，节点中的关键字`"timeout_duration"`，该字段表示节点的超时时间，单位是毫秒(ms)，若不设置该字段，则默认不开启Node的超时告警机制。这里的示例我们设置TimeoutAlarm超时时间是2000ms，即2s。

```json
{
    "inputs": [
        "in_bbox"
    ],
    "outputs": [
        "out_bbox"
    ],
    "workflow": [
        {
            "method_type": "TimeoutAlarm",
            "unique_name": "TimeoutAlarm",
            "timeout_duration": 2000,
            "inputs": [
                "in_bbox"
            ],
            "outputs": [
                "out_bbox"
            ],
            "method_config_file": null
        }
    ]
}
```

同样设置超时时间为5秒，10帧数据经过上述workflow后的输出如下，可以看到当Node内sleep时间超过2秒后，会有Node的告警日志。
```
TimeoutAlarm::DoProcess
sleep 4 seconds
(node.cpp:73): TimeoutAlarm: Process Time Out!!  Process Time Out!!!

TimeoutAlarm::DoProcess
sleep 3 seconds
(node.cpp:73): TimeoutAlarm: Process Time Out!!  Process Time Out!!!

TimeoutAlarm::DoProcess
sleep 4 seconds
(node.cpp:73): TimeoutAlarm: Process Time Out!!  Process Time Out!!!

TimeoutAlarm::DoProcess
sleep 3 seconds
(node.cpp:73): TimeoutAlarm: Process Time Out!!  Process Time Out!!!

TimeoutAlarm::DoProcess
sleep 5 seconds
(node.cpp:73): TimeoutAlarm: Process Time Out!!  Process Time Out!!!

TimeoutAlarm::DoProcess
sleep 9 seconds
(node.cpp:73): TimeoutAlarm: Process Time Out!!  Process Time Out!!!
(scheduler.cpp:541): source_id: 0, sequence_id: 5, not finished in 5 seconds!!

TimeoutAlarm::DoProcess
sleep 5 seconds
(node.cpp:73): TimeoutAlarm: Process Time Out!!  Process Time Out!!!

TimeoutAlarm::DoProcess
sleep 10 seconds
(node.cpp:73): TimeoutAlarm: Process Time Out!!  Process Time Out!!!
(scheduler.cpp:541): source_id: 0, sequence_id: 7, not finished in 5 seconds!!

TimeoutAlarm::DoProcess
sleep 3 seconds
(node.cpp:73): TimeoutAlarm: Process Time Out!!  Process Time Out!!!

TimeoutAlarm::DoProcess
sleep 4 seconds
(node.cpp:73): TimeoutAlarm: Process Time Out!!  Process Time Out!!!
```