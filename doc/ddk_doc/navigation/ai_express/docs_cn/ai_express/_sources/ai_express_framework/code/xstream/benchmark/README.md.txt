## XStream Framework调度性能评测
本节主要对XStream Framework调度性能进行评测说明。

我们将基于下面的workflow进行调度评测，具体如下。workflow输入数据是input, 输入数据先后经过两个PassThrough节点的运算，最终输出数据是output。其中PassThrough实现了输入输出数据的透传。
```json
{
    "inputs": ["input"],
    "outputs": ["output"],
    "workflow": [
      {
        "method_type": "PassThrough",
        "unique_name": "pass_through_1",
        "inputs": [
          "input"
        ],
        "outputs": [
          "data_tmp"
        ],
        "method_config_file": "null"
      },
      {
        "method_type": "PassThrough",
        "unique_name": "pass_through_2",
        "inputs": [
          "data_tmp"
        ],
        "outputs": [
          "output"
        ],
        "method_config_file": "null"
      }
    ]
}
```

### 定义PassThrough Method
现在需要定义PassThrough Method并实现其核心功能。
```c++
std::vector<std::vector<BaseDataPtr>> PassThrough::DoProcess(
    const std::vector<std::vector<BaseDataPtr>> &input,
    const std::vector<xstream::InputParamPtr> &param) override {
  return input;
}
```

### 帧率统计
程序中通过异步接口`AsyncPredict()`将输入数据送入框架进行运算，通过回调函数进行帧率的统计，每调用一次回调函数，说明有一帧数据处理完成。

在示例程序中我们每隔0.5ms添加一次任务，在评测时用户可以根据需要修改此时间间隔。

```c++
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", config);
  flow->SetCallback(std::bind(&FrameFPS, std::placeholders::_1));
  flow->Init();

  while (!exit_) {
    InputDataPtr inputdata(new InputData());
    BaseDataPtr data(new BaseData());
    data->name_ = "input";   // corresponding the inputs in workflow
    inputdata->datas_.push_back(data);

    // async mode
    flow->AsyncPredict(inputdata);
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
```

帧率统计函数定义如下：
```c++
void FrameFPS(xstream::OutputDataPtr output) {
  static auto last_time = std::chrono::system_clock::now();
  static int fps = 0;
  static int frameCount = 0;

  frameCount++;

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now() - last_time);
  if (duration.count() > 1000) {
    fps = frameCount;
    frameCount = 0;
    last_time = std::chrono::system_clock::now();
    std::cout << "fps = " << fps << std::endl;
  }
}
```

### 调度性能
运行benchmark_main程序后，会持续输出帧率大小，用户可以通过`ctrl+C`来终止。
```
PassThrough Init
PassThrough Init
fps = 1693
fps = 1691
fps = 1690
fps = 1690
fps = 1689
fps = 1689
fps = 1690
fps = 1691
fps = 1691
fps = 1691
fps = 1690
fps = 1691
fps = 1690
fps = 1690
fps = 1690
fps = 1690
^Crecv signal 2, stop
PassThrough Finalize
PassThrough Finalize
```

#### xstream v.s. hobotsdk
构建包含两个节点的workflow，节点内仅透传无其他操作。分别以0.5ms、1ms、2ms、5ms、10ms、20ms的间隔输入数据，测试xstream和hobotsdk调度性能，结果如下。可以发现当输入FPS处于100时，xstream和hobotsdk框架调度的耗时基本可以忽略。

<table>
    <tr>
        <th> </th><th colspan=2>0.5ms</th><th> </th><th colspan=2>1ms</th><th></th><th colspan="2">2ms</th><th></th><th colspan="2">5ms</th><th></th><th colspan="2">10ms</th><th></th><th colspan="2">20ms</th>
    </tr>
    <tr>
        <td></td><td>xstream</td><td>hobotsdk</td><td> </td><td>xstream</td><td>hobotsdk</td><td></td><td>xstream</td><td>hobotsdk</td><td></td><td>xstream</td><td>hobotsdk</td><td></td><td>xstream</td><td>hobotsdk</td><td></td><td>xstream</td><td>hobotsdk</td>
    </tr>
    <tr>
        <td>FPS</td><td>1690.35</td><td>1728.7</td><td></td><td>909.4</td><td>901.7</td><td></td><td>467.3</td><td>471.4</td><td></td><td>197.0</td><td>197.4</td><td></td><td>100.0</td><td>99.3</td><td></td><td>50.0</td><td>49.7</td>
    </tr>
    <tr>
        <td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td>
    </tr>
    <tr>
        <td></td><td>-2.2%</td><td></td><td></td><td>0.84%</td><td></td><td></td><td>-0.87%</td><td></td><td></td><td>-0.20%</td><td></td><td></td><td>0.70%</td><td></td><td></td><td>0.60%</td><td></td>
    </tr>
</table>
