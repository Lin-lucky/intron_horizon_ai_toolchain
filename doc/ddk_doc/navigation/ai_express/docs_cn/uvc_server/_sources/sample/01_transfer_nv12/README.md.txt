# 简介
本示例使用uvc_server组件，实现uvc传输nv12图像，PC侧通过播放器播放图像画面。

# 代码编译运行
## 代码目录
.
├── build.sh
├── CMakeLists.txt
├── data
│   └── 1080p.nv12
├── include
│   └── transfer_nv12.h
├── main.cc
├── README.md
├── run.sh
└── src
    └── transfer_nv12.cc
- build.sh：编译部署脚本
- data：示例运行nv12图片数据
- run.sh：示例运行脚本
## 编译
1、安装统一发版中的host pakcage
2、进入sample/01_transfer_nv12目录，执行如下脚本完成编译和部署
```shell
bash build.sh
```

## 运行
1、把deploy拷贝到目标开发板上
2、进入开发板deploy目录，执行run.sh，默认是ut模式（运行10s后自动退出）
```shell
sh run.sh
```
3、运行normal模式
```shell
sh run.sh normal
```

# 代码说明
示例基于uvc_server的接口实现了TestTransfer类，实现uvc传输的功能：
```c++
class TestTransfer : public UvcEventCallback {
 public:
  TestTransfer();
  ~TestTransfer() = default;

  int Init();
  int DeInit();
  int Start();
  int Stop();

  int SendNv12Data(void* nv12_data, int len, int nv12_width, int nv12_height);
  inline bool IsUvcStreamOn() {
    std::lock_guard<std::mutex> lg(mutex_);
    return uvc_stream_on_ > 0;
  }

  inline void SetUvcStreamOn(int on) {
    std::lock_guard<std::mutex> lg(mutex_);
    uvc_stream_on_ = on;
  }
  inline void SetNv12IsOn(bool is_on) {
    std::lock_guard<std::mutex> lg(mutex_);
    nv12_is_on_ = is_on;
  }

  inline bool IsNv12On() {
    std::lock_guard<std::mutex> lg(mutex_);
    return nv12_is_on_;
  }

 private:
  void OnUvcEvent(UvcEvent event_type, void* data, int data_len) override;

 private:
  int uvc_stream_on_;
  bool nv12_is_on_;
  bool is_inited_;
  std::mutex mutex_;
  int request_width_;
  int request_height_;
  std::shared_ptr<UvcServer> uvc_server_;
};
```
TestTransfer类封装了如下接口：
- OnUvcEvent：实现了接口`UvcEventCallback`，用于接收处理uvc视频开启/关闭以及设备add/remove消息
- SendNv12Data：函数内部调用uvc_server的接口发送数据，仅支持nv12数据传输
- Init：初始化uvc_server组件，并启动uvc_server服务
- DeInit：反初始化
- IsUvcStreamOn：获取Uvc Stream使能标记
- SetUvcStreamOn：设置Uvc Stream使能标记
- IsNv12On：uvc支持nv12传输使能标记
- SetNv12IsOn：设置uvc支持nv12传输使能标记

main.cc代码如下：
```c++
int main(int argc, char **argv) {
  if (argc < 4) {
    std::cout
        << "[Usage]: uvc_sample nv12_path nv12_height nv12_width [ut normal]"
        << std::endl;
    return -1;
  }
  std::string run_mode = "ut";
  if (argc > 4) {
    run_mode.assign(argv[4]);
    std::cout << "run_mode:" << run_mode << std::endl;
    if (run_mode != "ut" && run_mode != "normal") {
      std::cout << "[ERROR] not support mode: " << run_mode << std::endl;
      return 0;
    }
  }
  // 1. read nv12 image data
  std::ifstream ifs(argv[1], std::ios::in | std::ios::binary);
  if (!ifs) {
    std::cout << "[ERROR] Open nv12 file: " << argv[1] << " failed"
              << std::endl;
    return -1;
  }
  ifs.seekg(0, std::ios::end);
  int img_length = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  char *img_nv12 = new char[sizeof(char) * img_length];
  ifs.read(img_nv12, img_length);
  ifs.close();

  int height = atoi(argv[2]);
  int width = atoi(argv[3]);
  // 2. Create TestTransfer Instance
  std::shared_ptr<TestTransfer> uvc_sender = std::make_shared<TestTransfer>();
  // 3. Init TestTransfer
  auto ret = uvc_sender->Init();
  if (ret != 0) {
    std::cout << "[ERROR] uvc server init failed" << std::endl;
    return -1;
  }
  // 4. Start TestTransfer
  ret = uvc_sender->Start();
  if (ret != 0) {
    std::cout << "[ERROR] rtsp server start failed" << std::endl;
    return -1;
  }
  // 5. Wait some time
  if (run_mode == "ut") {
    int count = 10000 / 40;
    while (!exit_ && count-- > 0) {
      uvc_sender->SendNv12Data(img_nv12, img_length, width, height);
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
  } else {
    while (!exit_) {
      uvc_sender->SendNv12Data(img_nv12, img_length, width, height);
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
  }
  // 6. Stop&DeInit
  uvc_sender->Stop();
  uvc_sender->DeInit();

  return 0;
}
```
1. 读取预先准备的nv12图像数据及图像宽高，用来回复uvc协议的每一帧数据请求
2. 创建TestTransfer对象
3. 初始化uvc
4. 调用TestTransfer的Start接口
5. 调用TestTransfer的SendNv12Data接口发送数据到uvc协议，根据运行模式循环若干时间
6. uvc反初始化&资源清理