
# 1. 系统sensor库
mipi camera的运行需要依赖系统软件驱动库，有些sensor库并没有放入到系统镜像中，因此需要用户手动更新，具体更新位置如下：

- f37 sensor：
```shell
mount -o remount,rw /               # 重新mount，获取根目录的权限
cp f37/libf37.so /lib/sensorlib/    # 驱动库
cp f37/libjxf37_linear.so /etc/cam/  # isp turning库
```
- imx586 sensor:
```shell
cp imx586/libimx586.so /lib/sensorlib/    # 驱动库
cp imx586/libimx586_linear.so /etc/cam/  # isp turning库
```
# 2. video source内部sensor插件库
video source对于集成一款新的sensor，有两种方式：
- 枚举方式：通过下面配置的sensor_id来使用
- 插件库方式：通过打开下面的sensor_plugin_en字段来使用。默认集成了f37和imx586的插件库，用户可以根据需要，集成自己的sensor插件库

当用户不想修改video source源代码时，可以使用sensor插件库的方式集成进video source

- 例如libf37_plugin.so这插件库的使用方式如下：
  - 在vin的配置文件中，设置sensor_plugin_en为1
  - 设置sensor_plugin_path字段，这个为libf37_plugin.so的存放位置
  - 设置设置sensor_plugin_name字段，这个是寻找的插件库的名字
  - sensor_plugin_type字段，这个是该sensor启动的模式

```json
{
.....
  "sensor": {
    "sensor_id": 5,                                # sensor的枚举号
    "sensor_plugin_en": 1,                         # 是否是插件形式导入sensor参数配置，默认关闭状态
    "sensor_plugin_path": "./lib/",                # sensor插件库的存放目录
    "sensor_plugin_name": "f37_plugin",         # sensor插件库的名称
    "sensor_plugin_type": "linear",                # sensor插件库的类型，支持linear、dol2、dol3模块，根据sensor的配置来决定
.....
  }
....
```
设置完上面的操作后，video source将会从插件库中读取保存的参数，然后初始化sensor，此时camera就会正常跑起来了。
