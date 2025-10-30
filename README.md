# imu_ros2

## 简介
`imu_ros2` 是一个用于 ROS2 环境下 IMU（惯性测量单元）数据解析与发布的功能包，支持加速度、角速度、欧拉角等数据的接收、处理和发布。

## 功能特点
- **IMU数据解析**：通过 `ImuParser` 类高效解析原始 IMU 数据。
- **自定义消息类型**：定义了 `ImuData` 消息用于传输 IMU 数据。
- **节点实现**：实现了 IMU 节点，支持订阅原始数据并发布解析后的 IMU 数据。

## 安装方法
在 Ubuntu + ROS2 Humble 环境下，推荐如下安装流程：

1. 克隆仓库：
   ```
   git clone <仓库地址>
   cd imu_ros2
   ```

2. 安装依赖：
   ```
   rosdep install -i --from-path src --rosdistro humble -y
   ```

3. 编译功能包：
   ```
   colcon build
   ```

4. 加载环境变量：
   ```
   source install/setup.zsh
   ```


## 使用方法
### 启动 IMU 节点
```zsh
ros2 launch imu_ros2 test_imu_launch.py
```
节点启动后会自动处理串口 IMU 数据并发布到 `/imu/data`。

### 自定义串口号和波特率
你可以通过 launch 文件参数或命令行自定义串口端口和波特率，例如：

**命令行方式**：
```zsh
ros2 launch imu_ros2 test_imu_launch.py port:=/dev/ttyS1 baud:=9600
```

**修改 launch 文件参数**：
在 `launch/test_imu_launch.py` 的 `parameters` 字典中修改：
```python
parameters=[
   {'port': '/dev/ttyS1'},
   {'baud': 9600}
]
```

这样即可根据实际硬件配置灵活调整串口号和波特率。

## 主要文件结构
- `CMakeLists.txt`：CMake 构建配置文件
- `package.xml`：ROS2 包配置与依赖声明
- `README.md`：功能包说明文档
- `include/imu_ros2/imu_parser.hpp`：IMU数据解析类头文件
- `launch/test_imu_launch.py`：节点启动 launch 文件
- `src/imu_node.cpp`：IMU 节点实现
- `src/imu_parser.cpp`：IMU数据解析实现
- `msg/ImuData.msg`：自定义 IMU 数据消息类型

## 其他说明
如需自定义串口参数、数据发布频率或集成其他功能，请参考源码或 launch 文件参数配置。