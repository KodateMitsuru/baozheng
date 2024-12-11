# 工导课抱枕

使用ros2搭建的驱动框架

# 编译

```bash
colcon build --symlink-install
```

# 运行

```bash
source install/setup.sh
ros2 launch startup.py
```

# topic

```shell
/servo_node/mode
/audio_node/play
```

```shell
ros2 topic pub /servo_node/mode std_msgs/msg/Int8 'data: 1' #启动舵机
ros2 topic pub /audio_node/play std_msgs/msg/String "data: 'assets/otto.wav'" -1
```

# config

## 位置

config/

# 还有什么

。。。。
