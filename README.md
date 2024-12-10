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
```

```shell
ros2 topic pub /servo_node/mode std_msgs::msg::Int8 {'1'} //启动
```

# config

## 位置

config/

# 还有什么

。。。。
