# 工导课抱枕

使用ros2搭建的驱动框架

## 编译

```bash
colcon build --symlink-install
```

## 运行

```bash
source install/setup.sh
ros2 launch startup.py
```

## topic

```shell
/servo_node/mode
/audio_node/play
/audio_node/stop
```

### smaple command

```shell
ros2 topic pub /servo_node/mode std_msgs/msg/Int8 'data: 1' #启动舵机
ros2 topic pub /audio_node/play std_msgs/msg/String "data: 'assets/music/igs.mp3'" -1 #播放音频
ros2 topic pub /audio_node/stop std_msgs/msg/Bool "data: true" -1 #停止音频
```

## config

```shell
config
└── servo_node.yaml
```

## assets

存储音频文件
使用lfs

```shell
assets
├── music
│   ├── igs.mp3
│   └── otto_bxsn.mp3
├── tokens
│   ├── _.wav
|   ├── ...
│   └── zuo.wav
└── ysddTokens
    ├── bbd.wav
    ├── ...
    └── ydglm.wav
```

### 当前音频文件

```shell
igs.mp3 -- i got smoke
otto_bxsn.mp3 -- 波西唢呐狂想曲
```

## known_bugs

目前舵机控制需要root

## 还有什么

。。。。
