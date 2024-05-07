#!/bin/bash

# 查找指定名称的进程ID
pid=$(pgrep -f "python simulator.py")

# 检查是否找到了进程ID
if [ -z "$pid" ]; then
  echo "未找到进程：python simulator.py"
else
  # 使用kill命令关闭进程
  kill "$pid"
  echo "进程已关闭"
fi

docker restart carla-chenpansong-simulator-1
