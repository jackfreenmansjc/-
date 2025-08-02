#!/bin/bash

# 定义要查找的端口
PORT=6666

# 查找占用该端口的进程ID
PID=$(lsof -t -i:$PORT)

# 检查是否找到了进程ID
if [ -n "$PID" ]; then
    echo "Found process with PID: $PID occupying port $PORT. Killing it..."
    kill -9 $PID
    echo "Process $PID has been killed."
else
    echo "No process found occupying port $PORT."
fi
