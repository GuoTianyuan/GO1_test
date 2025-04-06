#!/bin/bash

# ---------- 配置区 ----------
MAIN_WS="$HOME/GO1_test_ws"          # 工作空间路径
DEFAULT_GAIT="trot"                  # 默认步态参数
GAZEBO_LAUNCH="legged_unitree_description empty_world.launch"
CONTROLLER_LAUNCH="legged_controllers load_controller.launch cheater:=false"
VISION_SCRIPT_DIR="src/target_tracking/scripts/Yolov5-Deepsort-main"
VISION_SCRIPT="demo_gazebo.py"
# ---------------------------

# 检查工作空间
check_workspace() {
    [ ! -d "$MAIN_WS" ] && echo "错误：工作空间 $MAIN_WS 不存在" && exit 1
    return 0
}

# 启动Gazebo仿真环境
launch_gazebo() {
    gnome-terminal --title="[1] Gazebo仿真" -- bash -c \
    "cd $MAIN_WS && source devel/setup.bash && \
    roslaunch $GAZEBO_LAUNCH; exec bash"
}

# 启动运动控制器（带自动输入）
launch_controller() {
    gnome-terminal --title="[2] 运动控制器" -- bash -c \
    "cd $MAIN_WS && source devel/setup.bash && \
    echo '正在加载控制器... (自动输入步态: $DEFAULT_GAIT)' && \
    roslaunch $CONTROLLER_LAUNCH && $DEFAULT_GAIT ; exec bash"
}

# 启动视觉跟踪程序
launch_vision() {
    gnome-terminal --title="[3] 视觉跟踪" -- bash -c \
    "cd $MAIN_WS && source devel/setup.bash && cd $VISION_SCRIPT_DIR && \
    echo '启动视觉跟踪系统...' && \
    python $VISION_SCRIPT; exec bash"
}

# 主执行流程
main() {
    check_workspace
    
    echo "正在启动 Gazebo 仿真..."
    launch_gazebo
    sleep 20  
    
    echo "正在启动 运动控制器..."
    launch_controller
    sleep 20
    
    echo "正在启动 视觉跟踪系统..."
    launch_vision
    
    echo "所有终端已独立启动！"
    echo "──────────────────────────────────"
    echo "终端布局建议："
    echo "1号终端（左侧）：Gazebo仿真"
    echo "2号终端（中上）：运动控制器"
    echo "3号终端（右侧）：视觉跟踪"
}

main
