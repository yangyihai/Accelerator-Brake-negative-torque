#!/bin/bash

# 整合脚本: 集成油门、刹车和负扭矩测试
source /ssd/loader_ws/src/loader/bash/loader_terminal.bash

# 清理函数 - 用于终止进程并等待
cleanup() {
    local pid=$1
    if [ -n "$pid" ]; then
        kill "$pid" 2>/dev/null && wait "$pid" 2>/dev/null
    fi
} 

# 清理所有相关进程
cleanup_all() {
    [ -n "$PWM_PID" ]   && kill "$PWM_PID"   2>/dev/null && wait "$PWM_PID"   2>/dev/null
    [ -n "$BRAKE_PID" ] && kill "$BRAKE_PID" 2>/dev/null && wait "$BRAKE_PID"   2>/dev/null
    pkill -f "rostopic pub /pwm loader_common/State" 2>/dev/null || true
    pkill -f "rosbag record" 2>/dev/null || true
}

#########################################
# 功能1: 刹车数据收集
#########################################
brake_test() {
    # 让用户输入油门和起始刹车值
    read -p "Enter constant throttle value (0-1000): " throttle_value
    read -p "Enter initial brake value (0-1000): " initial_brake

    # 验证输入范围
    if ! [[ "$throttle_value" =~ ^[0-9]+$ ]] || [ "$throttle_value" -gt 1000 ]; then
        echo "Error: Throttle must be a number between 0 and 1000"
        return 1
    fi
    
    if ! [[ "$initial_brake" =~ ^[0-9]+$ ]] || [ "$initial_brake" -gt 1000 ]; then
        echo "Error: Brake must be a number between 0 and 1000"
        return 1
    fi

    # 创建日志目录
    local LOG_DIR="brake_logs_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$LOG_DIR"

    # 按照刹车值增长进行测试
    for brake in $(seq $initial_brake 50 1000); do
        # 前进档测试
        brake_gear_test $throttle_value $brake "FORWARD" "1" "$LOG_DIR"
        
        # 后退档测试
        brake_gear_test $throttle_value $brake "REVERSE" "-1" "$LOG_DIR"
    done

    echo "Brake tests completed. Logs saved in ${LOG_DIR}"
}

brake_gear_test() {
    local throttle=$1
    local brake=$2
    local gear_type=$3
    local gear_value=$4
    local log_dir=$5
    
    echo "==== Throttle ${throttle}, Brake ${brake}: ${gear_type} ===="
    
    # 清理上一轮的进程
    cleanup_all

    # 开始录制rosbag
    BAG_NAME="${log_dir}/throttle${throttle}_brake${brake}_${gear_type,,}_$(date +%Y%m%d_%H%M%S)"
    echo "Recording rosbag: ${BAG_NAME}.bag"
    nohup rosbag record -O "${BAG_NAME}" /WorldPose /plc_states /pwm /tf /robot_odometer_raw >/dev/null 2>&1 &
    RECORD_PID=$!
    sleep 1

    # 切换档位并应用油门
    switch g ${gear_value}
    sleep 2
    echo "pwm throttle ${throttle}"
    pwm throttle ${throttle} &
    PWM_PID=$!
    sleep 8
    cleanup "$PWM_PID"
    pkill -f "rostopic pub /pwm loader_common/State" 2>/dev/null || true

    # 应用初始刹车值
    echo "pwm brake ${brake}"
    pwm brake ${brake} &
    BRAKE_PID=$!
    sleep 7
    cleanup "$BRAKE_PID"

    # 停止录制
    kill -INT "$RECORD_PID" 2>/dev/null && wait "$RECORD_PID"
    echo "Stopped recording ${BAG_NAME}.bag"

    # 应用最大刹车
    echo "pwm brake 1000"
    pwm brake 1000 &
    BRAKE_PID=$!
    sleep 3
    cleanup "$BRAKE_PID"
    pkill -f "rostopic pub /pwm loader_common/State" 2>/dev/null || true
}

#########################################
# 功能2: 油门数据收集
#########################################
throttle_test() {
    # 让用户输入起始油门值
    read -p "Enter initial throttle (0-1000, default: 0): " initial_throttle
    initial_throttle=${initial_throttle:-0}

    # 验证输入值范围
    if ! [[ "$initial_throttle" =~ ^[0-9]+$ ]] || [ "$initial_throttle" -gt 1000 ]; then
        echo "Error: Initial throttle must be a number between 0 and 1000"
        return 1
    fi

    # 创建日志目录
    LOG_DIR="throttle_logs_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$LOG_DIR"

    # 按照油门值增长进行测试
    for throttle in $(seq "$initial_throttle" 50 1000); do
        # 前进测试
        run_throttle_test "$throttle" "FORWARD" 1 "$LOG_DIR"
        
        # 后退测试
        run_throttle_test "$throttle" "REVERSE" -1 "$LOG_DIR"
    done

    echo "Throttle tests completed. Logs saved in ${LOG_DIR}"
}

run_throttle_test() {
    local throttle=$1
    local direction=$2
    local gear=$3
    local log_dir=$4
    
    echo "==== Throttle ${throttle}: ${direction} ===="
    
    # 清理进程
    cleanup_all

    # 开始录制rosbag
    local BAG_NAME="${log_dir}/throttle_${direction}_${throttle}_$(date +%Y%m%d_%H%M%S)"
    echo "Recording rosbag: ${BAG_NAME}.bag"
    nohup rosbag record -O "${BAG_NAME}" /WorldPose /plc_states /pwm /tf /robot_odometer_raw >/dev/null 2>&1 &
    local RECORD_PID=$!
    sleep 1

    # 切换档位并应用油门
    switch g "$gear"
    echo "pwm throttle ${throttle}"
    pwm throttle "${throttle}" &
    PWM_PID=$!
    sleep 8
    cleanup "$PWM_PID"
    pkill -f "rostopic pub /pwm loader_common/State" 2>/dev/null || true

    # 停止录制
    kill -INT "$RECORD_PID" 2>/dev/null && wait "$RECORD_PID"
    echo "Stopped recording ${BAG_NAME}.bag"

    # 应用刹车
    echo "pwm brake 1000"
    pwm brake 1000 &
    BRAKE_PID=$!
    sleep 3
    cleanup "$BRAKE_PID"
    pkill -f "rostopic pub /pwm loader_common/State" 2>/dev/null || true
}

#########################################
# 功能3: 负扭矩数据收集
#########################################
negative_torque_test() {
    # 让用户输入油门和起始负扭矩值
    read -p "Enter constant throttle value (0-1000): " throttle_value
    read -p "Enter initial brake value (-1300 to -100): " initial_brake

    # 验证输入范围
    if ! [[ "$throttle_value" =~ ^[0-9]+$ ]] || [ "$throttle_value" -gt 1000 ]; then
        echo "Error: Throttle must be a number between 0 and 1000"
        return 1
    fi
    
    if ! [[ "$initial_brake" =~ ^-?[0-9]+$ ]] || [ "$initial_brake" -lt -1300 ] || [ "$initial_brake" -gt -100 ]; then
        echo "Error: Brake must be a number between -1300 and -100"
        return 1
    fi

    # 创建日志目录
    local LOG_DIR="torque_logs_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$LOG_DIR"

    # 按照负扭矩值减少进行测试
    for brake in $(seq $initial_brake -50 -1300); do
        # 前进档测试
        torque_gear_test $throttle_value $brake "FORWARD" "1" "$LOG_DIR"
        
        # 后退档测试
        torque_gear_test $throttle_value $brake "REVERSE" "-1" "$LOG_DIR"
    done

    echo "Negative torque tests completed. Logs saved in ${LOG_DIR}"
}

torque_gear_test() {
    local throttle=$1
    local brake=$2
    local gear_type=$3
    local gear_value=$4
    local log_dir=$5
    
    echo "==== Starting cycle for brake=${brake}, ${gear_type} ===="

    # 清理进程
    cleanup_all

    # 开始录制
    BAG_NAME="${log_dir}/torque_${gear_type,,}_${brake}_$(date +%Y%m%d_%H%M%S)"
    echo "Recording rosbag: ${BAG_NAME}.bag"
    nohup rosbag record -O "${BAG_NAME}" /WorldPose /plc_states /pwm /tf /robot_odometer_raw >/dev/null 2>&1 &
    RECORD_PID=$!
    sleep 1

    # 切换档位并应用初始油门
    switch g ${gear_value}
    sleep 2
    echo "pwm throttle ${throttle}"
    pwm throttle "${throttle}" &
    PWM_PID=$!
    sleep 8
    cleanup "$PWM_PID"
    pkill -f "rostopic pub /pwm loader_common/State" 2>/dev/null || true

    # 应用负扭矩油门
    echo "pwm throttle ${brake}"
    pwm throttle "${brake}" &
    BRAKE_PID=$!
    sleep 7
    
    # 停止录制
    kill -INT "$RECORD_PID" 2>/dev/null && wait "$RECORD_PID"
    echo "Stopped recording ${BAG_NAME}.bag"
    
    # 应用最大刹车
    echo "pwm brake 1000"
    pwm brake 1000 &
    BRAKE_PID=$!
    sleep 3
    
    # 清理
    cleanup "$BRAKE_PID"
    pkill -f "rostopic pub /pwm loader_common/State" 2>/dev/null || true
}

#########################################
# 主菜单
#########################################
show_menu() {
    clear
    echo "======================================="
    echo "    装载机测试脚本集成菜单    "
    echo "======================================="
    echo "1. 刹车数据收集测试"
    echo "2. 油门数据收集测试" 
    echo "3. 负扭矩数据收集测试"
    echo "0. 退出"
    echo "======================================="
    echo -n "请选择功能 [0-3]: "
}

# 检查必要的命令是否存在
check_requirements() {
    command -v rosbag >/dev/null 2>&1 || { echo "错误: 需要安装rosbag命令"; return 1; }
    command -v switch >/dev/null 2>&1 || { echo "错误: 需要安装switch命令"; return 1; }
    command -v pwm >/dev/null 2>&1 || { echo "错误: 需要安装pwm命令"; return 1; }
    return 0
}

# 主程序
main() {
    if ! check_requirements; then
        exit 1
    fi
    
    while true; do
        show_menu
        read choice
        
        case $choice in
            1)
                echo "执行刹车数据收集测试..."
                brake_test
                read -p "按Enter键继续..."
                ;;
            2)
                echo "执行油门数据收集测试..."
                throttle_test
                read -p "按Enter键继续..."
                ;;
            3)
                echo "执行负扭矩数据收集测试..."
                negative_torque_test
                read -p "按Enter键继续..."
                ;;
            0)
                echo "退出程序"
                exit 0
                ;;
            *)
                echo "无效选择，请重新输入"
                read -p "按Enter键继续..."
                ;;
        esac
    done
}

# 执行主程序
main