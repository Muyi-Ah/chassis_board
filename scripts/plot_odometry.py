import serial
import serial.tools.list_ports
import time
import math
import sys
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

# --- 配置参数 ---
BAUDRATE = 921600  # 请根据你的单片机实际波特率修改
DT = 0.001         # 采样周期 1ms (1000Hz)
SCALE_FACTOR = 0.001 # 单片机发上来乘了1000，这里要乘以0.001还原

# 机器人物理参数 (用于绘制机器人本体，单位：米)
ROBOT_WIDTH = 0.28  
ROBOT_LENGTH = 0.34

# --- 全局变量 ---
# 里程计位姿
pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0

# 累计里程 (仅用于显示，不再做累加)
total_dist_x = 0.0  
total_dist_y = 0.0  
total_angle = 0.0

# 绘图历史数据
history_x = [0.0]
history_y = [0.0]
history_time = [0.0]

# 接收线程控制
is_running = True
serial_port = None
data_lock = threading.Lock()

def select_serial_port():
    """交互式选择串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到任何串口设备！请检查连接。")
        sys.exit(1)
        
    print("=== 发现以下可用串口 ===")
    for i, port in enumerate(ports):
        print(f"[{i}] {port.device} - {port.description}")
        
    if len(ports) == 1:
        print(f"自动选择唯一的串口: {ports[0].device}")
        return ports[0].device
        
    while True:
        try:
            choice = input(f"请输入要连接的串口序号 (0-{len(ports)-1}): ").strip()
            if not choice:
                continue
            index = int(choice)
            if 0 <= index < len(ports):
                return ports[index].device
            else:
                print("序号超出范围，请重新输入。")
        except ValueError:
            print("请输入有效的数字序号。")

def serial_receive_thread():
    """后台接收串口数据并进行航位推算"""
    global pose_x, pose_y, pose_theta, is_running
    global total_dist_x, total_dist_y, total_angle
    
    try:
        buffer = b""
        while is_running:
            if serial_port and serial_port.in_waiting > 0:
                buffer += serial_port.read(serial_port.in_waiting)
                
                # 按行分割
                while b'\r\n' in buffer:
                    line, buffer = buffer.split(b'\r\n', 1)
                    try:
                        line_str = line.decode('utf-8').strip()
                        if not line_str:
                            continue
                            
                        # 解析数据: x, y, theta
                        parts = line_str.split(',')
                        if len(parts) >= 3:
                            with data_lock:
                                # 直接使用单片机发上来的绝对位置
                                pose_x = float(parts[0]) * SCALE_FACTOR
                                pose_y = float(parts[1]) * SCALE_FACTOR
                                pose_theta = float(parts[2]) * SCALE_FACTOR
                                
                    except Exception as e:
                        # 打印异常信息，避免静默失败
                        print(f"Data parse error: {e}, raw line: {line_str}")
                        pass
            else:
                time.sleep(0.001)
    except Exception as e:
        print(f"串口接收异常: {e}")
        is_running = False

def update_plot(frame):
    """动画更新函数"""
    global history_x, history_y, history_time
    global total_dist_x, total_dist_y, total_angle
    
    with data_lock:
        current_x = pose_x
        current_y = pose_y
        current_theta = pose_theta
        t_dist_x = total_dist_x
        t_dist_y = total_dist_y
        t_angle_deg = math.degrees(total_angle)
        
    # 记录历史轨迹 (为了性能，最多保留最新的 5000 个点)
    history_x.append(current_x)
    history_y.append(current_y)
    if len(history_x) > 5000:
        history_x.pop(0)
        history_y.pop(0)
        
    # 更新轨迹线
    traj_line.set_data(history_x, history_y)
    
    # 更新机器人本体矩形的位置和旋转
    # 计算矩形左下角的位置
    rect_x = current_x - (ROBOT_LENGTH/2)*math.cos(current_theta) + (ROBOT_WIDTH/2)*math.sin(current_theta)
    rect_y = current_y - (ROBOT_LENGTH/2)*math.sin(current_theta) - (ROBOT_WIDTH/2)*math.cos(current_theta)
    
    robot_patch.set_xy((rect_x, rect_y))
    robot_patch.set_angle(math.degrees(current_theta))
    
    # 更新机器人朝向指示线
    head_x = current_x + (ROBOT_LENGTH/2)*math.cos(current_theta)
    head_y = current_y + (ROBOT_LENGTH/2)*math.sin(current_theta)
    head_line.set_data([current_x, head_x], [current_y, head_y])
    
    # 动态调整坐标轴范围 (保持比例 1:1)
    margin = 1.0
    # 确保 history_x 和 history_y 至少有一个有效的值且不是完全相同的极值
    if len(history_x) > 0:
        min_x, max_x = min(history_x) - margin, max(history_x) + margin
        min_y, max_y = min(history_y) - margin, max(history_y) + margin
    else:
        min_x, max_x = -margin, margin
        min_y, max_y = -margin, margin
    
    # 保证视野是个正方形，避免图形被拉伸
    x_range = max_x - min_x
    y_range = max_y - min_y
    max_range = max(x_range, y_range)
    
    center_x = (max_x + min_x) / 2
    center_y = (max_y + min_y) / 2
    
    ax.set_xlim(center_x - max_range/2, center_x + max_range/2)
    ax.set_ylim(center_y - max_range/2, center_y + max_range/2)
    
    # 更新标题信息
    title_str = (f"Chassis Odometry (Dead Reckoning)\n"
                 f"Pose -> X: {current_x:+.2f}m, Y: {current_y:+.2f}m, Yaw: {math.degrees(current_theta):+.1f}°")
    ax.set_title(title_str)
    
    return traj_line, robot_patch, head_line

def on_close(event):
    """窗口关闭事件"""
    global is_running
    print("正在关闭...")
    is_running = False

def on_key_press(event):
    """键盘按键事件：用于清零统计数据"""
    global pose_x, pose_y, pose_theta
    global total_dist_x, total_dist_y, total_angle
    global history_x, history_y
    
    if event.key == 'r' or event.key == 'R':
        with data_lock:
            # 清零所有位姿和累计里程
            pose_x = 0.0
            pose_y = 0.0
            pose_theta = 0.0
            total_dist_x = 0.0
            total_dist_y = 0.0
            total_angle = 0.0
            
            # 清空历史轨迹
            history_x.clear()
            history_y.clear()
            history_x.append(0.0)
            history_y.append(0.0)
            
        print("\n>>> 数据已重置归零！可以开始新的标定。 <<<\n")

if __name__ == "__main__":
    print("=== 麦克纳姆轮底盘 轮速计可视化工具 ===")
    
    # 1. 连接串口
    port_name = select_serial_port()
    
    # 允许用户输入自定义波特率
    try:
        baud_input = input(f"请输入波特率 (直接回车默认 {BAUDRATE}): ").strip()
        if baud_input:
            BAUDRATE = int(baud_input)
    except ValueError:
        print("波特率输入无效，使用默认值。")
        
    try:
        serial_port = serial.Serial(port_name, BAUDRATE, timeout=0.1)
        print(f"成功打开串口: {port_name} @ {BAUDRATE} bps")
    except Exception as e:
        print(f"打开串口失败: {e}")
        sys.exit(1)
        
    # 2. 启动后台接收与解算线程
    rx_thread = threading.Thread(target=serial_receive_thread, daemon=True)
    rx_thread.start()
    
    # 3. 初始化 matplotlib 画布
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.mpl_connect('close_event', on_close)
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_xlabel("World X (meters)")
    ax.set_ylabel("World Y (meters)")
    
    # 轨迹线
    traj_line, = ax.plot([], [], 'b-', linewidth=1.5, label='Trajectory')
    
    # 机器人本体 (矩形)
    robot_patch = patches.Rectangle((0, 0), ROBOT_LENGTH, ROBOT_WIDTH, 
                                    angle=0.0, fill=True, color='orange', alpha=0.5)
    ax.add_patch(robot_patch)
    
    # 机器人朝向指示线 (车头红线)
    head_line, = ax.plot([], [], 'r-', linewidth=3.0, label='Heading')
    
    ax.legend(loc='upper right')
    
    # 4. 启动动画定时器 (每 50ms 刷新一次界面，20帧)
    print("正在启动绘图界面...")
    print("--------------------------------------------------")
    print("操作提示:")
    print(" - 保持绘图窗口为焦点，按下键盘 'r' 键，可一键清零所有位姿和累计数据！")
    print(" - 关闭绘图窗口即可退出程序。")
    print("--------------------------------------------------")
    # 取消 blit=False, cache_frame_data=False 的硬编码要求，尝试让 matplotlib 自己决定最优方式
    # 为避免无限帧导致的缓存内存溢出警告，显式传入 cache_frame_data=False
    ani = FuncAnimation(fig, update_plot, interval=50, cache_frame_data=False)
    
    plt.show()
    
    # 5. 清理退出
    is_running = False
    rx_thread.join(timeout=1.0)
    if serial_port and serial_port.is_open:
        serial_port.close()
    print("程序已退出。")