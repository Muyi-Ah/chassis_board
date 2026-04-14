import sys
import time
import threading
from datetime import datetime

# 检查依赖
try:
    import serial
    import serial.tools.list_ports
    import keyboard
except ImportError:
    print("错误: 缺少依赖库。")
    print("请使用您的环境包管理器安装，例如: uv pip install pyserial keyboard")
    sys.exit(1)

BAUD_RATE = 921600
running = True

def esc_listener():
    """
    监听 ESC 键并在按下时设置退出标志
    """
    global running
    try:
        # 阻塞等待按下 ESC
        keyboard.wait('esc')
        print("\n[中断] 检测到 ESC 键被按下！准备退出...")
        running = False
    except Exception:
        pass

def select_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未检测到任何可用串口，请检查设备连接。")
        return None
    
    print("发现以下可用串口:")
    for i, port in enumerate(ports):
        print(f"[{i}] {port.device} - {port.description}")
        
    while True:
        try:
            choice = input(f"请输入串口序号 (0-{len(ports)-1}): ")
            index = int(choice)
            if 0 <= index < len(ports):
                return ports[index].device
            else:
                print("序号超出范围，请重新输入。")
        except ValueError:
            print("请输入有效的数字序号。")

def main():
    global running
    print("=== STM32 串口数据接收并保存为 CSV 工具 ===")
    port_name = select_port()
    if not port_name:
        return

    # 询问用户缩放倍数
    while True:
        try:
            scale_input = input("请输入数据缩放除数 (即下位机乘了多少发过来，直接回车默认 1000): ")
            if not scale_input.strip():
                scale_factor = 1000.0
            else:
                scale_factor = float(scale_input)
            
            if scale_factor == 0:
                print("除数不能为 0，请重新输入。")
                continue
            break
        except ValueError:
            print("请输入有效的数字。")

    # 生成带时间戳的文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"data_{timestamp}.csv"
    
    print(f"\n正在尝试连接 {port_name} (波特率: {BAUD_RATE})...")
    
    try:
        ser = serial.Serial(port_name, BAUD_RATE, timeout=0.1)
        print("连接成功！")
        print(f"开始接收数据并保存到: {filename}")
        print(f"当前数据除数: {scale_factor}")
        print("按 [ESC] 键 或 Ctrl+C 停止接收并退出。\n")
        
        # 启动后台线程监听 ESC
        listener_thread = threading.Thread(target=esc_listener, daemon=True)
        listener_thread.start()
        
        with open(filename, 'w', encoding='utf-8') as f:
            # 写入 CSV 表头
            f.write("vx,vy,vw\n")
            
            while running:
                if ser.in_waiting > 0:
                    try:
                        # 读取一行并解码，去除两端空白字符
                        line = ser.readline().decode('utf-8').strip()
                        if line:
                            try:
                                # 兼容逗号分隔或空格分隔
                                parts = line.split(',') if ',' in line else line.split()
                                if len(parts) == 3:
                                    # 将整数还原为真实浮点数
                                    vx = int(parts[0]) / scale_factor
                                    vy = int(parts[1]) / scale_factor
                                    vw = int(parts[2]) / scale_factor
                                    
                                    # 格式化为浮点数并写入 CSV
                                    parsed_line = f"{vx:.3f},{vy:.3f},{vw:.3f}"
                                    f.write(parsed_line + "\n")
                                    f.flush()
                                    print(f"Rx: {parsed_line}")
                                else:
                                    # 格式不符时直接保存原始数据
                                    f.write(line + "\n")
                                    f.flush()
                                    print(f"Rx (Raw): {line}")
                            except ValueError:
                                # 解析失败时也保存原始数据
                                f.write(line + "\n")
                                f.flush()
                                print(f"Rx (Raw): {line}")
                    except UnicodeDecodeError:
                        print("Rx: [解码错误, 可能波特率不匹配或包含二进制数据]")
                else:
                    # 短暂休眠释放 CPU
                    time.sleep(0.001)
                        
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n\n接收被用户中断 (Ctrl+C)。")
    finally:
        running = False
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭。")
        print("脚本退出。")

if __name__ == "__main__":
    main()
