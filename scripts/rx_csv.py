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

    # 生成带时间戳的文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"data_{timestamp}.csv"
    
    print(f"\n正在尝试连接 {port_name} (波特率: {BAUD_RATE})...")
    
    try:
        ser = serial.Serial(port_name, BAUD_RATE, timeout=0.1)
        print("连接成功！")
        print(f"开始接收数据并保存到: {filename}")
        print("按 [ESC] 键 或 Ctrl+C 停止接收并退出。\n")
        
        # 启动后台线程监听 ESC
        listener_thread = threading.Thread(target=esc_listener, daemon=True)
        listener_thread.start()
        
        with open(filename, 'w', encoding='utf-8') as f:
            # 可以根据需要在这里写入 CSV 表头
            # f.write("Motor1_RPM,Motor1_Filtered,Motor2_RPM,Motor2_Filtered,Motor3_RPM,Motor3_Filtered,Motor4_RPM,Motor4_Filtered\n")
            
            while running:
                if ser.in_waiting > 0:
                    try:
                        # 读取一行并解码，去除两端空白字符
                        line = ser.readline().decode('utf-8').strip()
                        if line:
                            # 写入文件并立即刷新缓冲区，防止数据丢失
                            f.write(line + "\n")
                            f.flush()
                            # 同时在控制台打印方便观察
                            print(f"Rx: {line}")
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
