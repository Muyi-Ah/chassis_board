import sys
import time
import threading

try:
    import serial
    import serial.tools.list_ports
    import keyboard
except ImportError:
    print("错误: 缺少依赖库。")
    print("请使用您的包管理器安装依赖，例如: uv pip install pyserial keyboard")
    sys.exit(1)

BAUD_RATE = 921600

# 全局变量
ser_global = None
running = True

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

def send_command(ser, cmd_str):
    """
    辅助发送函数，添加回车换行并打印提示
    """
    if ser and ser.is_open:
        # 遵循 Vofa+ / 纯文本风格，以 \r\n 结尾
        full_cmd = f"{cmd_str}\r\n"
        ser.write(full_cmd.encode('utf-8'))
        ser.flush()
        print(f"\n[串口发送] -> {cmd_str}")
    else:
        print("\n[错误] 串口未打开，发送失败。")

def on_esc_pressed():
    """
    当检测到 ESC 键时的处理函数
    """
    global ser_global
    print("\n[中断] 检测到 ESC 键被按下！下发停止指令...")
    send_command(ser_global, "CMD:STOP")
    # 清理当前行并重新打印提示符
    print(">> 请输入指令 (start/stop/motor/exit): ", end='', flush=True)

def keyboard_listener():
    """
    独立线程：使用阻塞式的 wait 来监听 ESC 键
    这通常比 hook 钩子更可靠，特别是遇到 IDE 终端焦点问题时
    """
    global running
    while running:
        try:
            # 阻塞等待 ESC 按下
            keyboard.wait('esc')
            if running:
                on_esc_pressed()
        except Exception:
            pass
        time.sleep(0.1)

def main():
    global ser_global, running
    
    print("=== STM32 纯文本指令下发工具 (按 ESC 紧急停止) ===")
    port_name = select_port()
    if not port_name:
        return

    print(f"\n正在尝试连接 {port_name} (波特率: {BAUD_RATE})...")
    
    try:
        ser_global = serial.Serial(port_name, BAUD_RATE, timeout=1)
        print("连接成功！")
        
        # 启动后台线程监听 ESC，这样不会被主线程的 input() 阻塞
        listener_thread = threading.Thread(target=keyboard_listener, daemon=True)
        listener_thread.start()
        print("已注册全局快捷键: [ESC] -> 立即下发停止指令 (CMD:STOP)")
        
        print("\n=== 可用交互指令说明 ===")
        print("1. start             -> 发送 CMD:START")
        print("2. stop              -> 发送 CMD:STOP (也可直接按 ESC 键)")
        print("3. motor <id> <rpm>  -> 发送 MOTOR:<id>:<rpm> (示例: motor 1 1000)")
        print("4. exit / quit       -> 退出脚本")
        print("========================\n")
        
        while running:
            try:
                # input 是阻塞的，但 keyboard_listener 线程会在后台独立运行
                cmd_input = input(">> 请输入指令 (start/stop/motor/exit): ").strip()
                if not running:
                    break
                if not cmd_input:
                    continue
                    
                parts = cmd_input.lower().split()
                cmd = parts[0]
                
                if cmd == 'exit' or cmd == 'quit':
                    print("准备退出...")
                    running = False
                    break
                elif cmd == 'start':
                    send_command(ser_global, "CMD:START")
                elif cmd == 'stop':
                    send_command(ser_global, "CMD:STOP")
                elif cmd == 'motor':
                    if len(parts) >= 3:
                        motor_id = parts[1]
                        motor_rpm = parts[2]
                        # 拼接为 MOTOR:1:1000 形式的纯文本指令
                        send_command(ser_global, f"MOTOR:{motor_id}:{motor_rpm}")
                    else:
                        print("[错误] motor 指令参数不足。用法: motor <id> <rpm>")
                else:
                    # 自定义直接发送，便于后续扩展
                    send_command(ser_global, cmd_input.upper())
                    
            except EOFError:
                running = False
                break
                
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，准备退出...")
    finally:
        running = False
        if ser_global and ser_global.is_open:
            ser_global.close()
            print("串口已关闭。")
        print("脚本已退出。")

if __name__ == "__main__":
    main()
