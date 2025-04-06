import serial
import serial.tools.list_ports
import time

try:
    import readline  # 在 Unix 系统可用
except ImportError:
    try:
        import pyreadline as readline  # Windows 需安装 pyreadline3
    except ImportError:
        print("⚠️ 无法加载 readline，↑ ↓ 键历史可能无效")

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def choose_port():
    ports = list_serial_ports()
    if not ports:
        print("❌ 没有发现串口设备。请检查连接。")
        return None
    print("\n可用串口：")
    for i, p in enumerate(ports):
        print(f"  {i + 1}: {p}")
    while True:
        choice = input("请输入串口编号（或直接输入串口名）: ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            return ports[int(choice) - 1]
        elif choice in ports:
            return choice
        else:
            print("⚠️ 无效选择，请重试。")

def send_nextion_command(ser, command):
    if not command.endswith('\xff\xff\xff'):
        command += '\xff\xff\xff'
    ser.write(command.encode('latin1'))
    print(f"\n✅ 发送: {command.encode('latin1').hex().upper()}")

def read_nextion_response(ser):
    time.sleep(0.1)
    if ser.in_waiting:
        response = ser.read(ser.in_waiting)
        print(f"📩 返回: {response.hex().upper()}\n")
    else:
        print("（无返回）\n")

def main():
    print("=== Nextion 串口调试工具（交互增强） ===")

    while True:
        port = choose_port()
        if not port:
            return

        baud_input = input("请输入波特率（默认 9600）: ").strip()
        baudrate = int(baud_input) if baud_input.isdigit() else 9600

        try:
            ser = serial.Serial(port, baudrate, bytesize=8, parity='N', stopbits=1, timeout=1)
            print(f"\n✅ 已打开串口 {port} @ {baudrate}bps\n")
        except Exception as e:
            print(f"\n❌ 串口打开失败: {e}\n")
            continue

        print("输入指令发送给 Nextion（输入 exit 退出，clear 清屏）：")
        print("可用 ↑ ↓ 键查看历史记录\n")

        while True:
            try:
                cmd = input(">>> ").strip()
                if cmd.lower() == 'exit':
                    ser.close()
                    print("\n👋 串口已关闭，程序退出。\n")
                    return
                elif cmd.lower() == 'clear':
                    print("\033[H\033[J", end="")  # 清屏（部分终端有效）
                    continue
                elif cmd == '':
                    continue
                send_nextion_command(ser, cmd)
                read_nextion_response(ser)
            except KeyboardInterrupt:
                ser.close()
                print("\n👋 中断退出。串口已关闭。\n")
                return
            except Exception as e:
                print(f"\n⚠️ 错误: {e}\n")
                continue

if __name__ == "__main__":
    main()
