#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 Bootloader Protocol Checksum Calculator
根据协议规范自动计算校验和
"""

def calculate_checksum(length, msg_type, status, data=None):
    """
    计算协议帧的校验和
    
    校验和计算公式: CHECKSUM = ~(LENGTH + TYPE + STATUS + 每个DATA字节)
    
    参数:
        length: 数据长度 (uint8_t)
        msg_type: 消息类型 (uint8_t)
        status: 状态码 (uint8_t)
        data: 数据字节数组 (可选)
    
    返回:
        校验和 (uint8_t)
    """
    # 累加 LENGTH, TYPE, STATUS
    sum_value = length + msg_type + status
    
    # 如果有数据，累加所有数据字节
    if data is not None:
        if isinstance(data, (list, tuple, bytes, bytearray)):
            for byte in data:
                sum_value += byte
        else:
            raise ValueError("data 必须是 list, tuple, bytes 或 bytearray 类型")
    
    # 取低8位并按位取反
    checksum = (~sum_value) & 0xFF
    
    return checksum


def build_frame(msg_type, status, data=None, direction='pc_to_mcu'):
    """
    构建完整的协议帧
    
    参数:
        msg_type: 消息类型 (uint8_t)
        status: 状态码 (uint8_t)
        data: 数据字节数组 (可选)
        direction: 'pc_to_mcu' (0xAA) 或 'mcu_to_pc' (0xBB)
    
    返回:
        完整的帧字节数组
    """
    # 确定帧头
    if direction == 'pc_to_mcu':
        header = 0xAA
    elif direction == 'mcu_to_pc':
        header = 0xBB
    else:
        raise ValueError("direction 必须是 'pc_to_mcu' 或 'mcu_to_pc'")
    
    # 计算数据长度
    if data is None:
        length = 0
        data_bytes = []
    else:
        if isinstance(data, (list, tuple)):
            data_bytes = list(data)
        elif isinstance(data, (bytes, bytearray)):
            data_bytes = list(data)
        else:
            raise ValueError("data 必须是 list, tuple, bytes 或 bytearray 类型")
        length = len(data_bytes)
    
    # 计算校验和
    checksum = calculate_checksum(length, msg_type, status, data_bytes if data_bytes else None)
    
    # 构建完整帧
    frame = [header, length, msg_type, status] + data_bytes + [checksum]
    
    return bytearray(frame)


def print_frame(frame, description=""):
    """
    打印帧的十六进制表示
    
    参数:
        frame: 帧字节数组
        description: 帧描述
    """
    if description:
        print(f"\n{description}")
    
    # 打印十六进制
    hex_str = ' '.join(f'{b:02X}' for b in frame)
    print(f"HEX: {hex_str}")
    
    # 打印详细解析
    if len(frame) >= 5:
        print(f"  Header:   0x{frame[0]:02X} ({'PC→MCU' if frame[0] == 0xAA else 'MCU→PC'})")
        print(f"  Length:   0x{frame[1]:02X} ({frame[1]} bytes)")
        print(f"  Type:     0x{frame[2]:02X}")
        print(f"  Status:   0x{frame[3]:02X}")
        if frame[1] > 0:
            data_hex = ' '.join(f'{b:02X}' for b in frame[4:4+frame[1]])
            print(f"  Data:     {data_hex}")
        print(f"  Checksum: 0x{frame[-1]:02X}")
    
    # 打印Python bytes格式
    print(f"Python: {bytes(frame)}")
    
    # 打印C数组格式
    c_array = ', '.join(f'0x{b:02X}' for b in frame)
    print(f"C Array: uint8_t frame[] = {{{c_array}}};")


def verify_checksum(frame):
    """
    验证帧的校验和是否正确
    
    参数:
        frame: 帧字节数组
    
    返回:
        True 如果校验和正确，否则 False
    """
    if len(frame) < 5:
        return False
    
    length = frame[1]
    msg_type = frame[2]
    status = frame[3]
    
    # 提取数据部分
    if length > 0:
        data = frame[4:4+length]
    else:
        data = None
    
    received_checksum = frame[4+length]
    
    # 计算校验和
    calculated_checksum = calculate_checksum(length, msg_type, status, data)
    
    return calculated_checksum == received_checksum


# ============================================================================
# 协议命令定义
# ============================================================================

# 消息类型
CMD_SYSTEM_INFO = 0x01
CMD_MEMORY_INFO = 0x02
CMD_STATUS_REPORT = 0x03
CMD_ERROR_REPORT = 0x04
CMD_JUMP_RESPONSE = 0x05
CMD_GET_INFO = 0x10
CMD_GET_MEMORY = 0x11
CMD_HEARTBEAT = 0x12
CMD_JUMP_TO_MODE = 0x13
CMD_SOFT_RESET = 0x14
CMD_SOFT_RESET_RESPONSE = 0x15
CMD_ERASE_FLASH = 0x20
CMD_ERASE_RESPONSE = 0x21
CMD_WRITE_FLASH = 0x30
CMD_WRITE_RESPONSE = 0x31

# 状态码
STATUS_OK = 0x00
STATUS_READY = 0x01
STATUS_BUSY = 0x02
STATUS_IDLE = 0x03
ERROR_INVALID_CMD = 0x10
ERROR_CHECKSUM = 0x11
ERROR_LENGTH = 0x12
ERROR_TIMEOUT = 0x13
ERROR_INVALID_PARAM = 0x14
ERROR_INVALID_ADDRESS = 0x42
ERROR_WRITE_PROTECTED = 0x44

# ============================================================================
# 示例用法
# ============================================================================

def example_heartbeat():
    """示例：心跳包"""
    print("\n" + "="*60)
    print("示例 1: 心跳包 (PC → MCU)")
    print("="*60)
    frame = build_frame(CMD_HEARTBEAT, 0x00, direction='pc_to_mcu')
    print_frame(frame, "心跳请求帧")
    print(f"校验和验证: {'通过' if verify_checksum(frame) else '失败'}")


def example_soft_reset():
    """示例：软复位命令"""
    print("\n" + "="*60)
    print("示例: 软复位命令 (PC → MCU)")
    print("="*60)
    print("\n功能: 命令设备执行软件复位重启")
    print("说明: 设备收到此命令后会发送确认响应，然后在10ms内执行NVIC_SystemReset()")
    print("      设备将自动重启，串口连接会中断")
    
    # PC发送的软复位请求
    frame = build_frame(CMD_SOFT_RESET, 0x00, direction='pc_to_mcu')
    print_frame(frame, "\n软复位请求帧 (PC → MCU)")
    print(f"校验和验证: {'通过' if verify_checksum(frame) else '失败'}")
    
    # 预期的MCU响应
    response = build_frame(CMD_SOFT_RESET_RESPONSE, STATUS_OK, direction='mcu_to_pc')
    print_frame(response, "\n预期响应帧 (MCU → PC)")
    print(f"校验和验证: {'通过' if verify_checksum(response) else '失败'}")
    
    print("\n使用场景:")
    print("  • 固件烧录完成后自动重启设备")
    print("  • 远程重启设备以应用配置更改")
    print("  • 从异常状态恢复设备")
    
    print("\n注意事项:")
    print("  • 设备复位后串口连接会中断，需要重新连接")
    print("  • 建议在发送命令后等待至少500ms再重新连接")
    print("  • 设备将从Bootloader或Application启动（取决于启动配置）")


def quick_soft_reset():
    """
    快速生成软复位命令
    :return: 生成的命令帧
    """
    frame = build_frame(CMD_SOFT_RESET, 0x00, direction='pc_to_mcu')
    return frame


def quick_erase(start_address, page_count):
    """
    快速生成擦除Flash命令
    :param start_address: 起始地址 (必须是页对齐, 0x400的整数倍)
    :param page_count: 要擦除的页数 (1-48)
    :return: 生成的命令帧
    """
    # 地址转换为4字节小端序
    addr_bytes = [
        start_address & 0xFF,
        (start_address >> 8) & 0xFF,
        (start_address >> 16) & 0xFF,
        (start_address >> 24) & 0xFF
    ]
    # 页数转换为2字节小端序
    count_bytes = [
        page_count & 0xFF,
        (page_count >> 8) & 0xFF
    ]
    
    data = addr_bytes + count_bytes
    frame = build_frame(CMD_ERASE_FLASH, 0x00, data, direction='pc_to_mcu')
    return frame


def example_erase_flash():
    """示例：擦除Flash"""
    print("\n" + "="*60)
    print("示例 2: 擦除Flash (PC → MCU)")
    print("="*60)
    
    # 擦除请求: 起始地址=0x08004000, 页数=10
    frame = quick_erase(0x08004000, 10)
    print_frame(frame, "擦除Flash请求帧 (地址: 0x08004000, 页数: 10)")
    print(f"校验和验证: {'通过' if verify_checksum(frame) else '失败'}")


def quick_write(start_address, data_bytes):
    """
    快速生成写入Flash命令
    :param start_address: 起始地址 (必须是偶数)
    :param data_bytes: 要写入的数据列表 (长度必须是2-100的偶数)
    :return: 生成的命令帧
    """
    # 地址转换为4字节小端序
    addr_bytes = [
        start_address & 0xFF,
        (start_address >> 8) & 0xFF,
        (start_address >> 16) & 0xFF,
        (start_address >> 24) & 0xFF
    ]
    
    data_length = len(data_bytes)
    # 构建数据部分: start_address + data_length + data[实际长度]
    data_field = addr_bytes + [data_length] + data_bytes
    
    frame = build_frame(CMD_WRITE_FLASH, 0x00, data_field, direction='pc_to_mcu')
    return frame


def example_write_flash():
    """示例：写入Flash"""
    print("\n" + "="*60)
    print("示例 3: 写入Flash (PC → MCU)")
    print("="*60)
    
    # 写入请求: 起始地址=0x08005000, 数据长度=10字节
    address = 0x08005000
    data_length = 10
    test_data = list(range(1, data_length + 1))  # 0x01 ~ 0x0A
    
    frame = quick_write(address, test_data)
    print_frame(frame, f"写入Flash请求帧 (地址: 0x{address:08X}, 长度: {data_length} bytes)")
    print(f"校验和验证: {'通过' if verify_checksum(frame) else '失败'}")
    print(f"\n实际数据内容: {' '.join(f'0x{b:02X}' for b in test_data)}")
    print(f"协议帧总长度: {len(frame)} 字节 (节省 {105 + 5 - len(frame)} 字节)")


def example_write_flash_100_bytes():
    """示例：写入Flash (100字节)"""
    print("\n" + "="*60)
    print("示例 4: 写入Flash 100字节 (PC → MCU)")
    print("="*60)
    
    # 写入请求: 起始地址=0x08005000, 数据长度=100字节
    address = 0x08005000
    address_bytes = [
        address & 0xFF,
        (address >> 8) & 0xFF,
        (address >> 16) & 0xFF,
        (address >> 24) & 0xFF
    ]
    
    data_length = 100
    # 生成100字节测试数据 (循环0x00~0xFF)
    test_data = [(i % 256) for i in range(data_length)]
    
    # 构建数据部分: start_address + data_length + data[实际长度]
    data_field = address_bytes + [data_length] + test_data
    
    frame = build_frame(CMD_WRITE_FLASH, 0x00, data_field, direction='pc_to_mcu')
    print_frame(frame, f"写入Flash请求帧 (地址: 0x{address:08X}, 长度: {data_length} bytes)")
    print(f"校验和验证: {'通过' if verify_checksum(frame) else '失败'}")
    print(f"\n数据前10字节: {' '.join(f'0x{b:02X}' for b in test_data[:10])}")
    print(f"数据后10字节: {' '.join(f'0x{b:02X}' for b in test_data[-10:])}")
    print(f"协议帧总长度: {len(frame)} 字节")


def interactive_mode():
    """交互式模式"""
    print("\n" + "="*60)
    print("交互式校验和计算器")
    print("="*60)
    
    while True:
        print("\n请选择操作:")
        print("1. 计算心跳包")
        print("2. 计算软复位命令")
        print("3. 计算擦除Flash命令 (自定义)")
        print("4. 计算写入Flash命令 (自定义)")
        print("5. 验证已有帧的校验和")
        print("6. 退出")
        
        choice = input("\n请输入选择 (1-6): ").strip()
        
        if choice == '1':
            example_heartbeat()
        elif choice == '2':
            example_soft_reset()
        elif choice == '3':
            try:
                print("\n擦除Flash命令生成")
                print("-" * 60)
                
                # 输入起始地址
                addr_str = input("请输入擦除起始地址 (十六进制, 如 0x08004000): ").strip()
                erase_address = int(addr_str, 16)
                
                # 检查地址是否为页对齐（1KB = 0x400）
                if erase_address % 0x400 != 0:
                    print(f"警告: 地址 0x{erase_address:08X} 不是页对齐的!")
                    print(f"最近的页对齐地址: 0x{(erase_address // 0x400) * 0x400:08X}")
                    confirm = input("是否继续? (y/n): ").strip().lower()
                    if confirm != 'y':
                        continue
                
                # 输入擦除页数
                page_count = int(input("请输入擦除页数 (1-48): ").strip())
                
                if page_count < 1 or page_count > 48:
                    print("错误: 页数必须在1-48之间!")
                    continue
                
                # 检查地址范围
                BOOTLOADER_END = 0x08003FFF
                APPLICATION_START = 0x08004000
                APPLICATION_END = 0x0800FFFF
                
                if erase_address < APPLICATION_START:
                    print(f"错误: 地址 0x{erase_address:08X} 位于Bootloader区域 (< 0x08004000)!")
                    print("不允许擦除Bootloader区域!")
                    continue
                
                erase_end = erase_address + page_count * 0x400 - 1
                if erase_end > APPLICATION_END:
                    print(f"错误: 擦除范围超出Flash边界!")
                    print(f"结束地址: 0x{erase_end:08X}, Flash末尾: 0x{APPLICATION_END:08X}")
                    continue
                
                # 显示擦除信息
                print(f"\n擦除信息:")
                print(f"  起始地址: 0x{erase_address:08X}")
                print(f"  结束地址: 0x{erase_end:08X}")
                print(f"  页数: {page_count}")
                print(f"  总大小: {page_count * 0x400} 字节 ({page_count} KB)")
                
                # 计算页号
                start_page = (erase_address - 0x08000000) // 0x400
                end_page = start_page + page_count - 1
                print(f"  页面范围: 页面{start_page} - 页面{end_page}")
                
                confirm = input("\n确认擦除? (y/n): ").strip().lower()
                if confirm != 'y':
                    print("已取消")
                    continue
                
                # 构建擦除命令
                # start_page_address (4字节, 小端序)
                # page_count (2字节, 小端序)
                erase_data = [
                    erase_address & 0xFF,
                    (erase_address >> 8) & 0xFF,
                    (erase_address >> 16) & 0xFF,
                    (erase_address >> 24) & 0xFF,
                    page_count & 0xFF,
                    (page_count >> 8) & 0xFF
                ]
                
                erase_frame = build_frame(CMD_ERASE_FLASH, 0x00, erase_data, direction='pc_to_mcu')
                print_frame(erase_frame, f"擦除Flash命令 (地址: 0x{erase_address:08X}, 页数: {page_count})")
                print(f"校验和验证: {'通过' if verify_checksum(erase_frame) else '失败'}")
                
                print("\n预期响应:")
                print("  消息类型: CMD_ERASE_RESPONSE (0x21)")
                print("  状态: STATUS_ERASE_COMPLETED (0x40) 表示成功")
                print("  或  : ERROR_ERASE_* (0x41-0x44) 表示错误")
                
            except ValueError as e:
                print(f"输入错误: {e}")
            except Exception as e:
                print(f"错误: {e}")
        elif choice == '4':
            try:
                addr_str = input("请输入写入地址 (十六进制, 如 0x08005000): ").strip()
                address = int(addr_str, 16)
                
                length = int(input("请输入数据长度 (2-100, 偶数): ").strip())
                
                if length < 2 or length > 100 or length % 2 != 0:
                    print("错误: 数据长度必须是2~100之间的偶数!")
                    continue
                
                print(f"请输入{length}字节数据 (十六进制，空格分隔，如: 01 02 03):")
                data_str = input().strip()
                test_data = [int(x, 16) for x in data_str.split()]
                
                if len(test_data) != length:
                    print(f"错误: 输入了{len(test_data)}字节，但指定长度为{length}字节!")
                    continue
                
                # 构建数据部分（可变长度协议）
                address_bytes = [
                    address & 0xFF,
                    (address >> 8) & 0xFF,
                    (address >> 16) & 0xFF,
                    (address >> 24) & 0xFF
                ]
                
                # 只传输实际需要的字节数，不填充到100字节
                data_field = address_bytes + [length] + test_data
                
                frame = build_frame(CMD_WRITE_FLASH, 0x00, data_field, direction='pc_to_mcu')
                print_frame(frame, f"写入Flash请求帧 (地址: 0x{address:08X}, 长度: {length} bytes)")
                print(f"校验和验证: {'通过' if verify_checksum(frame) else '失败'}")
                print(f"协议帧总长度: {len(frame)} 字节 (可变长度协议)")
                
            except Exception as e:
                print(f"错误: {e}")
        
        elif choice == '5':
            try:
                print("请输入帧数据 (十六进制，空格分隔，如: AA 00 12 00 ED):")
                frame_str = input().strip()
                frame = bytearray([int(x, 16) for x in frame_str.split()])
                
                if verify_checksum(frame):
                    print("✓ 校验和正确!")
                    print_frame(frame, "帧内容")
                else:
                    print("✗ 校验和错误!")
                    if len(frame) >= 5:
                        length = frame[1]
                        msg_type = frame[2]
                        status = frame[3]
                        data = frame[4:4+length] if length > 0 else None
                        correct_checksum = calculate_checksum(length, msg_type, status, data)
                        print(f"接收到的校验和: 0x{frame[-1]:02X}")
                        print(f"正确的校验和:   0x{correct_checksum:02X}")
                
            except Exception as e:
                print(f"错误: {e}")
        
        elif choice == '6':
            print("\n再见!")
            break
        else:
            print("无效的选择，请重新输入!")


# ============================================================================
# 主程序
# ============================================================================

if __name__ == "__main__":
    print("="*60)
    print("STM32 Bootloader 协议校验和计算器")
    print("="*60)
    print("\n提供了以下快速函数供程序调用:")
    print("  - quick_soft_reset()                      # 生成软复位命令")
    print("  - quick_erase(start_address, page_count)  # 生成擦除命令")
    print("  - quick_write(start_address, data_bytes)  # 生成写入命令")
    print("\n运行示例:")
    
    # 运行所有示例
    example_heartbeat()
    example_soft_reset()
    example_erase_flash()
    example_write_flash()
    example_write_flash_100_bytes()
    
    # 进入交互模式
    print("\n\n是否进入交互模式? (y/n): ", end='')
    choice = input().strip().lower()
    if choice == 'y':
        interactive_mode()
    else:
        print("\n程序结束。")
