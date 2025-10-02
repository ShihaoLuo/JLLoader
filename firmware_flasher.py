#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 固件烧录工具
自动检测运行模式，擦除Flash，并烧录固件到0x08004000
"""

import serial
import time
import os
import sys
from pathlib import Path

# ============================================================================
# 协议定义
# ============================================================================

# 消息类型
CMD_HEARTBEAT = 0x12
CMD_STATUS_REPORT = 0x03
CMD_JUMP_TO_MODE = 0x13
CMD_JUMP_RESPONSE = 0x05
CMD_ERASE_FLASH = 0x20
CMD_ERASE_RESPONSE = 0x21
CMD_WRITE_FLASH = 0x30
CMD_WRITE_RESPONSE = 0x31
CMD_ERROR_REPORT = 0x04

# 状态码
STATUS_OK = 0x00
STATUS_READY = 0x01
STATUS_BUSY = 0x02
STATUS_IDLE = 0x03
ERROR_INVALID_CMD = 0x10
ERROR_CHECKSUM = 0x11
ERROR_LENGTH = 0x12
ERROR_TIMEOUT = 0x13

# Flash操作状态
STATUS_ERASE_COMPLETED = 0x40
ERROR_ERASE_FAILED = 0x41
ERROR_INVALID_ADDRESS = 0x42
ERROR_INVALID_PAGE_COUNT = 0x43
ERROR_WRITE_PROTECTED = 0x44

# 运行模式
MODE_BOOTLOADER = 0x70
MODE_APPLICATION = 0x71
MODE_SYSTEM_MEMORY = 0x72
MODE_UNKNOWN = 0x73

# 模式跳转状态
MODE_JUMP_REQUESTED = 0x52
MODE_JUMP_PREPARING = 0x53
MODE_JUMP_SUCCESS = 0x54
MODE_JUMP_FAILED = 0x55
MODE_INVALID_TARGET = 0x56
MODE_JUMP_TIMEOUT = 0x57

# 帧头
HEADER_PC_TO_MCU = 0xAA
HEADER_MCU_TO_PC = 0xBB

# Flash配置
APP_START_ADDRESS = 0x08004000
APP_AREA_SIZE = 48 * 1024  # 48KB
PAGE_SIZE = 1024  # 1KB per page
TOTAL_APP_PAGES = 48
BOOTLOADER_END_ADDRESS = 0x08003FFF

# 写入配置
MAX_WRITE_SIZE = 100  # 最大写入字节数
CHUNK_SIZE = 100  # 每次写入100字节以获得最佳性能


# ============================================================================
# 协议工具函数
# ============================================================================

def calculate_checksum(length, msg_type, status, data=None):
    """计算协议帧的校验和"""
    sum_value = length + msg_type + status
    if data is not None:
        for byte in data:
            sum_value += byte
    checksum = (~sum_value) & 0xFF
    return checksum


def build_frame(msg_type, status, data=None, direction='pc_to_mcu'):
    """构建完整的协议帧"""
    header = HEADER_PC_TO_MCU if direction == 'pc_to_mcu' else HEADER_MCU_TO_PC
    length = len(data) if data is not None else 0
    
    frame = bytearray()
    frame.append(header)
    frame.append(length)
    frame.append(msg_type)
    frame.append(status)
    
    if data is not None:
        frame.extend(data)
    
    checksum = calculate_checksum(length, msg_type, status, data)
    frame.append(checksum)
    
    return bytes(frame)


def verify_checksum(frame):
    """验证接收到的帧的校验和"""
    if len(frame) < 5:
        return False
    
    length = frame[1]
    msg_type = frame[2]
    status = frame[3]
    data = frame[4:4+length] if length > 0 else None
    received_checksum = frame[-1]
    
    calculated_checksum = calculate_checksum(length, msg_type, status, data)
    return received_checksum == calculated_checksum


def parse_frame(frame):
    """解析接收到的帧"""
    if len(frame) < 5:
        return None
    
    result = {
        'header': frame[0],
        'length': frame[1],
        'type': frame[2],
        'status': frame[3],
        'data': frame[4:-1] if frame[1] > 0 else b'',
        'checksum': frame[-1],
        'valid': verify_checksum(frame)
    }
    return result


# ============================================================================
# 串口通信类
# ============================================================================

class STM32Flasher:
    def __init__(self, port, baudrate=115200, timeout=2):
        """初始化串口连接"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.current_mode = MODE_UNKNOWN
        
    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"✓ 串口 {self.port} 已连接 (波特率: {self.baudrate})")
            time.sleep(0.1)  # 等待串口稳定
            return True
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ 串口已断开")
    
    def send_frame(self, frame):
        """发送帧"""
        if not self.ser or not self.ser.is_open:
            print("✗ 串口未连接")
            return False
        
        try:
            self.ser.write(frame)
            self.ser.flush()
            return True
        except Exception as e:
            print(f"✗ 发送失败: {e}")
            return False
    
    def receive_frame(self, timeout=None, debug=False):
        """接收帧"""
        if not self.ser or not self.ser.is_open:
            print("✗ 串口未连接")
            return None
        
        old_timeout = self.ser.timeout
        if timeout is not None:
            self.ser.timeout = timeout
        
        try:
            # 逐字节读取完整帧
            frame = bytearray()
            
            # 1. 读取帧头
            header_byte = self.ser.read(1)
            if len(header_byte) == 0:
                if debug: print("  [调试] 未收到帧头")
                return None
            
            if header_byte[0] != HEADER_MCU_TO_PC:
                print(f"✗ 无效的帧头: 0x{header_byte[0]:02X}")
                return None
            
            frame.extend(header_byte)
            if debug: print(f"  [调试] 帧头: 0x{header_byte[0]:02X}")
            
            # 2. 读取长度字节
            length_byte = self.ser.read(1)
            if len(length_byte) == 0:
                print("✗ 接收长度超时")
                return None
            
            frame.extend(length_byte)
            length = length_byte[0]
            if debug: print(f"  [调试] 长度: {length}")
            
            # 3. 读取类型字节
            type_byte = self.ser.read(1)
            if len(type_byte) == 0:
                print("✗ 接收类型超时")
                return None
            
            frame.extend(type_byte)
            if debug: print(f"  [调试] 类型: 0x{type_byte[0]:02X}")
            
            # 4. 读取状态字节
            status_byte = self.ser.read(1)
            if len(status_byte) == 0:
                print("✗ 接收状态超时")
                return None
            
            frame.extend(status_byte)
            if debug: print(f"  [调试] 状态: 0x{status_byte[0]:02X}")
            
            # 5. 读取数据字节 (length 字节)
            if length > 0:
                data_bytes = self.ser.read(length)
                if len(data_bytes) < length:
                    print(f"✗ 数据不完整: 期望 {length} 字节, 收到 {len(data_bytes)} 字节")
                    return None
                frame.extend(data_bytes)
                if debug: print(f"  [调试] 数据 ({len(data_bytes)}字节): {data_bytes.hex(' ').upper()}")
            
            # 6. 读取校验和字节
            checksum_byte = self.ser.read(1)
            if len(checksum_byte) == 0:
                print("✗ 接收校验和超时")
                return None
            
            frame.extend(checksum_byte)
            if debug: print(f"  [调试] 校验和: 0x{checksum_byte[0]:02X}")
            if debug: print(f"  [调试] 完整帧长度: {len(frame)}")
            if debug: print(f"  [调试] 完整帧: {bytes(frame).hex(' ').upper()}")
            
            return bytes(frame)
            
        except Exception as e:
            print(f"✗ 接收失败: {e}")
            return None
        finally:
            self.ser.timeout = old_timeout
    
    def send_and_receive(self, frame, timeout=2, retry=3, debug=False):
        """发送帧并等待响应"""
        for attempt in range(retry):
            if not self.send_frame(frame):
                continue
            
            response = self.receive_frame(timeout, debug)
            if response:
                return response
            
            if attempt < retry - 1:
                print(f"  重试 {attempt + 1}/{retry - 1}...")
                time.sleep(0.1)
        
        return None
    
    # ========================================================================
    # 高级命令函数
    # ========================================================================
    
    def send_heartbeat(self, debug=False):
        """发送心跳包并检测运行模式"""
        print("\n[心跳检测]")
        frame = build_frame(CMD_HEARTBEAT, 0x00, None)
        print(f"  发送: {frame.hex(' ').upper()}")
        
        response = self.send_and_receive(frame, timeout=1, debug=debug)
        if not response:
            print("✗ 无响应")
            return False
        
        parsed = parse_frame(response)
        print(f"  接收: {response.hex(' ').upper()}")
        print(f"  接收长度: {len(response)} 字节")
        
        if not parsed['valid']:
            print("✗ 校验和错误")
            print(f"  解析结果: {parsed}")
            # 手动验证校验和
            if len(response) >= 5:
                length = response[1]
                msg_type = response[2]
                status = response[3]
                data = response[4:4+length] if length > 0 else None
                expected_checksum = calculate_checksum(length, msg_type, status, data)
                received_checksum = response[-1]
                print(f"  期望校验和: 0x{expected_checksum:02X}")
                print(f"  接收校验和: 0x{received_checksum:02X}")
            return False
        
        if parsed['type'] == CMD_STATUS_REPORT and len(parsed['data']) >= 12:
            # 解析状态报告
            data = parsed['data']
            uptime_ms = int.from_bytes(data[0:4], 'little')
            last_command = int.from_bytes(data[4:6], 'little')
            system_status = data[6]
            error_count = data[7]
            running_mode = data[8]
            
            self.current_mode = running_mode
            
            mode_name = {
                MODE_BOOTLOADER: "Bootloader",
                MODE_APPLICATION: "Application",
                MODE_SYSTEM_MEMORY: "System Memory",
                MODE_UNKNOWN: "Unknown"
            }.get(running_mode, f"Unknown (0x{running_mode:02X})")
            
            print(f"✓ 设备在线")
            print(f"  当前模式: {mode_name}")
            print(f"  运行时间: {uptime_ms}ms")
            print(f"  系统状态: 0x{system_status:02X}")
            print(f"  错误计数: {error_count}")
            return True
        else:
            print("✗ 响应格式错误")
            return False
    
    def jump_to_bootloader(self):
        """跳转到Bootloader模式"""
        print("\n[跳转到Bootloader]")
        
        # 构建跳转请求
        # target_mode (1) + jump_delay_ms (1) + timeout_ms (2) + magic_word (4)
        data = bytearray()
        data.append(MODE_BOOTLOADER)  # target_mode
        data.append(10)  # jump_delay_ms (10ms)
        data.extend((1000).to_bytes(2, 'little'))  # timeout_ms (1000ms)
        data.extend((0x12345678).to_bytes(4, 'little'))  # magic_word
        
        frame = build_frame(CMD_JUMP_TO_MODE, 0x00, data)
        print(f"  发送: {frame.hex(' ').upper()}")
        
        response = self.send_and_receive(frame, timeout=3)
        if not response:
            print("✗ 无响应")
            return False
        
        parsed = parse_frame(response)
        print(f"  接收: {response.hex(' ').upper()}")
        
        if not parsed['valid']:
            print("✗ 校验和错误")
            return False
        
        if parsed['type'] == CMD_JUMP_RESPONSE:
            status = parsed['status']
            status_names = {
                MODE_JUMP_REQUESTED: "跳转请求已收到",
                MODE_JUMP_PREPARING: "正在准备跳转", 
                MODE_JUMP_SUCCESS: "跳转成功",
                MODE_JUMP_FAILED: "跳转失败",
                MODE_INVALID_TARGET: "无效的目标模式",
                MODE_JUMP_TIMEOUT: "跳转超时"
            }
            
            status_name = status_names.get(status, f"未知状态 (0x{status:02X})")
            print(f"  状态: {status_name}")
            
            # 0x52-0x54 都是成功状态
            if status in [MODE_JUMP_REQUESTED, MODE_JUMP_PREPARING, MODE_JUMP_SUCCESS]:
                print("✓ 跳转成功，等待设备重启...")
                time.sleep(1)  # 等待设备重启
                self.current_mode = MODE_BOOTLOADER
                return True
            else:
                print(f"✗ 跳转失败，状态码: 0x{status:02X}")
                return False
        else:
            print("✗ 响应类型错误")
            return False
    
    def erase_flash(self, start_address, page_count):
        """擦除Flash"""
        print(f"\n[擦除Flash]")
        print(f"  起始地址: 0x{start_address:08X}")
        print(f"  页数: {page_count}")
        print(f"  擦除范围: 0x{start_address:08X} - 0x{start_address + page_count * PAGE_SIZE - 1:08X}")
        
        # 构建擦除请求
        data = bytearray()
        data.extend(start_address.to_bytes(4, 'little'))
        data.extend(page_count.to_bytes(2, 'little'))
        
        frame = build_frame(CMD_ERASE_FLASH, 0x00, data)
        print(f"  发送: {frame.hex(' ').upper()}")
        
        # 擦除操作需要较长时间，设置更长的超时
        erase_timeout = 5 + page_count * 0.1  # 基础5秒 + 每页100ms
        print(f"  等待擦除完成 (超时: {erase_timeout:.1f}秒)...")
        
        response = self.send_and_receive(frame, timeout=erase_timeout, retry=1)
        if not response:
            print("✗ 无响应或超时")
            return False
        
        parsed = parse_frame(response)
        print(f"  接收: {response.hex(' ').upper()}")
        
        if not parsed['valid']:
            print("✗ 校验和错误")
            return False
        
        if parsed['type'] == CMD_ERASE_RESPONSE and len(parsed['data']) >= 11:
            data = parsed['data']
            erase_status = data[0]
            returned_address = int.from_bytes(data[1:5], 'little')
            pages_erased = int.from_bytes(data[5:7], 'little')
            duration_ms = int.from_bytes(data[7:11], 'little')
            
            if erase_status == STATUS_ERASE_COMPLETED:
                print(f"✓ 擦除成功")
                print(f"  擦除地址: 0x{returned_address:08X}")
                print(f"  擦除页数: {pages_erased}")
                print(f"  耗时: {duration_ms}ms")
                return True
            else:
                error_names = {
                    ERROR_ERASE_FAILED: "硬件错误",
                    ERROR_INVALID_ADDRESS: "地址无效",
                    ERROR_INVALID_PAGE_COUNT: "页数无效",
                    ERROR_WRITE_PROTECTED: "写保护"
                }
                error_name = error_names.get(erase_status, f"未知错误 (0x{erase_status:02X})")
                print(f"✗ 擦除失败: {error_name}")
                return False
        else:
            print("✗ 响应格式错误")
            return False
    
    def write_flash_chunk(self, address, data):
        """写入一块数据到Flash (2-100字节，必须为偶数)"""
        if len(data) < 2 or len(data) > MAX_WRITE_SIZE or len(data) % 2 != 0:
            print(f"✗ 数据长度错误: {len(data)} (必须是2-{MAX_WRITE_SIZE}的偶数)")
            return False
        
        # 构建写入请求
        request_data = bytearray()
        request_data.extend(address.to_bytes(4, 'little'))
        request_data.append(len(data))
        request_data.extend(data)
        
        frame = build_frame(CMD_WRITE_FLASH, 0x00, request_data)
        
        response = self.send_and_receive(frame, timeout=2, retry=2)
        if not response:
            return False
        
        parsed = parse_frame(response)
        
        if not parsed['valid']:
            print("✗ 校验和错误")
            return False
        
        if parsed['type'] == CMD_WRITE_RESPONSE and len(parsed['data']) >= 13:
            data_resp = parsed['data']
            status = data_resp[0]
            returned_address = int.from_bytes(data_resp[1:5], 'little')
            written_size = int.from_bytes(data_resp[5:9], 'little')
            duration_ms = int.from_bytes(data_resp[9:13], 'little')
            
            if status == STATUS_OK:
                return True
            else:
                error_names = {
                    ERROR_INVALID_ADDRESS: "地址无效",
                    ERROR_LENGTH: "长度错误",
                    ERROR_WRITE_PROTECTED: "写保护"
                }
                error_name = error_names.get(status, f"未知错误 (0x{status:02X})")
                print(f"✗ 写入失败: {error_name} (地址: 0x{address:08X})")
                return False
        else:
            print("✗ 响应格式错误")
            return False
    
    def flash_firmware(self, firmware_data, start_address=APP_START_ADDRESS):
        """烧录完整固件"""
        print(f"\n[烧录固件]")
        print(f"  固件大小: {len(firmware_data)} 字节")
        print(f"  起始地址: 0x{start_address:08X}")
        print(f"  结束地址: 0x{start_address + len(firmware_data) - 1:08X}")
        
        # 确保数据长度为偶数
        if len(firmware_data) % 2 != 0:
            firmware_data = firmware_data + b'\xFF'
            print(f"  数据长度已补齐为偶数: {len(firmware_data)} 字节")
        
        total_bytes = len(firmware_data)
        total_chunks = (total_bytes + CHUNK_SIZE - 1) // CHUNK_SIZE
        bytes_written = 0
        failed_chunks = []
        
        print(f"  总共 {total_chunks} 块数据")
        print("\n开始写入:")
        
        start_time = time.time()
        
        for i in range(total_chunks):
            chunk_start = i * CHUNK_SIZE
            chunk_end = min(chunk_start + CHUNK_SIZE, total_bytes)
            chunk_data = firmware_data[chunk_start:chunk_end]
            chunk_address = start_address + chunk_start
            
            # 确保块大小为偶数
            if len(chunk_data) % 2 != 0:
                chunk_data = chunk_data + b'\xFF'
            
            progress = (i + 1) / total_chunks * 100
            print(f"  [{i+1}/{total_chunks}] 0x{chunk_address:08X} ({len(chunk_data):3d}字节) ", end='')
            
            if self.write_flash_chunk(chunk_address, chunk_data):
                bytes_written += len(chunk_data)
                print(f"✓ [{progress:5.1f}%]")
            else:
                print(f"✗ 失败")
                failed_chunks.append(i)
                # 尝试继续写入其他块
        
        elapsed_time = time.time() - start_time
        
        print(f"\n烧录完成:")
        print(f"  写入字节: {bytes_written}/{total_bytes}")
        print(f"  成功块数: {total_chunks - len(failed_chunks)}/{total_chunks}")
        print(f"  失败块数: {len(failed_chunks)}")
        print(f"  耗时: {elapsed_time:.2f}秒")
        print(f"  速度: {bytes_written / elapsed_time:.1f} 字节/秒")
        
        if failed_chunks:
            print(f"\n✗ 失败的块: {failed_chunks}")
            return False
        else:
            print("\n✓ 固件烧录成功!")
            return True


# ============================================================================
# 主程序
# ============================================================================

def main():
    print("="*70)
    print("STM32 固件烧录工具")
    print("="*70)
    
    # 配置参数
    SERIAL_PORT = "COM5"  # 修改为你的串口号
    BAUDRATE = 115200
    FIRMWARE_PATH = "app/MDK-ARM/app/app.bin"
    
    # 检查固件文件
    if not os.path.exists(FIRMWARE_PATH):
        print(f"\n✗ 固件文件不存在: {FIRMWARE_PATH}")
        print("请确保路径正确，或修改脚本中的 FIRMWARE_PATH 变量")
        return 1
    
    # 读取固件
    with open(FIRMWARE_PATH, 'rb') as f:
        firmware_data = f.read()
    
    print(f"\n✓ 已加载固件: {FIRMWARE_PATH}")
    print(f"  文件大小: {len(firmware_data)} 字节 ({len(firmware_data)/1024:.2f} KB)")
    
    if len(firmware_data) > APP_AREA_SIZE:
        print(f"\n✗ 固件太大! 最大支持 {APP_AREA_SIZE} 字节 ({APP_AREA_SIZE/1024} KB)")
        return 1
    
    # 创建烧录器
    flasher = STM32Flasher(SERIAL_PORT, BAUDRATE)
    
    try:
        # 1. 连接串口
        if not flasher.connect():
            return 1
        
        # 2. 发送心跳，检测运行模式
        if not flasher.send_heartbeat():
            print("\n✗ 设备无响应，请检查连接")
            return 1
        
        # 3. 如果不在Bootloader模式，跳转过去
        if flasher.current_mode != MODE_BOOTLOADER:
            print(f"\n当前处于 Application 模式，需要跳转到 Bootloader")
            if not flasher.jump_to_bootloader():
                print("✗ 跳转失败")
                return 1
            
            # 跳转后再次心跳确认
            time.sleep(0.5)
            if not flasher.send_heartbeat():
                print("✗ 跳转后设备无响应")
                return 1
            
            if flasher.current_mode != MODE_BOOTLOADER:
                print("✗ 仍未进入Bootloader模式")
                return 1
        
        print("\n✓ 设备处于Bootloader模式，可以开始烧录")
        
        # 4. 擦除应用区域 (48页，从0x08004000开始)
        if not flasher.erase_flash(APP_START_ADDRESS, TOTAL_APP_PAGES):
            print("\n✗ 擦除失败")
            return 1
        
        # 5. 烧录固件
        if not flasher.flash_firmware(firmware_data, APP_START_ADDRESS):
            print("\n✗ 烧录失败")
            return 1
        
        print("\n" + "="*70)
        print("✓ 全部完成! 固件已成功烧录到 0x08004000")
        print("="*70)
        print("\n提示: 你可以发送跳转命令让设备运行新固件，或手动复位设备。")
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
        return 1
    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        flasher.disconnect()


if __name__ == "__main__":
    exit(main())
