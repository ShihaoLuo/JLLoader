# STM32 Bootloader Serial Communication Protocol

## 协议概述
本协议设计用于STM32F103C8T6 Bootloader与上位机之间的二进制通信，替代文本字符串输出，提供高效、可解析的数据交换。

## 数据帧格式
```
+--------+--------+--------+-6. 心跳包间隔建议1秒
7. 协议头区分：
   - 接收到0xAA开头的帧：PC发送给MCU的命令
   - 发送0xBB开头的帧：MCU发送给PC的响应
   - 这种设计便于协议解析和方向识别
8. 运行模式检测：
   - 通过PC（程序计数器）地址自动检测当前运行模式
   - Bootloader模式：PC在0x08000000-0x080027FF范围内
   - Application模式：PC在0x08002800-0x0800FFFF范围内
   - System Memory模式：PC在0x1FFFF000-0x1FFFF7FF范围内--+--------+--------+--------+--------+
| HEADER | LENGTH |  TYPE  | STATUS |   DATA (0-252 bytes)   | CHECKSUM |
+--------+--------+--------+--------+--------+--------+--------+--------+
```

### 帧结构说明
- **HEADER** (1 byte): 帧起始标识
  - 0xAA: PC → MCU (主机发送给MCU)
  - 0xBB: MCU → PC (MCU发送给主机)
- **LENGTH** (1 byte): 数据长度 (0-252)，不包括帧头、长度、类型、状态和校验和
- **TYPE** (1 byte): 消息类型
- **STATUS** (1 byte): 状态码
- **DATA** (0-252 bytes): 可变长度数据
- **CHECKSUM** (1 byte): 校验和 (LENGTH + TYPE + STATUS + DATA各字节之和的低8位取反)

## 消息类型 (TYPE)

| 值   | 名称                  | 方向       | 描述                   |
|------|-----------------------|------------|------------------------|
| 0x01 | CMD_SYSTEM_INFO      | MCU → Host | 系统信息响应           |
| 0x02 | CMD_MEMORY_INFO      | MCU → Host | 内存布局信息           |
| 0x03 | CMD_STATUS_REPORT    | MCU → Host | 状态报告               |
| 0x04 | CMD_ERROR_REPORT     | MCU → Host | 错误报告               |
| 0x10 | CMD_GET_INFO         | Host → MCU | 获取系统信息命令       |
| 0x11 | CMD_GET_MEMORY       | Host → MCU | 获取内存信息命令       |
| 0x12 | CMD_HEARTBEAT        | Host → MCU | 心跳包                 |

## 状态码 (STATUS)

### 系统状态 (0x00-0x0F)
| 值   | 名称               | 描述                           |
|------|--------------------|--------------------------------|
| 0x00 | STATUS_OK          | 操作成功                       |
| 0x01 | STATUS_READY       | 系统就绪                       |
| 0x02 | STATUS_BUSY        | 系统忙碌                       |
| 0x03 | STATUS_IDLE        | 系统空闲                       |

### 错误状态 (0x10-0x2F)
| 值   | 名称               | 描述                           |
|------|--------------------|--------------------------------|
| 0x10 | ERROR_INVALID_CMD  | 无效命令                       |
| 0x11 | ERROR_CHECKSUM     | 校验和错误                     |
| 0x12 | ERROR_LENGTH       | 长度错误                       |
| 0x13 | ERROR_TIMEOUT      | 超时错误                       |

### 内存状态 (0x30-0x4F)
| 值   | 名称               | 描述                           |
|------|--------------------|--------------------------------|
| 0x30 | MEM_BOOTLOADER_OK  | Bootloader区域正常             |
| 0x31 | MEM_APP_VALID      | 应用程序有效                   |
| 0x32 | MEM_APP_INVALID    | 应用程序无效                   |
| 0x33 | MEM_CONSTRAINT_OK  | 内存约束检查通过               |
| 0x34 | MEM_CONSTRAINT_ERR | 内存约束检查失败               |

### 硬件状态 (0x50-0x6F)
| 值   | 名称               | 描述                           |
|------|--------------------|--------------------------------|
| 0x50 | HW_CLOCK_OK        | 时钟配置正常                   |
| 0x51 | HW_UART_OK         | UART配置正常                   |
| 0x52 | HW_GPIO_OK         | GPIO配置正常                   |
| 0x53 | HW_TIMER_OK        | 定时器配置正常                 |

### 运行模式状态 (0x70-0x8F)
| 值   | 名称               | 描述                           |
|------|--------------------|--------------------------------|
| 0x70 | MODE_BOOTLOADER    | 当前运行在Bootloader模式       |
| 0x71 | MODE_APPLICATION   | 当前运行在Application模式      |
| 0x72 | MODE_SYSTEM_MEMORY | 当前运行在系统存储器模式       |
| 0x73 | MODE_UNKNOWN       | 未知运行模式                   |

## 数据结构定义

### 系统信息数据 (CMD_SYSTEM_INFO)
```c
struct SystemInfo {
    uint32_t firmware_version;    // 固件版本号
    uint32_t build_timestamp;     // 编译时间戳
    uint32_t cpu_frequency;       // CPU频率 (Hz)
    uint16_t device_id;          // 设备ID
    uint8_t  reset_reason;       // 复位原因
    uint8_t  boot_mode;          // 启动模式 (0x01=Bootloader, 0x02=Application)
    uint8_t  running_mode;       // 当前运行模式 (0x70-0x73)
    uint8_t  reserved;           // 保留字节，保持4字节对齐
};
```

### 内存信息数据 (CMD_MEMORY_INFO)
```c
struct MemoryInfo {
    uint32_t bootloader_start;    // Bootloader起始地址
    uint32_t bootloader_size;     // Bootloader大小
    uint32_t application_start;   // 应用程序起始地址
    uint32_t application_size;    // 应用程序最大大小
    uint32_t ram_start;          // RAM起始地址
    uint32_t ram_size;           // RAM大小
    uint16_t page_size;          // Flash页大小
    uint8_t  page_count;         // Bootloader页数
    uint8_t  reserved;           // 保留字节
};
```

### 状态报告数据 (CMD_STATUS_REPORT)
```c
struct StatusReport {
    uint32_t uptime_ms;          // 运行时间 (毫秒)
    uint16_t last_command;       // 最后执行的命令
    uint8_t  system_status;      // 系统状态
    uint8_t  error_count;        // 错误计数
    uint8_t  running_mode;       // 当前运行模式 (0x70-0x73)
    uint8_t  reserved[3];        // 保留字节，保持4字节对齐
};
```

## 命令交互示例

### 1. 获取系统信息
**主机发送:**
```
AA 00 10 00 EF
```
- HEADER: 0xAA (PC → MCU)
- LENGTH: 0x00 (无数据)
- TYPE: 0x10 (CMD_GET_INFO)
- STATUS: 0x00 (STATUS_OK)
- CHECKSUM: 0xEF (0x100 - 0x10 - 0x00 = 0xF0, 取反 = 0xEF)

**MCU响应:**
```
BB 14 01 00 [20字节系统信息数据] CS
```
- HEADER: 0xBB (MCU → PC)
- LENGTH: 0x14 (20字节数据，包含新的running_mode字段)
- TYPE: 0x01 (CMD_SYSTEM_INFO)
- 数据包含: firmware_version, build_timestamp, cpu_frequency, device_id, reset_reason, boot_mode, running_mode, reserved

### 2. 获取内存信息
**主机发送:**
```
AA 00 11 00 EE
```

**MCU响应:**
```
BB 18 02 30 [24字节内存信息数据] CS
```

### 3. 心跳包
**主机发送:**
```
AA 00 12 00 ED
```

**MCU响应:**
```
BB 0C 03 01 [12字节状态报告数据] CS
```
- LENGTH: 0x0C (12字节数据，包含新的running_mode字段)
- 数据包含: uptime_ms, last_command, system_status, error_count, running_mode, reserved[3]

## 错误处理
1. 校验和错误：返回ERROR_CHECKSUM状态
2. 无效命令：返回ERROR_INVALID_CMD状态
3. 数据长度错误：返回ERROR_LENGTH状态
4. 超时：返回ERROR_TIMEOUT状态

## 实现注意事项
1. 所有多字节数据使用小端序（Little-Endian）
2. 超时时间建议设置为100ms
3. 重试机制：最多重试3次
4. 缓冲区大小至少256字节
5. 心跳包间隔建议1秒
6. 协议头区分：
   - 接收到0xAA开头的帧：PC发送给MCU的命令
   - 发送0xBB开头的帧：MCU发送给PC的响应
   - 这种设计便于协议解析和方向识别