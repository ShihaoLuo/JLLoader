# STM32 Bootloader & App 串口通信协议规范

## 1. 概述

本协议为上位机 (PC) 与 STM32 设备 (MCU) 之间的通信提供了一套标准化的二进制数据交换格式。该协议旨在替代低效的文本字符串通信，确保数据传输的高效性、可靠性和可解析性。

Bootloader 和 Application 固件均遵循此协议，实现了统一的通信接口。

## 2. 数据帧格式

所有数据交换都基于一个统一的帧结构。该结构为变长格式，最大数据负载为252字节。

```
+--------+--------+--------+--------+------------------------+----------+
| HEADER | LENGTH |  TYPE  | STATUS |   DATA (0-252字节)     | CHECKSUM |
+--------+--------+--------+--------+------------------------+----------+
| 1字节  | 1字节  | 1字节  | 1字节  |      可变长度          |  1字节   |
+--------+--------+--------+--------+------------------------+----------+
```

### 字段详解

-   **`HEADER` (1字节):** 帧起始标识符，用于识别数据方向。
    -   `0xAA`: PC → MCU (上位机发送给设备)
    -   `0xBB`: MCU → PC (设备回复给上位机)

-   **`LENGTH` (1字节):** `DATA` 字段的实际字节长度。该值不包括 `HEADER`, `LENGTH`, `TYPE`, `STATUS`, `CHECKSUM` 本身。取值范围为 `0` 到 `252`。

-   **`TYPE` (1字节):** 消息类型，用于定义该帧的功能或意图。详见 [4. 消息类型 (TYPE)](#4-消息类型-type)。

-   **`STATUS` (1字节):** 状态码，用于表示操作结果、设备当前状态或错误类型。详见 [5. 状态码 (STATUS)](#5-状态码-status)。

-   **`DATA` (0-252字节):** 可变长度的数据负载。其具体结构和内容由 `TYPE` 字段决定。当 `LENGTH` 为 `0` 时，此字段不存在。详见 [6. 数据结构定义](#6-数据结构定义)。

-   **`CHECKSUM` (1字节):** 校验和，用于验证数据帧的完整性，防止传输错误。

## 3. 校验和计算

校验和是确保通信可靠性的核心机制。

-   **计算公式:**
    ```
    CHECKSUM = ~(LENGTH + TYPE + STATUS + 每个DATA字节)
    ```

-   **计算步骤:**
    1.  将 `LENGTH`, `TYPE`, `STATUS` 以及 `DATA` 字段中的所有字节逐一相加。
    2.  对累加和的结果取低8位（即模256）。
    3.  将此8位结果按位取反。

-   **重要提示:** `HEADER` 字段 **不参与** 校验和的计算。

## 4. 消息类型 (TYPE)

`TYPE` 字段定义了帧的用途。

| 值 | 名称 | 方向 | 描述 |
| :--- | :--- | :--- | :--- |
| `0x10` | `CMD_GET_INFO` | PC → MCU | 请求获取设备的系统信息。 |
| `0x11` | `CMD_GET_MEMORY` | PC → MCU | 请求获取设备的内存布局信息。 |
| `0x12` | `CMD_HEARTBEAT` | PC → MCU | 心跳包，用于确认设备是否在线及获取基本状态。 |
| `0x13` | `CMD_JUMP_TO_MODE` | PC → MCU | 命令设备跳转到指定运行模式（如从Bootloader到App）。 |
| `0x01` | `CMD_SYSTEM_INFO` | MCU → PC | 对 `CMD_GET_INFO` 的响应，返回详细系统信息。 |
| `0x02` | `CMD_MEMORY_INFO` | MCU → PC | 对 `CMD_GET_MEMORY` 的响应，返回内存布局。 |
| `0x03` | `CMD_STATUS_REPORT` | MCU → PC | 对 `CMD_HEARTBEAT` 的响应，报告设备当前状态。 |
| `0x04` | `CMD_ERROR_REPORT` | MCU → PC | 当发生错误时（如校验和错误、命令无效），设备主动发送此报告。 |
| `0x05` | `CMD_JUMP_RESPONSE` | MCU → PC | 对 `CMD_JUMP_TO_MODE` 的响应，报告模式跳转的结果。 |

## 5. 状态码 (STATUS)

`STATUS` 字段提供了关于操作结果或设备状态的详细信息。

### 系统与错误状态

| 值 | 名称 | 描述 |
| :--- | :--- | :--- |
| `0x00` | `STATUS_OK` | 操作成功完成。 |
| `0x01` | `STATUS_READY` | 系统已就绪，可以接收命令。 |
| `0x02` | `STATUS_BUSY` | 系统正忙，暂时无法处理新命令。 |
| `0x03` | `STATUS_IDLE` | 系统处于空闲状态。 |
| `0x10` | `ERROR_INVALID_CMD` | 接收到无法识别的无效命令。 |
| `0x11` | `ERROR_CHECKSUM` | 接收到的数据帧校验和错误。 |
| `0x12` | `ERROR_LENGTH` | 数据帧的长度字段与实际负载不符。 |
| `0x13` | `ERROR_TIMEOUT` | 操作超时。 |

### 运行模式状态

| 值 | 名称 | 描述 |
| :--- | :--- | :--- |
| `0x70` | `MODE_BOOTLOADER` | 设备当前运行在Bootloader模式。 |
| `0x71` | `MODE_APPLICATION` | 设备当前运行在Application模式。 |
| `0x72` | `MODE_SYSTEM_MEMORY` | 设备当前运行在系统存储器模式（用于ISP）。 |
| `0x73` | `MODE_UNKNOWN` | 未知的运行模式。 |

## 6. 数据结构定义

`DATA` 字段的具体内容取决于 `TYPE`。所有多字节数据均采用**小端模式** (Little-Endian)。

### `CMD_SYSTEM_INFO` (0x01)

-   **描述:** 包含设备的核心硬件和固件信息。
-   **结构体 (`Protocol_SystemInfo_t`):**
    ```c
    struct {
        uint32_t firmware_version;  // 固件版本 (e.g., 0x00010000 for v1.0.0)
        uint32_t build_timestamp;   // 固件构建时间戳的哈希值
        uint32_t cpu_frequency;     // CPU 频率 (Hz)
        uint16_t device_id;         // MCU 设备ID
        uint8_t  reset_reason;      // 上次复位原因
        uint8_t  boot_mode;         // 启动模式 (Bootloader/App)
        uint8_t  running_mode;      // 当前运行模式
        uint8_t  reserved;          // 保留字节
    } __attribute__((packed));
    ```

### `CMD_STATUS_REPORT` (0x03)

-   **描述:** 包含设备运行时的动态状态。
-   **结构体 (`Protocol_StatusReport_t`):**
    ```c
    struct {
        uint32_t uptime_ms;         // 设备启动后的运行时间 (毫秒)
        uint16_t last_command;      // 上一个成功执行的命令类型
        uint8_t  system_status;     // 当前系统状态
        uint8_t  error_count;       // 累计错误计数
        uint8_t  running_mode;      // 当前运行模式
        uint8_t  reserved[3];       // 保留字节
    } __attribute__((packed));
    ```

### `CMD_JUMP_TO_MODE` (0x13)

-   **描述:** 上位机发起模式跳转时携带的参数。
-   **结构体 (`Protocol_JumpModeRequest_t`):**
    ```c
    struct {
        uint8_t  target_mode;       // 目标模式 (e.g., 0x71 for Application)
        uint8_t  jump_delay_ms;     // 延迟跳转时间 (毫秒)
        uint16_t timeout_ms;        // 跳转超时时间 (毫秒)
        uint32_t magic_word;        // 安全验证幻数 (固定为 0x12345678)
    } __attribute__((packed));
    ```

## 7. 通信流程示例

### 示例：心跳检测

1.  **PC → MCU:** 上位机发送一个心跳包以确认设备状态。
    -   **Frame:** `AA 00 12 00 ED`
    -   **解析:**
        -   `AA`: PC发往MCU。
        -   `00`: 数据长度为0。
        -   `12`: 命令类型为 `CMD_HEARTBEAT`。
        -   `00`: 状态码，请求时通常为0。
        -   `ED`: 校验和 `~(0x00 + 0x12 + 0x00)`。

2.  **MCU → PC:** 设备收到心跳包后，回复一个状态报告。
    -   **Frame (示例):** `BB 0C 03 03 D0 0F 00 00 12 00 03 00 70 00 00 00 C4`
    -   **解析:**
        -   `BB`: MCU回复给PC。
        -   `0C`: 数据长度为12字节。
        -   `03`: 消息类型为 `CMD_STATUS_REPORT`。
        -   `03`: 设备状态为 `STATUS_IDLE` (空闲)。
        -   `D0 0F 00 00...`: 12字节的 `Protocol_StatusReport_t` 数据。
        -   `C4`: 根据前面所有字段计算出的正确校验和。
