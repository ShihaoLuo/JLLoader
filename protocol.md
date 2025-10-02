# STM32 Bootloader Serial Communication Protocol

## 协议概述
本协议设计用于STM32F103C8T6 Bootloader与上位机之间的二进制通信，替代文本字符串输出，提供高效、可解析的数据交换。

## 数据帧格式
```
+--------+--------+--------+--------+--------+--------+--------+--------+
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
| 0x05 | CMD_JUMP_RESPONSE    | MCU → Host | 模式跳转响应           |
| 0x10 | CMD_GET_INFO         | Host → MCU | 获取系统信息命令       |
| 0x11 | CMD_GET_MEMORY       | Host → MCU | 获取内存信息命令       |
| 0x12 | CMD_HEARTBEAT        | Host → MCU | 心跳包                 |
| 0x13 | CMD_JUMP_TO_MODE     | Host → MCU | 模式跳转命令           |

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

### 模式状态 (0x50-0x6F)
| 值   | 名称                    | 描述                           |
|------|-------------------------|--------------------------------|
| 0x50 | MODE_BOOTLOADER         | 当前在Bootloader模式           |
| 0x51 | MODE_APPLICATION        | 当前在Application模式          |
| 0x52 | MODE_JUMP_REQUESTED     | 模式跳转请求已接收             |
| 0x53 | MODE_JUMP_PREPARING     | 正在准备模式跳转               |
| 0x54 | MODE_JUMP_SUCCESS       | 模式跳转成功                   |
| 0x55 | MODE_JUMP_FAILED        | 模式跳转失败                   |
| 0x56 | MODE_INVALID_TARGET     | 无效的目标模式                 |
| 0x57 | MODE_JUMP_TIMEOUT       | 模式跳转超时                   |
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

### 转换状态 (0x90-0x9F)
| 值   | 名称                     | 描述                           |
|------|--------------------------|--------------------------------|
| 0x90 | STATUS_JUMPING_TO_APP    | 正在跳转到应用程序             |

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

### 模式跳转请求数据 (CMD_JUMP_TO_MODE)
```c
struct JumpModeRequest {
    uint8_t  target_mode;        // 目标模式 (0x70=Bootloader, 0x71=Application)
    uint8_t  jump_delay_ms;      // 跳转延迟时间 (毫秒)
    uint16_t timeout_ms;         // 跳转超时时间 (毫秒)
    uint32_t magic_word;         // 魔法字 (0x12345678) 安全验证
};
```

### 模式跳转响应数据 (CMD_JUMP_RESPONSE)
```c
struct JumpModeResponse {
    uint8_t  current_mode;       // 当前模式
    uint8_t  target_mode;        // 目标模式
    uint8_t  jump_status;        // 跳转状态 (0x52-0x57)
    uint8_t  result_code;        // 结果代码
    uint32_t uptime_before_jump; // 跳转前运行时间
    uint16_t estimated_jump_time;// 预计跳转时间 (毫秒)
    uint8_t  reserved[2];        // 保留字节
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

### 4. 模式跳转
#### 从Application跳转到Bootloader
**主机发送:**
```
AA 08 13 00 70 64 E8 03 78 56 34 12 [CS]
```
- LENGTH: 0x08 (8字节数据)
- TYPE: 0x13 (CMD_JUMP_TO_MODE)
- STATUS: 0x00 (STATUS_OK)
- DATA: 
  - 0x70: target_mode (MODE_BOOTLOADER)
  - 0x64: jump_delay_ms (100ms延迟)
  - 0x03E8: timeout_ms (1000ms超时)
  - 0x12345678: magic_word (安全验证)

**MCU响应 (跳转前确认):**
```
BB 0C 05 52 [12字节跳转响应数据] CS
```
- LENGTH: 0x0C (12字节数据)
- TYPE: 0x05 (CMD_JUMP_RESPONSE)
- STATUS: 0x52 (MODE_JUMP_REQUESTED)
- DATA:
  - current_mode: 0x71 (Application)
  - target_mode: 0x70 (Bootloader)
  - jump_status: 0x52 (MODE_JUMP_REQUESTED)
  - result_code: 0x00 (准备就绪)
  - uptime_before_jump: 当前运行时间
  - estimated_jump_time: 预计跳转时间
  - reserved[2]: 保留字节

#### 从Bootloader跳转到Application
**主机发送:**
```
AA 08 13 00 71 32 D0 07 78 56 34 12 [CS]
```
- DATA:
  - 0x71: target_mode (MODE_APPLICATION)
  - 0x32: jump_delay_ms (50ms延迟)
  - 0x07D0: timeout_ms (2000ms超时)
  - 0x12345678: magic_word

#### 错误情况处理
**无效目标模式:**
```
BB 0C 05 56 [12字节跳转响应数据] CS
```
- STATUS: 0x56 (MODE_INVALID_TARGET)

**魔法字错误:**
```
BB 00 04 12 [CS]
```
- TYPE: 0x04 (CMD_ERROR_REPORT)
- STATUS: 0x12 (ERROR_LENGTH或校验错误)

**跳转超时:**
```
BB 0C 05 57 [12字节跳转响应数据] CS
```
- STATUS: 0x57 (MODE_JUMP_TIMEOUT)

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
7. 运行模式检测：
   - 通过PC（程序计数器）地址自动检测当前运行模式
   - Bootloader模式：PC在0x08000000-0x08003FFF范围内
   - Application模式：PC在0x08004000-0x0800FFFF范围内
   - System Memory模式：PC在0x1FFFF000-0x1FFFF7FF范围内
8. 校验和计算：LENGTH + TYPE + STATUS + DATA各字节之和的低8位取反
9. 帧长度限制：最小5字节，最大257字节
10. 错误处理：根据不同错误类型返回相应状态码

## 模式跳转实现规范

### 跳转时序要求
1. **跳转确认阶段 (0-100ms)**
   - 接收跳转命令后立即发送确认响应
   - 验证魔法字(0x12345678)和目标模式有效性
   - 保存当前系统状态和运行时间

2. **跳转准备阶段 (100ms-延迟时间)**
   - 关闭所有外设中断
   - 清理UART缓冲区
   - 保存必要的跳转信息到特定内存区域
   - 发送最后的状态更新(可选)

3. **跳转执行阶段 (延迟时间后)**
   - 重置系统时钟配置
   - 设置新的向量表地址
   - 执行跳转指令

### 跳转安全机制
1. **魔法字验证**
   - 固定值0x12345678防止意外跳转
   - 魔法字错误时拒绝跳转并返回错误

2. **目标有效性检查**
   - Bootloader→Application: 检查Application起始地址是否有效
   - Application→Bootloader: 直接允许(Bootloader总是存在)
   - 无效目标时返回MODE_INVALID_TARGET状态

3. **超时保护**
   - 跳转过程中设置看门狗超时
   - 异常情况下自动重启到安全模式

### 跳转状态持久化
```c
// 特殊内存区域用于跳转信息传递 (0x20000010-0x2000001F)
// 使用RAM起始地址附近的安全区域，避免与栈和其他数据冲突
struct JumpInfo {
    uint32_t magic;              // 0xDEADBEEF (跳转标记)
    uint8_t  source_mode;        // 跳转前模式
    uint8_t  target_mode;        // 目标模式
    uint8_t  jump_reason;        // 跳转原因
    uint8_t  reserved;           // 保留
    uint32_t timestamp;          // 跳转时间戳
};
```

### 模式检测机制
```c
typedef enum {
    MODE_UNKNOWN      = 0x70,    // 未知模式
    MODE_BOOTLOADER   = 0x70,    // Bootloader模式  
    MODE_APPLICATION  = 0x71,    // Application模式
    MODE_SYSTEM_MEM   = 0x72,    // 系统内存模式
    MODE_ERROR        = 0x73     // 错误模式
} SystemMode_t;

// 模式检测函数
SystemMode_t GetCurrentMode(void) {
    uint32_t pc = __get_MSP();  // 或其他方式获取PC
    
    if (pc >= 0x08000000 && pc < 0x08004000) {
        return MODE_BOOTLOADER;
    } else if (pc >= 0x08004000 && pc < 0x08010000) {
        return MODE_APPLICATION;
    } else if (pc >= 0x1FFFF000 && pc < 0x1FFFF800) {
        return MODE_SYSTEM_MEM;
    } else {
        return MODE_ERROR;
    }
}
```

### 跳转实现建议
1. **Bootloader中实现**
   - 支持跳转到Application
   - 检查Application有效性(如栈指针、复位向量)
   - 提供安全的跳转函数

2. **Application中实现**
   - 支持跳转回Bootloader
   - 保存应用状态(可选)
   - 使用软件复位或直接跳转

3. **错误恢复**
   - 跳转失败时自动重启到Bootloader
   - 记录跳转失败原因
   - 提供故障诊断信息

### ⚠️ 关键实现注意事项 - 避免UART阻塞问题

**问题描述：**
在串口跳转过程中，如果在跳转函数内调用阻塞的UART发送函数（如`HAL_UART_Transmit`），可能导致系统挂起，无法完成跳转。

**根本原因：**
1. `HAL_UART_Transmit` 是阻塞函数，依赖系统时钟和中断状态
2. 跳转过程中系统状态复杂，可能导致UART发送超时或死锁
3. 从中断上下文调用跳转时，中断嵌套可能导致系统不稳定

**解决方案：**

**❌ 错误做法：**
```c
void Bootloader_JumpToApplication(uint32_t app_address) {
    // 验证应用程序
    if (!Bootloader_CheckApplication(app_address)) return;
    
    // ❌ 在跳转前发送状态 - 可能导致阻塞
    Protocol_SendStatus(STATUS_JUMPING_TO_APP);
    HAL_Delay(10);  // ❌ 等待发送完成 - 可能挂起
    
    // 执行跳转
    Bootloader_PrepareJump();
    // ...跳转代码
}
```

**✅ 正确做法：**
```c
void Bootloader_JumpToApplication(uint32_t app_address) {
    // 验证应用程序
    if (!Bootloader_CheckApplication(app_address)) return;
    
    // ✅ 跳转前不发送任何UART数据
    // 跳转操作应该尽可能简洁，避免任何可能阻塞的I/O操作
    
    // 立即执行跳转
    Bootloader_PrepareJump();
    SCB->VTOR = app_address;
    __set_MSP(*((volatile uint32_t*)app_address));
    // ...继续跳转
}
```

**实现规范：**
1. **跳转确认与状态发送分离**
   ```c
   // 在跳转命令处理中立即发送确认
   bool Protocol_ProcessJumpRequest(const Protocol_JumpModeRequest_t* request) {
       // 验证请求
       if (request->magic_word != JUMP_MAGIC_WORD) return false;
       
       // ✅ 立即发送确认响应
       Protocol_SendJumpResponse(&response);
       
       // ✅ 设置延迟跳转标志，不立即跳转
       jump_request_pending = true;
       jump_target_mode = request->target_mode;
       jump_delay_ms = request->jump_delay_ms;
       jump_request_time = HAL_GetTick();
       
       return true;
   }
   ```

2. **主循环中执行跳转**
   ```c
   // 在主循环中检查并执行跳转
   void Protocol_CheckPendingJump(void) {
       if (jump_request_pending) {
           uint32_t current_time = HAL_GetTick();
           if ((current_time - jump_request_time) >= jump_delay_ms) {
               jump_request_pending = false;
               
               // ✅ 从主循环上下文执行跳转，避免中断冲突
               if (jump_target_mode == MODE_APPLICATION) {
                   Bootloader_JumpToApplication(APPLICATION_ADDRESS);
               }
           }
       }
   }
   ```

3. **中断上下文限制**
   ```c
   // ❌ 永远不要从中断上下文直接跳转
   void UART_RxCpltCallback(UART_HandleTypeDef *huart) {
       // 处理数据...
       if (jump_command_received) {
           // ❌ 不要在这里直接调用跳转函数
           // Bootloader_JumpToApplication(app_addr);  // 危险！
           
           // ✅ 设置标志，让主循环处理
           jump_request_pending = true;
       }
   }
   ```

**测试验证：**
- ✅ **10秒自动跳转**：直接从主循环调用，无UART发送，工作正常
- ❌ **串口跳转v1**：跳转前发送状态消息，在`HAL_UART_Transmit`后挂起
- ✅ **串口跳转v2**：移除状态发送，使用延迟执行机制，工作正常

**关键要点：**
1. 跳转函数应该是"纯净"的，不包含任何I/O操作
2. 状态通知和跳转执行必须分离
3. 使用标志位和主循环实现延迟跳转
4. 避免从中断上下文执行系统级操作

### 典型使用场景
1. **固件升级**: Application → Bootloader → 升级完成 → Application
2. **系统维护**: Application → Bootloader → 诊断/配置 → Application  
3. **紧急恢复**: Application故障 → 自动跳转Bootloader → 恢复操作
4. **功能切换**: 根据用户需求在不同模式间切换

## 串口跳转完整实现流程

### 实现架构
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   UART中断处理   │───▶│   协议解析处理   │───▶│   主循环跳转     │
│                │    │                │    │                │
│ 接收跳转命令     │    │ 验证并设置标志   │    │ 执行安全跳转     │
│ 触发协议处理     │    │ 发送确认响应     │    │ 清理系统状态     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### 详细实现步骤

#### 第一阶段：命令接收与验证
```c
// 在UART中断或协议处理函数中
bool Protocol_ProcessJumpRequest(const Protocol_JumpModeRequest_t* request) {
    // 1. 验证魔法字
    if (request->magic_word != 0x12345678) {
        Protocol_SendError(ERROR_INVALID_CMD);
        return false;
    }
    
    // 2. 验证目标模式
    if (request->target_mode != MODE_APPLICATION && 
        request->target_mode != MODE_BOOTLOADER) {
        Protocol_SendJumpResponse_Error(MODE_INVALID_TARGET);
        return false;
    }
    
    // 3. 立即发送确认响应（在设置跳转标志前）
    Protocol_JumpModeResponse_t response = {
        .current_mode = GetCurrentMode(),
        .target_mode = request->target_mode,
        .jump_status = MODE_JUMP_REQUESTED,
        .result_code = 0x00,
        .uptime_before_jump = HAL_GetTick(),
        .estimated_jump_time = request->jump_delay_ms,
        .reserved = {0, 0}
    };
    Protocol_SendJumpResponse(&response);
    
    // 4. 设置跳转标志（关键：在发送响应后设置）
    jump_request_pending = true;
    jump_target_mode = request->target_mode;
    jump_delay_ms = request->jump_delay_ms;
    jump_request_time = HAL_GetTick();
    
    return true;
}
```

#### 第二阶段：主循环跳转检查
```c
// 在main()函数的主循环中调用
void Protocol_CheckPendingJump(void) {
    if (jump_request_pending) {
        uint32_t current_time = HAL_GetTick();
        
        // 检查是否到了跳转时间
        if ((current_time - jump_request_time) >= jump_delay_ms) {
            jump_request_pending = false; // 清除标志位
            
            if (jump_target_mode == MODE_APPLICATION) {
                // 在主循环中执行跳转，安全无中断冲突
                Bootloader_JumpToApplication(APPLICATION_ADDRESS);
            }
            // 注意：如果跳转成功，此函数不会返回
        }
    }
}
```

#### 第三阶段：安全跳转执行
```c
// 简化的跳转函数，移除所有可能阻塞的操作
void Bootloader_JumpToApplication(uint32_t app_address) {
    uint32_t jump_address;
    pFunction jump_to_application;
    
    // 1. 验证应用程序有效性
    if (!Bootloader_CheckApplication(app_address)) {
        return; // 无效应用程序，静默返回
    }
    
    // 2. 注意：不发送任何UART状态消息
    // 跳转操作应该尽可能简洁，避免阻塞
    
    // 3. 准备系统跳转
    Bootloader_PrepareJump();
    
    // 4. 重映射中断向量表
    SCB->VTOR = app_address;
    
    // 5. 设置新的栈指针
    __set_MSP(*((volatile uint32_t*)app_address));
    
    // 6. 获取复位处理程序地址
    jump_address = *((volatile uint32_t*)(app_address + 4));
    jump_to_application = (pFunction)jump_address;
    
    // 7. 执行跳转（此处不应返回）
    jump_to_application();
}
```

### 主循环集成示例
```c
int main(void) {
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_UART1_Init();
    
    // 启动UART接收
    UART_StartReceive();
    
    // 主循环
    while (1) {
        // 1. 处理UART接收到的数据
        Protocol_ProcessReceiveBuffer();
        
        // 2. 检查并执行待处理的跳转（关键！）
        Protocol_CheckPendingJump();
        
        // 3. 其他系统任务
        // ...
        
        // 4. 简短延迟
        HAL_Delay(1);
    }
}
```

### 关键文件修改总结

#### Core/Inc/protocol.h
```c
// 添加跳转标志变量声明
extern volatile bool jump_request_pending;
extern volatile uint8_t jump_target_mode;
extern volatile uint32_t jump_request_time;
extern volatile uint32_t jump_delay_ms;

// 添加函数声明
void Protocol_CheckPendingJump(void);
bool Protocol_ProcessJumpRequest(const Protocol_JumpModeRequest_t* request);
```

#### Core/Src/protocol.c
```c
// 跳转标志变量定义
static volatile bool jump_request_pending = false;
static volatile uint8_t jump_target_mode = 0;
static volatile uint32_t jump_request_time = 0;
static volatile uint32_t jump_delay_ms = 0;

// 实现跳转检查函数和处理函数
// （如上面代码所示）
```

#### Core/Src/bootloader_jump.c
```c
// 移除跳转函数中的UART发送代码
void Bootloader_JumpToApplication(uint32_t app_address) {
    // 验证应用程序
    if (!Bootloader_CheckApplication(app_address)) return;
    
    // 移除：Protocol_SendStatus(STATUS_JUMPING_TO_APP);
    // 移除：HAL_Delay(10);
    
    // 直接执行跳转
    Bootloader_PrepareJump();
    // ...跳转代码
}
```

#### Core/Src/main.c
```c
int main(void) {
    // 初始化...
    
    while (1) {
        Protocol_ProcessReceiveBuffer();
        Protocol_CheckPendingJump();  // 添加这行
        // 其他任务...
    }
}
```

### 测试验证
1. **命令:** `AA 08 13 00 71 32 D0 07 78 56 34 12 [CS]`
2. **预期响应:** `BB 0C 05 52 ...` (跳转确认)
3. **跳转结果:** 成功跳转到Application，收到Application的协议响应
4. **关键指标:** 
   - 跳转延迟：50ms (0x32)
   - 无UART阻塞问题
   - 系统状态切换正常

### 故障排除
1. **跳转无响应**: 检查主循环是否调用`Protocol_CheckPendingJump()`
2. **跳转失败**: 验证APPLICATION_ADDRESS和应用程序有效性
3. **系统挂起**: 确认跳转函数中无UART发送操作
4. **协议错误**: 验证魔法字(0x12345678)和校验和计算