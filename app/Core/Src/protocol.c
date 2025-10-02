/**
  ******************************************************************************
  * @file    protocol.c
  * @author  Generated
  * @brief   Custom serial communication protocol implementation for Application
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "protocol.h"
#include "uart.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
static uint32_t protocol_uptime_ms = 0;
static uint8_t protocol_error_count = 0;
static uint16_t protocol_last_command = 0;

/* Jump request variables */
static volatile bool jump_request_pending = false;
static volatile uint8_t jump_target_mode = 0;
static volatile uint32_t jump_request_time = 0;
static volatile uint32_t jump_delay_ms = 0;

/* Firmware version and build info */
#define FIRMWARE_VERSION        0x00010000  // v1.0.0
#define BUILD_TIMESTAMP         (__DATE__ __TIME__)

/* Memory layout constants for Application */
#define BOOTLOADER_START_ADDRESS   0x08000000  // Bootloader base address
#define BOOTLOADER_SIZE            0x4000      // 16KB for bootloader  
#define APPLICATION_START_ADDRESS  0x08004000  // Application base address
#define APPLICATION_MAX_SIZE       0xC000      // 48KB for application (64KB - 16KB)
#define APPLICATION_END_ADDRESS    0x0800FFFF  // End of flash
#define BOOTLOADER_END_ADDRESS     0x08003FFF  // End of bootloader area
#define BOOTLOADER_PAGE_COUNT      10          // 10 pages of 1KB each
// FLASH_PAGE_SIZE is defined in stm32f1xx_hal_flash_ex.h as 0x400U (1024 bytes)

/* Private function prototypes -----------------------------------------------*/
static uint32_t Protocol_GetBuildTimestamp(void);
static uint8_t Protocol_GetResetReason(void);

/**
 * @brief Get current running mode based on PC (Program Counter) address
 */
uint8_t Protocol_GetRunningMode(void)
{
    uint32_t pc_address;
    
    // Get current Program Counter (PC) address
    __asm volatile ("mov %0, pc" : "=r" (pc_address));
    
    // Check which memory region the PC is in
    if (pc_address >= BOOTLOADER_START_ADDRESS && pc_address <= BOOTLOADER_END_ADDRESS) {
        return MODE_BOOTLOADER;
    }
    else if (pc_address >= APPLICATION_START_ADDRESS && pc_address <= APPLICATION_END_ADDRESS) {
        return MODE_APPLICATION;  // App应该返回APPLICATION模式
    }
    else if (pc_address >= 0x1FFFF000 && pc_address <= 0x1FFFF7FF) {
        // STM32F103 System Memory range
        return MODE_SYSTEM_MEMORY;
    }
    else {
        return MODE_UNKNOWN;
    }
}

/* Function implementations --------------------------------------------------*/

/**
 * @brief Initialize protocol module
 */
void Protocol_Init(void)
{
    protocol_uptime_ms = 0;
    protocol_error_count = 0;
    protocol_last_command = 0;
}

/**
 * @brief Calculate frame checksum
 */
uint8_t Protocol_CalculateChecksum(const Protocol_Frame_t* frame, uint8_t data_length)
{
    uint16_t sum = 0;
    
    sum += frame->length;
    sum += frame->type;
    sum += frame->status;
    
    for (uint8_t i = 0; i < data_length; i++) {
        sum += frame->data[i];
    }
    
    return (uint8_t)(~sum);  // Invert the low 8 bits
}

/**
 * @brief Verify frame checksum
 */
bool Protocol_VerifyChecksum(const Protocol_Frame_t* frame, uint8_t data_length)
{
    uint8_t calculated = Protocol_CalculateChecksum(frame, data_length);
    return (calculated == frame->checksum);
}

/**
 * @brief Verify frame header for direction
 */
bool Protocol_VerifyHeader(uint8_t header, uint8_t expected_direction)
{
    if (expected_direction == 0) {
        return (header == PROTOCOL_HEADER_PC_TO_MCU);  // Expecting PC→MCU (0xAA)
    } else {
        return (header == PROTOCOL_HEADER_MCU_TO_PC);  // Expecting MCU→PC (0xBB)
    }
}

/**
 * @brief Send protocol frame
 */
bool Protocol_SendFrame(uint8_t type, uint8_t status, const uint8_t* data, uint8_t length)
{
    if (length > PROTOCOL_MAX_DATA_LENGTH) {
        protocol_error_count++;
        return false;
    }

    // 1. 创建一个足够大的缓冲区来手动构建帧
    uint8_t frame_buffer[PROTOCOL_MIN_FRAME_LENGTH + PROTOCOL_MAX_DATA_LENGTH + 1];
    uint16_t frame_size = PROTOCOL_MIN_FRAME_LENGTH + length + 1; // header, length, type, status, data, checksum

    // 2. 填充帧的元数据
    frame_buffer[0] = PROTOCOL_HEADER_MCU_TO_PC;
    frame_buffer[1] = length;
    frame_buffer[2] = type;
    frame_buffer[3] = status;

    // 3. 复制数据负载
    if (data != NULL && length > 0) {
        for (int i = 0; i < length; i++) {
            frame_buffer[PROTOCOL_MIN_FRAME_LENGTH + i] = data[i];
        }
    }

    // 4. 计算校验和
    // 校验和的计算应该只包括 LENGTH, TYPE, STATUS 和 DATA
    uint16_t sum = 0;
    sum += length;
    sum += type;
    sum += status;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    uint8_t checksum = (uint8_t)(~sum);

    // 5. 将校验和放在缓冲区的最后一个字节
    frame_buffer[frame_size - 1] = checksum;

    // 6. 发送整个手动构建的帧
    return UART_SendData(frame_buffer, frame_size);
}

/**
 * @brief Send system information
 */
bool Protocol_SendSystemInfo(void)
{
    Protocol_SystemInfo_t sys_info = {0};
    
    sys_info.firmware_version = FIRMWARE_VERSION;
    sys_info.build_timestamp = Protocol_GetBuildTimestamp();
    sys_info.cpu_frequency = SystemCoreClock;
    sys_info.device_id = HAL_GetDEVID();
    sys_info.reset_reason = Protocol_GetResetReason();
    sys_info.boot_mode = 0x02;  // Application mode (not bootloader)
    sys_info.running_mode = Protocol_GetRunningMode();
    sys_info.reserved = 0;
    
    return Protocol_SendFrame(CMD_SYSTEM_INFO, STATUS_OK, 
                            (const uint8_t*)&sys_info, sizeof(sys_info));
}

/**
 * @brief Send memory information
 */
bool Protocol_SendMemoryInfo(void)
{
    Protocol_MemoryInfo_t mem_info = {0};
    
    mem_info.bootloader_start = BOOTLOADER_START_ADDRESS;
    mem_info.bootloader_size = BOOTLOADER_SIZE;
    mem_info.application_start = APPLICATION_START_ADDRESS;
    mem_info.application_size = APPLICATION_MAX_SIZE;
    mem_info.ram_start = 0x20000000;  // SRAM_BASE for STM32F103C8T6
    mem_info.ram_size = 20 * 1024;    // 20KB for STM32F103C8T6
    mem_info.page_size = FLASH_PAGE_SIZE;
    mem_info.page_count = BOOTLOADER_PAGE_COUNT;
    mem_info.reserved = 0;
    
    uint8_t status = MEM_APP_VALID;  // Application context - app is valid
    
    return Protocol_SendFrame(CMD_MEMORY_INFO, status, 
                            (const uint8_t*)&mem_info, sizeof(mem_info));
}

/**
 * @brief Send status report
 */
bool Protocol_SendStatusReport(uint8_t system_status)
{
    Protocol_StatusReport_t status_report = {0};
    
    status_report.uptime_ms = HAL_GetTick();
    status_report.last_command = protocol_last_command;
    status_report.system_status = system_status;
    status_report.error_count = protocol_error_count;
    status_report.running_mode = Protocol_GetRunningMode();
    status_report.reserved[0] = 0;
    status_report.reserved[1] = 0;
    status_report.reserved[2] = 0;
    
    return Protocol_SendFrame(CMD_STATUS_REPORT, system_status, 
                            (const uint8_t*)&status_report, sizeof(status_report));
}

/**
 * @brief Send error report
 */
bool Protocol_SendErrorReport(uint8_t error_code)
{
    protocol_error_count++;
    uint8_t error_data[2] = {error_code, protocol_error_count};
    
    return Protocol_SendFrame(CMD_ERROR_REPORT, error_code, error_data, sizeof(error_data));
}

/**
 * @brief Process received command
 */
bool Protocol_ProcessCommand(uint8_t command)
{
    protocol_last_command = command;
    
    switch (command) {
        case 'i':
        case 'I':
            return Protocol_SendSystemInfo();
            
        case 'm':
        case 'M':
            return Protocol_SendMemoryInfo();
            
        case 'h':
        case 'H':
            return Protocol_SendStatusReport(STATUS_READY);
            
        case CMD_GET_INFO:
            return Protocol_SendSystemInfo();
            
        case CMD_GET_MEMORY:
            return Protocol_SendMemoryInfo();
            
        case CMD_HEARTBEAT:
            return Protocol_SendStatusReport(STATUS_IDLE);
            
        default:
            return Protocol_SendErrorReport(ERROR_INVALID_CMD);
    }
}

/**
 * @brief Process received data block (for interrupt mode)
 * @return true if processed any complete frame, false otherwise
 */
bool Protocol_ProcessReceivedData(const uint8_t* data, uint16_t length)
{
    if (data == NULL || length == 0) {
        return false;
    }
    
    // 检查最小帧长度
    if (length < PROTOCOL_MIN_FRAME_LENGTH) {
        return false;
    }
    
    uint16_t offset = 0;
    bool processed_any_frame = false;  // 跟踪是否处理了任何完整帧
    
    // 处理接收到的数据中可能包含的多个帧
    while (offset < length) {
        // 查找帧头0xAA (PC → MCU)
        while (offset < length && data[offset] != PROTOCOL_HEADER_PC_TO_MCU) {
            offset++;
        }
        
        // 检查是否还有足够的数据构成一个完整帧
        if (offset + PROTOCOL_MIN_FRAME_LENGTH > length) {
            break;  // 数据不完整，等待更多数据
        }
        
        // 解析帧
        uint8_t header = data[offset];
        uint8_t frame_length = data[offset + 1];
        uint8_t type = data[offset + 2];
        uint8_t status = data[offset + 3];
        
        // 计算完整帧长度
        uint16_t total_frame_length = 4 + frame_length + 1; // header + length + type + status + data + checksum
        
        // 检查是否有完整帧
        if (offset + total_frame_length > length) {
            break;  // 数据不完整，等待更多数据
        }
        
        // 获取数据和校验和
        const uint8_t* frame_data = (frame_length > 0) ? &data[offset + 4] : NULL;
        uint8_t received_checksum = data[offset + 4 + frame_length];
        
        // 计算校验和
        uint8_t calculated_checksum = frame_length + type + status;
        for (uint8_t i = 0; i < frame_length; i++) {
            calculated_checksum += frame_data[i];
        }
        calculated_checksum = ~calculated_checksum; // 取反
        
        // 验证校验和
        if (calculated_checksum != received_checksum) {
            Protocol_SendErrorReport(ERROR_CHECKSUM);
            offset += total_frame_length;
            processed_any_frame = true;  // 错误帧也算"处理"了，避免污染后续数据
            continue;  // 继续处理下一帧
        }
        
        // 处理命令 - 只有到这里才算成功处理了一帧
        switch (type) {
            case CMD_GET_INFO:
                Protocol_SendSystemInfo();
                processed_any_frame = true;
                break;
                
            case CMD_GET_MEMORY:
                Protocol_SendMemoryInfo();
                processed_any_frame = true;
                break;
                
            case CMD_HEARTBEAT:
                Protocol_SendStatusReport(STATUS_IDLE);
                processed_any_frame = true;
                break;
                
            case CMD_JUMP_TO_MODE:
                if (frame_length >= sizeof(Protocol_JumpModeRequest_t)) {
                    Protocol_JumpModeRequest_t* request = (Protocol_JumpModeRequest_t*)frame_data;
                    Protocol_ProcessJumpRequest(request);
                    processed_any_frame = true;
                } else {
                    Protocol_SendErrorReport(ERROR_LENGTH);
                    processed_any_frame = true;
                }
                break;
                
            default:
                Protocol_SendErrorReport(ERROR_INVALID_CMD);
                processed_any_frame = true;  // 无效命令也算"处理"了，避免污染后续数据
                break;
        }
        
        offset += total_frame_length;
    }
    
    return processed_any_frame;  // 只有成功处理了至少一帧才返回true
}

/**
 * @brief Send simple status
 */
bool Protocol_SendStatus(uint8_t status)
{
    return Protocol_SendFrame(CMD_STATUS_REPORT, status, NULL, 0);
}

/* Private function implementations ------------------------------------------*/

/**
 * @brief Get build timestamp as uint32_t
 */
static uint32_t Protocol_GetBuildTimestamp(void)
{
    // Simple hash of build date/time string for demonstration
    // In production, use actual timestamp
    const char* build_str = __DATE__ __TIME__;
    uint32_t hash = 0;
    
    for (int i = 0; build_str[i] != '\0'; i++) {
        hash = hash * 31 + (uint32_t)build_str[i];
    }
    
    return hash;
}

/**
 * @brief Get reset reason code
 */
static uint8_t Protocol_GetResetReason(void)
{
    // Check RCC reset flags
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
        return 0x01;  // Power-on reset
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
        return 0x02;  // Software reset
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        return 0x03;  // Independent watchdog reset
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
        return 0x04;  // Window watchdog reset
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
        return 0x05;  // Low-power reset
    }
    
    return 0x00;  // Unknown reset
}

/**
 * @brief Process jump mode request
 */
bool Protocol_ProcessJumpRequest(const Protocol_JumpModeRequest_t* request)
{
    if (request == NULL) {
        return false;
    }
    
    // 验证魔法字
    if (request->magic_word != JUMP_REQUEST_MAGIC) {
        Protocol_JumpModeResponse_t response = {
            .current_mode = MODE_APPLICATION,
            .target_mode = request->target_mode,
            .jump_status = MODE_JUMP_FAILED,
            .result_code = ERROR_INVALID_CMD,
            .uptime_before_jump = HAL_GetTick(),
            .estimated_jump_time = 0,
            .reserved = {0, 0}
        };
        Protocol_SendJumpResponse(&response);
        return false;
    }
    
    // 验证目标模式 (App只能跳转到Bootloader)
    if (request->target_mode != MODE_BOOTLOADER) {
        Protocol_JumpModeResponse_t response = {
            .current_mode = MODE_APPLICATION,
            .target_mode = request->target_mode,
            .jump_status = MODE_INVALID_TARGET,
            .result_code = ERROR_INVALID_CMD,
            .uptime_before_jump = HAL_GetTick(),
            .estimated_jump_time = 0,
            .reserved = {0, 0}
        };
        Protocol_SendJumpResponse(&response);
        return false;
    }
    
    // 发送确认响应
    Protocol_JumpModeResponse_t response = {
        .current_mode = MODE_APPLICATION,
        .target_mode = MODE_BOOTLOADER,
        .jump_status = MODE_JUMP_REQUESTED,
        .result_code = STATUS_OK,
        .uptime_before_jump = HAL_GetTick(),
        .estimated_jump_time = request->jump_delay_ms,
        .reserved = {0, 0}
    };
    Protocol_SendJumpResponse(&response);
    
    // 设置跳转挂起标志，在主循环中执行跳转
    jump_request_pending = true;
    jump_target_mode = request->target_mode;
    jump_request_time = HAL_GetTick();
    jump_delay_ms = request->jump_delay_ms;
    
    return true;
}

/**
 * @brief Send jump mode response
 */
bool Protocol_SendJumpResponse(const Protocol_JumpModeResponse_t* response)
{
    if (response == NULL) {
        return false;
    }
    
    return Protocol_SendFrame(CMD_JUMP_RESPONSE, response->jump_status, 
                             (const uint8_t*)response, sizeof(Protocol_JumpModeResponse_t));
}

/**
 * @brief Reset system to bootloader (using bootloader's proven method)
 */
void Protocol_ResetToBootloader(void)
{
    typedef void (*pFunction)(void);
    pFunction jump_to_bootloader;
    uint32_t jump_address;
    
    // === 额外清理App特有的状态 ===
    
    // 0. 重置Flash访问状态
    __HAL_FLASH_PREFETCH_BUFFER_DISABLE();
    
    // 1. 重置所有GPIO到默认状态（特别是PC13 LED）
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // 重置PC13到默认状态（输入、浮空）
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // 2. 重置NVIC优先级组到默认
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    
    // === 完全参考bootloader的清理步骤 ===
    
    // 3. 禁用所有中断 (来自DisableInterrupts函数)
    __disable_irq();
    
    // 4. 禁用SysTick完全
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    
    // 5. 完全重置NVIC中断状态
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;  // 禁用所有中断
        NVIC->ICPR[i] = 0xFFFFFFFF;  // 清除所有挂起中断
    }
    
    // 6. 重置所有外设 (来自ResetSystemPeripherals函数)
    __HAL_RCC_APB1_FORCE_RESET();
    __HAL_RCC_APB1_RELEASE_RESET();
    __HAL_RCC_APB2_FORCE_RESET();
    __HAL_RCC_APB2_RELEASE_RESET();
    
    // 7. 重置RCC到默认状态 (来自ResetRCC函数)
    // 启用HSI
    RCC->CR |= RCC_CR_HSION;
    
    // 等待HSI准备就绪
    while(!(RCC->CR & RCC_CR_HSIRDY));
    
    // 重置CFGR寄存器
    RCC->CFGR = 0x00000000;
    
    // 禁用HSE、CSS、PLL
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
    
    // 8. 重新映射中断向量表到bootloader地址
    SCB->VTOR = BOOTLOADER_START_ADDRESS;
    
    // 9. 从bootloader地址设置新的堆栈指针
    __set_MSP(*((volatile uint32_t*)BOOTLOADER_START_ADDRESS));
    
    // 10. 从bootloader地址+4获取复位处理程序地址
    jump_address = *((volatile uint32_t*)(BOOTLOADER_START_ADDRESS + 4));
    
    // 11. 清除LSB确保正确的函数地址 (来自bootloader逻辑)
    jump_to_bootloader = (pFunction)jump_address;
    
    // 12. 跳转到bootloader
    jump_to_bootloader();
    
    // 此函数不应返回
    while(1);
}

/**
 * @brief Check and execute pending jump request (call from main loop)
 */
void Protocol_CheckPendingJump(void)
{
    if (jump_request_pending) {
        uint32_t current_time = HAL_GetTick();
        
        // 检查是否到了跳转时间
        if ((current_time - jump_request_time) >= jump_delay_ms) {
            jump_request_pending = false; // 清除标志位
            
            if (jump_target_mode == MODE_BOOTLOADER) {
                // 在主循环中执行跳转，安全无中断冲突
                Protocol_ResetToBootloader();
            }
        }
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/