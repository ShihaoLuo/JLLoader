/**
  ******************************************************************************
  * @file    protocol.c
  * @author  Generated
  * @brief   Custom serial communication protocol implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "protocol.h"
#include "stm32f1xx_hal.h"
#include "uart.h"
#include "flash_config.h"
#include "bootloader_jump.h"
#include "main.h"
#include <stdbool.h>

/* Private defines -----------------------------------------------------------*/
#ifndef SystemCoreClock
extern uint32_t SystemCoreClock;
#endif

#ifndef SRAM_BASE
#define SRAM_BASE           0x20000000UL
#endif

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE     1024UL    // STM32F103C8T6
#endif

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
        return MODE_APPLICATION;
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
    sys_info.boot_mode = 0x01;  // Bootloader mode
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
    mem_info.ram_start = SRAM_BASE;
    mem_info.ram_size = 20 * 1024;  // 20KB for STM32F103C8T6
    mem_info.page_size = FLASH_PAGE_SIZE;
    mem_info.page_count = BOOTLOADER_PAGE_COUNT;
    mem_info.reserved = 0;
    
    uint8_t status = MEM_CONSTRAINT_OK;
    // Check memory constraints (10KB limit for bootloader)
    if (BOOTLOADER_SIZE > 0x2800) {  // 10KB = 0x2800 bytes
        status = MEM_CONSTRAINT_ERR;
    }
    
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
    bool processed_any_frame = false;  // 改为跟踪是否处理了任何完整帧
    
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
        
        // 更新最后执行的命令
        protocol_last_command = type;

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
                // 检查数据长度是否足够
                if (frame_length >= sizeof(Protocol_JumpModeRequest_t)) {
                    Protocol_JumpModeRequest_t* request = (Protocol_JumpModeRequest_t*)frame_data;
                    Protocol_ProcessJumpRequest(request);
                } else {
                    Protocol_SendErrorReport(ERROR_LENGTH);
                }
                processed_any_frame = true;
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
 * @brief Check and execute pending jump request (call from main loop)
 */
void Protocol_CheckPendingJump(void)
{
    if (jump_request_pending) {
        uint32_t current_time = HAL_GetTick();
        
        // 检查是否到了跳转时间
        if ((current_time - jump_request_time) >= jump_delay_ms) {
            jump_request_pending = false; // 清除标志位
            
            if (jump_target_mode == MODE_APPLICATION) {
                // 在主循环中执行跳转，安全无中断冲突
                Bootloader_JumpToApplication(APPLICATION_ADDRESS);
            }
        }
    }
}

/**
 * @brief 简化版本的跳转检查，不使用HAL_GetTick
 */
void Protocol_CheckPendingJump_Simple(void)
{
    static uint32_t jump_delay_counter = 0;
    
    if (jump_request_pending) {
        jump_delay_counter++;
        
        // 简单计数延迟，大约相当于指定的延迟时间
        // 假设每次调用间隔约0.1ms，所以 jump_delay_ms * 10 次调用
        if (jump_delay_counter >= (jump_delay_ms * 10)) {
            jump_request_pending = false; // 清除标志位
            jump_delay_counter = 0;
            
            if (jump_target_mode == MODE_APPLICATION) {
                // 在主循环中执行跳转，安全无中断冲突
                Bootloader_JumpToApplication(APPLICATION_ADDRESS);
            }
        }
    } else {
        jump_delay_counter = 0; // 重置计数器
    }
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

/* Jump-related function implementations -------------------------------------*/

/**
 * @brief Process mode jump request
 */
bool Protocol_ProcessJumpRequest(const Protocol_JumpModeRequest_t* request)
{
    Protocol_JumpModeResponse_t response;
    Protocol_JumpInfo_t jump_info;
    
    if (request == NULL) {
        return false;
    }
    
    // 验证跳转请求
    if (!Protocol_ValidateJumpRequest(request)) {
        // 发送错误响应
        response.current_mode = Protocol_GetRunningMode();
        response.target_mode = request->target_mode;
        response.jump_status = MODE_INVALID_TARGET;
        response.result_code = ERROR_INVALID_CMD;
        response.uptime_before_jump = HAL_GetTick();
        response.estimated_jump_time = 0;
        response.reserved[0] = 0;
        response.reserved[1] = 0;
        
        Protocol_SendJumpResponse(&response);
        return false;
    }
    
    // 发送确认响应
    response.current_mode = Protocol_GetRunningMode();
    response.target_mode = request->target_mode;
    response.jump_status = MODE_JUMP_REQUESTED;
    response.result_code = STATUS_OK;
    response.uptime_before_jump = HAL_GetTick();
    response.estimated_jump_time = request->jump_delay_ms + 50; // 估计跳转时间
    response.reserved[0] = 0;
    response.reserved[1] = 0;
    
    // 设置跳转请求标志，延迟到主循环中执行
    if (request->target_mode == MODE_APPLICATION) {
        // 设置跳转标志位
        jump_request_pending = true;
        jump_target_mode = request->target_mode;
        jump_request_time = HAL_GetTick();
        jump_delay_ms = request->jump_delay_ms;
    }
    
    Protocol_SendJumpResponse(&response);
    
    // 准备跳转信息
    jump_info.magic = JUMP_INFO_MAGIC;
    jump_info.source_mode = response.current_mode;
    jump_info.target_mode = request->target_mode;
    jump_info.jump_reason = 0x01; // Serial command jump
    jump_info.reserved = 0;
    jump_info.timestamp = HAL_GetTick();
    
    // 写入跳转信息到内存
    Protocol_WriteJumpInfo(&jump_info);
    
    return true;
}

/**
 * @brief Send mode jump response
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
 * @brief Validate jump request (magic word and target mode)
 */
bool Protocol_ValidateJumpRequest(const Protocol_JumpModeRequest_t* request)
{
    if (request == NULL) {
        return false;
    }
    
    // 验证魔法字
    if (request->magic_word != JUMP_REQUEST_MAGIC) {
        return false;
    }
    
    // 验证目标模式
    uint8_t current_mode = Protocol_GetRunningMode();
    
    // Bootloader只支持跳转到Application
    if (current_mode == MODE_BOOTLOADER && request->target_mode == MODE_APPLICATION) {
        // 检查Application是否有效
        return Bootloader_CheckApplication(APPLICATION_ADDRESS);
    }
    
    // 其他情况暂不支持
    return false;
}

/**
 * @brief Write jump information to special memory region
 */
bool Protocol_WriteJumpInfo(const Protocol_JumpInfo_t* jump_info)
{
    if (jump_info == NULL) {
        return false;
    }
    
    // 使用RAM起始地址附近的安全区域 (0x20000010-0x2000001F)
    // 避开栈顶和中断向量表可能使用的区域
    volatile Protocol_JumpInfo_t* jump_mem = (volatile Protocol_JumpInfo_t*)0x20000010;
    
    jump_mem->magic = jump_info->magic;
    jump_mem->source_mode = jump_info->source_mode;
    jump_mem->target_mode = jump_info->target_mode;
    jump_mem->jump_reason = jump_info->jump_reason;
    jump_mem->reserved = jump_info->reserved;
    jump_mem->timestamp = jump_info->timestamp;
    
    return true;
}

/**
 * @brief Read jump information from special memory region
 */
bool Protocol_ReadJumpInfo(Protocol_JumpInfo_t* jump_info)
{
    if (jump_info == NULL) {
        return false;
    }
    
    // 从RAM起始地址附近的安全区域读取 (0x20000010-0x2000001F)
    volatile Protocol_JumpInfo_t* jump_mem = (volatile Protocol_JumpInfo_t*)0x20000010;
    
    // 检查魔法字
    if (jump_mem->magic != JUMP_INFO_MAGIC) {
        return false;
    }
    
    jump_info->magic = jump_mem->magic;
    jump_info->source_mode = jump_mem->source_mode;
    jump_info->target_mode = jump_mem->target_mode;
    jump_info->jump_reason = jump_mem->jump_reason;
    jump_info->reserved = jump_mem->reserved;
    jump_info->timestamp = jump_mem->timestamp;
    
    return true;
}

/**
 * @brief Clear jump information from special memory region
 */
void Protocol_ClearJumpInfo(void)
{
    // 清除RAM起始地址附近的安全区域数据 (0x20000010-0x2000001F)
    volatile uint32_t* jump_mem = (volatile uint32_t*)0x20000010;
    for (int i = 0; i < 4; i++) {  // 清除16字节 (4 x 4字节)
        jump_mem[i] = 0;
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/