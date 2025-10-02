/**
  ******************************************************************************
  * @file    protocol.c
  * @author  Generated
  * @brief   Custom serial communication protocol implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "protocol.h"
#include "uart.h"
#include "flash_config.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
static uint32_t protocol_uptime_ms = 0;
static uint8_t protocol_error_count = 0;
static uint16_t protocol_last_command = 0;

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
    
    Protocol_Frame_t frame;
    frame.header = PROTOCOL_HEADER_MCU_TO_PC;  // MCU向PC发送使用0xBB
    frame.length = length;
    frame.type = type;
    frame.status = status;
    
    // Copy data if provided
    if (data != NULL && length > 0) {
        for (uint8_t i = 0; i < length; i++) {
            frame.data[i] = data[i];
        }
    }
    
    // Calculate and set checksum
    frame.checksum = Protocol_CalculateChecksum(&frame, length);
    
    // Send frame via UART
    uint8_t* frame_bytes = (uint8_t*)&frame;
    uint16_t frame_size = PROTOCOL_MIN_FRAME_LENGTH + length;
    
    return UART_SendData(frame_bytes, frame_size);
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
            continue;  // 继续处理下一帧，但不标记为成功
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
                
            default:
                Protocol_SendErrorReport(ERROR_INVALID_CMD);
                break;  // 无效命令不算成功处理
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/