/**
  ******************************************************************************
  * @file    protocol.h
  * @author  Generated
  * @brief   Header file for custom serial communication protocol
  ******************************************************************************
  */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Protocol Constants -------------------------------------------------------*/
#define PROTOCOL_HEADER_PC_TO_MCU       0xAA    // PC → MCU frame header
#define PROTOCOL_HEADER_MCU_TO_PC       0xBB    // MCU → PC frame header
#define PROTOCOL_MAX_DATA_LENGTH        252
#define PROTOCOL_MIN_FRAME_LENGTH       5    // Header + Length + Type + Status + Checksum
#define PROTOCOL_MAX_FRAME_LENGTH       257  // Min + Max Data
#define PROTOCOL_TIMEOUT_MS             100
#define PROTOCOL_MAX_RETRIES            3

/* Message Types (TYPE field) -----------------------------------------------*/
#define CMD_SYSTEM_INFO                 0x01  // MCU → Host: System info response
#define CMD_MEMORY_INFO                 0x02  // MCU → Host: Memory layout info
#define CMD_STATUS_REPORT               0x03  // MCU → Host: Status report
#define CMD_ERROR_REPORT                0x04  // MCU → Host: Error report
#define CMD_JUMP_RESPONSE               0x05  // MCU → Host: Mode jump response
#define CMD_GET_INFO                    0x10  // Host → MCU: Get system info
#define CMD_GET_MEMORY                  0x11  // Host → MCU: Get memory info
#define CMD_HEARTBEAT                   0x12  // Host → MCU: Heartbeat
#define CMD_JUMP_TO_MODE                0x13  // Host → MCU: Mode jump command

/* Status Codes (STATUS field) ----------------------------------------------*/
/* System Status (0x00-0x0F) */
#define STATUS_OK                       0x00  // Operation successful
#define STATUS_READY                    0x01  // System ready
#define STATUS_BUSY                     0x02  // System busy
#define STATUS_IDLE                     0x03  // System idle

/* Error Status (0x10-0x2F) */
#define ERROR_INVALID_CMD               0x10  // Invalid command
#define ERROR_CHECKSUM                  0x11  // Checksum error
#define ERROR_LENGTH                    0x12  // Length error
#define ERROR_TIMEOUT                   0x13  // Timeout error

/* Memory Status (0x30-0x4F) */
#define MEM_BOOTLOADER_OK               0x30  // Bootloader area OK
#define MEM_APP_VALID                   0x31  // Application valid
#define MEM_APP_INVALID                 0x32  // Application invalid
#define MEM_CONSTRAINT_OK               0x33  // Memory constraint check passed
#define MEM_CONSTRAINT_ERR              0x34  // Memory constraint check failed

/* Hardware Status (0x50-0x6F) */
#define HW_CLOCK_OK                     0x50  // Clock configuration OK
#define HW_UART_OK                      0x51  // UART configuration OK
#define HW_GPIO_OK                      0x52  // GPIO configuration OK
#define HW_TIMER_OK                     0x53  // Timer configuration OK

/* Running Mode Status (0x70-0x8F) */
#define MODE_BOOTLOADER                 0x70  // Running in Bootloader mode
#define MODE_APPLICATION                0x71  // Running in Application mode
#define MODE_SYSTEM_MEMORY              0x72  // Running in System Memory mode
#define MODE_UNKNOWN                    0x73  // Unknown running mode

/* Mode Jump Status (0x50-0x6F) */
#define MODE_JUMP_REQUESTED             0x52  // Mode jump request received
#define MODE_JUMP_PREPARING             0x53  // Preparing for mode jump
#define MODE_JUMP_SUCCESS               0x54  // Mode jump successful
#define MODE_JUMP_FAILED                0x55  // Mode jump failed
#define MODE_INVALID_TARGET             0x56  // Invalid target mode
#define MODE_JUMP_TIMEOUT               0x57  // Mode jump timeout

/* Jump Magic Words */
#define JUMP_REQUEST_MAGIC              0x12345678  // Magic word for jump request
#define JUMP_INFO_MAGIC                 0xDEADBEEF  // Magic word for jump info

/* Data Structures -----------------------------------------------------------*/

/**
 * @brief Protocol frame structure
 */
typedef struct {
    uint8_t header;                             /*!< Frame header (0xAA: PC→MCU, 0xBB: MCU→PC) */
    uint8_t length;                             /*!< Data length (0-252) */
    uint8_t type;                               /*!< Message type */
    uint8_t status;                             /*!< Status code */
    uint8_t data[PROTOCOL_MAX_DATA_LENGTH];     /*!< Variable length data */
    uint8_t checksum;                           /*!< Frame checksum */
} Protocol_Frame_t;

/**
 * @brief System information data structure
 */
typedef struct {
    uint32_t firmware_version;                  /*!< Firmware version */
    uint32_t build_timestamp;                   /*!< Build timestamp */
    uint32_t cpu_frequency;                     /*!< CPU frequency in Hz */
    uint16_t device_id;                         /*!< Device ID */
    uint8_t  reset_reason;                      /*!< Reset reason */
    uint8_t  boot_mode;                         /*!< Boot mode (0x01=Bootloader, 0x02=Application) */
    uint8_t  running_mode;                      /*!< Current running mode (0x70-0x73) */
    uint8_t  reserved;                          /*!< Reserved byte for alignment */
} __attribute__((packed)) Protocol_SystemInfo_t;

/**
 * @brief Memory information data structure
 */
typedef struct {
    uint32_t bootloader_start;                  /*!< Bootloader start address */
    uint32_t bootloader_size;                   /*!< Bootloader size */
    uint32_t application_start;                 /*!< Application start address */
    uint32_t application_size;                  /*!< Application max size */
    uint32_t ram_start;                         /*!< RAM start address */
    uint32_t ram_size;                          /*!< RAM size */
    uint16_t page_size;                         /*!< Flash page size */
    uint8_t  page_count;                        /*!< Bootloader page count */
    uint8_t  reserved;                          /*!< Reserved byte */
} __attribute__((packed)) Protocol_MemoryInfo_t;

/**
 * @brief Status report data structure
 */
typedef struct {
    uint32_t uptime_ms;                         /*!< Uptime in milliseconds */
    uint16_t last_command;                      /*!< Last executed command */
    uint8_t  system_status;                     /*!< System status */
    uint8_t  error_count;                       /*!< Error count */
    uint8_t  running_mode;                      /*!< Current running mode (0x70-0x73) */
    uint8_t  reserved[3];                       /*!< Reserved bytes for alignment */
} __attribute__((packed)) Protocol_StatusReport_t;

/**
 * @brief Mode jump request data structure
 */
typedef struct {
    uint8_t  target_mode;                       /*!< Target mode (0x70=Bootloader, 0x71=Application) */
    uint8_t  jump_delay_ms;                     /*!< Jump delay time (milliseconds) */
    uint16_t timeout_ms;                        /*!< Jump timeout (milliseconds) */
    uint32_t magic_word;                        /*!< Magic word (0x12345678) for security */
} __attribute__((packed)) Protocol_JumpModeRequest_t;

/**
 * @brief Mode jump response data structure
 */
typedef struct {
    uint8_t  current_mode;                      /*!< Current mode */
    uint8_t  target_mode;                       /*!< Target mode */
    uint8_t  jump_status;                       /*!< Jump status (0x52-0x57) */
    uint8_t  result_code;                       /*!< Result code */
    uint32_t uptime_before_jump;                /*!< Uptime before jump */
    uint16_t estimated_jump_time;               /*!< Estimated jump time (ms) */
    uint8_t  reserved[2];                       /*!< Reserved bytes */
} __attribute__((packed)) Protocol_JumpModeResponse_t;

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Initialize protocol module
 * @retval None
 */
void Protocol_Init(void);

/**
 * @brief Calculate frame checksum
 * @param frame: Pointer to protocol frame
 * @param data_length: Length of data field
 * @retval Calculated checksum
 */
uint8_t Protocol_CalculateChecksum(const Protocol_Frame_t* frame, uint8_t data_length);

/**
 * @brief Verify frame checksum
 * @param frame: Pointer to protocol frame
 * @param data_length: Length of data field
 * @retval true if checksum is valid, false otherwise
 */
bool Protocol_VerifyChecksum(const Protocol_Frame_t* frame, uint8_t data_length);

/**
 * @brief Verify frame header for direction
 * @param header: Header byte to verify
 * @param expected_direction: 0 for PC→MCU (0xAA), 1 for MCU→PC (0xBB)
 * @retval true if header matches expected direction, false otherwise
 */
bool Protocol_VerifyHeader(uint8_t header, uint8_t expected_direction);

/**
 * @brief Send protocol frame
 * @param type: Message type
 * @param status: Status code
 * @param data: Pointer to data (can be NULL)
 * @param length: Data length
 * @retval true if sent successfully, false otherwise
 */
bool Protocol_SendFrame(uint8_t type, uint8_t status, const uint8_t* data, uint8_t length);

/**
 * @brief Send system information
 * @retval true if sent successfully, false otherwise
 */
bool Protocol_SendSystemInfo(void);

/**
 * @brief Send memory information
 * @retval true if sent successfully, false otherwise
 */
bool Protocol_SendMemoryInfo(void);

/**
 * @brief Send status report
 * @param system_status: Current system status
 * @retval true if sent successfully, false otherwise
 */
bool Protocol_SendStatusReport(uint8_t system_status);

/**
 * @brief Send error report
 * @param error_code: Error code
 * @retval true if sent successfully, false otherwise
 */
bool Protocol_SendErrorReport(uint8_t error_code);

/**
 * @brief Process received command
 * @param command: Command byte received
 * @retval true if command processed successfully, false otherwise
 */
bool Protocol_ProcessCommand(uint8_t command);

/**
 * @brief Process received data block (for interrupt mode)
 * @param data: Pointer to received data buffer
 * @param length: Length of received data
 * @retval true if data processed successfully, false otherwise
 */
bool Protocol_ProcessReceivedData(const uint8_t* data, uint16_t length);

/**
 * @brief Send simple status
 * @param status: Status code to send
 * @retval true if sent successfully, false otherwise
 */
bool Protocol_SendStatus(uint8_t status);

/**
 * @brief Get current running mode
 * @retval Current running mode (MODE_BOOTLOADER, MODE_APPLICATION, etc.)
 */
uint8_t Protocol_GetRunningMode(void);

/**
 * @brief Process jump mode request
 * @param request: Pointer to jump request structure
 * @retval true if request is valid and processed, false otherwise
 */
bool Protocol_ProcessJumpRequest(const Protocol_JumpModeRequest_t* request);

/**
 * @brief Send jump mode response
 * @param response: Pointer to jump response structure
 * @retval true if sent successfully, false otherwise
 */
bool Protocol_SendJumpResponse(const Protocol_JumpModeResponse_t* response);

/**
 * @brief Reset system to bootloader
 * @retval None (this function should not return)
 */
void Protocol_ResetToBootloader(void);

/**
 * @brief Check and execute pending jump request (call from main loop)
 * @retval None
 */
void Protocol_CheckPendingJump(void);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/