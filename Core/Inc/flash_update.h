/**
  ******************************************************************************
  * @file    flash_update.h
  * @author  Generated
  * @brief   Header file for Flash firmware update operations
  ******************************************************************************
  */

#ifndef __FLASH_UPDATE_H
#define __FLASH_UPDATE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "flash_config.h"
#include <stdint.h>
#include <stdbool.h>

/* Flash Memory Layout - Use existing definitions from flash_config.h -------*/
/* The following addresses are already defined in flash_config.h:
 * - FLASH_BASE_ADDRESS        0x08000000
 * - BOOTLOADER_START_ADDRESS  0x08000000  
 * - BOOTLOADER_SIZE           0x4000 (16KB)
 * - APPLICATION_START_ADDRESS 0x08004000
 * - APPLICATION_MAX_SIZE      0xC000 (48KB)
 */

/* Flash Page Configuration */
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE                 1024        // STM32F103 page size (1KB)
#endif
#define FLASH_PAGES_PER_SECTOR          1           // 1 page per sector on STM32F103
#define BOOTLOADER_PAGE_COUNT           (BOOTLOADER_SIZE / FLASH_PAGE_SIZE)      // 16 pages
#define APPLICATION_PAGE_COUNT          (APPLICATION_MAX_SIZE / FLASH_PAGE_SIZE) // 48 pages

/* Flash Update Constants */
#define FLASH_WORD_SIZE                 4           // 32-bit word size
#define FLASH_HALFWORD_SIZE             2           // 16-bit halfword size
#define FLASH_BYTE_SIZE                 1           // 8-bit byte size

/* Update Block Size Constants */
#define UPDATE_MAX_BLOCK_SIZE           200         // Maximum block size for data transfer (保守设置适应协议)
#define UPDATE_MIN_BLOCK_SIZE           32          // Minimum block size for data transfer  
#define UPDATE_DEFAULT_BLOCK_SIZE       200         // Default block size
#define UPDATE_MAX_BLOCKS               256         // Maximum number of blocks

/* Update Timeout Constants */
#define UPDATE_TIMEOUT_MS               30000       // Update timeout (30 seconds)
#define UPDATE_BLOCK_TIMEOUT_MS         5000        // Block timeout (5 seconds)

/* Flash Operation Timeouts */
#define FLASH_ERASE_TIMEOUT_MS          1000        // Page erase timeout
#define FLASH_WRITE_TIMEOUT_MS          100         // Word write timeout
#define FLASH_VERIFY_TIMEOUT_MS         5000        // Verify timeout

/* Flash Update Buffer Configuration */
#define FLASH_WRITE_BUFFER_SIZE         1024        // Write buffer size
#define FLASH_VERIFY_CHUNK_SIZE         256         // Verify chunk size

/* Flash Update Status */
typedef enum {
    FLASH_UPDATE_OK                 = 0x00,         // Operation successful
    FLASH_UPDATE_ERROR              = 0x01,         // Generic error
    FLASH_UPDATE_INVALID_ADDRESS    = 0x02,         // Invalid address
    FLASH_UPDATE_INVALID_SIZE       = 0x03,         // Invalid size
    FLASH_UPDATE_ERASE_ERROR        = 0x04,         // Erase error
    FLASH_UPDATE_WRITE_ERROR        = 0x05,         // Write error
    FLASH_UPDATE_VERIFY_ERROR       = 0x06,         // Verify error
    FLASH_UPDATE_PROTECTED_AREA     = 0x07,         // Protected area access
    FLASH_UPDATE_TIMEOUT            = 0x08,         // Operation timeout
    FLASH_UPDATE_BUSY               = 0x09,         // Flash busy
    FLASH_UPDATE_INVALID_ALIGNMENT  = 0x0A          // Invalid alignment
} FlashUpdate_Status_t;

/* Flash Update Context */
typedef struct {
    uint32_t    target_address;                     // Target flash address
    uint32_t    total_size;                         // Total firmware size
    uint32_t    written_size;                       // Written size
    uint16_t    total_blocks;                       // Total blocks
    uint16_t    written_blocks;                     // Written blocks
    uint16_t    block_size;                         // Block size
    uint32_t    expected_crc32;                     // Expected CRC32
    uint32_t    calculated_crc32;                   // Calculated CRC32
    bool        erase_completed;                    // Erase completion flag
    bool        write_completed;                    // Write completion flag
    bool        verify_completed;                   // Verify completion flag
    uint32_t    last_activity_time;                 // Last activity timestamp
    uint8_t     write_buffer[FLASH_WRITE_BUFFER_SIZE]; // Write buffer
    uint32_t    buffer_address;                     // Buffer target address
    uint16_t    buffer_size;                        // Current buffer size
    bool        buffer_ready;                       // Buffer ready flag
} FlashUpdate_Context_t;

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Initialize flash update module
 * @retval FlashUpdate_Status_t: Operation status
 */
FlashUpdate_Status_t FlashUpdate_Init(void);

/**
 * @brief Validate flash address and size
 * @param address: Target flash address
 * @param size: Data size
 * @retval true if valid, false otherwise
 */
bool FlashUpdate_ValidateAddress(uint32_t address, uint32_t size);

/**
 * @brief Check if address is in bootloader protected area
 * @param address: Target address
 * @param size: Data size
 * @retval true if in protected area, false otherwise
 */
bool FlashUpdate_IsProtectedArea(uint32_t address, uint32_t size);

/**
 * @brief Erase application flash area
 * @param start_address: Start address (must be page-aligned)
 * @param size: Size to erase (will be rounded up to page boundary)
 * @retval FlashUpdate_Status_t: Operation status
 */
FlashUpdate_Status_t FlashUpdate_ErasePages(uint32_t start_address, uint32_t size);

/**
 * @brief Write data to flash
 * @param address: Target flash address (must be word-aligned)
 * @param data: Data buffer
 * @param size: Data size (must be multiple of word size)
 * @retval FlashUpdate_Status_t: Operation status
 */
FlashUpdate_Status_t FlashUpdate_WriteData(uint32_t address, const uint8_t* data, uint32_t size);

/**
 * @brief Verify flash data against expected data
 * @param address: Flash address to verify
 * @param expected_data: Expected data buffer
 * @param size: Data size
 * @retval FlashUpdate_Status_t: Operation status
 */
FlashUpdate_Status_t FlashUpdate_VerifyData(uint32_t address, const uint8_t* expected_data, uint32_t size);

/**
 * @brief Calculate CRC32 of flash data
 * @param address: Flash start address
 * @param size: Data size
 * @param crc32: Pointer to store calculated CRC32
 * @retval FlashUpdate_Status_t: Operation status
 */
FlashUpdate_Status_t FlashUpdate_CalculateCRC32(uint32_t address, uint32_t size, uint32_t* crc32);

/**
 * @brief Start firmware update session
 * @param target_address: Target flash address
 * @param total_size: Total firmware size
 * @param block_size: Data block size
 * @param expected_crc32: Expected firmware CRC32
 * @retval FlashUpdate_Status_t: Operation status
 */
FlashUpdate_Status_t FlashUpdate_StartSession(uint32_t target_address, uint32_t total_size, 
                                               uint16_t block_size, uint32_t expected_crc32);

/**
 * @brief Write firmware data block
 * @param block_number: Block number (starting from 0)
 * @param block_offset: Offset in firmware
 * @param data: Block data
 * @param size: Block size
 * @retval FlashUpdate_Status_t: Operation status
 */
FlashUpdate_Status_t FlashUpdate_WriteBlock(uint16_t block_number, uint32_t block_offset, 
                                             const uint8_t* data, uint16_t size);

/**
 * @brief Finalize firmware update (verify and complete)
 * @retval FlashUpdate_Status_t: Operation status
 */
FlashUpdate_Status_t FlashUpdate_Finalize(void);

/**
 * @brief Abort firmware update session
 * @retval FlashUpdate_Status_t: Operation status
 */
FlashUpdate_Status_t FlashUpdate_Abort(void);

/**
 * @brief Get current update context
 * @retval Pointer to update context
 */
const FlashUpdate_Context_t* FlashUpdate_GetContext(void);

/**
 * @brief Check if update session is active
 * @retval true if active, false otherwise
 */
bool FlashUpdate_IsSessionActive(void);

/**
 * @brief Get update progress percentage
 * @retval Progress percentage (0-100)
 */
uint8_t FlashUpdate_GetProgressPercent(void);

/**
 * @brief Process background flash tasks (call from main loop)
 * @retval None
 */
void FlashUpdate_ProcessBackgroundTasks(void);

/**
 * @brief Check for update timeouts
 * @retval true if timeout occurred, false otherwise
 */
bool FlashUpdate_CheckTimeout(void);

/* CRC32 Calculation Functions */

/**
 * @brief Initialize CRC32 calculation
 * @retval Initial CRC32 value
 */
uint32_t CRC32_Init(void);

/**
 * @brief Update CRC32 with single byte
 * @param crc: Current CRC32 value
 * @param data: Data byte
 * @retval Updated CRC32 value
 */
uint32_t CRC32_Update(uint32_t crc, uint8_t data);

/**
 * @brief Update CRC32 with data buffer
 * @param crc: Current CRC32 value
 * @param data: Data buffer
 * @param length: Data length
 * @retval Updated CRC32 value
 */
uint32_t CRC32_UpdateBuffer(uint32_t crc, const uint8_t* data, uint32_t length);

/**
 * @brief Finalize CRC32 calculation
 * @param crc: Current CRC32 value
 * @retval Final CRC32 value
 */
uint32_t CRC32_Finalize(uint32_t crc);

/**
 * @brief Calculate CRC32 of data buffer
 * @param data: Data buffer
 * @param length: Data length
 * @retval Calculated CRC32 value
 */
uint32_t CRC32_Calculate(const uint8_t* data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_UPDATE_H */