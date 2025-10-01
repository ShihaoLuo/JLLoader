/**
  ******************************************************************************
  * @file    bootloader_jump.h
  * @author  Generated
  * @brief   Header file for bootloader jump functionality
  ******************************************************************************
  */

#ifndef __BOOTLOADER_JUMP_H
#define __BOOTLOADER_JUMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/
#define APPLICATION_ADDRESS    0x08002800
#define STACK_POINTER_MASK     0x2FFE0000
#define STACK_POINTER_VALID    0x20000000
#define THUMB_BIT_MASK         0xFFFFFFFE

/* Exported types ------------------------------------------------------------*/
typedef void (*pFunction)(void);

/* Exported function prototypes ---------------------------------------------*/
bool Bootloader_CheckApplication(uint32_t app_address);
void Bootloader_JumpToApplication(uint32_t app_address);
void Bootloader_PrepareJump(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_JUMP_H */