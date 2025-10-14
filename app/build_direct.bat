@echo off
REM ============================================
REM  Direct Build Script (No Make Required)
REM  STM32F103C8T6 Application Firmware
REM ============================================

echo ========================================
echo   Building APP Firmware (Direct)
echo ========================================
echo.

REM Change to app directory
cd /d "%~dp0"

REM Set toolchain path
set GCC_BIN=arm-gnu-toolchain\bin
set CC=%GCC_BIN%\arm-none-eabi-gcc.exe
set AS=%GCC_BIN%\arm-none-eabi-gcc.exe
set OBJCOPY=%GCC_BIN%\arm-none-eabi-objcopy.exe
set SIZE=%GCC_BIN%\arm-none-eabi-size.exe

REM Build directory
set BUILD_DIR=build
if not exist %BUILD_DIR% mkdir %BUILD_DIR%

REM Compiler flags (Release mode with maximum optimization)
set MCU=-mcpu=cortex-m3 -mthumb
set DEFS=-DUSE_HAL_DRIVER -DSTM32F103xB -DNDEBUG
set OPT=-O3 -flto
set INCLUDES=-ICore/Inc -IDrivers/STM32F1xx_HAL_Driver/Inc -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32F1xx/Include -IDrivers/CMSIS/Include
set CFLAGS=%MCU% %DEFS% %INCLUDES% %OPT% -Wall -fdata-sections -ffunction-sections -fno-strict-aliasing

REM Linker flags (Release mode with LTO)
set LDSCRIPT=STM32F103C8Tx_FLASH.ld
set LDFLAGS=%MCU% -specs=nano.specs -T%LDSCRIPT% -lc -lm -lnosys -Wl,-Map=%BUILD_DIR%/app.map,--cref -Wl,--gc-sections -flto -O3

echo [Step 1/4] Cleaning previous build...
if exist %BUILD_DIR%\*.o del /Q %BUILD_DIR%\*.o
if exist %BUILD_DIR%\*.d del /Q %BUILD_DIR%\*.d
if exist %BUILD_DIR%\*.elf del /Q %BUILD_DIR%\*.elf
if exist %BUILD_DIR%\*.hex del /Q %BUILD_DIR%\*.hex
if exist %BUILD_DIR%\*.bin del /Q %BUILD_DIR%\*.bin
echo [OK] Clean completed.
echo.

echo [Step 2/4] Compiling source files...

REM Compile Core sources
echo   Compiling Core sources...
%CC% -c %CFLAGS% Core/Src/main.c -o %BUILD_DIR%/main.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Core/Src/app_can_protocol.c -o %BUILD_DIR%/app_can_protocol.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Core/Src/app_init.c -o %BUILD_DIR%/app_init.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Core/Src/protocol.c -o %BUILD_DIR%/protocol.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Core/Src/uart.c -o %BUILD_DIR%/uart.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Core/Src/stm32f1xx_it.c -o %BUILD_DIR%/stm32f1xx_it.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Core/Src/stm32f1xx_hal_msp.c -o %BUILD_DIR%/stm32f1xx_hal_msp.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Core/Src/system_stm32f1xx.c -o %BUILD_DIR%/system_stm32f1xx.o
if %ERRORLEVEL% NEQ 0 goto ERROR

REM Compile HAL drivers
echo   Compiling HAL drivers...
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c -o %BUILD_DIR%/stm32f1xx_hal.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c -o %BUILD_DIR%/stm32f1xx_hal_can.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c -o %BUILD_DIR%/stm32f1xx_hal_cortex.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c -o %BUILD_DIR%/stm32f1xx_hal_dma.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c -o %BUILD_DIR%/stm32f1xx_hal_exti.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c -o %BUILD_DIR%/stm32f1xx_hal_flash.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c -o %BUILD_DIR%/stm32f1xx_hal_flash_ex.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c -o %BUILD_DIR%/stm32f1xx_hal_gpio.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c -o %BUILD_DIR%/stm32f1xx_hal_gpio_ex.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c -o %BUILD_DIR%/stm32f1xx_hal_pwr.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c -o %BUILD_DIR%/stm32f1xx_hal_rcc.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c -o %BUILD_DIR%/stm32f1xx_hal_rcc_ex.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c -o %BUILD_DIR%/stm32f1xx_hal_tim.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c -o %BUILD_DIR%/stm32f1xx_hal_tim_ex.o
if %ERRORLEVEL% NEQ 0 goto ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c -o %BUILD_DIR%/stm32f1xx_hal_uart.o
if %ERRORLEVEL% NEQ 0 goto ERROR

REM Compile startup file
echo   Compiling startup file...
%AS% -c %MCU% -x assembler-with-cpp Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s -o %BUILD_DIR%/startup_stm32f103xb.o
if %ERRORLEVEL% NEQ 0 goto ERROR

echo [OK] Compilation completed.
echo.

echo [Step 3/4] Linking...
%CC% %BUILD_DIR%/main.o %BUILD_DIR%/app_can_protocol.o %BUILD_DIR%/app_init.o %BUILD_DIR%/protocol.o %BUILD_DIR%/uart.o %BUILD_DIR%/stm32f1xx_it.o %BUILD_DIR%/stm32f1xx_hal_msp.o %BUILD_DIR%/system_stm32f1xx.o %BUILD_DIR%/stm32f1xx_hal.o %BUILD_DIR%/stm32f1xx_hal_can.o %BUILD_DIR%/stm32f1xx_hal_cortex.o %BUILD_DIR%/stm32f1xx_hal_dma.o %BUILD_DIR%/stm32f1xx_hal_exti.o %BUILD_DIR%/stm32f1xx_hal_flash.o %BUILD_DIR%/stm32f1xx_hal_flash_ex.o %BUILD_DIR%/stm32f1xx_hal_gpio.o %BUILD_DIR%/stm32f1xx_hal_gpio_ex.o %BUILD_DIR%/stm32f1xx_hal_pwr.o %BUILD_DIR%/stm32f1xx_hal_rcc.o %BUILD_DIR%/stm32f1xx_hal_rcc_ex.o %BUILD_DIR%/stm32f1xx_hal_tim.o %BUILD_DIR%/stm32f1xx_hal_tim_ex.o %BUILD_DIR%/stm32f1xx_hal_uart.o %BUILD_DIR%/startup_stm32f103xb.o %LDFLAGS% -o %BUILD_DIR%/app.elf
if %ERRORLEVEL% NEQ 0 goto ERROR
echo [OK] Linking completed.
echo.

echo [Step 4/4] Creating output files...
%OBJCOPY% -O ihex %BUILD_DIR%/app.elf %BUILD_DIR%/app.hex
if %ERRORLEVEL% NEQ 0 goto ERROR
%OBJCOPY% -O binary -S %BUILD_DIR%/app.elf %BUILD_DIR%/app.bin
if %ERRORLEVEL% NEQ 0 goto ERROR
echo [OK] Output files created.
echo.

echo ========================================
echo   Build Successful!
echo ========================================
echo.
echo Firmware size:
%SIZE% %BUILD_DIR%/app.elf
echo.
echo Output files:
echo   - build/app.elf  (ELF executable)
echo   - build/app.hex  (Intel HEX format)
echo   - build/app.bin  (Binary format)
for %%I in (%BUILD_DIR%\app.bin) do echo   Binary size: %%~zI bytes
echo.
goto END

:ERROR
echo.
echo [ERROR] Build failed!
echo.
pause
exit /b 1

:END
pause
