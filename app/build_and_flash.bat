@echo off
REM ============================================
REM  Build and Flash Script
REM  STM32F103C8T6 Application Firmware
REM  Combines build_direct.bat + firmware_flasher.py
REM ============================================

setlocal enabledelayedexpansion

echo ========================================
echo   Build and Flash APP Firmware
echo ========================================
echo.

REM Change to app directory
cd /d "%~dp0"

REM Parse command line arguments
set AUTO_FLASH=0
set FLASH_PORT=
set FLASH_BAUDRATE=115200

:parse_args
if "%~1"=="" goto end_parse
if /i "%~1"=="--auto" set AUTO_FLASH=1
if /i "%~1"=="--port" (
    set FLASH_PORT=%~2
    shift
)
if /i "%~1"=="--baudrate" (
    set FLASH_BAUDRATE=%~2
    shift
)
shift
goto parse_args
:end_parse

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

echo ========================================
echo   PHASE 1: BUILDING FIRMWARE
echo ========================================
echo.

echo [Step 1/4] Cleaning previous build...
if exist %BUILD_DIR%\*.o del /Q %BUILD_DIR%\*.o >nul 2>&1
if exist %BUILD_DIR%\*.d del /Q %BUILD_DIR%\*.d >nul 2>&1
if exist %BUILD_DIR%\*.elf del /Q %BUILD_DIR%\*.elf >nul 2>&1
if exist %BUILD_DIR%\*.hex del /Q %BUILD_DIR%\*.hex >nul 2>&1
if exist %BUILD_DIR%\*.bin del /Q %BUILD_DIR%\*.bin >nul 2>&1
echo [OK] Clean completed.
echo.

echo [Step 2/4] Compiling source files...

REM Compile Core sources
echo   Compiling Core sources...
%CC% -c %CFLAGS% Core/Src/main.c -o %BUILD_DIR%/main.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Core/Src/app_can_protocol.c -o %BUILD_DIR%/app_can_protocol.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Core/Src/app_init.c -o %BUILD_DIR%/app_init.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Core/Src/protocol.c -o %BUILD_DIR%/protocol.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Core/Src/uart.c -o %BUILD_DIR%/uart.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Core/Src/stm32f1xx_it.c -o %BUILD_DIR%/stm32f1xx_it.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Core/Src/stm32f1xx_hal_msp.c -o %BUILD_DIR%/stm32f1xx_hal_msp.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Core/Src/system_stm32f1xx.c -o %BUILD_DIR%/system_stm32f1xx.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR

REM Compile HAL drivers
echo   Compiling HAL drivers...
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c -o %BUILD_DIR%/stm32f1xx_hal.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c -o %BUILD_DIR%/stm32f1xx_hal_can.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c -o %BUILD_DIR%/stm32f1xx_hal_cortex.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c -o %BUILD_DIR%/stm32f1xx_hal_dma.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c -o %BUILD_DIR%/stm32f1xx_hal_exti.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c -o %BUILD_DIR%/stm32f1xx_hal_flash.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c -o %BUILD_DIR%/stm32f1xx_hal_flash_ex.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c -o %BUILD_DIR%/stm32f1xx_hal_gpio.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c -o %BUILD_DIR%/stm32f1xx_hal_gpio_ex.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c -o %BUILD_DIR%/stm32f1xx_hal_pwr.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c -o %BUILD_DIR%/stm32f1xx_hal_rcc.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c -o %BUILD_DIR%/stm32f1xx_hal_rcc_ex.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c -o %BUILD_DIR%/stm32f1xx_hal_tim.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c -o %BUILD_DIR%/stm32f1xx_hal_tim_ex.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%CC% -c %CFLAGS% Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c -o %BUILD_DIR%/stm32f1xx_hal_uart.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR

REM Compile startup file
echo   Compiling startup file...
%AS% -c %MCU% -x assembler-with-cpp Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s -o %BUILD_DIR%/startup_stm32f103xb.o >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR

echo [OK] Compilation completed.
echo.

echo [Step 3/4] Linking...
%CC% %BUILD_DIR%/main.o %BUILD_DIR%/app_can_protocol.o %BUILD_DIR%/app_init.o %BUILD_DIR%/protocol.o %BUILD_DIR%/uart.o %BUILD_DIR%/stm32f1xx_it.o %BUILD_DIR%/stm32f1xx_hal_msp.o %BUILD_DIR%/system_stm32f1xx.o %BUILD_DIR%/stm32f1xx_hal.o %BUILD_DIR%/stm32f1xx_hal_can.o %BUILD_DIR%/stm32f1xx_hal_cortex.o %BUILD_DIR%/stm32f1xx_hal_dma.o %BUILD_DIR%/stm32f1xx_hal_exti.o %BUILD_DIR%/stm32f1xx_hal_flash.o %BUILD_DIR%/stm32f1xx_hal_flash_ex.o %BUILD_DIR%/stm32f1xx_hal_gpio.o %BUILD_DIR%/stm32f1xx_hal_gpio_ex.o %BUILD_DIR%/stm32f1xx_hal_pwr.o %BUILD_DIR%/stm32f1xx_hal_rcc.o %BUILD_DIR%/stm32f1xx_hal_rcc_ex.o %BUILD_DIR%/stm32f1xx_hal_tim.o %BUILD_DIR%/stm32f1xx_hal_tim_ex.o %BUILD_DIR%/stm32f1xx_hal_uart.o %BUILD_DIR%/startup_stm32f103xb.o %LDFLAGS% -o %BUILD_DIR%/app.elf >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
echo [OK] Linking completed.
echo.

echo [Step 4/4] Creating output files...
%OBJCOPY% -O ihex %BUILD_DIR%/app.elf %BUILD_DIR%/app.hex >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
%OBJCOPY% -O binary -S %BUILD_DIR%/app.elf %BUILD_DIR%/app.bin >nul 2>nul
if %ERRORLEVEL% NEQ 0 goto BUILD_ERROR
echo [OK] Output files created.
echo.

echo ========================================
echo   BUILD SUCCESSFUL!
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

REM Check if firmware exists
if not exist "%BUILD_DIR%\app.bin" (
    echo [ERROR] app.bin not found! Build may have failed.
    goto END_ERROR
)

echo ========================================
echo   PHASE 2: FLASHING FIRMWARE
echo ========================================
echo.

REM Check for Python
python --version >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Python not found! Please install Python to use the flash feature.
    echo [INFO] You can manually flash using: build/app.hex or build/app.bin
    goto END_SUCCESS
)

REM Check if firmware_flasher.py exists
if not exist "firmware_flasher.py" (
    echo [ERROR] firmware_flasher.py not found!
    echo [INFO] You can manually flash using: build/app.hex or build/app.bin
    goto END_SUCCESS
)

echo [INFO] Starting firmware flash process...
echo.

REM Prepare flash command
set FLASH_CMD=python firmware_flasher.py --file %BUILD_DIR%\app.bin

REM Add port if specified
if not "%FLASH_PORT%"=="" (
    set FLASH_CMD=%FLASH_CMD% --port %FLASH_PORT%
)

REM Add baudrate
set FLASH_CMD=%FLASH_CMD% --baudrate %FLASH_BAUDRATE%

REM Add auto flag if specified
if %AUTO_FLASH%==1 (
    set FLASH_CMD=%FLASH_CMD% --auto
)

echo [EXEC] %FLASH_CMD%
echo.

REM Execute flash command
%FLASH_CMD%
set FLASH_RESULT=%ERRORLEVEL%

echo.
if %FLASH_RESULT%==0 (
    echo ========================================
    echo   BUILD AND FLASH COMPLETED!
    echo ========================================
    echo.
    echo [SUCCESS] Firmware built and flashed successfully!
) else (
    echo ========================================
    echo   FLASH FAILED!
    echo ========================================
    echo.
    echo [WARNING] Build successful but flash failed.
    echo [INFO] You can manually flash using: build/app.hex or build/app.bin
)

goto END_SUCCESS

:BUILD_ERROR
echo.
echo ========================================
echo   BUILD FAILED!
echo ========================================
echo.
echo [ERROR] Build process failed during compilation/linking.
echo [INFO] Check the error messages above for details.
goto END_ERROR

:END_SUCCESS
@REM pause >nul
exit /b 0

:END_ERROR
@REM pause >nul
exit /b 1