@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM Merge ESP32 firmware bins by scanning from BAT directory.
REM Required files: bootloader.bin, partitions.bin, firmware.bin

set "SCRIPT_DIR=%~dp0"
set "BOOTLOADER_BIN="
set "PARTITIONS_BIN="
set "FIRMWARE_BIN="
set "OUTPUT_BIN=%SCRIPT_DIR%merged-firmware.bin"

echo.
echo [INFO] Scanning for required BIN files under:
echo        %SCRIPT_DIR%
echo.

for /R "%SCRIPT_DIR%" %%F in (bootloader.bin) do (
    if not defined BOOTLOADER_BIN set "BOOTLOADER_BIN=%%~fF"
)
for /R "%SCRIPT_DIR%" %%F in (partitions.bin) do (
    if not defined PARTITIONS_BIN set "PARTITIONS_BIN=%%~fF"
)
for /R "%SCRIPT_DIR%" %%F in (firmware.bin) do (
    if not defined FIRMWARE_BIN set "FIRMWARE_BIN=%%~fF"
)

if not defined BOOTLOADER_BIN (
    echo [ERROR] bootloader.bin not found.
    goto :fail
)
if not defined PARTITIONS_BIN (
    echo [ERROR] partitions.bin not found.
    goto :fail
)
if not defined FIRMWARE_BIN (
    echo [ERROR] firmware.bin not found.
    goto :fail
)

echo [FOUND] bootloader.bin: !BOOTLOADER_BIN!
echo [FOUND] partitions.bin: !PARTITIONS_BIN!
echo [FOUND] firmware.bin:   !FIRMWARE_BIN!
echo.

if exist "%OUTPUT_BIN%" (
    del /f /q "%OUTPUT_BIN%" >nul 2>nul
)

if exist "%SCRIPT_DIR%esptool.exe" (
    echo [INFO] Using esptool.exe in BAT directory...
    "%SCRIPT_DIR%esptool.exe" --chip esp32s3 merge_bin -o "%OUTPUT_BIN%" ^
      0x0 "!BOOTLOADER_BIN!" ^
      0x8000 "!PARTITIONS_BIN!" ^
      0x10000 "!FIRMWARE_BIN!"
) else (
    echo [INFO] esptool.exe not found in BAT directory, trying Python module...
    python -m esptool --chip esp32s3 merge_bin -o "%OUTPUT_BIN%" ^
      0x0 "!BOOTLOADER_BIN!" ^
      0x8000 "!PARTITIONS_BIN!" ^
      0x10000 "!FIRMWARE_BIN!"
)

if errorlevel 1 goto :fail

echo.
echo [OK] Merge completed:
echo      %OUTPUT_BIN%
echo.
pause
exit /b 0

:fail
echo.
echo [FAILED] Could not merge firmware bins.
echo.
pause
exit /b 1
