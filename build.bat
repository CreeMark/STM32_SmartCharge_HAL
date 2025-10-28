@echo off
echo ========================================
echo Building STM32F103 Project...
echo ========================================
cd MDK-ARM
D:\Keil_v5\UV4\UV4.exe -b STM32F103.uvprojx -o build.log
if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo Build SUCCESS!
    echo ========================================
) else (
    echo.
    echo ========================================
    echo Build FAILED! Check build.log
    echo ========================================
    type build.log
)
pause
