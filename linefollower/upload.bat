@echo off
REM Upload script for ESP32 Line Follower
REM This batch file helps you quickly upload your code to the ESP32

echo ================================================
echo ESP32 Line Follower - Upload Script
echo ================================================
echo.

echo Step 1: Building project...
pio run
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Build failed! Fix errors and try again.
    pause
    exit /b 1
)

echo.
echo Step 2: Uploading to ESP32...
pio run --target upload
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Upload failed!
    echo - Check USB connection
    echo - Try: pio device list
    echo - Manually specify port: pio run -t upload --upload-port COM3
    pause
    exit /b 1
)

echo.
echo ================================================
echo Upload successful!
echo ================================================
echo.
echo Opening serial monitor (9600 baud)...
echo Press Ctrl+C to exit monitor
echo.
timeout /t 2
pio device monitor

pause

