@echo off
chcp 65001 >nul
setlocal

rem ============================================================
rem  陀螺仪上位机 —— 双击本文件即可打开（连接真实 USB-CAN 适配器）
rem  第一次双击会自动编译（约 1 分钟），之后就很快。
rem  不接板子想先看看界面，请双击「启动上位机（模拟模式）.bat」。
rem ============================================================

set "ROOT=%~dp0"

call "%ROOT%host_app\prepare-app.bat"
if errorlevel 1 (
    echo.
    pause
    exit /b 1
)

if defined PREPARE_NOTE (
    echo.
    pause
)

start "" "%APP_EXE%"
exit /b 0
