@echo off
chcp 65001 >nul
setlocal

rem ============================================================
rem  陀螺仪上位机 —— 模拟模式（不接板子也能看界面和曲线）
rem  用和真实接收完全相同的协议解析与显示逻辑产生假数据。
rem  连接真实设备请双击「启动上位机.bat」。
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

start "" "%APP_EXE%" --simulate
exit /b 0
