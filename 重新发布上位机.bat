@echo off
chcp 65001 >nul
setlocal

rem ============================================================
rem  重新发布上位机 —— 双击本文件，把当前源码编译成
rem  host_app\publish\GyroHost.exe
rem
rem  只改代码、不需要重启电脑；重新发布后直接双击
rem  「启动上位机.bat」就是新版本。
rem
rem  做法：先发布到 publish.new，再替换 publish。
rem  如果程序正在运行（publish 被占用），替换会失败，此时
rem  保留 publish.new；下次双击「启动上位机.bat」会自动装上。
rem ============================================================

set "ROOT=%~dp0"
set "HOST_APP=%ROOT%host_app"
set "PROJ=%HOST_APP%\src\GyroHost.App\GyroHost.App.csproj"
set "PUBLISH=%HOST_APP%\publish"
set "STAGING=%HOST_APP%\publish.new"
set "BACKUP=%HOST_APP%\publish.old"

where dotnet >nul 2>nul
if errorlevel 1 (
    echo [错误] 没有找到 dotnet 命令。
    echo        请先安装 .NET 10 SDK：https://dotnet.microsoft.com/download/dotnet/10.0
    echo.
    if not defined NO_PAUSE pause
    exit /b 1
)

if exist "%STAGING%\" rmdir /s /q "%STAGING%"

echo 正在编译并发布上位机，请稍候...
echo.
dotnet publish "%PROJ%" -c Release -o "%STAGING%" --nologo -v minimal
if errorlevel 1 (
    echo.
    echo [错误] 发布失败，publish 目录保持原样未被改动。
    rmdir /s /q "%STAGING%" >nul 2>nul
    echo.
    if not defined NO_PAUSE pause
    exit /b 1
)
echo.

if not exist "%PUBLISH%\" goto :promote

rem 先移走旧版本：程序正在运行时这一步会失败，说明有新版本待安装。
move "%PUBLISH%" "%BACKUP%" >nul 2>nul
if errorlevel 1 (
    echo [提示] 程序正在运行，publish 目录被占用。
    echo        新版本已放在 host_app\publish.new。
    echo        关闭程序窗口后重新双击「启动上位机.bat」，会自动更新。
    echo.
    if not defined NO_PAUSE pause
    exit /b 0
)

:promote
move "%STAGING%" "%PUBLISH%" >nul 2>nul
if errorlevel 1 (
    echo [警告] 新版本安装失败，正在回退到旧版本。
    move "%BACKUP%" "%PUBLISH%" >nul 2>nul
    echo.
    if not defined NO_PAUSE pause
    exit /b 1
)

rmdir /s /q "%BACKUP%" >nul 2>nul
echo 发布完成：%PUBLISH%\GyroHost.exe
echo 直接双击「启动上位机.bat」即可运行新版本。
echo.
if not defined NO_PAUSE pause
exit /b 0
