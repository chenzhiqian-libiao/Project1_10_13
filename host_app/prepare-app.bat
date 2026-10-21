@echo off
rem ============================================================
rem  内部脚本：由根目录的「启动上位机.bat」调用，一般不需要直接双击。
rem
rem  职责：
rem    1) 若上次编译时程序正在运行、新版本没能装上，这次启动前先装上；
rem    2) 第一次运行时自动编译出可执行文件；
rem    3) 通过环境变量 APP_EXE 把程序路径交回给调用方。
rem
rem  可选参数：工程根目录（默认 = 本脚本所在目录；仅供自动化测试使用）。
rem ============================================================
chcp 65001 >nul
setlocal

set "HOST_APP=%~1"
if "%HOST_APP%"=="" set "HOST_APP=%~dp0"
rem 去掉结尾的反斜杠，方便后面拼接
if "%HOST_APP:~-1%"=="\" set "HOST_APP=%HOST_APP:~0,-1%"

set "PUBLISH=%HOST_APP%\publish"
set "STAGING=%HOST_APP%\publish.new"
set "BACKUP=%HOST_APP%\publish.old"
set "PROJ=%HOST_APP%\src\GyroHost.App\GyroHost.App.csproj"

rem ---------- 1) 安装待更新的新版本（编译时程序正在运行的情况） ----------
if not exist "%STAGING%\" goto :check_built

if not exist "%PUBLISH%\" goto :promote

move "%PUBLISH%" "%BACKUP%" >nul 2>nul
if errorlevel 1 (
    echo [提示] 有新版本待安装，但旧版本仍在运行，本次先启动旧版本。
    echo        关闭程序窗口后重新双击启动，就会自动更新。
    set "PREPARE_NOTE=1"
    goto :check_built
)

:promote
move "%STAGING%" "%PUBLISH%" >nul 2>nul
if errorlevel 1 (
    echo [警告] 新版本安装失败，正在回退到旧版本。
    move "%BACKUP%" "%PUBLISH%" >nul 2>nul
    set "PREPARE_NOTE=1"
    goto :check_built
)
rmdir /s /q "%BACKUP%" >nul 2>nul
echo 已更新到最新版本。
echo.
echo [重要] 本次更新包含协议比例的改动（角度和角速度都改成 x100000）。
echo        如果你还没重新烧写固件，请到 VisualGDB 里重新编译烧写；
echo        只换上位机不换固件（或反过来）会让角度或角速度正好差 1000 倍，且不会报错。
echo        详见 host_app\README.md 第一节「升级须知」。
echo.

rem ---------- 2) 还没有可执行文件时自动编译 ----------
:check_built
if exist "%PUBLISH%\GyroHost.exe" goto :done

echo 首次运行：正在编译上位机，请稍候（约 1 分钟）...
echo.
where dotnet >nul 2>nul
if errorlevel 1 (
    echo [错误] 没有找到 dotnet 命令。
    echo        请先安装 .NET 10 SDK：https://dotnet.microsoft.com/download/dotnet/10.0
    exit /b 1
)

dotnet publish "%PROJ%" -c Release -o "%PUBLISH%" --nologo -v minimal
if errorlevel 1 exit /b 1
echo.
echo 编译完成。

:done
endlocal & set "APP_EXE=%PUBLISH%\GyroHost.exe" & set "PREPARE_NOTE=%PREPARE_NOTE%"
exit /b 0
