@echo off
chcp 65001 >nul
setlocal

REM 杀掉可能占用的进程
taskkill /F /IM MidSurfDemo.exe >nul 2>nul

REM 等待 2 秒避免系统未完全释放资源
timeout /T 2 >nul

REM ======== 设置 Qt 绝对路径 ==========
set QT_PATH=C:\Qt\Qt5.9.1\5.9.1\msvc2017_64

REM Qt 路径
set QT_BIN_PATH=%QT_PATH%\bin
set QT_PLATFORM_PATH=%QT_PATH%\plugins\platforms

REM ======== 设置第三方库路径（使用项目内的 3rdParty）========
set THIRD_PARTY_DIR=%~dp03rdParty

REM OpenVDB 路径
set OPENVDB_BIN_PATH=%THIRD_PARTY_DIR%\openvdb\bin

REM NLopt 路径
set NLOPT_BIN_PATH=%THIRD_PARTY_DIR%\nlopt

REM OpenCASCADE 路径
set OCC_BIN_PATH=%THIRD_PARTY_DIR%\occ\bin

REM 设置目标构建输出目录（Debug 模式）
set TARGET_PATH=build\Debug

REM 清理旧构建
if exist build (
    rmdir /s /q build
)

REM 等待 build 目录彻底删除
:WaitForDeletion
if exist build (
    timeout /t 1 >nul
    goto WaitForDeletion
)

REM 生成 VS 2022 解决方案
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
if errorlevel 1 (
    echo ❌ CMake 配置失败
    exit /b 1
)

REM 编译 Debug 配置
cmake --build build --config Debug

if errorlevel 1 (
    echo ❌ 编译失败
    exit /b 1
)

REM ======== 拷贝运行时 DLL 到输出目录 ========

REM 1. 拷贝 Qt DLL
if exist "%QT_BIN_PATH%" (
    echo 🔁 正在复制 Qt DLL 到 Debug 目录...
    xcopy /Y /D "%QT_BIN_PATH%\Qt5Core.dll" "%TARGET_PATH%"
    xcopy /Y /D "%QT_BIN_PATH%\Qt5Gui.dll" "%TARGET_PATH%"
    xcopy /Y /D "%QT_BIN_PATH%\Qt5Widgets.dll" "%TARGET_PATH%"
) else (
    echo ⚠️ Qt bin 目录不存在: %QT_BIN_PATH%
)

REM 2. 拷贝 OpenVDB DLL
if exist "%OPENVDB_BIN_PATH%" (
    echo 🔁 正在复制 OpenVDB DLL 到 Debug 目录...
    xcopy /Y /D "%OPENVDB_BIN_PATH%\*.dll" "%TARGET_PATH%"
) else (
    echo ⚠️ OpenVDB bin 目录不存在: %OPENVDB_BIN_PATH%
)

REM 3. 拷贝 NLopt DLL（如果存在）
if exist "%NLOPT_BIN_PATH%" (
    echo 🔁 正在复制 NLopt DLL 到 Debug 目录...
    xcopy /Y /D "%NLOPT_BIN_PATH%\*.dll" "%TARGET_PATH%"
)

REM 4. 拷贝 OpenCASCADE DLL（如果存在）
if exist "%OCC_BIN_PATH%" (
    echo 🔁 正在复制 OpenCASCADE DLL 到 Debug 目录...
    xcopy /Y /D "%OCC_BIN_PATH%\*.dll" "%TARGET_PATH%"
)

REM 5. 拷贝 Qt 平台插件（qwindows.dll 等）
if exist "%QT_PLATFORM_PATH%" (
    echo 🔁 拷贝 platforms 插件...
    mkdir "%TARGET_PATH%\platforms" 2>nul
    xcopy /Y /D "%QT_PLATFORM_PATH%\qwindows.dll" "%TARGET_PATH%\platforms\"
    xcopy /Y /D "%QT_BIN_PATH%\*.dll" "%TARGET_PATH%"
)

echo ✅ 构建完成

endlocal
pause