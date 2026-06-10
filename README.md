# MidSurfDemo 项目配置说明

## 环境要求

- Visual Studio 2022（含 C++ 开发工具）
- CMake 3.20 或更高版本
- Qt 5.15.2（MSVC 2019 64-bit）

------

## 第一步：下载安装 Qt 5.15.2

1. 下载 Qt 5.15.2 安装包（推荐使用 Qt 官方安装程序或在线安装器）
2. 安装时选择以下组件：
   - `MSVC 2019 64-bit`
   - `Qt Charts`（如需要）
   - `Qt Debug Information Files`
3. 记录安装路径，例如：`D:\QT\5.15\5.15.2\msvc2019_64`

------

## 第二步：修改项目中的 Qt 路径配置

项目中有 **3 个地方** 需要修改为你的 Qt 安装路径，请逐一修改：

### 1. 修改 `cmake_debug.bat`（第 12 行）

打开项目根目录下的 `cmake_debug.bat` 文件，找到第 12 行：

batch

```
set QT_PATH=D:\QT\5.15\5.15.2\msvc2019_64
```



将路径改为你自己的 Qt 安装路径，例如：

batch

```
set QT_PATH=C:\Qt\5.15.2\msvc2019_64
```



------

### 2. 修改 `CMakeLists.txt`

打开项目根目录下的 `CMakeLists.txt` 文件，找到 Qt 配置部分：

cmake

```
set(QT_PREFIX_PATH "D:/QT/5.15/5.15.2/msvc2019_64")
```



将路径改为你自己的 Qt 安装路径（注意使用正斜杠 `/`），例如：

cmake

```
set(QT_PREFIX_PATH "C:/Qt/5.15.2/msvc2019_64")
```



------

### 3. 修改 `main.cpp`

打开 `src/main.cpp` 文件，找到 `qputenv` 行：

cpp

```
qputenv("QT_QPA_PLATFORM_PLUGIN_PATH", "D:/QT/5.15/5.15.2/msvc2019_64/plugins/platforms");
```



将路径改为你自己的 Qt 安装路径（注意使用正斜杠 `/`），例如：

cpp

```
qputenv("QT_QPA_PLATFORM_PLUGIN_PATH", "C:/Qt/5.15.2/msvc2019_64/plugins/platforms");
```



------

## 第三步：编译项目

1. 打开命令提示符（cmd）或 PowerShell

2. 进入项目根目录：

   batch

   ```
   cd D:\code-zjh
   ```

   

3. 运行编译脚本：

   batch

   ```
   cmake_debug.bat
   ```

   

4. 等待编译完成，输出 `✅ 构建完成` 表示成功

------

## 第四步：运行程序

编译成功后，可执行文件位于 `build\Debug\MidSurfDemo.exe`

- 直接双击运行（会自动设置插件路径）

- 或通过命令行运行：

  batch

  ```
  build\Debug\MidSurfDemo.exe
  ```

  

------

## 常见问题

### Q1: 编译时提示找不到 Qt 头文件或库

**原因**：`CMakeLists.txt` 中的 Qt 路径未正确修改

**解决**：检查第 2 步中的路径配置是否正确

------

### Q2: 运行时提示 "Could not find the Qt platform plugin 'windows'"

**原因**：`main.cpp` 中的插件路径未正确修改

**解决**：检查第 3 步中的 `qputenv` 路径是否正确

------

### Q3: 运行时提示找不到 DLL

**原因**：缺少 Qt 运行时 DLL

**解决**：编译脚本会自动从 Qt 安装目录拷贝必要的 DLL 和插件到 `build\Debug` 目录。如果仍有缺失，可以手动运行：

batch

```
D:\QT\5.15\5.15.2\msvc2019_64\bin\windeployqt.exe build\Debug\MidSurfDemo.exe
```



------

## 项目目录结构

text

```
D:\code-zjh\
├── 3rdParty/               # 第三方库
│   ├── occ/                # OpenCASCADE
│   ├── openvdb/            # OpenVDB
│   ├── eigen-3.3.8/        # Eigen
│   └── nlopt/              # NLopt
├── src/                    # 源代码
│   ├── imso/               # IMSO 模块源文件
│   └── main.cpp            # 主程序入口
├── build/                  # 编译输出目录（自动生成）
├── CMakeLists.txt          # CMake 配置文件
└── cmake_debug.bat         # 编译脚本
```



------

## 注意事项

- 路径中的分隔符：CMakeLists.txt 和 main.cpp 中使用 **正斜杠 `/`**，批处理脚本中使用 **反斜杠 `\`**
- 确保 Qt 安装路径中**不包含中文或空格**
- 首次编译时间较长，请耐心等待