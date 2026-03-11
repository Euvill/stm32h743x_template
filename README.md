# led_blink

这是一个基于 STM32H743 的裸机/HAL 工程，可使用 `VS Code + OpenOCD + CMSIS-DAP` 完成编译、烧录和调试。

## 工程特性

- 编译系统：`Makefile`（GNU Make）
- 编译器：`arm-none-eabi-gcc`
- 调试器：VS Code `cortex-debug` 扩展 + `openocd`
- 调试探针：`CMSIS-DAP`（SWD）
- 目标芯片：`STM32H743VIT6`

## 依赖工具清单

在 Windows 环境下，至少需要安装以下工具并加入 `PATH`：

1. `Visual Studio Code`
2. VS Code 扩展：
   - `marus25.cortex-debug`（必须，用于调试）
   - `ms-vscode.cpptools`（建议，用于 C/C++ 工程支持）
3. `GNU Arm Embedded Toolchain`（必须）：
   - `arm-none-eabi-gcc`
   - `arm-none-eabi-gdb`
   - `arm-none-eabi-objcopy`
   - `arm-none-eabi-size`
4. `GNU Make`（必须）
5. `OpenOCD`（必须，需包含 cmsis-dap 支持）
6. `Python 3`（可选，用于生成 `compile_commands.json`）
7. `CMSIS-DAP` 硬件仿真器（必须）

## 快速检查环境

在终端执行以下命令，确认工具可用：

```powershell
arm-none-eabi-gcc --version
arm-none-eabi-gdb --version
make --version
openocd --version
python --version
```

## VS Code 工作流

### 1. 编译

- 在 VS Code 中执行任务：`build`
- 或终端执行：

```powershell
make -j
```

编译后会在 `build/` 目录生成：

- `led_blink.elf`
- `led_blink.hex`
- `led_blink.bin`

### 2. 烧录

- 在 VS Code 中执行任务：`flash`
- 该任务等价于：

```powershell
openocd -f ./Debug/interface/cmsis-dap.cfg -c "transport select swd" -f ./Debug/target/stm32h7x.cfg -c "adapter speed 1000" -c "program ./build/led_blink.hex verify reset exit"
```

### 3. 调试

1. 连接 CMSIS-DAP 与目标板（SWD）。
2. 在 VS Code 调试面板选择：`STM32H743 Debug (CMSIS-DAP)`。
3. 启动调试（`F5`）。

工程使用以下关键配置：

- 可执行文件：`./build/led_blink.elf`
- SVD 文件：`./Debug/svd/STM32H743.svd`
- OpenOCD 接口脚本：`./Debug/interface/cmsis-dap.cfg`
- OpenOCD 目标脚本：`./Debug/target/stm32h7x.cfg`

## 常用任务

- `build`：编译工程
- `clean`：清理 `build` 目录
- `flash`：烧录到目标板
- `gen compile_commands`：生成 `build/compile_commands.json`（依赖 `python -m compiledb`）

## 目录说明（节选）

- `Core/`：应用代码与中断处理
- `Drivers/`：CMSIS 与 STM32 HAL 驱动
- `.vscode/`：VS Code 任务和调试配置
- `Debug/interface/cmsis-dap.cfg`：CMSIS-DAP 接口脚本
- `Debug/target/stm32h7x.cfg`：STM32H7 目标脚本
- `Makefile`：构建入口

