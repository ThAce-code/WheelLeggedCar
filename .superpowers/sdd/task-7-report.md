# Task 7 验证报告（基线 `b86f8315`）

## 结论

主机互操作测试、静态归属扫描和三核 IAR Debug 构建均完成。CM7_0、CM7_1、CM0+ 均为 0 errors；CM7_1 与 CM0+ 为 0 warnings。CM7_0 报告 3 个 `control_leg.c:16-18` 的既有 Pe550（局部变量未使用）警告，未涉及本任务改动，也未修改 `libraries`。共享 SRAM linker/map/MPU 检查通过。硬件步骤（上电、调试器、UART 序列、带电机测试）本次未执行，因此不能替代板级 go/no-go。

## Step 1：完整 host suite

在干净的 `project/tests/build` 下执行 brief 中的 gcc 命令。PowerShell 直接调用 gcc 未返回诊断；使用同一 gcc 通过 MSYS2 UCRT64 shell 重复执行，编译退出码为 `0`，运行输出为：

```text
compile_exit=0
intercore_control_foundation_test: PASS
```

## Step 2：静态检查

`git diff --check` 退出码 `0`。

三条归属/阻塞 `rg` 扫描均无匹配（退出码 `1`，即空结果）。EWP 检查确认 CM7_0 仅包含 `cm7_0_isr.c`/`main_cm7_0.c`，CM7_1 仅包含 `cm7_1_isr.c`/`main_cm7_1.c`。MPU 源检查确认 `INTERCORE_SHARED_BASE_ADDRESS` 配置为 8 KiB、full access、non-executable、normal shared non-cacheable，并调用 `Cy_MPU_Setup`。

## Step 3：IAR 三核重建

使用 `D:\IAR\common\bin\IarBuild.exe`，按 CM7_0、CM7_1、CM0+ 顺序执行 brief 中三条 `-build Debug -log all` 命令。日志：`project/tests/build/cyt4bb7_cm_7_0.log`、`cyt4bb7_cm_7_1.log`、`cyt4bb7_cm_0_plus.log`。

| image | IAR 结果 | warnings |
|---|---|---|
| CM7_0 | `Build succeeded`; errors `0` | `3`: `project/code/control_leg.c(16-18)`, Pe550，`control_leg_*_cmd` set but never used |
| CM7_1 | `Build succeeded`; errors `0` | `0` |
| CM0+ | `Build succeeded`; errors `0` | `0` |

三份 map 都导出 `__intercore_shared_sram_base = 0x2808'0000`、`__intercore_shared_sram_size = 0x2000`；CM7_0 可用 SRAM 结束于 `0x2807'ffff`，CM7_1 可用区从 `0x2808'2000` 开始，正好保留 8 KiB 共享窗口且无重叠。CM7_0 map 含 `uart0_isr`，CM7_1 map 没有 `uart0_isr`。

## Step 4-6：硬件检查

未执行：环境没有连接目标板/调试器，未刷写三镜像，未进行电机禁用启动、UART 路由序列、MPU fault/heartbeat/调度间隙观测，也未执行带电机低速基线。板级时序、UART0 实际归属、MPU 运行时行为及电机安全语义仍需硬件确认。

## Step 7：仓库状态

`git status --short --branch` 与 `git log --oneline -7` 显示目标工作树仅增加本验证报告；源代码和工程文件均已在基线提交。本报告作为 Task 7 验证证据提交。
