@echo off
echo ========================================
echo 禁用不需要的模块（保留步进电机模块）
echo ========================================

echo.
echo 保留以下模块：
echo - main.c (原始主程序)
echo - 所有Stepper相关模块
echo - CAN通信模块
echo.

echo 禁用不需要的模块...

REM === 禁用电梯系统核心模块 ===
echo [1/6] 禁用电梯系统核心...
if exist Core\Modules\Global_bb\blackboard.c ren Core\Modules\Global_bb\blackboard.c blackboard.c_
if exist Core\Modules\Global_FSM\elevator_fsm.c ren Core\Modules\Global_FSM\elevator_fsm.c elevator_fsm.c_
if exist Core\Modules\Button\button_handler.c ren Core\Modules\Button\button_handler.c button_handler.c_

REM === 禁用RS485通信 ===
echo [2/6] 禁用RS485通信...
if exist Core\Modules\RS485\rs485_master.c ren Core\Modules\RS485\rs485_master.c rs485_master.c_
for %%f in (Core\Modules\RS485_Common\Core\*.c) do ren "%%f" "%%~nxf_"
for %%f in (Core\Modules\RS485_Common\Protocol\*.c) do ren "%%f" "%%~nxf_"
for %%f in (Core\Modules\RS485_Common\Utils\*.c) do ren "%%f" "%%~nxf_"

REM === 禁用显示模块 ===
echo [3/6] 禁用显示模块...
if exist Core\Modules\oled\oled.c ren Core\Modules\oled\oled.c oled.c_

REM === 禁用u8g2显示库（文件很多） ===
echo [4/6] 禁用u8g2显示库...
for %%f in (Core\Src\u8g2src\*.c) do ren "%%f" "%%~nxf_"

REM === 禁用通用工具模块 ===
echo [5/6] 禁用通用工具...
for %%f in (Core\Modules\Common_Utils\*.c) do ren "%%f" "%%~nxf_"

REM === 禁用其他不需要的源文件 ===
echo [6/6] 禁用其他模块...
if exist Core\Src\delay.c ren Core\Src\delay.c delay.c_
if exist Core\Src\usart.c ren Core\Src\usart.c usart.c_

REM === 保留必需文件 ===
echo.
echo ========================================
echo 以下文件保持启用状态：
echo - Core\Src\main.c (原始主程序)
echo - Core\Src\stm32f4xx_hal_msp.c
echo - Core\Src\stm32f4xx_it.c
echo - Core\Src\system_stm32f4xx.c
echo - Core\Src\motor_advanced.c
echo - Core\Src\calibration.c
echo - Core\Modules\CAN\can.c (CAN通信)
echo - Core\Modules\Stepper\* (所有步进电机模块)
echo - STM32 HAL驱动文件
echo ========================================

echo.
echo 步进电机测试环境已准备就绪！
echo.
echo 下一步：
echo 1. 在 STM32CubeIDE 中刷新项目 (F5)
echo 2. 清理项目 (Project -^> Clean...)
echo 3. 重新编译 (Project -^> Build Project)
echo.
echo 注意：
echo - 在 main.c 中编写或调用步进电机测试代码
echo - 如需恢复，运行 restore_all_modules.bat
echo.
pause