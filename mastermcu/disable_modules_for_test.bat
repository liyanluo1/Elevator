@echo off
echo ========================================
echo 禁用不需要的模块（改为.c_后缀）
echo ========================================

REM 备份原始main.c并使用测试版本
if exist Core\Src\main.c (
    echo 禁用原始 main.c
    ren Core\Src\main.c main.c_
)

REM 使用简化的步进电机测试主程序
if exist Core\Src\main_stepper_simple.c (
    echo 启用步进电机测试 main
    copy /Y Core\Src\main_stepper_simple.c Core\Src\main.c
)

echo.
echo 禁用不需要的模块...

REM === 禁用电梯系统核心模块 ===
echo [1/8] 禁用电梯系统核心...
if exist Core\Modules\Global_bb\blackboard.c ren Core\Modules\Global_bb\blackboard.c blackboard.c_
if exist Core\Modules\Global_FSM\elevator_fsm.c ren Core\Modules\Global_FSM\elevator_fsm.c elevator_fsm.c_
if exist Core\Modules\Button\button_handler.c ren Core\Modules\Button\button_handler.c button_handler.c_

REM === 禁用RS485通信 ===
echo [2/8] 禁用RS485通信...
if exist Core\Modules\RS485\rs485_master.c ren Core\Modules\RS485\rs485_master.c rs485_master.c_
for %%f in (Core\Modules\RS485_Common\Core\*.c) do ren "%%f" "%%~nxf_"
for %%f in (Core\Modules\RS485_Common\Protocol\*.c) do ren "%%f" "%%~nxf_"
for %%f in (Core\Modules\RS485_Common\Utils\*.c) do ren "%%f" "%%~nxf_"

REM === 禁用高级电机控制和校准 ===
echo [3/8] 禁用高级电机控制...
if exist Core\Src\motor_advanced.c ren Core\Src\motor_advanced.c motor_advanced.c_
if exist Core\Src\calibration.c ren Core\Src\calibration.c calibration.c_
for %%f in (Core\Modules\Stepper\FSM\*.c) do ren "%%f" "%%~nxf_"
for %%f in (Core\Modules\Stepper\Adapter\*.c) do ren "%%f" "%%~nxf_"
for %%f in (Core\Modules\Stepper\CANopen\*.c) do ren "%%f" "%%~nxf_"

REM === 禁用显示模块 ===
echo [4/8] 禁用显示模块...
if exist Core\Modules\oled\oled.c ren Core\Modules\oled\oled.c oled.c_

REM === 禁用u8g2显示库（文件很多） ===
echo [5/8] 禁用u8g2显示库...
for %%f in (Core\Src\u8g2src\*.c) do ren "%%f" "%%~nxf_"

REM === 禁用通用工具模块 ===
echo [6/8] 禁用通用工具...
for %%f in (Core\Modules\Common_Utils\*.c) do ren "%%f" "%%~nxf_"

REM === 禁用其他不需要的源文件 ===
echo [7/8] 禁用其他模块...
if exist Core\Src\delay.c ren Core\Src\delay.c delay.c_
if exist Core\Src\usart.c ren Core\Src\usart.c usart.c_

REM === 保留必需文件 ===
echo [8/8] 检查必需文件...
echo.
echo 以下文件保持启用状态：
echo - Core\Src\main.c (测试主程序)
echo - Core\Src\stm32f4xx_hal_msp.c
echo - Core\Src\stm32f4xx_it.c
echo - Core\Src\system_stm32f4xx.c
echo - Core\Modules\CAN\can.c (CAN通信)
echo - STM32 HAL驱动文件

echo.
echo ========================================
echo 步进电机测试模式已激活！
echo ========================================
echo.
echo 下一步：
echo 1. 在 STM32CubeIDE 中刷新项目 (F5)
echo 2. 清理项目 (Project -^> Clean...)
echo 3. 重新编译 (Project -^> Build Project)
echo.
echo 注意：如需恢复，运行 restore_all_modules.bat
echo.
pause