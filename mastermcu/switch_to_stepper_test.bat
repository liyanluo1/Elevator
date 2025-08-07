@echo off
echo ========================================
echo 切换到步进电机测试模式
echo ========================================

REM 备份原始main.c
if exist Core\Src\main.c (
    echo 备份 main.c 到 main_original.c
    move /Y Core\Src\main.c Core\Src\main_original.c
)

REM 使用模块化的main
echo 激活 main_modular.c
copy /Y Core\Src\main_modular.c Core\Src\main.c

REM 临时重命名不需要的文件（添加.bak扩展名）
echo.
echo 禁用不需要的模块...

REM 禁用电梯系统模块
if exist Core\Modules\Global_bb\blackboard.c ren Core\Modules\Global_bb\blackboard.c blackboard.c.bak
if exist Core\Modules\Global_FSM\elevator_fsm.c ren Core\Modules\Global_FSM\elevator_fsm.c elevator_fsm.c.bak
if exist Core\Modules\Button\button_handler.c ren Core\Modules\Button\button_handler.c button_handler.c.bak

REM 禁用RS485
if exist Core\Modules\RS485\rs485_master.c ren Core\Modules\RS485\rs485_master.c rs485_master.c.bak
for %%f in (Core\Modules\RS485_Common\Core\*.c) do ren "%%f" "%%~nxf.bak"
for %%f in (Core\Modules\RS485_Common\Protocol\*.c) do ren "%%f" "%%~nxf.bak"
for %%f in (Core\Modules\RS485_Common\Utils\*.c) do ren "%%f" "%%~nxf.bak"

REM 禁用高级电机控制
if exist Core\Src\motor_advanced.c ren Core\Src\motor_advanced.c motor_advanced.c.bak
if exist Core\Src\calibration.c ren Core\Src\calibration.c calibration.c.bak
if exist Core\Modules\Stepper\FSM\motor_fsm.c ren Core\Modules\Stepper\FSM\motor_fsm.c motor_fsm.c.bak

REM 禁用显示模块
if exist Core\Modules\oled\oled.c ren Core\Modules\oled\oled.c oled.c.bak
for %%f in (Core\Src\u8g2src\*.c) do ren "%%f" "%%~nxf.bak"

REM 禁用通用工具（如果不需要）
for %%f in (Core\Modules\Common_Utils\*.c) do ren "%%f" "%%~nxf.bak"

echo.
echo 步进电机测试模式已激活！
echo.
echo 请在 STM32CubeIDE 中：
echo 1. 清理项目 (Project -^> Clean...)
echo 2. 重新编译 (Project -^> Build Project)
echo.
pause