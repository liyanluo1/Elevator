@echo off
echo ========================================
echo 恢复原始配置
echo ========================================

REM 恢复原始main.c
if exist Core\Src\main_original.c (
    echo 恢复原始 main.c
    move /Y Core\Src\main_original.c Core\Src\main.c
)

REM 恢复所有.bak文件
echo.
echo 恢复所有模块...

REM 恢复电梯系统模块
if exist Core\Modules\Global_bb\blackboard.c.bak ren Core\Modules\Global_bb\blackboard.c.bak blackboard.c
if exist Core\Modules\Global_FSM\elevator_fsm.c.bak ren Core\Modules\Global_FSM\elevator_fsm.c.bak elevator_fsm.c
if exist Core\Modules\Button\button_handler.c.bak ren Core\Modules\Button\button_handler.c.bak button_handler.c

REM 恢复RS485
if exist Core\Modules\RS485\rs485_master.c.bak ren Core\Modules\RS485\rs485_master.c.bak rs485_master.c
for %%f in (Core\Modules\RS485_Common\Core\*.c.bak) do (
    set "filename=%%~nf"
    ren "%%f" "!filename!"
)
for %%f in (Core\Modules\RS485_Common\Protocol\*.c.bak) do (
    set "filename=%%~nf"
    ren "%%f" "!filename!"
)
for %%f in (Core\Modules\RS485_Common\Utils\*.c.bak) do (
    set "filename=%%~nf"
    ren "%%f" "!filename!"
)

REM 恢复高级电机控制
if exist Core\Src\motor_advanced.c.bak ren Core\Src\motor_advanced.c.bak motor_advanced.c
if exist Core\Src\calibration.c.bak ren Core\Src\calibration.c.bak calibration.c
if exist Core\Modules\Stepper\FSM\motor_fsm.c.bak ren Core\Modules\Stepper\FSM\motor_fsm.c.bak motor_fsm.c

REM 恢复显示模块
if exist Core\Modules\oled\oled.c.bak ren Core\Modules\oled\oled.c.bak oled.c
for %%f in (Core\Src\u8g2src\*.c.bak) do (
    set "filename=%%~nf"
    ren "%%f" "!filename!"
)

REM 恢复通用工具
for %%f in (Core\Modules\Common_Utils\*.c.bak) do (
    set "filename=%%~nf"
    ren "%%f" "!filename!"
)

echo.
echo 原始配置已恢复！
echo.
echo 请在 STM32CubeIDE 中：
echo 1. 清理项目 (Project -^> Clean...)
echo 2. 重新编译 (Project -^> Build Project)
echo.
pause