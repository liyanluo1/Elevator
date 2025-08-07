@echo off
echo ========================================
echo 恢复所有模块（.c_改回.c）
echo ========================================

REM 恢复原始main.c
if exist Core\Src\main.c_ (
    echo 恢复原始 main.c
    if exist Core\Src\main.c del Core\Src\main.c
    ren Core\Src\main.c_ main.c
)

echo.
echo 恢复所有模块...

REM === 恢复电梯系统核心模块 ===
echo [1/8] 恢复电梯系统核心...
if exist Core\Modules\Global_bb\blackboard.c_ ren Core\Modules\Global_bb\blackboard.c_ blackboard.c
if exist Core\Modules\Global_FSM\elevator_fsm.c_ ren Core\Modules\Global_FSM\elevator_fsm.c_ elevator_fsm.c
if exist Core\Modules\Button\button_handler.c_ ren Core\Modules\Button\button_handler.c_ button_handler.c

REM === 恢复RS485通信 ===
echo [2/8] 恢复RS485通信...
if exist Core\Modules\RS485\rs485_master.c_ ren Core\Modules\RS485\rs485_master.c_ rs485_master.c
for %%f in (Core\Modules\RS485_Common\Core\*.c_) do (
    set "filename=%%~nf"
    ren "%%f" "!filename:~0,-2!.c"
)
for %%f in (Core\Modules\RS485_Common\Protocol\*.c_) do (
    set "filename=%%~nf"
    ren "%%f" "!filename:~0,-2!.c"
)
for %%f in (Core\Modules\RS485_Common\Utils\*.c_) do (
    set "filename=%%~nf"
    ren "%%f" "!filename:~0,-2!.c"
)

REM === 恢复高级电机控制和校准 ===
echo [3/8] 恢复高级电机控制...
if exist Core\Src\motor_advanced.c_ ren Core\Src\motor_advanced.c_ motor_advanced.c
if exist Core\Src\calibration.c_ ren Core\Src\calibration.c_ calibration.c
for %%f in (Core\Modules\Stepper\FSM\*.c_) do (
    set "filename=%%~nf"
    ren "%%f" "!filename:~0,-2!.c"
)
for %%f in (Core\Modules\Stepper\Adapter\*.c_) do (
    set "filename=%%~nf"
    ren "%%f" "!filename:~0,-2!.c"
)
for %%f in (Core\Modules\Stepper\CANopen\*.c_) do (
    set "filename=%%~nf"
    ren "%%f" "!filename:~0,-2!.c"
)

REM === 恢复显示模块 ===
echo [4/8] 恢复显示模块...
if exist Core\Modules\oled\oled.c_ ren Core\Modules\oled\oled.c_ oled.c

REM === 恢复u8g2显示库 ===
echo [5/8] 恢复u8g2显示库...
for %%f in (Core\Src\u8g2src\*.c_) do (
    set "filename=%%~nf"
    ren "%%f" "!filename:~0,-2!.c"
)

REM === 恢复通用工具模块 ===
echo [6/8] 恢复通用工具...
for %%f in (Core\Modules\Common_Utils\*.c_) do (
    set "filename=%%~nf"
    ren "%%f" "!filename:~0,-2!.c"
)

REM === 恢复其他源文件 ===
echo [7/8] 恢复其他模块...
if exist Core\Src\delay.c_ ren Core\Src\delay.c_ delay.c
if exist Core\Src\usart.c_ ren Core\Src\usart.c_ usart.c

echo [8/8] 完成！

echo.
echo ========================================
echo 所有模块已恢复！
echo ========================================
echo.
echo 下一步：
echo 1. 在 STM32CubeIDE 中刷新项目 (F5)
echo 2. 清理项目 (Project -^> Clean...)
echo 3. 重新编译 (Project -^> Build Project)
echo.
pause