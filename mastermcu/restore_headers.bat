@echo off
echo ========================================
echo 恢复所有被禁用的头文件
echo ========================================

echo.
echo 恢复头文件...

REM === 恢复电梯系统核心模块头文件 ===
echo [1/5] 恢复电梯系统核心头文件...
if exist Core\Modules\Global_bb\blackboard.h_ ren Core\Modules\Global_bb\blackboard.h_ blackboard.h
if exist Core\Modules\Global_FSM\elevator_fsm.h_ ren Core\Modules\Global_FSM\elevator_fsm.h_ elevator_fsm.h
if exist Core\Modules\Button\button_handler.h_ ren Core\Modules\Button\button_handler.h_ button_handler.h

REM === 恢复RS485通信头文件 ===
echo [2/5] 恢复RS485通信头文件...
if exist Core\Modules\RS485\rs485_master.h_ ren Core\Modules\RS485\rs485_master.h_ rs485_master.h
if exist Core\Modules\RS485_Common\Core\rs485_core.h_ ren Core\Modules\RS485_Common\Core\rs485_core.h_ rs485_core.h
if exist Core\Modules\RS485_Common\Core\rs485_base.h_ ren Core\Modules\RS485_Common\Core\rs485_base.h_ rs485_base.h
if exist Core\Modules\RS485_Common\Protocol\rs485_protocol.h_ ren Core\Modules\RS485_Common\Protocol\rs485_protocol.h_ rs485_protocol.h
if exist Core\Modules\RS485_Common\Utils\rs485_crc.h_ ren Core\Modules\RS485_Common\Utils\rs485_crc.h_ rs485_crc.h

REM === 恢复显示模块头文件 ===
echo [3/5] 恢复显示模块头文件...
if exist Core\Modules\oled\oled.h_ ren Core\Modules\oled\oled.h_ oled.h

REM === 恢复u8g2显示库头文件 ===
echo [4/5] 恢复u8g2显示库头文件...
for %%f in (Core\Inc\u8g2src\*.h_) do (
    set "filename=%%~nf"
    ren "%%f" "!filename:~0,-1!%%~xf"
)

REM === 恢复通用工具模块头文件 ===
echo [5/5] 恢复通用工具头文件...
if exist Core\Modules\Common_Utils\gpio_utils.h_ ren Core\Modules\Common_Utils\gpio_utils.h_ gpio_utils.h
if exist Core\Modules\Common_Utils\debounce.h_ ren Core\Modules\Common_Utils\debounce.h_ debounce.h
if exist Core\Modules\Common_Utils\ring_buffer.h_ ren Core\Modules\Common_Utils\ring_buffer.h_ ring_buffer.h

echo.
echo ========================================
echo 所有头文件已恢复！
echo ========================================

echo.
echo 下一步：
echo 1. 在 STM32CubeIDE 中刷新项目 (F5)
echo 2. 清理项目 (Project -^> Clean...)
echo 3. 重新编译 (Project -^> Build Project)
echo.
pause