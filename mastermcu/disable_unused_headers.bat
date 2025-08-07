@echo off
echo ========================================
echo 禁用不需要的头文件（保留步进电机相关）
echo ========================================

echo.
echo 禁用不需要的头文件...

REM === 禁用电梯系统核心模块头文件 ===
echo [1/5] 禁用电梯系统核心头文件...
if exist Core\Modules\Global_bb\blackboard.h ren Core\Modules\Global_bb\blackboard.h blackboard.h_
if exist Core\Modules\Global_FSM\elevator_fsm.h ren Core\Modules\Global_FSM\elevator_fsm.h elevator_fsm.h_
if exist Core\Modules\Button\button_handler.h ren Core\Modules\Button\button_handler.h button_handler.h_

REM === 禁用RS485通信头文件 ===
echo [2/5] 禁用RS485通信头文件...
if exist Core\Modules\RS485\rs485_master.h ren Core\Modules\RS485\rs485_master.h rs485_master.h_
if exist Core\Modules\RS485_Common\Core\rs485_core.h ren Core\Modules\RS485_Common\Core\rs485_core.h rs485_core.h_
if exist Core\Modules\RS485_Common\Core\rs485_base.h ren Core\Modules\RS485_Common\Core\rs485_base.h rs485_base.h_
if exist Core\Modules\RS485_Common\Protocol\rs485_protocol.h ren Core\Modules\RS485_Common\Protocol\rs485_protocol.h rs485_protocol.h_
if exist Core\Modules\RS485_Common\Utils\rs485_crc.h ren Core\Modules\RS485_Common\Utils\rs485_crc.h rs485_crc.h_

REM === 禁用显示模块头文件 ===
echo [3/5] 禁用显示模块头文件...
if exist Core\Modules\oled\oled.h ren Core\Modules\oled\oled.h oled.h_

REM === 禁用u8g2显示库头文件 ===
echo [4/5] 禁用u8g2显示库头文件...
for %%f in (Core\Inc\u8g2src\*.h) do (
    if exist "%%f" ren "%%f" "%%~nxf_"
)

REM === 禁用通用工具模块头文件 ===
echo [5/5] 禁用通用工具头文件...
if exist Core\Modules\Common_Utils\gpio_utils.h ren Core\Modules\Common_Utils\gpio_utils.h gpio_utils.h_
if exist Core\Modules\Common_Utils\debounce.h ren Core\Modules\Common_Utils\debounce.h debounce.h_
if exist Core\Modules\Common_Utils\ring_buffer.h ren Core\Modules\Common_Utils\ring_buffer.h ring_buffer.h_

echo.
echo ========================================
echo 以下头文件保持启用状态：
echo - Core\Modules\CAN\can.h (CAN通信)
echo - Core\Modules\Stepper\* (所有步进电机相关头文件)
echo - Core\Modules\Common_Types\elevator_types.h (可能被Stepper使用)
echo - STM32 HAL头文件
echo ========================================

echo.
echo 头文件禁用完成！
echo.
echo 下一步：
echo 1. 在 STM32CubeIDE 中刷新项目 (F5)
echo 2. 清理项目 (Project -^> Clean...)
echo 3. 重新编译 (Project -^> Build Project)
echo.
echo 注意：如需恢复，需要运行相应的恢复脚本
echo.
pause