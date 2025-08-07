@echo off
echo =========================================
echo Disabling non-servo modules for testing
echo =========================================

cd /d "%~dp0"

echo.
echo Disabling keyboard module...
if exist "Core\Modules\keyboard\keyboard.c" ren "Core\Modules\keyboard\keyboard.c" "keyboard.c._"
if exist "Core\Modules\keyboard\keyboard.h" ren "Core\Modules\keyboard\keyboard.h" "keyboard.h._"

echo Disabling LED module...
if exist "Core\Modules\LED\LED.c" ren "Core\Modules\LED\LED.c" "LED.c._"
if exist "Core\Modules\LED\LED.h" ren "Core\Modules\LED\LED.h" "LED.h._"

echo Disabling PhotoSensor module...
if exist "Core\Modules\PhotoSensor\photo_sensor.c" ren "Core\Modules\PhotoSensor\photo_sensor.c" "photo_sensor.c._"
if exist "Core\Modules\PhotoSensor\photo_sensor.h" ren "Core\Modules\PhotoSensor\photo_sensor.h" "photo_sensor.h._"

echo Disabling RS485 slave module...
if exist "Core\Modules\Local_RS485\rs485_slave_adapter.c" ren "Core\Modules\Local_RS485\rs485_slave_adapter.c" "rs485_slave_adapter.c._"
if exist "Core\Modules\Local_RS485\rs485_slave_adapter.h" ren "Core\Modules\Local_RS485\rs485_slave_adapter.h" "rs485_slave_adapter.h._"

echo Disabling RS485 Common modules...
if exist "Core\Modules\RS485_Common\Core\rs485_core.c" ren "Core\Modules\RS485_Common\Core\rs485_core.c" "rs485_core.c._"
if exist "Core\Modules\RS485_Common\Core\rs485_core.h" ren "Core\Modules\RS485_Common\Core\rs485_core.h" "rs485_core.h._"
if exist "Core\Modules\RS485_Common\Core\rs485_base.h" ren "Core\Modules\RS485_Common\Core\rs485_base.h" "rs485_base.h._"
if exist "Core\Modules\RS485_Common\Protocol\rs485_protocol.c" ren "Core\Modules\RS485_Common\Protocol\rs485_protocol.c" "rs485_protocol.c._"
if exist "Core\Modules\RS485_Common\Protocol\rs485_protocol.h" ren "Core\Modules\RS485_Common\Protocol\rs485_protocol.h" "rs485_protocol.h._"
if exist "Core\Modules\RS485_Common\Utils\rs485_crc.c" ren "Core\Modules\RS485_Common\Utils\rs485_crc.c" "rs485_crc.c._"
if exist "Core\Modules\RS485_Common\Utils\rs485_crc.h" ren "Core\Modules\RS485_Common\Utils\rs485_crc.h" "rs485_crc.h._"

echo Disabling Local blackboard module...
if exist "Core\Modules\Local_BB\local_blackboard.c" ren "Core\Modules\Local_BB\local_blackboard.c" "local_blackboard.c._"
if exist "Core\Modules\Local_BB\local_blackboard.h" ren "Core\Modules\Local_BB\local_blackboard.h" "local_blackboard.h._"

echo Disabling Common Utils (except what servo might need)...
if exist "Core\Modules\Common_Utils\debounce.c" ren "Core\Modules\Common_Utils\debounce.c" "debounce.c._"
if exist "Core\Modules\Common_Utils\debounce.h" ren "Core\Modules\Common_Utils\debounce.h" "debounce.h._"
if exist "Core\Modules\Common_Utils\ring_buffer.c" ren "Core\Modules\Common_Utils\ring_buffer.c" "ring_buffer.c._"
if exist "Core\Modules\Common_Utils\ring_buffer.h" ren "Core\Modules\Common_Utils\ring_buffer.h" "ring_buffer.h._"

echo.
echo =========================================
echo Non-servo modules disabled successfully!
echo Servo module files remain active:
echo - servo.c/h
echo - servo_control.c/h
echo =========================================
echo.
echo Please clean and rebuild the project in STM32CubeIDE.
pause