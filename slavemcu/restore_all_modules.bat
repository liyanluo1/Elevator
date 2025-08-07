@echo off
echo =========================================
echo Restoring all disabled modules
echo =========================================

cd /d "%~dp0"

echo.
echo Restoring keyboard module...
if exist "Core\Modules\keyboard\keyboard.c._" ren "Core\Modules\keyboard\keyboard.c._" "keyboard.c"
if exist "Core\Modules\keyboard\keyboard.h._" ren "Core\Modules\keyboard\keyboard.h._" "keyboard.h"

echo Restoring LED module...
if exist "Core\Modules\LED\LED.c._" ren "Core\Modules\LED\LED.c._" "LED.c"
if exist "Core\Modules\LED\LED.h._" ren "Core\Modules\LED\LED.h._" "LED.h"

echo Restoring PhotoSensor module...
if exist "Core\Modules\PhotoSensor\photo_sensor.c._" ren "Core\Modules\PhotoSensor\photo_sensor.c._" "photo_sensor.c"
if exist "Core\Modules\PhotoSensor\photo_sensor.h._" ren "Core\Modules\PhotoSensor\photo_sensor.h._" "photo_sensor.h"

echo Restoring RS485 slave module...
if exist "Core\Modules\Local_RS485\rs485_slave_adapter.c._" ren "Core\Modules\Local_RS485\rs485_slave_adapter.c._" "rs485_slave_adapter.c"
if exist "Core\Modules\Local_RS485\rs485_slave_adapter.h._" ren "Core\Modules\Local_RS485\rs485_slave_adapter.h._" "rs485_slave_adapter.h"

echo Restoring RS485 Common modules...
if exist "Core\Modules\RS485_Common\Core\rs485_core.c._" ren "Core\Modules\RS485_Common\Core\rs485_core.c._" "rs485_core.c"
if exist "Core\Modules\RS485_Common\Core\rs485_core.h._" ren "Core\Modules\RS485_Common\Core\rs485_core.h._" "rs485_core.h"
if exist "Core\Modules\RS485_Common\Core\rs485_base.h._" ren "Core\Modules\RS485_Common\Core\rs485_base.h._" "rs485_base.h"
if exist "Core\Modules\RS485_Common\Protocol\rs485_protocol.c._" ren "Core\Modules\RS485_Common\Protocol\rs485_protocol.c._" "rs485_protocol.c"
if exist "Core\Modules\RS485_Common\Protocol\rs485_protocol.h._" ren "Core\Modules\RS485_Common\Protocol\rs485_protocol.h._" "rs485_protocol.h"
if exist "Core\Modules\RS485_Common\Utils\rs485_crc.c._" ren "Core\Modules\RS485_Common\Utils\rs485_crc.c._" "rs485_crc.c"
if exist "Core\Modules\RS485_Common\Utils\rs485_crc.h._" ren "Core\Modules\RS485_Common\Utils\rs485_crc.h._" "rs485_crc.h"

echo Restoring Local blackboard module...
if exist "Core\Modules\Local_BB\local_blackboard.c._" ren "Core\Modules\Local_BB\local_blackboard.c._" "local_blackboard.c"
if exist "Core\Modules\Local_BB\local_blackboard.h._" ren "Core\Modules\Local_BB\local_blackboard.h._" "local_blackboard.h"

echo Restoring Common Utils...
if exist "Core\Modules\Common_Utils\debounce.c._" ren "Core\Modules\Common_Utils\debounce.c._" "debounce.c"
if exist "Core\Modules\Common_Utils\debounce.h._" ren "Core\Modules\Common_Utils\debounce.h._" "debounce.h"
if exist "Core\Modules\Common_Utils\ring_buffer.c._" ren "Core\Modules\Common_Utils\ring_buffer.c._" "ring_buffer.c"
if exist "Core\Modules\Common_Utils\ring_buffer.h._" ren "Core\Modules\Common_Utils\ring_buffer.h._" "ring_buffer.h"
if exist "Core\Modules\Common_Utils\gpio_utils.c._" ren "Core\Modules\Common_Utils\gpio_utils.c._" "gpio_utils.c"
if exist "Core\Modules\Common_Utils\gpio_utils.h._" ren "Core\Modules\Common_Utils\gpio_utils.h._" "gpio_utils.h"

echo Restoring servo_control files...
if exist "Core\Modules\servo\servo_control.c._" ren "Core\Modules\servo\servo_control.c._" "servo_control.c"
if exist "Core\Modules\servo\servo_control.h._" ren "Core\Modules\servo\servo_control.h._" "servo_control.h"

echo.
echo =========================================
echo All modules restored successfully!
echo =========================================
echo.
echo Please clean and rebuild the project in STM32CubeIDE.
pause