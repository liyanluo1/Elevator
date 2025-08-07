# 步进电机测试编译配置

## 需要保留的文件（必须编译）：

### Core/Src/
- main_stepper_test.c （新创建的测试主文件）
- stm32f4xx_hal_msp.c （HAL MSP初始化）
- stm32f4xx_it.c （中断处理）
- system_stm32f4xx.c （系统初始化）

### Core/Modules/
- CAN/can.c （CAN通信必需）

### Drivers/
- 所有STM32 HAL驱动文件（保持默认）

## 需要排除的文件（不编译）：

### Core/Src/
- main.c （原主文件，与main_stepper_test.c冲突）
- calibration.c
- motor_advanced.c
- delay.c
- usart.c
- 整个u8g2src/文件夹（所有显示相关文件）

### Core/Modules/
- Button/button_handler.c
- Common_Utils/（整个文件夹）
- Global_bb/blackboard.c
- Global_FSM/elevator_fsm.c
- oled/oled.c
- RS485/（整个文件夹）
- RS485_Common/（整个文件夹）
- Stepper/（整个文件夹，因为我们直接使用CAN命令）

## 在STM32CubeIDE中的操作步骤：

1. 右键点击项目 -> Properties
2. 选择 C/C++ Build -> Settings
3. 在 Tool Settings 标签下，选择 MCU GCC Compiler -> Preprocessor
4. 添加宏定义：STEPPER_TEST_MODE

或者：

1. 右键点击要排除的文件/文件夹
2. 选择 Resource Configurations -> Exclude from Build...
3. 勾选当前配置
4. 点击OK

## 编译步骤：

1. 清理项目：Project -> Clean...
2. 构建项目：Project -> Build Project
3. 下载到MCU运行

## 测试说明：

程序会循环执行：
- LED亮：电机正转5秒（低速，最大力矩）
- LED灭：电机反转5秒（低速，最大力矩）
- 停止2秒
- 重复循环

## 注意事项：

1. 确保CAN总线连接正确
2. 电机驱动器已上电
3. 电机ID设置为0x01
4. CAN波特率匹配（当前配置为1Mbps）