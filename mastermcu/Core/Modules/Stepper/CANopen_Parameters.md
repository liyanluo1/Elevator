# CANopen 步进电机驱动器参数表

## 1. 重要控制参数

### 1.1 控制字 (0x6040)
控制电机的运行状态，关键值：
- `0x0000`: **完全断电/自由状态** (电机无锁力，可手动转动)
- `0x0006`: 脱机状态
- `0x0007`: 使能锁机
- `0x000F`: 标准使能运行
- `0x001F`: 开始移动（PP模式）
- `0x010F`: 停止
- `0x0080`: 故障复位

### 1.2 状态字 (0x6041)
读取电机状态：
- Bit 0: Ready to switch on
- Bit 1: Switched on  
- Bit 2: Operation enabled
- Bit 10: Target reached (0x0400)
- Bit 12: Setpoint acknowledge (0x1000)

### 1.3 工作模式 (0x6060)
- `1`: Profile Position Mode (PP模式，轮廓位置模式)
- `3`: Profile Velocity Mode (PV模式，轮廓速度模式)
- `4`: Torque Profile Mode (力矩模式)
- `6`: Homing Mode (回原点模式)
- `7`: Interpolated Position Mode (插补位置模式)
- `8`: Cyclic Synchronous Position Mode (CSP模式)

## 2. 电机配置参数

### 2.1 基本参数
- `0x2000`: 驱动器峰值电流 (mA)，默认1000mA
- `0x2001`: 驱动器细分数，默认4000（脉冲/圈）
- `0x2003`: 最小电流参数，默认0x1919
  - 高8位：运行时最小电流百分比
  - 低8位：空闲时最小电流百分比
- `0x2007`: 编码器线数，默认1000线
- `0x3200`: **电机使能选择**
  - `0`: 上电自动锁轴（默认）
  - `1`: 上电不锁轴

### 2.2 位置控制参数
- `0x607A`: 目标位置 (脉冲数)
- `0x6064`: 实际位置 (读取编码器，脉冲数)
- `0x6081`: 梯形速度 (脉冲/秒)
- `0x6083`: 梯形加速度 (脉冲/秒²)
- `0x6084`: 梯形减速度 (脉冲/秒²)

### 2.3 速度控制参数
- `0x60FF`: 目标速度 (脉冲/秒，符号表示方向)
- `0x606C`: 实际速度 (读取，脉冲/秒)

## 3. 解除锁力的正确方法

### 方法1：通过控制字直接控制（临时）
```c
// 完全释放锁力
uint16_t control_word = 0x0000;
StepperMotor_SendSDO(node_id, 0x6040, 0, (uint8_t*)&control_word, 2);
```

### 方法2：通过电机使能选择（永久）
```c
// 设置为上电不锁轴
uint8_t enable_mode = 1;
StepperMotor_SendSDO(node_id, 0x3200, 0, &enable_mode, 1);

// 保存参数（需要写入特定值）
uint32_t save_cmd = 0x65766173;  // "save"的ASCII码
StepperMotor_SendSDO(node_id, 0x1010, 4, (uint8_t*)&save_cmd, 4);
```

## 4. 常用操作序列

### 4.1 启用电机
```c
// 1. 故障复位
control_word = 0x0080;
// 2. 脱机
control_word = 0x0006;  
// 3. 使能锁机
control_word = 0x0007;
// 4. 使能运行
control_word = 0x000F;
```

### 4.2 位置移动（PP模式）
```c
// 1. 设置模式
mode = 1;  // PP模式
SendSDO(0x6060, mode);

// 2. 设置速度和加速度
SendSDO(0x6081, velocity);
SendSDO(0x6083, acceleration);

// 3. 设置目标位置
SendSDO(0x607A, target_position);

// 4. 启动移动
control_word = 0x001F;
SendSDO(0x6040, control_word);
```

### 4.3 读取编码器
```c
// 读取实际位置（编码器反馈）
ReadSDO(0x6064, &actual_position);
```

## 5. 注意事项

1. **锁力问题**：默认情况下电机上电自动锁轴，需要通过0x3200设置并保存才能改变默认行为
2. **保存参数**：修改0x3200后需要执行保存操作（0x1010:04 = 0x65766173）
3. **编码器读数**：0x6064返回的是实际编码器位置，用于闭环控制和位置反馈
4. **细分与编码器关系**：1000线编码器4倍频=4000脉冲/圈，与细分数匹配

## 6. 故障代码 (0x603F)
常见故障代码及含义（具体参见手册7.2节）