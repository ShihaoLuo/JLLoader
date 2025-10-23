# CAN电机控制API参考

## 核心API

### 启动电机
```c
bool AppCANProtocol_StartMotor(uint8_t motor_id, uint16_t speed, MotorDirection_t direction);
// motor_id: 1-20
// speed: 0 or 180-800 RPM
// direction: MOTOR_DIR_CW(顺时针) 或 MOTOR_DIR_CCW(逆时针)
```

### 停止电机
```c
bool AppCANProtocol_StopMotor(uint8_t motor_id);
```

### 设置转速
```c
bool AppCANProtocol_SetMotorSpeed(uint8_t motor_id, uint16_t speed);
```

### 查询电机状态
```c
bool AppCANProtocol_QueryMotorStatus(uint8_t motor_id);
MotorStatus_t* AppCANProtocol_GetMotorStatus(uint8_t motor_id);
// 返回: status, actual_speed, direction, current, temperature, fault
```

### 紧急停止所有
```c
bool AppCANProtocol_EmergencyStopAll(void);
```

### 通用控制（高级）
```c
bool AppCANProtocol_ControlMotor(uint8_t motor_id, MotorControl_t *control);
// 支持加减速、命令类型等参数
```

## 主循环框架

```c
int main(void) {
    App_Init();
    AppCANProtocol_Init(&hcan1);
    HAL_Delay(1000);
    AppCANProtocol_StartDiscovery();
    
    while (1) {
        Protocol_CheckPendingJump();
        App_CAN_ProcessUARTOutput();
        AppCANProtocol_Task();  // 必须保留
        
        // 在这里添加你的电机控制代码
        // 例如: AppCANProtocol_StartMotor(1, 300, MOTOR_DIR_CW);
        
        HAL_Delay(10);
    }
}
```

## 重要参数

| 参数 | 值 |
|------|-----|
| 最大转速 | 800 RPM |
| 最低转速 | 0 或者 180 RPM |
| 电机ID | 1-20 |
| 方向常量 | MOTOR_DIR_CW / MOTOR_DIR_CCW |
| 心跳超时 | 3000 ms |

## 常见修改

### 改转速
```c
AppCANProtocol_SetMotorSpeed(motor_id, 600);
```

### 改方向
```c
AppCANProtocol_StartMotor(1, 300, MOTOR_DIR_CCW);
```

### 改电机ID
```c
AppCANProtocol_StartMotor(2, 300, MOTOR_DIR_CW);
```

## 查询状态

```c
AppCANProtocol_QueryMotorStatus(1);
HAL_Delay(100);  // 等待响应
MotorStatus_t *status = AppCANProtocol_GetMotorStatus(1);
if (status) {
    uint16_t speed = status->actual_speed;
    uint8_t temp = status->temperature;
}
```

## 修改检查清单

- [ ] 保留 App_Init() 和 AppCANProtocol_Init(&hcan1)
- [ ] 主循环保留 AppCANProtocol_Task()
- [ ] 电机ID在 1-20 范围
- [ ] 转速在 0-800 范围
- [ ] 方向使用 MOTOR_DIR_CW 或 MOTOR_DIR_CCW
