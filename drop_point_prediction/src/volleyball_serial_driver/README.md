# volleyball_serial_driver

# 排球机器人串口数据包协议

## robotArray - RX 需接受数据

| Byte | 类型 | Data | 说明 |
| - | - | - | - |
| 0 | uint8 | 0xAA | 包头 |
| 1 | uint8 | mode | 执行模式 |
| 2-5 | float32 | x | 机器人位置，单位m |
| 6-9 | float32 | y | - |
| 10-13| float32 | yaw | 机器人朝向航向角，单位弧度|
| 14-17 | float32 | pitch | 机器人delta机械臂俯仰角，单位弧度|
| 18 | uint8 | xor | 异或校验位 |
| 19 | uint8 | 0x55 | 包尾 |

### 枚举说明
`mode` 说明：
- 0 IDLE 待机等待指示
- 1 REMOTE 遥控接管中
- 2 SELF 自主接球中

## planArray - TX 发送内容
由于实际有多个命令：移动规划,这里先列出总的框架：

| Byte | 类型 | Data | 说明 | 
| - | - | - | - |
| 0 | uint8 | 0xAA | 包头 | 
| 1 | uint8 | cmd | 指令位 |
| 2-3 | uint16 | len | 数据内容长度（在该字节之后数据字节长度，不包括校验位与包尾） |
| ... |
| -2 | uint8 | xor | 异或校验位 | 
| -1 | uint8 | 0x55 | 包尾 |

### 枚举说明
`cmd`：
- 0 move_plan 移动规划

### 移动规划
| Byte | 类型 | Data | 说明 | 
| - | - | - | - |
| 0 | uint8 | 0xAA | 包头 | 
| 1 | uint8 | cmd | 指令位 |
| 2-3 | uint16 | len | 数据内容长度，固定为4*4 = 16
| 4-7 | float32 | x | 接球位置，单位m |
| 8-11 | float32 | y | - |
| 12 | uint8 | is_hit | 是否击球 | 
| 13-16 | float32 | time | 距离球到合适击球点的预测时间，单位s | 
| 17 | uint8 | xor | 异或校验位 | 
| 18  | uint8 | 0x55 | 包尾 |



