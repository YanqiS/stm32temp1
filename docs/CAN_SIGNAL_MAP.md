# CAN 信号/报文控制说明

> 本文根据 `main.c` 当前实现整理，描述固件实际解析/发送的 CAN 标准帧。所有字节序均按代码中的 `buf_rec[0]` ~ `buf_rec[7]` 表示。

## 总览

### CAN1 接收（上位机/测试工具发给板子）

| CAN ID | 名称/用途 | 主要控制对象 | 处理结果 |
| --- | --- | --- | --- |
| `0x052` | `TSA_3` | KL15、USB、钥匙/按键、继电器 1~6 | 更新 `TA531SysEnv`，置 `TSA3_0x52_Flag`，ACK Byte3 |
| `0x053` | `TSA_4` | 车窗、门开关、HSD 输出 | 更新门/继电器状态，后续 `Door_Control()` 驱动 GPIO，ACK Byte4 |
| `0x054` | `TSA_PWM` | 舵机/PWM 角度 | 更新 `PWM_1_Ag`~`PWM_8_Ag`，前 4 路会输出到 TIM2/TIM3 PWM |
| `0x064` | `TSA_RC1 mm` | 触屏机器人 XY 目标，单位 mm | 以 `X0/Y0` 为原点映射到目标坐标 |
| `0x065` | `TSA_RC1p %` | 触屏机器人 XY 目标，百分比 | 按 `id4` 选择 `/100` 或 `/255` 映射到 `X0~X1/Y0~Y1` |
| `0x103` | `TSA_LIN_SWS G2.5` | LIN SWS G2.5 信号桥接 | 更新 `TA531_LIN_SWS`，ACK Byte5 |
| `0x104` | `TSA_LIN_SWS G3.0` | LIN SWS G3.0 信号桥接 | 更新 `TA531_LIN_SWS_G3`，ACK Byte5 |
| `0x105` | `EBS LIN FrP00 bridge` | EBS/LIN 电池相关信号 | 更新 EBS 原始值，ACK Byte6 |
| `0x531` | `TSA_ACK for RESET` | 远程状态复位确认 | 数据全 0 时清 `Remote_state`/锁定状态 |

### CAN1 发送（板子发给上位机）

| CAN ID | 名称/用途 | 数据含义 |
| --- | --- | --- |
| `0x531` | `TSA_Ack` | 通用 ACK，各 Byte 对应不同接收报文处理完成标志 |
| `0x050` | `TSA1_ADC` | ADC/传感器上报（代码中配置了发送头） |
| `0x051` | `TSA2_LS` | 光照/传感器上报（代码中配置了发送头） |
| `0x110` | `TSA_GP_IN` | 通用输入上报（代码中配置了发送头） |
| `0x532` | `TSA_ActionDone` | `data[0]=1`，动作完成 ACK |
| `0x533` | `TSA_XYDone` | `data[0]=1`，XY 主目标到位 ACK |
| `0x534` | `TSA_XYMoveDone` | `data[0]=1`，XYMove 目标到位 ACK |
| `0x002` | `TSA_Door_Relay` | 门/继电器动作反馈，Byte3/Byte4 打包 6 路门状态 |

### CAN2 电机总线

| CAN ID | 方向 | 名称/用途 |
| --- | --- | --- |
| `0x0D1` | 发送到电机 | M1，Y 轴电机之一 |
| `0x0D2` | 发送到电机 | M2，Y 轴电机之一 |
| `0x0D3` | 发送到电机 | M3，X 轴电机 |
| `0x001` | 接收自电机控制器 | HostID，应答帧里再从 Byte0/Byte1 解析实际电机 ID |

---

## CAN1 接收详细说明

## `0x052` — TSA_3：KL15/USB/钥匙/继电器

| 字节/位 | 变量 | 含义 |
| --- | --- | --- |
| Byte0 bit0~1 | `TA531_env_KL15` | KL15 状态 |
| Byte1 bit0~3 | `TA531_env_USB1` | USB1 状态 |
| Byte2 bit0~1 | `TA531_env_KeyLock` | 锁车键 |
| Byte2 bit2~3 | `TA531_env_KeyUnlock` | 解锁键 |
| Byte2 bit4~5 | `TA531_env_KeyRearDoor` | 后门键 |
| Byte2 bit6~7 | `TA531_env_KeyBeep` | 鸣笛/提示键 |
| Byte3 bit0~1 | `TA531_env_KeyWindow` | 车窗键 |
| Byte3 bit2~3 | `TA531_env_KeyLeftDoor` | 左门键 |
| Byte3 bit4~5 | `TA531_env_KeyRightDoor` | 右门键 |
| Byte4 bit0~1 | `TA531_env_Relay1` | 继电器 1 |
| Byte4 bit2~3 | `TA531_env_Relay2` | 继电器 2 |
| Byte4 bit4~5 | `TA531_env_Relay3` | 继电器 3 |
| Byte4 bit6~7 | `TA531_env_Relay4` | 继电器 4 |
| Byte5 bit0~1 | `TA531_env_Relay5` | 继电器 5 |
| Byte5 bit2~3 | `TA531_env_Relay6` | 继电器 6 |

处理完成后设置 `TSA_Ack_DATA[3] = 1`。

## `0x053` — TSA_4：车窗/门/HSD

| 字节/位 | 变量 | 含义 |
| --- | --- | --- |
| Byte0 bit0~3 | `TA531_env_WindowFL` | 左前窗 |
| Byte0 bit4~7 | `TA531_env_WindowFR` | 右前窗 |
| Byte1 bit0~3 | `TA531_env_WindowRL` | 左后窗 |
| Byte1 bit4~7 | `TA531_env_WindowRR` | 右后窗 |
| Byte4 bit0~1 | `TA531_env_DoorSwF` | 前盖/前门类输入，映射到 `Door_Hood` |
| Byte4 bit2~3 | `TA531_env_DoorSwR` | 后备箱/后门类输入，映射到 `Door_Trunk` |
| Byte4 bit4 | `TA531_env_HSD12_1` | HSD1 输出 |
| Byte4 bit5 | `TA531_env_HSD12_2` | HSD2 输出 |
| Byte4 bit6 | `TA531_env_HSD5_1` | HSD3 输出 |
| Byte4 bit7 | `TA531_env_HSD5_2` | HSD4 输出 |
| Byte5 bit0~1 | `TA531_env_DoorSwFL` | 左前门，映射到 `Door_FL` |
| Byte5 bit2~3 | `TA531_env_DoorSwFR` | 右前门，映射到 `Door_FR` |
| Byte5 bit4~5 | `TA531_env_DoorSwRL` | 左后门，映射到 `Door_RL` |
| Byte5 bit6~7 | `TA531_env_DoorSwRR` | 预留/右后门，映射到 `Door_Reserve` |

`Door_Control()` 中实际 GPIO 关系：

| 逻辑门状态 | GPIO 输出 |
| --- | --- |
| `Door_FL` | `Y_RELAY_1` |
| `Door_FR` | `Y_RELAY_2` |
| `Door_RL` | `Y_RELAY_3` |
| `Door_Hood` | `Y_RELAY_4` |
| `Door_Trunk` | `Y_RELAY_5` |
| `Door_Reserve` | `Y_RELAY_6` |

处理完成后设置 `TSA_Ack_DATA[4] = 1`。

## `0x054` — TSA_PWM：舵机/PWM

当前已经按要求退回“最开始”的解析方式。

| 字节/位 | 信号 | 固件解析 |
| --- | --- | --- |
| Byte0 | `PWM_1_Ag` | 直接角度值，先赋给 `TA531_env_PWM_Ag_1` |
| Byte1 | `PWM_2_Ag` | 直接角度值，先赋给 `TA531_env_PWM_Ag_2` |
| Byte2 | `PWM_3_Ag` | 直接角度值，先赋给 `TA531_env_PWM_Ag_3` |
| Byte3 | `PWM_4_Ag` | 直接角度值，先赋给 `TA531_env_PWM_Ag_4` |
| Byte4 bit0~1 | `PWM_1_level` | 仅当 `PWM_1_Ag == 0` 时，`level * 90` 覆盖 PWM1 角度 |
| Byte4 bit2~3 | `PWM_2_level` | 仅当 `PWM_2_Ag == 0` 时，`level * 90` 覆盖 PWM2 角度 |
| Byte4 bit4~5 | `PWM_3_level` | 仅当 `PWM_3_Ag == 0` 时，`level * 90` 覆盖 PWM3 角度 |
| Byte4 bit6~7 | `PWM_4_level` | 仅当 `PWM_4_Ag == 0` 时，`level * 90` 覆盖 PWM4 角度 |
| Byte5 bit0~1 | `PWM_5_level` | 按原表达式解析到 `TA531_env_PWM_Ag_5`，当前未输出到硬件 PWM |
| Byte5 bit2~3 | `PWM_6_level` | 按原表达式解析到 `TA531_env_PWM_Ag_6`，当前未输出到硬件 PWM |
| Byte5 bit4~5 | `PWM_7_level` | 按原表达式解析到 `TA531_env_PWM_Ag_7`，当前未输出到硬件 PWM |
| Byte5 bit6~7 | `PWM_8_level` | 按原表达式解析到 `TA531_env_PWM_Ag_8`，当前未输出到硬件 PWM |

硬件输出关系：

| CAN 信号 | 代码变量 | 输出函数 | MCU PWM 通道 |
| --- | --- | --- | --- |
| `PWM_1_Ag` | `TA531_env_PWM_Ag_1` | `PWMServo2_3_AGout()` | `TIM2_CH3` |
| `PWM_2_Ag` | `TA531_env_PWM_Ag_2` | `PWMServo2_4_AGout()` | `TIM2_CH4` |
| `PWM_3_Ag` | `TA531_env_PWM_Ag_3` | `PWMServo3_1_AGout()` | `TIM3_CH1` |
| `PWM_4_Ag` | `TA531_env_PWM_Ag_4` | `PWMServo3_2_AGout()` | `TIM3_CH2` |

角度到 PWM 脉宽公式：

```c
PWM_Pulse = PWM_ag0 + ag * (PWM_ag90 - PWM_ag0) / 90;
```

当前参数：`PWM_ag0=550`、`PWM_ag90=1500`、`PWM_agMAX=145`。超过 `PWM_agMAX` 会先限幅。

## `0x064` — TSA_RC1 mm：机器人 XY 毫米坐标

| 字节/位 | 变量 | 含义 |
| --- | --- | --- |
| Byte0~1 little-endian | `rx_x_mm` | X 目标，单位 mm，以屏幕 `X0` 为逻辑原点 |
| Byte2 | `TA531_RC_X_Mov` / `g_rc1_x_mov_raw` | XYMove 的 X 参数，按 `int8_t` 使用 |
| Byte3~4 little-endian | `rx_y_mm` | Y 目标，单位 mm，以屏幕 `Y0` 为逻辑原点 |
| Byte5 | `TA531_RC_Y_Mov` / `g_rc1_y_mov_raw` | XYMove 的 Y 参数，按 `int8_t` 使用 |
| Byte6 | `TA531_RC_Z` | Z/触笔参数原始值 |
| Byte7 bit0~1 | `TA531_RC_Z_code` | 触笔按压时长代码：`1=100ms`、`2=2000ms`、`3=5000ms` |
| Byte7 bit6~7 | `TA531_RC_Reset` | Reset 标志 |

普通目标映射：

```text
X_target = DispX0 + rx_x_mm
Y_target = DispY0 + rx_y_mm
```

当 `Reset == 1` 且 `rx_x_mm == 0` 且 `rx_y_mm == 0` 时，目标变成机械零点 `(0,0)`。

## `0x065` — TSA_RC1p %：机器人 XY 百分比坐标

| 字节/位 | 变量 | 含义 |
| --- | --- | --- |
| Byte0 | `x_pct_raw` | X 百分比/比例原始值 |
| Byte2 | `TA531_RC_X_Mov` / `g_rc1_x_mov_raw` | XYMove 的 X 参数 |
| Byte3 | `y_pct_raw` | Y 百分比/比例原始值 |
| Byte5 | `TA531_RC_Y_Mov` / `g_rc1_y_mov_raw` | XYMove 的 Y 参数 |
| Byte6 | `TA531_RC_Z` | Z/触笔参数原始值 |
| Byte7 bit0~1 | `TA531_RC_Z_code` | 触笔按压时长代码：`1=100ms`、`2=2000ms`、`3=5000ms` |
| Byte7 bit6~7 | `TA531_RC_Reset` | Reset 标志 |

`id4` 决定比例分母：

| `id4` | 分母 | 说明 |
| --- | --- | --- |
| `0` | `100` | 新语义，0~100%，超过 100 会夹到 100 |
| `1` | `255` | 旧兼容语义，0~255 |

目标映射：

```text
X_target = DispX0 + x_pct_raw * (DispX1 - DispX0) / scale_den
Y_target = DispY0 + y_pct_raw * (DispY1 - DispY0) / scale_den
```

当 `Reset == 1` 且 X/Y 原始值均为 0 时，目标变成机械零点 `(0,0)`。

## `0x103` — TSA_LIN_SWS G2.5

此报文主要把 CAN 信号转入 `TA531_LIN_SWS`，供 LIN SWS G2.5 数据构建使用。

| 字节/位 | 变量 |
| --- | --- |
| Byte0 bit0 | `LIN_SWS2_ErrRespSWS_1` |
| Byte0 bit1 | `LIN_SWS2_RespErSWSF_1` |
| Byte0 bit4 | `LIN_SWS_CCSwStsDistIncSwA_1` |
| Byte0 bit5 | `LIN_SWS_CCSwStsCCASwA_1` |
| Byte0 bit6~7 | `LIN_SWS_PfTrTapUpDwnSecySwSta_2` |
| Byte1 bit0~1 | `LIN_SWS_CCSwStsSwDataIntgty_2` |
| Byte1 bit2 | `LIN_SWS_CCSwStsSpdDecSwA_1` |
| Byte1 bit3 | `LIN_SWS_CCSwStsSetSwA_1` |
| Byte1 bit4 | `LIN_SWS_CCSwStsRsmSwA_1` |
| Byte1 bit5 | `LIN_SWS_CCSwStsOnSwA_1` |
| Byte1 bit6 | `LIN_SWS_CCSwStsDistDecSwA_1` |
| Byte1 bit7 | `LIN_SWS_CCSwStsSpdIncSwA_1` |
| Byte2 bit0~1 | `LIN_SWS_StrgWhlSwtDataIntgty_2` |
| Byte2 bit2~3 | `LIN_SWS_SWSSelUpSwAL_2` |
| Byte2 bit4~5 | `LIN_SWS_SWSSelDwnSwAL_2` |
| Byte2 bit6~7 | `LIN_SWS_SWSSelLSwAL_2` |
| Byte3 bit0~1 | `LIN_SWS_SWSSelRSwAL_2` |
| Byte3 bit2~3 | `LIN_SWS_SWSCnfmSwAL_2` |
| Byte3 bit4~5 | `LIN_SWS_SWSPB1SwAL_2` |
| Byte3 bit6~7 | `LIN_SWS_SWSPB2SwAL_2` |
| Byte4 bit0~1 | `LIN_SWS_SWSPB3SwAL_2` |
| Byte4 bit2~3 | `LIN_SWS_SWSSelUpSwReq_2` |
| Byte4 bit4~5 | `LIN_SWS_SWSSelDwnSwReq_2` |
| Byte4 bit6~7 | `LIN_SWS_SWSSelLSwReq_2` |
| Byte5 bit0~1 | `LIN_SWS_SWSSelRSwReq_2` |
| Byte5 bit2~3 | `LIN_SWS_SWSCnfmSwReq_2` |
| Byte5 bit4~5 | `LIN_SWS_SWSPB1SwReq_2` |
| Byte5 bit6~7 | `LIN_SWS_SWSPB2SwReq_2` |
| Byte6 bit0~1 | `LIN_SWS_SWSPB3SwReq_2` |
| Byte6 bit2 | `LIN_SWS_SWSSelLSwStuck_1` |
| Byte6 bit3 | `LIN_SWS_SWSSelRSwStuck_1` |
| Byte6 bit4 | `LIN_SWS_SWSCnfmSwStuck_1` |
| Byte6 bit5 | `LIN_SWS_SWSPB1SwStuck_1` |
| Byte6 bit6 | `LIN_SWS_SWSPB2SwStuck_1` |
| Byte6 bit7 | `LIN_SWS_SWSPB3SwStuck_1` |
| Byte7 bit0 | `LIN_SWS_StrgWhlDrvModSwA_1` |
| Byte7 bit1 | `LIN_SWS_PadSSelLSwA_1` |
| Byte7 bit2 | `LIN_SWS_PadSSelRSwA_1` |
| Byte7 bit3 | `LIN_SWS_PadSSelLSwStuck_1` |
| Byte7 bit4 | `LIN_SWS_PadSSelRSwStuck_1` |
| Byte7 bit5 | `LIN_SWS_CCSwStsOnOffSwA_1` |

## `0x104` — TSA_LIN_SWS G3.0

此报文主要把 CAN 信号转入 `TA531_LIN_SWS_G3`，供 LIN SWS G3.0 数据构建使用。常用调试字段：Byte1 bit4~5 是 Up，Byte1 bit6~7 是 Down；代码会同步到 `DEBUG_CAN_Up` / `DEBUG_CAN_Down`。

| 字节/位 | 变量 |
| --- | --- |
| Byte0 bit0~1 | `SpdCtrlLvrSwtChksm_l` |
| Byte0 bit2~3 | `SpdCtrlLvrSwtCntr_l` |
| Byte0 bit4 | `CCSwStsDistIncSwA_l` |
| Byte0 bit5 | `CCSwStsCCASwA_l` |
| Byte0 bit6~7 | `PfTrTapUpDwnSecySwSta_l` |
| Byte1 bit0~1 | `CCSwStsSwDataIntgty_l` |
| Byte1 bit2 | `CCSwStsSpdDecSwA_l` |
| Byte1 bit3 | `CCSwStsSetSwA_l` |
| Byte1 bit4~5 | `SWSSelUpSwAL_l` |
| Byte1 bit6~7 | `SWSSelDwnSwAL_l` |
| Byte2 bit0~1 | `SWSSelLSwAL_l` |
| Byte2 bit2~3 | `SWSSelRSwAL_l` |
| Byte2 bit4~5 | `SWSCnfmSwAL_l` |
| Byte2 bit6~7 | `SWSPB1SwAL_l` |
| Byte3 bit0~1 | `SWSPB2SwAL_l` |
| Byte3 bit2~3 | `SWSPB3SwAL_l` |
| Byte3 bit4~5 | `SWSSelUpSwReq_l` |
| Byte3 bit6~7 | `SWSSelDwnSwReq_l` |
| Byte4 bit0~1 | `SWSSelLSwReq_l` |
| Byte4 bit2~3 | `SWSSelRSwReq_l` |
| Byte4 bit4~5 | `SWSCnfmSwReq_l` |
| Byte4 bit6~7 | `SWSPB1SwReq_l` |
| Byte5 bit0~1 | `SWSPB2SwReq_l` |
| Byte5 bit2~3 | `SWSPB3SwReq_l` |
| Byte5 bit4~5 | `PfTrTapUpDwnSecySwSta_l` |
| Byte5 bit6~7 | `PadSSelLSwA_l` |
| Byte6 bit0~1 | `PadSSelRSwA_l` |
| Byte6 bit2 | `SWSSelLSwStuckL_l` |
| Byte6 bit3 | `SWSSelRSwStuckL_l` |
| Byte6 bit4 | `SWSCnfmSwStuckL_l` |
| Byte6 bit5 | `SWSPB1SwStuckL_l` |
| Byte6 bit6 | `SWSPB2SwStuckL_l` |
| Byte6 bit7 | `SWSPB3SwStuckL_l` |
| Byte7 bit0 | `StrgWhlEntrtnSwDataIntgty_l` |
| Byte7 bit1 | `StrgWhlDrvngMdSwA_l` |
| Byte7 bit2 | `StrgWhlDrvngMdSwDataIntgty_l` |
| Byte7 bit3 | `StrgWhlTipcSwDataIntgty_l` |
| Byte7 bit4 | `PadSSelLSwStuck_l` |
| Byte7 bit5 | `PadSSelRSwStuck_l` |
| Byte7 bit6 | `RespErSWSF_l` |

## `0x105` — EBS LIN FrP00 bridge

| 字节/位 | 变量 | 含义 |
| --- | --- | --- |
| Byte0 + Byte1 bit0~5 | `EBSBatVol_raw` | 电池电压原始值，14 bit |
| Byte2 bit0~1 | `EBSVolSts_raw` | 电压状态 |
| Byte3 bit0~1 | `EBSBatCrntRng_raw` | 电流范围 |
| Byte4 bit0 | `EBSBatIncnstncyFlag_raw` | 电池不一致标志 |
| Byte5 bit0 | `EBSRespEr_raw` | EBS 响应错误 |

## `0x531` — RESET ACK 输入

当收到 `0x531` 且 Byte0~Byte7 全为 `0x00` 时：

- `Remote_state = 0`
- `TA531_Lock = 0`
- `lvLED_Sts_LIN = 0`

---

## CAN2 电机反馈

CAN2 接收帧 ID 为 `HostID = 0x001`。代码从 Byte0/Byte1 里解析实际电机 ID：

```text
TeMP = (buf_rec[0] << 3) + ((buf_rec[1] >> 5) & 0x07)
```

| `TeMP` | 电机 | 反馈处理 |
| --- | --- | --- |
| `0x0D1` | M1，Y 轴 | `0x41` 表示初始化完成；`0x42` 表示位置反馈，位置/160 后用于更新 `Y_act` |
| `0x0D2` | M2，Y 轴 | `0x41` 表示初始化完成；`0x42` 表示位置反馈，当前只保存 M2 内部位置 |
| `0x0D3` | M3，X 轴 | `0x41` 表示初始化完成；`0x42` 表示位置反馈，位置/160 后用于更新 `X_act` |

位置反馈换算：

```text
M_Position = little_endian(Byte3..Byte6) / 160
X_act = -M3_Position - 10
Y_act = -M1_Position - 10
```

---

## 电机目标发送格式

`MotoCtrl_PositionLoop(PositionX_mm, PositionY_mm)` 是所有 XY 目标最终进入电机控制的入口：

1. 先把 `PositionX_mm`/`PositionY_mm` 限幅到机械范围。
2. X 轴由 M3 控制：`DataCode = -(PositionX_mm + 10) * 160`。
3. Y 轴由 M1/M2 同步控制：`DataCode = -(PositionY_mm + 10) * 160`。
4. 通过 CAN2 分别发送到 `0x0D3`、`0x0D1`、`0x0D2`。

