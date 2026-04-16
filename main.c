/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/**
 * version .3.1
 * last change: add KL15 channel
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
//#include <Device_V1.h>
#include <OLED.h>
//#include <USBPD.h>

#include <stm32g0xx_hal_fdcan.h>

////mylib
#include <EasySyslib.h>
#include <FlashAddr.h>
//#include "ModBus.h"
//#include "TCHUBv3.h"

#include "lvgl.h"
#include "lv_conf.h"
#include "E:\stm32\STM32CubeIDE_1.18.1\workspace\v4_3.0ZXD\G0B1VET6_TSTA_v4_3.0ZXD\Core\lvgl\examples\porting\lv_port_disp.h"
#include "E:\stm32\STM32CubeIDE_1.18.1\workspace\v4_3.0ZXD\G0B1VET6_TSTA_v4_3.0ZXD\Core\lvgl\examples\porting\lv_port_indev.h"
#include "E:\stm32\STM32CubeIDE_1.18.1\workspace\v4_3.0ZXD\G0B1VET6_TSTA_v4_3.0ZXD\Core\lvgl\gui_guider\gui_guider.h"
#include "E:\stm32\STM32CubeIDE_1.18.1\workspace\v4_3.0ZXD\G0B1VET6_TSTA_v4_3.0ZXD\Core\lvgl\gui_guider\events_init.h"

#include "E:\stm32\STM32CubeIDE_1.18.1\workspace\v4_3.0ZXD\G0B1VET6_TSTA_v4_3.0ZXD\Core\lvgl\gui_guider\widgets_init.h"
#include "E:\stm32\STM32CubeIDE_1.18.1\workspace\v4_3.0ZXD\G0B1VET6_TSTA_v4_3.0ZXD\Core\lvgl\custom\custom.h"

lv_ui guider_ui;

uint8_t lv_refresh_flg, lv_refresh_flg_old;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SPI_HandleTypeDef *Flash_SPI;
SPI_HandleTypeDef *TFT_SPI;

UART_HandleTypeDef *Serial_Num;

#define LINProfile_AH4EM		0
#define LINProfile_IP5PM		1
#define LINProfile_ZS32			0

#define PWM_ag0			550
#define PWM_ag90			1500
#define PWM_agMAX			145

#define ADC_CHANNELS 	6
#define LightSensr_Gate 	50
#define LIGHT_SENSOR_INVERT	0	// 0: keep raw mapping; 1: invert when hardware is wired opposite
uint16_t adc_buffer[ADC_CHANNELS] = { 0 };

static uint8_t NormalizeLightSensor(uint16_t raw_adc) {
	uint8_t level = (uint8_t) ((raw_adc * 255U) / 4095U);
	if (LIGHT_SENSOR_INVERT) {
		level = (uint8_t) (255U - level);
	}
	return level;
}

static uint8_t EncodeLightAlarm(uint8_t light_level) {
	if (light_level < LightSensr_Gate) {
		if (light_level < (LightSensr_Gate / 2)) {
			if (light_level < (LightSensr_Gate / 3)) {
				return 3;
			}
			return 2;
		}
		return 1;
	}
	return 0;
}

int Version_A = 4;	//Ver  A.BC
int Version_B = 0;
int Version_C = 3;	//0.08

uint16_t LIN_Data_LENGTH = 1;	//1;
uint16_t Serial_Data_LENGTH = 24;

uint16_t CAN1_2Ser_ID[32];
uint16_t CAN2_2Ser_ID[32];

#define LINProfile_IP5PM_L 0
#define LINProfile_IP5PM_H 1

/* ====================== 交接配置区（可直接改这里） ======================
 * 说明：
 * 1) 想切换功能时，只改下面宏开关，不要到处改业务代码。
 * 2) 每个功能都保留 A/B 两套逻辑（通过 #if/#else 切换）。
 */
// 开机是否执行 Flash 自检（1=执行；0=跳过）
#define CFG_BOOT_FLASH_SELF_TEST_EN      1
// OLED 第4行显示方式（1=显示 LIN RID 22/34/35/36 收包状态；0=显示 A1/A2）
#define CFG_OLED_SHOW_RID_FLAGS_EN       1

// Motor motion loop timing (ms)
#define MOTOR_INIT_RETRY_MS          100U
#define MOTOR_LOOP_INTERVAL_MS       10U
#define MOTOR_WAIT_POLL_MS           100U
#define MOTOR_SEND_GAP_MS            1U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//////// ////////system level
//sys time
uint8_t Enp_YEAR, Enp_MONTH, Enp_DAY, Enp_HOUR, Enp_MINUTE, Enp_SECOND;
uint8_t EncrypKey;

uint8_t dateTimeBuffer[7];

bool SW_UP, SW_UP_pre, SW_DW, SW_DW_pre, SW_LEFT, SW_LEFT_pre, SW_RIGHT,
		SW_RIGHT_pre, SW_BUTTON, SW_BUTTON_pre;
uint8_t SW_UP_cnt = 0;
uint8_t SW_DW_cnt = 0;
uint8_t SW_LEFT_cnt = 0;
uint8_t SW_RIGHT_cnt = 0;

//W25Q64 	sector		block		page
//8M/64m	 256	x	 64		x	 256
int Sys_CNT, Sys_RunCNT, Sys_CNT1, Sys_RunCNT1;
////	DispX0,DispX1,DispY0,DispY1;

uint32_t Sys_TIM_TICK;	//1-1000ms
bool Sys_TIM_Flag;

//config
int8_t BoardID;
bool SW1_1, SW1_2, SW2_1, SW2_2;
int Mode_ID;//IO_CFG_1/2/3/4	电阻上拉，拨码开关off时高电平，on时与GND导�?�，低电�??????????????????????????????????????????????????????????????????????????????????????

//state
int sys_state = 0; //0 - 6; 	0 - none; 1- green	;2 - cyan(青色，天蓝）;3 - blue	;4 - yellow	;5 - purple	;6 - red
bool ERR = 0;
int Remote_state = 0;
uint8_t Web_ConnectSts;

uint8_t lvLED_Sts_TPRobot, lvLED_Sts_LIN, lvLED_Sts_CAN, lvLED_Sts_Sensor; // 0 grey,	1	green,	2	oragne,	3	red

////LIN
uint8_t DataReceiveflag;
uint8_t DataProcess;
uint8_t ReceivePID, ReceiveID;
uint8_t FrameReceiveOverFlag;
uint8_t LinReceiveData[9];
uint8_t ReceiveData, SendCheckSum, ReceiveCheckSum;
uint8_t DtRxProcess;

#define CANMsg_MNumb		32
FDCAN_TxHeaderTypeDef CAN_TxHeader[CANMsg_MNumb];
FDCAN_TxHeaderTypeDef FDCAN_TxHeader[CANMsg_MNumb];

// 添加调试用的全局变量
uint8_t DEBUG_CAN_Up = 0xFF;    // 0xFF表示未收到数据
uint8_t DEBUG_CAN_Down = 0xFF;
uint8_t DEBUG_CAN_RX_Flag = 0;  // 收到新数据标志
uint8_t DEBUG_LIN_Byte1 = 0;     // LIN打包后的Byte1
uint8_t DEBUG_LIN_Pack_Flag = 0; // 打包完成标志
uint8_t DEBUG_LIN_Checksum = 0;

uint8_t DEBUG_UART_RX_Count = 0;      // UART接收计数
uint8_t DEBUG_ReceiveID = 0;          // 收到的ID
uint8_t DEBUG_ReceivePID = 0;         // 最近收到的PID字节
uint8_t DEBUG_LIN_Send_Count = 0;     // LIN发送计数
uint8_t DEBUG_DataProcess = 0;        // DataProcess状态
uint8_t DEBUG_CAN_104_Count = 0;      // 收到0x104的计数
uint16_t DEBUG_RID22_Count = 0;       // 识别到RID 0x22的累计次数
uint16_t DEBUG_RID34_Count = 0;       // 识别到RID 0x34的累计次数
uint16_t DEBUG_RID35_Count = 0;       // 识别到RID 0x35的累计次数
uint16_t DEBUG_RID36_Count = 0;       // 识别到RID 0x36的累计次数
uint16_t EBSBatVol_raw = 16383;           // 14bit raw, 16383 means invalid per LDF
uint8_t EBSVolSts_raw = 0;                // 2bit status, 0=No_error
uint8_t EBSBatCrntRng_raw = 0;            // 2bit status, 0=±1A range
uint8_t EBSBatIncnstncyFlag_raw = 0;      // 1bit, 0=no error
uint8_t EBSRespEr_raw = 0;                // 1bit, response error

//////// ////////app level

//Motor Control
struct MotorCtrl_TypeDef MotorCtrl_M1, MotorCtrl_M2, MotorCtrl_M3, MotorCtrl_M4;
uint16_t HostID = 0x01;
uint16_t M1_ID = 0x0D1;        //D1	D2,	X
uint16_t M2_ID = 0x0D2;        //D1	D2,	X
uint16_t M3_ID = 0x0D3;        //D3	D4,	Y
uint16_t M4_ID = 0x0D5;        //D5,	Z

uint8_t MotorInit_M1, MotorInit_M2, MotorInit_M3, MotorInit_M4;
bool MotorERR_M1, MotorERR_M2, MotorERR_M3, MotorERR_M4;

#define XmaxLimit 	600		//290
#define YmaxLimit 	600		//435

//#define RCtrl_Mode_0m1p		1

//#define DispX0 	100		////#define DispX1 	120		////#define DispY0 	100		////#define DispY1 	120		//
//uint8_t	DispX0[4],DispX1[4],DispY0[4],DispY1[4];
//int	DispX0_32b,DispX1_32b,DispY0_32b,DispY1_32b;

struct ScreenSize_TypeDef ScreenSz_1;

//web232
struct Ser2CAN_Msg_TypeDef Ser2CAN_Msg[CANMsg_MNumb];
struct SerLoCtrl_Msg_TypeDef SerLoCtrl_Msg;
struct Bench_AckInfo_TypeDef Bench_Info;

//TA531
struct TA531_env_TypeDef TA531SysEnv;
struct TA531_TimCallback_TypeDef TA531TimCallback;
struct TA531_RobotCtrl_TypeDef TA531_RC1;

struct TA531_GP_IN_TypeDef TA531_GP_IN;
struct TA531_Door_TypeDef TA531_Door;
uint8_t TSA_GP_IN_DATA[8];

uint8_t TA531_RC1_fg; // 0-Error;	1-no task;	2-command received;	3-command accomplish;
uint8_t TA531_RC1_x_ready, TA531_RC1_y_ready, TA531_RC1_z_ready;
uint8_t TA531_RC1_Ack, TA531_Lock;

//stm32 header

FDCAN_TxHeaderTypeDef CAN_TxHeader[CANMsg_MNumb];
FDCAN_TxHeaderTypeDef FDCAN_TxHeader[CANMsg_MNumb];

FDCAN_TxHeaderTypeDef MotrCtrl_1_TxHeader, MotrCtrl_2_TxHeader,
		MotrCtrl_3_TxHeader, MotrCtrl_4_TxHeader;
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader, FDCAN2_RxHeader, MotrCtrl_1_RxHeader,
		MotrCtrl_2_RxHeader, MotrCtrl_3_RxHeader;
FDCAN_FilterTypeDef FDCAN_Filter1[28], FDCAN_Filter2[28]; //
FDCAN_TxHeaderTypeDef TSA_Ack_Header, TSA1_ADC_Header, TSA2_LS_Header,
		TSA_Ack_RC_Header, TSA_GP_IN_Header;
FDCAN_TxHeaderTypeDef TSA_Door_Relay_Header;
uint8_t TSA_Door_Relay_DATA[8];
//communication

uint8_t CH1RxData[8], CH2RxData[8];
uint8_t CH1TxData[8], CH2TxData[8], TSA1_ADC_DATA[8], TSA2_LS_DATA[8],
		TSA_Ack_DATA[8], TSA_Ack_RC_DATA[8];
uint8_t SWS_0x00_Data[9], SWS_0x02_Data[9], HOD_0x0_Data[9], EBS_0x0_Data[9],
		EBS_0x1_Data[9], EBS_0x2_Data[9], IFP_0x0_Data[9];
uint8_t SWS_0x22_Data[9]; //G3.0
bool SWS_0x00_Flag, SWS_0x02_Flag, HOD_0x0_Flag, EBS_0x0_Flag, IFP_0x0_Flag,
		SWS_0x22_Flag;
uint8_t MotrCtrl_1_DATA[8], MotrCtrl_2_DATA[8], MotrCtrl_3_DATA[8],
		MotrCtrl_4_DATA[8];

uint8_t I2CRxData[REC_LENGTH];
uint8_t u1RxData[REC_LENGTH], u2RxData[REC_LENGTH], u3RxData[REC_LENGTH],
		u4RxData[REC_LENGTH], u5RxData[REC_LENGTH], u6RxData[REC_LENGTH],
		RxData[REC_LENGTH];
uint8_t Serial_TxData[REC_LENGTH] = { 0 };
uint8_t Bench_Info_TxData[REC_LENGTH] = { 0 };

struct TA531_LIN_GW_TypeDef TA531_LIN_GW;
struct TA531_LIN_SWS_TypeDef TA531_LIN_SWS;
struct TA531_LIN_SWS_G3_TypeDef TA531_LIN_SWS_G3;

uint32_t len;

bool rxflag;
bool U1RXFlag = 0, U2RXFlag = 0, U3RXFlag = 0, U4RXFlag = 0, U5RXFlag = 0,
		U6RXFlag = 0;

bool TSA3_0x52_Flag = 0, TSA4_0x53_Flag = 0, TSA4_0x54_Flag = 0,
		TSA4_0x103_Flag = 0, TSA4_0x104_Flag = 0;

////////TMP1075
float temp1075_ZXD, temp1075_ZPD, temp1075_LHZCU, temp1075_RHZCU, temp1075_RZCU,
		temp1075_Bk1, temp1075_Bk2, temp1075_Bk3;
uint8_t temp1075_ZXD_i, temp1075_ZPD_i, temp1075_LHZCU_i, temp1075_RHZCU_i,
		temp1075_RZCU_i, temp1075_Bk1_i, temp1075_Bk2_i, temp1075_Bk3_i;
uint8_t temp1075_ZXD_f, temp1075_ZPD_f, temp1075_LHZCU_f, temp1075_RHZCU_f,
		temp1075_RZCU_f, temp1075_Bk1_f, temp1075_Bk2_f, temp1075_Bk3_f;
bool temper_flag;

uint32_t adc_value;
uint32_t mVoltage;

struct TA531_Door_TypeDef {
	uint8_t Door_FL;        // 左前门：0-无动作, 1-开, 2-关, 3-保持
	uint8_t Door_FR;        // 右前门
	uint8_t Door_RL;        // 左后门
	uint8_t Door_Hood;      // 引擎盖
	uint8_t Door_Trunk;     // 后备箱
	uint8_t Door_Reserve;
};

// ========== 电机保护系统 ==========
struct Motor_Protection_TypeDef {
	int16_t last_X_pos;
	int16_t last_Y_pos;
	int8_t X_direction_changes;
	int8_t Y_direction_changes;
	uint16_t stuck_counter;
	uint16_t movement_timeout;
	uint32_t last_progress_tick;
	uint8_t protection_triggered;
	uint8_t error_type;
	uint32_t total_errors;
};

struct Motor_Protection_TypeDef Motor_Protection;

#define MOTOR_PROTECTION_ENABLED        1
#define MAX_DIRECTION_CHANGES          3
#define MAX_STUCK_COUNT               8
#define MAX_MOVEMENT_TIMEOUT         100
#define MOVE_WAIT_TIMEOUT_MS        8000
#define MOVE_WAIT_TIMEOUT_INIT_MS   15000
#define POSITION_TOLERANCE             5
#define REACH_POSITION_TOLERANCE       2
#define MAX_MOVEMENT_TIMEOUT_MS (MAX_MOVEMENT_TIMEOUT * MOTOR_WAIT_POLL_MS)
// ==================================

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void UART_RESET(UART_HandleTypeDef *huart);
void UART_Init(UART_HandleTypeDef *huart, uint32_t data_length);
void LIN_RESET(UART_HandleTypeDef *huart);
uint8_t Lin_CheckPID(uint8_t id);
uint8_t Lin_Checksum(uint8_t id, uint8_t data[]);
uint8_t Calc_SWS_G3_CRC8(const uint8_t *data, uint8_t len);
void Build_EBS_0x34_Data(void);
void Build_EBS_0x35_Data(void);
void Build_EBS_0x36_Data(void);
void Lin_SendData(uint8_t *data);
void Lin_DataProcess_loop(void);

void SPI_Stop(SPI_HandleTypeDef *hspi);
void SPI_Flash_Start(SPI_HandleTypeDef *hspi);
void SPI_TFT_Start(SPI_HandleTypeDef *hspi);

void Ser2CAN(void);
uint32_t mRead_ADC1_ch(uint8_t ch);
void Sys_tune1();
void Sys_tuneX(uint32_t fq);
uint32_t PWMServo_Ag2Pulse(uint32_t ag);
void PWMServo2_3_AGout(uint32_t ag);
void PWMServo2_4_AGout(uint32_t ag);
void PWMServo3_1_AGout(uint32_t ag);
void PWMServo3_2_AGout(uint32_t ag);
void MoC_Init();
void MotoCtrl_PackSend12();
//void MotoCtrl_PackSend2();
void MotoCtrl_PackSend3();
void MotoCtrl_PackSend4();
void MotoCtrl_PositionLoop(int PositionX_mm, int PositionY_mm);
bool WaitMotorToTargetWithProtection(uint32_t timeout_ms, uint16_t poll_ms,
		bool trigger_emergency);
//void CAN2Ser_Config(void);
void Set_SystemReboot();
void Clamp_Position(int *x, int *y, bool allow_reset);
void Door_Control(void);
uint8_t ByteEncryp(uint8_t byteData);

void Motor_Protection_Init(void);
void Motor_Protection_Reset(void);
uint8_t Motor_Protection_Check(int16_t current_X, int16_t current_Y,
		int16_t target_X, int16_t target_Y);
void Motor_Protection_EmergencyStop(void);

/* ====================== 可交接功能函数（中文注释） ====================== */
// 开机流程：执行 Flash 自检 + 读取 UID（A 版本）
static void Boot_RunFlashSelfTest_AndLoadUID(char *str_buf);
// 开机流程：跳过 Flash 自检（B 版本）
static void Boot_SkipFlashSelfTest(void);
// OLED 第4行：显示 LIN RID 收包状态
static void OLED_ShowRIDFlagsLine(uint8_t row, char *oled_line);
// OLED 运行页：id1==0（非 MoC）
static void OLED_UpdatePage_Id0(char *oled_line);
// OLED 运行页：id1==1（MoC）
static void OLED_UpdatePage_Id1(char *oled_line);
// LIN 工具：重启 USART3 的 LIN 接收
static void Lin_RearmUart3(void);
// LIN 工具：处理常见 RID（22/34/35/36），返回 true 表示已处理
static bool Lin_HandleKnownRid(uint8_t rid);
// LIN 工具：各 RID 独立处理函数（便于交接和扩展）
static void Lin_HandleRid22(void);
static void Lin_HandleRid34(void);
static void Lin_HandleRid35(void);
static void Lin_HandleRid36(void);
// LIN 工具：处理未知 RID 的通用流程
static void Lin_HandleUnknownRid(void);
// LIN 工具：从 USART3 读取本次接收字节/帧
static void Lin_ReadRxDataFromUart3(void);
// LIN 工具：更新本次接收的调试状态
static void Lin_UpdateDebugOnRx(void);
// UART 工具：按句柄重启接收（用于 huart1/2/3）
static void Uart_RearmByHandle(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_FDCAN1_Init();
	MX_FDCAN2_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_CRC_Init();
	MX_I2C3_Init();
	MX_SPI1_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_TIM14_Init();
	MX_ADC1_Init();
	MX_TIM16_Init();
	MX_TIM17_Init();
	MX_TIM1_Init();
	MX_TIM15_Init();
	MX_USART3_UART_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	///////use OLED for debug msg display
	HAL_Delay(300);
	OLED_Init(OLED_I2C_ch, OLED_type);
	OLED_Fill(OLED_I2C_ch, OLED_type, 0xff);
	HAL_Delay(1000);
	OLED_Fill(OLED_I2C_ch, OLED_type, 0x00);
	HAL_Delay(1000);

	HAL_FDCAN_MspInit(&hfdcan1);
	HAL_FDCAN_MspInit(&hfdcan2);

	Flash_SPI = &hspi1;
	TFT_SPI = &hspi1;

	TSA_Ack_Header.Identifier = 0x531;
	TSA_Ack_Header.DataLength = FDCAN_DLC_BYTES_8;
	TSA_Ack_Header.FDFormat = FDCAN_CLASSIC_CAN;	//FDCAN_FD_CAN;
	TSA_Ack_Header.BitRateSwitch = FDCAN_BRS_OFF;	//FDCAN_BRS_ON; //
	TSA_Ack_Header.TxFrameType = FDCAN_DATA_FRAME;///*!< Data frame > < Remote frame */
	TSA_Ack_Header.IdType = FDCAN_STANDARD_ID;
	TSA_Ack_Header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TSA_Ack_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TSA_Ack_Header.MessageMarker = 0;

	TSA2_LS_Header.Identifier = 0x51;
	TSA2_LS_Header.DataLength = FDCAN_DLC_BYTES_8;
	TSA2_LS_Header.FDFormat = FDCAN_CLASSIC_CAN;	//FDCAN_FD_CAN;
	TSA2_LS_Header.BitRateSwitch = FDCAN_BRS_OFF;	//FDCAN_BRS_ON; //
	TSA2_LS_Header.TxFrameType = FDCAN_DATA_FRAME;///*!< Data frame > < Remote frame */
	TSA2_LS_Header.IdType = FDCAN_STANDARD_ID;
	TSA2_LS_Header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TSA2_LS_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TSA2_LS_Header.MessageMarker = 0;

	TSA1_ADC_Header.Identifier = 0x50;
	TSA1_ADC_Header.DataLength = FDCAN_DLC_BYTES_8;
	TSA1_ADC_Header.FDFormat = FDCAN_CLASSIC_CAN;	//FDCAN_FD_CAN;
	TSA1_ADC_Header.BitRateSwitch = FDCAN_BRS_OFF;	//FDCAN_BRS_ON; //
	TSA1_ADC_Header.TxFrameType = FDCAN_DATA_FRAME;	///*!< Data frame > < Remote frame */
	TSA1_ADC_Header.IdType = FDCAN_STANDARD_ID;
	TSA1_ADC_Header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TSA1_ADC_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TSA1_ADC_Header.MessageMarker = 0;

	TSA_GP_IN_Header.Identifier = 0x110;
	TSA_GP_IN_Header.DataLength = FDCAN_DLC_BYTES_8;
	TSA_GP_IN_Header.FDFormat = FDCAN_CLASSIC_CAN;	//FDCAN_FD_CAN;
	TSA_GP_IN_Header.BitRateSwitch = FDCAN_BRS_OFF;	//FDCAN_BRS_ON; //
	TSA_GP_IN_Header.TxFrameType = FDCAN_DATA_FRAME;///*!< Data frame > < Remote frame */
	TSA_GP_IN_Header.IdType = FDCAN_STANDARD_ID;
	TSA_GP_IN_Header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TSA_GP_IN_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TSA_GP_IN_Header.MessageMarker = 0;

	// M1电机
	MotrCtrl_1_TxHeader.Identifier = M1_ID;  // ← 关键！0x0D1
	MotrCtrl_1_TxHeader.IdType = FDCAN_STANDARD_ID;
	MotrCtrl_1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	MotrCtrl_1_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	MotrCtrl_1_TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	MotrCtrl_1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	MotrCtrl_1_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	MotrCtrl_1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	MotrCtrl_1_TxHeader.MessageMarker = 0;

	// M2电机
	MotrCtrl_2_TxHeader.Identifier = M2_ID;  // ← 关键！0x0D2
	MotrCtrl_2_TxHeader.IdType = FDCAN_STANDARD_ID;
	MotrCtrl_2_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	MotrCtrl_2_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	MotrCtrl_2_TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	MotrCtrl_2_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	MotrCtrl_2_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	MotrCtrl_2_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	MotrCtrl_2_TxHeader.MessageMarker = 0;

	// M3电机
	MotrCtrl_3_TxHeader.Identifier = M3_ID;  // ← 关键！0x0D3
	MotrCtrl_3_TxHeader.IdType = FDCAN_STANDARD_ID;
	MotrCtrl_3_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	MotrCtrl_3_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	MotrCtrl_3_TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	MotrCtrl_3_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	MotrCtrl_3_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	MotrCtrl_3_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	MotrCtrl_3_TxHeader.MessageMarker = 0;

	TSA_Door_Relay_Header.Identifier = 0x002;  // 使用与电机相同的ACK ID
	TSA_Door_Relay_Header.DataLength = FDCAN_DLC_BYTES_8;
	TSA_Door_Relay_Header.FDFormat = FDCAN_CLASSIC_CAN;
	TSA_Door_Relay_Header.BitRateSwitch = FDCAN_BRS_OFF;
	TSA_Door_Relay_Header.TxFrameType = FDCAN_DATA_FRAME;
	TSA_Door_Relay_Header.IdType = FDCAN_STANDARD_ID;
	TSA_Door_Relay_Header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TSA_Door_Relay_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TSA_Door_Relay_Header.MessageMarker = 0;
////////TIM6定时器，20ms周期

	HAL_TIM_Base_Start_IT(&htim6);

////////TIM7定时器，100ms周期

	HAL_TIM_Base_Start_IT(&htim7);

////////TIM7定时器，1000ms周期

	HAL_TIM_Base_Start_IT(&htim14);

////////TIM16定时器，5ms周期

	HAL_TIM_Base_Start_IT(&htim16);

	////////TIM17定时器，1ms周期

	HAL_TIM_Base_Start_IT(&htim17);

	// 读取PCF8563的时间和日期
	//PCF8563_ReadDateTime(dateTimeBuffer);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

/////////////TempSensor
//	TMP1075_Init(temp1075_ch , TempSensor_ADDR_ZXD );
//	HAL_Delay(20);
////
	//SPI_TFT_NSS_Pin||SPI_Flash_NSS_Pin
//	SPI_Stop(Flash_SPI);
//	HAL_Delay(10);
//	SPI_TFT_Start(Flash_SPI);
//	HAL_Delay(50);
//Mode check
	bool id1, id2, id3, id4;
	id1 = HAL_GPIO_ReadPin(IO_CFG_1_GPIO_Port, IO_CFG_1_Pin);
	id2 = HAL_GPIO_ReadPin(IO_CFG_2_GPIO_Port, IO_CFG_2_Pin);
	id3 = HAL_GPIO_ReadPin(IO_CFG_3_GPIO_Port, IO_CFG_3_Pin);
	id4 = HAL_GPIO_ReadPin(IO_CFG_4_GPIO_Port, IO_CFG_4_Pin);

	Mode_ID = (id1 << 3) + (id2 << 2) + (id3 << 1) + (id4 << 0);

	HAL_Delay(2000);

	if (id1 == 0)	//id1 = 0, no RC
			{
		char *str = "FW: ";
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 0, str);
	} else			//id1 = 1,with RC
	{
		char *str = "MoC ";
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 0, str);
	}

	char str1[16] = { 0 };
	char str2[16] = { 0 };
	char oled_line[17] = { 0 };
	itoa(Version_A, str1, 10);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 4, 0, str1);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 5, 0, ".");
	itoa(Version_B, str1, 10);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 6, 0, str1);
	itoa(Version_C, str1, 10);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 7, 0, str1);

	if (HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == 0) // DOWN键按下
			{
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Hold 5s Clear! ");
		Sys_tune1();

		uint8_t countdown = 5;
		bool stillPressed = true;

		// 倒计时5秒
		for (int i = 0; i < 5; i++) {
			if (HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) != 0) // 松开了
					{
				stillPressed = false;
				break;
			}

			char str_count[16];
			itoa(countdown, str_count, 10);
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "Countdown: ");
			OLED_ShowString(OLED_I2C_ch, OLED_type, 11, 2, str_count);
			OLED_ShowString(OLED_I2C_ch, OLED_type, 13, 2, "s  ");

			HAL_Delay(1000);
			countdown--;
		}

		if (stillPressed
				&& (HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == 0)) {
			// ⬅️ 清除Flash配置
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 0, "================");
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Clearing Flash! ");
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "Please Wait...  ");
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3, "================");

			Sys_tune1();  // ⬅️ 如果没有Sys_tune2就用Sys_tune1

			// ⬅️⬅️⬅️ 关键：先切换SPI到Flash模式
			SPI_Stop(Flash_SPI);
			HAL_Delay(10);
			SPI_Flash_Start(Flash_SPI);
			HAL_Delay(10);

			uint8_t clearData[4] = { 0xFF, 0xFF, 0xFF, 0xFF };

			// ⬅️⬅️⬅️ 重要：每次写入前都要加写使能！
			SPI_Flash_WtritEnable();
			HAL_Delay(5);
			SPI_Flash_WriteSomeBytes(clearData, Sys_Addr_DispX0, sizeof(int));
			HAL_Delay(50);

			SPI_Flash_WtritEnable();
			HAL_Delay(5);
			SPI_Flash_WriteSomeBytes(clearData, Sys_Addr_DispX1, sizeof(int));
			HAL_Delay(50);

			SPI_Flash_WtritEnable();
			HAL_Delay(5);
			SPI_Flash_WriteSomeBytes(clearData, Sys_Addr_DispY0, sizeof(int));
			HAL_Delay(50);

			SPI_Flash_WtritEnable();
			HAL_Delay(5);
			SPI_Flash_WriteSomeBytes(clearData, Sys_Addr_DispY1, sizeof(int));
			HAL_Delay(50);

			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 0, "================");
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Config Cleared! ");
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "Rebooting...    ");
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3, "================");

			Sys_tune1();
			HAL_Delay(2000);

			// ⬅️ 自动重启（不需要等松开按键）
			HAL_NVIC_SystemReset();
		} else {
			// 松开了，取消清除
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Clear Cancelled ");
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "                ");
			HAL_Delay(1000);
		}
	}

//	if(id1 == 1)	//id1 = 0, no RC
//	{
//		if(RCtrl_Mode_0m1p == 1)
//		{
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,14, 0, "%");
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "P/255 Control");
//		}
//		else
//		{
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,14, 0, "mm");
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Position Mode");
//		}
//	}

	HAL_Delay(2000);

//	str = "|cfg: ";
//	OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 0, str );
//	OLED_ShowString(OLED_I2C_ch ,OLED_type,14, 0," .");

//	HAL_UART_Transmit(&huart2, "ATP_TD_mini Ready~", strlen("ATP_TD_mini Ready~") ,0xff);
//	HAL_UART_Transmit_IT(&huart1, "Hello TA531~", strlen("Hello TA531~") );

	for (int i = 0; i < CANMsg_MNumb; i++)	//
			{
		CAN_TxHeader[i].TxFrameType = FDCAN_DATA_FRAME;
		CAN_TxHeader[i].ErrorStateIndicator = FDCAN_ESI_PASSIVE;
		CAN_TxHeader[i].IdType = FDCAN_STANDARD_ID;
		CAN_TxHeader[i].MessageMarker = 0;
		CAN_TxHeader[i].TxEventFifoControl = FDCAN_NO_TX_EVENTS;

		FDCAN_TxHeader[i].TxFrameType = FDCAN_DATA_FRAME;
		FDCAN_TxHeader[i].ErrorStateIndicator = FDCAN_ESI_PASSIVE;
		FDCAN_TxHeader[i].IdType = FDCAN_STANDARD_ID;
		FDCAN_TxHeader[i].MessageMarker = 0;
		FDCAN_TxHeader[i].TxEventFifoControl = FDCAN_NO_TX_EVENTS;

		Ser2CAN_Msg[i].DataCycle = 0xffff;
	}

//	  HAL_Delay(500);

	/* ====================== 开机 Flash 检测 A/B 版本切换 ======================
	 * A 版本（推荐调试）: 执行 Flash 读写自检 + 读取 UID
	 * B 版本（快速启动）: 跳过 Flash 自检，直接给默认 EncrypKey
	 * 只需修改 CFG_BOOT_FLASH_SELF_TEST_EN 即可切换
	 */
#if CFG_BOOT_FLASH_SELF_TEST_EN
	Boot_RunFlashSelfTest_AndLoadUID(str1);
#else
	/* B 版本：跳过 Flash 检测，适合快速上电验证 */
	Boot_SkipFlashSelfTest();
#endif
////

//// ADC
	HAL_Delay(50);
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
		char *str0 = "ADC_Calib_Error! ";
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, str0);
		// 校准失败，处理错�????????????????????????????????????????????????????????
		Error_Handler();
		while (1)
			;
	}

	MX_DMA_Init();
	HAL_Delay(20);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, ADC_CHANNELS);

	SPI_Stop(Flash_SPI);
	HAL_Delay(10);
	SPI_TFT_Start(Flash_SPI);
	HAL_Delay(50);

	ST7789_Init();
	HAL_Delay(100);

	ST7789_Flush(TFT_RED);
	HAL_Delay(200);
	ST7789_Flush(TFT_GREEN);
	HAL_Delay(200);
	ST7789_Flush(TFT_BLUE);
	HAL_Delay(200);

//    Sys_tune1();	//1000

	for (int i = 10; i > 3; i--) {
//    	Sys_tuneX(i*300);
		HAL_Delay(100);
	}

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	PWMServo2_3_AGout(0);
	PWMServo2_4_AGout(0);
	PWMServo3_1_AGout(0);
	PWMServo3_2_AGout(0);

	if (id1 == 1)	//MoC
			{
		MoC_Init();
	} else {
		lvLED_Sts_TPRobot = 0;
	}

//    Set_SystemReboot();

// 启动LIN接收
	HAL_UART_Receive_IT(&huart1, u1RxData, LIN_Data_LENGTH);
	HAL_UART_Receive_IT(&huart2, u2RxData, LIN_Data_LENGTH);
	HAL_UART_Receive_IT(&huart3, u3RxData, LIN_Data_LENGTH);

////// LIN init

	HAL_GPIO_WritePin(LIN1_EN_GPIO_Port, LIN1_EN_Pin, 1);
	HAL_GPIO_WritePin(LIN2_EN_GPIO_Port, LIN2_EN_Pin, 1);
	HAL_GPIO_WritePin(LIN3_EN_GPIO_Port, LIN3_EN_Pin, 1);
//	// 解除LIN收发器复位（NRES低电平有效，拉高解除复位）
//	HAL_GPIO_WritePin(LIN1_NRES_GPIO_Port, LIN1_NRES_Pin, 1);
//	HAL_GPIO_WritePin(LIN2_NRES_GPIO_Port, LIN2_NRES_Pin, 1);
//	HAL_GPIO_WritePin(LIN3_NRES_GPIO_Port, LIN3_NRES_Pin, 1);

//	////Init XL9555
//	XL9555_Init( XL9555_1_addr_write , 1 , 1 );	//in in //out out
//	XL9555_Init( XL9555_2_addr_write , 1 , 1 );	//in in //out out

///////////lvgl Init
	lv_init();

	lv_port_disp_init();
	lv_port_indev_init();

	////// use gui_guider
	setup_ui(&guider_ui);
	events_init(&guider_ui);

	itoa(Version_A, str1, 10);
	lv_label_set_text(guider_ui.screen_label_A, str1);
	itoa(Version_B, str1, 10);
	lv_label_set_text(guider_ui.screen_label_B, str1);
	itoa(Version_C, str1, 10);
	lv_label_set_text(guider_ui.screen_label_C, str1);

	SPI_Stop(Flash_SPI);
	HAL_Delay(10);
	SPI_TFT_Start(Flash_SPI);
	HAL_Delay(50);

	lv_task_handler();

	Motor_Protection_Init();

	while (1) {
		Door_Control();

		if (Remote_state == 1)	//LIN simulate or real
				{
			HAL_GPIO_WritePin(LIN_RELAY_GPIO_Port, LIN_RELAY_Pin, 1);

		} else {
			HAL_GPIO_WritePin(LIN_RELAY_GPIO_Port, LIN_RELAY_Pin, 0);
		}

		Lin_DataProcess_loop();

		if (id1 == 1)	//MoC
				{
//			////
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Rds x");
//			itoa(TA531_RC1_x_ready ,str1,10);
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,5, 1, str1);
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,6, 1, "y");
//			itoa(TA531_RC1_y_ready ,str1,10);
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,7, 1, str1);
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 1, "z");
//			itoa(TA531_RC1_z_ready ,str1,10);
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,9, 1, str1);
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,12, 1, "Fg");
//			itoa(TA531_RC1_fg ,str1,10);
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,15, 1, str1);

			if ((TA531_RC1.TA531_RC_X_act == TA531_RC1.TA531_RC_X_trg)) {
				TA531_RC1_x_ready = 1;
			} else {
				TA531_RC1_x_ready = 0;
			}
			if ((TA531_RC1.TA531_RC_Y_act == TA531_RC1.TA531_RC_Y_trg)) {
				TA531_RC1_y_ready = 1;
			} else {
				TA531_RC1_y_ready = 0;
			}

			Motor_Protection_Reset();
			Motor_Protection.last_X_pos = TA531_RC1.TA531_RC_X_act;
			Motor_Protection.last_Y_pos = TA531_RC1.TA531_RC_Y_act;

			while ((TA531_RC1_fg == 2)
					& ((TA531_RC1_x_ready & TA531_RC1_y_ready) != 1)) {
				MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg,
						TA531_RC1.TA531_RC_Y_trg);

//				itoa(TA531_RC1.TA531_RC_X_trg ,str1,10);
//				OLED_ShowString(OLED_I2C_ch ,OLED_type,6, 2, str1);
//				itoa(TA531_RC1.TA531_RC_Y_trg ,str1,10);
//				OLED_ShowString(OLED_I2C_ch ,OLED_type,12, 2, str1);

				HAL_Delay(MOTOR_LOOP_INTERVAL_MS);

				uint8_t protection_status = Motor_Protection_Check(
						TA531_RC1.TA531_RC_X_act, TA531_RC1.TA531_RC_Y_act,
						TA531_RC1.TA531_RC_X_trg, TA531_RC1.TA531_RC_Y_trg);

				if (protection_status != 0) {
					Motor_Protection_EmergencyStop();
					break;
				}

				if ((TA531_RC1.TA531_RC_X_act == TA531_RC1.TA531_RC_X_trg)) {
					TA531_RC1_x_ready = 1;
				} else {
					TA531_RC1_x_ready = 0;
				}
				if ((TA531_RC1.TA531_RC_Y_act == TA531_RC1.TA531_RC_Y_trg)) {
					TA531_RC1_y_ready = 1;
				} else {
					TA531_RC1_y_ready = 0;
				}
			}

			if ((TA531_RC1_x_ready == 1) & (TA531_RC1_y_ready == 1)
					& (TA531_RC1_fg < 3)) {	//reach 1st point

//				itoa(TA531_RC1.TA531_RC_X_act ,str1,10);
//				OLED_ShowString(OLED_I2C_ch ,OLED_type,6, 3, str1);
//				itoa(TA531_RC1.TA531_RC_Y_act ,str1,10);
//				OLED_ShowString(OLED_I2C_ch ,OLED_type,12, 3, str1);

				if (TA531_RC1.TA531_RC_Z_code > 0)//(TA531_RC1.TA531_RC_Z_code2 != TA531_RC1.TA531_RC_Z_code)
						{
					switch (TA531_RC1.TA531_RC_Z_code) {
					case 0:		//none
						HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin,
								0);
						break;
					case 1:
						HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin,
								1);	//F
						HAL_Delay(250);
						break;
					case 2:
						HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin,
								1);	//F
						HAL_Delay(500);
						break;
					case 3:
						HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin,
								1);	//F
						HAL_Delay(500);
						break;
					}

					if ((TA531_RC1.TA531_RC_X_Mov != 0)
							| (TA531_RC1.TA531_RC_Y_Mov != 0)) {
						int temp_x = (int) (TA531_RC1.TA531_RC_X_trg
								+ TA531_RC1.TA531_RC_X_Mov);
						int temp_y = (int) (TA531_RC1.TA531_RC_Y_trg
								+ TA531_RC1.TA531_RC_Y_Mov);
						Clamp_Position(&temp_x, &temp_y, false);  // ← 添加限制
						MotoCtrl_PositionLoop(temp_x, temp_y);
						HAL_Delay(500);
					}

					HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin, 0);	//F
					TA531_RC1.TA531_RC_Z_code2 = TA531_RC1.TA531_RC_Z_code;	// 0
					TA531_RC1.TA531_RC_X_Mov = 0;
					TA531_RC1.TA531_RC_Y_Mov = 0;

					TA531_RC1.TA531_RC_Z_code = 0;
				}

				if (TA531_RC1.TA531_RC_Reset == 1) {
					Motor_Protection_Reset();  // ← 新增
					Motor_Protection.last_X_pos = TA531_RC1.TA531_RC_X_act; // ← 新增
					Motor_Protection.last_Y_pos = TA531_RC1.TA531_RC_Y_act; // ← 新增

					TA531_RC1.TA531_RC_X_trg = 0;
					TA531_RC1.TA531_RC_Y_trg = 0;
					MotoCtrl_PositionLoop(0, 0);
					HAL_Delay(500);

					// 检查复位是否成功（可选）
					if (abs(TA531_RC1.TA531_RC_X_act) > 10
							|| abs(TA531_RC1.TA531_RC_Y_act) > 10) {
						Motor_Protection.protection_triggered = 1;
						Motor_Protection.error_type = 2;
						Motor_Protection_EmergencyStop();
					}
				}

				TA531_RC1_fg = 3;
				TA531_Lock = 0;
				TA531_RC1.TA531_RC_Reset = 0;  // 清除复位标志 ← 关键！
				TA531_RC1.TA531_RC_Z_code = 0; // 清除Z轴动作标志
				TA531_RC1.TA531_RC_X_Mov = 0;  // 清除移动偏移
				TA531_RC1.TA531_RC_Y_Mov = 0;  // 清除移动偏移
				// ==============================================

				TSA_Ack_RC_DATA[0] = TA531_RC1_Ack & 0x0f;
				TSA_Ack_RC_DATA[3] = TA531_RC1.TA531_RC_X_act & 0xff;
				TSA_Ack_RC_DATA[4] = (TA531_RC1.TA531_RC_X_act >> 8) & 0xff;
				TSA_Ack_RC_DATA[5] = TA531_RC1.TA531_RC_Y_act & 0xff;
				TSA_Ack_RC_DATA[6] = (TA531_RC1.TA531_RC_Y_act >> 8) & 0xff;
				TSA_Ack_RC_DATA[7] = 0xff;

				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA_Ack_RC_Header,
						TSA_Ack_RC_DATA);
				lvLED_Sts_TPRobot = 1;
			}

			if (TA531_Lock == 0) {
				if ((SW_UP == 1) & (SW_UP_pre == 1)) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 1, "Go X+");
					TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act + 50;
					Clamp_Position(&TA531_RC1.TA531_RC_X_trg,
							&TA531_RC1.TA531_RC_Y_trg, false);  // ← 添加限制
					TA531_RC1_fg = 2;
				} else if ((SW_UP == 1) & (SW_UP_pre == 0)) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 1, "Go X+");
					TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act + 20;
					Clamp_Position(&TA531_RC1.TA531_RC_X_trg,
							&TA531_RC1.TA531_RC_Y_trg, false);  // ← 添加限制
					TA531_RC1_fg = 2;
				} else if ((SW_DW == 1) & (SW_DW_pre == 1)) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 1, "Go X-");
					TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act - 50;

					if (TA531_RC1.TA531_RC_X_trg < 0) {
						TA531_RC1.TA531_RC_X_trg = 0;
					}
					Clamp_Position(&TA531_RC1.TA531_RC_X_trg,
							&TA531_RC1.TA531_RC_Y_trg, false);  // ← 添加限制
					TA531_RC1_fg = 2;
				} else if ((SW_DW == 1) & (SW_DW_pre == 0)) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 1, "Go X-");
					TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act - 20;

					if (TA531_RC1.TA531_RC_X_trg < 0) {
						TA531_RC1.TA531_RC_X_trg = 0;
					}
					Clamp_Position(&TA531_RC1.TA531_RC_X_trg,
							&TA531_RC1.TA531_RC_Y_trg, false);  // ← 添加限制
					TA531_RC1_fg = 2;
				} else if ((SW_LEFT == 1) & (SW_LEFT_pre == 1)) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 1, "Go Y-");
					TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act - 50;

					if (TA531_RC1.TA531_RC_Y_trg < 0) {
						TA531_RC1.TA531_RC_Y_trg = 0;
					}
					Clamp_Position(&TA531_RC1.TA531_RC_X_trg,
							&TA531_RC1.TA531_RC_Y_trg, false);  // ← 添加限制
					TA531_RC1_fg = 2;
				} else if ((SW_LEFT == 1) & (SW_LEFT_pre == 0)) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 1, "Go Y-");
					TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act - 20;

					if (TA531_RC1.TA531_RC_Y_trg < 0) {
						TA531_RC1.TA531_RC_Y_trg = 0;
					}
					Clamp_Position(&TA531_RC1.TA531_RC_X_trg,
							&TA531_RC1.TA531_RC_Y_trg, false);  // ← 添加限制
					TA531_RC1_fg = 2;
				} else if ((SW_RIGHT == 1) & (SW_RIGHT_pre == 1)) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 1, "Go Y+");
					TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act + 50;
					Clamp_Position(&TA531_RC1.TA531_RC_X_trg,
							&TA531_RC1.TA531_RC_Y_trg, false);  // ← 添加限制
					TA531_RC1_fg = 2;
				} else if ((SW_RIGHT == 1) & (SW_RIGHT_pre == 0)) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 1, "Go Y+");
					TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act + 20;
					Clamp_Position(&TA531_RC1.TA531_RC_X_trg,
							&TA531_RC1.TA531_RC_Y_trg, false);  // ← 添加限制
					TA531_RC1_fg = 2;
				} else if (SW_BUTTON == 1) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 1, "Push!");

					TA531_RC1.TA531_RC_Z_code = 1;
					TA531_RC1.TA531_RC_Z_code2 = 0;

					TA531_RC1_fg = 2;
				}
			}

		}	//	end if MoC

/////////// XL9555
//	    uint8_t XL9555_tempdata;
//	    XL9555_tempdata = XL9555_Read(XL9555_1_addr_read, 0);
//		itoa(XL9555_tempdata,str1,2);
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 3, str1);
//
//	    XL9555_tempdata = XL9555_Read(XL9555_1_addr_read, 1);
//		itoa(XL9555_tempdata,str1,2);
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 3, str1);
//
//
//		//TSA_GP_IN_DATA

		///////// Light Sensor
		uint8_t *light_a[4] = { &TA531SysEnv.TA531_env_LightA1,
				&TA531SysEnv.TA531_env_LightA2, &TA531SysEnv.TA531_env_LightA3,
				&TA531SysEnv.TA531_env_LightA4, };
		uint8_t *light_d[4] = { &TA531SysEnv.TA531_env_LightD1,
				&TA531SysEnv.TA531_env_LightD2, &TA531SysEnv.TA531_env_LightD3,
				&TA531SysEnv.TA531_env_LightD4, };

		for (uint8_t i = 0; i < 4; i++) {
			*light_a[i] = NormalizeLightSensor(adc_buffer[i]);
			*light_d[i] = EncodeLightAlarm(*light_a[i]);
		}

//	TSA1_ADC_DATA[8], TSA2_LS_DATA[8]

		TSA2_LS_DATA[0] = (TA531SysEnv.TA531_env_LightD1 & 0x03)
				+ ((TA531SysEnv.TA531_env_LightD2 & 0x03) << 2)
				+ ((TA531SysEnv.TA531_env_LightD3 & 0x03) << 4)
				+ ((TA531SysEnv.TA531_env_LightD4 & 0x03) << 6);
		TSA2_LS_DATA[2] = TA531SysEnv.TA531_env_LightA1 & 0xff;
		TSA2_LS_DATA[3] = TA531SysEnv.TA531_env_LightA2 & 0xff;
		TSA2_LS_DATA[4] = TA531SysEnv.TA531_env_LightA3 & 0xff;
		TSA2_LS_DATA[5] = TA531SysEnv.TA531_env_LightA4 & 0xff;

//		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA2_LS_Header, TSA2_LS_DATA);

//////////ADC
		TA531SysEnv.TA531_env_ADC1 = adc_buffer[4] * 0.056;
		TA531SysEnv.TA531_env_ADC2 = adc_buffer[5] * 0.056;	//*32.50 / 4096 /1.65 *11.65;

		TSA1_ADC_DATA[0] = TA531SysEnv.TA531_env_ADC1 & 0xff;
		TSA1_ADC_DATA[1] = TA531SysEnv.TA531_env_ADC2 & 0xff;

		//	HAL_Delay(20);

		if (id1 == 0)	//id1 = 0, no RC
				{
			OLED_UpdatePage_Id0(oled_line);
		}

		if (TSA3_0x52_Flag == 1) {
			if (TA531SysEnv.TA531_env_KL15 == 1) {
				HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin, 1);	//F
//				char *str = "*KL15 Relay ON";
//				OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, str );
			} else if (TA531SysEnv.TA531_env_KL15 == 0) {
				HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin, 0);
			}

			if (TA531SysEnv.TA531_env_USB1 == 0) {
				HAL_GPIO_WritePin(USB_RELAY_1_GPIO_Port, USB_RELAY_1_Pin, 0);//F
			} else if (TA531SysEnv.TA531_env_USB1 == 1) {
				HAL_GPIO_WritePin(USB_RELAY_1_GPIO_Port, USB_RELAY_1_Pin, 1);
				HAL_GPIO_WritePin(USB_RELAY_2_GPIO_Port, USB_RELAY_2_Pin, 0);
			} else if (TA531SysEnv.TA531_env_USB1 == 2) {
				HAL_GPIO_WritePin(USB_RELAY_1_GPIO_Port, USB_RELAY_1_Pin, 1);
				HAL_GPIO_WritePin(USB_RELAY_2_GPIO_Port, USB_RELAY_2_Pin, 1);
			}

			// 1			//		TA531SysEnv.TA531_env_KeyLock = (buf_rec[2]>>0)&0x03;
			if ((TA531SysEnv.TA531_env_KeyLock == 0)
					& (TA531TimCallback.TA531_Callback_flag[1] == 0)) {
				HAL_GPIO_WritePin(COM_RELAY_1_GPIO_Port, COM_RELAY_1_Pin, 0);//F
			} else if (TA531SysEnv.TA531_env_KeyLock == 3) {
				HAL_GPIO_WritePin(COM_RELAY_1_GPIO_Port, COM_RELAY_1_Pin, 1);
			} else if (TA531SysEnv.TA531_env_KeyLock == 1)	//1s
					{
//				uint8_t TA531_Callback_flag[8] ;
//				uint8_t TA531_Callback_key[8] ;
//				uint32_t TA531_Callback_tim[8] ;

				HAL_GPIO_WritePin(COM_RELAY_1_GPIO_Port, COM_RELAY_1_Pin, 1);
//				//TA531TimCallback
				TA531TimCallback.TA531_Callback_flag[1] = 1;
				TA531TimCallback.TA531_Callback_tim[1] = Sys_TIM_TICK + 1500;
			} else if (TA531SysEnv.TA531_env_KeyLock == 2)	//6s
					{
				HAL_GPIO_WritePin(COM_RELAY_1_GPIO_Port, COM_RELAY_1_Pin, 1);
//				//TA531TimCallback
				TA531TimCallback.TA531_Callback_flag[1] = 1;
				TA531TimCallback.TA531_Callback_tim[1] = Sys_TIM_TICK + 6000;
			}

			// 2			//		TA531SysEnv.TA531_env_KeyUnlock = (buf_rec[2]>>2)&0x03;
			if ((TA531SysEnv.TA531_env_KeyUnlock == 0)
					& (TA531TimCallback.TA531_Callback_flag[2] == 0)) {
				HAL_GPIO_WritePin(COM_RELAY_2_GPIO_Port, COM_RELAY_2_Pin, 0);//F
			} else if (TA531SysEnv.TA531_env_KeyUnlock == 3) {
				HAL_GPIO_WritePin(COM_RELAY_2_GPIO_Port, COM_RELAY_2_Pin, 1);
			} else if (TA531SysEnv.TA531_env_KeyUnlock == 1)	//1s
					{
				HAL_GPIO_WritePin(COM_RELAY_2_GPIO_Port, COM_RELAY_2_Pin, 1);
//				//TA531TimCallback
				TA531TimCallback.TA531_Callback_flag[2] = 1;
				TA531TimCallback.TA531_Callback_tim[2] = Sys_TIM_TICK + 1500;
			} else if (TA531SysEnv.TA531_env_KeyLock == 2)	//6s
					{
				HAL_GPIO_WritePin(COM_RELAY_2_GPIO_Port, COM_RELAY_2_Pin, 1);
//				//TA531TimCallback

				TA531TimCallback.TA531_Callback_flag[2] = 1;
				TA531TimCallback.TA531_Callback_tim[2] = Sys_TIM_TICK + 6000;
			}

			// 3		//		TA531SysEnv.TA531_env_KeyRearDoor = (buf_rec[2]>>4)&0x03;
			if ((TA531SysEnv.TA531_env_KeyRearDoor == 0)
					& (TA531TimCallback.TA531_Callback_flag[3] == 0)) {
				HAL_GPIO_WritePin(COM_RELAY_3_GPIO_Port, COM_RELAY_3_Pin, 0);//F
			} else if (TA531SysEnv.TA531_env_KeyRearDoor == 3) {
				HAL_GPIO_WritePin(COM_RELAY_3_GPIO_Port, COM_RELAY_3_Pin, 1);
			} else if (TA531SysEnv.TA531_env_KeyRearDoor == 1)	//1s
					{
				HAL_GPIO_WritePin(COM_RELAY_3_GPIO_Port, COM_RELAY_3_Pin, 1);
//				//TA531TimCallback

				TA531TimCallback.TA531_Callback_flag[3] = 1;
				TA531TimCallback.TA531_Callback_tim[3] = Sys_TIM_TICK + 1500;

			} else if (TA531SysEnv.TA531_env_KeyLock == 2)	//6s
					{
				HAL_GPIO_WritePin(COM_RELAY_3_GPIO_Port, COM_RELAY_3_Pin, 1);
//				//TA531TimCallback

				TA531TimCallback.TA531_Callback_flag[3] = 1;
				TA531TimCallback.TA531_Callback_tim[3] = Sys_TIM_TICK + 6000;
			}

			// 4		//		TA531SysEnv.TA531_env_KeyBeep = (buf_rec[2]>>6)&0x03;

			// 5		//		TA531SysEnv.TA531_env_KeyWindow = (buf_rec[3]>>0)&0x03;

			// 6		//		TA531SysEnv.TA531_env_KeyLeftDoor = (buf_rec[3]>>2)&0x03;

			// 7		//		TA531SysEnv.TA531_env_KeyRightDoor = (buf_rec[3]>>4)&0x03;

			if (TA531SysEnv.TA531_env_Relay1 == 1)	//			A Y_RELAY_1_Pin
					{
				HAL_GPIO_WritePin(Y_RELAY_1_GPIO_Port, Y_RELAY_1_Pin, 0);	//F
			} else if (TA531SysEnv.TA531_env_Relay1 == 2) {
				HAL_GPIO_WritePin(Y_RELAY_1_GPIO_Port, Y_RELAY_1_Pin, 1);
			}

			if (TA531SysEnv.TA531_env_Relay2 == 1)	//
					{
				HAL_GPIO_WritePin(Y_RELAY_2_GPIO_Port, Y_RELAY_2_Pin, 0);	//F
			} else if (TA531SysEnv.TA531_env_Relay2 == 2) {
				HAL_GPIO_WritePin(Y_RELAY_2_GPIO_Port, Y_RELAY_2_Pin, 1);
			}

			if (TA531SysEnv.TA531_env_Relay3 == 1)	//
					{
				HAL_GPIO_WritePin(Y_RELAY_3_GPIO_Port, Y_RELAY_3_Pin, 0);	//F
			} else if (TA531SysEnv.TA531_env_Relay3 == 2) {
				HAL_GPIO_WritePin(Y_RELAY_3_GPIO_Port, Y_RELAY_3_Pin, 1);
			}

			if (TA531SysEnv.TA531_env_Relay4 == 1)	//
					{
				HAL_GPIO_WritePin(Y_RELAY_4_GPIO_Port, Y_RELAY_4_Pin, 0);	//F
			} else if (TA531SysEnv.TA531_env_Relay4 == 2) {
				HAL_GPIO_WritePin(Y_RELAY_4_GPIO_Port, Y_RELAY_4_Pin, 1);
			}

			if (TA531SysEnv.TA531_env_Relay5 == 1)	//			B
					{
				HAL_GPIO_WritePin(Y_RELAY_5_GPIO_Port, Y_RELAY_5_Pin, 0);	//F
			} else if (TA531SysEnv.TA531_env_Relay5 == 2) {
				HAL_GPIO_WritePin(Y_RELAY_5_GPIO_Port, Y_RELAY_5_Pin, 1);
			}

			if (TA531SysEnv.TA531_env_Relay6 == 1)	//			B
					{
				HAL_GPIO_WritePin(Y_RELAY_6_GPIO_Port, Y_RELAY_6_Pin, 0);	//F
			} else if (TA531SysEnv.TA531_env_Relay6 == 2) {
				HAL_GPIO_WritePin(Y_RELAY_6_GPIO_Port, Y_RELAY_6_Pin, 1);
			}

			TSA_Ack_DATA[3] = 0xff;
			TSA3_0x52_Flag = 0;
		}

		if (TSA4_0x53_Flag == 1) {

			if (TA531SysEnv.TA531_env_WindowFL == 0)//			B		//无输�?????????????????????????????????????	上升	手动
					{
			} else if (TA531SysEnv.TA531_env_WindowFL == 1)	//			B
					{
			} else if (TA531SysEnv.TA531_env_WindowFL == 2) {
			} else if (TA531SysEnv.TA531_env_WindowFL == 3) {
			} else if (TA531SysEnv.TA531_env_WindowFL == 4) {
			}
			if (TA531SysEnv.TA531_env_WindowFR == 0)	//			B
					{
			} else if (TA531SysEnv.TA531_env_WindowFR == 1)	//			B
					{
			} else if (TA531SysEnv.TA531_env_WindowFR == 2) {
			} else if (TA531SysEnv.TA531_env_WindowFR == 3) {
			} else if (TA531SysEnv.TA531_env_WindowFR == 4) {
			}
			if (TA531SysEnv.TA531_env_WindowRL == 0)	//			RL DDDD
					{
			} else if (TA531SysEnv.TA531_env_WindowRL == 1)	//			RL DDDD
					{
			} else if (TA531SysEnv.TA531_env_WindowRL == 2) {
			} else if (TA531SysEnv.TA531_env_WindowRL == 3) {
			} else if (TA531SysEnv.TA531_env_WindowRL == 4) {
			}
			if (TA531SysEnv.TA531_env_WindowRR == 0)	//			RR DDDD
					{
			} else if (TA531SysEnv.TA531_env_WindowRR == 1)	//			RR DDDD
					{
			} else if (TA531SysEnv.TA531_env_WindowRR == 2) {
			} else if (TA531SysEnv.TA531_env_WindowRR == 3) {
			} else if (TA531SysEnv.TA531_env_WindowRR == 4) {
			}
			if (TA531SysEnv.TA531_env_HSD12_1 == 0)	//
					{
				HAL_GPIO_WritePin(DOOR_RELAY_HSD1_GPIO_Port,
				DOOR_RELAY_HSD1_Pin, 0);
			} else if (TA531SysEnv.TA531_env_HSD12_1 == 1)	//
					{
				HAL_GPIO_WritePin(DOOR_RELAY_HSD1_GPIO_Port,
				DOOR_RELAY_HSD1_Pin, 1);
			}

			if (TA531SysEnv.TA531_env_HSD12_2 == 0)	//
					{
				HAL_GPIO_WritePin(DOOR_RELAY_HSD2_GPIO_Port,
				DOOR_RELAY_HSD2_Pin, 0);
			} else if (TA531SysEnv.TA531_env_HSD12_2 == 1)	//
					{
				HAL_GPIO_WritePin(DOOR_RELAY_HSD2_GPIO_Port,
				DOOR_RELAY_HSD2_Pin, 1);
			}
			if (TA531SysEnv.TA531_env_HSD12_3 == 0)	//
					{
				HAL_GPIO_WritePin(DOOR_RELAY_HSD3_GPIO_Port,
				DOOR_RELAY_HSD3_Pin, 0);
			} else if (TA531SysEnv.TA531_env_HSD12_3 == 1)	//
					{
				HAL_GPIO_WritePin(DOOR_RELAY_HSD3_GPIO_Port,
				DOOR_RELAY_HSD3_Pin, 1);
			}
			if (TA531SysEnv.TA531_env_HSD12_4 == 0)	//
					{
				HAL_GPIO_WritePin(DOOR_RELAY_HSD4_GPIO_Port,
				DOOR_RELAY_HSD4_Pin, 0);
			} else if (TA531SysEnv.TA531_env_HSD12_4 == 1)	//
					{
				HAL_GPIO_WritePin(DOOR_RELAY_HSD4_GPIO_Port,
				DOOR_RELAY_HSD4_Pin, 1);
			}
			TA531_Door.Door_FL = (TA531SysEnv.TA531_env_DoorSwFL & 0x03); // 左前门
			TA531_Door.Door_FR = (TA531SysEnv.TA531_env_DoorSwFR & 0x03); // 右前门
			TA531_Door.Door_RL = (TA531SysEnv.TA531_env_DoorSwRL & 0x03); // 左后门
			TA531_Door.Door_Hood = (TA531SysEnv.TA531_env_DoorSwF & 0x03); // 前引擎盖 ← 改
			TA531_Door.Door_Trunk = (TA531SysEnv.TA531_env_DoorSwR & 0x03); // 后备箱 ← 改
			TA531_Door.Door_Reserve = (TA531SysEnv.TA531_env_DoorSwRR & 0x03);

			TSA4_0x53_Flag = 0;
			TSA_Ack_DATA[4] = 0xff;
		}

		if (TSA4_0x54_Flag == 1) {
			PWMServo2_3_AGout(TA531SysEnv.TA531_env_PWM_Ag_1);
			PWMServo2_4_AGout(TA531SysEnv.TA531_env_PWM_Ag_2);
			PWMServo3_1_AGout(TA531SysEnv.TA531_env_PWM_Ag_3);
			PWMServo3_2_AGout(TA531SysEnv.TA531_env_PWM_Ag_4);

			TSA4_0x54_Flag = 0;
			TSA_Ack_DATA[4] = 0xff;
		}
//		if (TSA4_0x104_Flag == 1)
//		{
//		    Lin_DataProcess_loop();  // ← 添加这行
//		    TSA4_0x104_Flag = 0;
//		    TSA_Ack_DATA[5] = 0xff;
//		}

		//		uint8_t TA531_Callback_flag[8] ;
		//		uint8_t TA531_Callback_key[8] ;
		//		uint32_t TA531_Callback_tim[8] ;
		for (int i = 1; i < 8; i++)		//release key
				{
			if (TA531TimCallback.TA531_Callback_flag[i] == 1) {
				if (TA531TimCallback.TA531_Callback_tim[i] < Sys_TIM_TICK) {
					switch (i) {
					case 1:		//
						HAL_GPIO_WritePin(COM_RELAY_1_GPIO_Port,
						COM_RELAY_1_Pin, 0);
						break;
					case 2:		//
						HAL_GPIO_WritePin(COM_RELAY_2_GPIO_Port,
						COM_RELAY_2_Pin, 0);
						break;
					case 3:		//
						HAL_GPIO_WritePin(COM_RELAY_3_GPIO_Port,
						COM_RELAY_3_Pin, 0);
						break;
//					case 4:		//
//						HAL_GPIO_WritePin(GPIOE, COM_RELAY_4_Pin, 0);
//						break;
//					case 5:		//
//						HAL_GPIO_WritePin(GPIOE, COM_RELAY_5_Pin, 0);
//						break;
//					case 6:		//
//						HAL_GPIO_WritePin(GPIOB, COM_RELAY_6_Pin, 0);
//						break;
//					case 7:		//
//						HAL_GPIO_WritePin(GPIOB, COM_RELAY_7_Pin, 0);
//						break;
					}

					TA531TimCallback.TA531_Callback_flag[i] = 0;
				}
			}
		}

//////TEST LIN
//		TSA4_0x103_Flag = 1;
//		TA531_LIN_SWS.LIN_SWS_SWSCnfmSwA_1 = 	SW_BUTTON;
//		TA531_LIN_SWS.LIN_SWS_SWSSelUpSwA_1 = 	SW_UP;
//		TA531_LIN_SWS.LIN_SWS_SWSSelDwnSwA_1 = 	SW_DW;
//		TA531_LIN_SWS.LIN_SWS_SWSSelLSwA_1 = 	SW_LEFT;
//		TA531_LIN_SWS.LIN_SWS_SWSSelRSwA_1 = 	SW_RIGHT;

//		TSA4_0x104_Flag = 1;
//		if(SW_BUTTON)
//		{
//			Remote_state = 1;
//		}
//		TA531_LIN_SWS_G3.SWSCnfmSwReq_l = 	SW_BUTTON;
//		TA531_LIN_SWS_G3.SWSSelUpSwReq_l = 	SW_UP;
//		TA531_LIN_SWS_G3.SWSSelDwnSwReq_l = 	SW_DW;
//		TA531_LIN_SWS_G3.SWSSelLSwReq_l = 	SW_LEFT;
//		TA531_LIN_SWS_G3.SWSSelRSwReq_l = 	SW_RIGHT;

		if (temper_flag == 0) {
			SPI_Stop(Flash_SPI);
			HAL_Delay(5);
			SPI_TFT_Start(Flash_SPI);
			HAL_Delay(5);

			itoa(temp1075_ZXD_i, str1, 10);
			strcat(str1, ".");
			itoa(temp1075_ZXD_f, str2, 10);
			strncat(str1, str2, 1);
			lv_label_set_text(guider_ui.screen_label_Temp_ZXD, str1);
			lv_arc_set_value(guider_ui.screen_arc_TEMP_ZXD, temp1075_ZXD_i);

			itoa(temp1075_ZPD, str1, 10);
			lv_label_set_text(guider_ui.screen_label_Temp_ZPD, str1);
			lv_arc_set_value(guider_ui.screen_arc_TEMP_ZPD, temp1075_ZPD);

			itoa(temp1075_LHZCU, str1, 10);
			lv_label_set_text(guider_ui.screen_label_Temp_LHZCU, str1);
			lv_arc_set_value(guider_ui.screen_arc_TEMP_LHZCU, temp1075_LHZCU);

			itoa(temp1075_RHZCU, str1, 10);
			lv_label_set_text(guider_ui.screen_label_Temp_RHZCU, str1);
			lv_arc_set_value(guider_ui.screen_arc_TEMP_RHZCU, temp1075_RHZCU);

			itoa(temp1075_RZCU, str1, 10);
			lv_label_set_text(guider_ui.screen_label_Temp_RZCU, str1);
			lv_arc_set_value(guider_ui.screen_arc_TEMP_RZCU, temp1075_RZCU);

//			lvLED_Sts_TPRobot, lvLED_Sts_LIN, lvLED_Sts_CAN, lvLED_Sts_Sensor;
//			// 0 	GUI_GRAY,	1	GUI_GREEN,	2	GUI_ORANGE,	3	GUI_RED
//
//			lv_led_set_color(guider_ui.screen_led_TP, lv_color_hex(GUI_GRAY));
//			lv_led_set_color(guider_ui.screen_led_LIN, lv_color_hex(GUI_GREEN));
//			lv_led_set_color(guider_ui.screen_led_CAN, lv_color_hex(GUI_ORANGE));
//			lv_led_set_color(guider_ui.screen_led_Sensor, lv_color_hex(GUI_RED));

			switch (lvLED_Sts_TPRobot) {
			case 0:		//
				lv_led_set_color(guider_ui.screen_led_TP,
						lv_color_hex(GUI_GRAY));
				break;
			case 1:		//
				lv_led_set_color(guider_ui.screen_led_TP,
						lv_color_hex(GUI_GREEN));
				break;
			case 2:		//
				lv_led_set_color(guider_ui.screen_led_TP,
						lv_color_hex(GUI_ORANGE));
				break;
			case 3:		//
				lv_led_set_color(guider_ui.screen_led_TP,
						lv_color_hex(GUI_RED));
				break;
			}

			switch (lvLED_Sts_LIN) {
			case 0:		//
				lv_led_set_color(guider_ui.screen_led_LIN,
						lv_color_hex(GUI_GRAY));
				break;
			case 1:		//
				lv_led_set_color(guider_ui.screen_led_LIN,
						lv_color_hex(GUI_GREEN));
				break;
			case 2:		//
				lv_led_set_color(guider_ui.screen_led_LIN,
						lv_color_hex(GUI_ORANGE));
				break;
			case 3:		//
				lv_led_set_color(guider_ui.screen_led_LIN,
						lv_color_hex(GUI_RED));
				break;
			}

			switch (lvLED_Sts_CAN) {
			case 0:		//
				lv_led_set_color(guider_ui.screen_led_CAN,
						lv_color_hex(GUI_GRAY));
				break;
			case 1:		//
				lv_led_set_color(guider_ui.screen_led_CAN,
						lv_color_hex(GUI_GREEN));
				break;
			case 2:		//
				lv_led_set_color(guider_ui.screen_led_CAN,
						lv_color_hex(GUI_ORANGE));
				break;
			case 3:		//
				lv_led_set_color(guider_ui.screen_led_CAN,
						lv_color_hex(GUI_RED));
				break;
			}

			switch (lvLED_Sts_Sensor) {
			case 0:		//
				lv_led_set_color(guider_ui.screen_led_Sensor,
						lv_color_hex(GUI_GRAY));
				break;
			case 1:		//
				lv_led_set_color(guider_ui.screen_led_Sensor,
						lv_color_hex(GUI_GREEN));
				break;
			case 2:		//
				lv_led_set_color(guider_ui.screen_led_Sensor,
						lv_color_hex(GUI_ORANGE));
				break;
			case 3:		//
				lv_led_set_color(guider_ui.screen_led_Sensor,
						lv_color_hex(GUI_RED));
				break;
			}

			lv_task_handler();
			HAL_Delay(1);
//			SPI_Stop(Flash_SPI);
//			HAL_Delay(5);

		} else	//(temper_flag == 1)
		{
			//float temp1075_ZXD , temp1075_IPD , temp1075_LHZCU , temp1075_RHZCU , temp1075_RZCU , temp1075_Bk1 , temp1075_Bk2 , temp1075_Bk3;
			temp1075_ZXD = readTemperature(temp1075_ch, TempSensor_ADDR_ZXD);
			temp1075_ZPD = readTemperature(temp1075_ch, TempSensor_ADDR_ZPD);
			temp1075_LHZCU = readTemperature(temp1075_ch,
			TempSensor_ADDR_LHZCU);
			temp1075_RHZCU = readTemperature(temp1075_ch,
			TempSensor_ADDR_RHZCU);
			temp1075_RZCU = readTemperature(temp1075_ch, TempSensor_ADDR_RZCU);

			if (temp1075_ZXD < 10) {
				lvLED_Sts_Sensor = 0;
			} else if ((temp1075_ZXD < 50) & (temp1075_ZPD < 50)) {
				lvLED_Sts_Sensor = 1;
			} else if ((temp1075_ZXD < 75) & (temp1075_ZPD < 75)) {
				lvLED_Sts_Sensor = 2;
			} else {
				lvLED_Sts_Sensor = 3;
			}

			temp1075_ZXD_i = truncf(temp1075_ZXD);
			temp1075_ZXD_f = (temp1075_ZXD - temp1075_ZXD_i) * 100;

//			itoa(temp1075_ZXD_i,str1,10);
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, str1);
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,2, 2, ".");
//			itoa(temp1075_ZXD_f,str1,10);
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,3, 2, str1);

			temper_flag = 0;
		}
		if (id1 == 1)  // MoC模式下显示
				{
			OLED_UpdatePage_Id1(oled_line);
		}
		// =======================================
	}  //while

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
}

/* ====================== 可交接功能函数实现（中文） ======================
 * 目的：把易改需求（Flash开机策略/OLED显示策略）拆成独立函数，便于交接维护。
 */
static void Boot_RunFlashSelfTest_AndLoadUID(char *str_buf) {
	uint8_t temp1[4], temp2[4];
	uint64_t UID;

	temp1[0] = 123;
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Flash Test");

	SPI_Flash_Start(Flash_SPI);
	HAL_Delay(5);
	SPI_Flash_WtritEnable();
	HAL_Delay(5);
	SPI_Flash_WriteSomeBytes(temp1, Sys_Addr_DispTest, sizeof(int));
	HAL_Delay(5);
	SPI_Flash_ReadBytes(temp2, Sys_Addr_DispTest, sizeof(int));

	while (temp1[0] != temp2[0]) {
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Flash Test Err..");
		SPI_Stop(Flash_SPI);
		HAL_Delay(5);
		SPI_Flash_Start(Flash_SPI);
		HAL_Delay(5);
		SPI_Flash_WriteSomeBytes(temp1, Sys_Addr_DispTest, sizeof(int));
		HAL_Delay(5);
		SPI_Flash_ReadBytes(temp2, Sys_Addr_DispTest, sizeof(int));

		itoa(temp1[0], str_buf, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, str_buf);
		itoa(temp1[1], str_buf, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 4, 2, str_buf);
		itoa(temp2[0], str_buf, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 8, 2, str_buf);
		itoa(temp2[1], str_buf, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 12, 2, str_buf);
	}

	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Flash Test OK!");
	HAL_Delay(1000);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "UniqID:");
	HAL_Delay(1);
	UID = SPI_Flash_GUID();
	SPI_Stop(Flash_SPI);

	if ((UID == 0) | ((UID & 0xffff) == 0xffff)) {
		OLED_ShowString(OLED_I2C_ch, OLED_type, 8, 1, "Error! ");
		EncrypKey = 0x36;
	} else {
		itoa(UID, str_buf, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 8, 1, str_buf);
		EncrypKey = UID & 0xff;
	}
	HAL_Delay(500);
}

static void Boot_SkipFlashSelfTest(void) {
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Skip Flash Test ");
	EncrypKey = 0x36;
	HAL_Delay(200);
}

static void OLED_ShowRIDFlagsLine(uint8_t row, char *oled_line) {
	snprintf(oled_line, 17, "22%c34%c35%c36%c",
			(DEBUG_RID22_Count > 0) ? 'Y' : '-',
			(DEBUG_RID34_Count > 0) ? 'Y' : '-',
			(DEBUG_RID35_Count > 0) ? 'Y' : '-',
			(DEBUG_RID36_Count > 0) ? 'Y' : '-');
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, row, oled_line);
}

static void OLED_UpdatePage_Id0(char *oled_line) {
	snprintf(oled_line, 17, "L1:%3d L2:%3d", TA531SysEnv.TA531_env_LightA1,
			TA531SysEnv.TA531_env_LightA2);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, oled_line);

	snprintf(oled_line, 17, "L3:%3d L4:%3d", TA531SysEnv.TA531_env_LightA3,
			TA531SysEnv.TA531_env_LightA4);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, oled_line);

#if CFG_OLED_SHOW_RID_FLAGS_EN
	/* A 版本：第4行显示 LIN RID 收包状态，方便联调 */
	OLED_ShowRIDFlagsLine(3, oled_line);
#else
	/* B 版本：第4行显示传统 A1/A2（不看 RID） */
	snprintf(oled_line, 17, "A1:%3d A2:%3d", TA531SysEnv.TA531_env_ADC1,
			TA531SysEnv.TA531_env_ADC2);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3, oled_line);
#endif
}

static void OLED_UpdatePage_Id1(char *oled_line) {
	snprintf(oled_line, 17, "X:%4d Y:%4d", TA531_RC1.TA531_RC_X_act,
			TA531_RC1.TA531_RC_Y_act);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 0, oled_line);

	snprintf(oled_line, 17, "L1:%3d L2:%3d", TA531SysEnv.TA531_env_LightA1,
			TA531SysEnv.TA531_env_LightA2);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, oled_line);

	snprintf(oled_line, 17, "A1:%3d A2:%3d", TA531SysEnv.TA531_env_ADC1,
			TA531SysEnv.TA531_env_ADC2);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, oled_line);

#if CFG_OLED_SHOW_RID_FLAGS_EN
	/* A 版本：第4行显示 LIN RID 收包状态，方便联调 */
	OLED_ShowRIDFlagsLine(3, oled_line);
#else
	/* B 版本：保留空白，避免遮挡 MoC 调试显示 */
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3, "                ");
#endif
}

static void Lin_RearmUart3(void) {
	LIN_RESET(&huart3);
	HAL_UART_Receive_IT(&huart3, u3RxData, LIN_Data_LENGTH);
}

static bool Lin_HandleKnownRid(uint8_t rid) {
	if (rid == 0x22) {  // SWS response
		Lin_HandleRid22();
		return true;
	}
	if (rid == 0x34) {  // EBS_ICCLIN1_FrP00_ICC_LIN1
		Lin_HandleRid34();
		return true;
	}
	if (rid == 0x35) {  // EBS_ICCLIN1_FrP01_ICC_LIN1
		Lin_HandleRid35();
		return true;
	}
	if (rid == 0x36) {  // EBS_ICCLIN1_FrP02_ICC_LIN1
		Lin_HandleRid36();
		return true;
	}
	return false;
}

// 输入: rid=0x22，输出: 发送 SWS_0x22_Data；副作用: 清 SWS_0x22_Flag、重启LIN接收
static void Lin_HandleRid22(void) {
	DEBUG_LIN_Send_Count++;
	Lin_SendData(SWS_0x22_Data);
	SWS_0x22_Flag = 0;
	DataProcess = 0;
	Lin_RearmUart3();
}

// 输入: rid=0x34，输出: 发送 EBS_0x0_Data；副作用: 计数+重启LIN接收
static void Lin_HandleRid34(void) {
	DEBUG_RID34_Count++;
	DEBUG_LIN_Send_Count++;
	Build_EBS_0x34_Data();
	Lin_SendData(EBS_0x0_Data);
	DataProcess = 0;
	Lin_RearmUart3();
}

// 输入: rid=0x35，输出: 发送 EBS_0x1_Data；副作用: 计数+重启LIN接收
static void Lin_HandleRid35(void) {
	DEBUG_RID35_Count++;
	DEBUG_LIN_Send_Count++;
	Build_EBS_0x35_Data();
	Lin_SendData(EBS_0x1_Data);
	DataProcess = 0;
	Lin_RearmUart3();
}

// 输入: rid=0x36，输出: 发送 EBS_0x2_Data；副作用: 计数+重启LIN接收
static void Lin_HandleRid36(void) {
	DEBUG_RID36_Count++;
	DEBUG_LIN_Send_Count++;
	Build_EBS_0x36_Data();
	Lin_SendData(EBS_0x2_Data);
	DataProcess = 0;
	Lin_RearmUart3();
}

// 输入: u3RxData/LIN_Data_LENGTH，输出: 更新 ReceiveData/RxData
static void Lin_ReadRxDataFromUart3(void) {
	if (LIN_Data_LENGTH == 1) {
		ReceiveData = u3RxData[0];
	} else {
		for (int i = 0; i < LIN_Data_LENGTH; i++) {
			RxData[i] = u3RxData[i];
		}
		ReceiveData = u3RxData[0];
	}
}

// 输入: ReceiveData，输出: 更新 ReceivePID/ReceiveID 和 DEBUG 统计
static void Lin_UpdateDebugOnRx(void) {
	ReceivePID = ReceiveData;
	ReceiveID = ReceivePID & 0x3f;
	DEBUG_ReceivePID = ReceivePID;

	DEBUG_UART_RX_Count++;
	DEBUG_ReceiveID = ReceiveID;
	DEBUG_DataProcess = DataProcess;
	if (ReceiveID == 0x22) {
		DEBUG_RID22_Count++;
	}
}

static void Lin_HandleUnknownRid(void) {
	DataReceiveflag = 1;
	DataProcess = 2;
	Lin_RearmUart3();
}

static void Uart_RearmByHandle(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		HAL_UART_Receive_IT(&huart1, u1RxData, LIN_Data_LENGTH);
	} else if (huart == &huart2) {
		HAL_UART_Receive_IT(&huart2, u2RxData, LIN_Data_LENGTH);
	} else if (huart == &huart3) {
		HAL_UART_Receive_IT(&huart3, u3RxData, LIN_Data_LENGTH);
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 6;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_7CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_7CYCLES_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_6;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
	hcrc.Init.GeneratingPolynomial = 32773;
	hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
	hcrc.Init.InitValue = 0xFFFF;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */
	//hcrc.Init.GeneratingPolynomial = 32773; //0x8005
	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void) {

	/* USER CODE BEGIN FDCAN1_Init 0 */

	/* USER CODE END FDCAN1_Init 0 */

	/* USER CODE BEGIN FDCAN1_Init 1 */

	/* USER CODE END FDCAN1_Init 1 */
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = DISABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	hfdcan1.Init.NominalPrescaler = 16;
	hfdcan1.Init.NominalSyncJumpWidth = 1;
	hfdcan1.Init.NominalTimeSeg1 = 5;
	hfdcan1.Init.NominalTimeSeg2 = 2;
	hfdcan1.Init.DataPrescaler = 16;
	hfdcan1.Init.DataSyncJumpWidth = 1;
	hfdcan1.Init.DataTimeSeg1 = 5;
	hfdcan1.Init.DataTimeSeg2 = 2;
	hfdcan1.Init.StdFiltersNbr = 28;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN1_Init 2 */

	for (uint8_t i = 0; i < 28; i++) {
		FDCAN_Filter1[i].IdType = FDCAN_STANDARD_ID;
		FDCAN_Filter1[i].FilterIndex = i;
		FDCAN_Filter1[i].FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

		FDCAN_Filter1[i].FilterType = FDCAN_FILTER_RANGE;
		FDCAN_Filter1[i].FilterID1 = 0x000;
		FDCAN_Filter1[i].FilterID2 = 0x7FF;
	}

	//24,25,26,27
	FDCAN_Filter1[1].FilterID1 = 0x0050;
	FDCAN_Filter1[1].FilterID2 = 0x0055;

	FDCAN_Filter1[2].FilterID1 = 0x0060;
	FDCAN_Filter1[2].FilterID2 = 0x0066;

	FDCAN_Filter1[3].FilterID1 = 0x00100;
	FDCAN_Filter1[3].FilterID2 = 0x00106;

	FDCAN_Filter1[4].FilterID1 = 0x00530;
	FDCAN_Filter1[4].FilterID2 = 0x00532;

	for (uint8_t i = 0; i < 28; i++) {
		if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter1[i]) != HAL_OK) //过滤器初始化
				{
			Error_Handler();
		}
	}

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, DISABLE,
			DISABLE); //FDCAN_ACCEPT_IN_RX_FIFO0
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	/* USER CODE END FDCAN1_Init 2 */

}

/**
 * @brief FDCAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN2_Init(void) {

	/* USER CODE BEGIN FDCAN2_Init 0 */

	/* USER CODE END FDCAN2_Init 0 */

	/* USER CODE BEGIN FDCAN2_Init 1 */

	/* USER CODE END FDCAN2_Init 1 */
	hfdcan2.Instance = FDCAN2;
	hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan2.Init.AutoRetransmission = ENABLE;
	hfdcan2.Init.TransmitPause = ENABLE;
	hfdcan2.Init.ProtocolException = ENABLE;
	hfdcan2.Init.NominalPrescaler = 64;
	hfdcan2.Init.NominalSyncJumpWidth = 1;
	hfdcan2.Init.NominalTimeSeg1 = 5;
	hfdcan2.Init.NominalTimeSeg2 = 2;
	hfdcan2.Init.DataPrescaler = 16;
	hfdcan2.Init.DataSyncJumpWidth = 1;
	hfdcan2.Init.DataTimeSeg1 = 5;
	hfdcan2.Init.DataTimeSeg2 = 2;
	hfdcan2.Init.StdFiltersNbr = 28;
	hfdcan2.Init.ExtFiltersNbr = 0;
	hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN2_Init 2 */

	FDCAN_FilterTypeDef sFilterConfig2;

	sFilterConfig2.IdType = FDCAN_STANDARD_ID;
	sFilterConfig2.FilterIndex = 0;
	sFilterConfig2.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;

	// 接收所有标准帧 (0x000 ~ 0x7FF)
	sFilterConfig2.FilterID1 = 0x000;
	sFilterConfig2.FilterID2 = 0x7FF;

	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig2) != HAL_OK) {
		Error_Handler();
	}

	// 全局过滤器：接受所有标准/扩展帧到FIFO1
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
	FDCAN_ACCEPT_IN_RX_FIFO1,   // 标准帧 → FIFO1
			FDCAN_ACCEPT_IN_RX_FIFO1,   // 扩展帧 → FIFO1
			DISABLE, DISABLE);

	// ✅✅✅ 关键代码：启动FDCAN2 ✅✅✅
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
		Error_Handler();
	}

	// ✅✅✅ 关键代码：激活接收中断 ✅✅✅
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	/* USER CODE END FDCAN2_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00C12166;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x00C12166;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 63;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 200;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 63;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 20000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 1500;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 63;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 20000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 64000 - 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 500 - 1;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 64000 - 1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 100 - 1;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */

	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 64000 - 1;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 1000 - 1;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */

}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void) {

	/* USER CODE BEGIN TIM15_Init 0 */

	/* USER CODE END TIM15_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 63;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 1000 - 1;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM15_Init 2 */

	/* USER CODE END TIM15_Init 2 */
	HAL_TIM_MspPostInit(&htim15);

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 64000 - 1;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 50 - 1;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 64 - 1;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 1000 - 1;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */

	/* USER CODE END TIM17_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 19200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_LIN_Init(&huart1, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	HAL_UART_Receive_IT(&huart1, u1RxData, Serial_Data_LENGTH);
	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_LIN_Init(&huart2, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_LIN_Init(&huart3, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
	DOOR_RELAY_HSD1_Pin | DOOR_RELAY_HSD2_Pin | Y_RELAY_1_Pin | Y_RELAY_2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
			DOOR_RELAY_HSD3_Pin | DOOR_RELAY_HSD4_Pin | COM_RELAY_1_Pin
					| COM_RELAY_2_Pin | COM_RELAY_3_Pin | LIN_RELAY_Pin
					| SPI_Flash_NSS_Pin | SPI_TFT_NSS_Pin | IO_SYS_LED_B_Pin
					| IO_SYS_LED_G_Pin | IO_SYS_LED_R_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF,
			ST7789_DC_Pin | USB_RELAY_1_Pin | USB_RELAY_2_Pin | Y_RELAY_6_Pin
					| KL15_RELAY_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Y_RELAY_3_Pin | Y_RELAY_4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Y_RELAY_5_GPIO_Port, Y_RELAY_5_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	LIN3_EN_Pin | Eshft_RELAY_Pin | LIN1_EN_Pin | LIN2_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : DOOR_RELAY_HSD1_Pin DOOR_RELAY_HSD2_Pin */
	GPIO_InitStruct.Pin = DOOR_RELAY_HSD1_Pin | DOOR_RELAY_HSD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : DOOR_RELAY_HSD3_Pin DOOR_RELAY_HSD4_Pin */
	GPIO_InitStruct.Pin = DOOR_RELAY_HSD3_Pin | DOOR_RELAY_HSD4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : XL9555_1_INT_Pin */
	GPIO_InitStruct.Pin = XL9555_1_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(XL9555_1_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : XL9555_2_INT_Pin */
	GPIO_InitStruct.Pin = XL9555_2_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(XL9555_2_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ST7789_DC_Pin */
	GPIO_InitStruct.Pin = ST7789_DC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(ST7789_DC_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_RELAY_1_Pin USB_RELAY_2_Pin Y_RELAY_6_Pin KL15_RELAY_Pin */
	GPIO_InitStruct.Pin = USB_RELAY_1_Pin | USB_RELAY_2_Pin | Y_RELAY_6_Pin
			| KL15_RELAY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : Y_RELAY_1_Pin Y_RELAY_2_Pin */
	GPIO_InitStruct.Pin = Y_RELAY_1_Pin | Y_RELAY_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Y_RELAY_3_Pin Y_RELAY_4_Pin */
	GPIO_InitStruct.Pin = Y_RELAY_3_Pin | Y_RELAY_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : Y_RELAY_5_Pin */
	GPIO_InitStruct.Pin = Y_RELAY_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(Y_RELAY_5_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : COM_RELAY_1_Pin COM_RELAY_2_Pin COM_RELAY_3_Pin LIN_RELAY_Pin
	 SPI_Flash_NSS_Pin SPI_TFT_NSS_Pin */
	GPIO_InitStruct.Pin = COM_RELAY_1_Pin | COM_RELAY_2_Pin | COM_RELAY_3_Pin
			| LIN_RELAY_Pin | SPI_Flash_NSS_Pin | SPI_TFT_NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : LIN3_EN_Pin Eshft_RELAY_Pin LIN1_EN_Pin LIN2_EN_Pin */
	GPIO_InitStruct.Pin = LIN3_EN_Pin | Eshft_RELAY_Pin | LIN1_EN_Pin
			| LIN2_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : LIN3_NRES_Pin IO_CFG_2_Pin IO_CFG_3_Pin IO_CFG_4_Pin */
	GPIO_InitStruct.Pin = LIN3_NRES_Pin | IO_CFG_2_Pin | IO_CFG_3_Pin
			| IO_CFG_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : LIN1_NRES_Pin */
	GPIO_InitStruct.Pin = LIN1_NRES_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LIN1_NRES_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LIN2_NRES_Pin IO_CFG_1_Pin */
	GPIO_InitStruct.Pin = LIN2_NRES_Pin | IO_CFG_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : SW_RIGHT_Pin SW_BUTTON_Pin */
	GPIO_InitStruct.Pin = SW_RIGHT_Pin | SW_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : SW_UP_Pin SW_LEFT_Pin SW_DOWN_Pin */
	GPIO_InitStruct.Pin = SW_UP_Pin | SW_LEFT_Pin | SW_DOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : IO_SYS_LED_B_Pin IO_SYS_LED_G_Pin IO_SYS_LED_R_Pin */
	GPIO_InitStruct.Pin =
	IO_SYS_LED_B_Pin | IO_SYS_LED_G_Pin | IO_SYS_LED_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		if (hfdcan == &hfdcan1) 	//CAN1 Classic CAN
				{
			uint8_t buf_rec[8];

			while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)) {
				HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
						&FDCAN1_RxHeader, buf_rec);

				TA531_Lock = 1;
				Remote_state = 1;

				if (FDCAN1_RxHeader.Identifier == 0x052)	//TSA_3
						{
					TA531SysEnv.TA531_env_KL15 = buf_rec[0] & 0x03;
					TA531SysEnv.TA531_env_USB1 = buf_rec[1] & 0x0F;
					TA531SysEnv.TA531_env_KeyLock = (buf_rec[2] >> 0) & 0x03;
					TA531SysEnv.TA531_env_KeyUnlock = (buf_rec[2] >> 2) & 0x03;
					TA531SysEnv.TA531_env_KeyRearDoor = (buf_rec[2] >> 4)
							& 0x03;
					TA531SysEnv.TA531_env_KeyBeep = (buf_rec[2] >> 6) & 0x03;

					TA531SysEnv.TA531_env_KeyWindow = (buf_rec[3] >> 0) & 0x03;
					TA531SysEnv.TA531_env_KeyLeftDoor = (buf_rec[3] >> 2)
							& 0x03;
					TA531SysEnv.TA531_env_KeyRightDoor = (buf_rec[3] >> 4)
							& 0x03;

					TA531SysEnv.TA531_env_Relay1 = (buf_rec[4] >> 0) & 0x03;
					TA531SysEnv.TA531_env_Relay2 = (buf_rec[4] >> 2) & 0x03;
					TA531SysEnv.TA531_env_Relay3 = (buf_rec[4] >> 4) & 0x03;
					TA531SysEnv.TA531_env_Relay4 = (buf_rec[4] >> 6) & 0x03;
					TA531SysEnv.TA531_env_Relay5 = (buf_rec[5] >> 0) & 0x03;
					TA531SysEnv.TA531_env_Relay6 = (buf_rec[5] >> 2) & 0x03;

					TSA_Ack_DATA[3] = 1;
					TSA3_0x52_Flag = 1;
				} else if (FDCAN1_RxHeader.Identifier == 0x053)	//TSA_4
						{
					TA531SysEnv.TA531_env_WindowFL = (buf_rec[0] >> 0) & 0x0F;
					TA531SysEnv.TA531_env_WindowFR = (buf_rec[0] >> 4) & 0x0F;
					TA531SysEnv.TA531_env_WindowRL = (buf_rec[1] >> 0) & 0x0F;
					TA531SysEnv.TA531_env_WindowRR = (buf_rec[1] >> 4) & 0x0F;

					TA531SysEnv.TA531_env_DoorSwF = (buf_rec[4] >> 0) & 0x03;
					TA531SysEnv.TA531_env_DoorSwR = (buf_rec[4] >> 2) & 0x03;
					TA531SysEnv.TA531_env_HSD12_1 = (buf_rec[4] >> 4) & 0x01;
					TA531SysEnv.TA531_env_HSD12_2 = (buf_rec[4] >> 5) & 0x01;
					TA531SysEnv.TA531_env_HSD5_1 = (buf_rec[4] >> 6) & 0x01;
					TA531SysEnv.TA531_env_HSD5_2 = (buf_rec[4] >> 7) & 0x01;

					TA531SysEnv.TA531_env_DoorSwFL = (buf_rec[5] >> 0) & 0x03;
					TA531SysEnv.TA531_env_DoorSwFR = (buf_rec[5] >> 2) & 0x03;
					TA531SysEnv.TA531_env_DoorSwRL = (buf_rec[5] >> 4) & 0x03;
					TA531SysEnv.TA531_env_DoorSwRR = (buf_rec[5] >> 6) & 0x03;
//				    TA531SysEnv.TA531_env_DoorSwReserve = (buf_rec[5]>>6)&0x03;

					TA531_Door.Door_FL =
							(TA531SysEnv.TA531_env_DoorSwFL & 0x03);
					TA531_Door.Door_FR =
							(TA531SysEnv.TA531_env_DoorSwFR & 0x03);
					TA531_Door.Door_RL =
							(TA531SysEnv.TA531_env_DoorSwRL & 0x03);
					TA531_Door.Door_Hood =
							(TA531SysEnv.TA531_env_DoorSwF & 0x03);
					TA531_Door.Door_Trunk = (TA531SysEnv.TA531_env_DoorSwR
							& 0x03);
//				    TA531_Door.Door_Reserve = (TA531SysEnv.TA531_env_DoorSwReserve & 0x03);
					TA531_Door.Door_Reserve = (TA531SysEnv.TA531_env_DoorSwRR
							& 0x03);

					TSA_Ack_DATA[4] = 1;
					TSA4_0x53_Flag = 1;
				} else if (FDCAN1_RxHeader.Identifier == 0x054)	//TSA_PWM
						{
					TA531SysEnv.TA531_env_PWM_Ag_1 = (buf_rec[0] >> 0) & 0xFF;
					TA531SysEnv.TA531_env_PWM_Ag_2 = (buf_rec[1] >> 0) & 0xFF;
					TA531SysEnv.TA531_env_PWM_Ag_3 = (buf_rec[2] >> 0) & 0xFF;
					TA531SysEnv.TA531_env_PWM_Ag_4 = (buf_rec[3] >> 0) & 0xFF;

					if (TA531SysEnv.TA531_env_PWM_Ag_1 == 0) {
						TA531SysEnv.TA531_env_PWM_Ag_1 = ((buf_rec[4] >> 0)
								& 0x03) * 90;
					}
					if (TA531SysEnv.TA531_env_PWM_Ag_2 == 0) {
						TA531SysEnv.TA531_env_PWM_Ag_2 = ((buf_rec[4] >> 2)
								& 0x03) * 90;
					}
					if (TA531SysEnv.TA531_env_PWM_Ag_3 == 0) {
						TA531SysEnv.TA531_env_PWM_Ag_3 = ((buf_rec[4] >> 4)
								& 0x03) * 90;
					}
					if (TA531SysEnv.TA531_env_PWM_Ag_4 == 0) {
						TA531SysEnv.TA531_env_PWM_Ag_4 = ((buf_rec[4] >> 6)
								& 0x03) * 90;
					}

					TA531SysEnv.TA531_env_PWM_Ag_5 = (buf_rec[5] >> 0)
							& 0x03 * 90;
					TA531SysEnv.TA531_env_PWM_Ag_6 = (buf_rec[5] >> 2)
							& 0x03 * 90;
					TA531SysEnv.TA531_env_PWM_Ag_7 = (buf_rec[5] >> 4)
							& 0x03 * 90;
					TA531SysEnv.TA531_env_PWM_Ag_8 = (buf_rec[5] >> 6)
							& 0x03 * 90;

					TSA_Ack_DATA[4] = 1;
					TSA4_0x54_Flag = 1;
				} else if (FDCAN1_RxHeader.Identifier == 0x103)	//TSA_LIN_SWS G2.5
						{
					TA531_LIN_SWS.LIN_SWS2_ErrRespSWS_1 = (buf_rec[0] >> 0)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS2_RespErSWSF_1 = (buf_rec[0] >> 1)
							& 0x01;

					TA531_LIN_SWS.LIN_SWS_CCSwStsDistIncSwA_1 =
							(buf_rec[0] >> 4) & 0x01;
					TA531_LIN_SWS.LIN_SWS_CCSwStsCCASwA_1 = (buf_rec[0] >> 5)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_PfTrTapUpDwnSecySwSta_2 = (buf_rec[0]
							>> 6) & 0x03;

					TA531_LIN_SWS.LIN_SWS_CCSwStsSwDataIntgty_2 = (buf_rec[1]
							>> 0) & 0x03;
					TA531_LIN_SWS.LIN_SWS_CCSwStsSpdDecSwA_1 = (buf_rec[1] >> 2)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_CCSwStsSetSwA_1 = (buf_rec[1] >> 3)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_CCSwStsRsmSwA_1 = (buf_rec[1] >> 4)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_CCSwStsOnSwA_1 = (buf_rec[1] >> 5)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_CCSwStsDistDecSwA_1 =
							(buf_rec[1] >> 6) & 0x01;
					TA531_LIN_SWS.LIN_SWS_CCSwStsCanclSwA_1 = (buf_rec[1] >> 7)
							& 0x01;

					TA531_LIN_SWS.LIN_SWS_SWSFastrUserSwA_1 = (buf_rec[2] >> 0)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_SWSEntrtnUserSwA_1 = (buf_rec[2] >> 1)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_SWSCnfmSwA_1 = (buf_rec[2] >> 2)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_StrgWhlTipcSwDataIntgty_1 =
							(buf_rec[2] >> 3) & 0x01;
					TA531_LIN_SWS.LIN_SWS_StrgWhlEntrtnSwDtIntgty_1 =
							(buf_rec[2] >> 4) & 0x01;
					TA531_LIN_SWS.LIN_SWS_StrgWhlDrMdSwDtIntgty_1 = (buf_rec[2]
							>> 5) & 0x01;
					TA531_LIN_SWS.LIN_SWS_StrgWhlDrvngMdSwA_1 =
							(buf_rec[2] >> 6) & 0x01;
					TA531_LIN_SWS.LIN_SWS_CCSwStsSpdIncSwA_1 = (buf_rec[2] >> 7)
							& 0x01;

					TA531_LIN_SWS.LIN_SWS_SWSFnChngSwA_1 = (buf_rec[3] >> 0)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_SWSSelDwnSwA_1 = (buf_rec[3] >> 1)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_SWSSelLSwA_1 = (buf_rec[3] >> 2)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_SWSSelRSwA_1 = (buf_rec[3] >> 3)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_SWSSelUpSwA_1 = (buf_rec[3] >> 4)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_SWSSocContSwA_1 = (buf_rec[3] >> 5)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_SWSVcSwA_1 = (buf_rec[3] >> 6) & 0x01;
					TA531_LIN_SWS.LIN_SWS_050ms_PDU00_Reserve01_1 = (buf_rec[3]
							>> 7) & 0x01;

					TA531_LIN_SWS.LIN_SWS_050ms_PDU00_Reserve02_8 = (buf_rec[4]
							>> 0) & 0xff;

					TA531_LIN_SWS.LIN_SWS_SWSLFnChngSwA_1 = (buf_rec[5] >> 0)
							& 0x01;
					TA531_LIN_SWS.LIN_SWS_050ms_PDU00_Reserve03_7 = (buf_rec[5]
							>> 1) & 0x7f;

					TSA_Ack_DATA[5] = 1;
					TSA4_0x103_Flag = 1;
					lvLED_Sts_LIN = 2;
				} else if (FDCAN1_RxHeader.Identifier == 0x104)	//TSA_LIN_SWS G3.0
						{
					// 解析CAN数据

					TA531_LIN_SWS_G3.CCSwStsAlvRC_l = (buf_rec[1] >> 0) & 0x0f;
					TA531_LIN_SWS_G3.SWSSelUpSwAL_l = (buf_rec[1] >> 4) & 0x03;
					TA531_LIN_SWS_G3.SWSSelDwnSwAL_l = (buf_rec[1] >> 6) & 0x03;

					DEBUG_CAN_Up = TA531_LIN_SWS_G3.SWSSelUpSwAL_l;
					DEBUG_CAN_Down = TA531_LIN_SWS_G3.SWSSelDwnSwAL_l;
					DEBUG_CAN_RX_Flag = 1;  // 标记收到新数据
					DEBUG_CAN_104_Count++;

					TA531_LIN_SWS_G3.SWSSelLSwAL_l = (buf_rec[2] >> 0) & 0x03;
					TA531_LIN_SWS_G3.SWSSelRSwAL_l = (buf_rec[2] >> 2) & 0x03;
					TA531_LIN_SWS_G3.SWSCnfmSwAL_l = (buf_rec[2] >> 4) & 0x03;
					TA531_LIN_SWS_G3.SWSPB1SwAL_l = (buf_rec[2] >> 6) & 0x03;

					TA531_LIN_SWS_G3.SWSPB2SwAL_l = (buf_rec[3] >> 0) & 0x03;
					TA531_LIN_SWS_G3.SWSPB3SwAL_l = (buf_rec[3] >> 2) & 0x03;
					TA531_LIN_SWS_G3.SWSSelUpSwReq_l = (buf_rec[3] >> 4) & 0x03;
					TA531_LIN_SWS_G3.SWSSelDwnSwReq_l = (buf_rec[3] >> 6)
							& 0x03;

					TA531_LIN_SWS_G3.SWSSelLSwReq_l = (buf_rec[4] >> 0) & 0x03;
					TA531_LIN_SWS_G3.SWSSelRSwReq_l = (buf_rec[4] >> 2) & 0x03;
					TA531_LIN_SWS_G3.SWSCnfmSwReq_l = (buf_rec[4] >> 4) & 0x03;
					TA531_LIN_SWS_G3.SWSPB1SwReq_l = (buf_rec[4] >> 6) & 0x03;

					TA531_LIN_SWS_G3.SWSPB2SwReq_l = (buf_rec[5] >> 0) & 0x03;
					TA531_LIN_SWS_G3.SWSPB3SwReq_l = (buf_rec[5] >> 2) & 0x03;
					TA531_LIN_SWS_G3.PfTrTapUpDwnSecySwSta_l = (buf_rec[5] >> 4)
							& 0x03;
					TA531_LIN_SWS_G3.PadSSelLSwA_l = (buf_rec[5] >> 6) & 0x03;

					TA531_LIN_SWS_G3.PadSSelRSwA_l = (buf_rec[6] >> 0) & 0x03;
					TA531_LIN_SWS_G3.SWSSelLSwStuckL_l = (buf_rec[6] >> 2)
							& 0x01;
					TA531_LIN_SWS_G3.SWSSelRSwStuckL_l = (buf_rec[6] >> 3)
							& 0x01;
					TA531_LIN_SWS_G3.SWSCnfmSwStuckL_l = (buf_rec[6] >> 4)
							& 0x01;
					TA531_LIN_SWS_G3.SWSPB1SwStuckL_l = (buf_rec[6] >> 5)
							& 0x01;
					TA531_LIN_SWS_G3.SWSPB2SwStuckL_l = (buf_rec[6] >> 6)
							& 0x01;
					TA531_LIN_SWS_G3.SWSPB3SwStuckL_l = (buf_rec[6] >> 7)
							& 0x01;

					TA531_LIN_SWS_G3.StrgWhlEntrtnSwDataIntgty_l = (buf_rec[7]
							>> 0) & 0x01;  // 新增
					TA531_LIN_SWS_G3.StrgWhlDrvngMdSwA_l = (buf_rec[7] >> 1)
							& 0x01;
					TA531_LIN_SWS_G3.StrgWhlDrvngMdSwDataIntgty_l = (buf_rec[7]
							>> 2) & 0x01;
					TA531_LIN_SWS_G3.StrgWhlTipcSwDataIntgty_l = (buf_rec[7]
							>> 3) & 0x01;
					TA531_LIN_SWS_G3.PadSSelLSwStuck_l = (buf_rec[7] >> 4)
							& 0x01;
					TA531_LIN_SWS_G3.PadSSelRSwStuck_l = (buf_rec[7] >> 5)
							& 0x01;
					TA531_LIN_SWS_G3.RespErSWSF_l = (buf_rec[7] >> 6) & 0x01;

					TSA_Ack_DATA[5] = 1;
					TSA4_0x104_Flag = 1;
					lvLED_Sts_LIN = 2;
				} else if (FDCAN1_RxHeader.Identifier == 0x105)	//EBS LIN FrP00 bridge
						{
					EBSBatVol_raw = (uint16_t) (((uint16_t) buf_rec[1] << 8) | buf_rec[0]) & 0x3FFF;
					EBSVolSts_raw = buf_rec[2] & 0x03;
					EBSBatCrntRng_raw = buf_rec[3] & 0x03;
					EBSBatIncnstncyFlag_raw = buf_rec[4] & 0x01;
					EBSRespEr_raw = buf_rec[5] & 0x01;
					TSA_Ack_DATA[6] = 1;
					lvLED_Sts_LIN = 2;
				} else if (FDCAN1_RxHeader.Identifier == 0x064)	//TSA_RC1 mm
						{
					TA531_RC1.TA531_RC_X_trg = (int) ((buf_rec[1] << 8)
							+ (buf_rec[0] << 0));
					if ((buf_rec[2] & 0x80) == 0x80) {
						TA531_RC1.TA531_RC_X_Mov = 0 - buf_rec[2];
					} else {
						TA531_RC1.TA531_RC_X_Mov = buf_rec[2];
					}

					TA531_RC1.TA531_RC_Y_trg = (int) ((buf_rec[4] << 8)
							+ (buf_rec[3] << 0));
					if ((buf_rec[5] & 0x80) == 0x80) {
						TA531_RC1.TA531_RC_Y_Mov = 0 - buf_rec[5];
					} else {
						TA531_RC1.TA531_RC_Y_Mov = buf_rec[5];
					}

					TA531_RC1.TA531_RC_Z = (int) (buf_rec[6] << 0);
					TA531_RC1.TA531_RC_Z_code = buf_rec[7] & 0x03;
					TA531_RC1.TA531_RC_Reset = (buf_rec[7] >> 6) & 0x03;

					Clamp_Position(&TA531_RC1.TA531_RC_X_trg,
							&TA531_RC1.TA531_RC_Y_trg,
							(TA531_RC1.TA531_RC_Reset == 1));

					TA531_RC1_fg = 2;
				} else if (FDCAN1_RxHeader.Identifier == 0x065)	//TSA_RC1p %
						{
					TA531_RC1.TA531_RC_X_trg = ScreenSz_1.DispX0_32b
							+ (int) (buf_rec[0]
									* (ScreenSz_1.DispX1_32b
											- ScreenSz_1.DispX0_32b) / 255);

					if ((buf_rec[2] & 0x80) == 0x80) {
						TA531_RC1.TA531_RC_X_Mov = 0 - buf_rec[2];
					} else {
						TA531_RC1.TA531_RC_X_Mov = buf_rec[2];
					}

					TA531_RC1.TA531_RC_Y_trg = ScreenSz_1.DispY0_32b
							+ (int) (buf_rec[3]
									* (ScreenSz_1.DispY1_32b
											- ScreenSz_1.DispY0_32b) / 255);

					if ((buf_rec[5] & 0x80) == 0x80) {
						TA531_RC1.TA531_RC_Y_Mov = 0 - buf_rec[5];
					} else {
						TA531_RC1.TA531_RC_Y_Mov = buf_rec[5];
					}

					TA531_RC1.TA531_RC_Z = (int) (buf_rec[6] << 0);
					TA531_RC1.TA531_RC_Z_code = buf_rec[7] & 0x03;
					TA531_RC1.TA531_RC_Reset = (buf_rec[7] >> 6) & 0x03;

					Clamp_Position(&TA531_RC1.TA531_RC_X_trg,
							&TA531_RC1.TA531_RC_Y_trg,
							(TA531_RC1.TA531_RC_Reset == 1));

					TA531_RC1_fg = 2;
				} else if (FDCAN1_RxHeader.Identifier == 0x531)	//TSA_ACK for RESET
						{
					if ((buf_rec[0] == 0x05) && (buf_rec[1] == 0x31)
							&& (buf_rec[2] == 0x00) && (buf_rec[3] == 0x00)
							&& (buf_rec[4] == 0x00) && (buf_rec[5] == 0x00)
							&& (buf_rec[6] == 0x00) && (buf_rec[7] == 0x00)) {
						Remote_state = 0;
						TA531_Lock = 0;
						lvLED_Sts_LIN = 0;
					}
				}

				// 每帧都发送ACK并清空
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA_Ack_Header,
						TSA_Ack_DATA);
				memset(TSA_Ack_DATA, 0, 8);

			}  // while结束
		}  // if hfdcan == &hfdcan1 结束
	}  // if RxFifo0ITs 结束
}  // 函数结束

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
		if (hfdcan == &hfdcan2) 	//CAN2 Classic CAN
				{
			uint8_t buf_rec[8];

			while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO1)) {
				HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1,
						&FDCAN2_RxHeader, buf_rec);
			}

			if (FDCAN2_RxHeader.Identifier == HostID)	//
					{

				uint16_t TeMP = (buf_rec[0] << 3) + ((buf_rec[1] >> 5) & 0x07);
				if (TeMP == M1_ID)	//X
						{

					if ((buf_rec[2] == 0x41) & (buf_rec[3] == 0)
							& (buf_rec[4] == 0) & (buf_rec[5] == 0)
							& (buf_rec[6] == 0) & (buf_rec[7] == 0))//init ok
							{
						MotorInit_M1 = 2;
						MotorCtrl_M1.M_Position = 0;
					} else {
						if ((MotorInit_M1 == 2) & (buf_rec[2] == 0x42))	//X_Position
								{
							MotorCtrl_M1.M_Position = (buf_rec[3]
									+ (buf_rec[4] << 8) + (buf_rec[5] << 16)
									+ (buf_rec[6] << 24)) / 160;

							TA531_RC1.TA531_RC_X_act = -MotorCtrl_M1.M_Position
									- 10;
						}

						if ((buf_rec[7] >> 4) > 0)	//
								{
							MotorERR_M1 = 1;
						}
					}
				} else if (TeMP == M2_ID)	//x
						{
					if ((buf_rec[2] == 0x41) & (buf_rec[3] == 0)
							& (buf_rec[4] == 0) & (buf_rec[5] == 0)
							& (buf_rec[6] == 0) & (buf_rec[7] == 0))//init ok
							{
						MotorInit_M2 = 2;
						MotorCtrl_M2.M_Position = 0;
					} else {
						if ((MotorInit_M2 == 2) & (buf_rec[2] == 0x42))	//X_Position
								{
							MotorCtrl_M2.M_Position = (buf_rec[3]
									+ (buf_rec[4] << 8) + (buf_rec[5] << 16)
									+ (buf_rec[6] << 24)) / 160;
						}

						if ((buf_rec[7] >> 4) > 0)	//
								{
							MotorERR_M2 = 1;
						}
					}
				}

				else if (TeMP == M3_ID)	//y
						{
					if ((buf_rec[2] == 0x41) & (buf_rec[3] == 0)
							& (buf_rec[4] == 0) & (buf_rec[5] == 0)
							& (buf_rec[6] == 0) & (buf_rec[7] == 0))//init ok
							{
//						OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 0, "trigger" );
						MotorInit_M3 = 2;
						MotorCtrl_M3.M_Position = 0;
					} else {
						if ((MotorInit_M3 == 2) & (buf_rec[2] == 0x42))	//X_Position
								{
							MotorCtrl_M3.M_Position = (buf_rec[3]
									+ (buf_rec[4] << 8) + (buf_rec[5] << 16)
									+ (buf_rec[6] << 24)) / 160;

							TA531_RC1.TA531_RC_Y_act = -MotorCtrl_M3.M_Position
									- 10;
						}

						if ((buf_rec[7] >> 4) > 0)	//
								{
							MotorERR_M3 = 1;
						}
					}
				}
			}
		}
	}
}

//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//	char *str2;
//	sprintf(str2, "%c", I2CRxData[0]);
//	OLED_SSD1306_ShowString(3 ,0, 1, str2);
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6)	//500ms
			{
		temper_flag = 1;

	}
	if (htim == &htim7)	//100ms
			{

		////GPIO	// SW_UP, SW_UP_pre, SW_DW, SW_DW_pre, SW_LEFT, SW_LEFT_pre, SW_RIGHT, SW_RIGHT_pre, SW_BUTTON, SW_BUTTON_pre;
		SW_UP_pre = SW_UP;
		SW_DW_pre = SW_DW;
		SW_LEFT_pre = SW_LEFT;
		SW_RIGHT_pre = SW_RIGHT;
		SW_BUTTON_pre = SW_BUTTON;

			if (HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == 0) {
				SW_UP = 1;
				if (SW_UP_pre == 1) {
					SW_UP_cnt++;
				} else {
					SW_UP_cnt = 0;
				}
			} else {
				SW_UP = 0;
				SW_UP_cnt = 0;
			}
			if (HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == 0) {
				SW_DW = 1;
				if (SW_DW_pre == 1) {
					SW_DW_cnt++;
				} else {
					SW_DW_cnt = 0;
				}
			} else {
				SW_DW = 0;
				SW_DW_cnt = 0;
			}
			if (HAL_GPIO_ReadPin(SW_LEFT_GPIO_Port, SW_LEFT_Pin) == 0) {
				SW_LEFT = 1;
				if (SW_LEFT_pre == 1) {
					SW_LEFT_cnt++;
				} else {
					SW_LEFT_cnt = 0;
				}
			} else {
				SW_LEFT = 0;
				SW_LEFT_cnt = 0;
			}
			if (HAL_GPIO_ReadPin(SW_RIGHT_GPIO_Port, SW_RIGHT_Pin) == 0) {
				SW_RIGHT = 1;
				if (SW_RIGHT_pre == 1) {
					SW_RIGHT_cnt++;
				} else {
					SW_RIGHT_cnt = 0;
				}
			} else {
				SW_RIGHT = 0;
				SW_RIGHT_cnt = 0;
			}
			if (HAL_GPIO_ReadPin(SW_BUTTON_GPIO_Port, SW_BUTTON_Pin) == 0) {
				SW_BUTTON = 1;
			} else {
				SW_BUTTON = 0;
			}

		////Sys state

		sys_state++;

		//int sys_state; //0 - 6; 	0 - none; 1- green	;2 - cyan(青色，天蓝）;3 - blue	;4 - yellow	;5 - purple	;6 - red
		sys_state = sys_state % 7;
		switch (sys_state) {
		case 0:		//none
			HAL_GPIO_WritePin(IO_SYS_LED_B_GPIO_Port, IO_SYS_LED_B_Pin, 0);
			HAL_GPIO_WritePin(IO_SYS_LED_G_GPIO_Port, IO_SYS_LED_G_Pin, 0);
			HAL_GPIO_WritePin(IO_SYS_LED_R_GPIO_Port, IO_SYS_LED_R_Pin, 0);
			break;
		case 1:		//green
			HAL_GPIO_WritePin(IO_SYS_LED_B_GPIO_Port, IO_SYS_LED_B_Pin, 0);
			HAL_GPIO_WritePin(IO_SYS_LED_G_GPIO_Port, IO_SYS_LED_G_Pin, 1);
			HAL_GPIO_WritePin(IO_SYS_LED_R_GPIO_Port, IO_SYS_LED_R_Pin, 0);
			break;
		case 2:		//cyan
			HAL_GPIO_WritePin(IO_SYS_LED_B_GPIO_Port, IO_SYS_LED_B_Pin, 1);
			HAL_GPIO_WritePin(IO_SYS_LED_G_GPIO_Port, IO_SYS_LED_G_Pin, 1);
			HAL_GPIO_WritePin(IO_SYS_LED_R_GPIO_Port, IO_SYS_LED_R_Pin, 0);
			break;
		case 3:		//blue
			HAL_GPIO_WritePin(IO_SYS_LED_B_GPIO_Port, IO_SYS_LED_B_Pin, 1);
			HAL_GPIO_WritePin(IO_SYS_LED_G_GPIO_Port, IO_SYS_LED_G_Pin, 0);
			HAL_GPIO_WritePin(IO_SYS_LED_R_GPIO_Port, IO_SYS_LED_R_Pin, 0);
			break;
		case 4:		//yellow
			HAL_GPIO_WritePin(IO_SYS_LED_B_GPIO_Port, IO_SYS_LED_B_Pin, 0);
			HAL_GPIO_WritePin(IO_SYS_LED_G_GPIO_Port, IO_SYS_LED_G_Pin, 1);
			HAL_GPIO_WritePin(IO_SYS_LED_R_GPIO_Port, IO_SYS_LED_R_Pin, 1);
			break;
		case 5:		//purple
			HAL_GPIO_WritePin(IO_SYS_LED_B_GPIO_Port, IO_SYS_LED_B_Pin, 1);
			HAL_GPIO_WritePin(IO_SYS_LED_G_GPIO_Port, IO_SYS_LED_G_Pin, 0);
			HAL_GPIO_WritePin(IO_SYS_LED_R_GPIO_Port, IO_SYS_LED_R_Pin, 1);
			break;
		case 6:		//red
			HAL_GPIO_WritePin(IO_SYS_LED_B_GPIO_Port, IO_SYS_LED_B_Pin, 0);
			HAL_GPIO_WritePin(IO_SYS_LED_G_GPIO_Port, IO_SYS_LED_G_Pin, 0);
			HAL_GPIO_WritePin(IO_SYS_LED_R_GPIO_Port, IO_SYS_LED_R_Pin, 1);
			break;
		}

	}
	if (htim == &htim14)	//1000ms
			{

		//TA531 Ack
		TSA_Ack_DATA[0] = 0x64;	//100
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA_Ack_Header, TSA_Ack_DATA);
		TSA_Ack_DATA[0] = 0;

		Sys_CNT++;

		uint32_t value_ADC1_0;
		value_ADC1_0 = mRead_ADC1_ch(1);
		if (value_ADC1_0 < 4500) {
			Bench_Info.Bench_PowerSts = 0;
		} else if (value_ADC1_0 < 9000) {
			Bench_Info.Bench_PowerSts = 1;
			Sys_RunCNT++;
		} else if (value_ADC1_0 < 13500) {
			Bench_Info.Bench_PowerSts = 2;
			Sys_RunCNT++;
		} else {
			Bench_Info.Bench_PowerSts = 3;
			Sys_RunCNT++;
		}

///////// XL9555
		uint8_t XL9555_tempdata;
		XL9555_tempdata = XL9555_Read(XL9555_1_addr_read, 0);
		TA531_GP_IN.GP_IN_1_1 = !((XL9555_tempdata >> 0) & 0x01);
		TA531_GP_IN.GP_IN_1_2 = !((XL9555_tempdata >> 1) & 0x01);
		TA531_GP_IN.GP_IN_1_3 = !((XL9555_tempdata >> 2) & 0x01);
		TA531_GP_IN.GP_IN_1_4 = !((XL9555_tempdata >> 3) & 0x01);
		TA531_GP_IN.GP_IN_1_5 = !((XL9555_tempdata >> 4) & 0x01);
		TA531_GP_IN.GP_IN_1_6 = !((XL9555_tempdata >> 5) & 0x01);
		TA531_GP_IN.GP_IN_1_7 = !((XL9555_tempdata >> 6) & 0x01);
		TA531_GP_IN.GP_IN_1_8 = !((XL9555_tempdata >> 7) & 0x01);

		XL9555_tempdata = XL9555_Read(XL9555_1_addr_read, 1);
		TA531_GP_IN.GP_IN_2_8 = !((XL9555_tempdata >> 0) & 0x01);
		TA531_GP_IN.GP_IN_2_7 = !((XL9555_tempdata >> 1) & 0x01);
		TA531_GP_IN.GP_IN_2_6 = !((XL9555_tempdata >> 2) & 0x01);
		TA531_GP_IN.GP_IN_2_5 = !((XL9555_tempdata >> 3) & 0x01);
		TA531_GP_IN.GP_IN_2_4 = !((XL9555_tempdata >> 4) & 0x01);
		TA531_GP_IN.GP_IN_2_3 = !((XL9555_tempdata >> 5) & 0x01);
		TA531_GP_IN.GP_IN_2_2 = !((XL9555_tempdata >> 6) & 0x01);
		TA531_GP_IN.GP_IN_2_1 = !((XL9555_tempdata >> 7) & 0x01);

		XL9555_tempdata = XL9555_Read(XL9555_2_addr_read, 0);
		TA531_GP_IN.GP_IN_3_1 = !((XL9555_tempdata >> 0) & 0x01);
		TA531_GP_IN.GP_IN_3_2 = !((XL9555_tempdata >> 1) & 0x01);
		TA531_GP_IN.GP_IN_3_3 = !((XL9555_tempdata >> 2) & 0x01);
		TA531_GP_IN.GP_IN_3_4 = !((XL9555_tempdata >> 3) & 0x01);
		TA531_GP_IN.GP_IN_3_5 = !((XL9555_tempdata >> 4) & 0x01);
		TA531_GP_IN.GP_IN_3_6 = !((XL9555_tempdata >> 5) & 0x01);
		TA531_GP_IN.GP_IN_3_7 = !((XL9555_tempdata >> 6) & 0x01);
		TA531_GP_IN.GP_IN_3_8 = !((XL9555_tempdata >> 7) & 0x01);

		XL9555_tempdata = XL9555_Read(XL9555_2_addr_read, 1);
		TA531_GP_IN.GP_IN_4_8 = !((XL9555_tempdata >> 0) & 0x01);
		TA531_GP_IN.GP_IN_4_7 = !((XL9555_tempdata >> 1) & 0x01);
		TA531_GP_IN.GP_IN_4_6 = !((XL9555_tempdata >> 2) & 0x01);
		TA531_GP_IN.GP_IN_4_5 = !((XL9555_tempdata >> 3) & 0x01);
		TA531_GP_IN.GP_IN_4_4 = !((XL9555_tempdata >> 4) & 0x01);
		TA531_GP_IN.GP_IN_4_3 = !((XL9555_tempdata >> 5) & 0x01);
		TA531_GP_IN.GP_IN_4_2 = !((XL9555_tempdata >> 6) & 0x01);
		TA531_GP_IN.GP_IN_4_1 = !((XL9555_tempdata >> 7) & 0x01);

		TSA_GP_IN_DATA[0] = ((TA531_GP_IN.GP_IN_1_1 & 0x03) << 0)
				+ ((TA531_GP_IN.GP_IN_1_2 & 0x03) << 2)
				+ ((TA531_GP_IN.GP_IN_1_3 & 0x03) << 4)
				+ ((TA531_GP_IN.GP_IN_1_4 & 0x03) << 6);
		TSA_GP_IN_DATA[1] = ((TA531_GP_IN.GP_IN_1_5 & 0x03) << 0)
				+ ((TA531_GP_IN.GP_IN_1_6 & 0x03) << 2)
				+ ((TA531_GP_IN.GP_IN_1_7 & 0x03) << 4)
				+ ((TA531_GP_IN.GP_IN_1_8 & 0x03) << 6);
		TSA_GP_IN_DATA[2] = ((TA531_GP_IN.GP_IN_2_1 & 0x03) << 0)
				+ ((TA531_GP_IN.GP_IN_2_2 & 0x03) << 2)
				+ ((TA531_GP_IN.GP_IN_2_3 & 0x03) << 4)
				+ ((TA531_GP_IN.GP_IN_2_4 & 0x03) << 6);
		TSA_GP_IN_DATA[3] = ((TA531_GP_IN.GP_IN_2_5 & 0x03) << 0)
				+ ((TA531_GP_IN.GP_IN_2_6 & 0x03) << 2)
				+ ((TA531_GP_IN.GP_IN_2_7 & 0x03) << 4)
				+ ((TA531_GP_IN.GP_IN_2_8 & 0x03) << 6);
		TSA_GP_IN_DATA[4] = ((TA531_GP_IN.GP_IN_3_1 & 0x03) << 0)
				+ ((TA531_GP_IN.GP_IN_3_2 & 0x03) << 2)
				+ ((TA531_GP_IN.GP_IN_3_3 & 0x03) << 4)
				+ ((TA531_GP_IN.GP_IN_3_4 & 0x03) << 6);
		TSA_GP_IN_DATA[5] = ((TA531_GP_IN.GP_IN_3_5 & 0x03) << 0)
				+ ((TA531_GP_IN.GP_IN_3_6 & 0x03) << 2)
				+ ((TA531_GP_IN.GP_IN_3_7 & 0x03) << 4)
				+ ((TA531_GP_IN.GP_IN_3_8 & 0x03) << 6);
		TSA_GP_IN_DATA[6] = ((TA531_GP_IN.GP_IN_4_1 & 0x03) << 0)
				+ ((TA531_GP_IN.GP_IN_4_2 & 0x03) << 2)
				+ ((TA531_GP_IN.GP_IN_4_3 & 0x03) << 4)
				+ ((TA531_GP_IN.GP_IN_4_4 & 0x03) << 6);
		TSA_GP_IN_DATA[7] = ((TA531_GP_IN.GP_IN_4_5 & 0x03) << 0)
				+ ((TA531_GP_IN.GP_IN_4_6 & 0x03) << 2)
				+ ((TA531_GP_IN.GP_IN_4_7 & 0x03) << 4)
				+ ((TA531_GP_IN.GP_IN_4_8 & 0x03) << 6);

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA_GP_IN_Header,
				TSA_GP_IN_DATA);

	}

	if (htim == &htim16)	//50ms
			{
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA1_ADC_Header,
				TSA1_ADC_DATA);
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA2_LS_Header, TSA2_LS_DATA);

		////for touchpen short/long time count
		Sys_TIM_TICK = Sys_TIM_TICK + 5;

		if (Sys_TIM_TICK > 0xFFFFFFF0) {
			Sys_TIM_TICK = 0;
		}
	}
	if (htim == &htim17)	//1ms
			{
		lv_tick_inc(1);
	}

}

uint8_t Lin_CheckPID(uint8_t id) {
	uint8_t returnpid;
	uint8_t P0;
	uint8_t P1;

	P0 = (((id) ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 0x01) << 6;
	P1 = ((~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5))) & 0x01) << 7;

	returnpid = id | P0 | P1;

	return returnpid;
}

uint8_t Lin_Checksum(uint8_t id, uint8_t *data) {
	uint8_t t;
	uint16_t sum = 0;
	bool is_diag_frame = (id == 0x3C) || (id == 0x3D);

	for (t = 0; t < 8; t++) {
		sum += data[t];
		if (sum & 0xff00) {
			sum = (sum & 0x00ff) + 1;
		}
	}

	if (!is_diag_frame) {
		sum += Lin_CheckPID(id);
		if (sum & 0xff00) {
			sum = (sum & 0x00ff) + 1;
		}
	}

	sum = ~sum;
	data[8] = (uint8_t) sum;
	return (uint8_t) sum;
}

uint8_t Calc_SWS_G3_CRC8(const uint8_t *data, uint8_t len) {
	uint8_t crc = 0x00;

	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x80) {
				crc = (uint8_t) ((crc << 1) ^ 0x1D);
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

void Build_EBS_0x34_Data(void) {
	uint16_t v = EBSBatVol_raw & 0x3FFF;
	// EBSBatCrnt raw=0x8000 => 0x8000*0.0078125-256 = 0A (factor=0.0078125, offset=-256)
	uint16_t crnt = 0x8000;
	// EBSBatTem raw=65 => 65*1-40 = 25°C (factor=1, offset=-40)
	uint8_t tem = 65;

	memset(EBS_0x0_Data, 0, 9);

	// bit0..13: EBSBatVol_l (14bit)
	EBS_0x0_Data[0] = (uint8_t) (v & 0xFF);
	// bit8..13: EBSBatVol_l high, bit14..15: EBSBatCrntRng_l
	EBS_0x0_Data[1] = (uint8_t) (((v >> 8) & 0x3F) | ((EBSBatCrntRng_raw & 0x03) << 6));
	// bit16..31: EBSBatCrnt_l (16bit, little-endian)
	EBS_0x0_Data[2] = (uint8_t) (crnt & 0xFF);
	EBS_0x0_Data[3] = (uint8_t) ((crnt >> 8) & 0xFF);
	// bit32..39: EBSBatTem_l
	EBS_0x0_Data[4] = tem;
	// bit40..47: EBSQucntCrnt_l = 0
	EBS_0x0_Data[5] = 0;
	// bit48..49: EBSCrntSts_l=0 (No_error)
	// bit50..51: EBSVolSts_l
	// bit52..53: EBSTemSts_l=0 (No_error)
	// bit54:     EBSErInCalData_l=0
	// bit55:     EBSErInECUIdficn_l=0
	EBS_0x0_Data[6] = (uint8_t) ((EBSVolSts_raw & 0x03) << 2);
	// bit56:     EBSRespEr_l
	// bit57:     EBSBatIncnstncyFlag_l
	EBS_0x0_Data[7] = (uint8_t) ((EBSRespEr_raw & 0x01)
			| ((EBSBatIncnstncyFlag_raw & 0x01) << 1));
}

// EBS_ICCLIN1_FrP01_ICC_LIN1 (LIN ID 0x35): SOC, SOH, SOF signals
// Sends placeholder values to prevent LIN master timeout
void Build_EBS_0x35_Data(void) {
	memset(EBS_0x1_Data, 0, 9);
	// byte0: EBSSOC_l=0 (0%)
	// byte1: EBSSOHOfSlphtn_l=0
	// byte2: EBSBatSOFVol1_l=0
	// byte3: EBSBatSOFVol2_l=0
	// byte4: EBSSOHOfLAM_l=0
	// byte5: EBSSOHOfCrsn_l=0
	// byte6 bit2-3: EBSSOCSts_l=0 (deviation>15%, valid encoding)
	// byte6 bit4-5: EBSSOHOfSlphtnSts_l=0
	// byte6 bit6-7: EBSBatSOFVol1Sts_l=0
	// byte7 bit0-1: EBSBatSOFVol2Sts_l=0
	// byte7 bit2-3: EBSSOHOfLAMSts_l=0
	// byte7 bit4-5: EBSSOHOfCrsnSts_l=0
}

// EBS_ICCLIN1_FrP02_ICC_LIN1 (LIN ID 0x36): accumulated charge/discharge
// Sends placeholder values to prevent LIN master timeout
void Build_EBS_0x36_Data(void) {
	memset(EBS_0x2_Data, 0, 9);
	// byte0-1: EBSAcumtdBatChrg_l=0
	// byte2-3: EBSAcumtdBatDschrg_l=0
}

void Lin_SendData(uint8_t *data) {
	Lin_Checksum(ReceiveID, data);

	HAL_UART_Transmit_IT(&huart3, data, 9);

	lvLED_Sts_LIN = 1;
}

void Lin_DataProcess_loop(void)	//asap, if need to deal with LIN data; if not ,seems no use
{
	//// ========== 直接打包 SWS_0x22_Data，无条件执行（像main1_1.c一样）==========
	SWS_0x22_Data[1] = ((TA531_LIN_SWS_G3.CCSwStsAlvRC_l & 0x0f) << 0)
			+ ((TA531_LIN_SWS_G3.SWSSelUpSwAL_l & 0x03) << 4)
			+ ((TA531_LIN_SWS_G3.SWSSelDwnSwAL_l & 0x03) << 6);
	DEBUG_LIN_Byte1 = SWS_0x22_Data[1];
	DEBUG_LIN_Pack_Flag = 1;

	SWS_0x22_Data[2] = ((TA531_LIN_SWS_G3.SWSSelLSwAL_l & 0x03) << 0)
			+ ((TA531_LIN_SWS_G3.SWSSelRSwAL_l & 0x03) << 2)
			+ ((TA531_LIN_SWS_G3.SWSCnfmSwAL_l & 0x03) << 4)
			+ ((TA531_LIN_SWS_G3.SWSPB1SwAL_l & 0x03) << 6);

	SWS_0x22_Data[3] = ((TA531_LIN_SWS_G3.SWSPB2SwAL_l & 0x03) << 0)
			+ ((TA531_LIN_SWS_G3.SWSPB3SwAL_l & 0x03) << 2)
			+ ((TA531_LIN_SWS_G3.SWSSelUpSwReq_l & 0x03) << 4)
			+ ((TA531_LIN_SWS_G3.SWSSelDwnSwReq_l & 0x03) << 6);

	SWS_0x22_Data[4] = ((TA531_LIN_SWS_G3.SWSSelLSwReq_l & 0x03) << 0)
			+ ((TA531_LIN_SWS_G3.SWSSelRSwReq_l & 0x03) << 2)
			+ ((TA531_LIN_SWS_G3.SWSCnfmSwReq_l & 0x03) << 4)
			+ ((TA531_LIN_SWS_G3.SWSPB1SwReq_l & 0x03) << 6);

	SWS_0x22_Data[5] = ((TA531_LIN_SWS_G3.SWSPB2SwReq_l & 0x03) << 0)
			+ ((TA531_LIN_SWS_G3.SWSPB3SwReq_l & 0x03) << 2)
			+ ((TA531_LIN_SWS_G3.PfTrTapUpDwnSecySwSta_l & 0x03) << 4)
			+ ((TA531_LIN_SWS_G3.PadSSelLSwA_l & 0x03) << 6);

	SWS_0x22_Data[6] = ((TA531_LIN_SWS_G3.PadSSelRSwA_l & 0x03) << 0)
			+ ((TA531_LIN_SWS_G3.SWSSelLSwStuckL_l & 0x01) << 2)
			+ ((TA531_LIN_SWS_G3.SWSSelRSwStuckL_l & 0x01) << 3)
			+ ((TA531_LIN_SWS_G3.SWSCnfmSwStuckL_l & 0x01) << 4)
			+ ((TA531_LIN_SWS_G3.SWSPB1SwStuckL_l & 0x01) << 5)
			+ ((TA531_LIN_SWS_G3.SWSPB2SwStuckL_l & 0x01) << 6)
			+ ((TA531_LIN_SWS_G3.SWSPB3SwStuckL_l & 0x01) << 7);

	SWS_0x22_Data[7] = ((TA531_LIN_SWS_G3.StrgWhlEntrtnSwDataIntgty_l & 0x01)
			<< 0) + ((TA531_LIN_SWS_G3.StrgWhlDrvngMdSwA_l & 0x01) << 1)
			+ ((TA531_LIN_SWS_G3.StrgWhlDrvngMdSwDataIntgty_l & 0x01) << 2)
			+ ((TA531_LIN_SWS_G3.StrgWhlTipcSwDataIntgty_l & 0x01) << 3)
			+ ((TA531_LIN_SWS_G3.PadSSelLSwStuck_l & 0x01) << 4)
			+ ((TA531_LIN_SWS_G3.PadSSelRSwStuck_l & 0x01) << 5)
			+ ((TA531_LIN_SWS_G3.RespErSWSF_l & 0x01) << 6);

	SWS_0x22_Data[0] = Calc_SWS_G3_CRC8(&SWS_0x22_Data[1], 7);

	Build_EBS_0x34_Data();

	//// ========== 处理接收到的LIN数据 ==========
	uint8_t PIDChecksum;
	uint8_t SumCheck;

	if (DataReceiveflag == 1) {
		ReceiveID = ReceivePID & 0x3f;
		PIDChecksum = Lin_CheckPID(ReceiveID);
		if (PIDChecksum != ReceivePID) {
			return;
		}

		if (FrameReceiveOverFlag == 1) {
			SumCheck = Lin_Checksum(ReceiveID, LinReceiveData);
			if (ReceiveCheckSum != SumCheck) {
				return;
			}

			if (ReceiveID == 0x23) {
				if (LinReceiveData[3] == 0x01) {
				} else if (LinReceiveData[3] == 0x02) {
				}
			}
		}
		FrameReceiveOverFlag = 0;
		DataReceiveflag = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart != &huart3) {
		Uart_RearmByHandle(huart);
		return;
	}
	Lin_ReadRxDataFromUart3();
	Lin_UpdateDebugOnRx();

	// 常见 RID 在独立函数内处理（包含计数、打包、发送、重启接收）
	if (Lin_HandleKnownRid(ReceiveID)) {
		return;
	}

	// 未识别 RID：进入后续通用流程
	Lin_HandleUnknownRid();
}

void UART_Init(UART_HandleTypeDef *handle, uint32_t data_length) {

	if (handle == &huart1) {
		HAL_UART_Receive_IT(&huart1, u1RxData, Serial_Data_LENGTH);	//
		return;
	}
	Uart_RearmByHandle(handle);

}

void UART_RESET(UART_HandleTypeDef *handle) {

	HAL_UART_AbortReceive(handle);
	HAL_UART_AbortReceive_IT(handle);
	/* Clear RXNE interrupt flag *//* Discard the received data */
	__HAL_UART_SEND_REQ(handle, UART_RXDATA_FLUSH_REQUEST);

}
//void CAN2Ser_Config(void)
//{
//
//}

uint32_t mRead_ADC1_ch(uint8_t ch) {
	//start ADC
	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, 500);

	if (ch == 1) {

	} else if (ch == 2)	//```6
			{

	}

//    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))//读取ADC完成标志�?????????????????????????????????????????????????????????
//    {
//        adc_value = HAL_ADC_GetValue(&hadc1);//读出ADC数�??
//
//        // 将ADC值转换为电压
//        mVoltage = adc_value  *1165 /165 * 3250/ 4095; // 12位ADC，最�?????????????????????????????????????????????????????????4095
//    }

	// 停止ADC转换
	HAL_ADC_Stop(&hadc1);

	return mVoltage;
}

bool WaitMotorToTargetWithProtection(uint32_t timeout_ms, uint16_t poll_ms,
		bool trigger_emergency) {
	uint32_t start_tick = HAL_GetTick();

	while ((abs(TA531_RC1.TA531_RC_X_act - TA531_RC1.TA531_RC_X_trg)
			> REACH_POSITION_TOLERANCE)
			|| (abs(TA531_RC1.TA531_RC_Y_act - TA531_RC1.TA531_RC_Y_trg)
					> REACH_POSITION_TOLERANCE)) {
		MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg, TA531_RC1.TA531_RC_Y_trg);

		if (Motor_Protection_Check(TA531_RC1.TA531_RC_X_act,
				TA531_RC1.TA531_RC_Y_act, TA531_RC1.TA531_RC_X_trg,
				TA531_RC1.TA531_RC_Y_trg) != 0) {
			if (trigger_emergency) {
				Motor_Protection_EmergencyStop();
			}
			return false;
		}

		if ((HAL_GetTick() - start_tick) > timeout_ms) {
			Motor_Protection.protection_triggered = 1;
			Motor_Protection.error_type = 3;
			Motor_Protection.total_errors++;
			if (trigger_emergency) {
				Motor_Protection_EmergencyStop();
			}
			return false;
		}

		HAL_Delay(poll_ms);
	}

	return true;
}

void MoC_Init() {
	////42 MotorCtrl init
	SPI_Stop(Flash_SPI);
	HAL_Delay(10);
	SPI_Flash_Start(Flash_SPI);
	HAL_Delay(10);
	MotorInit_M1 = 0;
	MotorInit_M2 = 0;
	MotorInit_M3 = 0;
	MotorInit_M4 = 0;

	MotorCtrl_M1.MotorCtrl_HostID = HostID;
	MotorCtrl_M1.MotorCtrl_FuncType = 0x01;
	MotorCtrl_M1.MotorCtrl_FuncCode = 0x01;
	MotorCtrl_M1.MotorCtrl_ByteData = 0x01;		//06/07
	MotorCtrl_M1.MotorCtrl_DataCode = 0x000000;

	MotorCtrl_M2.MotorCtrl_HostID = HostID;
	MotorCtrl_M2.MotorCtrl_FuncType = 0x01;
	MotorCtrl_M2.MotorCtrl_FuncCode = 0x01;
	MotorCtrl_M2.MotorCtrl_ByteData = 0x01;		//06/07
	MotorCtrl_M2.MotorCtrl_DataCode = 0x000000;

	MotorInit_M1 = 1;	//init wait
	MotorInit_M2 = 1;	//init wait
	MotoCtrl_PackSend12();
	HAL_Delay(MOTOR_INIT_RETRY_MS);

	while ((MotorInit_M1 != 2) | (MotorInit_M2 != 2)) {
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "M1&2 Init Wait ");
		HAL_Delay(MOTOR_INIT_RETRY_MS);
		MotoCtrl_PackSend12();
	}

	char str1[16] = { 0 };

	OLED_ShowString(OLED_I2C_ch, OLED_type, 9, 0, "1 2");	// 9 11 13 15
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "M1&2 Init ok! ");

	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "X: ");
	itoa(M1_ID, str1, 16);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 4, 2, str1);
	itoa(TA531_RC1.TA531_RC_X_act, str1, 10);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 8, 2, str1);

	MotorCtrl_M3.MotorCtrl_HostID = HostID;
	MotorCtrl_M3.MotorCtrl_FuncType = 0x01;
	MotorCtrl_M3.MotorCtrl_FuncCode = 0x01;
	MotorCtrl_M3.MotorCtrl_ByteData = 0x01;		//06/07
	MotorCtrl_M3.MotorCtrl_DataCode = 0x000000;

	MotorInit_M3 = 1;	//init wait
	MotoCtrl_PackSend3();
	HAL_Delay(MOTOR_INIT_RETRY_MS);

	while (MotorInit_M3 != 2) {
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "M3 Init Wait ");
		HAL_Delay(MOTOR_INIT_RETRY_MS);
		MotoCtrl_PackSend3();
	}

	OLED_ShowString(OLED_I2C_ch, OLED_type, 13, 0, "3 ");	// 9 11 13 15
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "M3 Initialized! ");

	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3, "Y: ");
	itoa(M3_ID, str1, 16);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 4, 3, str1);
	itoa(TA531_RC1.TA531_RC_Y_act, str1, 10);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 8, 3, str1);

	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Check TouchPen! ");
	HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin, 1);	//F
	HAL_Delay(700);
	HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin, 0);	//F
	HAL_Delay(500);

	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "PuhUp Rst DispXY");
	HAL_Delay(500);

	if (SW_UP == 1)	////reset display xy
			{
		Sys_tune1();
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Hold Up 2s Reset");
		HAL_Delay(800);
		if (SW_UP == 1)	////reset display xy
				{
			Sys_tune1();
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Hold Up 1s Reset");
			HAL_Delay(800);
			Sys_tune1();
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Release & Reset");
			HAL_Delay(2000);

			if (SW_UP == 0)	// into reset display xy
					{
				OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1,
						"U/D/L/R Set X0Y0");
				HAL_Delay(800);
				OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "X0:");
				OLED_ShowString(OLED_I2C_ch, OLED_type, 8, 2, "Y0:");

				while (SW_BUTTON == 0)	//no push down
				{
					SW_UP_pre = SW_UP;
					SW_DW_pre = SW_DW;
					SW_LEFT_pre = SW_LEFT;
					SW_RIGHT_pre = SW_RIGHT;
					SW_BUTTON_pre = SW_BUTTON;

					SW_UP = (HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == 0);
					SW_DW = (HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin)
							== 0);
					SW_LEFT = (HAL_GPIO_ReadPin(SW_LEFT_GPIO_Port, SW_LEFT_Pin)
							== 0);
					SW_RIGHT = (HAL_GPIO_ReadPin(SW_RIGHT_GPIO_Port,
							SW_RIGHT_Pin) == 0);
					SW_BUTTON = (HAL_GPIO_ReadPin(SW_BUTTON_GPIO_Port,
							SW_BUTTON_Pin) == 0);

					if ((SW_UP == 1) & (SW_UP_pre == 1)) {
						TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act
								+ 10;

						TA531_RC1_fg = 2;
					} else if ((SW_UP == 1) & (SW_UP_pre == 0)) {
						TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act + 2;

						TA531_RC1_fg = 2;
					} else if ((SW_DW == 1) & (SW_DW_pre == 1)) {
						TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act
								- 10;

						if (TA531_RC1.TA531_RC_X_trg < 0) {
							TA531_RC1.TA531_RC_X_trg = 0;
						}
						TA531_RC1_fg = 2;
					} else if ((SW_DW == 1) & (SW_DW_pre == 0)) {
						TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act - 2;

						if (TA531_RC1.TA531_RC_X_trg < 0) {
							TA531_RC1.TA531_RC_X_trg = 0;
						}
						TA531_RC1_fg = 2;
					} else if ((SW_LEFT == 1) & (SW_LEFT_pre == 0)) {
						TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act - 2;

						if (TA531_RC1.TA531_RC_Y_trg < 0) {
							TA531_RC1.TA531_RC_Y_trg = 0;
						}
						TA531_RC1_fg = 2;
					} else if ((SW_LEFT == 1) & (SW_LEFT_pre == 1)) {
						TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act
								- 10;

						if (TA531_RC1.TA531_RC_Y_trg < 0) {
							TA531_RC1.TA531_RC_Y_trg = 0;
						}
						TA531_RC1_fg = 2;
					} else if ((SW_RIGHT == 1) & (SW_RIGHT_pre == 0)) {
						TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act + 2;

						TA531_RC1_fg = 2;
					} else if ((SW_RIGHT == 1) & (SW_RIGHT_pre == 1)) {
						TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act
								+ 10;

						TA531_RC1_fg = 2;
					}

					MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg,
							TA531_RC1.TA531_RC_Y_trg);
					HAL_Delay(MOTOR_LOOP_INTERVAL_MS);

					itoa(TA531_RC1.TA531_RC_X_trg, str1, 10);
					OLED_ShowString(OLED_I2C_ch, OLED_type, 3, 2, str1);
					itoa(TA531_RC1.TA531_RC_Y_trg, str1, 10);
					OLED_ShowString(OLED_I2C_ch, OLED_type, 11, 2, str1);
				}

				////push down
				ScreenSz_1.DispX0_32b = TA531_RC1.TA531_RC_X_trg;
				ScreenSz_1.DispX0[0] = TA531_RC1.TA531_RC_X_trg & 0xff;
				ScreenSz_1.DispX0[1] = (TA531_RC1.TA531_RC_X_trg >> 8) & 0xff;
				ScreenSz_1.DispX0[2] = (TA531_RC1.TA531_RC_X_trg >> 16) & 0xff;
				ScreenSz_1.DispX0[3] = (TA531_RC1.TA531_RC_X_trg >> 24) & 0xff;

				ScreenSz_1.DispY0_32b = TA531_RC1.TA531_RC_Y_trg;
				ScreenSz_1.DispY0[0] = TA531_RC1.TA531_RC_Y_trg & 0xff;
				ScreenSz_1.DispY0[1] = (TA531_RC1.TA531_RC_Y_trg >> 8) & 0xff;
				ScreenSz_1.DispY0[2] = (TA531_RC1.TA531_RC_Y_trg >> 16) & 0xff;
				ScreenSz_1.DispY0[3] = (TA531_RC1.TA531_RC_Y_trg >> 24) & 0xff;

				SPI_Flash_Start(Flash_SPI);
				SPI_Flash_WtritEnable();
				HAL_Delay(5);
					SPI_Flash_WriteSomeBytes(ScreenSz_1.DispX0, Sys_Addr_DispX0,
							sizeof(int));
					SPI_Flash_WaitNoBusy();
					HAL_Delay(1);
				SPI_Flash_WtritEnable();
				HAL_Delay(5);
					SPI_Flash_WriteSomeBytes(ScreenSz_1.DispY0, Sys_Addr_DispY0,
							sizeof(int));
					SPI_Flash_WaitNoBusy();
					HAL_Delay(1);
				uint8_t temp1[4];	//temp3,temp4;
				SPI_Flash_ReadBytes(temp1, Sys_Addr_DispX0, sizeof(int));

				if ((temp1[0] != ScreenSz_1.DispX0[0])
						|| (temp1[1] != ScreenSz_1.DispX0[1])
						|| (temp1[2] != ScreenSz_1.DispX0[2])
						|| (temp1[3] != ScreenSz_1.DispX0[3])) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1,
							"WriteFlash Err!");
					while ((temp1[0] != ScreenSz_1.DispX0[0])
							|| (temp1[1] != ScreenSz_1.DispX0[1])
							|| (temp1[2] != ScreenSz_1.DispX0[2])
							|| (temp1[3] != ScreenSz_1.DispX0[3])) {
						SPI_Flash_WtritEnable();
						HAL_Delay(5);
						SPI_Flash_WriteSomeBytes(ScreenSz_1.DispX0,
						Sys_Addr_DispX0, sizeof(int));
						HAL_Delay(5);
						SPI_Flash_ReadBytes(temp1, Sys_Addr_DispX0,
								sizeof(int));
					}
					OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1,
							"WriteFlash OK!");
				}

				SPI_Flash_ReadBytes(temp1, Sys_Addr_DispY0, sizeof(int));
				// ⬅️ | 改为 ||
				if ((temp1[0] != ScreenSz_1.DispY0[0])
						|| (temp1[1] != ScreenSz_1.DispY0[1])
						|| (temp1[2] != ScreenSz_1.DispY0[2])
						|| (temp1[3] != ScreenSz_1.DispY0[3])) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1,
							"WriteFlash Err!");
					while ((temp1[0] != ScreenSz_1.DispY0[0])
							|| (temp1[1] != ScreenSz_1.DispY0[1])
							|| (temp1[2] != ScreenSz_1.DispY0[2])
							|| (temp1[3] != ScreenSz_1.DispY0[3])) {
						SPI_Flash_WtritEnable();
						HAL_Delay(5);
						SPI_Flash_WriteSomeBytes(ScreenSz_1.DispY0,
						Sys_Addr_DispY0, sizeof(int));
						HAL_Delay(5);
						SPI_Flash_ReadBytes(temp1, Sys_Addr_DispY0,
								sizeof(int));
					}
					OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1,
							"WriteFlash OK!");
				}

				OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1,
						"U/D/L/R Set X1Y1");
				HAL_Delay(800);
				OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3, "X1:");
				OLED_ShowString(OLED_I2C_ch, OLED_type, 8, 3, "Y1:");

				while (SW_BUTTON == 0) {
					// UP键 - 渐进加速
					if (SW_UP == 1) {
						int step;
						if (SW_UP_cnt == 0) {
							step = 1;
						} else if (SW_UP_cnt < 8) {
							step = 5;
						} else if (SW_UP_cnt < 15) {
							step = 10;
						} else {
							step = 30;
						}
						TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act + step;
						TA531_RC1_fg = 2;
					}

					// DOWN键 - 渐进加速
					else if (SW_DW == 1) {
						int step;
						if (SW_DW_cnt == 0) {
							step = 1;
						} else if (SW_DW_cnt < 8) {
							step = 5;
						} else if (SW_DW_cnt < 15) {
							step = 10;
						} else {
							step = 30;
						}
						TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act - step;
						if (TA531_RC1.TA531_RC_X_trg < 0) {
							TA531_RC1.TA531_RC_X_trg = 0;
						}
						TA531_RC1_fg = 2;
					}

					// LEFT键 - 渐进加速
					else if (SW_LEFT == 1) {
						int step;
						if (SW_LEFT_cnt == 0) {
							step = 1;
						} else if (SW_LEFT_cnt < 8) {
							step = 5;
						} else if (SW_LEFT_cnt < 15) {
							step = 10;
						} else {
							step = 30;
						}
						TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act - step;
						if (TA531_RC1.TA531_RC_Y_trg < 0) {
							TA531_RC1.TA531_RC_Y_trg = 0;
						}
						TA531_RC1_fg = 2;
					}

					// RIGHT键 - 渐进加速
					else if (SW_RIGHT == 1) {
						int step;
						if (SW_RIGHT_cnt == 0) {
							step = 1;
						} else if (SW_RIGHT_cnt < 8) {
							step = 5;
						} else if (SW_RIGHT_cnt < 15) {
							step = 10;
						} else {
							step = 30;
						}
						TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act + step;
						TA531_RC1_fg = 2;
					}

					MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg,
							TA531_RC1.TA531_RC_Y_trg);
					HAL_Delay(50);

					itoa(TA531_RC1.TA531_RC_X_trg, str1, 10);
					OLED_ShowString(OLED_I2C_ch, OLED_type, 3, 2, str1);
					itoa(TA531_RC1.TA531_RC_Y_trg, str1, 10);
					OLED_ShowString(OLED_I2C_ch, OLED_type, 11, 2, str1);
				}

				////push down
				ScreenSz_1.DispX1_32b = TA531_RC1.TA531_RC_X_trg;
				ScreenSz_1.DispX1[0] = TA531_RC1.TA531_RC_X_trg & 0xff;
				ScreenSz_1.DispX1[1] = (TA531_RC1.TA531_RC_X_trg >> 8) & 0xff;
				ScreenSz_1.DispX1[2] = (TA531_RC1.TA531_RC_X_trg >> 16) & 0xff;
				ScreenSz_1.DispX1[3] = (TA531_RC1.TA531_RC_X_trg >> 24) & 0xff;

				ScreenSz_1.DispY1_32b = TA531_RC1.TA531_RC_Y_trg;
				ScreenSz_1.DispY1[0] = TA531_RC1.TA531_RC_Y_trg & 0xff;
				ScreenSz_1.DispY1[1] = (TA531_RC1.TA531_RC_Y_trg >> 8) & 0xff;
				ScreenSz_1.DispY1[2] = (TA531_RC1.TA531_RC_Y_trg >> 16) & 0xff;
				ScreenSz_1.DispY1[3] = (TA531_RC1.TA531_RC_Y_trg >> 24) & 0xff;

				SPI_Flash_Start(Flash_SPI);
				HAL_Delay(1);
				SPI_Flash_WtritEnable();
				HAL_Delay(5);
					SPI_Flash_WriteSomeBytes(ScreenSz_1.DispX1, Sys_Addr_DispX1,
							sizeof(int));
					SPI_Flash_WaitNoBusy();
					HAL_Delay(1);
				SPI_Flash_WtritEnable();
				HAL_Delay(5);
					SPI_Flash_WriteSomeBytes(ScreenSz_1.DispY1, Sys_Addr_DispY1,
							sizeof(int));
					SPI_Flash_WaitNoBusy();
					HAL_Delay(1);

				SPI_Flash_ReadBytes(temp1, Sys_Addr_DispX1, sizeof(int));
				if ((temp1[0] != ScreenSz_1.DispX1[0])
						|| (temp1[1] != ScreenSz_1.DispX1[1])
						|| (temp1[2] != ScreenSz_1.DispX1[2])
						|| (temp1[3] != ScreenSz_1.DispX1[3])) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1,
							"WriteFlash Err!");
					while ((temp1[0] != ScreenSz_1.DispX1[0])
							|| (temp1[1] != ScreenSz_1.DispX1[1])
							|| (temp1[2] != ScreenSz_1.DispX1[2])
							|| (temp1[3] != ScreenSz_1.DispX1[3])) {
						SPI_Flash_WtritEnable();
						HAL_Delay(5);
						SPI_Flash_WriteSomeBytes(ScreenSz_1.DispX1,
						Sys_Addr_DispX1, sizeof(int));
						HAL_Delay(5);
						SPI_Flash_ReadBytes(temp1, Sys_Addr_DispX1,
								sizeof(int));
					}
					OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1,
							"WriteFlash OK!");
				}

				SPI_Flash_ReadBytes(temp1, Sys_Addr_DispY1, sizeof(int));
				if ((temp1[0] != ScreenSz_1.DispY1[0])
						|| (temp1[1] != ScreenSz_1.DispY1[1])
						|| (temp1[2] != ScreenSz_1.DispY1[2])
						|| (temp1[3] != ScreenSz_1.DispY1[3])) {
					OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1,
							"WriteFlash Err!");
					while ((temp1[0] != ScreenSz_1.DispY1[0])
							|| (temp1[1] != ScreenSz_1.DispY1[1])
							|| (temp1[2] != ScreenSz_1.DispY1[2])
							|| (temp1[3] != ScreenSz_1.DispY1[3])) {
						SPI_Flash_WtritEnable();
						HAL_Delay(5);
						SPI_Flash_WriteSomeBytes(ScreenSz_1.DispY1,
						Sys_Addr_DispY1, sizeof(int));
						HAL_Delay(5);
						SPI_Flash_ReadBytes(temp1, Sys_Addr_DispY1,
								sizeof(int));
					}
					OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1,
							"WriteFlash OK!");
				}


				////push down

				////Display Area reset finish		//DispX0,DispX1,DispY0,DispY1;

				OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1,
						"Reset XY finish!");
				OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2,
						"                ");
				OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3,
						"                ");

			}	//finish reset display xy
		}	//reset display xy comfirm
	}	//reset display xy comfirm
	else {	//DispX0,DispX1,DispY0,DispY1;

//		SPI_Flash_Start(Flash_SPI);

		HAL_Delay(1);
		SPI_Flash_ReadBytes(ScreenSz_1.DispX0, Sys_Addr_DispX0, sizeof(int));
		SPI_Flash_ReadBytes(ScreenSz_1.DispY0, Sys_Addr_DispY0, sizeof(int));
		SPI_Flash_ReadBytes(ScreenSz_1.DispX1, Sys_Addr_DispX1, sizeof(int));
		SPI_Flash_ReadBytes(ScreenSz_1.DispY1, Sys_Addr_DispY1, sizeof(int));

		ScreenSz_1.DispX0_32b = ScreenSz_1.DispX0[0]
				+ (ScreenSz_1.DispX0[1] << 8) + (ScreenSz_1.DispX0[2] << 16)
				+ (ScreenSz_1.DispX0[3] << 24);
		ScreenSz_1.DispX1_32b = ScreenSz_1.DispX1[0]
				+ (ScreenSz_1.DispX1[1] << 8) + (ScreenSz_1.DispX1[2] << 16)
				+ (ScreenSz_1.DispX1[3] << 24);
		ScreenSz_1.DispY0_32b = ScreenSz_1.DispY0[0]
				+ (ScreenSz_1.DispY0[1] << 8) + (ScreenSz_1.DispY0[2] << 16)
				+ (ScreenSz_1.DispY0[3] << 24);
		ScreenSz_1.DispY1_32b = ScreenSz_1.DispY1[0]
				+ (ScreenSz_1.DispY1[1] << 8) + (ScreenSz_1.DispY1[2] << 16)
				+ (ScreenSz_1.DispY1[3] << 24);

		// ⬅️⬅️⬅️ 添加调试输出 - 显示读取的原始字节
		char str1[16];
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 0, "Raw Bytes:");

		// 显示X0的4个字节（16进制）
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "X0:");
		itoa(ScreenSz_1.DispX0[0], str1, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 3, 1, str1);
		itoa(ScreenSz_1.DispX0[1], str1, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 5, 1, str1);
		itoa(ScreenSz_1.DispX0[2], str1, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 7, 1, str1);
		itoa(ScreenSz_1.DispX0[3], str1, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 9, 1, str1);

		// 显示X1的4个字节（16进制）
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "X1:");
		itoa(ScreenSz_1.DispX1[0], str1, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 3, 2, str1);
		itoa(ScreenSz_1.DispX1[1], str1, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 5, 2, str1);
		itoa(ScreenSz_1.DispX1[2], str1, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 7, 2, str1);
		itoa(ScreenSz_1.DispX1[3], str1, 16);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 9, 2, str1);

		HAL_Delay(2000);  // 等待2秒让你看清

		// 显示转换后的32位值（10进制）
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 0, "32bit Values:");

		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "X0:");
		itoa(ScreenSz_1.DispX0_32b, str1, 10);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 3, 1, str1);

		OLED_ShowString(OLED_I2C_ch, OLED_type, 8, 1, "X1:");
		itoa(ScreenSz_1.DispX1_32b, str1, 10);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 11, 1, str1);

		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "Y0:");
		itoa(ScreenSz_1.DispY0_32b, str1, 10);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 3, 2, str1);

		OLED_ShowString(OLED_I2C_ch, OLED_type, 8, 2, "Y1:");
		itoa(ScreenSz_1.DispY1_32b, str1, 10);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 11, 2, str1);

		HAL_Delay(3000);  // 等待3秒让你看清

		HAL_Delay(10);
		OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "Read XY fm Flash");
		HAL_Delay(200);

		if ((ScreenSz_1.DispX0_32b > 0)
				& (ScreenSz_1.DispX0_32b < ScreenSz_1.DispX1_32b)
				& (ScreenSz_1.DispX1_32b < XmaxLimit)
				& (ScreenSz_1.DispY0_32b > 0)
				& (ScreenSz_1.DispY0_32b < ScreenSz_1.DispY1_32b)
				& (ScreenSz_1.DispY1_32b < YmaxLimit)) {
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "XY Check Pass!");
		} else {
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "XY Check Fail!");
			Sys_tune1();
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "Pls Reboot &");
			Sys_tune1();
			OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3, "Reset XY Area");
			Sys_tune1();

			while (1)
				;
		}
	}	//////finish reset display xy

		TA531_RC1.TA531_RC_X_trg = ScreenSz_1.DispX0_32b;
		TA531_RC1.TA531_RC_Y_trg = ScreenSz_1.DispY0_32b;
		TA531_RC1_fg = 2;
		Motor_Protection_Reset();
		Motor_Protection.last_X_pos = TA531_RC1.TA531_RC_X_act;
		Motor_Protection.last_Y_pos = TA531_RC1.TA531_RC_Y_act;
		if (!WaitMotorToTargetWithProtection(MOVE_WAIT_TIMEOUT_INIT_MS, MOTOR_WAIT_POLL_MS, true)) {
			return;
		}

	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "Trg: (");
	itoa(TA531_RC1.TA531_RC_X_trg, str1, 10);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 6, 2, str1);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 2, ", ");
	itoa(TA531_RC1.TA531_RC_Y_trg, str1, 10);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 12, 2, str1);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 15, 2, ")");

	HAL_Delay(1000);
	HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin, 1);	//touch pen push
	HAL_Delay(1000);
	HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin, 0);	//touch pen release
	HAL_Delay(200);

		TA531_RC1.TA531_RC_X_trg = ScreenSz_1.DispX1_32b;
		TA531_RC1.TA531_RC_Y_trg = ScreenSz_1.DispY1_32b;
		TA531_RC1_fg = 2;
		Motor_Protection_Reset();
		Motor_Protection.last_X_pos = TA531_RC1.TA531_RC_X_act;
		Motor_Protection.last_Y_pos = TA531_RC1.TA531_RC_Y_act;
		if (!WaitMotorToTargetWithProtection(MOVE_WAIT_TIMEOUT_INIT_MS, MOTOR_WAIT_POLL_MS, true)) {
			return;
		}

	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "Trg: (");
	itoa(TA531_RC1.TA531_RC_X_trg, str1, 10);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 6, 2, str1);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 2, ", ");
	itoa(TA531_RC1.TA531_RC_Y_trg, str1, 10);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 12, 2, str1);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 15, 2, ")");

	HAL_Delay(1000);
	HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin, 1);	//touch pen push
	HAL_Delay(1000);
	HAL_GPIO_WritePin(KL15_RELAY_GPIO_Port, KL15_RELAY_Pin, 0);	//touch pen release
	HAL_Delay(200);

		TA531_RC1_fg = 2;
		TA531_RC1.TA531_RC_X_trg = ScreenSz_1.DispX0_32b;  // 改为X0
		TA531_RC1.TA531_RC_Y_trg = ScreenSz_1.DispY0_32b;  // 改为Y0
		Motor_Protection_Reset();
		Motor_Protection.last_X_pos = TA531_RC1.TA531_RC_X_act;
		Motor_Protection.last_Y_pos = TA531_RC1.TA531_RC_Y_act;
		if (!WaitMotorToTargetWithProtection(MOVE_WAIT_TIMEOUT_INIT_MS, MOTOR_WAIT_POLL_MS, true)) {
			return;
		}

	itoa(TA531_RC1.TA531_RC_X_trg, str1, 10);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 6, 2, str1);
	itoa(TA531_RC1.TA531_RC_Y_trg, str1, 10);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 12, 2, str1);

	HAL_Delay(1000);

	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3, "Act: (");
	OLED_ShowString(OLED_I2C_ch, OLED_type, 10, 3, ", ");
	OLED_ShowString(OLED_I2C_ch, OLED_type, 15, 3, ")");

}

void MotoCtrl_PackSend12() {
	MotrCtrl_1_DATA[0] = (MotorCtrl_M1.MotorCtrl_HostID >> 3) & 0xff;
	MotrCtrl_1_DATA[1] = (MotorCtrl_M1.MotorCtrl_HostID & 0x07) << 5;
	MotrCtrl_1_DATA[2] = ((MotorCtrl_M1.MotorCtrl_FuncType & 0x07) << 5)
			+ (MotorCtrl_M1.MotorCtrl_FuncCode & 0x1f);
	MotrCtrl_1_DATA[3] = MotorCtrl_M1.MotorCtrl_DataCode & 0xff;
	MotrCtrl_1_DATA[4] = (MotorCtrl_M1.MotorCtrl_DataCode >> 8) & 0xff;
	MotrCtrl_1_DATA[5] = (MotorCtrl_M1.MotorCtrl_DataCode >> 16) & 0xff;
	MotrCtrl_1_DATA[6] = (MotorCtrl_M1.MotorCtrl_DataCode >> 24) & 0xff;
	MotrCtrl_1_DATA[7] = MotorCtrl_M1.MotorCtrl_ByteData;

	MotrCtrl_2_DATA[0] = (MotorCtrl_M2.MotorCtrl_HostID >> 3) & 0xff;
	MotrCtrl_2_DATA[1] = (MotorCtrl_M2.MotorCtrl_HostID & 0x07) << 5;
	MotrCtrl_2_DATA[2] = ((MotorCtrl_M2.MotorCtrl_FuncType & 0x07) << 5)
			+ (MotorCtrl_M2.MotorCtrl_FuncCode & 0x1f);
	MotrCtrl_2_DATA[3] = MotorCtrl_M2.MotorCtrl_DataCode & 0xff;
	MotrCtrl_2_DATA[4] = (MotorCtrl_M2.MotorCtrl_DataCode >> 8) & 0xff;
	MotrCtrl_2_DATA[5] = (MotorCtrl_M2.MotorCtrl_DataCode >> 16) & 0xff;
	MotrCtrl_2_DATA[6] = (MotorCtrl_M2.MotorCtrl_DataCode >> 24) & 0xff;
	MotrCtrl_2_DATA[7] = MotorCtrl_M2.MotorCtrl_ByteData;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &MotrCtrl_2_TxHeader,
				MotrCtrl_2_DATA);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &MotrCtrl_1_TxHeader,
				MotrCtrl_1_DATA);
	HAL_Delay(MOTOR_SEND_GAP_MS);
}

//void MotoCtrl_PackSend2()
//{
//    MotrCtrl_2_DATA[0] = (MotorCtrl_M2.MotorCtrl_HostID >>3) & 0xff ;
//    MotrCtrl_2_DATA[1] = (MotorCtrl_M2.MotorCtrl_HostID & 0x07) << 5 ;
//    MotrCtrl_2_DATA[2] = ((MotorCtrl_M2.MotorCtrl_FuncType & 0x07)<< 5) + (MotorCtrl_M2.MotorCtrl_FuncCode & 0x1f) ;
//    MotrCtrl_2_DATA[3] = MotorCtrl_M2.MotorCtrl_DataCode & 0xff ;
//    MotrCtrl_2_DATA[4] = (MotorCtrl_M2.MotorCtrl_DataCode >>8) & 0xff ;
//    MotrCtrl_2_DATA[5] = (MotorCtrl_M2.MotorCtrl_DataCode >>16) & 0xff ;
//    MotrCtrl_2_DATA[6] = (MotorCtrl_M2.MotorCtrl_DataCode >>24) & 0xff ;
//    MotrCtrl_2_DATA[7] = MotorCtrl_M2.MotorCtrl_ByteData;
//
//    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &MotrCtrl_2_TxHeader, MotrCtrl_2_DATA);
//}

void MotoCtrl_PackSend3() {
	MotrCtrl_3_DATA[0] = (MotorCtrl_M3.MotorCtrl_HostID >> 3) & 0xff;
	MotrCtrl_3_DATA[1] = (MotorCtrl_M3.MotorCtrl_HostID & 0x07) << 5;
	MotrCtrl_3_DATA[2] = ((MotorCtrl_M3.MotorCtrl_FuncType & 0x07) << 5)
			+ (MotorCtrl_M3.MotorCtrl_FuncCode & 0x1f);
	MotrCtrl_3_DATA[3] = MotorCtrl_M3.MotorCtrl_DataCode & 0xff;
	MotrCtrl_3_DATA[4] = (MotorCtrl_M3.MotorCtrl_DataCode >> 8) & 0xff;
	MotrCtrl_3_DATA[5] = (MotorCtrl_M3.MotorCtrl_DataCode >> 16) & 0xff;
	MotrCtrl_3_DATA[6] = (MotorCtrl_M3.MotorCtrl_DataCode >> 24) & 0xff;
	MotrCtrl_3_DATA[7] = MotorCtrl_M3.MotorCtrl_ByteData;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &MotrCtrl_3_TxHeader,
			MotrCtrl_3_DATA);
	HAL_Delay(MOTOR_SEND_GAP_MS);
}

void MotoCtrl_PackSend4() {

}

void MotoCtrl_PositionLoop(int PositionX_mm, int PositionY_mm) {
	if (PositionX_mm < 0)	//10mm -10 = 0
			{
		PositionX_mm = 0;
	} else if (PositionX_mm > XmaxLimit)	//190mm - 10mm = 180
	{
		PositionX_mm = XmaxLimit;
	}

	if (PositionY_mm < 0)	//10mm
			{
		PositionY_mm = 0;
	} else if (PositionY_mm > YmaxLimit)	//190mm + 10mm
	{
		PositionY_mm = YmaxLimit;
	}

	MotorCtrl_M3.MotorCtrl_HostID = HostID;
	MotorCtrl_M3.MotorCtrl_FuncType = 0x01;
	MotorCtrl_M3.MotorCtrl_FuncCode = 0x02;
	MotorCtrl_M3.MotorCtrl_ByteData = 0x01;		//06/07
	MotorCtrl_M3.MotorCtrl_DataCode = -(PositionY_mm + 10) * 160;

	MotoCtrl_PackSend3();

	MotorCtrl_M1.MotorCtrl_HostID = HostID;
	MotorCtrl_M1.MotorCtrl_FuncType = 0x01;
	MotorCtrl_M1.MotorCtrl_FuncCode = 0x02;
	MotorCtrl_M1.MotorCtrl_ByteData = 0x01;		//06/07
	MotorCtrl_M1.MotorCtrl_DataCode = -(PositionX_mm + 10) * 160;

	MotorCtrl_M2.MotorCtrl_HostID = HostID;
	MotorCtrl_M2.MotorCtrl_FuncType = 0x01;
	MotorCtrl_M2.MotorCtrl_FuncCode = 0x02;
	MotorCtrl_M2.MotorCtrl_ByteData = 0x01;		//06/07
	MotorCtrl_M2.MotorCtrl_DataCode = -(PositionX_mm + 10) * 160;

	MotoCtrl_PackSend12();

}

uint8_t ByteEncryp(uint8_t byteData) {
	return (EncrypKey ^ byteData);
}

//	MD5
#define ROTATELEFT(value, bits) (((value) << (bits)) | ((value) >> (32 - (bits))))

/**
 * @desc: convert message and mes_bkp string into integer array and store them in w
 */
//static void md5_process_part1(uint32_t *w, unsigned char *message,
//		uint32_t *pos, uint32_t mes_len, const unsigned char *mes_bkp) {
//	uint32_t i; // used in for loop
//
//	for (i = 0; i <= 15; i++) {
//		int32_t count = 0;
//		while (*pos < mes_len && count <= 24) {
//			w[i] += (((uint32_t) message[*pos]) << count);
//			(*pos)++;
//			count += 8;
//		}
//		while (count <= 24) {
//			w[i] += (((uint32_t) mes_bkp[*pos - mes_len]) << count);
//			(*pos)++;
//			count += 8;
//		}
//	}
//}
/**
 * @desc: start encryption based on w
 */
//static void md5_process_part2(uint32_t abcd[4], uint32_t *w,
//		const uint32_t k[64], const uint32_t s[64]) {
//	uint32_t i; // used in for loop
//
//	uint32_t a = abcd[0];
//	uint32_t b = abcd[1];
//	uint32_t c = abcd[2];
//	uint32_t d = abcd[3];
//	uint32_t f = 0;
//	uint32_t g = 0;
//
//	for (i = 0; i < 64; i++) {
//		if (i >= 0 && i <= 15) {
//			f = (b & c) | ((~b) & d);
//			g = i;
//		} else if (i >= 16 && i <= 31) {
//			f = (d & b) | ((~d) & c);
//			g = (5 * i + 1) % 16;
//		} else if (i >= 32 && i <= 47) {
//			f = b ^ c ^ d;
//			g = (3 * i + 5) % 16;
//		} else if (i >= 48 && i <= 63) {
//			f = c ^ (b | (~d));
//			g = (7 * i) % 16;
//		}
//		uint32_t temp = d;
//		d = c;
//		c = b;
//		b = ROTATELEFT((a + f + k[i] + w[g]), s[i]) + b;
//		a = temp;
//	}
//
//	abcd[0] += a;
//	abcd[1] += b;
//	abcd[2] += c;
//	abcd[3] += d;
//}
//
//static const uint32_t k_table[] = { 0xd76aa478, 0xe8c7b756, 0x242070db,
//		0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501, 0x698098d8,
//		0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e,
//		0x49b40821, 0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d,
//		0x02441453, 0xd8a1e681, 0xe7d3fbc8, 0x21e1cde6, 0xc33707d6, 0xf4d50d87,
//		0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a, 0xfffa3942,
//		0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60,
//		0xbebfbc70, 0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039,
//		0xe6db99e5, 0x1fa27cf8, 0xc4ac5665, 0xf4292244, 0x432aff97, 0xab9423a7,
//		0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1, 0x6fa87e4f,
//		0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb,
//		0xeb86d391 };
//
//static const uint32_t s_table[] = { 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
//		7, 12, 17, 22, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
//		4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 6, 10, 15,
//		21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21 };
//
//int32_t cal_md5(unsigned char *result, unsigned char *data, int length) {
//	if (result == NULL) {
//		return 1;
//	}
//
//	uint32_t w[16];
//
//	uint32_t i; // used in for loop
//
//	uint32_t mes_len = length;
//	uint32_t looptimes = (mes_len + 8) / 64 + 1;
//	uint32_t abcd[] = { 0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476 };
//
//	uint32_t pos = 0; // position pointer for message
//	uint32_t bkp_len = 64 * looptimes - mes_len; // 经过计算发现不超�?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????72
//
////    unsigned char *bkp_mes = (unsigned char *)calloc(1, bkp_len);
//	unsigned char bkp_mes[80];
//	for (int i = 0; i < 80; i++) //初始�?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
//			{
//		bkp_mes[i] = 0;
//	}
//
//	bkp_mes[0] = (unsigned char) (0x80);
//	uint64_t mes_bit_len = ((uint64_t) mes_len) * 8;
//	for (i = 0; i < 8; i++) {
//		bkp_mes[bkp_len - i - 1] = (unsigned char) ((mes_bit_len
//				& (0x00000000000000FF << (8 * (7 - i)))) >> (8 * (7 - i)));
//	}
//
//	for (i = 0; i < looptimes; i++) {
//		for (int j = 0; j < 16; j++) //初始�?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
//				{
//			w[j] = 0x00000000;
//		}
//
//		md5_process_part1(w, data, &pos, mes_len, bkp_mes); // compute w
//
//		md5_process_part2(abcd, w, k_table, s_table); // calculate md5 and store the result in abcd
//	}
//
//	for (int i = 0; i < 16; i++) {
//		result[i] = ((unsigned char*) abcd)[i];
//	}
//
//	return 0;
//}
//void PCF8563_ReadDateTime(uint8_t *buffer)
//{
//HAL_I2C_Mem_Read(&hi2c1, PCF8563_ADDRESS, 2, I2C_MEMADD_SIZE_8BIT, buffer, 7, HAL_MAX_DELAY);
//}
void Set_SystemReboot() {
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, "System Error...");
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "Reboot in 3s...");
	Sys_tune1();
	HAL_Delay(800);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "Reboot in 2s...");
	Sys_tune1();
	HAL_Delay(800);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "Reboot in 1s...");
	Sys_tune1();
	HAL_Delay(800);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 2, "Rebooting . . .");
	Sys_tune1();
	//_ASM volatile ("cpsid i");
	//__set_FAULTMASK(1);//关闭总中�????????????????????????
	HAL_NVIC_SystemReset();
}

void LIN_RESET(UART_HandleTypeDef *handle) {

	HAL_UART_AbortReceive(handle);
	HAL_UART_AbortReceive_IT(handle);
	/* Clear RXNE interrupt flag *//* Discard the received data */
	__HAL_UART_SEND_REQ(handle, UART_RXDATA_FLUSH_REQUEST);

}

// 停止SPI（禁用SPI外设�???????????
void SPI_Stop(SPI_HandleTypeDef *hspi) {
	// 等待传输完成（检查忙标志�???????????
	while (HAL_SPI_GetState(hspi) == HAL_SPI_STATE_BUSY)
		;

	// 禁用SPI外设（�?�过清除CR1寄存器的SPE位）
	__HAL_SPI_DISABLE(hspi);

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = SPI_Flash_NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(SPI_Flash_NSS_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SPI_TFT_NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(SPI_TFT_NSS_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(SPI_Flash_NSS_GPIO_Port, SPI_Flash_NSS_Pin, 1);
	HAL_GPIO_WritePin(SPI_TFT_NSS_GPIO_Port, SPI_TFT_NSS_Pin, 1);

}

// 重启SPI（重新使能）
void SPI_Flash_Start(SPI_HandleTypeDef *hspi) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = SPI_Flash_NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(SPI_Flash_NSS_GPIO_Port, &GPIO_InitStruct);

	// 重新使能SPI
	__HAL_SPI_ENABLE(hspi);

	MX_SPI1_Init();

	// Flash CS idle should stay high; each flash command function toggles CS itself.
	HAL_GPIO_WritePin(SPI_Flash_NSS_GPIO_Port, SPI_Flash_NSS_Pin, 1);
	HAL_GPIO_WritePin(SPI_TFT_NSS_GPIO_Port, SPI_TFT_NSS_Pin, 1);
}

void SPI_TFT_Start(SPI_HandleTypeDef *hspi) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = SPI_TFT_NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(SPI_TFT_NSS_GPIO_Port, &GPIO_InitStruct);

	// 重新使能SPI
	__HAL_SPI_ENABLE(hspi);

	MX_SPI1_Init();

	HAL_GPIO_WritePin(SPI_Flash_NSS_GPIO_Port, SPI_Flash_NSS_Pin, 1);
	HAL_GPIO_WritePin(SPI_TFT_NSS_GPIO_Port, SPI_TFT_NSS_Pin, 0);
}

void Sys_tune1() {
	htim1.Instance = TIM1;
//	  htim1.Init.Prescaler = 63;
//	  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000;
//	  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//	  htim1.Init.RepetitionCounter = 0;
//	  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 200;
//	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 0;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_Delay(200);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

void Sys_tuneX(uint32_t fq)	//fq bigger,sound lower
{

	htim1.Instance = TIM1;
	htim1.Init.Period = fq;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = fq / 4;

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 0;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_Delay(200);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

// 1. 角度转脉宽函数（保持不变）
uint32_t PWMServo_Ag2Pulse(uint32_t ag) {
	uint32_t PWM_Pulse;
	if (ag > PWM_agMAX) {
		ag = PWM_agMAX;
	}
	PWM_Pulse = PWM_ag0 + ag * (PWM_ag90 - PWM_ag0) / 90;
	return PWM_Pulse;
}

// 2. TIM2_CH3 舵机控制（修正版）
void PWMServo2_3_AGout(uint32_t ag) {
	// 直接设置CCR值，不需要重新初始化
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWMServo_Ag2Pulse(ag));
}

// 3. TIM2_CH4 舵机控制（修正版）
void PWMServo2_4_AGout(uint32_t ag) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWMServo_Ag2Pulse(ag));
}

// 4. TIM3_CH1 舵机控制（修正版）
void PWMServo3_1_AGout(uint32_t ag) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWMServo_Ag2Pulse(ag));
}

// 5. TIM3_CH2 舵机控制（修正版）
void PWMServo3_2_AGout(uint32_t ag) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWMServo_Ag2Pulse(ag));
}

/* ========== 电机保护系统实现 ========== */

void Motor_Protection_Init(void) {
	Motor_Protection.last_X_pos = 0;
	Motor_Protection.last_Y_pos = 0;
	Motor_Protection.X_direction_changes = 0;
	Motor_Protection.Y_direction_changes = 0;
	Motor_Protection.stuck_counter = 0;
	Motor_Protection.movement_timeout = 0;
	Motor_Protection.last_progress_tick = HAL_GetTick();
	Motor_Protection.protection_triggered = 0;
	Motor_Protection.error_type = 0;
	Motor_Protection.total_errors = 0;
}

void Motor_Protection_Reset(void) {
	Motor_Protection.X_direction_changes = 0;
	Motor_Protection.Y_direction_changes = 0;
	Motor_Protection.stuck_counter = 0;
	Motor_Protection.movement_timeout = 0;
	Motor_Protection.last_progress_tick = HAL_GetTick();
	Motor_Protection.protection_triggered = 0;
	Motor_Protection.error_type = 0;
}

uint8_t Motor_Protection_Check(int16_t current_X, int16_t current_Y,
		int16_t target_X, int16_t target_Y) {
	uint32_t now_tick = HAL_GetTick();

	if (MOTOR_PROTECTION_ENABLED == 0) {
		return 0;
	}

	if (Motor_Protection.protection_triggered > 0) {
		return 1;
	}

	int16_t delta_X = current_X - Motor_Protection.last_X_pos;
	int16_t delta_Y = current_Y - Motor_Protection.last_Y_pos;

	// ===== 跳过无效采样（位置没更新）=====
	if (delta_X == 0 && delta_Y == 0) {
		Motor_Protection.movement_timeout = now_tick
				- Motor_Protection.last_progress_tick;
		if (Motor_Protection.movement_timeout >= MAX_MOVEMENT_TIMEOUT_MS) {
			Motor_Protection.protection_triggered = 1;
			Motor_Protection.error_type = 3;
			Motor_Protection.total_errors++;
			return 1;
		}
		return 0;  // 跳过停滞检测
	}
	// =====================================

	int16_t target_delta_X = target_X - current_X;
	int16_t target_delta_Y = target_Y - current_Y;

	// 检测1: 方向反复变化
	if (delta_X != 0 && target_delta_X != 0) {
		if ((delta_X > 0 && target_delta_X < 0)
				|| (delta_X < 0 && target_delta_X > 0)) {
			Motor_Protection.X_direction_changes++;

			if (Motor_Protection.X_direction_changes >= MAX_DIRECTION_CHANGES) {
				Motor_Protection.protection_triggered = 1;
				Motor_Protection.error_type = 1;
				Motor_Protection.total_errors++;
				return 1;
			}
		}
	}

	if (delta_Y != 0 && target_delta_Y != 0) {
		if ((delta_Y > 0 && target_delta_Y < 0)
				|| (delta_Y < 0 && target_delta_Y > 0)) {
			Motor_Protection.Y_direction_changes++;

			if (Motor_Protection.Y_direction_changes >= MAX_DIRECTION_CHANGES) {
				Motor_Protection.protection_triggered = 1;
				Motor_Protection.error_type = 1;
				Motor_Protection.total_errors++;
				return 1;
			}
		}
	}

	// 检测2: 位置停滞
	if (abs(delta_X) <= POSITION_TOLERANCE && abs(delta_Y) <= POSITION_TOLERANCE) {
		Motor_Protection.stuck_counter++;

		if (Motor_Protection.stuck_counter >= MAX_STUCK_COUNT) {
			Motor_Protection.protection_triggered = 1;
			Motor_Protection.error_type = 2;
			Motor_Protection.total_errors++;
			return 1;
		}
	} else {
		Motor_Protection.stuck_counter = 0;
	}

	// 检测3: 运动超时（基于毫秒）
	Motor_Protection.last_progress_tick = now_tick;
	Motor_Protection.movement_timeout = 0;

	Motor_Protection.last_X_pos = current_X;
	Motor_Protection.last_Y_pos = current_Y;

	return 0;
}

void Motor_Protection_EmergencyStop(void) {
	Motor_Protection.protection_triggered = 2;
	TA531_RC1_fg = 0;

	char error_msg[20];
	switch (Motor_Protection.error_type) {
	case 1:
		sprintf(error_msg, "ERR:Motor Shake!");
		break;
	case 2:
		sprintf(error_msg, "ERR:Motor Stuck!");
		break;
	case 3:
		sprintf(error_msg, "ERR:Timeout!    ");
		break;
	default:
		sprintf(error_msg, "ERR:Unknown!    ");
		break;
	}

	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 0, error_msg);
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3, "Stopping Motor..");

	Sys_tune1();
	HAL_Delay(300);
	Sys_tune1();

	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3, "Resetting...    ");
	TA531_RC1.TA531_RC_X_trg = 0;
	TA531_RC1.TA531_RC_Y_trg = 0;
	Motor_Protection_Reset();
	Motor_Protection.last_X_pos = TA531_RC1.TA531_RC_X_act;
	Motor_Protection.last_Y_pos = TA531_RC1.TA531_RC_Y_act;
	(void) WaitMotorToTargetWithProtection(3000, MOTOR_WAIT_POLL_MS, false);

	TA531_RC1.TA531_RC_Reset = 0;
	TA531_RC1.TA531_RC_Z_code = 0;
	TA531_RC1.TA531_RC_X_Mov = 0;
	TA531_RC1.TA531_RC_Y_Mov = 0;
	TA531_Lock = 0;

	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 3, "System Ready    ");
	Sys_tune1();

	Motor_Protection_Reset();
	HAL_Delay(1000);
}

/* ====================================== */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */

void Clamp_Position(int *x, int *y, bool allow_reset) {
	// 如果是复位命令，允许归0
	if (allow_reset && (*x == 0 && *y == 0)) {
		return;
	}

	// 确保范围有效（X0 < X1, Y0 < Y1）
	int x_min =
			(ScreenSz_1.DispX0_32b < ScreenSz_1.DispX1_32b) ?
					ScreenSz_1.DispX0_32b : ScreenSz_1.DispX1_32b;
	int x_max =
			(ScreenSz_1.DispX0_32b > ScreenSz_1.DispX1_32b) ?
					ScreenSz_1.DispX0_32b : ScreenSz_1.DispX1_32b;
	int y_min =
			(ScreenSz_1.DispY0_32b < ScreenSz_1.DispY1_32b) ?
					ScreenSz_1.DispY0_32b : ScreenSz_1.DispY1_32b;
	int y_max =
			(ScreenSz_1.DispY0_32b > ScreenSz_1.DispY1_32b) ?
					ScreenSz_1.DispY0_32b : ScreenSz_1.DispY1_32b;

	// 限制X坐标
	if (*x < x_min)
		*x = x_min;
	if (*x > x_max)
		*x = x_max;

	// 限制Y坐标
	if (*y < y_min)
		*y = y_min;
	if (*y > y_max)
		*y = y_max;
}

void Door_Control(void) {
	// 声明静态变量保存上次状态（6个门，包括预留）
	static uint8_t door_state_old[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	uint8_t door_changed = 0;

	// ========== 左前门控制（Y_RELAY_1 / PC2）==========
	if (TA531_Door.Door_FL != door_state_old[0]) {
		door_state_old[0] = TA531_Door.Door_FL;
		door_changed = 1;

		if (TA531_Door.Door_FL == 0) {
			HAL_GPIO_WritePin(Y_RELAY_1_GPIO_Port, Y_RELAY_1_Pin, 0);
		} else if (TA531_Door.Door_FL == 1) {
			HAL_GPIO_WritePin(Y_RELAY_1_GPIO_Port, Y_RELAY_1_Pin, 1);
		}
	}

	// ========== 右前门控制（Y_RELAY_2 / PC3）==========
	if (TA531_Door.Door_FR != door_state_old[1]) {
		door_state_old[1] = TA531_Door.Door_FR;
		door_changed = 1;

		if (TA531_Door.Door_FR == 0) {
			HAL_GPIO_WritePin(Y_RELAY_2_GPIO_Port, Y_RELAY_2_Pin, 0);
		} else if (TA531_Door.Door_FR == 1) {
			HAL_GPIO_WritePin(Y_RELAY_2_GPIO_Port, Y_RELAY_2_Pin, 1);
		}
	}

	// ========== 左后门控制（Y_RELAY_3 / PA2）==========
	if (TA531_Door.Door_RL != door_state_old[2]) {
		door_state_old[2] = TA531_Door.Door_RL;
		door_changed = 1;

		if (TA531_Door.Door_RL == 0) {
			HAL_GPIO_WritePin(Y_RELAY_3_GPIO_Port, Y_RELAY_3_Pin, 0);
		} else if (TA531_Door.Door_RL == 1) {
			HAL_GPIO_WritePin(Y_RELAY_3_GPIO_Port, Y_RELAY_3_Pin, 1);
		}
	}

	// ========== 前引擎盖控制（Y_RELAY_4 / PA3）==========
	if (TA531_Door.Door_Hood != door_state_old[3]) {
		door_state_old[3] = TA531_Door.Door_Hood;
		door_changed = 1;

		if (TA531_Door.Door_Hood == 0) {
			HAL_GPIO_WritePin(Y_RELAY_4_GPIO_Port, Y_RELAY_4_Pin, 0);
		} else if (TA531_Door.Door_Hood == 1) {
			HAL_GPIO_WritePin(Y_RELAY_4_GPIO_Port, Y_RELAY_4_Pin, 1);
		}
	}

	// ========== 后备箱控制（Y_RELAY_5 / PB1）==========
	if (TA531_Door.Door_Trunk != door_state_old[4]) {
		door_state_old[4] = TA531_Door.Door_Trunk;
		door_changed = 1;

		if (TA531_Door.Door_Trunk == 0) {
			HAL_GPIO_WritePin(Y_RELAY_5_GPIO_Port, Y_RELAY_5_Pin, 0);
		} else if (TA531_Door.Door_Trunk == 1) {
			HAL_GPIO_WritePin(Y_RELAY_5_GPIO_Port, Y_RELAY_5_Pin, 1);
		}
	}

	// ========== Y_RELAY_6 预留占位（PF6）==========
	if (TA531_Door.Door_Reserve != door_state_old[5]) {
		door_state_old[5] = TA531_Door.Door_Reserve;
		door_changed = 1;

		if (TA531_Door.Door_Reserve == 0) {
			HAL_GPIO_WritePin(Y_RELAY_6_GPIO_Port, Y_RELAY_6_Pin, 0);
		} else if (TA531_Door.Door_Reserve == 1) {
			HAL_GPIO_WritePin(Y_RELAY_6_GPIO_Port, Y_RELAY_6_Pin, 1);
		}
	}

	// ========== 发送门控制ACK报文到CAN1 ==========
	if (door_changed) {
		TSA_Door_Relay_DATA[0] = 0x1A;
		TSA_Door_Relay_DATA[1] = 0x20;
		TSA_Door_Relay_DATA[2] = 0x42;

		// Byte3: FL, FR, RL, Hood（每个2 bits）
		TSA_Door_Relay_DATA[3] = (TA531_Door.Door_FL & 0x03)
				| ((TA531_Door.Door_FR & 0x03) << 2)
				| ((TA531_Door.Door_RL & 0x03) << 4)
				| ((TA531_Door.Door_Hood & 0x03) << 6);

		// Byte4: Trunk, Reserve（每个2 bits，还有4 bits空闲）
		TSA_Door_Relay_DATA[4] = (TA531_Door.Door_Trunk & 0x03)
				| ((TA531_Door.Door_Reserve & 0x03) << 2);

		TSA_Door_Relay_DATA[5] = 0x00;
		TSA_Door_Relay_DATA[6] = 0x00;
		TSA_Door_Relay_DATA[7] = 0x00;

		// 发送到CAN1
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA_Door_Relay_Header,
				TSA_Door_Relay_DATA);
	}
}

void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	char *str1 = "SYS ERROR!";
	OLED_ShowString(OLED_I2C_ch, OLED_type, 0, 1, str1);

	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
