/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小
#define REFEREE_TIMEOUT_MS 300U

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据

static uint8_t referee_rx_buffer[RE_RX_BUFFER_SIZE];
static uint16_t referee_rx_len;
static uint32_t referee_rx_tick;

static uint8_t JudgeParseFrame(uint8_t *buff, uint16_t data_len)
{
	uint16_t cmd_id = (uint16_t)buff[5] | ((uint16_t)buff[6] << 8);
	void *destination;
	uint16_t expected_len;
	switch (cmd_id)
	{
	case ID_game_state:
		destination = &referee_info.GameState;
		expected_len = LEN_game_state;
		break;
	case ID_game_result:
		destination = &referee_info.GameResult;
		expected_len = LEN_game_result;
		break;
	case ID_game_robot_survivors:
		destination = &referee_info.GameRobotHP;
		expected_len = LEN_game_robot_HP;
		break;
	case ID_event_data:
		destination = &referee_info.EventData;
		expected_len = LEN_event_data;
		break;
	case ID_supply_projectile_action:
		destination = &referee_info.SupplyProjectileAction;
		expected_len = LEN_supply_projectile_action;
		break;
	case ID_game_robot_state:
		destination = &referee_info.GameRobotState;
		expected_len = LEN_game_robot_state;
		break;
	case ID_power_heat_data:
		destination = &referee_info.PowerHeatData;
		expected_len = LEN_power_heat_data;
		break;
	case ID_game_robot_pos:
		destination = &referee_info.GameRobotPos;
		expected_len = LEN_game_robot_pos;
		break;
	case ID_buff_musk:
		destination = &referee_info.BuffMusk;
		expected_len = LEN_buff_musk;
		break;
	case ID_aerial_robot_energy:
		destination = &referee_info.AerialRobotEnergy;
		expected_len = LEN_aerial_robot_energy;
		break;
	case ID_robot_hurt:
		destination = &referee_info.RobotHurt;
		expected_len = LEN_robot_hurt;
		break;
	case ID_shoot_data:
		destination = &referee_info.ShootData;
		expected_len = LEN_shoot_data;
		break;
	case ID_student_interactive:
		destination = &referee_info.ReceiveData;
		expected_len = LEN_receive_data;
		break;
	default:
		return 0;
	}
	if (data_len != expected_len)
		return 0;
	memcpy(destination, buff + DATA_Offset, data_len);
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);
	referee_info.CmdID = cmd_id;
	return 1;
}

/* 保留跨 DMA/IDLE 回调的半帧；只有完整且长度匹配的有效帧才更新数据并喂狗。 */
static void JudgeReadData(const uint8_t *buff, uint16_t size)
{
	for (uint16_t i = 0; i < size; ++i)
	{
		referee_rx_buffer[referee_rx_len++] = buff[i];
		while (referee_rx_len > 0)
		{
			uint16_t consumed = 1;
			if (referee_rx_buffer[0] == REFEREE_SOF)
			{
				if (referee_rx_len < LEN_HEADER)
					break;
				if (Verify_CRC8_Check_Sum(referee_rx_buffer, LEN_HEADER) == TRUE)
				{
					uint16_t data_len = (uint16_t)referee_rx_buffer[1] |
						((uint16_t)referee_rx_buffer[2] << 8);
					if (data_len <= RE_RX_BUFFER_SIZE - LEN_HEADER - LEN_CMDID - LEN_TAIL)
					{
						uint16_t frame_len = data_len + LEN_HEADER + LEN_CMDID + LEN_TAIL;
						if (referee_rx_len < frame_len)
							break;
						if (Verify_CRC16_Check_Sum(referee_rx_buffer, frame_len) == TRUE)
						{
							if (JudgeParseFrame(referee_rx_buffer, data_len))
								DaemonReload(referee_daemon);
							consumed = frame_len;
						}
					}
				}
			}
			referee_rx_len -= consumed;
			memmove(referee_rx_buffer, referee_rx_buffer + consumed, referee_rx_len);
		}
	}
}

static void RefereeRxCallback(void)
{
	uint32_t now = HAL_GetTick();
	if ((uint32_t)(now - referee_rx_tick) >= REFEREE_TIMEOUT_MS)
		referee_rx_len = 0;
	referee_rx_tick = now;
	JudgeReadData(referee_usart_instance->recv_buff, referee_usart_instance->recv_len);
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
	LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = DAEMON_COUNT_FROM_MS(REFEREE_TIMEOUT_MS),
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(115);
}
