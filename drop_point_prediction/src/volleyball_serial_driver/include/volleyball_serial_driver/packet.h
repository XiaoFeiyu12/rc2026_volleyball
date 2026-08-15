/*******************************************************************************
 * @file packet.h
 * @brief 串口数据包定义，包含上下位机通信的数据结构
 *******************************************************************************/

#ifndef SERIAL_PACKET_H
#define SERIAL_PACKET_H

// STL
#include <cstdint>

#pragma pack(1)

/*******************************************************************************
 * @struct RobotMsg
 * @brief 机器人状态数据包结构体（下位机→上位机）
 * @param header 帧头(0xAA)
 * @param mode 机器人当前运动模式
 * @param x 机器人当前x坐标
 * @param y 机器人当前y坐标
 * @param self_yaw 机器人当前偏航角度
 * @param my_xor 校验异或值
 * @param tail 帧尾(0x55)
 *******************************************************************************/
typedef struct _robot_msg_
{
	uint8_t header;
	uint8_t mode;
	float x;
	float y;
	float self_yaw;
	uint8_t my_xor;
	uint8_t tail;
} RobotMsg;

/*******************************************************************************
 * @struct PlanMsg
 * @brief 规划指令数据包结构体（上位机→下位机）
 * @param header 帧头(0xAA)
 * @param cmd 指令类型
 * @param len 数据长度
 * @param x 目标x坐标
 * @param y 目标y坐标
 * @param self_yaw 目标偏航角度
 * @param landing_time 预计落点时间（含传输延迟补偿）
 * @param my_xor 校验异或值
 * @param tail 帧尾(0x55)
 *******************************************************************************/
typedef struct _plan_msg_
{
	uint8_t header;
	uint8_t cmd;
	uint16_t len;
	float x;
	float y;
	float self_yaw;
	float landing_time;
	uint8_t my_xor;
	uint8_t tail;
} PlanMsg;

/*******************************************************************************
 * @union RobotArray
 * @brief 机器人状态数据包联合体，用于字节流与结构体转换
 *******************************************************************************/
union RobotArray
{
	RobotMsg msg;
	uint8_t array[sizeof(RobotMsg)];
};

/*******************************************************************************
 * @union PlanArray
 * @brief 规划指令数据包联合体，用于字节流与结构体转换
 *******************************************************************************/
union PlanArray
{
	PlanMsg msg;
	uint8_t array[sizeof(PlanMsg)];
};

#pragma pack()
#endif	// SERIAL_PACKET_H