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
 * @param state 机器人当前状态
 * @param x 机器人当前x坐标
 * @param y 机器人当前y坐标
 * @param self_yaw 机器人当前偏航角度
 * @param my_xor 校验异或值
 * @param tail 帧尾(0x55)
 *******************************************************************************/
typedef struct _robot_msg_
{
	uint8_t header_;
	uint8_t mode_;
	float self_pitch_;
	uint8_t my_xor_;
	uint8_t tail_;
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
	uint8_t header_;
	uint8_t cmd_;
	uint16_t len_;
	float x_;
	float y_;
	uint8_t is_hit_;
	uint8_t my_xor_;
	uint8_t tail_;
} PlanMsg;

/*******************************************************************************
 * @union RobotArray
 * @brief 机器人状态数据包联合体，用于字节流与结构体转换
 *******************************************************************************/
union RobotArray
{
	RobotMsg msg_;
	uint8_t array_[sizeof(RobotMsg)];
};

/*******************************************************************************
 * @union PlanArray
 * @brief 规划指令数据包联合体，用于字节流与结构体转换
 *******************************************************************************/
union PlanArray
{
	PlanMsg msg_;
	uint8_t array_[sizeof(PlanMsg)];
};

#pragma pack()
#endif	// SERIAL_PACKET_H
