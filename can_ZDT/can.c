#include "can.h"

/**********************************************************
***	ZDT_X57步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

uint8_t can_RxFlag = 0;
/* 测试 CAN 链路用：实际接收帧数与发送邮箱不足次数。 */
volatile uint32_t can_rx_frame_count = 0;
volatile uint32_t can_tx_no_mailbox_count = 0;
volatile uint32_t can_tx_ok_count = 0;
volatile uint32_t can_tx_failed_count = 0;
volatile uint32_t can_tx_timeout_count = 0;
volatile uint8_t can_tx_last_cmd_ok = 0U;
uint8_t test4 = 0;

#define CAN_TX_WAIT_LIMIT 500000U

__IO CAN_t can = {0};
volatile int64_t arm_motor_position_raw[2] = {0, 0};
volatile uint32_t arm_motor_position_count[2] = {0U, 0U};

/**
	* @brief   CAN1_RX0接收中断
	* @param   无
	* @retval  无
	*/
void CAN1_RX0_IRQHandler (void)
{
	// 接收一包数据
	CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)(&can.CAN_RxMsg));
	++can_rx_frame_count;
    /* Emm 0x36 position response: function, sign, 32-bit value, checksum. */
    if (can.CAN_RxMsg.IDE == CAN_Id_Extended && can.CAN_RxMsg.RTR == CAN_RTR_Data &&
        can.CAN_RxMsg.DLC == 7U &&
        (can.CAN_RxMsg.ExtId == 0x500U || can.CAN_RxMsg.ExtId == 0x600U) &&
        can.CAN_RxMsg.Data[0] == 0x36U && can.CAN_RxMsg.Data[1] <= 1U &&
        can.CAN_RxMsg.Data[6] == 0x6BU) {
        uint8_t index = (can.CAN_RxMsg.ExtId == 0x500U) ? 0U : 1U;
        uint32_t raw = ((uint32_t)can.CAN_RxMsg.Data[2] << 24) |
                       ((uint32_t)can.CAN_RxMsg.Data[3] << 16) |
                       ((uint32_t)can.CAN_RxMsg.Data[4] << 8) |
                       (uint32_t)can.CAN_RxMsg.Data[5];
        arm_motor_position_raw[index] = can.CAN_RxMsg.Data[1] ? -(int64_t)raw : (int64_t)raw;
        ++arm_motor_position_count[index];
    }


//	if(can.CAN_RxMsg.Data[1] == 0x9F && can.CAN_RxMsg.ExtId == 0x00000200){
//		can.CAN_RxMsg.Data[1] = 0x00;
//		can_RxFlag = 1;
//	}

	// 一帧数据接收完成，置位帧标志位
	can.rxFrameFlag = true;
	// 清除中断挂起标志位
	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
}

/**
	* @brief   CAN发送多个字节
	* @param   无
	* @retval  无
	*/
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    can_tx_last_cmd_ok = 0U;
	__IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
	uint8_t mailbox;
	uint8_t tx_status;
	uint32_t wait_count;

	// 除去ID地址和功能码后的数据长度
	j = len - 2;

	// 发送数据
	while(i < j)
	{
		// 数据个数
		k = j - i;

		// 填充缓存
		can.CAN_TxMsg.StdId = 0x00;
		can.CAN_TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
		can.CAN_TxMsg.Data[0] = cmd[1];
		can.CAN_TxMsg.IDE = CAN_Id_Extended;
		can.CAN_TxMsg.RTR = CAN_RTR_Data;

		// 小于8字节命令
		if(k < 8)
		{
			for(l=0; l < k; l++,i++) { can.CAN_TxMsg.Data[l + 1] = cmd[i + 2]; } can.CAN_TxMsg.DLC = k + 1;
		}
		// 大于8字节命令，分包发送，每包数据最多发送8个字节
		else
		{
			for(l=0; l < 7; l++,i++) { can.CAN_TxMsg.Data[l + 1] = cmd[i + 2]; } can.CAN_TxMsg.DLC = 8;
		}

		// 发送数据
		mailbox = CAN_Transmit(CAN1, (CanTxMsg *)(&can.CAN_TxMsg));
		if (mailbox == CAN_TxStatus_NoMailBox) {
            ++can_tx_no_mailbox_count;
			return;
        }

		/* Wait for an ACK/result so later commands cannot silently fill all mailboxes. */
		wait_count = CAN_TX_WAIT_LIMIT;
		do {
			tx_status = CAN_TransmitStatus(CAN1, mailbox);
			if (tx_status == CAN_TxStatus_Ok) {
				++can_tx_ok_count;
				break;
			}
			if (tx_status == CAN_TxStatus_Failed) {
				++can_tx_failed_count;
				return;
			}
		} while (--wait_count != 0U);

		if (wait_count == 0U) {
			CAN_CancelTransmit(CAN1, mailbox);
			++can_tx_timeout_count;
			return;
		}

		// 记录发送的第几包的数据
		++packNum;
	}
    can_tx_last_cmd_ok = 1U;
}

uint8_t can_GetRxFlag(void)
{
	if (can_RxFlag == 1)			//如果标志位为1
	{
		can_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}
