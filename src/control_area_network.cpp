/*
 * control_area_network.cpp
 *
 *  Created on: 2019/07/27
 *      Author: SotaKobayashi
 */

#include "control_area_network.hpp"

std::map<CAN_Interrupt* const,const std::function<void(const CanRxMsg&)>> CAN_Interrupt::can_call_functions_;

void ControlAreaNetwork::sendData(uint8_t *Data, uint8_t DataLenge, uint8_t Address)
{
	CanTxMsg can_tx_msg;
	can_tx_msg.StdId		= static_cast<uint32_t>(Address);
	can_tx_msg.IDE		= CAN_ID_STD;
	can_tx_msg.RTR		= CAN_RTR_DATA;
	can_tx_msg.DLC		= DataLenge;
	for(uint8_t i = 0 ; i < DataLenge ; i++)can_tx_msg.Data[i] = Data[i];

	/*If all message boxes are pending, the data is transmitted without moving to queue.*/
	if((CAN1->TSR & CAN_TSR_TME0) && (CAN1->TSR & CAN_TSR_TME1) && (CAN1->TSR & CAN_TSR_TME2))
	{
		CAN_Transmit(CAN1 , &can_tx_msg);
	}
	else
	{
		transmit_queue_.push(std::move(can_tx_msg));
	}
}

void ControlAreaNetwork::sendRemote(uint8_t Address)
{
	CanTxMsg can_tx_msg;
	can_tx_msg.StdId	= static_cast<uint32_t>(Address);
	can_tx_msg.IDE		= CAN_ID_STD;
	can_tx_msg.RTR		= CAN_RTR_REMOTE;
	can_tx_msg.DLC		= 0;

	/*If all message boxes are empty, the data is transmitted without moving to queue.*/
	if((CAN1->TSR & CAN_TSR_TME0) && (CAN1->TSR & CAN_TSR_TME1) && (CAN1->TSR & CAN_TSR_TME2))
	{
		CAN_Transmit(CAN1 , &can_tx_msg);
	}
	else
	{
		transmit_queue_.push(std::move(can_tx_msg));
	}

	return;
}

extern "C"
{
	void CAN1_RX0_IRQHandler()
	{
		CAN_Interrupt::callback();
	}

	void CAN1_TX_IRQHandler()
	{
		ControlAreaNetwork::sendDataByQueue();
	}
}
