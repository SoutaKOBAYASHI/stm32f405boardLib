/*
 * control_area_network.cpp
 *
 *  Created on: 2019/07/27
 *      Author: SotaKobayashi
 */

#include "control_area_network.hpp"

std::map<CAN_Interrupt* const,const std::function<void(const CanRxMsg&)>> CAN_Interrupt::can_call_functions_;

void ControlAreaNetwork::sendDataByQueue()
{
	if(transmit_queue_.size() > 0)
	{
		CAN_Transmit(CAN1, &transmit_queue_.front());
		transmit_queue_.pop();
	}
}

void ControlAreaNetwork::sendRequest_(CanTxMsg& send_msg)
{
	/*If all message boxes are pending, the data is transmitted without moving to queue.*/
	if((CAN1->TSR & CAN_TSR_TME0) && (CAN1->TSR & CAN_TSR_TME1) && (CAN1->TSR & CAN_TSR_TME2))
	{
		CAN_Transmit(CAN1 , &send_msg);
	}
	else
	{
		transmit_queue_.push(std::move(send_msg));
	}
}

void ControlAreaNetwork::sendData(const std::vector<uint8_t>& send_data_arr, const uint8_t address)
{
	std::queue<uint8_t> send_queue;
	for(auto& i : send_data_arr)
	{
		send_queue.push(i);
	}
	CanTxMsg can_tx_msg;
	can_tx_msg.StdId		= static_cast<uint32_t>(address);
	can_tx_msg.IDE		= CAN_ID_STD;
	can_tx_msg.RTR		= CAN_RTR_DATA;
	while(!send_queue.empty())
	{
		uint8_t count = 0;
		for(count = 0; count < 8 && !send_queue.empty(); ++count)
		{
			can_tx_msg.Data[count] = send_queue.front();
			if(!send_queue.empty())send_queue.pop();
		}
		can_tx_msg.DLC = count;
		sendRequest_(can_tx_msg);
	}
}

void ControlAreaNetwork::sendData(uint8_t *Data, uint8_t DataLenge, const uint8_t Address)
{
	CanTxMsg can_tx_msg;
	can_tx_msg.StdId	= static_cast<uint32_t>(Address);
	can_tx_msg.IDE		= CAN_ID_STD;
	can_tx_msg.RTR		= CAN_RTR_DATA;
	can_tx_msg.DLC		= DataLenge;
	for(uint8_t i = 0 ; (i < DataLenge) && (i < 8) ; i++)can_tx_msg.Data[i] = Data[i];

	sendRequest_(can_tx_msg);
	if(DataLenge > 8)
	{
		sendData((Data + 8), (DataLenge - 8), Address);
	}
}

void ControlAreaNetwork::sendRemote(uint8_t Address)
{
	CanTxMsg can_tx_msg;
	can_tx_msg.StdId	= static_cast<uint32_t>(Address);
	can_tx_msg.IDE		= CAN_ID_STD;
	can_tx_msg.RTR		= CAN_RTR_REMOTE;
	can_tx_msg.DLC		= 0;

	sendRequest_(can_tx_msg);
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
