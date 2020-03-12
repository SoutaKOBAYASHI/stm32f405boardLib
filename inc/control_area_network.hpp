
#ifndef CONTROL_AREA_NETWORK_HPP__
#define CONTROL_AREA_NETWORK_HPP__

#include <array>
#include <vector>
#include <map>
#include <queue>
#include <utility>
#include <functional>
#include <queue>

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

enum class CAN_TimeQuanta : uint8_t
{
	tq1		= 0x00,
	tq2		= 0x01,
	tq3		= 0x02,
	tq4		= 0x03,
	tq5		= 0x04,
	tq6		= 0x05,
	tq7		= 0x06,
	tq8		= 0x07,
	tq9		= 0x08,
	tq10	= 0x09,
	tq11	= 0x0A,
	tq12	= 0x0B,
	tq13	= 0x0C,
	tq14	= 0x0D,
	tq15	= 0x0E,
	tq16	= 0x0F
};

class ControlAreaNetwork
{
private:
	static inline std::queue<CanTxMsg> transmit_queue_ = {};
public:
	static void sendDataByQueue();
	template<size_t S>
	void sendData(const std::array<uint8_t, S> &transmit_data_arr, uint8_t Address)
	{
		static_assert( !(S > 8), "Size of SendDataArray has to be less than eight.");

		CanTxMsg can_tx_msg;
		can_tx_msg.StdId	= static_cast<uint32_t>(Address);
		can_tx_msg.IDE		= CAN_ID_STD;
		can_tx_msg.RTR		= CAN_RTR_DATA;
		can_tx_msg.DLC		= S;
		uint8_t count = 0;
		for(auto& i : transmit_data_arr)
		{
			can_tx_msg.Data[count] = i;
			++count;
		}

		if((CAN1->TSR & CAN_TSR_TME0) && (CAN1->TSR & CAN_TSR_TME1) && (CAN1->TSR & CAN_TSR_TME2))
		{
			CAN_Transmit(CAN1 , &can_tx_msg);
		}
		else
		{
			transmit_queue_.push(std::move(can_tx_msg));
		}
	}

	void sendData(uint8_t *Data, uint8_t DataLenge, uint8_t Address);

	void sendRemote(uint8_t Address);
};

template<uint8_t setAddress, CAN_TimeQuanta BS1_timeQuanta = CAN_TimeQuanta::tq4, CAN_TimeQuanta BS2_timeQuanta = CAN_TimeQuanta::tq2, uint8_t prescaler = 6>
class CAN_Initialize
{
public:
	CAN_Initialize()
	{
		static_assert(static_cast<uint8_t>(BS2_timeQuanta) <= static_cast<uint8_t>(CAN_TimeQuanta::tq8), "BS2_timeQuanta has to be less than 8.");

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_11 | GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
		GPIO_Init(GPIOA , &GPIO_InitStructure);

		GPIO_PinAFConfig(GPIOA , GPIO_PinSource11 , GPIO_AF_CAN1);
		GPIO_PinAFConfig(GPIOA , GPIO_PinSource12 , GPIO_AF_CAN1);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 , ENABLE);

		CAN_InitTypeDef CAN_InitStructure;
		CAN_StructInit(&CAN_InitStructure);
		CAN_InitStructure.CAN_Mode 		= CAN_Mode_Normal;
		CAN_InitStructure.CAN_TXFP 		= ENABLE;
		CAN_InitStructure.CAN_Prescaler = prescaler;
		CAN_InitStructure.CAN_BS1 = static_cast<uint8_t>(BS1_timeQuanta);
		CAN_InitStructure.CAN_BS2 = static_cast<uint8_t>(BS2_timeQuanta);
		CAN_Init(CAN1 , &CAN_InitStructure);

		CAN_FilterInitTypeDef CAN_FilterInitStructure;
		CAN_FilterInitStructure.CAN_FilterNumber			= 0;

		/*ID-Mask Mode*/

		CAN_FilterInitStructure.CAN_FilterMode 				= CAN_FilterMode_IdMask;
		CAN_FilterInitStructure.CAN_FilterScale				= CAN_FilterScale_32bit;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh		= 0xFFE0;
		CAN_FilterInitStructure.CAN_FilterMaskIdLow			= 0x0000;
		CAN_FilterInitStructure.CAN_FilterIdHigh			= static_cast<uint16_t>(static_cast<uint16_t>(setAddress) << 5);
		CAN_FilterInitStructure.CAN_FilterIdLow				= 0x0000;

		/*END*/

		CAN_FilterInitStructure.CAN_FilterFIFOAssignment	= CAN_FilterFIFO0;
		CAN_FilterInitStructure.CAN_FilterActivation		= ENABLE;
		CAN_FilterInit(&CAN_FilterInitStructure);

		CAN_ITConfig(CAN1 , CAN_IT_FMP0 , ENABLE);

		/*Initial interrupting for RX0*/
		NVIC_InitTypeDef rx0_nvic_init_struct;
		rx0_nvic_init_struct.NVIC_IRQChannel						= CAN1_RX0_IRQn;
		rx0_nvic_init_struct.NVIC_IRQChannelPreemptionPriority	= 0;
		rx0_nvic_init_struct.NVIC_IRQChannelCmd					= ENABLE;
		NVIC_Init(&rx0_nvic_init_struct);

		/*Initial interrupting  complete TX*/
		NVIC_InitTypeDef tx_nvic_init_struct;
		tx_nvic_init_struct.NVIC_IRQChannel						= CAN1_TX_IRQn;
		tx_nvic_init_struct.NVIC_IRQChannelPreemptionPriority	= 0;
		tx_nvic_init_struct.NVIC_IRQChannelCmd					= ENABLE;
		NVIC_Init(&tx_nvic_init_struct);
	}

	ControlAreaNetwork can_interface;
}; // namespase CAN_Initialize

class CAN_Interrupt
{
public:
	CAN_Interrupt() = delete;

	CAN_Interrupt(const std::function<void(const CanRxMsg&)>&& addFunc)
	{
		can_call_functions_.insert(std::make_pair(this, addFunc));
	}

	static void callback()
	{
		CanRxMsg can_rx_msg_struct;
		CAN_Receive(CAN1, CAN_FIFO0, &can_rx_msg_struct);
		for(auto i : can_call_functions_)i.second(can_rx_msg_struct);
	}


	virtual ~CAN_Interrupt()
	{
		can_call_functions_.erase(this);
	}
private:
	static std::map<CAN_Interrupt* const,const std::function<void(const CanRxMsg&)>> can_call_functions_;
};
#endif
