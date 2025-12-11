#include "main.h"
#include "rfal_platform.h"
#include "rfal_rf.h"
#include "rfal_nfc.h"

extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern CAN_HandleTypeDef hcan1;
extern uint32_t TxMailbox;
extern uint8_t TxData[8];
extern uint32_t total_CAN_Rx;


extern TIM_HandleTypeDef htim2, htim3, htim4;
extern uint8_t timerEN;
extern uint32_t time;
extern uint32_t maxTime2, maxTime3, maxTime4;

extern ring_buffer Ring_FNFC;

ReturnCode demoTransceiveBlocking( uint8_t *txBuf, uint16_t txBufSize, uint8_t **rxBuf, uint16_t **rcvLen, uint32_t fwt );

void initializeRing(ring_buffer *r)
{
	r->buffer = (uint8_t *)malloc(RING_SIZE);
	r->frame_head = r->frame_tail = 0;
	r->packet_head = r->packet_tail = 0;
	r->cnt = 0;
	//memset(r->frame_cnt,0,RING_MAX_PACKET);
}

void Push_FNFC(CAN_HandleTypeDef *hcan, ring_buffer *r)
{
	uint8_t frame_head_next, packet_head_next;
	uint32_t index = r->packet_head * 256 + r->frame_head * FRAME_LEN;

	if(total_CAN_Rx >= 3000)
	{
		timerEN = 0;
	}
	else
	{
		timerEN = 0xFF;
	}

	if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0U)
	{
		return;
	}
	total_CAN_Rx++;
	frame_head_next = r->frame_head + 1;
	packet_head_next = r->packet_head;
	if(frame_head_next == PACKET_MAX_FRAME)
	{
		frame_head_next = 0;
		packet_head_next++;
		if(packet_head_next == RING_MAX_PACKET)
		{
			packet_head_next = 0;
		}
	}
	// Ring full
	if(r->cnt >= RING_MAX_FRAME)
	{
		return;
	}

	// Load Data
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, r->buffer+index+3);
	//HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	uint8_t header[4];
	header[0] = ((RxHeader.IDE<<5) | (RxHeader.RTR<<4) | (RxHeader.DLC));

	if(RxHeader.IDE)
	{
		// Extended ID  // unimplemented
		header[1] = RxHeader.ExtId>>16;
		header[2] = RxHeader.ExtId>>8;
		header[3] = RxHeader.ExtId;
	}
	else
	{
		//Standard ID
		header[1] = RxHeader.StdId>>8;
		header[2] = RxHeader.StdId;
	}
	memcpy(r->buffer+index, header, RxHeader.IDE?4:3);

	r->cnt ++;
	r->frame_head = frame_head_next;
	r->packet_head = packet_head_next;
}

void Forward_CAN(uint8_t Data[], uint8_t num_frame)
{
	uint8_t cntr=0, index;
	while(cntr < num_frame)
	{
		index = cntr * FRAME_LEN;
		TxHeader.IDE = (Data[index] & 0x30)>>5;
		TxHeader.RTR = (Data[index] & 0x10)>>4;
		TxHeader.DLC = Data[index] & 0x0F;
		TxHeader.TransmitGlobalTime = DISABLE;
		if(TxHeader.IDE)
		{
			// Extended ID  unimplemented
			TxHeader.ExtId = ((Data[index+1]<<16) | (Data[index+2]<<8) | Data[index+3]);
			TxHeader.StdId = 0;
		}
		else
		{
			//Standard ID
			TxHeader.StdId = ((Data[index+1]<<8) | Data[index+2]);
			TxHeader.ExtId = 0;
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader,Data+index+3, &TxMailbox) == HAL_OK)
		{
			cntr++;
		}
	}
}


void PollCANRx()
{
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
	{
		Push_FNFC(&hcan1,&Ring_FNFC);
	}
}

void NFC_CAN_DataExchange()
{
	uint16_t   *rxLen;
	uint8_t    *rxData;
	uint32_t num_frame = 0;
	uint8_t empty = 0x00;
	uint8_t packet_tail_next = Ring_FNFC.packet_tail;
	ReturnCode err = RFAL_ERR_NONE;

	while(err == RFAL_ERR_NONE)
	{
		__HAL_CAN_DISABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

		// Nothing to transmit
		if(Ring_FNFC.cnt == 0)
		{
			err = demoTransceiveBlocking(&empty, 1, &rxData, &rxLen, RFAL_FWT_NONE);
		}
		else{
			uint32_t tx_len = 0;
			uint32_t tx_index = Ring_FNFC.packet_tail * 256;

			if(Ring_FNFC.cnt >= PACKET_MAX_FRAME) //The ring has 1 full packet
			{
				num_frame = PACKET_MAX_FRAME;
				if(++packet_tail_next == RING_MAX_PACKET)
					packet_tail_next = 0;
			}
			else{
				num_frame = Ring_FNFC.cnt;
			}

			tx_len = num_frame * FRAME_LEN;
			Ring_FNFC.cnt -= num_frame;
			Ring_FNFC.frame_head = 0;
			Ring_FNFC.packet_tail = packet_tail_next;
			err = demoTransceiveBlocking((Ring_FNFC.buffer+tx_index), tx_len, &rxData, &rxLen, RFAL_FWT_NONE);
		}

		if( err != RFAL_ERR_NONE )
		{	break;  }

		if(memcmp(rxData, &empty, 1) != 0)
		{
			uint8_t rx_frame = *rxLen / FRAME_LEN;
			Forward_CAN(rxData, rx_frame);
			//total_frame += rx_frame;
		}
		__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}

	platformLog("\r\n Device removed.\r\n");
	//platformLog("Total CAN Rx: %u \r\n", total_CAN_Rx);
	//platformLog("timer: %u %u %u\r\n", maxTime2, maxTime3, maxTime4);
	//maxTime2 = maxTime3 = maxTime4 = 0;
	//total_CAN_Rx = 0;
}

/*!
 *****************************************************************************
 * \brief Demo Blocking Transceive
 *
 * Helper function to send data in a blocking manner via the rfalNfc module
 *
 * \warning A protocol transceive handles long timeouts (several seconds),
 * transmission errors and retransmissions which may lead to a long period of
 * time where the MCU/CPU is blocked in this method.
 * This is a demo implementation, for a non-blocking usage example please
 * refer to the Examples available with RFAL
 *
 * \param[in]  txBuf      : data to be transmitted
 * \param[in]  txBufSize  : size of the data to be transmited
 * \param[out] rxData     : location where the received data has been placed
 * \param[out] rcvLen     : number of data bytes received
 * \param[in]  fwt        : FWT to be used (only for RF frame interface,
 *                                          otherwise use RFAL_FWT_NONE)
 *
 *
 *  \return ERR_PARAM     : Invalid parameters
 *  \return ERR_TIMEOUT   : Timeout error
 *  \return ERR_FRAMING   : Framing error detected
 *  \return ERR_PROTO     : Protocol error detected
 *  \return RFAL_ERR_NONE : No error, activation successful
 *
 *****************************************************************************
 */
ReturnCode demoTransceiveBlocking( uint8_t *txBuf, uint16_t txBufSize, uint8_t **rxData, uint16_t **rcvLen, uint32_t fwt )
{
    ReturnCode err;

    err = rfalNfcDataExchangeStart( txBuf, txBufSize, rxData, rcvLen, fwt );

    if( err == RFAL_ERR_NONE )
    {
        do{
            rfalNfcWorker();
            err = rfalNfcDataExchangeGetStatus();
        }
        while( err == RFAL_ERR_BUSY );
    }

    return err;
}

void timer_start(uint8_t n)
{
	switch(n)
	{
		case 2:
			htim2.Instance->CNT = 0;
			HAL_TIM_Base_Start(&htim2);
			break;
		case 3:
			htim3.Instance->CNT = 0;
			HAL_TIM_Base_Start(&htim3);
			break;
		case 4:
			htim4.Instance->CNT = 0;
			HAL_TIM_Base_Start(&htim4);
			break;
		default:;
	}
}

void timer_stop(uint8_t n)
{
	switch(n)
	{
		case 2:
			HAL_TIM_Base_Stop(&htim2);
			time = __HAL_TIM_GetCounter(&htim2);
			if( (time > maxTime2) && (timerEN))
			//if( time > maxTime2 )
			{
				maxTime2 = time;
			}
			htim2.Instance->CNT = 0;
			break;

		case 3:
			HAL_TIM_Base_Stop(&htim3);
			time = __HAL_TIM_GetCounter(&htim3);
			if( (time > maxTime3) && (timerEN))
			//if( time > maxTime3 )
			{
				maxTime3 = time;
			}
			htim3.Instance->CNT = 0;
			break;

		case 4:
			HAL_TIM_Base_Stop(&htim4);
			time = __HAL_TIM_GetCounter(&htim4);
			if( (time > maxTime4) && (timerEN))
			//if( time > maxTime4 )
			{
				maxTime4 = time;
			}
			htim4.Instance->CNT = 0;
			break;

		default:;
	}

}

