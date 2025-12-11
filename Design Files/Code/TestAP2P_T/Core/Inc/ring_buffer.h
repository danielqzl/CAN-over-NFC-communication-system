/*
 * Include this header file in main.h
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

/*  Marcos ------------------------------------- --*/
#define RING_SIZE 8192
#define RING_MAX_PACKET 32
#define RING_MAX_FRAME 736
#define PACKET_MAX_FRAME 23
#define FRAME_LEN 11
#define MAX_PACKET_LEN 253

/*  TypeDef ---------------------------------------*/
typedef struct {
    uint8_t *buffer;
	//uint8_t buffer[RING_SIZE];
    uint8_t packet_head;
    uint8_t packet_tail;
    //uint8_t frame_cnt[RING_MAX_PACKET];
    uint32_t frame_head;
	uint32_t frame_tail;
	uint32_t cnt;
} ring_buffer;

/* Functions prototypes ---------------------------*/
void Forward_CAN(uint8_t Data[], uint8_t num_frame);
//void exchange(ring_buffer *r_tx, ring_buffer *r_rx);
void initializeRing(ring_buffer *r);
void Load_Ring(ring_buffer *r, uint8_t n, _Bool frame_cnt_en);
void Push_FNFC(CAN_HandleTypeDef *hcan, ring_buffer *r);
void PollCANRx();
void timer_stop(uint8_t n);
void timer_start(uint8_t n);
void NFC_CAN_DataExchange();



#endif
