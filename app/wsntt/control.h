
#ifndef __9000H_CONTROL_H
#define __9000H_CONTROL_H

typedef enum {
	SEND_JOIN = 0x01,
	SEND_ALL_OPEN = 0x03,
	SEND_ALL_CLOSE = 0x04,
	SEND_TOGGLE = 0x05,
	SEND_ID = 0x11,
	SEND_ACK_OPEN = 0x13,
        SEND_ACK_CLOSE = 0x14
} CMD_9000H;

#endif