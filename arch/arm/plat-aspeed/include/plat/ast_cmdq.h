/*
 *  ast_cmdq_h  
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

//header
//CMD Type
#define CMDQ_HEADER_TYPE_BURST		0x1
#define CMDQ_HEADER_TYPE_FIRE		(0x1 << 1)
#define CMDQ_HEADER_TYPE_SUSPEND	(0x1 << 2)
#define CMDQ_HEADER_TYPE_INT		(0x1 << 3)

//Check sum

//LEN 
#define CMDQ_HEADER_LEN(x)			(x << 16)

//DEVICE Mask
#define CMDQ_HEADER_DEVICE_BB			(0x1 << 24)
#define CMDQ_HEADER_DEVICE_RLE			(0x1 << 25)
#define CMDQ_HEADER_DEVICE_ENTROPY	(0x1 << 26)
#define CMDQ_HEADER_DEVICE_BITMAP		(0x1 << 27)
#define CMDQ_HEADER_DEVICE_VMASK		(0x1 << 28)
#define CMDQ_HEADER_DEVICE_GMASK		(0x1 << 29)
#define CMDQ_HEADER_DEVICE_BULK		(0x1 << 30)

struct cmdq_ctrl_queue {
	int qsize;
	u32 *data;
};
	 
extern int ast_cmdq_enqueue(struct cmdq_ctrl_queue *en_queue);
