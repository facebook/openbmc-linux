/*
 *  ast-snoop_h
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

struct ast_snoop_channel {
	u8	snoop_ch;
	u8	snoop_port;	
	u8	snoop_data;	
};

struct ast_snoop {
	struct ast_snoop_channel	*snoop_ch0;
	struct ast_snoop_channel	*snoop_ch1;	
};

struct ast_snoop_dma_channel {
	u8	snoop_ch;	
	u8	snoop_port;
	u8	snoop_mask;
	u8	snoop_mode;
	u8	snoop_index;
	u32	dma_virt;	
	dma_addr_t dma_addr;	
	u16 dma_size;
};

extern int ast_snoop_init(struct ast_snoop *snoop);
extern void ast_snoop_dma_init(struct ast_snoop_dma_channel *ast_dma_ch);



