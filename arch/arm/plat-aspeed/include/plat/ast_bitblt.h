/*
 *  ast_bitblt_h  
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

struct ast_bitblt_basic {
	u8 src_format;
	u8 dest_format;
	u16 resv;
	u16 src_pitch;	
	u16 src_w;
	u16 src_h;		
	u16 src_x;
	u16 src_y;	
	u16 dest_pitch;	
	u16 dest_w;
	u16 dest_h;		
	u16 dest_x;
	u16 dest_y;	
	u32 src_base;
	u32 dest_base;	
};
 
extern void ast_h264_bitblt(struct ast_bitblt_basic *bitblt);

