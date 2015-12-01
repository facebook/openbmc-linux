/*
 *  ast_formatter_h  
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */
struct formatter_yuv420_to_422 {
	u32 src;
	u32 src_virt;	
	u32 width;
	u32 height;
};
 
extern void formatter_yuv420_to_422(struct formatter_yuv420_to_422 *formatter);
extern void ast_formatter_reset(void);
