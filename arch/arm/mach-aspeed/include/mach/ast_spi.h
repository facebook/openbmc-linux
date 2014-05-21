/*
 *  ast_spi_h  
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

struct ast_spi_driver_data {
		u32 	(*get_div)(u32 max_speed_hz);
		u16		num_chipselect;
};
