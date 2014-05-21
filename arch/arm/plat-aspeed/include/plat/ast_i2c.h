/*
 *  ast_i2c_h  
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

//I2C MEMORY Device state machine
typedef enum i2c_slave_stage {
		INIT_STAGE,
        CMD_STAGE,
        CMD_DATA_STAGE,
        DATA_STAGE
} stage;

typedef enum i2c_xfer_mode {
        BYTE_XFER,
        BUFF_XFER,			
        DMA_XFER
} i2c_xfer_mode_t;

//1. usage flag , 2 size,	3. request address
struct buf_page
{
	u8 flag; //0:free to usage, 1: used
	u8 page_no; //for AST2400 usage
	u16 page_size;
	u32 page_addr;
	u32 page_addr_point;
};

typedef enum i2c_slave_event_e {
        I2C_SLAVE_EVENT_START_READ,
        I2C_SLAVE_EVENT_READ,			
        I2C_SLAVE_EVENT_START_WRITE,
        I2C_SLAVE_EVENT_WRITE,     
        I2C_SLAVE_EVENT_NACK,
        I2C_SLAVE_EVENT_STOP
} i2c_slave_event_t;

#define BYTE_MODE	0
#define BUFF_MODE	1
#define DMA_MODE	2

struct ast_i2c_driver_data {
		void __iomem		*reg_gr;
		u32		bus_clk;
		u8		master_dma;		//0,byte mode 1,Buffer pool mode 256 , or 2048 , 2: DMA mode
		u8		slave_dma;		//0,byte mode 1,Buffer pool mode 256 , or 2048 , 2: DMA mode
		u8 		(*request_pool_buff_page)(struct buf_page **page);
		void 	(*free_pool_buff_page)(struct buf_page *page);
		unsigned char	*buf_pool;
		void (*slave_xfer)(i2c_slave_event_t event, struct i2c_msg **msgs);
		void (*slave_init)(struct i2c_msg **msgs);
		u32 (*get_i2c_clock)(void);
};

#ifdef CONFIG_AST_I2C_SLAVE_MODE
extern void i2c_slave_init(struct i2c_msg **msgs);
extern void i2c_slave_xfer(i2c_slave_event_t event, struct i2c_msg **msgs);
#endif