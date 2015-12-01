/*
 *  ast_i2c_h  
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */
#include <plat/aspeed.h>

#if defined(CONFIG_ARCH_AST1070)
#define AST_I2C_DMA_SIZE 4096
#endif

/*AST I2C Register Definition */
#if defined(AST_SOC_G4)
#define AST_I2C_DMA_SIZE				0
#define AST_I2C_PAGE_SIZE 				256
#define MASTER_XFER_MODE				BUFF_MODE
#define SLAVE_XFER_MODE				BYTE_MODE
#define NUM_BUS 14

#define AST_I2C_POOL_BUFF_2048	
#define AST_I2C_BUFFER_POOL2	0x200
#define AST_I2C_BUFFER_POOL1	0x800
#elif defined(AST_SOC_G3)
#define AST_I2C_PAGE_SIZE 				256
#define MASTER_XFER_MODE				BUFF_MODE
#define SLAVE_XFER_MODE				BYTE_MODE
#define NUM_BUS 9

#define AST_I2C_POOL_BUFF_256
#define AST_I2C_BUFFER_POOL2	0x200
#elif defined(CONFIG_ARCH_AST1010)
#define AST_I2C_DMA_SIZE 				512
#define MASTER_XFER_MODE				BYTE_MODE
#define SLAVE_XFER_MODE				BYTE_MODE
#define NUM_BUS 15

#elif defined(AST_SOC_G5)
#define AST_I2C_DMA_SIZE 				4095
#define AST_I2C_PAGE_SIZE 				16
#define MASTER_XFER_MODE				INC_DMA_MODE
#define SLAVE_XFER_MODE				INC_DMA_MODE

#define NUM_BUS 8

#define AST_I2C_POOL_BUFF_16

#else
#err "NO define for I2C"
#endif


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
	DEC_DMA_XFER,
	INC_DMA_XFER
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
#define DEC_DMA_MODE	2
#define INC_DMA_MODE	3

struct ast_i2c_driver_data {
		u32		bus_clk;
		u16		dma_size;
		u8		master_dma;		//0,byte mode 1,Buffer pool mode 256 , or 2048 , 2: DMA mode
		u8		slave_dma;		//0,byte mode 1,Buffer pool mode 256 , or 2048 , 2: DMA mode
		u8 		(*request_pool_buff_page)(struct buf_page **page);
//		struct buf_page * (*request_pool_buff_page1)(u8 num);
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

extern u8 request_pool_buff_page(struct buf_page **req_page);
extern void free_pool_buff_page(struct buf_page *req_page);
