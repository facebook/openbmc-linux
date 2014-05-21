/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/i2c-slave-eeprom.c
* Author        : Ryan chen
* Description   : ASPEED I2C Device
*
* Copyright (C) ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* History      :
*    1. 2013/05/30 ryan chen create this file
*
********************************************************************************/
#include <linux/i2c.h>
#if defined(CONFIG_COLDFIRE)
#include <asm/arch/ast_i2c.h>
#else
#include <plat/ast_i2c.h>
#endif

#ifdef I2C_EEPROM
#define EEPROM_DBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define EEPROM_DBUG(fmt, args...)
#endif

static u8 cmd_buf[1] = {0};
static struct i2c_msg cmd_msg = {
	.addr = 0x04,
	.len = 1,
	.buf = cmd_buf,
};

//Note 10 byte data memory share for all bus slave device ...........
#define BUF_SIZE 		10
	
static u8 store_memory[BUF_SIZE] = {0x03,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
						//	RO,     RW, .................
static struct i2c_msg data_msg = {
	.addr = 0x04,
	.len = BUF_SIZE,
	.buf = store_memory,
};
static u8 mem_index = 0;
static u8 slave_stage = INIT_STAGE;

extern void i2c_slave_init(struct i2c_msg **msgs)
{
	*msgs = &cmd_msg;
}

extern void i2c_slave_xfer(i2c_slave_event_t event, struct i2c_msg **msgs)
{
	EEPROM_DBUG("[event %d] \n",event);
	switch(event) {
		case I2C_SLAVE_EVENT_START_READ:
			cmd_msg.flags = I2C_M_RD;
			data_msg.flags = I2C_M_RD;
			if(slave_stage == INIT_STAGE) {
				EEPROM_DBUG("Rt DATA_MSG [%x]\n",data_msg.buf[0]);
				slave_stage = DATA_STAGE;
				*msgs = &data_msg;
			} else {
				//CMD_STAGE
				if(cmd_msg.buf[0] != ((cmd_msg.addr << 1)|1))
					printk("START READ ADDR Error %x\n",cmd_msg.buf[0]);
				
				EEPROM_DBUG("Rt CMD_DATA_MSG data [%x]\n",store_memory[mem_index]);
				cmd_msg.buf[0] = store_memory[mem_index];
				mem_index++;
				mem_index %=BUF_SIZE;					
				slave_stage = CMD_DATA_STAGE;
				*msgs = &cmd_msg;
			}
			break;
		case I2C_SLAVE_EVENT_START_WRITE:
			EEPROM_DBUG("Rt CMD_MSG START_WRITE %x\n",cmd_msg.buf[0]);
			cmd_msg.flags = 0;
			if(cmd_msg.buf[0] != cmd_msg.addr <<1)
				printk("ERROR ADDRESS Match [%x] \n", cmd_msg.buf[0]);
			slave_stage = CMD_STAGE;
			
			*msgs = &cmd_msg;
			
			break;
				
		case I2C_SLAVE_EVENT_WRITE:
			cmd_msg.flags = 0;
			if(slave_stage == CMD_STAGE) {
				EEPROM_DBUG("w CMD = [index %x] \n",cmd_msg.buf[0]);
				mem_index = cmd_msg.buf[0];				
				mem_index %= BUF_SIZE;
				slave_stage = CMD_DATA_STAGE;
				*msgs = &cmd_msg;
			} else {
				EEPROM_DBUG("w index %d CMD_DATA [%x] \n",mem_index, cmd_msg.buf[0]);
				if(mem_index !=0)
					store_memory[mem_index] = cmd_msg.buf[0];
				mem_index++;
				mem_index %=BUF_SIZE;	
				EEPROM_DBUG("Rt CMD_DATA_MSG \n");
				*msgs = &cmd_msg;
			}
			break;
		case I2C_SLAVE_EVENT_READ:
			cmd_msg.flags = I2C_M_RD;			
			if(slave_stage == CMD_DATA_STAGE) {
				cmd_msg.buf[0] = store_memory[mem_index];
				mem_index++;
				mem_index %=BUF_SIZE;					
				EEPROM_DBUG("Rt CMD_DATA_MSG [%x]\n",cmd_msg.buf[0]);		
				*msgs = &cmd_msg;
			} else {
				EEPROM_DBUG("Rt DATA_MSG [%x]\n",data_msg.buf[0]);		
				*msgs = &data_msg;
			}			
			break;
		case I2C_SLAVE_EVENT_NACK:
			cmd_msg.flags = I2C_M_RD;			
			slave_stage = INIT_STAGE;
			*msgs = &cmd_msg;

			break;
			
		case I2C_SLAVE_EVENT_STOP:
			cmd_msg.flags = I2C_M_RD;			
			slave_stage = INIT_STAGE;
			*msgs = &cmd_msg;
			break;
	}

}
