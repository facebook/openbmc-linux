// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton NPCM7xx SMB Controller driver
 *
 * Copyright (C) 2018 Nuvoton Technologies tali.perry@nuvoton.com
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk/nuvoton.h>
#include <linux/crc8.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define I2C_VERSION "0.0.3"

enum smb_mode {
	SMB_SLAVE = 1,
	SMB_MASTER
};

/*
 * External SMB Interface driver xfer indication values, which indicate status
 * of the bus.
 */
enum smb_state_ind {
	SMB_NO_STATUS_IND = 0,
	SMB_SLAVE_RCV_IND = 1,
	SMB_SLAVE_XMIT_IND = 2,
	SMB_SLAVE_XMIT_MISSING_DATA_IND = 3,
	SMB_SLAVE_RESTART_IND = 4,
	SMB_SLAVE_DONE_IND = 5,
	SMB_MASTER_DONE_IND = 6,
	SMB_NO_DATA_IND = 7,
	SMB_NACK_IND = 8,
	SMB_BUS_ERR_IND = 9,
	SMB_WAKE_UP_IND = 10,
	SMB_MASTER_PEC_ERR_IND = 11,
	SMB_BLOCK_BYTES_ERR_IND = 12,
	SMB_SLAVE_PEC_ERR_IND = 13,
	SMB_SLAVE_RCV_MISSING_DATA_IND = 14,
};

// SMBus Operation type values
enum smb_oper {
	SMB_NO_OPER = 0,
	SMB_WRITE_OPER = 1,
	SMB_READ_OPER = 2
};

// SMBus Bank (FIFO mode)
enum smb_bank {
	SMB_BANK_0 = 0,
	SMB_BANK_1 = 1
};

// Internal SMB states values (for the SMB module state machine).
enum smb_state {
	SMB_DISABLE = 0,
	SMB_IDLE,
	SMB_MASTER_START,
	SMB_SLAVE_MATCH,
	SMB_OPER_STARTED,
	SMB_REPEATED_START,
	SMB_STOP_PENDING
};

// Module supports setting multiple own slave addresses:
enum smb_addr {
	SMB_SLAVE_ADDR1 = 0,
	SMB_SLAVE_ADDR2,
	SMB_SLAVE_ADDR3,
	SMB_SLAVE_ADDR4,
	SMB_SLAVE_ADDR5,
	SMB_SLAVE_ADDR6,
	SMB_SLAVE_ADDR7,
	SMB_SLAVE_ADDR8,
	SMB_SLAVE_ADDR9,
	SMB_SLAVE_ADDR10,
	SMB_GC_ADDR,
	SMB_ARP_ADDR
};

// global regs
static struct regmap *gcr_regmap;
static struct regmap *clk_regmap;

#define NPCM_I2CSEGCTL  0xE4
#define NPCM_SECCNT	0x68
#define NPCM_CNTR25M	0x6C
#define I2CSEGCTL_VAL	0x0333F000

// Common regs
#define NPCM_SMBSDA			0x000
#define NPCM_SMBST			0x002
#define NPCM_SMBCST			0x004
#define NPCM_SMBCTL1			0x006
#define NPCM_SMBADDR1			0x008
#define NPCM_SMBCTL2			0x00A
#define NPCM_SMBADDR2			0x00C
#define NPCM_SMBCTL3			0x00E
#define NPCM_SMBCST2			0x018
#define NPCM_SMBCST3			0x019
#define SMB_VER				0x01F

// BANK 0 regs
#define NPCM_SMBADDR3			0x010
#define NPCM_SMBADDR7			0x011
#define NPCM_SMBADDR4			0x012
#define NPCM_SMBADDR8			0x013
#define NPCM_SMBADDR5			0x014
#define NPCM_SMBADDR9			0x015
#define NPCM_SMBADDR6			0x016
#define NPCM_SMBADDR10			0x017

// SMBADDR array: because the addr regs are sprincled all over the address space
const int  NPCM_SMBADDR[10] = {NPCM_SMBADDR1, NPCM_SMBADDR2, NPCM_SMBADDR3,
			       NPCM_SMBADDR4, NPCM_SMBADDR5, NPCM_SMBADDR6,
			       NPCM_SMBADDR7, NPCM_SMBADDR8, NPCM_SMBADDR9,
			       NPCM_SMBADDR10};

#define NPCM_SMBCTL4			0x01A
#define NPCM_SMBCTL5			0x01B
#define NPCM_SMBSCLLT			0x01C // SCL Low Time
#define NPCM_SMBFIF_CTL			0x01D // FIFO Control
#define NPCM_SMBSCLHT			0x01E // SCL High Time

// BANK 1 regs
#define NPCM_SMBFIF_CTS			0x010 // FIFO Control
#define NPCM_SMBTXF_CTL			0x012 // Tx-FIFO Control
#define NPCM_SMBT_OUT			0x014 // Bus T.O.
#define NPCM_SMBPEC			0x016 // PEC Data
#define NPCM_SMBTXF_STS			0x01A // Tx-FIFO Status
#define NPCM_SMBRXF_STS			0x01C // Rx-FIFO Status
#define NPCM_SMBRXF_CTL			0x01E // Rx-FIFO Control

// NPCM_SMBST reg fields
#define NPCM_SMBST_XMIT			BIT(0)
#define NPCM_SMBST_MASTER		BIT(1)
#define NPCM_SMBST_NMATCH		BIT(2)
#define NPCM_SMBST_STASTR		BIT(3)
#define NPCM_SMBST_NEGACK		BIT(4)
#define NPCM_SMBST_BER			BIT(5)
#define NPCM_SMBST_SDAST		BIT(6)
#define NPCM_SMBST_SLVSTP		BIT(7)

// NPCM_SMBCST reg fields
#define NPCM_SMBCST_BUSY		BIT(0)
#define NPCM_SMBCST_BB			BIT(1)
#define NPCM_SMBCST_MATCH		BIT(2)
#define NPCM_SMBCST_GCMATCH		BIT(3)
#define NPCM_SMBCST_TSDA		BIT(4)
#define NPCM_SMBCST_TGSCL		BIT(5)
#define NPCM_SMBCST_MATCHAF		BIT(6)
#define NPCM_SMBCST_ARPMATCH		BIT(7)

// NPCM_SMBCTL1 reg fields
#define NPCM_SMBCTL1_START		BIT(0)
#define NPCM_SMBCTL1_STOP		BIT(1)
#define NPCM_SMBCTL1_INTEN		BIT(2)
#define NPCM_SMBCTL1_EOBINTE		BIT(3)
#define NPCM_SMBCTL1_ACK		BIT(4)
#define NPCM_SMBCTL1_GCMEN		BIT(5)
#define NPCM_SMBCTL1_NMINTE		BIT(6)
#define NPCM_SMBCTL1_STASTRE		BIT(7)

// RW1S fields (inside a RW reg):
#define NPCM_SMBCTL1_RWS_FIELDS	  (NPCM_SMBCTL1_START | NPCM_SMBCTL1_STOP | \
				   NPCM_SMBCTL1_ACK)
// NPCM_SMBADDR reg fields
#define NPCM_SMBADDR_ADDR		GENMASK(6, 0)
#define NPCM_SMBADDR_SAEN		BIT(7)

// NPCM_SMBCTL2 reg fields
#define SMBCTL2_ENABLE			BIT(0)
#define SMBCTL2_SCLFRQ6_0		GENMASK(7, 1)

// NPCM_SMBCTL3 reg fields
#define SMBCTL3_SCLFRQ8_7		GENMASK(1, 0)
#define SMBCTL3_ARPMEN			BIT(2)
#define SMBCTL3_IDL_START		BIT(3)
#define SMBCTL3_400K_MODE		BIT(4)
#define SMBCTL3_BNK_SEL			BIT(5)
#define SMBCTL3_SDA_LVL			BIT(6)
#define SMBCTL3_SCL_LVL			BIT(7)

// NPCM_SMBCST2 reg fields
#define NPCM_SMBCST2_MATCHA1F		BIT(0)
#define NPCM_SMBCST2_MATCHA2F		BIT(1)
#define NPCM_SMBCST2_MATCHA3F		BIT(2)
#define NPCM_SMBCST2_MATCHA4F		BIT(3)
#define NPCM_SMBCST2_MATCHA5F		BIT(4)
#define NPCM_SMBCST2_MATCHA6F		BIT(5)
#define NPCM_SMBCST2_MATCHA7F		BIT(5)
#define NPCM_SMBCST2_INTSTS		BIT(7)

// NPCM_SMBCST3 reg fields
#define NPCM_SMBCST3_MATCHA8F		BIT(0)
#define NPCM_SMBCST3_MATCHA9F		BIT(1)
#define NPCM_SMBCST3_MATCHA10F		BIT(2)
#define NPCM_SMBCST3_EO_BUSY		BIT(7)

// NPCM_SMBCTL4 reg fields
#define SMBCTL4_HLDT			GENMASK(5, 0)
#define SMBCTL4_LVL_WE			BIT(7)

// NPCM_SMBCTL5 reg fields
#define SMBCTL5_DBNCT			GENMASK(3, 0)

// NPCM_SMBFIF_CTS reg fields
#define NPCM_SMBFIF_CTS_RXF_TXE		BIT(1)
#define NPCM_SMBFIF_CTS_RFTE_IE		BIT(3)
#define NPCM_SMBFIF_CTS_CLR_FIFO	BIT(6)
#define NPCM_SMBFIF_CTS_SLVRSTR		BIT(7)

// NPCM_SMBTXF_CTL reg fields
#ifdef SMB_CAPABILITY_32B_FIFO
#define NPCM_SMBTXF_CTL_TX_THR		GENMASK(5, 0)
#else
#define NPCM_SMBTXF_CTL_TX_THR		GENMASK(4, 0)
#endif
#define NPCM_SMBTXF_CTL_THR_TXIE	BIT(6)

// NPCM_SMBT_OUT reg fields
#define NPCM_SMBT_OUT_TO_CKDIV		GENMASK(5, 0)
#define NPCM_SMBT_OUT_T_OUTIE		BIT(6)
#define NPCM_SMBT_OUT_T_OUTST		BIT(7)

// NPCM_SMBTXF_STS reg fields
#ifdef SMB_CAPABILITY_32B_FIFO
#define NPCM_SMBTXF_STS_TX_BYTES	GENMASK(5, 0)
#else
#define NPCM_SMBTXF_STS_TX_BYTES	GENMASK(4, 0)
#endif
#define NPCM_SMBTXF_STS_TX_THST		BIT(6)

// NPCM_SMBRXF_STS reg fields
#ifdef SMB_CAPABILITY_32B_FIFO
#define NPCM_SMBRXF_STS_RX_BYTES	GENMASK(5, 0)
#else
#define NPCM_SMBRXF_STS_RX_BYTES	GENMASK(4, 0)
#endif
#define NPCM_SMBRXF_STS_RX_THST		BIT(6)

// NPCM_SMBFIF_CTL reg fields
#define NPCM_SMBFIF_CTL_FIFO_EN		BIT(4)

// NPCM_SMBRXF_CTL reg fields
// Note: on the next HW version of this module, this HW is about to switch to
//	 32 bytes FIFO. This size will be set using a config.
//	 on current version 16 bytes FIFO is set using a define
#ifdef SMB_CAPABILITY_32B_FIFO
#define NPCM_SMBRXF_CTL_RX_THR		GENMASK(5, 0)
#define NPCM_SMBRXF_CTL_THR_RXIE	BIT(6)
#define NPCM_SMBRXF_CTL_LAST_PEC	BIT(7)
#define SMBUS_FIFO_SIZE			32
#else
#define NPCM_SMBRXF_CTL_RX_THR		GENMASK(4, 0)
#define NPCM_SMBRXF_CTL_LAST_PEC	BIT(5)
#define NPCM_SMBRXF_CTL_THR_RXIE	BIT(6)
#define SMBUS_FIFO_SIZE			16
#endif

// SMB_VER reg fields
#define SMB_VER_VERSION			GENMASK(6, 0)
#define SMB_VER_FIFO_EN			BIT(7)

// stall/stuck timeout
const unsigned int DEFAULT_STALL_COUNT =	25;

// Data abort timeout
const unsigned int ABORT_TIMEOUT =	 1000;

// SMBus spec. values in KHZ
const unsigned int SMBUS_FREQ_MIN = 10;
const unsigned int SMBUS_FREQ_MAX = 1000;
const unsigned int SMBUS_FREQ_100KHZ = 100;
const unsigned int SMBUS_FREQ_400KHZ = 400;
const unsigned int SMBUS_FREQ_1MHZ = 1000;

// SCLFRQ min/max field values
const unsigned int SCLFRQ_MIN = 10;
const unsigned int SCLFRQ_MAX = 511;

// SCLFRQ field position
#define SCLFRQ_0_TO_6		GENMASK(6, 0)
#define SCLFRQ_7_TO_8		GENMASK(8, 7)

// SMB Maximum Retry Trials (on Bus Arbitration Loss)
const unsigned int SMB_RETRY_MAX_COUNT = 2;
const unsigned int SMB_NUM_OF_ADDR = 10;

// for logging:
#define NPCM_I2C_EVENT_START	BIT(0)
#define NPCM_I2C_EVENT_STOP	BIT(1)
#define NPCM_I2C_EVENT_ABORT	BIT(2)
#define NPCM_I2C_EVENT_WRITE	BIT(3)
#define NPCM_I2C_EVENT_READ	BIT(4)
#define NPCM_I2C_EVENT_BER	BIT(5)
#define NPCM_I2C_EVENT_NACK	BIT(6)
#define NPCM_I2C_EVENT_TO	BIT(7)
#define NPCM_I2C_EVENT_EOB	BIT(8)

#define NPCM_I2C_EVENT_LOG(event)   (bus->event_log |= event)

#define SMB_RECOVERY_SUPPORT

// slave mode: if end device reads more data than available, ask issuer or
// request for more data:
#define SMB_WRAP_AROUND_BUFFER

// Status of one SMBus module
struct npcm_i2c {
	struct i2c_adapter	adap;
	struct device		*dev;
	unsigned char __iomem	*reg;
	spinlock_t		lock;   /* IRQ synchronization */
	struct completion	cmd_complete;
	int			irq;
	int			cmd_err;
	struct i2c_msg		*msgs;
	int			msgs_num;
	int			num;
	u32			apb_clk;
	enum smb_state		state;
	enum smb_oper		operation;
	enum smb_mode		master_or_slave;
	enum smb_state_ind	stop_ind;
	u8			dest_addr;
	u8			*rd_buf;
	u16			rd_size;
	u16			rd_ind;
	u8			*wr_buf;
	u16			wr_size;
	u16			wr_ind;
	bool			fifo_use;
	u8			threshold_fifo;

	// PEC bit mask per slave address.
	//		1: use PEC for this address,
	//		0: do not use PEC for this address
	u16			PEC_mask;
	bool			PEC_use;
	u8			crc_data;
	bool			read_block_use;
	u8			retry_count;
	u8			int_cnt;
	u32			event_log;
	u32			clk_period_us;
	u32			int_time_stamp[2];
};

static inline void _npcm7xx_get_time_stamp(u32 *time_quad0, u32 *time_quad1)
{
	u32 seconds, seconds_last;
	u32 ref_clock;

	regmap_read(clk_regmap, NPCM_SECCNT, &seconds_last);

	do {
		regmap_read(clk_regmap, NPCM_SECCNT, &seconds);
		regmap_read(clk_regmap, NPCM_CNTR25M, &ref_clock);
		regmap_read(clk_regmap, NPCM_SECCNT, &seconds_last);
	} while (seconds_last != seconds);

	*time_quad0 = ref_clock;
	*time_quad1 = seconds;
}

#define EXT_CLOCK_FREQUENCY_MHZ 25
#define CNTR25M_ACCURECY	EXT_CLOCK_FREQUENCY_MHZ  // minimum accurecy

// Function:	 _npcm7xx_delay_relative
// Parameters:
//		 us_delay -  number of microseconds to delay since t0_time.
//				  if zero: no delay.
//
//		t0_time	      - start time , to measure time from.
// get a time stamp, delay us_delay from it. If us_delay has already passed
// since the time stamp , then no delay is executed. returns the time elapsed
// since t0_time

static inline u32 _npcm7xx_delay_relative(u32 us_delay, u32 t0_time0,
					  u32 t0_time1)
{
	u32 t1_time_0, t1_time_1;
	u32 time_elapsed;
	u32 minimum_delay = (us_delay * EXT_CLOCK_FREQUENCY_MHZ)
		+ CNTR25M_ACCURECY;

	// this is equivalent to microSec/0.64 + minimal tic length.
	do {
		_npcm7xx_get_time_stamp(&t1_time_0, &t1_time_1);
		time_elapsed = ((EXT_CLOCK_FREQUENCY_MHZ * 1000000) *
				(t1_time_1 - t0_time1)) +
				(t1_time_0 - t0_time0);
	} while (time_elapsed < minimum_delay);

	// return elapsed time
	return (u32)(time_elapsed / EXT_CLOCK_FREQUENCY_MHZ);
}

static inline void npcm_smb_select_bank(struct npcm_i2c *bus,
					enum smb_bank bank)
{
	if (bus->fifo_use)
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL3) & ~SMBCTL3_BNK_SEL) |
			 FIELD_PREP(SMBCTL3_BNK_SEL, bank),
			 bus->reg + NPCM_SMBCTL3);
}

DECLARE_CRC8_TABLE(npcm7xx_crc8);

static u8 npcm_smb_calc_crc8(u8 crc_data, u8 data)
{
	crc_data = crc8(npcm7xx_crc8, &data, 1, crc_data);
	return crc_data;
}

static void npcm_smb_calc_PEC(struct npcm_i2c *bus, u8 data)
{
	if (bus->PEC_use)
		bus->crc_data = npcm_smb_calc_crc8(bus->crc_data, data);
}

static inline void npcm_smb_wr_byte(struct npcm_i2c *bus, u8 data)
{
	iowrite8(data, bus->reg + NPCM_SMBSDA);
	npcm_smb_calc_PEC(bus, data);
}

static inline void npcm_smb_rd_byte(struct npcm_i2c *bus, u8 *data)
{
	*data = ioread8(bus->reg + NPCM_SMBSDA);
	npcm_smb_calc_PEC(bus, *data);
}

static inline u8 npcm_smb_get_PEC(struct npcm_i2c *bus)
{
	if (bus->PEC_use)
		return bus->crc_data;
	else
		return 0;
}

static inline void npcm_smb_write_PEC(struct npcm_i2c *bus)
{
	if (bus->PEC_use) {
		// get PAC value and write to the bus:
		npcm_smb_wr_byte(bus, npcm_smb_get_PEC(bus));
	}
}

//
//  NPCM7XX SMB module allows writing to SCL and SDA pins directly
//  without the need to change muxing of pins.
//  This feature will be used for recovery sequences i.e.
//
static void npcm_smb_set_SCL(struct i2c_adapter *_adap, int level)
{
#ifdef SMB_CAPABILITY_FORCE_SCL_SDA
	unsigned long flags;
	struct npcm_i2c *bus = container_of(_adap, struct npcm_i2c, adap);

	// Select Bank 0 to access NPCM_SMBCTL4
	spin_lock_irqsave(&bus->lock, flags);
	npcm_smb_select_bank(bus, SMB_BANK_0);

	// Set SCL_LVL, SDA_LVL bits as Read/Write (R/W)
	iowrite8(ioread8(bus->reg + NPCM_SMBCTL4) | SMBCTL4_LVL_WE,
		 bus->reg + NPCM_SMBCTL4);

	// Set level
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL3)
		& ~SMBCTL3_SCL_LVL) | FIELD_PREP(SMBCTL3_SCL_LVL,
		level), bus->reg + NPCM_SMBCTL3);

	// Set SCL_LVL, SDA_LVL bits as Read Only (RO)
	iowrite8(ioread8(bus->reg + NPCM_SMBCTL4)
		 & ~SMBCTL4_LVL_WE, bus->reg + NPCM_SMBCTL4);

	// Return to Bank 1
	npcm_smb_select_bank(bus, SMB_BANK_1);
	spin_unlock_irqrestore(&bus->lock, flags);
#endif
}

static int npcm_smb_get_SCL(struct i2c_adapter *_adap)
{
	unsigned long flags;
	unsigned int ret = 0;
	struct npcm_i2c *bus = container_of(_adap, struct npcm_i2c, adap);

	// Select Bank 0 to access NPCM_SMBCTL4
	spin_lock_irqsave(&bus->lock, flags);
	npcm_smb_select_bank(bus, SMB_BANK_0);

	// Get SCL level
	ret = FIELD_GET(SMBCTL3_SCL_LVL,  ioread8(bus->reg + NPCM_SMBCTL3));

	// Return to Bank 1
	npcm_smb_select_bank(bus, SMB_BANK_1);
	spin_unlock_irqrestore(&bus->lock, flags);
	return ret;
}

static int npcm_smb_get_SDA(struct i2c_adapter *_adap)
{
	unsigned long flags;
	unsigned int ret = 0;
	struct npcm_i2c *bus = container_of(_adap, struct npcm_i2c, adap);

	// Select Bank 0 to access NPCM_SMBCTL4
	spin_lock_irqsave(&bus->lock, flags);
	npcm_smb_select_bank(bus, SMB_BANK_0);

	// Get SDA level
	ret = FIELD_GET(SMBCTL3_SDA_LVL,  ioread8(bus->reg + NPCM_SMBCTL3));

	// Return to Bank 1
	npcm_smb_select_bank(bus, SMB_BANK_1);
	spin_unlock_irqrestore(&bus->lock, flags);
	return ret;
}

static inline u16 npcm_smb_get_index(struct npcm_i2c *bus)
{
	u16 index = 0;

	if (bus->operation == SMB_READ_OPER)
		index = bus->rd_ind;
	else if (bus->operation == SMB_WRITE_OPER)
		index = bus->wr_ind;

	return index;
}

// quick protocol:
static inline bool npcm_smb_is_quick(struct npcm_i2c *bus)
{
	if (bus->wr_size == 0 && bus->rd_size == 0)
		return true;
	return false;
}

static void npcm_smb_disable(struct npcm_i2c *bus)
{
	int i;

	// select bank 0 for SMB addresses
	npcm_smb_select_bank(bus, SMB_BANK_0);

	// Slave Addresses Removal
	for (i = SMB_SLAVE_ADDR1; i < SMB_NUM_OF_ADDR; i++)
		iowrite8(0, bus->reg + NPCM_SMBADDR[i]);

	// select bank 0 for SMB addresses
	npcm_smb_select_bank(bus, SMB_BANK_1);

	// Disable module.
	iowrite8(ioread8(bus->reg + NPCM_SMBCTL2) & ~SMBCTL2_ENABLE,
		 bus->reg + NPCM_SMBCTL2);

	// Set module disable
	bus->state = SMB_DISABLE;
}

static void npcm_smb_enable(struct npcm_i2c *bus)
{
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL2) | SMBCTL2_ENABLE),
		 bus->reg + NPCM_SMBCTL2);
}

// enable\disable end of busy (EOB) interrupt
static inline void npcm_smb_eob_int(struct npcm_i2c *bus, bool enable)
{
	if (enable) {
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) |
			 NPCM_SMBCTL1_EOBINTE)  & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);
	} else {
		iowrite8(ioread8(bus->reg + NPCM_SMBCTL1) &
			 ~NPCM_SMBCTL1_EOBINTE & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);

		// Clear EO_BUSY pending bit:
		iowrite8(ioread8(bus->reg + NPCM_SMBCST3) |
			 NPCM_SMBCST3_EO_BUSY, bus->reg + NPCM_SMBCST3);
	}
}

static inline bool npcm_smb_tx_fifo_full(struct npcm_i2c *bus)
{
	// check if TX FIFO full:
	return (bool)FIELD_GET(NPCM_SMBTXF_STS_TX_THST,
			       ioread8(bus->reg + NPCM_SMBTXF_STS));
}

static inline bool npcm_smb_rx_fifo_full(struct npcm_i2c *bus)
{
	// check if RX FIFO full:
	return (bool)FIELD_GET(NPCM_SMBRXF_STS_RX_THST,
			       ioread8(bus->reg + NPCM_SMBRXF_STS));
}

static inline void npcm_smb_clear_tx_fifo(struct npcm_i2c *bus)
{
	// clear TX FIFO:
	iowrite8(ioread8(bus->reg + NPCM_SMBTXF_STS) |
		 NPCM_SMBTXF_STS_TX_THST,
		 bus->reg + NPCM_SMBTXF_STS);
}

static inline void npcm_smb_clear_rx_fifo(struct npcm_i2c *bus)
{
	// clear RX FIFO:
	iowrite8(ioread8(bus->reg + NPCM_SMBRXF_STS) |
			 NPCM_SMBRXF_STS_RX_THST,
			 bus->reg + NPCM_SMBRXF_STS);
}

static void npcm_smb_int_enable(struct npcm_i2c *bus, bool enable)
{
	if (enable)
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) |
			 NPCM_SMBCTL1_INTEN) & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);
	else
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) &
			 ~NPCM_SMBCTL1_INTEN) & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);
}

static inline void npcm_smb_master_start(struct npcm_i2c *bus)
{
	NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_START);

	iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) | NPCM_SMBCTL1_START) &
		 ~(NPCM_SMBCTL1_STOP | NPCM_SMBCTL1_ACK),
		 bus->reg + NPCM_SMBCTL1);
}

static inline void npcm_smb_master_stop(struct npcm_i2c *bus)
{
	NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_STOP);

	// override HW issue: SMBus may fail to supply stop condition in Master
	// Write operation.
	// Need to delay at least 5 us from the last int, before issueing a stop
	_npcm7xx_delay_relative(5, bus->int_time_stamp[0],
				bus->int_time_stamp[1]);

	iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) | NPCM_SMBCTL1_STOP) &
		 ~(NPCM_SMBCTL1_START | NPCM_SMBCTL1_ACK),
		 bus->reg + NPCM_SMBCTL1);

	if (bus->fifo_use) {
		npcm_smb_select_bank(bus, SMB_BANK_1);

		if (bus->operation == SMB_READ_OPER)
			npcm_smb_clear_rx_fifo(bus);
		else
			npcm_smb_clear_tx_fifo(bus);

		iowrite8(ioread8(bus->reg + NPCM_SMBFIF_CTS) |
				 NPCM_SMBFIF_CTS_SLVRSTR |
				 NPCM_SMBFIF_CTS_RXF_TXE,
				 bus->reg + NPCM_SMBFIF_CTS);

		iowrite8(0, bus->reg + NPCM_SMBTXF_CTL);
	}
}

static inline void npcm_smb_abort_data(struct npcm_i2c *bus)
{
	unsigned int timeout = ABORT_TIMEOUT;
	u8 data;

	NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_ABORT);
	// Generate a STOP condition
	npcm_smb_master_stop(bus);
	npcm_smb_rd_byte(bus, &data);

	// Clear NEGACK, STASTR and BER bits
	iowrite8(NPCM_SMBST_STASTR | NPCM_SMBST_NEGACK |
		NPCM_SMBST_BER, bus->reg + NPCM_SMBST);

	// Wait till STOP condition is generated
	while (FIELD_GET(NPCM_SMBCTL1_STOP, ioread8(bus->reg + NPCM_SMBCTL1))) {
		timeout--;
		if (!FIELD_GET(NPCM_SMBCTL1_STOP,
			       ioread8(bus->reg + NPCM_SMBCTL1)))
			break;
		if (timeout <= 1) {
			dev_err(bus->dev, "%s, abort timeout!\n", __func__);
			break;
		}
	}
}

static inline void npcm_smb_stall_after_start(struct npcm_i2c *bus, bool stall)
{
	if (stall)
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) |
			 NPCM_SMBCTL1_STASTRE)  & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);
	else
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) &
			 ~NPCM_SMBCTL1_STASTRE)  & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);
}

static inline void npcm_smb_nack(struct npcm_i2c *bus)
{
	if (bus->rd_ind < (bus->rd_size - 1))
		dev_info(bus->dev,
			 "\tNACK err bus%d, SA=0x%x, rd(%d\%d), op=%d st=%d\n",
			 bus->num, bus->dest_addr, bus->rd_ind, bus->rd_size,
			 bus->operation, bus->state);
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) | NPCM_SMBCTL1_ACK) &
		 ~(NPCM_SMBCTL1_STOP | NPCM_SMBCTL1_START),
		 bus->reg + NPCM_SMBCTL1);
}

static void npcm_smb_reset(struct npcm_i2c *bus)
{
	// Save NPCM_SMBCTL1 relevant bits. It is being cleared when the
	// module is disabled
	u8 smbctl1 = ioread8(bus->reg + NPCM_SMBCTL1) & (NPCM_SMBCTL1_GCMEN
						      | NPCM_SMBCTL1_INTEN
						      | NPCM_SMBCTL1_NMINTE);

	// Disable the SMB module
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL2) & ~SMBCTL2_ENABLE),
		 bus->reg + NPCM_SMBCTL2);

	// Enable the SMB module
	npcm_smb_enable(bus);

	// Restore NPCM_SMBCTL1 status
	iowrite8(smbctl1 & ~NPCM_SMBCTL1_RWS_FIELDS, bus->reg + NPCM_SMBCTL1);

	// Reset driver status
	bus->state = SMB_IDLE;
	//
	// Configure FIFO disabled mode so slave will not use fifo
	//  (master will set it on if supported)
	iowrite8(ioread8(bus->reg + NPCM_SMBFIF_CTL) &
		~NPCM_SMBFIF_CTL_FIFO_EN,
		bus->reg + NPCM_SMBFIF_CTL);
	bus->fifo_use = false;
}

static inline bool npcm_smb_is_master(struct npcm_i2c *bus)
{
	return (bool)FIELD_GET(NPCM_SMBST_MASTER,
			       ioread8(bus->reg + NPCM_SMBST));
}

static int npcm_smb_master_abort(struct npcm_i2c *bus)
{
	int ret = -(EIO);

	// Only current master is allowed to issue Stop Condition
	if (npcm_smb_is_master(bus)) {
		npcm_smb_abort_data(bus);
		ret = 0;
	}

	npcm_smb_reset(bus);

	return ret;
}

static void npcm_smb_callback(struct npcm_i2c *bus,
			      enum smb_state_ind op_status, u16 info)
{
	struct i2c_msg *msgs = bus->msgs;
	int msgs_num = bus->msgs_num;

	switch (op_status) {
	case SMB_MASTER_DONE_IND:
	// Master transaction finished and all transmit bytes were sent
	// info: number of bytes actually received after the Master
	//	receive operation (if Master didn't issue receive it
	//	should be 0)
	// Notify that not all data was received on Master or Slave
	// info:
	//	on receive: number of actual bytes received
	//	when PEC is used even if 'info' is the expected number
	//	of bytes, it means that PEC error occurred.
	{
		if (msgs[0].flags & I2C_M_RD)
			msgs[0].len = info;
		else if (msgs_num == 2 && msgs[1].flags & I2C_M_RD)
			msgs[1].len = info;

		bus->cmd_err = 0;
		complete(&bus->cmd_complete);
	}
	break;

	case SMB_NO_DATA_IND:
	// Notify that not all data was received on Master or Slave
	// info:
	//on receive: number of actual bytes received
	//	when PEC is used even if 'info' is the expected number
	//	of bytes,it means that PEC error occurred.
	{
		if (msgs[0].flags & I2C_M_RD)
			msgs[0].len = info;
		else if (msgs_num == 2 && msgs[1].flags & I2C_M_RD)
			msgs[1].len = info;

		bus->cmd_err = -EFAULT;
		complete(&bus->cmd_complete);
	}
	break;
	case SMB_NACK_IND:
		// MASTER transmit got a NAK before transmitting all bytes
		// info: number of transmitted bytes
		bus->cmd_err = -EAGAIN;
		complete(&bus->cmd_complete);

		break;
	case SMB_BUS_ERR_IND:
		// Bus error
		// info: has no meaning
		bus->cmd_err = -EIO;
		complete(&bus->cmd_complete);
		break;
	case SMB_WAKE_UP_IND:
		// SMBus wake up
		// info: has no meaning
		break;
	default:
		break;
	}
}

static u32 npcm_smb_get_fifo_fullness(struct npcm_i2c *bus)
{
	if (bus->operation == SMB_WRITE_OPER)
		return FIELD_GET(NPCM_SMBTXF_STS_TX_BYTES,
				 ioread8(bus->reg + NPCM_SMBTXF_STS));
	else if (bus->operation == SMB_READ_OPER)
		return FIELD_GET(NPCM_SMBRXF_STS_RX_BYTES,
				 ioread8(bus->reg + NPCM_SMBRXF_STS));
	return 0;
}

static void npcm_smb_write_to_fifo(struct npcm_i2c *bus, u16 max_bytes_to_send)
{
	// Fill the FIFO, while the FIFO is not full and there are more bytes to
	// write
	while ((max_bytes_to_send--) && (SMBUS_FIFO_SIZE -
					 npcm_smb_get_fifo_fullness(bus))) {
		// write the data
		if (bus->wr_ind < bus->wr_size) {
			if (bus->PEC_use &&
			    (bus->wr_ind + 1 == bus->wr_size) &&
			    (bus->rd_size == 0 ||
			     bus->master_or_slave == SMB_SLAVE)) {
				// Master send PEC in write protocol, Slave send
				// PEC in read protocol.
				npcm_smb_write_PEC(bus);
				bus->wr_ind++;
			} else {
				npcm_smb_wr_byte(bus,
						 bus->wr_buf[bus->wr_ind++]);
			}
		} else {
#ifdef SMB_WRAP_AROUND_BUFFER
			// We're out of bytes. Ask the higher level for
			// more bytes. Let it know that driver
			// used all its' bytes

			npcm_smb_clear_tx_fifo(bus);

			// Reset state for the remaining bytes transaction
			bus->state = SMB_SLAVE_MATCH;

			// Notify upper layer of transaction completion
			npcm_smb_callback(bus, SMB_SLAVE_XMIT_MISSING_DATA_IND,
					  bus->wr_ind);

			iowrite8(NPCM_SMBST_SDAST, bus->reg + NPCM_SMBST);
#else
			npcm_smb_wr_byte(bus, 0xFF);
#endif
		}
	}
}

// configure the FIFO before using it. If nread is -1 RX FIFO will not be
// configured. same for	nwrite
static void npcm_smb_set_fifo(struct npcm_i2c *bus, int nread, int nwrite)
{
	if (!bus->fifo_use)
		return;
	npcm_smb_select_bank(bus, SMB_BANK_1);
	npcm_smb_clear_tx_fifo(bus);
	npcm_smb_clear_rx_fifo(bus);

	// configure RX FIFO
	if (nread > 0) {
		// clear LAST bit:
		iowrite8(ioread8(bus->reg + NPCM_SMBRXF_CTL) &
					(~NPCM_SMBRXF_CTL_LAST_PEC),
					bus->reg + NPCM_SMBRXF_CTL);

		if (nread > SMBUS_FIFO_SIZE)
			iowrite8((ioread8(bus->reg + NPCM_SMBRXF_CTL) &
				~NPCM_SMBRXF_CTL_RX_THR)
				| FIELD_PREP(NPCM_SMBRXF_CTL_RX_THR,
				SMBUS_FIFO_SIZE), bus->reg + NPCM_SMBRXF_CTL);
		else
			iowrite8((ioread8(bus->reg + NPCM_SMBRXF_CTL) &
					  ~NPCM_SMBRXF_CTL_RX_THR) |
					  FIELD_PREP(NPCM_SMBRXF_CTL_RX_THR,
						     (u8)(nread)),
				 bus->reg + NPCM_SMBRXF_CTL);

		if (nread <= SMBUS_FIFO_SIZE && !bus->read_block_use)
			iowrite8(ioread8(bus->reg + NPCM_SMBRXF_CTL) |
				 NPCM_SMBRXF_CTL_LAST_PEC,
				 bus->reg + NPCM_SMBRXF_CTL);
	}

	// configure TX FIFO
	if (nwrite > 0) {
		if (nwrite > SMBUS_FIFO_SIZE)
			// data to send is more then FIFO size.
			// Configure the FIFO int to be mid of FIFO.
			iowrite8(NPCM_SMBTXF_CTL_THR_TXIE |
				(SMBUS_FIFO_SIZE / 2),
				bus->reg + NPCM_SMBTXF_CTL);
		else if (nwrite > (SMBUS_FIFO_SIZE / 2) &&
			 bus->wr_ind != 0)
			// wr_ind != 0 means that this is not the first
			// write. since int is in the mid of FIFO, only
			// half of the fifo is empty.
			// Continue to configure the FIFO int to be mid
			// of FIFO.
			iowrite8(NPCM_SMBTXF_CTL_THR_TXIE |
				 (SMBUS_FIFO_SIZE / 2),
				 bus->reg + NPCM_SMBTXF_CTL);
		else
			// This is the either first write (wr_ind = 0)
			// and data to send is less or equal to FIFO
			// size.
			// Or this is the last write and data to send
			// is less or equal half FIFO size.
			// In both cases disable the FIFO threshold int.
			// The next int will happen after the FIFO will
			// get empty.
			iowrite8(0, bus->reg + NPCM_SMBTXF_CTL);
		npcm_smb_clear_tx_fifo(bus);
	}
}

static void npcm_smb_read_from_fifo(struct npcm_i2c *bus, u8 bytes_in_fifo)
{
	while (bytes_in_fifo--) {
		// Keep read data
		u8 data = ioread8(bus->reg + NPCM_SMBSDA);

		npcm_smb_calc_PEC(bus, data);
		if (bus->rd_ind < bus->rd_size) {
			bus->rd_buf[bus->rd_ind++] = data;
			if (bus->rd_ind == 1 && bus->read_block_use)
				// First byte indicates length in block protocol
				bus->rd_size = data;
		}
	}
}

static void npcm_smb_master_fifo_read(struct npcm_i2c *bus)
{
	u16 rcount;
	u8 fifo_bytes;
	enum smb_state_ind ind = SMB_MASTER_DONE_IND;

	rcount = bus->rd_size - bus->rd_ind;

	// In order not to change the RX_TRH during transaction (we found that
	// this might be problematic if it takes too much time to read the FIFO)
	//  we read the data in the following way. If the number of bytes to
	// read == FIFO Size + C (where C < FIFO Size)then first read C bytes
	// and in the next int we read rest of the data.
	if (rcount < (2 * SMBUS_FIFO_SIZE) && rcount > SMBUS_FIFO_SIZE)
		fifo_bytes = (u8)(rcount - SMBUS_FIFO_SIZE);
	else
		fifo_bytes = npcm_smb_get_fifo_fullness(bus);

	if (rcount - fifo_bytes == 0) {
		// last byte is about to be read - end of transaction.
		// Stop should be set before reading last byte.
		npcm_smb_eob_int(bus, true);
		npcm_smb_master_stop(bus);
		npcm_smb_read_from_fifo(bus, fifo_bytes);

		if (npcm_smb_get_PEC(bus) != 0)
			ind = SMB_MASTER_PEC_ERR_IND;
		bus->state = SMB_STOP_PENDING;
		bus->stop_ind = ind;

	} else {
		npcm_smb_read_from_fifo(bus, fifo_bytes);
		rcount = bus->rd_size - bus->rd_ind;
		npcm_smb_set_fifo(bus, rcount, -1);
	}
}

static void npcm_smb_int_master_handler_write(struct npcm_i2c *bus)
{
	u16 wcount;

	NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_WRITE);
	if (bus->fifo_use)
		npcm_smb_clear_tx_fifo(bus);

	// Master write operation - last byte handling
	if (bus->wr_ind == bus->wr_size) {
		if (bus->fifo_use && npcm_smb_get_fifo_fullness(bus) > 0)
	// No more bytes to send (to add to the FIFO), however the FIFO is not
	// empty yet. It is still in the middle of tx. Currently there's nothing
	// to do except for waiting to the end of the tx.
	// We will get an int when the FIFO will get empty.
			return;

		if (bus->rd_size == 0) {
			// all bytes have been written, in a pure wr operation
			npcm_smb_eob_int(bus, true);

			// Issue a STOP condition on the bus
			npcm_smb_master_stop(bus);
			// Clear SDA Status bit (by writing dummy byte)
			npcm_smb_wr_byte(bus, 0xFF);

			bus->state = SMB_STOP_PENDING;
			bus->stop_ind = SMB_MASTER_DONE_IND;
		} else {
			// last write-byte written on previous int - need to
			// restart & send slave address
			if (bus->PEC_use && !bus->read_block_use &&
			    !npcm_smb_is_quick(bus))
			    // PEC is used but the protocol is not block read
			    // then we add extra bytes for PEC support
				bus->rd_size += 1;

			if (bus->fifo_use) {
				if (bus->rd_size == 1 || bus->read_block_use) {
					// SMBus Block read transaction.
					iowrite8(0, bus->reg + NPCM_SMBTXF_CTL);
					iowrite8(1, bus->reg + NPCM_SMBRXF_CTL);
				}
			}

			npcm_smb_set_fifo(bus, bus->rd_size, -1);

			// Generate (Repeated) Start upon next write to SDA
			npcm_smb_master_start(bus);

			if (bus->rd_size == 1)

	// Receiving one byte only - stall after successful completion of send
	// address byte. If we NACK here, and slave doesn't ACK the address, we
	// might unintentionally NACK the next multi-byte read

				npcm_smb_stall_after_start(bus, true);

			// send the slave address in read direction
			npcm_smb_wr_byte(bus, bus->dest_addr | 0x1);

			// Next int will occur on read
			bus->operation = SMB_READ_OPER;
		}
	} else {
		if (bus->PEC_use && !npcm_smb_is_quick(bus))
			// extra bytes for PEC support
			bus->wr_size += 1;

		// write next byte not last byte and not slave address
		if (!bus->fifo_use || bus->wr_size == 1) {
			if (bus->PEC_use && bus->rd_size == 0 &&
			    (bus->wr_ind + 1 == bus->wr_size)) {
				// Master write protocol to send PEC byte.
				npcm_smb_write_PEC(bus);
				bus->wr_ind++;
			} else {
				npcm_smb_wr_byte(bus,
						 bus->wr_buf[bus->wr_ind++]);
			}
		} else { // FIFO is used
			wcount = bus->wr_size - bus->wr_ind;
			npcm_smb_set_fifo(bus, -1, wcount);
			npcm_smb_write_to_fifo(bus, wcount);
		}
	}
}

static void npcm_smb_int_master_handler_read(struct npcm_i2c *bus)
{
	u16 block_zero_bytes;
	u32 fifo_bytes;

	// Master read operation (pure read or following a write operation).
	NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_READ);

	// Initialize number of bytes to include only the first byte (presents
	// a case where number of bytes to read is zero); add PEC if applicable
	block_zero_bytes = 1;
	if (bus->PEC_use)
		block_zero_bytes++;

	fifo_bytes = FIELD_GET(NPCM_SMBRXF_CTL_RX_THR,
			       ioread8(bus->reg + NPCM_SMBRXF_CTL));

	// Perform master read, distinguishing between last byte and the rest of
	// the bytes. The last byte should be read when the clock is stopped
	if ((bus->rd_ind < (bus->rd_size - 1)) ||  bus->fifo_use) {
		u8 data;

		// byte to be read is not the last one
		// Check if byte-before-last is about to be read
		if ((bus->rd_ind == (bus->rd_size - 2)) &&
		    !bus->fifo_use){
			// Set nack before reading byte-before-last, so that
			// nack will be generated after receive of last byte
			npcm_smb_nack(bus);

			if (!FIELD_GET(NPCM_SMBST_SDAST,
				       ioread8(bus->reg + NPCM_SMBST))) {
				// No data available - reset state for new xfer
				bus->state = SMB_IDLE;

				// Notify upper layer of rx completion
				npcm_smb_callback(bus, SMB_NO_DATA_IND,
						  bus->rd_ind);
			}
		} else if (bus->rd_ind == 0) { //first byte handling:
			// in block protocol first byte is the size
			if (bus->read_block_use) {
				npcm_smb_rd_byte(bus, &data);

				// First byte indicates length in block protocol
				bus->rd_buf[bus->rd_ind++] = data;
				bus->rd_size = data + 1;

				if (bus->PEC_use) {
					bus->rd_size += 1;
					data += 1;
				}

				if (bus->fifo_use) {
					iowrite8(NPCM_SMBFIF_CTS_RXF_TXE |
						 ioread8(bus->reg +
							 NPCM_SMBFIF_CTS),
						 bus->reg + NPCM_SMBFIF_CTS);

					// first byte in block protocol
					// is zero -> not supported. read at
					// least one byte
					if (data == 0)
						data = 1;
				}
				npcm_smb_set_fifo(bus, bus->rd_size, -1);
			} else {
				if (!bus->fifo_use) {
					npcm_smb_rd_byte(bus, &data);
					bus->rd_buf[bus->rd_ind++] = data;
				} else {
					npcm_smb_clear_tx_fifo(bus);
					npcm_smb_master_fifo_read(bus);
				}
			}

		} else {
			if (bus->fifo_use) {
				if (bus->rd_size == block_zero_bytes &&
				    bus->read_block_use) {
					npcm_smb_eob_int(bus, true);
					npcm_smb_master_stop(bus);
					npcm_smb_read_from_fifo(bus,
								fifo_bytes);
					bus->state = SMB_STOP_PENDING;
					bus->stop_ind = SMB_BLOCK_BYTES_ERR_IND;

				} else {
					npcm_smb_master_fifo_read(bus);
				}
			} else {
				npcm_smb_rd_byte(bus, &data);
				bus->rd_buf[bus->rd_ind++] = data;
			}
		}
	} else {
		// last byte is about to be read - end of transaction.
		// Stop should be set before reading last byte.
		u8 data;
		enum smb_state_ind ind = SMB_MASTER_DONE_IND;

		npcm_smb_eob_int(bus, true);

		npcm_smb_master_stop(bus);

		npcm_smb_rd_byte(bus, &data);

		if (bus->rd_size == block_zero_bytes && bus->read_block_use) {
			ind = SMB_BLOCK_BYTES_ERR_IND;
		} else {
			bus->rd_buf[bus->rd_ind++] = data;
			if (npcm_smb_get_PEC(bus) != 0)
				ind = SMB_MASTER_PEC_ERR_IND;
		}

		bus->state = SMB_STOP_PENDING;
		bus->stop_ind = ind;
	}
}

static void npcm_smb_int_master_handler(struct npcm_i2c *bus)
{
	// A negative acknowledge has occurred
	if (FIELD_GET(NPCM_SMBST_NEGACK, ioread8(bus->reg + NPCM_SMBST))) {
		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_NACK);
		if (bus->fifo_use) {
			// if there are still untransmitted bytes in TX FIFO
			// reduce them from wr_ind

			if (bus->operation == SMB_WRITE_OPER)
				bus->wr_ind -= npcm_smb_get_fifo_fullness(bus);
			// clear the FIFO
			iowrite8(NPCM_SMBFIF_CTS_CLR_FIFO,
				 bus->reg + NPCM_SMBFIF_CTS);
		}

		// In master write operation, NACK is a problem
		// number of bytes sent to master less than required
		npcm_smb_master_abort(bus);
		bus->state = SMB_IDLE;

		// In Master mode, NEGACK should be cleared only after
		// generating STOP.
		// In such case, the bus is released from stall only after the
		// software clears NEGACK bit.
		// Then a Stop condition is sent.
		iowrite8(NPCM_SMBST_NEGACK, bus->reg + NPCM_SMBST);
		npcm_smb_callback(bus, SMB_NACK_IND, bus->wr_ind);
		return;
	}

	// Master mode: a Bus Error has been identified
	if (FIELD_GET(NPCM_SMBST_BER, ioread8(bus->reg + NPCM_SMBST))) {
		// Check whether bus arbitration or Start or Stop during data
		// xfer bus arbitration problem should not result in recovery
		if (npcm_smb_is_master(bus)) {
			// Only current master is allowed to issue stop
			npcm_smb_master_abort(bus);
		} else {
			// Bus arbitration loss
			if (bus->retry_count-- > 0) {
				// Perform a retry (generate a start condition)
				// as soon as the SMBus is free
				iowrite8(NPCM_SMBST_BER, bus->reg + NPCM_SMBST);
				npcm_smb_master_start(bus);
				return;
			}
		}
		iowrite8(NPCM_SMBST_BER, bus->reg + NPCM_SMBST);
		bus->state = SMB_IDLE;
		npcm_smb_callback(bus, SMB_BUS_ERR_IND,
				  npcm_smb_get_index(bus));
		return;
	}

	// A Master End of Busy (meaning Stop Condition happened)
	// End of Busy int is on and End of Busy is set
	if ((FIELD_GET(NPCM_SMBCTL1_EOBINTE,
		       ioread8(bus->reg + NPCM_SMBCTL1)) == 1) &&
	    (FIELD_GET(NPCM_SMBCST3_EO_BUSY,
		       ioread8(bus->reg + NPCM_SMBCST3)))) {
		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_EOB);
		npcm_smb_eob_int(bus, false);
		bus->state = SMB_IDLE;
		if (npcm_smb_is_quick(bus))
			npcm_smb_callback(bus, bus->stop_ind, 0);
		else
			npcm_smb_callback(bus, bus->stop_ind, bus->rd_ind);
		return;
	}

	// Address sent and requested stall occurred (Master mode)
	if (FIELD_GET(NPCM_SMBST_STASTR, ioread8(bus->reg + NPCM_SMBST))) {
		// Check for Quick Command SMBus protocol
		if (npcm_smb_is_quick(bus)) {
			npcm_smb_eob_int(bus, true);
			npcm_smb_master_stop(bus);

			// Update status
			bus->state = SMB_STOP_PENDING;
			bus->stop_ind = SMB_MASTER_DONE_IND;

		} else if (bus->rd_size == 1) {
			// Receiving one byte only - set NACK after ensuring
			// slave ACKed the address byte
			npcm_smb_nack(bus);
		}

		// Reset stall-after-address-byte
		npcm_smb_stall_after_start(bus, false);

		// Clear stall only after setting STOP
		iowrite8(NPCM_SMBST_STASTR, bus->reg + NPCM_SMBST);

		return;
	}

	// SDA status is set - transmit or receive, master
	if (FIELD_GET(NPCM_SMBST_SDAST, ioread8(bus->reg + NPCM_SMBST)) ||
	    (bus->fifo_use &&
	    (npcm_smb_tx_fifo_full(bus) || npcm_smb_rx_fifo_full(bus)))) {
		// Status Bit is cleared by writing to or reading from SDA
		// (depending on current direction)
		switch (bus->state) {
		// Handle unsuccessful bus mastership
		case SMB_IDLE:
			npcm_smb_master_abort(bus);
			return;

		case SMB_MASTER_START:
			if (npcm_smb_is_master(bus)) {
				u8 addr_byte = bus->dest_addr;

				bus->crc_data = 0;
				if (npcm_smb_is_quick(bus)) {
					// Need to stall after successful
					// completion of sending address byte
					npcm_smb_stall_after_start(bus, true);
				} else if (bus->wr_size == 0) {
					// Set direction to Read
					addr_byte |= (u8)0x1;
					bus->operation = SMB_READ_OPER;
				} else {
					bus->operation = SMB_WRITE_OPER;
				}

	// Receiving one byte only - stall after successful completion of
	// sending address byte. If we NACK here, and slave doesn't ACK the
	// address, we might unintentionally NACK the next multi-byte read
				if (bus->wr_size == 0 && bus->rd_size == 1)
					npcm_smb_stall_after_start(bus, true);

				// Write the address to the bus
				bus->state = SMB_OPER_STARTED;
				npcm_smb_wr_byte(bus, addr_byte);
			} else {
				dev_err(bus->dev,
					"SDA, bus%d is not master, wr %d 0x%x...\n",
					bus->num, bus->wr_size,
					bus->wr_buf[0]);
			}
			break;

		// SDA status is set - transmit or receive: Handle master mode
		case SMB_OPER_STARTED:
			if (bus->operation == SMB_WRITE_OPER)
				npcm_smb_int_master_handler_write(bus);
			else if (bus->operation == SMB_READ_OPER)
				npcm_smb_int_master_handler_read(bus);
			else
				pr_err("I2C%d: unknown operation\n", bus->num);
			break;
		default:
			dev_err(bus->dev, "i2c%d master sda err on state machine\n",
				bus->num);
		}
	}
}

static int npcm_smb_recovery(struct i2c_adapter *_adap)
{
	u8   iter = 27;	  // Allow one byte to be sent by the Slave
	u16  timeout;
	bool done = false;
	struct npcm_i2c *bus = container_of(_adap, struct npcm_i2c, adap);

	dev_info(bus->dev, "recovery bus%d\n", bus->num);

	might_sleep();

	// Disable int
	npcm_smb_int_enable(bus, false);

	// Check If the SDA line is active (low)
	if (FIELD_GET(NPCM_SMBCST_TSDA, ioread8(bus->reg + NPCM_SMBCST)) == 0) {
		// Repeat the following sequence until SDA is released
		do {
			// Issue a single SCL cycle
			iowrite8(NPCM_SMBCST_TGSCL, bus->reg + NPCM_SMBCST);
			timeout = ABORT_TIMEOUT;
			while (timeout != 0 &&
			       FIELD_GET(NPCM_SMBCST_TGSCL,
					 ioread8(bus->reg + NPCM_SMBCST) == 0))
				timeout--;

			// If SDA line is inactive (high), stop
			if (FIELD_GET(NPCM_SMBCST_TSDA,
				      ioread8(bus->reg + NPCM_SMBCST)) == 1)
				done = true;
		} while ((!done) && (--iter != 0));

		// If SDA line is released (high)
		if (done) {
			// Clear BB (BUS BUSY) bit
			iowrite8(NPCM_SMBCST_BB, bus->reg + NPCM_SMBCST);

			// Generate a START, to synchronize Master and Slave
			npcm_smb_master_start(bus);

			// Wait until START condition is sent, or timeout
			timeout = ABORT_TIMEOUT;
			while (timeout != 0 && !npcm_smb_is_master(bus))
				timeout--;

			// If START condition was sent
			if (timeout > 0) {
				// Send an address byte
				npcm_smb_wr_byte(bus, bus->dest_addr);

				// Generate a STOP condition
				npcm_smb_master_stop(bus);
			}
			return 0;
		}
	}

	// check if success:
	if (npcm_smb_get_SCL(_adap) == 1 && npcm_smb_get_SDA(_adap) == 1)
		goto npcm_smb_recovery_done;

	// hold clock low for 35ms: 25 and some spair:
	npcm_smb_set_SCL(_adap, 0);
	usleep_range(35000, 40000);
	npcm_smb_set_SCL(_adap, 1);
	usleep_range(1000, 2000);

	// check if success:
	if (npcm_smb_get_SCL(_adap) == 1 && npcm_smb_get_SDA(_adap) == 1)
		goto npcm_smb_recovery_done;

	return 0;

npcm_smb_recovery_done:

	npcm_smb_int_enable(bus, true);
	return -(ENOTRECOVERABLE);
}

static bool npcm_smb_init_clk(struct npcm_i2c *bus, enum smb_mode mode,
			      u32 bus_freq)
{
	u32  k1 = 0;
	u32  k2 = 0;
	u8   dbnct = 0;
	u32  sclfrq = 0;
	u8   hldt = 7;
	bool fast_mode = false;
	u32  src_clk_freq; // in KHz

	src_clk_freq = bus->apb_clk / 1000;

	if (bus_freq <= SMBUS_FREQ_100KHZ) {
		sclfrq = src_clk_freq / (bus_freq * 4);

		if (sclfrq < SCLFRQ_MIN || sclfrq > SCLFRQ_MAX)
			return false;

		if (src_clk_freq >= 40000)
			hldt = 17;
		else if (src_clk_freq >= 12500)
			hldt = 15;
		else
			hldt = 7;
	}

	else if (bus_freq == SMBUS_FREQ_400KHZ) {
		sclfrq = 0;
		fast_mode = true;

		if ((mode == SMB_MASTER && src_clk_freq < 7500) ||
		    (mode == SMB_SLAVE && src_clk_freq < 10000))
		  // 400KHZ cannot be supported for master core clock < 7.5 MHZ
		  // or slave core clock < 10 MHZ
			return false;

		// Master or Slave with frequency > 25 MHZ
		if (mode == SMB_MASTER || src_clk_freq > 25000) {
			hldt = (u8)__KERNEL_DIV_ROUND_UP(src_clk_freq * 300,
							 1000000) + 7;
			if (mode == SMB_MASTER) {
				k1 = __KERNEL_DIV_ROUND_UP(src_clk_freq * 1600,
							   1000000);
				k2 = __KERNEL_DIV_ROUND_UP(src_clk_freq * 900,
							   1000000);
				k1 = round_up(k1, 2);
				k2 = round_up(k2 + 1, 2);
				if (k1 < SCLFRQ_MIN || k1 > SCLFRQ_MAX ||
				    k2 < SCLFRQ_MIN || k2 > SCLFRQ_MAX)
					return false;
			}
		} else { // Slave with frequency 10-25 MHZ
			hldt = 7;
			dbnct = 2;
		}
	}

	else if (bus_freq == SMBUS_FREQ_1MHZ) {
		sclfrq = 0;
		fast_mode = true;

		if ((mode == SMB_MASTER && src_clk_freq < 15000) ||
		    (mode == SMB_SLAVE	&& src_clk_freq < 24000))
		// 1MHZ cannot be supported for master core clock < 15 MHZ
		// or slave core clock < 24 MHZ
			return false;

		if (mode == SMB_MASTER) {
			k1 = round_up((__KERNEL_DIV_ROUND_UP(src_clk_freq * 620,
							     1000000)), 2);
			k2 = round_up((__KERNEL_DIV_ROUND_UP(src_clk_freq * 380,
							     1000000) + 1), 2);
			if (k1 < SCLFRQ_MIN || k1 > SCLFRQ_MAX ||
			    k2 < SCLFRQ_MIN || k2 > SCLFRQ_MAX) {
				return false;
			}
		}

		// Master or Slave with frequency > 40 MHZ
		if (mode == SMB_MASTER || src_clk_freq > 40000) {
			// Set HLDT:
			// SDA hold time:  (HLDT-7) * T(CLK) >= 120
			// HLDT = 120/T(CLK) + 7 = 120 * FREQ(CLK) + 7
			hldt = (u8)__KERNEL_DIV_ROUND_UP(src_clk_freq * 120,
							 1000000) + 7;

		// Slave with frequency 24-40 MHZ
		} else {
			hldt = 7;
			dbnct = 2;
		}
	}

	// Frequency larger than 1 MHZ
	else
		return false;

	// After clock parameters calculation update the reg
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL2)
		& ~SMBCTL2_SCLFRQ6_0) | FIELD_PREP(SMBCTL2_SCLFRQ6_0,
		sclfrq & 0x7F), bus->reg + NPCM_SMBCTL2);

	iowrite8((ioread8(bus->reg + NPCM_SMBCTL3) & ~SMBCTL3_SCLFRQ8_7) |
		 FIELD_PREP(SMBCTL3_SCLFRQ8_7, (sclfrq >> 7) & 0x3),
		 bus->reg + NPCM_SMBCTL3);

	iowrite8((ioread8(bus->reg + NPCM_SMBCTL3) & ~SMBCTL3_400K_MODE) |
		 FIELD_PREP(SMBCTL3_400K_MODE, fast_mode),
		 bus->reg + NPCM_SMBCTL3);

	// Select Bank 0 to access NPCM_SMBCTL4/NPCM_SMBCTL5
	npcm_smb_select_bank(bus, SMB_BANK_0);

	if (bus_freq >= SMBUS_FREQ_400KHZ) {
		// k1 and k2 are relevant for master mode only
		if (mode == SMB_MASTER) {
			// Set SCL Low/High Time:
			// k1 = 2 * SCLLT7-0 -> Low Time  = k1 / 2
			// k2 = 2 * SCLLT7-0 -> High Time = k2 / 2
			iowrite8((u8)k1 / 2, bus->reg + NPCM_SMBSCLLT);
			iowrite8((u8)k2 / 2, bus->reg + NPCM_SMBSCLHT);
		} else { // DBNCT is relevant for slave mode only
			iowrite8((ioread8(bus->reg + NPCM_SMBCTL5) &
				 ~SMBCTL5_DBNCT) |
				 FIELD_PREP(SMBCTL5_DBNCT, dbnct),
				 bus->reg + NPCM_SMBCTL5);
		}
	}

	iowrite8((ioread8(bus->reg + NPCM_SMBCTL4) & ~SMBCTL4_HLDT)
		 | FIELD_PREP(SMBCTL4_HLDT, hldt), bus->reg + NPCM_SMBCTL4);

	// Return to Bank 1, and stay there by default:
	npcm_smb_select_bank(bus, SMB_BANK_1);

	dev_dbg(bus->dev, "k1 = %d k2 = %d dbnct = %d sclfrq = %d hldt = %d src_clk_freq %d fast_mode %d\n",
		k1, k2, dbnct, sclfrq, hldt, src_clk_freq, fast_mode);

	return true;
}

static bool npcm_smb_init_module(struct npcm_i2c *bus, enum smb_mode mode,
				 u32 bus_freq)
{
	// Check whether module already enabled or frequency is out of bounds
	if ((bus->state != SMB_DISABLE && bus->state != SMB_IDLE) ||
	    bus_freq < SMBUS_FREQ_MIN || bus_freq > SMBUS_FREQ_MAX)
		return false;
	// Configure FIFO disabled mode so slave will not use fifo
	// (maste will set it on if supported)
	bus->threshold_fifo = SMBUS_FIFO_SIZE;
	iowrite8(ioread8(bus->reg + NPCM_SMBFIF_CTL) & ~NPCM_SMBFIF_CTL_FIFO_EN,
		 bus->reg + NPCM_SMBFIF_CTL);

	bus->fifo_use = false;

	// Configure SMB module clock frequency
	if (!npcm_smb_init_clk(bus, mode, bus_freq)) {
		pr_err("npcm_smb_init_clk failed\n");
		return false;
	}
	npcm_smb_disable(bus);

	// Enable module (before configuring CTL1)
	npcm_smb_enable(bus);
	bus->state = SMB_IDLE;

	// Enable SMB int and New Address Match int source
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) | NPCM_SMBCTL1_NMINTE) &
		 ~NPCM_SMBCTL1_RWS_FIELDS,
		 bus->reg + NPCM_SMBCTL1);

	npcm_smb_int_enable(bus, true);
	return true;
}

static int __npcm_i2c_init(struct npcm_i2c *bus, struct platform_device *pdev)
{
	u32 clk_freq;
	int ret;

	// Initialize the internal data structures
	bus->state = SMB_DISABLE;
	bus->master_or_slave = SMB_SLAVE;

	ret = of_property_read_u32(pdev->dev.of_node,
				   "bus-frequency", &clk_freq);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Could not read bus-frequency property\n");
		clk_freq = 100000;
	}
	ret = npcm_smb_init_module(bus, SMB_MASTER, clk_freq / 1000);
	if (!ret) {
		dev_err(&pdev->dev,
			"npcm_smb_init_module() failed\n");
		return -1;
	}

	crc8_populate_lsb(npcm7xx_crc8, 0x07);
	crc8_populate_msb(npcm7xx_crc8, 0x07);
	return 0;
}

static irqreturn_t npcm_i2c_bus_irq(int irq, void *dev_id)
{
	struct npcm_i2c *bus = dev_id;

	bus->int_cnt++;
	_npcm7xx_get_time_stamp(&bus->int_time_stamp[0],
				&bus->int_time_stamp[1]);
	if (bus->master_or_slave == SMB_MASTER)	{
		npcm_smb_int_master_handler(bus);
		return IRQ_HANDLED;
	}

	dev_err(bus->dev, "int unknown on bus%d\n", bus->num);
	return IRQ_NONE;
}

static bool npcm_smb_master_start_xmit(struct npcm_i2c *bus,
				       u8 slave_addr, u16 nwrite, u16 nread,
				       u8 *write_data, u8 *read_data,
				       bool use_PEC)
{
	//
	// Allow only if bus is not busy
	//
	if (bus->state != SMB_IDLE) {
		dev_info(bus->dev, "\tbus%d->state != SMB_IDLE\n", bus->num);
		return false;
	}

	// Configure FIFO mode :
	if (FIELD_GET(SMB_VER_FIFO_EN, ioread8(bus->reg + SMB_VER))) {
		bus->fifo_use = true;
		iowrite8(ioread8(bus->reg + NPCM_SMBFIF_CTL) |
			 NPCM_SMBFIF_CTL_FIFO_EN, bus->reg + NPCM_SMBFIF_CTL);
	} else {
		bus->fifo_use = false;
	}

	// Update driver state
	bus->master_or_slave = SMB_MASTER;
	bus->state = SMB_MASTER_START;
	if (nwrite > 0)
		bus->operation = SMB_WRITE_OPER;
	else
		bus->operation = SMB_READ_OPER;

	if (npcm_smb_is_quick(bus))
		bus->operation = SMB_WRITE_OPER; // send the address with W bit.

	bus->dest_addr = (u8)(slave_addr << 1);// Translate 7bit to 8bit format
	bus->wr_buf = write_data;
	bus->wr_size = nwrite;
	bus->wr_ind = 0;
	bus->rd_buf = read_data;
	bus->rd_size = nread;
	bus->rd_ind = 0;
	bus->PEC_use = use_PEC;
	bus->retry_count = SMB_RETRY_MAX_COUNT;

	// clear BER just in case it is set due to a previous transaction
	iowrite8(NPCM_SMBST_BER, bus->reg + NPCM_SMBST);

	// Initiate SMBus master transaction
	// Generate a Start condition on the SMBus
	if (bus->fifo_use) {
		// select bank 1 for FIFO regs
		npcm_smb_select_bank(bus, SMB_BANK_1);

		// clear FIFO and relevant status bits.
		iowrite8(ioread8(bus->reg + NPCM_SMBFIF_CTS) |
			 NPCM_SMBFIF_CTS_SLVRSTR |
			 NPCM_SMBFIF_CTS_CLR_FIFO |
			 NPCM_SMBFIF_CTS_RXF_TXE, bus->reg + NPCM_SMBFIF_CTS);

		if (bus->operation == SMB_READ_OPER) {
			//This is a read only operation. Configure the FIFO
			//threshold according to the needed # of bytes to read.
			npcm_smb_set_fifo(bus, nread, -1);
		} else if (bus->operation == SMB_WRITE_OPER) {
			npcm_smb_set_fifo(bus, -1, nwrite);
		}
	}

	bus->int_cnt = 0;
	bus->event_log = 0;
	npcm_smb_master_start(bus);

	return true;
}

static int npcm_i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
				int num)
{
	struct npcm_i2c *bus = adap->algo_data;
	struct i2c_msg *msg0, *msg1;
	unsigned long time_left, flags;
	u16 nwrite, nread;
	u8 *write_data, *read_data;
	u8 slave_addr;
	int ret = 0;

	spin_lock_irqsave(&bus->lock, flags);
	bus->cmd_err = -EPERM;
	bus->int_cnt = 0;
	bus->stop_ind = SMB_NO_STATUS_IND;
	bus->read_block_use = false;

	iowrite8(0xFF, bus->reg + NPCM_SMBST);

	if (num > 2 || num < 1) {
		pr_err("I2C command not supported, num of msgs = %d\n", num);
		spin_unlock_irqrestore(&bus->lock, flags);
		return -EINVAL;
	}

	msg0 = &msgs[0];
	slave_addr = msg0->addr;
	if (msg0->flags & I2C_M_RD) { // read
		if (num == 2) {
			pr_err(" num = 2 but first msg is rd instead of wr\n");
			spin_unlock_irqrestore(&bus->lock, flags);
			return -EINVAL;
		}
		nwrite = 0;
		write_data = NULL;
		if (msg0->flags & I2C_M_RECV_LEN) {
			nread = 1;
			bus->read_block_use = true;

		} else {
			nread = msg0->len;
		}
		read_data = msg0->buf;

	} else { // write
		nwrite = msg0->len;
		write_data = msg0->buf;
		nread = 0;
		read_data = NULL;
		if (num == 2) {
			msg1 = &msgs[1];
			if (slave_addr != msg1->addr) {
				pr_err("SA==%02x but msg1->addr == %02x\n",
				       slave_addr, msg1->addr);
				spin_unlock_irqrestore(&bus->lock, flags);
				return -EINVAL;
			}
			if ((msg1->flags & I2C_M_RD) == 0) {
				pr_err("num = 2 but both msg are write.\n");
				spin_unlock_irqrestore(&bus->lock, flags);
				return -EINVAL;
			}
			if (msg1->flags & I2C_M_RECV_LEN) {
				nread = 1;
				bus->read_block_use = true;
			} else {
				nread = msg1->len;
				bus->read_block_use = false;
			}

			read_data = msg1->buf;
		}
	}

	bus->msgs = msgs;
	bus->msgs_num = num;

	if (nwrite >= 32 * 1024 ||  nread >= 32 * 1024) {
		pr_err("i2c%d buffer too big\n", bus->num);
		return -EINVAL;
	}

	reinit_completion(&bus->cmd_complete);

	if (npcm_smb_master_start_xmit(bus, slave_addr, nwrite, nread,
				       write_data, read_data, 0) == false)
		ret = -(EBUSY);

	if (ret != -(EBUSY)) {
		time_left = wait_for_completion_timeout(&bus->cmd_complete,
							bus->adap.timeout);

		if (time_left == 0 && bus->cmd_err == -EPERM) {
			npcm_smb_master_abort(bus);
			ret = -ETIMEDOUT;
		} else {
			ret = bus->cmd_err;
		}
	}

	bus->msgs = NULL;
	bus->msgs_num = 0;
	spin_unlock_irqrestore(&bus->lock, flags);

	// If nothing went wrong, return number of messages xferred.
	if (ret >= 0)
		return num;
	else
		return ret;
}

static u32 npcm_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static const struct i2c_algorithm npcm_i2c_algo = {
	.master_xfer = npcm_i2c_master_xfer,
	.functionality = npcm_i2c_functionality,
};

static struct i2c_bus_recovery_info npcm_i2c_recovery = {
	.recover_bus = npcm_smb_recovery,
	.get_scl = npcm_smb_get_SCL,
	.set_scl = npcm_smb_set_SCL,
	.get_sda = npcm_smb_get_SDA,
};

static int  npcm_i2c_probe_bus(struct platform_device *pdev)
{
	struct npcm_i2c *bus;
	struct resource *res;
	struct clk *i2c_clk;
	int ret;
	int num;

	bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

#ifdef CONFIG_OF
	num = of_alias_get_id(pdev->dev.of_node, "i2c");
	bus->num = num;
	i2c_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2c_clk)) {
		pr_err(" I2C probe failed: can't read clk.\n");
		return	-EPROBE_DEFER;
	}
	bus->apb_clk = clk_get_rate(i2c_clk);
	dev_dbg(bus->dev, "I2C APB clock is %d\n", bus->apb_clk);
#endif //  CONFIG_OF

	gcr_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-gcr");
	if (IS_ERR(gcr_regmap)) {
		pr_err("%s: failed to find nuvoton,npcm750-gcr\n", __func__);
		return IS_ERR(gcr_regmap);
	}
	regmap_write(gcr_regmap, NPCM_I2CSEGCTL, I2CSEGCTL_VAL);
	dev_dbg(bus->dev, "I2C%d: gcr mapped\n", bus->num);

	clk_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-clk");
	if (IS_ERR(clk_regmap)) {
		pr_err("%s: failed to find nuvoton,npcm750-clk\n", __func__);
		return IS_ERR(clk_regmap);
	}
	dev_dbg(bus->dev, "I2C%d: clk mapped\n", bus->num);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev_dbg(bus->dev, "resource: %pR\n", res);
	bus->reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR((bus)->reg))
		return PTR_ERR((bus)->reg);
	dev_dbg(bus->dev, "base = %p\n", bus->reg);

	// Initialize the I2C adapter
	spin_lock_init(&bus->lock);
	init_completion(&bus->cmd_complete);
	bus->adap.owner = THIS_MODULE;
	bus->adap.class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	bus->adap.retries = 0;
	bus->adap.timeout = 500 * HZ / 1000;
	bus->adap.algo = &npcm_i2c_algo;
	bus->adap.algo_data = bus;
	bus->adap.dev.parent = &pdev->dev;
	bus->adap.dev.of_node = pdev->dev.of_node;
	bus->adap.bus_recovery_info = &npcm_i2c_recovery;

	snprintf(bus->adap.name, sizeof(bus->adap.name), "Nuvoton i2c");

	bus->dev = &pdev->dev;

	ret = __npcm_i2c_init(bus, pdev);
	if (ret < 0)
		return ret;

	bus->irq = platform_get_irq(pdev, 0);
	if (bus->irq < 0) {
		pr_err("I2C platform_get_irq error.");
		return -ENODEV;
	}
	dev_dbg(bus->dev, "irq = %d\n", bus->irq);

	ret = request_irq(bus->irq, npcm_i2c_bus_irq, 0,
			  dev_name(&pdev->dev), (void *)bus);
	if (ret) {
		dev_err(&pdev->dev, "I2C%d: request_irq fail\n", bus->num);
		return ret;
	}

	ret = i2c_add_adapter(&bus->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "I2C%d: i2c_add_adapter fail\n", bus->num);
		return ret;
	}

	platform_set_drvdata(pdev, bus);
	pr_info("i2c bus %d registered\n", bus->adap.nr);

	return 0;
}

static int  npcm_i2c_remove_bus(struct platform_device *pdev)
{
	unsigned long lock_flags;
	struct npcm_i2c *bus = platform_get_drvdata(pdev);

	spin_lock_irqsave(&bus->lock, lock_flags);
	npcm_smb_disable(bus);
	spin_unlock_irqrestore(&bus->lock, lock_flags);
	i2c_del_adapter(&bus->adap);

	return 0;
}

static const struct of_device_id npcm_i2c_bus_of_table[] = {
	{ .compatible = "nuvoton,npcm750-i2c", },
	{},
};
MODULE_DEVICE_TABLE(of, npcm_i2c_bus_of_table);

static struct platform_driver npcm_i2c_bus_driver = {
	.probe = npcm_i2c_probe_bus,
	.remove = npcm_i2c_remove_bus,
	.driver = {
		.name = "nuvoton-i2c",
		.of_match_table = npcm_i2c_bus_of_table,
	}
};
module_platform_driver(npcm_i2c_bus_driver);

MODULE_AUTHOR("Avi Fishman <avi.fishman@gmail.com>");
MODULE_AUTHOR("Tali Perry <tali.perry@nuvoton.com>");
MODULE_DESCRIPTION("Nuvoton I2C Bus Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(I2C_VERSION);
