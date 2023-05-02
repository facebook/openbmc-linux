// SPDX-License-Identifier: GPL-2.0
// drivers/jtag/aspeed-jtag.c
//
// Copyright (c) 2018 Mellanox Technologies. All rights reserved.
// Copyright (c) 2018 Oleksandr Shamray <oleksandrs@mellanox.com>

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/jtag.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <uapi/linux/ioctl.h>
#include <uapi/linux/jtag.h>

#define ASPEED_2600_JTAG_MASTER2_FREQ	10000000
#define ASPEED_2600_RESET_WAIT_COUNT	10
#define ASPEED_NS_CONSTANT		1000000000
#define ASPEED_MS_CONSTANT		1000
#define ASPEED_SCU_RESET_JTAG		BIT(22)
#define ASPEED_2600_SCU_CLEAR_REGISTER	0x04

/* AST2600 JTAG master pins */
#define ASPEED_2600_SCU_ENABLE_PIN_TDI		BIT(4)
#define ASPEED_2600_SCU_ENABLE_PIN_TMS		BIT(3)
#define ASPEED_2600_SCU_ENABLE_PIN_TCK		BIT(2)
#define ASPEED_2600_SCU_ENABLE_PIN_TDO		BIT(1)
#define ASPEED_2600_SCU_ENABLE_PIN_TRSTN	BIT(0)

#define ASPEED_JTAG_DATA		0x00
#define ASPEED_JTAG_INST		0x04
#define ASPEED_JTAG_CTRL		0x08
#define ASPEED_JTAG_ISR			0x0C
#define ASPEED_JTAG_SW			0x10
#define ASPEED_JTAG_TCK			0x14
#define ASPEED_JTAG_EC			0x18

#define ASPEED_JTAG_DATA_MSB		0x01
#define ASPEED_JTAG_DATA_CHUNK_SIZE	0x20
#define ASPEED_JTAG_512BITS_CHUNK_SIZE	0x200

/* ASPEED_JTAG_CTRL: Engine Control */
#define ASPEED_JTAG_CTL_ENG_EN		BIT(31)
#define ASPEED_JTAG_CTL_ENG_OUT_EN	BIT(30)
#define ASPEED_JTAG_CTL_FORCE_TMS	BIT(29)
#define ASPEED_JTAG_CTL_IR_UPDATE	BIT(26)
#define ASPEED_JTAG_CTL_INST_LEN(x)	((x) << 20)
#define ASPEED_JTAG_CTL_LASPEED_INST	BIT(17)
#define ASPEED_JTAG_CTL_INST_EN		BIT(16)
#define ASPEED_JTAG_CTL_DR_UPDATE	BIT(10)
#define ASPEED_JTAG_CTL_DATA_LEN(x)	((x) << 4)
#define ASPEED_JTAG_CTL_LASPEED_DATA	BIT(1)
#define ASPEED_JTAG_CTL_DATA_EN		BIT(0)
#define ASPEED_JTAG_CTL_2600_RST_FIFO	BIT(21)
#define ASPEED_JTAG_CTL_2600_FIFO_MODE	BIT(20)
#define ASPEED_JTAG_CTL_2600_TX_LEN(x)	((x) << 8)
#define ASPEED_JTAG_CTL_2600_LAST_TX	BIT(4)
#define ASPEED_JTAG_CTL_2600_INST_EN	BIT(1)

/* ASPEED_JTAG_ISR : Interrupt status and enable */
#define ASPEED_JTAG_ISR_INST_PAUSE	BIT(19)
#define ASPEED_JTAG_ISR_INST_COMPLETE	BIT(18)
#define ASPEED_JTAG_ISR_DATA_PAUSE	BIT(17)
#define ASPEED_JTAG_ISR_DATA_COMPLETE	BIT(16)
#define ASPEED_JTAG_ISR_INST_PAUSE_EN	BIT(3)
#define ASPEED_JTAG_ISR_INST_COMPLETE_EN BIT(2)
#define ASPEED_JTAG_ISR_DATA_PAUSE_EN	BIT(1)
#define ASPEED_JTAG_ISR_DATA_COMPLETE_EN BIT(0)
#define ASPEED_JTAG_ISR_INT_EN_MASK	GENMASK(3, 0)
#define ASPEED_JTAG_ISR_INT_MASK	GENMASK(19, 16)

/* ASPEED_JTAG_SW : Software Mode and Status */
#define ASPEED_JTAG_SW_MODE_EN		BIT(19)
#define ASPEED_JTAG_SW_MODE_TCK		BIT(18)
#define ASPEED_JTAG_SW_MODE_TMS		BIT(17)
#define ASPEED_JTAG_SW_MODE_TDIO	BIT(16)

/* ASPEED_JTAG_TCK : TCK Control */
#define ASPEED_JTAG_TCK_DIVISOR_MASK	GENMASK(10, 0)
#define ASPEED_JTAG_TCK_GET_DIV(x)	((x) & ASPEED_JTAG_TCK_DIVISOR_MASK)

/* ASPEED_JTAG_EC : Controller set for go to IDLE */
#define ASPEED_JTAG_EC_GO_IDLE		BIT(0)

#define ASPEED_JTAG_IOUT_LEN(len) \
	(ASPEED_JTAG_CTL_ENG_EN | \
	 ASPEED_JTAG_CTL_ENG_OUT_EN | \
	 ASPEED_JTAG_CTL_INST_LEN(len))

#define ASPEED_JTAG_DOUT_LEN(len) \
	(ASPEED_JTAG_CTL_ENG_EN | \
	 ASPEED_JTAG_CTL_ENG_OUT_EN | \
	 ASPEED_JTAG_CTL_DATA_LEN(len))

#define ASPEED_JTAG_2600_TX_LEN(len) \
	(ASPEED_JTAG_CTL_ENG_EN | \
	ASPEED_JTAG_CTL_ENG_OUT_EN | \
	ASPEED_JTAG_CTL_2600_TX_LEN(len))

#define ASPEED_JTAG_SW_TDIO (ASPEED_JTAG_SW_MODE_EN | ASPEED_JTAG_SW_MODE_TDIO)

#define ASPEED_JTAG_GET_TDI(direction, byte) \
	(((direction) & JTAG_WRITE_XFER) ? byte : UINT_MAX)

#define ASPEED_JTAG_TCK_WAIT		10
#define ASPEED_JTAG_RESET_CNTR		10
#define WAIT_ITERATIONS		75

/* Enable/Disable FIFO Controller Mode */
#define ENABLE_FIFO_CTRL

static DEFINE_SPINLOCK(JTAG_SPINLOCK);

static const char * const regnames[] = {
	[ASPEED_JTAG_DATA] = "ASPEED_JTAG_DATA",
	[ASPEED_JTAG_INST] = "ASPEED_JTAG_INST",
	[ASPEED_JTAG_CTRL] = "ASPEED_JTAG_CTRL",
	[ASPEED_JTAG_ISR]  = "ASPEED_JTAG_ISR",
	[ASPEED_JTAG_SW]   = "ASPEED_JTAG_SW",
	[ASPEED_JTAG_TCK]  = "ASPEED_JTAG_TCK",
	[ASPEED_JTAG_EC]   = "ASPEED_JTAG_EC",
};

#define ASPEED_JTAG_NAME		"jtag-aspeed"

struct aspeed_jtag {
	void __iomem			*reg_base;
	void __iomem			*scu_base;
	void __iomem			*scupin_ctrl;
	struct device			*dev;
	struct clk			*pclk;
	enum jtag_endstate		status;
	int				irq;
	struct reset_control		*rst;
	u32				flag;
	wait_queue_head_t		jtag_wq;
	u32				mode;
	int				scu_clear_reg;
	int				use_irq;
	u32				freq;
};

/*
 * This structure represents a TMS cycle, as expressed in a set of bits and a
 * count of bits (note: there are no start->end state transitions that require
 * more than 1 byte of TMS cycles)
 */
struct tms_cycle {
	unsigned char		tmsbits;
	unsigned char		count;
};

/*
 * This is the complete set TMS cycles for going from any TAP state to any
 * other TAP state, following a "shortest path" rule.
 */
static const struct tms_cycle _tms_cycle_lookup[][16] = {
/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* TLR  */{{0x00, 0}, {0x00, 1}, {0x02, 2}, {0x02, 3}, {0x02, 4}, {0x0a, 4},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x0a, 5}, {0x2a, 6}, {0x1a, 5}, {0x06, 3}, {0x06, 4}, {0x06, 5},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x16, 5}, {0x16, 6}, {0x56, 7}, {0x36, 6} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* RTI  */{{0x07, 3}, {0x00, 0}, {0x01, 1}, {0x01, 2}, {0x01, 3}, {0x05, 3},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x05, 4}, {0x15, 5}, {0x0d, 4}, {0x03, 2}, {0x03, 3}, {0x03, 4},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x0b, 4}, {0x0b, 5}, {0x2b, 6}, {0x1b, 5} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* SelDR*/{{0x03, 2}, {0x03, 3}, {0x00, 0}, {0x00, 1}, {0x00, 2}, {0x02, 2},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x02, 3}, {0x0a, 4}, {0x06, 3}, {0x01, 1}, {0x01, 2}, {0x01, 3},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x05, 3}, {0x05, 4}, {0x15, 5}, {0x0d, 4} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* CapDR*/{{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x00, 0}, {0x00, 1}, {0x01, 1},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x01, 2}, {0x05, 3}, {0x03, 2}, {0x0f, 4}, {0x0f, 5}, {0x0f, 6},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x2f, 6}, {0x2f, 7}, {0xaf, 8}, {0x6f, 7} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* SDR  */{{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x07, 4}, {0x00, 0}, {0x01, 1},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x01, 2}, {0x05, 3}, {0x03, 2}, {0x0f, 4}, {0x0f, 5}, {0x0f, 6},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x2f, 6}, {0x2f, 7}, {0xaf, 8}, {0x6f, 7} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* Ex1DR*/{{0x0f, 4}, {0x01, 2}, {0x03, 2}, {0x03, 3}, {0x02, 3}, {0x00, 0},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x00, 1}, {0x02, 2}, {0x01, 1}, {0x07, 3}, {0x07, 4}, {0x07, 5},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x17, 5}, {0x17, 6}, {0x57, 7}, {0x37, 6} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* PDR  */{{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x07, 4}, {0x01, 2}, {0x05, 3},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x00, 0}, {0x01, 1}, {0x03, 2}, {0x0f, 4}, {0x0f, 5}, {0x0f, 6},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x2f, 6}, {0x2f, 7}, {0xaf, 8}, {0x6f, 7} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* Ex2DR*/{{0x0f, 4}, {0x01, 2}, {0x03, 2}, {0x03, 3}, {0x00, 1}, {0x02, 2},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x02, 3}, {0x00, 0}, {0x01, 1}, {0x07, 3}, {0x07, 4}, {0x07, 5},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x17, 5}, {0x17, 6}, {0x57, 7}, {0x37, 6} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* UpdDR*/{{0x07, 3}, {0x00, 1}, {0x01, 1}, {0x01, 2}, {0x01, 3}, {0x05, 3},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x05, 4}, {0x15, 5}, {0x00, 0}, {0x03, 2}, {0x03, 3}, {0x03, 4},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x0b, 4}, {0x0b, 5}, {0x2b, 6}, {0x1b, 5} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* SelIR*/{{0x01, 1}, {0x01, 2}, {0x05, 3}, {0x05, 4}, {0x05, 5}, {0x15, 5},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x15, 6}, {0x55, 7}, {0x35, 6}, {0x00, 0}, {0x00, 1}, {0x00, 2},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x02, 2}, {0x02, 3}, {0x0a, 4}, {0x06, 3} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* CapIR*/{{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x07, 4}, {0x07, 5}, {0x17, 5},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x17, 6}, {0x57, 7}, {0x37, 6}, {0x0f, 4}, {0x00, 0}, {0x00, 1},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x01, 1}, {0x01, 2}, {0x05, 3}, {0x03, 2} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* SIR  */{{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x07, 4}, {0x07, 5}, {0x17, 5},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x17, 6}, {0x57, 7}, {0x37, 6}, {0x0f, 4}, {0x0f, 5}, {0x00, 0},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x01, 1}, {0x01, 2}, {0x05, 3}, {0x03, 2} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* Ex1IR*/{{0x0f, 4}, {0x01, 2}, {0x03, 2}, {0x03, 3}, {0x03, 4}, {0x0b, 4},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x0b, 5}, {0x2b, 6}, {0x1b, 5}, {0x07, 3}, {0x07, 4}, {0x02, 3},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x00, 0}, {0x00, 1}, {0x02, 2}, {0x01, 1} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* PIR  */{{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x07, 4}, {0x07, 5}, {0x17, 5},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x17, 6}, {0x57, 7}, {0x37, 6}, {0x0f, 4}, {0x0f, 5}, {0x01, 2},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x05, 3}, {0x00, 0}, {0x01, 1}, {0x03, 2} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* Ex2IR*/{{0x0f, 4}, {0x01, 2}, {0x03, 2}, {0x03, 3}, {0x03, 4}, {0x0b, 4},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x0b, 5}, {0x2b, 6}, {0x1b, 5}, {0x07, 3}, {0x07, 4}, {0x00, 1},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x02, 2}, {0x02, 3}, {0x00, 0}, {0x01, 1} },

/*	    TLR        RTI        SelDR      CapDR      SDR        Ex1DR*/
/* UpdIR*/{{0x07, 3}, {0x00, 1}, {0x01, 1}, {0x01, 2}, {0x01, 3}, {0x05, 3},
/*	    PDR        Ex2DR      UpdDR      SelIR      CapIR      SIR*/
	    {0x05, 4}, {0x15, 5}, {0x0d, 4}, {0x03, 2}, {0x03, 3}, {0x03, 4},
/*	    Ex1IR      PIR        Ex2IR      UpdIR*/
	    {0x0b, 4}, {0x0b, 5}, {0x2b, 6}, {0x00, 0} },
};

static char *end_status_str[] = {
	"tlr", "idle", "selDR", "capDR", "sDR", "ex1DR", "pDR", "ex2DR",
	 "updDR", "selIR", "capIR", "sIR", "ex1IR", "pIR", "ex2IR", "updIR"
};

static u32 aspeed_jtag_read(struct aspeed_jtag *aspeed_jtag, u32 reg)
{
	u32 val = readl(aspeed_jtag->reg_base + reg);

	dev_dbg(aspeed_jtag->dev, "read:%s val = 0x%08x\n", regnames[reg], val);
	return val;
}

static void
aspeed_jtag_write(struct aspeed_jtag *aspeed_jtag, u32 val, u32 reg)
{
	dev_dbg(aspeed_jtag->dev, "write:%s val = 0x%08x\n",
		regnames[reg], val);
	writel(val, aspeed_jtag->reg_base + reg);
}

static int aspeed_jtag_freq_set(struct jtag *jtag, u32 freq)
{
	struct aspeed_jtag *aspeed_jtag = jtag_priv(jtag);
	unsigned long apb_frq;
	u32 tck_val;
	u16 div;

	apb_frq = clk_get_rate(aspeed_jtag->pclk);
	if (!apb_frq) {
		dev_err(aspeed_jtag->dev, "\nFailed to get clk rate.\n");
		return -ENOTSUPP;
	}

	div = (apb_frq - 1) / freq;
	tck_val = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK);
	aspeed_jtag_write(aspeed_jtag,
			  (tck_val & ~ASPEED_JTAG_TCK_DIVISOR_MASK) | div,
			  ASPEED_JTAG_TCK);

	aspeed_jtag->freq = freq;

	return 0;
}

static int aspeed_jtag_freq_get(struct jtag *jtag, u32 *frq)
{
	struct aspeed_jtag *aspeed_jtag = jtag_priv(jtag);
	u32 pclk;
	u32 tck;

	pclk = clk_get_rate(aspeed_jtag->pclk);
	tck = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK);
	*frq = pclk / (ASPEED_JTAG_TCK_GET_DIV(tck) + 1);

	return 0;
}

static inline void aspeed_jtag_2600_master_pins_enable(struct aspeed_jtag
			*aspeed_jtag)
{
	u32 val;

	if (aspeed_jtag->scupin_ctrl) {
		val = readl(aspeed_jtag->scupin_ctrl);

		/* Enable JTAG MASTER pins */
		writel((val |
			ASPEED_2600_SCU_ENABLE_PIN_TDI |
			ASPEED_2600_SCU_ENABLE_PIN_TMS |
			ASPEED_2600_SCU_ENABLE_PIN_TCK |
			ASPEED_2600_SCU_ENABLE_PIN_TDO) &
			~ASPEED_2600_SCU_ENABLE_PIN_TRSTN,
			aspeed_jtag->scupin_ctrl);
	}
}

static inline int aspeed_jtag_2600_reset_state_machine(struct aspeed_jtag
			*aspeed_jtag)
{
	u32 i;
	u32 val;
	u32 status;

	val = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_CTRL);

	/* enable JTAG engine & JTAG engine Output */
	aspeed_jtag_write(aspeed_jtag,	ASPEED_JTAG_CTL_ENG_EN |
					ASPEED_JTAG_CTL_ENG_OUT_EN,
					ASPEED_JTAG_CTRL);

	mdelay(1);

	/* assert TMS for at least 5 cycles */
	aspeed_jtag_write(aspeed_jtag,	ASPEED_JTAG_CTL_ENG_EN |
					ASPEED_JTAG_CTL_ENG_OUT_EN |
					ASPEED_JTAG_CTL_FORCE_TMS,
					ASPEED_JTAG_CTRL);

	/* wait for reset complete */
	for (i = 0; i < ASPEED_2600_RESET_WAIT_COUNT; i++) {
		mdelay(5);
		val = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_CTRL);
		status = val & ASPEED_JTAG_CTL_FORCE_TMS;
		if (status == 0)
			break;
	}

	/* clear interrupt status */
	aspeed_jtag_write(aspeed_jtag,	ASPEED_JTAG_ISR_INST_PAUSE |
					ASPEED_JTAG_ISR_INST_COMPLETE |
					ASPEED_JTAG_ISR_DATA_PAUSE |
					ASPEED_JTAG_ISR_DATA_COMPLETE,
					ASPEED_JTAG_ISR);

	if (status) {
		/* reset incomplete */
		return -EINVAL;
	}

	return status;
}

static inline void aspeed_jtag_slave(struct aspeed_jtag *aspeed_jtag)
{
	u32 scu_reg;
	if (aspeed_jtag->scu_clear_reg) {
		writel(ASPEED_SCU_RESET_JTAG, aspeed_jtag->scu_base);
	} else {
		scu_reg = readl(aspeed_jtag->scu_base);
		writel(scu_reg | ASPEED_SCU_RESET_JTAG, aspeed_jtag->scu_base);
	}
}

static inline void aspeed_jtag_master(struct aspeed_jtag *aspeed_jtag)
{
	u32 scu_reg;
	u32 val;
	if (aspeed_jtag->scu_clear_reg) {
		writel(ASPEED_SCU_RESET_JTAG,
				aspeed_jtag->scu_base +
				ASPEED_2600_SCU_CLEAR_REGISTER);
	} else {
		scu_reg = readl(aspeed_jtag->scu_base);
		writel(scu_reg & ~ASPEED_SCU_RESET_JTAG,
				aspeed_jtag->scu_base);
	}

	if (aspeed_jtag->scupin_ctrl) {
		val = readl(aspeed_jtag->scupin_ctrl);
		writel((val |
					ASPEED_2600_SCU_ENABLE_PIN_TDI |
					ASPEED_2600_SCU_ENABLE_PIN_TMS |
					ASPEED_2600_SCU_ENABLE_PIN_TCK |
					ASPEED_2600_SCU_ENABLE_PIN_TDO) &
					~ASPEED_2600_SCU_ENABLE_PIN_TRSTN,
				aspeed_jtag->scupin_ctrl);
	}

	aspeed_jtag_write(aspeed_jtag, (ASPEED_JTAG_CTL_ENG_EN |
					ASPEED_JTAG_CTL_ENG_OUT_EN),
					ASPEED_JTAG_CTRL);

	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_MODE_EN |
			ASPEED_JTAG_SW_MODE_TDIO,
			ASPEED_JTAG_SW);
	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_ISR_INST_PAUSE |
			ASPEED_JTAG_ISR_INST_COMPLETE |
			ASPEED_JTAG_ISR_DATA_PAUSE |
			ASPEED_JTAG_ISR_DATA_COMPLETE |
			ASPEED_JTAG_ISR_INST_PAUSE_EN |
			ASPEED_JTAG_ISR_INST_COMPLETE_EN |
			ASPEED_JTAG_ISR_DATA_PAUSE_EN |
			ASPEED_JTAG_ISR_DATA_COMPLETE_EN,
			ASPEED_JTAG_ISR);  /* Enable Interrupt */
}

static void aspeed_jtag_2600_reset_master(struct aspeed_jtag *aspeed_jtag)
{
	/* Reset JTAG master controller */
	writel(ASPEED_SCU_RESET_JTAG, aspeed_jtag->scu_base);

	mdelay(1);

	/* Enable JTAG master mode */
	writel(ASPEED_SCU_RESET_JTAG, aspeed_jtag->scu_base +
				ASPEED_2600_SCU_CLEAR_REGISTER);

	mdelay(1);
}

static int aspeed_jtag_2600_HW_Mode(struct aspeed_jtag *aspeed_jtag)
{
	struct jtag *jtag;
	int status = 0;

	/* Clear ASPEED_JTAG_CTRL register */
	aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_CTRL);

	/* Disable interrupts */
	aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_ISR);

	/* Reset JTAG master controller */
	aspeed_jtag_2600_reset_master(aspeed_jtag);

	/* disable SW mode */
	aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW);

	/* set expected TCK frequency */
	jtag = dev_get_drvdata(aspeed_jtag->dev);
	aspeed_jtag_freq_set(jtag, aspeed_jtag->freq);

	/* enable JTAG master pins */
	aspeed_jtag_2600_master_pins_enable(aspeed_jtag);

	/* reset state machine */
	status = aspeed_jtag_2600_reset_state_machine(aspeed_jtag);

	return status;
}

static int aspeed_jtag_mode_set(struct jtag *jtag, struct jtag_mode *jtag_mode)
{
	struct aspeed_jtag *aspeed_jtag = jtag_priv(jtag);

	switch (jtag_mode->feature) {
	case JTAG_XFER_MODE:
		aspeed_jtag->mode = jtag_mode->mode;

		if (jtag_mode->mode == JTAG_XFER_HW_MODE) {
			if (of_device_is_compatible(aspeed_jtag->dev->of_node,
						"aspeed,ast2600-jtag")) {
				aspeed_jtag_2600_HW_Mode(aspeed_jtag);
				aspeed_jtag->use_irq = 1;
			} else {
				aspeed_jtag->use_irq = 0;
			}
		} else {
			aspeed_jtag->use_irq = 0;
		}
		break;
	case JTAG_CONTROL_MODE:
		if (jtag_mode->mode == JTAG_SLAVE_MODE)
			aspeed_jtag_slave(aspeed_jtag);
		else if (jtag_mode->mode == JTAG_MASTER_MODE)
			aspeed_jtag_master(aspeed_jtag);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static char aspeed_jtag_tck_cycle(struct aspeed_jtag *aspeed_jtag,
				  u8 tms, u8 tdi)
{
	char tdo = 0;

	/* TCK = 0 */
	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_MODE_EN |
			(tms * ASPEED_JTAG_SW_MODE_TMS) |
			(tdi * ASPEED_JTAG_SW_MODE_TDIO), ASPEED_JTAG_SW);

	aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW);

	/* TCK = 1 */
	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_MODE_EN |
			ASPEED_JTAG_SW_MODE_TCK |
			(tms * ASPEED_JTAG_SW_MODE_TMS) |
			(tdi * ASPEED_JTAG_SW_MODE_TDIO), ASPEED_JTAG_SW);

	if (aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) &
		ASPEED_JTAG_SW_MODE_TDIO)
		tdo = 1;

	return tdo;
}

static int aspeed_jtag_bitbang(struct jtag *jtag,
					struct tck_bitbang *tck_bitbang)
{
	struct aspeed_jtag *aspeed_jtag = jtag_priv(jtag);

	tck_bitbang->tdo = aspeed_jtag_tck_cycle(aspeed_jtag,
						tck_bitbang->tms,
						tck_bitbang->tdi);
	return 0;
}

static int aspeed_jtag_wait_instruction_pause(struct aspeed_jtag *aspeed_jtag)
{
	int res = 0;

	if ((aspeed_jtag->use_irq) && (aspeed_jtag->irq > 0)) {
		res = wait_event_interruptible(aspeed_jtag->jtag_wq,
						aspeed_jtag->flag &
						ASPEED_JTAG_ISR_INST_PAUSE);
		aspeed_jtag->flag &= ~ASPEED_JTAG_ISR_INST_PAUSE;

	} else {
		u32 status = 0;
		u32 iterations = 0;

		while ((status & ASPEED_JTAG_ISR_INST_PAUSE) == 0) {
			status = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_ISR);
			iterations++;
			if (iterations > WAIT_ITERATIONS) {
				res = -EFAULT;
				break;
			}
			if ((status & ASPEED_JTAG_ISR_DATA_COMPLETE) == 0) {
				if (iterations % 25 == 0)
					usleep_range(1, 5);
				else
					udelay(1);
			}
		}
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_ISR_INST_PAUSE |
						(status & 0xf),
						ASPEED_JTAG_ISR);
	}

	return res;
}

static int
aspeed_jtag_wait_instruction_complete(struct aspeed_jtag *aspeed_jtag)
{
	int res = 0;

	if ((aspeed_jtag->use_irq) && (aspeed_jtag->irq > 0)) {
		res = wait_event_interruptible(aspeed_jtag->jtag_wq,
						aspeed_jtag->flag &
						ASPEED_JTAG_ISR_INST_COMPLETE);
		aspeed_jtag->flag &= ~ASPEED_JTAG_ISR_INST_COMPLETE;

	} else {
		u32 status = 0;
		u32 iterations = 0;

		while ((status & ASPEED_JTAG_ISR_INST_COMPLETE) == 0) {
			status = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_ISR);
			iterations++;
			if (iterations > WAIT_ITERATIONS) {
				res = -EFAULT;
				break;
			}
			if ((status & ASPEED_JTAG_ISR_DATA_COMPLETE) == 0) {
				if (iterations % 25 == 0)
					usleep_range(1, 5);
				else
					udelay(1);
			}
		}
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_ISR_INST_COMPLETE |
						(status & 0xf),
						ASPEED_JTAG_ISR);
	}
	return res;
}

static int
aspeed_jtag_wait_data_pause_complete(struct aspeed_jtag *aspeed_jtag)
{
	int res = 0;

	if ((aspeed_jtag->use_irq) && (aspeed_jtag->irq > 0)) {
		res = wait_event_interruptible(aspeed_jtag->jtag_wq,
						aspeed_jtag->flag &
						ASPEED_JTAG_ISR_DATA_PAUSE);
		aspeed_jtag->flag &= ~ASPEED_JTAG_ISR_DATA_PAUSE;

	} else {
		u32 status = 0;
		u32 iterations = 0;

		while ((status & ASPEED_JTAG_ISR_DATA_PAUSE) == 0) {
			status = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_ISR);
			iterations++;
			if (iterations > WAIT_ITERATIONS) {
				res = -EFAULT;
				break;
			}
			if ((status & ASPEED_JTAG_ISR_DATA_COMPLETE) == 0) {
				if (iterations % 25 == 0)
					usleep_range(1, 5);
				else
					udelay(1);
			}
		}
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_ISR_DATA_PAUSE |
						(status & 0xf),
						ASPEED_JTAG_ISR);
	}
	return res;
}

static int aspeed_jtag_wait_data_complete(struct aspeed_jtag *aspeed_jtag)
{
	int res = 0;

	if ((aspeed_jtag->use_irq) && (aspeed_jtag->irq > 0)) {
		res = wait_event_interruptible(aspeed_jtag->jtag_wq,
						aspeed_jtag->flag &
						ASPEED_JTAG_ISR_DATA_COMPLETE);
		aspeed_jtag->flag &= ~ASPEED_JTAG_ISR_DATA_COMPLETE;

	} else {
		u32 status = 0;
		u32 iterations = 0;

		while ((status & ASPEED_JTAG_ISR_DATA_COMPLETE) == 0) {
			status = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_ISR);
			iterations++;
			if (iterations > WAIT_ITERATIONS) {
				res = -EFAULT;
				break;
			}
			if ((status & ASPEED_JTAG_ISR_DATA_COMPLETE) == 0) {
				if (iterations % 25 == 0)
					usleep_range(1, 5);
				else
					udelay(1);
			}
		}
		aspeed_jtag_write(aspeed_jtag,
					ASPEED_JTAG_ISR_DATA_COMPLETE |
					(status & 0xf),
					ASPEED_JTAG_ISR);
	}

	return res;
}

static void aspeed_jtag_set_tap_state(struct aspeed_jtag *aspeed_jtag,
						enum jtag_endstate endstate)
{
	int i = 0;
	enum jtag_endstate from, to;

	from = aspeed_jtag->status;
	to = endstate;
	for (i = 0; i < _tms_cycle_lookup[from][to].count; i++)
		aspeed_jtag_tck_cycle(aspeed_jtag,
			((_tms_cycle_lookup[from][to].tmsbits >> i) & 0x1), 0);
	aspeed_jtag->status = endstate;
}

static void aspeed_jtag_end_tap_state_sw(struct aspeed_jtag *aspeed_jtag,
					struct jtag_end_tap_state *endstate)
{
	/* SW mode from curent tap state -> to end_state */
	if (endstate->reset) {
		int i = 0;

		for (i = 0; i < ASPEED_JTAG_RESET_CNTR; i++)
			aspeed_jtag_tck_cycle(aspeed_jtag, 1, 0);
		aspeed_jtag->status = JTAG_STATE_TLRESET;
	}

	aspeed_jtag_set_tap_state(aspeed_jtag, endstate->endstate);
}

static int aspeed_jtag_status_set(struct jtag *jtag,
					struct jtag_end_tap_state *endstate)
{
	struct aspeed_jtag *aspeed_jtag = jtag_priv(jtag);

	if (!(aspeed_jtag->mode & JTAG_XFER_HW_MODE)) {
		aspeed_jtag_end_tap_state_sw(aspeed_jtag, endstate);
		return 0;
	} else {
		/* AST2600 JTAG HW mode */
		if (of_device_is_compatible(aspeed_jtag->dev->of_node,
						"aspeed,ast2600-jtag")) {
			if (endstate->reset) {
				aspeed_jtag_2600_reset_state_machine(
						aspeed_jtag);
				aspeed_jtag->status = JTAG_STATE_TLRESET;
			} else {
				aspeed_jtag->status = endstate->endstate;
			}
			return 0;
		}
	}

	/* x TMS high + 1 TMS low */
	if (endstate->reset) {
		/* Disable sw mode */
		aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW);
		mdelay(1);
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_CTL_ENG_EN |
				ASPEED_JTAG_CTL_ENG_OUT_EN |
				ASPEED_JTAG_CTL_FORCE_TMS, ASPEED_JTAG_CTRL);
		mdelay(1);
		aspeed_jtag_write(aspeed_jtag,
					ASPEED_JTAG_SW_TDIO, ASPEED_JTAG_SW);
		aspeed_jtag->status = JTAG_STATE_TLRESET;
	}

	return 0;
}

static void aspeed_jtag_xfer_sw(struct aspeed_jtag *aspeed_jtag,
				struct jtag_xfer *xfer, u32 *data)
{
	unsigned long remain_xfer = xfer->length;
	unsigned long shift_bits = 0;
	unsigned long index = 0;
	unsigned long tdi;
	char tdo;

	if (xfer->type == JTAG_SIR_XFER)
		aspeed_jtag_set_tap_state(aspeed_jtag, JTAG_STATE_SHIFTIR);
	else
		aspeed_jtag_set_tap_state(aspeed_jtag, JTAG_STATE_SHIFTDR);

	tdi = ASPEED_JTAG_GET_TDI(xfer->direction, data[index]);
	data[index] = 0;
	while (remain_xfer > 1) {
		tdo = aspeed_jtag_tck_cycle(aspeed_jtag, 0,
						tdi & ASPEED_JTAG_DATA_MSB);
		data[index] |= tdo << (shift_bits %
						ASPEED_JTAG_DATA_CHUNK_SIZE);
		tdi >>= 1;
		shift_bits++;
		remain_xfer--;

		if (shift_bits % ASPEED_JTAG_DATA_CHUNK_SIZE == 0) {
			tdo = 0;
			index++;
			tdi = ASPEED_JTAG_GET_TDI(xfer->direction, data[index]);
			data[index] = 0;
		}
	}

	if ((xfer->endstate == (xfer->type == JTAG_SIR_XFER ?
				JTAG_STATE_SHIFTIR : JTAG_STATE_SHIFTDR))) {
		/* Stay in Shift IR/DR*/
		tdo = aspeed_jtag_tck_cycle(aspeed_jtag, 0,
						tdi & ASPEED_JTAG_DATA_MSB);
		data[index] |= tdo << (shift_bits %
					ASPEED_JTAG_DATA_CHUNK_SIZE);
	} else  {
		/* Goto end state */
		tdo = aspeed_jtag_tck_cycle(aspeed_jtag, 1,
						tdi & ASPEED_JTAG_DATA_MSB);
		data[index] |= tdo << (shift_bits %
					ASPEED_JTAG_DATA_CHUNK_SIZE);
		aspeed_jtag->status = (xfer->type == JTAG_SIR_XFER) ?
					JTAG_STATE_EXIT1IR : JTAG_STATE_EXIT1DR;
		aspeed_jtag_set_tap_state(aspeed_jtag, xfer->endstate);
	}
}

static int aspeed_jtag_xfer_push_data(struct aspeed_jtag *aspeed_jtag,
					enum jtag_xfer_type type, u32 bits_len)
{
	int res = 0;

	if (type == JTAG_SIR_XFER) {
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_IOUT_LEN(bits_len),
				ASPEED_JTAG_CTRL);
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_IOUT_LEN(bits_len) |
				ASPEED_JTAG_CTL_INST_EN, ASPEED_JTAG_CTRL);
		res = aspeed_jtag_wait_instruction_pause(aspeed_jtag);
	} else {
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_DOUT_LEN(bits_len),
				ASPEED_JTAG_CTRL);
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_DOUT_LEN(bits_len) |
				ASPEED_JTAG_CTL_DATA_EN, ASPEED_JTAG_CTRL);
		res = aspeed_jtag_wait_data_pause_complete(aspeed_jtag);
	}
	return res;
}

static int aspeed_jtag_xfer_push_data_last(struct aspeed_jtag *aspeed_jtag,
						enum jtag_xfer_type type,
						u32 shift_bits,
						enum jtag_endstate endstate)
{
	int res = 0;

	if (type == JTAG_SIR_XFER) {
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_IOUT_LEN(shift_bits) |
				ASPEED_JTAG_CTL_LASPEED_INST,
				ASPEED_JTAG_CTRL);
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_IOUT_LEN(shift_bits) |
				ASPEED_JTAG_CTL_LASPEED_INST |
				ASPEED_JTAG_CTL_INST_EN,
				ASPEED_JTAG_CTRL);
		res = aspeed_jtag_wait_instruction_complete(aspeed_jtag);
	} else {
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_DOUT_LEN(shift_bits) |
				ASPEED_JTAG_CTL_LASPEED_DATA,
				ASPEED_JTAG_CTRL);
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_DOUT_LEN(shift_bits) |
				ASPEED_JTAG_CTL_LASPEED_DATA |
				ASPEED_JTAG_CTL_DATA_EN,
				ASPEED_JTAG_CTRL);
		res = aspeed_jtag_wait_data_complete(aspeed_jtag);
	}
	return res;
}

static int aspeed_jtag_xfer_hw(struct aspeed_jtag *aspeed_jtag,
					struct jtag_xfer *xfer, u32 *data)
{
	unsigned long remain_xfer = xfer->length;
	unsigned long index = 0;
	char shift_bits;
	u32 data_reg;
	u32 scan_end;
	u32 t1 = 0;
	u32 t2 = 0;

	spin_lock(&JTAG_SPINLOCK);

	data_reg = xfer->type == JTAG_SIR_XFER ?
			ASPEED_JTAG_INST : ASPEED_JTAG_DATA;
	if (xfer->endstate == JTAG_STATE_SHIFTIR ||
		xfer->endstate == JTAG_STATE_SHIFTDR ||
		xfer->endstate == JTAG_STATE_PAUSEIR ||
		xfer->endstate == JTAG_STATE_PAUSEDR) {
		scan_end = 0;
	} else {
		scan_end = 1;
	}

	while (remain_xfer) {
		if (xfer->direction & JTAG_WRITE_XFER)
			aspeed_jtag_write(aspeed_jtag, data[index], data_reg);
		else
			aspeed_jtag_write(aspeed_jtag, 0, data_reg);

		if (remain_xfer > ASPEED_JTAG_DATA_CHUNK_SIZE) {
			shift_bits = ASPEED_JTAG_DATA_CHUNK_SIZE;

			/*
			 * Read bytes were not equals to column length
			 * and continue in Shift IR/DR
			 */
			if (aspeed_jtag_xfer_push_data(aspeed_jtag, xfer->type,
							shift_bits) != 0) {
				spin_unlock(&JTAG_SPINLOCK);
				return -EFAULT;
			}
		} else {
			/*
			 * Read bytes equals to column length
			 */
			shift_bits = remain_xfer;
			if (scan_end) {
				/*
				 * If this data is the end of the transmission
				 * send remaining bits and go to endstate
				 */
				if (aspeed_jtag_xfer_push_data_last(
							aspeed_jtag,
							xfer->type,
							shift_bits,
							xfer->endstate) != 0) {
					spin_unlock(&JTAG_SPINLOCK);
					return -EFAULT;
				}
			} else {
				/*
				 * If transmission is waiting for additional
				 * data send remaining bits and stay in
				 * SHIFT IR/DR
				 */
				if (aspeed_jtag_xfer_push_data(aspeed_jtag,
								xfer->type,
								shift_bits)
								!= 0) {
					spin_unlock(&JTAG_SPINLOCK);
					return -EFAULT;
				}
			}
		}

		if (xfer->direction & JTAG_READ_XFER) {
			/* calculate wait time */
			t1 = (ASPEED_NS_CONSTANT / aspeed_jtag->freq) *
				shift_bits;
			t2 = t1 / ASPEED_MS_CONSTANT;
			if (t1 % ASPEED_NS_CONSTANT)
				t2++;

			t2 += ASPEED_JTAG_TCK_WAIT;
			mdelay(t2);

			if (shift_bits < ASPEED_JTAG_DATA_CHUNK_SIZE) {
				data[index] = aspeed_jtag_read(aspeed_jtag,
								data_reg);

				data[index] >>= ASPEED_JTAG_DATA_CHUNK_SIZE -
								shift_bits;
			} else {
				data[index] = aspeed_jtag_read(aspeed_jtag,
								data_reg);
			}
		}

		remain_xfer = remain_xfer - shift_bits;
		index++;
	}

	aspeed_jtag->status = xfer->endstate;
	spin_unlock(&JTAG_SPINLOCK);

	return 0;
}

static int aspeed_jtag_2600_xfer_push_data(struct aspeed_jtag *aspeed_jtag,
		enum jtag_xfer_type type, u32 bits_len)
{
	int res = 0;

	if (type == JTAG_SIR_XFER) {	/* Instruction Type */
		/* enable instruction TX pause interrupt */
		aspeed_jtag_write(aspeed_jtag,	ASPEED_JTAG_ISR_INST_PAUSE_EN,
						ASPEED_JTAG_ISR);

		/* write instruction length */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(bits_len),
				ASPEED_JTAG_CTRL);

#ifdef ENABLE_FIFO_CTRL
		/* switch FIFO to controller mode */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(bits_len) |
				ASPEED_JTAG_CTL_2600_FIFO_MODE,
				ASPEED_JTAG_CTRL);

		mdelay(1);

		/* enable tx of instruction */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(bits_len) |
				ASPEED_JTAG_CTL_2600_FIFO_MODE |
				ASPEED_JTAG_CTL_2600_INST_EN,
				ASPEED_JTAG_CTRL);
#else
		/* enable tx of instruction */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(bits_len) |
				ASPEED_JTAG_CTL_2600_INST_EN,
				ASPEED_JTAG_CTRL);
#endif

		/* wait for instruction tx completion */
		res = aspeed_jtag_wait_instruction_pause(aspeed_jtag);

	} else {	/* Data Type */
		/* enable data TX pause interrupt */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_ISR_DATA_PAUSE_EN,
				ASPEED_JTAG_ISR);

		/* write data length */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(bits_len),
				ASPEED_JTAG_CTRL);

#ifdef ENABLE_FIFO_CTRL
		/* switch FIFO to controller mode */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(bits_len) |
				ASPEED_JTAG_CTL_2600_FIFO_MODE,
				ASPEED_JTAG_CTRL);

		mdelay(1);

		/* enable tx of instruction */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(bits_len) |
				ASPEED_JTAG_CTL_2600_FIFO_MODE |
				ASPEED_JTAG_CTL_DATA_EN,
				ASPEED_JTAG_CTRL);
#else
		/* enable tx of instruction */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(bits_len) |
				ASPEED_JTAG_CTL_DATA_EN,
				ASPEED_JTAG_CTRL);
#endif

		/* wait for data tx completion */
		res = aspeed_jtag_wait_data_pause_complete(aspeed_jtag);
	}

	return res;
}

static int aspeed_jtag_2600_xfer_push_data_last(struct aspeed_jtag *aspeed_jtag,
						enum jtag_xfer_type type,
						u32 shift_bits)
{
	int res = 0;

	if (type == JTAG_SIR_XFER) {
		/* enable instruction tx completion interrupt */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_ISR_INST_COMPLETE_EN,
				ASPEED_JTAG_ISR);

		/* write instruction length */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(shift_bits),
				ASPEED_JTAG_CTRL);

#ifdef ENABLE_FIFO_CTRL
		/* switch FIFO to controller mode */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(shift_bits) |
				ASPEED_JTAG_CTL_2600_FIFO_MODE,
				ASPEED_JTAG_CTRL);

		mdelay(1);

		/* enable last transmission of instruction */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(shift_bits) |
				ASPEED_JTAG_CTL_2600_FIFO_MODE |
				ASPEED_JTAG_CTL_2600_LAST_TX |
				ASPEED_JTAG_CTL_2600_INST_EN,
				ASPEED_JTAG_CTRL);

#else
		/* enable last transmission of instruction */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(shift_bits) |
				ASPEED_JTAG_CTL_2600_LAST_TX |
				ASPEED_JTAG_CTL_2600_INST_EN,
				ASPEED_JTAG_CTRL);
#endif

		/* wait for instruction tx completion */
		res = aspeed_jtag_wait_instruction_complete(aspeed_jtag);

	} else {
		/* enable data tx completion interrupt */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_ISR_DATA_COMPLETE_EN,
				ASPEED_JTAG_ISR);

		/* write data length */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(shift_bits),
				ASPEED_JTAG_CTRL);

#ifdef ENABLE_FIFO_CTRL
		/* switch FIFO to controller mode */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(shift_bits) |
				ASPEED_JTAG_CTL_2600_FIFO_MODE,
				ASPEED_JTAG_CTRL);

		mdelay(1);

		/* enable last transmission of data */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(shift_bits) |
				ASPEED_JTAG_CTL_2600_FIFO_MODE |
				ASPEED_JTAG_CTL_2600_LAST_TX |
				ASPEED_JTAG_CTL_DATA_EN,
				ASPEED_JTAG_CTRL);

#else
		/* enable last transmission of data */
		aspeed_jtag_write(aspeed_jtag,
				ASPEED_JTAG_2600_TX_LEN(shift_bits) |
				ASPEED_JTAG_CTL_2600_LAST_TX |
				ASPEED_JTAG_CTL_DATA_EN,
				ASPEED_JTAG_CTRL);
#endif

		/* wait for data tx completion */
		res = aspeed_jtag_wait_data_complete(aspeed_jtag);
	}
	return res;
}

static int aspeed_jtag_2600_xfer_hw(struct aspeed_jtag *aspeed_jtag,
					struct jtag_xfer *xfer, u32 *data)
{
	u32 remain_xfer = xfer->length;
	u32 index = 0;
	u32 shift_bits = 0;
	u32 data_reg = 0;
	u32 scan_end = 0;
	u32 count = 0;
	u32 rem_bits = 0;
	u32 i = 0;
	u32 t1 = 0;
	u32 t2 = 0;

	spin_lock(&JTAG_SPINLOCK);

	data_reg = xfer->type == JTAG_SIR_XFER ?
			ASPEED_JTAG_INST : ASPEED_JTAG_DATA;

	if (xfer->endstate == JTAG_STATE_SHIFTIR ||
		xfer->endstate == JTAG_STATE_SHIFTDR ||
		xfer->endstate == JTAG_STATE_PAUSEIR ||
		xfer->endstate == JTAG_STATE_PAUSEDR) {
		scan_end = 0;
	} else {
		scan_end = 1;
	}

#ifdef ENABLE_FIFO_CTRL
	/* reset internal FIFO */
	aspeed_jtag_write(aspeed_jtag, (ASPEED_JTAG_CTL_ENG_EN |
					ASPEED_JTAG_CTL_ENG_OUT_EN |
					ASPEED_JTAG_CTL_2600_RST_FIFO),
					ASPEED_JTAG_CTRL);

	mdelay(1);
#endif

	while (remain_xfer) {
		if (remain_xfer > ASPEED_JTAG_512BITS_CHUNK_SIZE) {
			count = ASPEED_JTAG_512BITS_CHUNK_SIZE / 32;
			shift_bits = ASPEED_JTAG_512BITS_CHUNK_SIZE;
		} else {
			count = remain_xfer / 32;
			if (remain_xfer % 32)
				count++;
			shift_bits = remain_xfer;
		}

#ifdef ENABLE_FIFO_CTRL
		/* switch into CPU mode */
		aspeed_jtag_write(aspeed_jtag,
				((ASPEED_JTAG_CTL_ENG_EN |
				ASPEED_JTAG_CTL_ENG_OUT_EN) &
				~ASPEED_JTAG_CTL_2600_FIFO_MODE),
				ASPEED_JTAG_CTRL);

		mdelay(1);
#endif

		/* write data into 32-bit data register */
		for (i = 0; i < count; i++) {
			if (xfer->direction & JTAG_WRITE_XFER) {
				aspeed_jtag_write(aspeed_jtag,
						data[index+i], data_reg);
			} else {
				aspeed_jtag_write(aspeed_jtag, 0, data_reg);
			}
		}
		/* ensure write data into 32-bit data register complete */
		smp_mb();

		if (remain_xfer > ASPEED_JTAG_512BITS_CHUNK_SIZE) {
			/*
			 * If transmission is waiting for additional data,
			 * send remaining bits and stay in SHIFT IR/DR
			 */
			if (aspeed_jtag_2600_xfer_push_data(aspeed_jtag,
					xfer->type, shift_bits) != 0) {
				spin_unlock(&JTAG_SPINLOCK);
				return -EFAULT;
			}
		} else {
			if (scan_end) {
				/*
				 * If this data is the end of the transmission,
				 * send remaining bits and go to endstate
				 */
				if (aspeed_jtag_2600_xfer_push_data_last(
						aspeed_jtag,
						xfer->type,
						shift_bits) != 0) {
					spin_unlock(&JTAG_SPINLOCK);
					return -EFAULT;
				}
			} else {
				/*
				 * If transmission is waiting for additional
				 * data, send remaining bits and stay in
				 * SHIFT IR/DR
				 */
				if (aspeed_jtag_2600_xfer_push_data(
						aspeed_jtag,
						xfer->type,
						shift_bits) != 0) {
					spin_unlock(&JTAG_SPINLOCK);
					return -EFAULT;
				}
			}
		}

		if (xfer->direction & JTAG_READ_XFER) {
			/* calculate wait time */
			t1 = (ASPEED_NS_CONSTANT / aspeed_jtag->freq) *
					shift_bits;
			t2 = t1 / ASPEED_MS_CONSTANT;
			if (t1 % ASPEED_MS_CONSTANT)
				t2++;

			t2 += ASPEED_JTAG_TCK_WAIT;
			mdelay(t2);

#ifdef ENABLE_FIFO_CTRL
			/* switch FIFO into CPU mode */
			aspeed_jtag_write(aspeed_jtag,
					((ASPEED_JTAG_CTL_ENG_EN |
					ASPEED_JTAG_CTL_ENG_OUT_EN) &
					~ASPEED_JTAG_CTL_2600_FIFO_MODE),
					ASPEED_JTAG_CTRL);

			mdelay(1);
#endif
			for (i = 0; i < count; i++) {
				data[index + i] = aspeed_jtag_read(aspeed_jtag,
							data_reg);
			}

			if (shift_bits < ASPEED_JTAG_512BITS_CHUNK_SIZE) {
				rem_bits = shift_bits %
						ASPEED_JTAG_DATA_CHUNK_SIZE;
				if (rem_bits) {
					data[index + i] >>=
						(ASPEED_JTAG_DATA_CHUNK_SIZE -
						rem_bits);
				}
			}
		}

		remain_xfer = remain_xfer - shift_bits;
		index += count;
	}

	aspeed_jtag->status = xfer->endstate;
	spin_unlock(&JTAG_SPINLOCK);

	return 0;
}

static int aspeed_jtag_xfer(struct jtag *jtag, struct jtag_xfer *xfer,
				u8 *xfer_data)
{
	struct aspeed_jtag *aspeed_jtag = jtag_priv(jtag);

	if (!(aspeed_jtag->mode & JTAG_XFER_HW_MODE)) {
		/* SW mode */
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_TDIO,
					ASPEED_JTAG_SW);

		aspeed_jtag_xfer_sw(aspeed_jtag, xfer, (u32 *)xfer_data);
	} else {
		/* HW mode */
		aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW);

		if (of_device_is_compatible(aspeed_jtag->dev->of_node,
					"aspeed,ast2600-jtag")) {
			if (aspeed_jtag_2600_xfer_hw(aspeed_jtag, xfer,
					(u32 *)xfer_data) != 0) {
				return -EFAULT;
			}
		} else {
			if (aspeed_jtag_xfer_hw(aspeed_jtag, xfer,
					(u32 *)xfer_data) != 0) {
				return -EFAULT;
			}
		}
	}

	aspeed_jtag->status = xfer->endstate;
	return 0;
}

static int aspeed_jtag_status_get(struct jtag *jtag, u32 *status)
{
	struct aspeed_jtag *aspeed_jtag = jtag_priv(jtag);

	*status = aspeed_jtag->status;
	return 0;
}

static irqreturn_t aspeed_jtag_interrupt(s32 this_irq, void *dev_id)
{
	struct aspeed_jtag *aspeed_jtag = dev_id;
	irqreturn_t ret = IRQ_HANDLED;
	u32 status;

	status = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_ISR);

	if (status & ASPEED_JTAG_ISR_INT_MASK) {
		aspeed_jtag_write(aspeed_jtag,
					(status & ASPEED_JTAG_ISR_INT_MASK),
					ASPEED_JTAG_ISR);
		aspeed_jtag->flag |= status & ASPEED_JTAG_ISR_INT_MASK;
	}

	if (aspeed_jtag->flag) {
		wake_up_interruptible(&aspeed_jtag->jtag_wq);
		ret = IRQ_HANDLED;
	} else {
		dev_err(aspeed_jtag->dev, "irq status:%x\n",
			status);
		ret = IRQ_NONE;
	}
	return ret;
}

static int aspeed_jtag_enable(struct jtag *jtag)
{
	struct aspeed_jtag *aspeed_jtag = jtag_priv(jtag);

	aspeed_jtag_master(aspeed_jtag);
	return 0;
}

static int aspeed_jtag_disable(struct jtag *jtag)
{
	struct aspeed_jtag *aspeed_jtag = jtag_priv(jtag);

	aspeed_jtag_slave(aspeed_jtag);
	return 0;
}

static int aspeed_jtag_init(struct platform_device *pdev,
				struct aspeed_jtag *aspeed_jtag)
{
	struct resource *res;
	struct resource *scu_res;
	struct resource *scupin_res;
	struct jtag *jtag;
	int err = 0;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(aspeed_jtag->dev, "Failed to get IORESOURCE_MEM.\n");
		ret = -ENOENT;
		goto out;
	}

	aspeed_jtag->reg_base = devm_ioremap_resource(aspeed_jtag->dev, res);
	if (IS_ERR(aspeed_jtag->reg_base)) {
		ret = -ENOMEM;
		goto out;
	}

	scu_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (scu_res == NULL) {
		ret = -ENOENT;
		goto out;
	}

	aspeed_jtag->scu_base = devm_ioremap_resource(aspeed_jtag->dev,
					scu_res);
	if (IS_ERR(aspeed_jtag->scu_base)) {
		ret = -ENOMEM;
		goto out;
	}

	if (of_device_is_compatible(pdev->dev.of_node,
					"aspeed,ast2600-jtag")) {
		scupin_res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (scu_res == NULL) {
			ret = -ENOENT;
			goto out;
		}

		aspeed_jtag->scupin_ctrl =
			devm_ioremap_resource(aspeed_jtag->dev, scupin_res);
		if (IS_ERR(aspeed_jtag->scupin_ctrl)) {
			ret = -ENOMEM;
			goto out;
		}

		aspeed_jtag->scu_clear_reg = 1;

		aspeed_jtag->irq = platform_get_irq(pdev, 0);
		if (aspeed_jtag->irq < 0) {
			dev_err(aspeed_jtag->dev, "no irq specified\n");
			ret = -ENOENT;
			goto out;
		}

		aspeed_jtag->use_irq = 0;
	} else {
		aspeed_jtag->scupin_ctrl = NULL;
		aspeed_jtag->scu_clear_reg = 0;
		aspeed_jtag->irq = -1;
		aspeed_jtag->use_irq = 0;
	}

	aspeed_jtag->pclk = devm_clk_get(aspeed_jtag->dev, NULL);
	if (IS_ERR(aspeed_jtag->pclk)) {
		ret = -ENOENT;
		goto out;
	}

	if (clk_prepare_enable(aspeed_jtag->pclk)) {
		ret = -ENOENT;
		goto out;
	}

	aspeed_jtag->rst = devm_reset_control_get_shared(&pdev->dev, NULL);
	if (IS_ERR(aspeed_jtag->rst)) {
		ret = -ENOENT;
		goto out_clk;
	}

	reset_control_deassert(aspeed_jtag->rst);

	if (aspeed_jtag->irq > 0) {
		err = devm_request_irq(aspeed_jtag->dev, aspeed_jtag->irq,
					aspeed_jtag_interrupt, 0,
					"aspeed-jtag", aspeed_jtag);

		if (err) {
			ret = -ENOENT;
			goto out_clk;
		}
	}

	aspeed_jtag_slave(aspeed_jtag);

	if (of_device_is_compatible(pdev->dev.of_node,
					"aspeed,ast2600-jtag")) {
		/* set expected TCK frequency */
		jtag = dev_get_drvdata(aspeed_jtag->dev);
		aspeed_jtag->freq = ASPEED_2600_JTAG_MASTER2_FREQ;
		aspeed_jtag_freq_set(jtag, aspeed_jtag->freq);
	}

	aspeed_jtag->flag = 0;
	aspeed_jtag->mode = 0;
	init_waitqueue_head(&aspeed_jtag->jtag_wq);

	return 0;

out_clk:
	if (aspeed_jtag->pclk)
		clk_disable_unprepare(aspeed_jtag->pclk);
out:
	return ret;
}

#ifdef CONFIG_JTAG_ASPEED_LEGACY_UIO
static u32 g_sw_tdi;
static u32 g_sw_tck;
static u32 g_sw_tms;

#define JTAG_SW_MODE_VAL_MASK	(ASPEED_JTAG_SW_MODE_TDIO | \
			ASPEED_JTAG_SW_MODE_TCK | ASPEED_JTAG_SW_MODE_TMS)

static ssize_t show_tdo(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct jtag *jtag = dev_get_drvdata(dev);
	struct aspeed_jtag *ast_jtag = jtag_priv(jtag);
	u32 val = aspeed_jtag_read(ast_jtag, ASPEED_JTAG_SW);

	return sprintf(buf, "%s\n", (val & ASPEED_JTAG_SW_MODE_TDIO) ?
				"1" : "0");
}

static DEVICE_ATTR(tdo, S_IRUGO, show_tdo, NULL);

static void aspeed_jtag_write_sw_reg(struct device *dev)
{
	struct jtag *jtag = dev_get_drvdata(dev);
	struct aspeed_jtag *ast_jtag = jtag_priv(jtag);
	u32 old_val = aspeed_jtag_read(ast_jtag, ASPEED_JTAG_SW);
	u32 new_val = (old_val & ~JTAG_SW_MODE_VAL_MASK) |
				(g_sw_tdi | g_sw_tck | g_sw_tms);

	aspeed_jtag_write(ast_jtag, new_val, ASPEED_JTAG_SW);
}

#define STORE_COMMON(dev, buf, count, tdx, true_value) do {	\
	unsigned long val;					\
	int err;						\
	err = kstrtoul(buf, 0, &val);				\
	if (err)						\
		return err;					\
	tdx = val ? true_value : 0;				\
	aspeed_jtag_write_sw_reg(dev);				\
	return count;						\
} while (0);

static ssize_t store_tdi(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	STORE_COMMON(dev, buf, count, g_sw_tdi, ASPEED_JTAG_SW_MODE_TDIO);
}

static DEVICE_ATTR(tdi, S_IWUSR, NULL, store_tdi);

static ssize_t store_tms(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	STORE_COMMON(dev, buf, count, g_sw_tms, ASPEED_JTAG_SW_MODE_TMS);
}

static DEVICE_ATTR(tms, S_IWUSR, NULL, store_tms);

static ssize_t store_tck(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	STORE_COMMON(dev, buf, count, g_sw_tck, ASPEED_JTAG_SW_MODE_TCK);
}

static DEVICE_ATTR(tck, S_IWUSR, NULL, store_tck);

static ssize_t show_sts(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct jtag *jtag = dev_get_drvdata(dev);
	struct aspeed_jtag *ast_jtag = jtag_priv(jtag);

	/*
	 * NOTE: not all the defined states are supported, and this is
	 * to make sure kernel ABI is consistent with old kernel.
	 */
	switch (ast_jtag->status) {
	case JTAG_STATE_IDLE:
	case JTAG_STATE_PAUSEIR:
	case JTAG_STATE_PAUSEDR:
		return sprintf(buf, "%s\n", end_status_str[ast_jtag->status]);

	default:
		break;
	}

	return sprintf(buf, "ERROR\n");
}

static DEVICE_ATTR(sts, S_IRUGO, show_sts, NULL);

static ssize_t show_frequency(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u32 frq;
	struct jtag *jtag = dev_get_drvdata(dev);

	aspeed_jtag_freq_get(jtag, &frq);

	return sprintf(buf, "Frequency : %d\n", frq);
}

static ssize_t store_frequency(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	struct jtag *jtag = dev_get_drvdata(dev);
	int err;
	err = kstrtoul(buf, 0, &val);
	if (err)
		return err;
	aspeed_jtag_freq_set(jtag, val);

	return count;
}

static DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, show_frequency, store_frequency);

static struct attribute *ast_jtag_sysfs_entries[] = {
	&dev_attr_freq.attr,
	&dev_attr_sts.attr,
	&dev_attr_tck.attr,
	&dev_attr_tms.attr,
	&dev_attr_tdi.attr,
	&dev_attr_tdo.attr,
	NULL
};

static struct attribute_group ast_jtag_attr_group = {
	.attrs = ast_jtag_sysfs_entries,
};

struct run_cycle_param {
	unsigned char tdi;
	unsigned char tms;
	unsigned char tck;
	unsigned char tdo;
};
#define JTAG_RUN_CYCLE _IOR(__JTAG_IOCTL_MAGIC, 11, struct run_cycle_param)

static void aspeed_jtag_run_cycle(struct jtag *jtag,
				  struct run_cycle_param *run_cycle)
{
	u32 new_val;
	struct aspeed_jtag *ast_jtag = jtag_priv(jtag);
	u32 old_val = aspeed_jtag_read(ast_jtag, ASPEED_JTAG_SW);

	g_sw_tdi = run_cycle->tdi ? ASPEED_JTAG_SW_MODE_TDIO : 0;
	g_sw_tms = run_cycle->tms ? ASPEED_JTAG_SW_MODE_TMS : 0;
	g_sw_tck = run_cycle->tck ? ASPEED_JTAG_SW_MODE_TCK : 0;
	new_val = (old_val & ~JTAG_SW_MODE_VAL_MASK) |
		  (g_sw_tdi | g_sw_tck | g_sw_tms);
	aspeed_jtag_write(ast_jtag, new_val, ASPEED_JTAG_SW);

	new_val = aspeed_jtag_read(ast_jtag, ASPEED_JTAG_SW);
	run_cycle->tdo = (new_val & ASPEED_JTAG_SW_MODE_TDIO) ? 1 : 0;
}

static int aspeed_jtag_ioctl(struct jtag *jtag, unsigned int cmd,
					unsigned long arg)
{
	int err = 0;
	struct run_cycle_param jtag_run_cycle;

	if (!arg)
		return -EINVAL;

	switch (cmd) {
	case JTAG_RUN_CYCLE:
		if (copy_from_user(&jtag_run_cycle, (void __user*)arg,
				sizeof(struct run_cycle_param))) {
			err = -EFAULT;
			break;
		}

		aspeed_jtag_run_cycle(jtag, &jtag_run_cycle);

		if (copy_to_user((void __user*)(arg), &jtag_run_cycle,
				sizeof(struct run_cycle_param)))
			err = -EFAULT;

		break;

	default:
		return -EINVAL;
	}

	return err;
}

#endif /* CONFIG_JTAG_ASPEED_LEGACY_UIO */

static int aspeed_jtag_deinit(struct platform_device *pdev,
					struct aspeed_jtag *aspeed_jtag)
{
#ifdef CONFIG_JTAG_ASPEED_LEGACY_UIO
	sysfs_remove_group(&pdev->dev.kobj, &ast_jtag_attr_group);
#endif
	aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_ISR);
	/* Disable clock */
	aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_CTRL);
	reset_control_assert(aspeed_jtag->rst);
	clk_disable_unprepare(aspeed_jtag->pclk);

	if (aspeed_jtag->irq > 0)
		devm_free_irq(aspeed_jtag->dev, aspeed_jtag->irq, aspeed_jtag);

	return 0;
}

static const struct jtag_ops aspeed_jtag_ops = {
	.freq_get = aspeed_jtag_freq_get,
	.freq_set = aspeed_jtag_freq_set,
	.status_get = aspeed_jtag_status_get,
	.status_set = aspeed_jtag_status_set,
	.xfer = aspeed_jtag_xfer,
	.mode_set = aspeed_jtag_mode_set,
	.bitbang = aspeed_jtag_bitbang,
	.enable = aspeed_jtag_enable,
	.disable = aspeed_jtag_disable,
#ifdef CONFIG_JTAG_ASPEED_LEGACY_UIO
	.ioctl = aspeed_jtag_ioctl,
#endif
};

static int aspeed_jtag_probe(struct platform_device *pdev)
{
	struct aspeed_jtag *aspeed_jtag;
	struct jtag *jtag;
	int err;

	jtag = jtag_alloc(&pdev->dev, sizeof(*aspeed_jtag), &aspeed_jtag_ops);
	if (!jtag)
		return -ENOMEM;

	platform_set_drvdata(pdev, jtag);
	aspeed_jtag = jtag_priv(jtag);
	aspeed_jtag->dev = &pdev->dev;

	/* Initialize device*/
	err = aspeed_jtag_init(pdev, aspeed_jtag);
	if (err)
		goto err_jtag_init;

	/* Initialize JTAG core structure*/
	err = devm_jtag_register(aspeed_jtag->dev, jtag);
	if (err)
		goto err_jtag_register;

#ifdef CONFIG_JTAG_ASPEED_LEGACY_UIO
	err = sysfs_create_group(&pdev->dev.kobj, &ast_jtag_attr_group);
	if (err)
		goto err_jtag_register;
#endif /* CONFIG_JTAG_ASPEED_LEGACY_UIO */

	return 0;

err_jtag_register:
	aspeed_jtag_deinit(pdev, aspeed_jtag);
err_jtag_init:
	jtag_free(jtag);
	return err;
}

static int aspeed_jtag_remove(struct platform_device *pdev)
{
	struct jtag *jtag = platform_get_drvdata(pdev);

	aspeed_jtag_deinit(pdev, jtag_priv(jtag));
	return 0;
}

static const struct of_device_id aspeed_jtag_of_match[] = {
	{ .compatible = "aspeed,ast2400-jtag", },
	{ .compatible = "aspeed,ast2500-jtag", },
	{ .compatible = "aspeed,ast2600-jtag", },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_jtag_of_match);

static struct platform_driver aspeed_jtag_driver = {
	.probe = aspeed_jtag_probe,
	.remove = aspeed_jtag_remove,
	.driver = {
		.name = ASPEED_JTAG_NAME,
		.of_match_table = aspeed_jtag_of_match,
	},
};
module_platform_driver(aspeed_jtag_driver);

MODULE_AUTHOR("Oleksandr Shamray <oleksandrs@mellanox.com>");
MODULE_DESCRIPTION("ASPEED JTAG driver");
MODULE_LICENSE("GPL v2");
