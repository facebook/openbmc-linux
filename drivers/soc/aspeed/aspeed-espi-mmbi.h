// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2020 Aspeed Technology Inc.
 */

#ifndef ASPEED_ESPI_MMBI_H
#define ASPEED_ESPI_MMBI_H //

#define ESPI_MMBI_CTRL			(0x800)
#define ESPI_MMBI_INT_STS		(0x808)
#define ESPI_MMBI_INT_ENL		(0x80C)
#define ESPI_MMBI_RWP_0			(0x810)
#define ESPI_MMBI_RWP_N(x)		(ESPI_MMBI_RWP_0+(x<<3))


typedef enum  {
	ESPI_MMBI_INST_8KB=0,
	ESPI_MMBI_INST_16KB,
	ESPI_MMBI_INST_32KB,
	ESPI_MMBI_INST_64KB,
	ESPI_MMBI_INST_128KB,
	ESPI_MMBI_INST_256KB,
	ESPI_MMBI_INST_512KB,
	ESPI_MMBI_INST_1024KB,
}ESPI_MMBI_INST_SIZE;

typedef enum  {
	ESPI_MMBI_TOTAL_64KB=0,
	ESPI_MMBI_TOTAL_128KB,
	ESPI_MMBI_TOTAL_256KB,
	ESPI_MMBI_TOTAL_512KB,
	ESPI_MMBI_TOTAL_1024KB,
	ESPI_MMBI_TOTAL_2048KB,
	ESPI_MMBI_TOTAL_4096KB,
	ESPI_MMBI_TOTAL_8192KB,
}ESPI_MMBI_TOTAL_SIZE;


#define MMBI_PART_SIZE_ENUM			(ESPI_MMBI_INST_8KB)
#define MMBI_PART_SIZE_8KB_POW2     (13)
#define MMBI_PART_SZIE_8KB			(1<<MMBI_PART_SIZE_8KB_POW2)
#define MMBI_PART_SIZE_PER_APP		(MMBI_PART_SZIE_8KB<<MMBI_PART_SIZE_ENUM)
#define MMBI_TOTAL_SIZE_ENUM		(ESPI_MMBI_TOTAL_64KB)
#define MMBI_SIZE_64KB        		(0x80000)
#define MMBI_TOTAL_SIZE     		(MMBI_SIZE_64KB<<MMBI_TOTAL_SIZE_ENUM)
#define MMBI_NUM_INST				(MMBI_BLOCK_SIZE>>MMBI_PART_SIZE_8KB_POW2)

#define MMBI_CTRL_INST_SIZE_POS		(8)
#define MMBI_CTRL_TOTAL_SIZE_POS    (4)

struct aspeed_espi_mmbi_partition_op {
	u32 offset;
	u32 len;
	u8	*data_buf;
};

struct aspeed_espi_mmbi_reg {
	unsigned int offset;
	unsigned int value;
};

struct aspeed_espi_mmbi_xfer {
	u8 app_index;
	u8 rsvd[3];
	u32 len;		//read/write len
	u32 offset;		//mmbi block offset
	u8	*xfer_buf;
};

struct aspeed_espi_mmbi_ints {
	unsigned int app_index;
	unsigned int host_rwp[2];
	unsigned char int_sts;
	unsigned char rsvd[3];
};

//MMBI Desc TBD, can be changed later(don't care) total 36 bytes
struct MMBIDesc {
	u32 H2B_BA;   //Host-to-BMC Circular buffer base address
	u32 B2H_BA;   //BMC-to-Host Circular buffer base address
	u16  H2B_D;    //The size of H2B circular buffer in bytes (Cannot pow(2))
	u16  B2H_D;    //The size of B2H circular buffer in bytes
	//tU16 rsvd0[2]; //12 bytes
	u32 H2B_RP;   //BMC read location(offset in bytes from the beginning of the H2B buffer)
	u8  B_FLAG;   //B_RDY|B_RST|B_IE
	u8  H_FLAG;   //H_RDY|H_RST|H_SMI
	u8 rsvd[2]; //alignment
	u32 B2H_WP;   //B2H write pointer-where BMC can write the next data in the B2H CB
	u32 B2H_RP;   //B2H read pointer-where the next data will be available. Host advances
	u32 H2B_WP;   //Host shall only advance the pointer during regular time
};

struct mmbi_host_rwp {
	u32 b2h_host_rd;
	u32 h2b_host_wr;
};

typedef enum  {
   IOCTL_ESPI_MMBI_CTRL = 1,
   IOCTL_ESPI_MMBI_RX,
   IOCTL_ESPI_MMBIP_TX,
   IOCTL_ESPI_MMBI_WFI,
   IOCTL_ESPI_MMBI_PARTITION_READ,
   IOCTL_ESPI_MMBI_PARTITION_WRITE,
   IOCTL_ESPI_MMBI_REG,
}ESPI_MMBI_IOCTLS;

#define ESPI_MMBI_IOC_BASE       'M'

#define ASPEED_ESPI_MMBI_IOCTRL		_IOWR(ESPI_MMBI_IOC_BASE, IOCTL_ESPI_MMBI_CTRL, unsigned int reg_value)
#define ASPEED_ESPI_MMBI_IOCRX		_IOWR(ESPI_MMBI_IOC_BASE, IOCTL_ESPI_MMBI_RX, struct aspeed_espi_mmbi_xfer)
#define ASPEED_ESPI_MMBIP_IOCTX		_IOW(ESPI_MMBI_IOC_BASE, IOCTL_ESPI_MMBIP_TX, struct aspeed_espi_mmbi_xfer)	//post tx
//#define ASPEED_ESPI_MMBINP_IOCTX	_IOW(ESPI_MMBI_IOC_BASE, IOCTL_ESPI_MMBINP_TX, struct aspeed_espi_mmbi_xfer)	//non-post tx
#define ASPEED_ESPI_MMBI_IOWFI		_IOWR(ESPI_MMBI_IOC_BASE, IOCTL_ESPI_MMBI_WFI, struct aspeed_espi_mmbi_ints)
#define ASPEED_ESPI_MMBI_IOREAD		_IOR(ESPI_MMBI_IOC_BASE, IOCTL_ESPI_MMBI_PARTITION_READ, struct aspeed_espi_mmbi_partition_op)
#define ASPEED_ESPI_MMBI_IOWRITE	_IOW(ESPI_MMBI_IOC_BASE, IOCTL_ESPI_MMBI_PARTITION_WRITE, struct aspeed_espi_mmbi_partition_op)
#define ASPEED_ESPI_MMBI_REG		_IOR(ESPI_MMBI_IOC_BASE, IOCTL_ESPI_MMBI_REG, struct aspeed_espi_mmbi_reg)


#endif
