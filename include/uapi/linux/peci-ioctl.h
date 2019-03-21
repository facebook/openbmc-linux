/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2018 Intel Corporation */

#ifndef __PECI_IOCTL_H
#define __PECI_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* Base Address of 48d */
#define PECI_BASE_ADDR  0x30  /* The PECI client's default address of 0x30 */
#define PECI_OFFSET_MAX 8     /* Max numver of CPU clients */

/* PCI Access */
#define MAX_PCI_READ_LEN 24   /* Number of bytes of the PCI Space read */

#define PCI_BUS0_CPU0      0x00
#define PCI_BUS0_CPU1      0x80
#define PCI_CPUBUSNO_BUS   0x00
#define PCI_CPUBUSNO_DEV   0x08
#define PCI_CPUBUSNO_FUNC  0x02
#define PCI_CPUBUSNO       0xcc
#define PCI_CPUBUSNO_1     0xd0
#define PCI_CPUBUSNO_VALID 0xd4

/* Package Identifier Read Parameter Value */
#define PKG_ID_CPU_ID               0x0000  /* CPUID Info */
#define PKG_ID_PLATFORM_ID          0x0001  /* Platform ID */
#define PKG_ID_UNCORE_ID            0x0002  /* Uncore Device ID */
#define PKG_ID_MAX_THREAD_ID        0x0003  /* Max Thread ID */
#define PKG_ID_MICROCODE_REV        0x0004  /* CPU Microcode Update Revision */
#define PKG_ID_MACHINE_CHECK_STATUS 0x0005  /* Machine Check Status */

/* RdPkgConfig Index */
#define MBX_INDEX_CPU_ID            0   /* Package Identifier Read */
#define MBX_INDEX_VR_DEBUG          1   /* VR Debug */
#define MBX_INDEX_PKG_TEMP_READ     2   /* Package Temperature Read */
#define MBX_INDEX_ENERGY_COUNTER    3   /* Energy counter */
#define MBX_INDEX_ENERGY_STATUS     4   /* DDR Energy Status */
#define MBX_INDEX_WAKE_MODE_BIT     5   /* "Wake on PECI" Mode bit */
#define MBX_INDEX_EPI               6   /* Efficient Performance Indication */
#define MBX_INDEX_PKG_RAPL_PERF     8   /* Pkg RAPL Performance Status Read */
#define MBX_INDEX_PER_CORE_DTS_TEMP 9   /* Per Core DTS Temperature Read */
#define MBX_INDEX_DTS_MARGIN        10  /* DTS thermal margin */
#define MBX_INDEX_SKT_PWR_THRTL_DUR 11  /* Socket Power Throttled Duration */
#define MBX_INDEX_CFG_TDP_CONTROL   12  /* TDP Config Control */
#define MBX_INDEX_CFG_TDP_LEVELS    13  /* TDP Config Levels */
#define MBX_INDEX_DDR_DIMM_TEMP     14  /* DDR DIMM Temperature */
#define MBX_INDEX_CFG_ICCMAX        15  /* Configurable ICCMAX */
#define MBX_INDEX_TEMP_TARGET       16  /* Temperature Target Read */
#define MBX_INDEX_CURR_CFG_LIMIT    17  /* Current Config Limit */
#define MBX_INDEX_DIMM_TEMP_READ    20  /* Package Thermal Status Read */
#define MBX_INDEX_DRAM_IMC_TMP_READ 22  /* DRAM IMC Temperature Read */
#define MBX_INDEX_DDR_CH_THERM_STAT 23  /* DDR Channel Thermal Status */
#define MBX_INDEX_PKG_POWER_LIMIT1  26  /* Package Power Limit1 */
#define MBX_INDEX_PKG_POWER_LIMIT2  27  /* Package Power Limit2 */
#define MBX_INDEX_TDP               28  /* Thermal design power minimum */
#define MBX_INDEX_TDP_HIGH          29  /* Thermal design power maximum */
#define MBX_INDEX_TDP_UNITS         30  /* Units for power/energy registers */
#define MBX_INDEX_RUN_TIME          31  /* Accumulated Run Time */
#define MBX_INDEX_CONSTRAINED_TIME  32  /* Thermally Constrained Time Read */
#define MBX_INDEX_TURBO_RATIO       33  /* Turbo Activation Ratio */
#define MBX_INDEX_DDR_RAPL_PL1      34  /* DDR RAPL PL1 */
#define MBX_INDEX_DDR_PWR_INFO_HIGH 35  /* DRAM Power Info Read (high) */
#define MBX_INDEX_DDR_PWR_INFO_LOW  36  /* DRAM Power Info Read (low) */
#define MBX_INDEX_DDR_RAPL_PL2      37  /* DDR RAPL PL2 */
#define MBX_INDEX_DDR_RAPL_STATUS   38  /* DDR RAPL Performance Status */
#define MBX_INDEX_DDR_HOT_ABSOLUTE  43  /* DDR Hottest Dimm Absolute Temp */
#define MBX_INDEX_DDR_HOT_RELATIVE  44  /* DDR Hottest Dimm Relative Temp */
#define MBX_INDEX_DDR_THROTTLE_TIME 45  /* DDR Throttle Time */
#define MBX_INDEX_DDR_THERM_STATUS  46  /* DDR Thermal Status */
#define MBX_INDEX_TIME_AVG_TEMP     47  /* Package time-averaged temperature */
#define MBX_INDEX_TURBO_RATIO_LIMIT 49  /* Turbo Ratio Limit Read */
#define MBX_INDEX_HWP_AUTO_OOB      53  /* HWP Autonomous Out-of-band */
#define MBX_INDEX_DDR_WARM_BUDGET   55  /* DDR Warm Power Budget */
#define MBX_INDEX_DDR_HOT_BUDGET    56  /* DDR Hot Power Budget */
#define MBX_INDEX_PKG_PSYS_PWR_LIM3 57  /* Package/Psys Power Limit3 */
#define MBX_INDEX_PKG_PSYS_PWR_LIM1 58  /* Package/Psys Power Limit1 */
#define MBX_INDEX_PKG_PSYS_PWR_LIM2 59  /* Package/Psys Power Limit2 */
#define MBX_INDEX_PKG_PSYS_PWR_LIM4 60  /* Package/Psys Power Limit4 */
#define MBX_INDEX_PERF_LIMIT_REASON 65  /* Performance Limit Reasons */

/* WrPkgConfig Index */
#define MBX_INDEX_DIMM_AMBIENT      19
#define MBX_INDEX_DIMM_TEMP         24

/* Device Specific Completion Code (CC) Definition */
#define DEV_PECI_CC_SUCCESS          0x40
#define DEV_PECI_CC_TIMEOUT          0x80
#define DEV_PECI_CC_OUT_OF_RESOURCE  0x81
#define DEV_PECI_CC_UNAVAIL_RESOURCE 0x82
#define DEV_PECI_CC_INVALID_REQ      0x90

/* Completion Code mask to check retry needs */
#define DEV_PECI_CC_RETRY_CHECK_MASK 0xf0
#define DEV_PECI_CC_NEED_RETRY       0x80

/* Skylake EDS says to retry for 250ms */
#define DEV_PECI_RETRY_TIME_MS       250
#define DEV_PECI_RETRY_INTERVAL_USEC 10000
#define DEV_PECI_RETRY_BIT           0x01

#define GET_TEMP_WR_LEN   1
#define GET_TEMP_RD_LEN   2
#define GET_TEMP_PECI_CMD 0x01

#define GET_DIB_WR_LEN   1
#define GET_DIB_RD_LEN   8
#define GET_DIB_PECI_CMD 0xf7

#define RDPKGCFG_WRITE_LEN     5
#define RDPKGCFG_READ_LEN_BASE 1
#define RDPKGCFG_PECI_CMD      0xa1

#define WRPKGCFG_WRITE_LEN_BASE 6
#define WRPKGCFG_READ_LEN       1
#define WRPKGCFG_PECI_CMD       0xa5

#define RDIAMSR_WRITE_LEN 5
#define RDIAMSR_READ_LEN  9
#define RDIAMSR_PECI_CMD  0xb1

#define WRIAMSR_PECI_CMD  0xb5

#define RDPCICFG_WRITE_LEN 6
#define RDPCICFG_READ_LEN  5
#define RDPCICFG_PECI_CMD  0x61

#define WRPCICFG_PECI_CMD  0x65

#define RDPCICFGLOCAL_WRITE_LEN     5
#define RDPCICFGLOCAL_READ_LEN_BASE 1
#define RDPCICFGLOCAL_PECI_CMD      0xe1

#define WRPCICFGLOCAL_WRITE_LEN_BASE 6
#define WRPCICFGLOCAL_READ_LEN       1
#define WRPCICFGLOCAL_PECI_CMD       0xe5

#define PECI_BUFFER_SIZE 32

/**
 * enum peci_cmd - PECI client commands
 * @PECI_CMD_XFER: raw PECI transfer
 * @PECI_CMD_PING: ping, a required message for all PECI devices
 * @PECI_CMD_GET_DIB: get DIB (Device Info Byte)
 * @PECI_CMD_GET_TEMP: get maximum die temperature
 * @PECI_CMD_RD_PKG_CFG: read access to the PCS (Package Configuration Space)
 * @PECI_CMD_WR_PKG_CFG: write access to the PCS (Package Configuration Space)
 * @PECI_CMD_RD_IA_MSR: read access to MSRs (Model Specific Registers)
 * @PECI_CMD_WR_IA_MSR: write access to MSRs (Model Specific Registers)
 * @PECI_CMD_RD_PCI_CFG: sideband read access to the PCI configuration space
 *	maintained in downstream devices external to the processor
 * @PECI_CMD_WR_PCI_CFG: sideband write access to the PCI configuration space
 *	maintained in downstream devices external to the processor
 * @PECI_CMD_RD_PCI_CFG_LOCAL: sideband read access to the PCI configuration
 *	space that resides within the processor
 * @PECI_CMD_WR_PCI_CFG_LOCAL: sideband write access to the PCI configuration
 *	space that resides within the processor
 *
 * Available commands depend on client's PECI revision.
 */
enum peci_cmd {
	PECI_CMD_XFER = 0,
	PECI_CMD_PING,
	PECI_CMD_GET_DIB,
	PECI_CMD_GET_TEMP,
	PECI_CMD_RD_PKG_CFG,
	PECI_CMD_WR_PKG_CFG,
	PECI_CMD_RD_IA_MSR,
	PECI_CMD_WR_IA_MSR,
	PECI_CMD_RD_PCI_CFG,
	PECI_CMD_WR_PCI_CFG,
	PECI_CMD_RD_PCI_CFG_LOCAL,
	PECI_CMD_WR_PCI_CFG_LOCAL,
	PECI_CMD_MAX
};

/**
 * struct peci_xfer_msg - raw PECI transfer command
 * @addr; address of the client
 * @tx_len: number of data to be written in bytes
 * @rx_len: number of data to be read in bytes
 * @tx_buf: data to be written, or NULL
 * @rx_buf: data to be read, or NULL
 *
 * raw PECI transfer
 */
struct peci_xfer_msg {
	__u8 addr;
	__u8 tx_len;
	__u8 rx_len;
	__u8 tx_buf[PECI_BUFFER_SIZE];
	__u8 rx_buf[PECI_BUFFER_SIZE];
} __attribute__((__packed__));

/**
 * struct peci_ping_msg - ping command
 * @addr: address of the client
 *
 * Ping() is a required message for all PECI devices. This message is used to
 * enumerate devices or determine if a device has been removed, been
 * powered-off, etc.
 */
struct peci_ping_msg {
	__u8 addr;
} __attribute__((__packed__));

/**
 * struct peci_get_dib_msg - GetDIB command
 * @addr: address of the client
 * @dib: DIB data to be read
 *
 * The processor PECI client implementation of GetDIB() includes an 8-byte
 * response and provides information regarding client revision number and the
 * number of supported domains. All processor PECI clients support the GetDIB()
 * command.
 */
struct peci_get_dib_msg {
	__u8  addr;
	__u64 dib;
} __attribute__((__packed__));

/**
 * struct peci_get_temp_msg - GetTemp command
 * @addr: address of the client
 * @temp_raw: raw temperature data to be read
 *
 * The GetTemp() command is used to retrieve the maximum die temperature from a
 * target PECI address. The temperature is used by the external thermal
 * management system to regulate the temperature on the die. The data is
 * returned as a negative value representing the number of degrees centigrade
 * below the maximum processor junction temperature.
 */
struct peci_get_temp_msg {
	__u8  addr;
	__s16 temp_raw;
} __attribute__((__packed__));

/**
 * struct peci_rd_pkg_cfg_msg - RdPkgConfig command
 * @addr: address of the client
 * @index: encoding index for the requested service
 * @param: specific data being requested
 * @rx_len: number of data to be read in bytes
 * @pkg_config: package config data to be read
 *
 * The RdPkgConfig() command provides read access to the Package Configuration
 * Space (PCS) within the processor, including various power and thermal
 * management functions. Typical PCS read services supported by the processor
 * may include access to temperature data, energy status, run time information,
 * DIMM temperatures and so on.
 */
struct peci_rd_pkg_cfg_msg {
	__u8  addr;
	__u8  index;
	__u16 param;
	__u8  rx_len;
	__u8  pkg_config[4];
} __attribute__((__packed__));

/**
 * struct peci_wr_pkg_cfg_msg - WrPkgConfig command
 * @addr: address of the client
 * @index: encoding index for the requested service
 * @param: specific data being requested
 * @tx_len: number of data to be written in bytes
 * @value: package config data to be written
 *
 * The WrPkgConfig() command provides write access to the Package Configuration
 * Space (PCS) within the processor, including various power and thermal
 * management functions. Typical PCS write services supported by the processor
 * may include power limiting, thermal averaging constant programming and so on.
 */
struct peci_wr_pkg_cfg_msg {
	__u8  addr;
	__u8  index;
	__u16 param;
	__u8  tx_len;
	__u32 value;
} __attribute__((__packed__));

/**
 * struct peci_rd_ia_msr_msg - RdIAMSR command
 * @addr: address of the client
 * @thread_id: ID of the specific logical processor
 * @address: address of MSR to read from
 * @value: data to be read
 *
 * The RdIAMSR() PECI command provides read access to Model Specific Registers
 * (MSRs) defined in the processor's Intel Architecture (IA).
 */
struct peci_rd_ia_msr_msg {
	__u8  addr;
	__u8  thread_id;
	__u16 address;
	__u64 value;
} __attribute__((__packed__));

/**
 * struct peci_rd_pci_cfg_msg - RdPCIConfig command
 * @addr: address of the client
 * @bus: PCI bus number
 * @device: PCI device number
 * @function: specific function to read from
 * @reg: specific register to read from
 * @pci_config: config data to be read
 *
 * The RdPCIConfig() command provides sideband read access to the PCI
 * configuration space maintained in downstream devices external to the
 * processor.
 */
struct peci_rd_pci_cfg_msg {
	__u8  addr;
	__u8  bus;
	__u8  device;
	__u8  function;
	__u16 reg;
	__u8  pci_config[4];
} __attribute__((__packed__));

/**
 * struct peci_rd_pci_cfg_local_msg - RdPCIConfigLocal command
 * @addr: address of the client
 * @bus: PCI bus number
 * @device: PCI device number
 * @function: specific function to read from
 * @reg: specific register to read from
 * @rx_len: number of data to be read in bytes
 * @pci_config: config data to be read
 *
 * The RdPCIConfigLocal() command provides sideband read access to the PCI
 * configuration space that resides within the processor. This includes all
 * processor IIO and uncore registers within the PCI configuration space.
 */
struct peci_rd_pci_cfg_local_msg {
	__u8  addr;
	__u8  bus;
	__u8  device;
	__u8  function;
	__u16 reg;
	__u8  rx_len;
	__u8  pci_config[4];
} __attribute__((__packed__));

/**
 * struct peci_wr_pci_cfg_local_msg - WrPCIConfigLocal command
 * @addr: address of the client
 * @bus: PCI bus number
 * @device: PCI device number
 * @function: specific function to read from
 * @reg: specific register to read from
 * @tx_len: number of data to be written in bytes
 * @value: config data to be written
 *
 * The WrPCIConfigLocal() command provides sideband write access to the PCI
 * configuration space that resides within the processor. PECI originators can
 * access this space even before BIOS enumeration of the system buses.
 */
struct peci_wr_pci_cfg_local_msg {
	__u8  addr;
	__u8  bus;
	__u8  device;
	__u8  function;
	__u16 reg;
	__u8  tx_len;
	__u32 value;
} __attribute__((__packed__));

#define PECI_IOC_BASE  0xb7

#define PECI_IOC_XFER \
	_IOWR(PECI_IOC_BASE, PECI_CMD_XFER, struct peci_xfer_msg)

#define PECI_IOC_PING \
	_IOWR(PECI_IOC_BASE, PECI_CMD_PING, struct peci_ping_msg)

#define PECI_IOC_GET_DIB \
	_IOWR(PECI_IOC_BASE, PECI_CMD_GET_DIB, struct peci_get_dib_msg)

#define PECI_IOC_GET_TEMP \
	_IOWR(PECI_IOC_BASE, PECI_CMD_GET_TEMP, struct peci_get_temp_msg)

#define PECI_IOC_RD_PKG_CFG \
	_IOWR(PECI_IOC_BASE, PECI_CMD_RD_PKG_CFG, struct peci_rd_pkg_cfg_msg)

#define PECI_IOC_WR_PKG_CFG \
	_IOWR(PECI_IOC_BASE, PECI_CMD_WR_PKG_CFG, struct peci_wr_pkg_cfg_msg)

#define PECI_IOC_RD_IA_MSR \
	_IOWR(PECI_IOC_BASE, PECI_CMD_RD_IA_MSR, struct peci_rd_ia_msr_msg)

#define PECI_IOC_RD_PCI_CFG \
	_IOWR(PECI_IOC_BASE, PECI_CMD_RD_PCI_CFG, struct peci_rd_pci_cfg_msg)

#define PECI_IOC_RD_PCI_CFG_LOCAL \
	_IOWR(PECI_IOC_BASE, PECI_CMD_RD_PCI_CFG_LOCAL, \
	      struct peci_rd_pci_cfg_local_msg)

#define PECI_IOC_WR_PCI_CFG_LOCAL \
	_IOWR(PECI_IOC_BASE, PECI_CMD_WR_PCI_CFG_LOCAL, \
	      struct peci_wr_pci_cfg_local_msg)

#endif /* __PECI_IOCTL_H */
