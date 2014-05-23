#define		JTAG_DATA		0x00
#define		JTAG_INSTRUCTION	0x04
#define		JTAG_CONTROL		0x08
#define		JTAG_INTERRUPT		0x0c
#define		JTAG_STATUS		0x10
#define		JTAG_TCK_CONTROL	0x14

//lattice
#define		LATTICE_CPLD_IDCODE	0x01809043
#define		LATTICE_INS_LENGTH	0x08
#define		LATTICE_COLUMN_LENGTH	352

//lattice's cmd
#define		BYPASS			0xff
#define		PLD_ADDRESS_SHIFT	0x01
#define		BULK_ERASE		0x03
#define		DISCHARGE		0x14
#define		PROGRAM_ENABLE		0x15
#define		IDCODE			0x16
#define		UES_READ		0x17
#define		UES_PROGRAM		0x1a
#define		SAMPLE			0x1c
#define		PROGRAM_DISABLE		0x1e
#define		PLD_INIT_ADDR_FOR_PROG	0x21
#define		PLD_PROG_INCR		0x27
#define		PLD_VERIFY_INCR		0x2a
#define		PROGRAM_DONE		0x2f


typedef struct _JTAG_DEVICE_INFO {
	unsigned char	Device_Name[50];
	unsigned int	Device_ID;
	unsigned long	Device_Column_Length;
} JTAG_DEVICE_INFO;