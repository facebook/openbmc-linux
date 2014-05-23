/*
This is a JTAG driver sample for ASPEED AST2300 series produces
It is a primitive driver for ASPEED's software partners only
  
Notice:
1. Before you read this sample code, you have to know the IEEE1149.1 page 19 -->
TAP controller state diagram

Sample waveform: Get IDCODE (cmd: 0x16 --> 00010110b)
To      1      2      3      4      4      4      4      4      4      4      4      5      6      7
(State)


TCK __|---|__|---|__|---|__|---|__|---|__|---|__|---|__|---|__|---|__|---|__|---|__|---|__|---|__|---|__

TMS _|---------|_________________________________________________________________|----------|___________

TDI ____________________________________|---------|__________|---|______________________________________

1: Select-DR-Scan
2: Select-IR-Scan
3: Capture-IR
4: Shift-IR
5: Exit-IR
6: Update-IR
7: Run-Test/Idle


2. The ASPEED JTAG controller's read/write length is equal or less than 32bits
in a fire cycle
3. The sample code only supports LATTICE's CPLD in this version
4. The sample code does not implement interrupt behavior in this version (SDK 0.14 supports interrupt.
You need to use AST2300 A1 chip to verify)
5. Support maximum 1MB file in this version
6. Support single device only in this version
*/

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include "ioaccess.h"
#include "jtag.h"

MODULE_LICENSE ("GPL");
DECLARE_WAIT_QUEUE_HEAD (jtag_queue);

int ioaccess_major = 0;

void JTAG_reset (void);
void JTAG_check_instruction_complete (void);
void JTAG_check_data_pause_complete (void);
void JTAG_check_data_complete (void);
u32 JTAG_get_idcode (void);
u32 JTAG_erase_lattice_CPLD (void);
u32 JTAG_erase_program_verify_lattice_CPLD (unsigned long*, unsigned long);
u32 JTAG_verify_lattice_CPLD (unsigned long*, unsigned long);

int  ioaccess_ioctl (struct inode *inode, struct file *filp, unsigned cmd, unsigned long arg);
int  flag = 0; //1 = instruction, 2 = data pause, 3 = data complete

volatile u8 *jtag_v_add;
unsigned long *JTAG_read_buffer;
unsigned long *JTAG_write_buffer;
unsigned long *JTAG_other_buffer;
JTAG_DEVICE_INFO JTAG_device_information;


struct file_operations ioaccess_fops = {
    ioctl:    ioaccess_ioctl,
};
                                                                   
int ioaccess_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg)
{

    int ret = 1;

    IO_ACCESS_DATA  Kernal_IO_Data;
 
    memset (&Kernal_IO_Data, 0, sizeof(IO_ACCESS_DATA));

    Kernal_IO_Data = *(IO_ACCESS_DATA *)arg;

    switch (cmd) {
        case IOCTL_IO_READ:
            Kernal_IO_Data.Data  = *(u32 *)(IO_ADDRESS(Kernal_IO_Data.Address));
            *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
            ret = 0;
        break;

        case IOCTL_IO_WRITE:
            *(u32 *)(IO_ADDRESS(Kernal_IO_Data.Address)) = Kernal_IO_Data.Data;
            ret = 0;
        break;

        case IOCTL_JTAG_RESET:
	    JTAG_reset ();
            *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
            ret = 0;
        break;

        case IOCTL_JTAG_IDCODE_READ:
  	    Kernal_IO_Data.Data = JTAG_get_idcode ();
            *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
            ret = 0;
        break;

        case IOCTL_JTAG_ERASE_CPLD:
	    Kernal_IO_Data.Data = JTAG_erase_lattice_CPLD ();
            *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
            ret = 0;
        break;

        case IOCTL_JTAG_PROGRAM_CPLD:
            ret = copy_from_user ((u32 *)JTAG_write_buffer, (u8 *)Kernal_IO_Data.Input_Buffer_Base, Kernal_IO_Data.Data);
	    Kernal_IO_Data.Data = JTAG_erase_program_verify_lattice_CPLD (JTAG_write_buffer, Kernal_IO_Data.Data * 8);
            *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
            ret = 0;
        break;

        case IOCTL_JTAG_VERIFY_CPLD:
            ret = copy_from_user ((u32 *)JTAG_write_buffer, (u8 *)Kernal_IO_Data.Input_Buffer_Base, Kernal_IO_Data.Data);
	    Kernal_IO_Data.Data = JTAG_verify_lattice_CPLD (JTAG_write_buffer, Kernal_IO_Data.Data * 8);
            *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
            ret = 0;
        break;

        default:
            ret = 3;
    }
    return ret;
}

/*
//JTAG_write_data(): transfer data in Shift-DR state
//JTAG_read_data(): receive data in Shift-DR state
Notice:
1: The ASPEED JTAG controller's read/write length is equal or less than
32bits in a fire cycle
2: If the D[1] in JTAG_CONTROL register does not set then hardware will
stay in Pause-DR state after transfer data
3: The lattice CPLD's column length is 352bits therefore you need to enter
in update-DR state after transfer 352bits
4: Even though you get the DR transfer pause idle or DR transfer complete
idle flags, you need to wait 3ms(experiment values) because those flags
only represent the JTAG bus status, not the CPLD's EEPROM status
*/
void JTAG_write_data (unsigned long *JTAG_data_buffer, unsigned long data_length_bits, unsigned long column_length_bits)
{
	unsigned long index = 0, count = 0;

	while (data_length_bits > 0) {
		while (data_length_bits > 32) {
			iowrite32(JTAG_data_buffer[index], jtag_v_add + JTAG_DATA);
			barrier();
			count ++;
			if (count == (column_length_bits / 32)) { // write bytes equals to column length => Update-DR
				iowrite32(0xC0000202, jtag_v_add + JTAG_CONTROL);
				barrier();
				iowrite32(0xC0000203, jtag_v_add + JTAG_CONTROL);
				count = 0;
				JTAG_check_data_complete ();
			}
			else { // write bytes were not equals to column length ==> Pause-DR
				iowrite32(0xC0000200, jtag_v_add + JTAG_CONTROL);
				barrier();
				iowrite32(0xC0000201, jtag_v_add + JTAG_CONTROL);
				JTAG_check_data_pause_complete ();
			}
			data_length_bits -= 32;
			mdelay(3);
			index ++;
		}
		if (data_length_bits != 0) { // last 32bits --> Update-DR
			iowrite32(JTAG_data_buffer[index], jtag_v_add + JTAG_DATA);
			barrier();
			iowrite32(0xC0000002 | (data_length_bits << 4), jtag_v_add + JTAG_CONTROL);
			barrier();
			iowrite32(0xC0000003 | (data_length_bits << 4), jtag_v_add + JTAG_CONTROL);
			data_length_bits -= data_length_bits;
			JTAG_check_data_complete ();
			mdelay(3);
		}
	}
}

void JTAG_read_data (unsigned long *JTAG_data_buffer, unsigned long data_length_bits)
{
	unsigned long index = 0, count = 0;

	while (data_length_bits > 0) {
		while (data_length_bits > 32) {
			count ++;
			iowrite32(0, jtag_v_add + JTAG_DATA);
			barrier();
			if (count == (JTAG_device_information.Device_Column_Length / 32)) { // read bytes equals to column length => Update-DR
				iowrite32(0xC0000202, jtag_v_add + JTAG_CONTROL);
				barrier();
				iowrite32(0xC0000203, jtag_v_add + JTAG_CONTROL);
				count = 0;
				JTAG_check_data_complete ();
			}
			else { // read bytes were not equals to column length ==> Pause-DR
				iowrite32(0xC0000200, jtag_v_add + JTAG_CONTROL);
				barrier();
				iowrite32(0xC0000201, jtag_v_add + JTAG_CONTROL);
				JTAG_check_data_pause_complete ();
			}
			data_length_bits -= 32;
			JTAG_data_buffer[index] = ioread32(jtag_v_add + JTAG_DATA);
			index ++;
		}
		if (data_length_bits != 0) { // last 32bits --> Update-DR
			iowrite32(0, jtag_v_add + JTAG_DATA);
			barrier();
			iowrite32(0xC0000002 | (data_length_bits << 4), jtag_v_add + JTAG_CONTROL);
			barrier();
			iowrite32(0xC0000003 | (data_length_bits << 4), jtag_v_add + JTAG_CONTROL);
			data_length_bits -= data_length_bits;
			JTAG_check_data_complete ();
			JTAG_data_buffer[index] = ioread32(jtag_v_add + JTAG_DATA);
		}
	}
}

void JTAG_write_instruction (unsigned char instruction, unsigned char instruction_bits)
{
	iowrite32(instruction, jtag_v_add + JTAG_INSTRUCTION);
	barrier();
	iowrite32(0xC0020000 | (instruction_bits << 20), jtag_v_add + JTAG_CONTROL);
	barrier();
	iowrite32(0xC0030000 | (instruction_bits << 20), jtag_v_add + JTAG_CONTROL);
	JTAG_check_instruction_complete ();
	if (instruction == 0x1a) {
		mdelay(1);
	}
	else {
		mdelay(20);
	}
}



/*
JTAG_reset() is to generate at least 9 TMS high and 1 TMS low to force devices into Run-Test/Idle State
*/
void JTAG_reset ()
{
	iowrite32(0xC0000000, jtag_v_add + JTAG_CONTROL);
	barrier();
	mdelay (5);	
	iowrite32(0xE0000000, jtag_v_add + JTAG_CONTROL);	// x TMS high + 1 TMS low
	mdelay (5);
}



/*
//JTAG_check_instruction_complete()
//JTAG_check_data_pause_complete()
//JTAG_check_data_complete()
check status subroutine
*/
void JTAG_check_instruction_complete ()
{
	wait_event_interruptible (jtag_queue, (flag == 1));
	flag = 0;
}

void JTAG_check_data_pause_complete ()
{
	wait_event_interruptible (jtag_queue, (flag == 2));
	flag = 0;
}

void JTAG_check_data_complete ()
{
	wait_event_interruptible (jtag_queue, (flag == 3));
	flag = 0;
}



/*
//JTAG_get_idcode(): returns the JTAG device's IDCODE
The first 32bits in receive buffer is IDCODE after you send IDCODE command to device
*/
u32 JTAG_get_idcode ()
{
	JTAG_write_instruction (IDCODE, LATTICE_INS_LENGTH);
	JTAG_read_data (JTAG_read_buffer, 32);

	return JTAG_read_buffer[0];
}



/*
//JTAG_erase_lattice_CPLD(): Erase JTAG device
The 100ms delay is the experiment value to make sure that the CPLD's EEPROM is erased
*/
u32 JTAG_erase_lattice_CPLD ()
{
	unsigned long idcode;

	idcode = JTAG_get_idcode();
	switch (idcode) {
		case LATTICE_CPLD_IDCODE:
			JTAG_device_information.Device_ID = LATTICE_CPLD_IDCODE;
			strcpy (JTAG_device_information.Device_Name, "LATTICE LC 4064V-3210 CPLD");
			JTAG_device_information.Device_Column_Length = LATTICE_COLUMN_LENGTH;
			printk ("Device IDCODE = %08x ---> %s\n", JTAG_device_information.Device_ID, JTAG_device_information.Device_Name);
		break;
		
		default:
			printk ("Device IDCODE = %08x\n", JTAG_device_information.Device_ID);
			printk ("The software can support LATTICE LC 4064V-3210 CPLD only in this version\n");
			return 1;
		break;
	}

	JTAG_write_instruction (SAMPLE, LATTICE_INS_LENGTH);
	JTAG_read_data (JTAG_read_buffer, 68);

	printk ("Starting to Erase Device . . .\n");
	JTAG_write_instruction (PROGRAM_ENABLE, LATTICE_INS_LENGTH);
	JTAG_write_instruction (BULK_ERASE, LATTICE_INS_LENGTH);
//The 100ms is experiment result
	mdelay(100);
	JTAG_write_instruction (DISCHARGE, LATTICE_INS_LENGTH);
	JTAG_write_instruction (PROGRAM_DISABLE, LATTICE_INS_LENGTH);
	printk ("Erase Done\n");
	
	return 0;
}

u32 JTAG_erase_program_verify_lattice_CPLD (unsigned long *JTAG_data_buffer, unsigned long data_length_bits)
{
	unsigned long i, row_number;

	if (JTAG_erase_lattice_CPLD ()) {
		printk ("ERASE CHIP FAILED!! Please Check if the JTAG Device is in the support list and the JTAG connector\n");
	}


	printk ("Starting to Program Device . . . This will take a few seconds\n");
	JTAG_write_instruction (PROGRAM_ENABLE, LATTICE_INS_LENGTH);
	JTAG_write_instruction (PLD_INIT_ADDR_FOR_PROG, LATTICE_INS_LENGTH);
//Program JTAG device by using the auto-incremen command
	JTAG_write_instruction (PLD_PROG_INCR, LATTICE_INS_LENGTH);
	JTAG_write_data (JTAG_write_buffer, data_length_bits, JTAG_device_information.Device_Column_Length);
	printk ("Program Done\n");


	printk ("Starting to Verify Device . . . This will take a few seconds\n");
//Move the address to verify data base
	row_number = data_length_bits / JTAG_device_information.Device_Column_Length;
	JTAG_write_instruction (PLD_ADDRESS_SHIFT, LATTICE_INS_LENGTH);
	for (i = 0; i < row_number / 32; i++) {
		JTAG_other_buffer[i] = 0;
	}
	JTAG_other_buffer[i] = (1 << ((row_number % 32) - 1));
	JTAG_write_data (JTAG_other_buffer, row_number, JTAG_device_information.Device_Column_Length);

//Verify JTAG device by using the auto-incremen command
	JTAG_write_instruction (PLD_VERIFY_INCR, LATTICE_INS_LENGTH);
//Read Data
	JTAG_read_data (JTAG_read_buffer, data_length_bits);
//Finish program
	JTAG_write_instruction (PROGRAM_DONE, LATTICE_INS_LENGTH);
	JTAG_write_instruction (PROGRAM_DISABLE, LATTICE_INS_LENGTH);


//Compare Data
	for (i = 0; i < data_length_bits / 32; i++) {
		if (JTAG_read_buffer[i] != JTAG_write_buffer[i]) {
			printk ("VERIFY ERROR!! ERROR at Index %08x\n", (unsigned int)i);
			printk ("The Original Value is %08x, Read Back Value is %08x\n", (unsigned int)JTAG_write_buffer[i], (unsigned int)JTAG_read_buffer[i]);
			return 1;
		}
	}
	printk ("Verify Done\n");
	
	return 0;
}

/*
//JTAG_verify_lattice_CPLD(): Verify JTAG device
Check the data read back from JTAG device with the input JEDEC file. If there's error then stop
*/
u32 JTAG_verify_lattice_CPLD (unsigned long *JTAG_data_buffer, unsigned long data_length_bits)
{
	unsigned long idcode, i, row_number;;

	idcode = JTAG_get_idcode();
	switch (idcode) {
		case LATTICE_CPLD_IDCODE:
			JTAG_device_information.Device_ID = LATTICE_CPLD_IDCODE;
			strcpy (JTAG_device_information.Device_Name, "LATTICE LC 4064V-3210 CPLD");
			JTAG_device_information.Device_Column_Length = 352;
			printk ("Device IDCODE = %08x ---> %s\n", JTAG_device_information.Device_ID, JTAG_device_information.Device_Name);
		break;
		
		default:
			printk ("Device IDCODE = %08x\n", JTAG_device_information.Device_ID);
			printk ("The software can support LATTICE LC 4064V-3210 CPLD only in this version\n");
			return 1;
		break;
	}

	JTAG_write_instruction (SAMPLE, LATTICE_INS_LENGTH);
	JTAG_read_data (JTAG_read_buffer, 68);


	printk ("Starting to Verify Device . . .\n");
	JTAG_write_instruction (PROGRAM_ENABLE, LATTICE_INS_LENGTH);
//Move the address to verify data base
	row_number = data_length_bits / JTAG_device_information.Device_Column_Length;
	JTAG_write_instruction (PLD_ADDRESS_SHIFT, LATTICE_INS_LENGTH);
	for (i = 0; i < row_number / 32; i++) {
		JTAG_other_buffer[i] = 0;
	}
	JTAG_other_buffer[i] = (1 << ((row_number % 32) - 1));
	JTAG_write_data (JTAG_other_buffer, row_number, JTAG_device_information.Device_Column_Length);

//Verify JTAG device by using the auto-incremen command
	JTAG_write_instruction (PLD_VERIFY_INCR, LATTICE_INS_LENGTH);
//Read Data
	JTAG_read_data (JTAG_read_buffer, data_length_bits);


//Compare Data
	for (i = 0; i < data_length_bits / 32; i++) {
		if (JTAG_read_buffer[i] != JTAG_write_buffer[i]) {
			printk ("VERIFY ERROR!! ERROR at Index %08x\n", (unsigned int)i);
			printk ("The Original Value is %08x, Read Back Value is %08x\n", (unsigned int)JTAG_write_buffer[i], (unsigned int)JTAG_read_buffer[i]);
			return 1;
		}
	}
	printk ("Verify Done\n");

	return 0;
}

static irqreturn_t jtag_interrupt (int irq, void *dev_id, struct pt_regs *regs)
{
    unsigned int status;

    status = ioread32(jtag_v_add + JTAG_INTERRUPT);

    if (status & 0x40000) {
    	flag = 1;
        iowrite32((0x40000 | (status & 0xf)), jtag_v_add + JTAG_INTERRUPT);
    }

    if (status & 0x20000) {
        flag = 2;
        iowrite32((0x20000 | (status & 0xf)), jtag_v_add + JTAG_INTERRUPT);
    }

    if (status & 0x10000) {
        flag = 3;
        iowrite32((0x10000 | (status & 0xf)), jtag_v_add + JTAG_INTERRUPT);
    }

    if ((flag == 1) || (flag == 2) || (flag == 3)) {
        wake_up_interruptible (&jtag_queue);
        return IRQ_HANDLED;
    }
    else {
    	printk ("Not JTAG's interrupt\n");
    	return IRQ_NONE;
    }
}

int my_init(void)
{
    int    result, status, retval;

    JTAG_read_buffer = kmalloc (0x100000, GFP_DMA|GFP_KERNEL);
    if (JTAG_read_buffer == NULL) {
    	printk ("Can't allocate read_buffer\n");
    	return 1;
    }
    JTAG_write_buffer = kmalloc (0x100000, GFP_DMA|GFP_KERNEL);
    if (JTAG_write_buffer == NULL) {
    	printk ("Can't allocate write_buffer\n");
    	kfree(JTAG_read_buffer);
    	return 1;
    }
    JTAG_other_buffer = kmalloc (0x100000, GFP_DMA|GFP_KERNEL);
    if (JTAG_other_buffer == NULL) {
    	printk ("Can't allocate other_buffer\n");
    	kfree(JTAG_read_buffer);
	kfree(JTAG_write_buffer);
    	return 1;
    }

    jtag_v_add = ioremap_nocache(0x1E6E4000, 0x40);
    printk ("jtag_v_add = %x\n", (unsigned int)jtag_v_add);
    if (jtag_v_add == NULL) {
    	printk ("Can't remap JTAG IO Address\n");
    	kfree(JTAG_read_buffer);
    	kfree(JTAG_write_buffer);
    	kfree(JTAG_other_buffer);
    	return 1;
    }
    memset (&JTAG_device_information, 0, sizeof(JTAG_DEVICE_INFO));

    result = register_chrdev (ioaccess_major, "jtag", &ioaccess_fops);
    if (result < 0) {
        printk ("<0>" "<0>Can't get major.\n");
        return result;
    }
    if (ioaccess_major == 0) {
        ioaccess_major = result;
        printk ("<0>" "A ioaccess_major = %d\n", ioaccess_major);
    }

    status = *(u32 *)(IO_ADDRESS(0x1E6E2004));
    *(u32 *)(IO_ADDRESS(0x1E6E2004)) = status &= ~(0x00400000); //Set JTAG Master Enable in SCU Reset Register
    iowrite32(0xC0000000, jtag_v_add + JTAG_CONTROL); //Eanble Clock
    barrier();

    retval = request_irq (43, &jtag_interrupt, IRQF_DISABLED, "jtag", NULL);
    if (retval) {
        printk ("Unable to get IRQ");
    }
//    IRQ_SET_HIGH_LEVEL (43);
//    IRQ_SET_LEVEL_TRIGGER (43);
    iowrite32(0x70007, jtag_v_add + JTAG_INTERRUPT); //Eanble Interrupt

    init_waitqueue_head (&jtag_queue);

    return 0;
}
                                                                                
void my_cleanup(void)
{
    kfree(JTAG_read_buffer);
    kfree(JTAG_write_buffer);
    kfree(JTAG_other_buffer);
    iounmap (jtag_v_add);
    free_irq (43, NULL);

    unregister_chrdev (ioaccess_major, "jtag");

    return;
}


module_init (my_init);
module_exit (my_cleanup);
