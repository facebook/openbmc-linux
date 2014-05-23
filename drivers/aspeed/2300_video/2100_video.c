/*
/  This is a video driver sample for AST2050/AST2100.
/  It is a very primitive driver for ASPEED's software partners only
*/

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include "ioaccess.h"
#include "i2c.h"

MODULE_LICENSE ("GPL");
DECLARE_WAIT_QUEUE_HEAD (my_queue);

int ioaccess_major = 0;
unsigned char  *Linear_address, *MMIO_linear_address, *Data_Linear_address, *Flag_Buffer_Linear_address;
int automode_trigger(void);
int pass3_trigger(void);
u8  aspeedi2c_read (u8, u8, u8);
u8  aspeedi2c_write (u8, u8, u8, u8);

int  ioaccess_ioctl (struct inode *inode, struct file *filp, unsigned cmd, unsigned long arg);
int  ioaccess_open (struct inode *inode, struct file *filp);
int  ioaccess_release (struct inode *inode, struct file *filp);
int  flag = 0;
int  Mode_Changed = 0, Capture_Ready = 0, Compression_Ready = 0, Mode_Changed_Flag = 0;

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

        case IOCTL_BIT_STREAM_BASE:
             copy_to_user ((u32 *)Kernal_IO_Data.Address, Data_Linear_address, Kernal_IO_Data.Data);
             *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
             ret = 0;
        break;

        case IOCTL_FLAG_BUFFER_CLEAR:
             Flag_Buffer_Linear_address = ioremap_nocache (Kernal_IO_Data.Address, 0x10000);
             memset ((u32 *)Flag_Buffer_Linear_address, 0, Kernal_IO_Data.Data);
             iounmap (Flag_Buffer_Linear_address);
             *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
             ret = 0;
        break;

        case IOCTL_AUTOMODE_TRIGGER:
            ret = automode_trigger();
	    if (Mode_Changed_Flag == 1) {
		Mode_Changed_Flag = 0;
	    }
        break;

        case IOCTL_PASS3_TRIGGER:
            ret = pass3_trigger();
	    if (Mode_Changed_Flag == 1) {
		Mode_Changed_Flag = 0;
	    }
        break;
        
        case IOCTL_I2C_READ:
            ret = aspeedi2c_read ((u8)Kernal_IO_Data.Address, (u8)Kernal_IO_Data.Data, (u8)Kernal_IO_Data.Value);
	break;

        case IOCTL_I2C_WRITE:
            ret = aspeedi2c_write ((u8)Kernal_IO_Data.Address, (u8)Kernal_IO_Data.Data, (u8)Kernal_IO_Data.Value, (u8)Kernal_IO_Data.I2CValue);
        break;

        default:
            ret = 3;
    }
    return ret;
}

u8  aspeedi2c_read (u8 channel, u8 device_address, u8 register_index)
{
    unsigned long    status;
    
    *(u32 *)(IO_ADDRESS(0x1e6e2000)) = 0x1688a8a8;
    status = *(u32 *)(IO_ADDRESS(0x1e6e2004));
//    *(u32 *)(IO_ADDRESS(0x1e6e2004)) = status | 0x04;
//    *(u32 *)(IO_ADDRESS(0x1e6e2004)) = status & ~(0x04);
    *(u32 *)(IO_ADDRESS(0x1e6e2004)) = 0;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_FUNCTION_CONTROL_REGISTER + 0x40 * channel)) = 0;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_AC_TIMING_REGISTER_1 + 0x40 * channel)) = AC_TIMING;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_AC_TIMING_REGISTER_2 + 0x40 * channel)) = 0;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) = ALL_CLEAR;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_FUNCTION_CONTROL_REGISTER + 0x40 * channel)) = MASTER_ENABLE;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_CONTROL_REGISTER + 0x40 * channel)) = 0xAF;
    
// Start and Send device address
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_BYTE_BUFFER_REGISTER + 0x40 * channel)) = device_address;
    barrier();
//    mdelay (10);
//    status = *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_BYTE_BUFFER_REGISTER + 0x40 * channel));
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_COMMAND_REGISTER + 0x40 * channel)) = MASTER_START_COMMAND | MASTER_TX_COMMAND;
    do {
        status = *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) & TX_ACK;
    } while (status != TX_ACK);
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) = ALL_CLEAR;
// Send register index
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_BYTE_BUFFER_REGISTER + 0x40 * channel)) = register_index;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_COMMAND_REGISTER + 0x40 * channel)) = MASTER_TX_COMMAND;
    do {
        status = *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) & TX_ACK;
    } while (status != TX_ACK);
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) = ALL_CLEAR;
// Start, Send device address + 1 (read mode)
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_BYTE_BUFFER_REGISTER + 0x40 * channel)) = device_address + 1;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_COMMAND_REGISTER + 0x40 * channel)) = MASTER_START_COMMAND | MASTER_TX_COMMAND | MASTER_RX_COMMAND | RX_COMMAND_LIST;
    do {
        status = *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) & RX_DONE;
    } while (status != RX_DONE);
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) = ALL_CLEAR;
// Stop    
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_CONTROL_REGISTER + 0x40 * channel)) = 0xBF;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_COMMAND_REGISTER + 0x40 * channel)) = MASTER_STOP_COMMAND;
    do {
        status = *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) & STOP_DONE;
    } while (status != STOP_DONE);
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_CONTROL_REGISTER + 0x40 * channel)) = 0xAF;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) = ALL_CLEAR;
    status = (*(u32 *)(IO_ADDRESS(I2C_BASE + I2C_BYTE_BUFFER_REGISTER + 0x40 * channel)) & 0xFF00) >> 8;
    
    return status;
}

u8  aspeedi2c_write (u8 channel, u8 device_address, u8 register_index, u8 register_value)
{
    unsigned long    status;
    
    *(u32 *)(IO_ADDRESS(0x1e6e2000)) = 0x1688a8a8;
    status = *(u32 *)(IO_ADDRESS(0x1e6e2004));
    *(u32 *)(IO_ADDRESS(0x1e6e2004)) = status | 0x04;
    *(u32 *)(IO_ADDRESS(0x1e6e2004)) = status & ~(0x04);
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_FUNCTION_CONTROL_REGISTER + 0x40 * channel)) = 0;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_AC_TIMING_REGISTER_1 + 0x40 * channel)) = AC_TIMING;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_AC_TIMING_REGISTER_2 + 0x40 * channel)) = 0;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) = ALL_CLEAR;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_FUNCTION_CONTROL_REGISTER + 0x40 * channel)) = MASTER_ENABLE;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_CONTROL_REGISTER + 0x40 * channel)) = 0xAF;
    
// Start and Send device address
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_BYTE_BUFFER_REGISTER + 0x40 * channel)) = device_address;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_COMMAND_REGISTER + 0x40 * channel)) = MASTER_START_COMMAND | MASTER_TX_COMMAND;
    do {
        status = *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) & TX_ACK;
    } while (status != TX_ACK);
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) = ALL_CLEAR;
// Send register index
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_BYTE_BUFFER_REGISTER + 0x40 * channel)) = register_index;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_COMMAND_REGISTER + 0x40 * channel)) = MASTER_TX_COMMAND;
    do {
        status = *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) & TX_ACK;
    } while (status != TX_ACK);
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) = ALL_CLEAR;
// Send register value
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_BYTE_BUFFER_REGISTER + 0x40 * channel)) = register_value;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_COMMAND_REGISTER + 0x40 * channel)) = MASTER_TX_COMMAND;
    do {
        status = *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) & TX_ACK;
    } while (status != TX_ACK);
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) = ALL_CLEAR;
// Stop    
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_CONTROL_REGISTER + 0x40 * channel)) = 0xBF;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_COMMAND_REGISTER + 0x40 * channel)) = MASTER_STOP_COMMAND;
    do {
        status = *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) & STOP_DONE;
    } while (status != STOP_DONE);
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_CONTROL_REGISTER + 0x40 * channel)) = 0xAF;
    *(u32 *)(IO_ADDRESS(I2C_BASE + I2C_INTERRUPT_STATUS_REGISTER + 0x40 * channel)) = ALL_CLEAR;
    status = (*(u32 *)(IO_ADDRESS(I2C_BASE + I2C_BYTE_BUFFER_REGISTER + 0x40 * channel)) & 0xFF00) >> 8;
    
    return status;
}

int automode_trigger()
{
    unsigned int status;

    status = *(u32 *)(IO_ADDRESS(0x1e700004));
    *(u32 *)(IO_ADDRESS(0x1e700004)) = (status & 0xFFFFFFE9);
    barrier();
    *(u32 *)(IO_ADDRESS(0x1e700004)) = ((status & 0xFFFFFFED) | 0x12);
    wait_event_interruptible (my_queue, (flag == 1));
    flag = 0;
    if (Mode_Changed == 1) {
	Mode_Changed = 0;
    }
    if ((Capture_Ready == 1) && (Compression_Ready == 1)) {
	Capture_Ready = 0;
	Compression_Ready = 0;
    }
    
    return Mode_Changed_Flag;
}

int pass3_trigger()
{
    unsigned int status;

    status = *(u32 *)(IO_ADDRESS(0x1e700004));
    status &= 0xFFFFFFC1;
    *(u32 *)(IO_ADDRESS(0x1e700004)) = status;
    barrier ();
    *(u32 *)(IO_ADDRESS(0x1e700004)) = (status | 0x10);
    wait_event_interruptible (my_queue, (flag == 1));
    flag = 0;
    if (Mode_Changed == 1) {
	Mode_Changed = 0;
    }
    if ((Capture_Ready == 1) && (Compression_Ready == 1)) {
	Capture_Ready = 0;
	Compression_Ready = 0;
    }

    return Mode_Changed_Flag;
}

static irqreturn_t video_interrupt (int irq, void *dev_id, struct pt_regs *regs){
    unsigned int mask, status;

    mask =  *(u32 *)(IO_ADDRESS(0x1e700304));
    status = *(u32 *)(IO_ADDRESS(0x1e700308));
	
    if (status & 0x01) {
        Mode_Changed = 1;
        Mode_Changed_Flag = 1;
       *(u32 *)(IO_ADDRESS(0x1e700308)) = 0xB;
    }
    else {
    }
    if (status & 0x08) {
        Compression_Ready = 1;
       *(u32 *)(IO_ADDRESS(0x1e700308)) = 0x8;
    }
    if (status & 0x02) {
        Capture_Ready = 1;
       *(u32 *)(IO_ADDRESS(0x1e700308)) = 0x2;
    }
    if ((Mode_Changed == 1) || ((Capture_Ready == 1) && (Compression_Ready == 1))) {
        flag = 1;
        wake_up_interruptible (&my_queue);
    }

    return IRQ_HANDLED;
}


int my_init(void)
{
    int    result, retval;
    unsigned long  ChipBounding;

//    SET_MODULE_OWNER (&ioaccess_fops);
    result = register_chrdev (ioaccess_major, "2100_video", &ioaccess_fops);
    if (result < 0) {
        printk ("<0>" "<0>Can't get major.\n");
        return result;
    }
    if (ioaccess_major == 0) {
        ioaccess_major = result;
        printk ("<0>" "A ioaccess_major = %d\n", ioaccess_major);
    }
//AST2100/AST2200 compress bitstream buffer is at 90MB
//AST2050 compress bitstream buffer is at 36MB
//All EVBs have 128MB now so compress buffer is at 90MB
    ChipBounding = ((*(u32 *)(IO_ADDRESS(0x1e6E207C)) >> 8) & 0x3);
    if ((ChipBounding == 3) || (ChipBounding == 1)) { //AST2100/AST2200
        Data_Linear_address = ioremap_nocache (0x5000000, 0x400000);
    }
    else { //AST2050
        Data_Linear_address = ioremap_nocache (0x5000000, 0x400000);
    }
    retval = request_irq (7, &video_interrupt, IRQF_DISABLED, "2100_video", NULL);
    if (retval) {
        printk ("Unable to get IRQ");
    }
//    IRQ_SET_HIGH_LEVEL (7);
//    IRQ_SET_LEVEL_TRIGGER (7);

    init_waitqueue_head (&my_queue);

    return 0;
}
                                                                                
void my_cleanup(void)
{
    unregister_chrdev (ioaccess_major, "test1");
    iounmap (Data_Linear_address);
    free_irq (7, NULL);
//    iounmap (MMIO_linear_address);
//    iounmap (Linear_address);
    return;
}


module_init (my_init);
module_exit (my_cleanup);
