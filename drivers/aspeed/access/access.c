/*
/  This is a IO ACCESS driver sample for AST1100/AST2050/AST2100.
/  It is a primitive driver for ASPEED's software partners only
*/

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include "ioaccess.h"

MODULE_LICENSE ("GPL");

int ioaccess_major = 0;
int  ioaccess_ioctl (struct inode *inode, struct file *filp, unsigned cmd, unsigned long arg);

struct file_operations ioaccess_fops = {
    ioctl:    ioaccess_ioctl,
};
                                                                   
int ioaccess_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg)
{

    int ret = 0, i;
    unsigned long Memory_Data0, Memory_Data1, Memory_Data2, Memory_Data3, temp;
    unsigned char  *Over_Data_Linear_address;

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

        case IOCTL_MEMORY_OVER_READ:
            Over_Data_Linear_address = ioremap_nocache (Kernal_IO_Data.Address, Kernal_IO_Data.Data * 17);
	    for (i = 0; i < (Kernal_IO_Data.Data / 16) + 1; i++) {
		Memory_Data0 = *(u32 *)(Over_Data_Linear_address + i * 4 * 4);
		Memory_Data1 = *(u32 *)(Over_Data_Linear_address + (i * 4 + 1) * 4);
		Memory_Data2 = *(u32 *)(Over_Data_Linear_address + (i * 4 + 2) * 4);
		Memory_Data3 = *(u32 *)(Over_Data_Linear_address + (i * 4 + 3) * 4);
	        printk ("%8x: %8x  %8x  %8x  %8x\n", Kernal_IO_Data.Address + (i * 4 * 4), Memory_Data0, Memory_Data1, Memory_Data2, Memory_Data3);
	    }
            *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
            iounmap (Over_Data_Linear_address);
            ret = 0;
        break;

        case IOCTL_MEMORY_OVER_WRITE:
            Over_Data_Linear_address = ioremap_nocache (Kernal_IO_Data.Address, Kernal_IO_Data.Data * 4);
	    for (i = 0; i < Kernal_IO_Data.Data / 4; i++) {
		*(u32 *)(Over_Data_Linear_address + i * 4) = Kernal_IO_Data.Value;
	    }
            *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
            iounmap (Over_Data_Linear_address);
            ret = 0;
        break;

        default:
            ret = 3;
    }
    return ret;
}

int my_init(void)
{
    int    result;

    result = register_chrdev (ioaccess_major, "access", &ioaccess_fops);
    if (result < 0) {
        printk ("<0>" "<0>Can't get major.\n");
        return result;
    }
    if (ioaccess_major == 0) {
        ioaccess_major = result;
        printk ("<0>" "A ioaccess_major = %d\n", ioaccess_major);
    }

    return 0;
}
                                                                                
void my_cleanup(void)
{
    unregister_chrdev (ioaccess_major, "access");

    return;
}


module_init (my_init);
module_exit (my_cleanup);
