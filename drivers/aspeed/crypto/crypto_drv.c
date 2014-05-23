#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <linux/socket.h>
#include <net/sock.h>
#include <linux/tcp.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "crypto_drv.h"

#include "aes.c"
#include "rc4.c"
#include "crypto_hw.c"

MODULE_LICENSE ("GPL");
MODULE_AUTHOR  ("Y.C. Chen");

#define VERSION	"0.32.00"

/* Global */
int crypto_major = 0;
int chip_type = 0;
_CRYPTO_HW crypto_hwinfo;
aes_context aes_ctx;
_AES_KEY aes_key_info;
_AES_IV aes_iv_info;
_AES_CRYPTO aes_crypto_info;
_RC4_KEY rc4_key_info;
_RC4_CRYPTO rc4_crypto_info;

int crypto_ioctl (struct inode *inode, struct file *filp, unsigned cmd, unsigned long arg);

struct file_operations crypto_fops = {
    ioctl:    crypto_ioctl,
};
                                                                   
int crypto_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg)
{
    long ret = 0;
    ULONG i, j, msglen; 
    
    switch (cmd) {
    case IOCTL_AES_KEY:
        aes_key_info = *(_AES_KEY *)arg;

        if (chip_type == AST2000)
            aes_key_ast2000(&aes_ctx, &aes_key_info, &crypto_hwinfo);        
	else
            aes_key_ast2100(&aes_ctx, &aes_key_info, &crypto_hwinfo);
        	        
        break;
        
    case IOCTL_AES_IV:
        aes_iv_info = *(_AES_IV *)arg;

        if (chip_type == AST2000)
            aes_iv_ast2000(&aes_iv_info, &crypto_hwinfo);
        else    
            aes_iv_ast2100(&aes_iv_info, &crypto_hwinfo);
    	
    	break;

    case IOCTL_AES_ENCRYPTO:
        aes_crypto_info = *(_AES_CRYPTO *)arg;

        /* 16bytes Align */
        msglen = aes_crypto_info.aes_msglength;
        j = ( (aes_crypto_info.aes_msglength + 15) >> 4) << 4;
        for (i=aes_crypto_info.aes_msglength; i<j; i++)
            aes_crypto_info.aes_plaintext[i] = 0;
        aes_crypto_info.aes_msglength = j;    

        if (chip_type == AST2000)
            aes_enc_ast2000(&aes_ctx, &aes_crypto_info, &crypto_hwinfo);
        else    
            aes_enc_ast2100(&aes_ctx, &aes_crypto_info, &crypto_hwinfo);
            
        aes_crypto_info.aes_msglength = msglen;
        if (copy_to_user((void *)arg, &aes_crypto_info, sizeof(_AES_CRYPTO)))
        {
	     ret = -1;
	}
        
    	break;
    	
    case IOCTL_AES_DECRYPTO:
        aes_crypto_info = *(_AES_CRYPTO *)arg;

        /* 16bytes Align */
        msglen = aes_crypto_info.aes_msglength;
        j = ( (aes_crypto_info.aes_msglength + 15) >> 4) << 4;
        for (i=aes_crypto_info.aes_msglength; i<j; i++)
            aes_crypto_info.aes_ciphertext[i] = 0;
        aes_crypto_info.aes_msglength = j;    

        if (chip_type == AST2000)
            aes_dec_ast2000(&aes_ctx, &aes_crypto_info, &crypto_hwinfo);
        else    
            aes_dec_ast2100(&aes_ctx, &aes_crypto_info, &crypto_hwinfo);

        aes_crypto_info.aes_msglength = msglen;
        if (copy_to_user((void *)arg, &aes_crypto_info, sizeof(_AES_CRYPTO)))
        {
	     ret = -1;
	}
        
    	break;

    case IOCTL_RC4_KEY:
        rc4_key_info = *(_RC4_KEY *)arg;

        if (chip_type == AST2000)
            rc4_key_ast2000(&rc4_key_info, &crypto_hwinfo);
        else    
            rc4_key_ast2100(&rc4_key_info, &crypto_hwinfo);
        	        
        break;

    case IOCTL_RC4_CRYPTO:
        rc4_crypto_info = *(_RC4_CRYPTO *)arg;

        /* 16bytes Align */
        msglen = rc4_crypto_info.rc4_msglength;
        j = ( (rc4_crypto_info.rc4_msglength + 15) >> 4) << 4;
        for (i=rc4_crypto_info.rc4_msglength; i<j; i++)
            rc4_crypto_info.rc4_plaintext[i] = 0;
        rc4_crypto_info.rc4_msglength = j;    

        if (chip_type == AST2000)
            rc4_crypt_ast2000(&rc4_crypto_info, &crypto_hwinfo);
        else    
            rc4_crypt_ast2100(&rc4_crypto_info, &crypto_hwinfo);

        rc4_crypto_info.rc4_msglength = msglen;
        if (copy_to_user((void *)arg, &rc4_crypto_info, sizeof(_RC4_CRYPTO)))
        {
	     ret = -1;
	}
        
    	break;
      		    	
    default:
        ret = -1;
            
    } /* cmd */

    return ret;
}

void CheckChip(void)
{
    ULONG ulTemp;
    	
    ReadDD_SOC(IO_ADDRESS(0x1e6e0000) + 0x207c, ulTemp);
    if (ulTemp == 0)
    {
        chip_type = AST2000;
        printk(KERN_INFO "AST1000/2000 has ben detected!! \n");
    }    
    else
    {
        chip_type = AST2100;
        printk(KERN_INFO "AST1100/2050/2100 has ben detected!! \n");                
    }

}
	
int my_init(void)
{
    int    result;

    printk(KERN_INFO "[Crypto] Load Crypto Driver %s \n", VERSION);
    
    result = register_chrdev (crypto_major, "crypto_drv", &crypto_fops);
    if (result < 0) {
        printk (KERN_ERR "[Crypto] Can't Get Major.\n");
        return result;
    }
    if (crypto_major == 0) {
        crypto_major = result;
	printk (KERN_INFO "[Crypto] Major = %d\n", crypto_major);
    }

    /* Check Chip */
    CheckChip();
    
    /* Alloc Resource */
    crypto_hwinfo.pjsrcvirt = (UCHAR *) kmalloc(MAX_AESTEXTLENGTH, GFP_KERNEL);
    crypto_hwinfo.pjdstvirt = (UCHAR *) kmalloc(MAX_AESTEXTLENGTH, GFP_KERNEL);
    crypto_hwinfo.pjcontextvirt = (UCHAR *) kmalloc(1024, GFP_KERNEL);
     
    printk(KERN_INFO "[Crypto] Crypto Src Virtual Address: %x \n", (ULONG) crypto_hwinfo.pjsrcvirt);
    printk(KERN_INFO "[Crypto] Crypto Dst Virtual Address: %x \n", (ULONG) crypto_hwinfo.pjdstvirt);
    printk(KERN_INFO "[Crypto] Crypto Context Virtual Address: %x \n", (ULONG) crypto_hwinfo.pjcontextvirt);
    
    /* Init H/W Engines */
    if (chip_type == AST2000)
        EnableHMAC2000(&crypto_hwinfo);    
    else
        EnableHMAC(&crypto_hwinfo);
       
    return 0;
}
                                                                                
void my_cleanup(void)
{
    kfree(crypto_hwinfo.pjsrcvirt);
    kfree(crypto_hwinfo.pjdstvirt);
    kfree(crypto_hwinfo.pjcontextvirt);
    	
    unregister_chrdev (crypto_major, "crypto_drv");
    return;
}

module_init (my_init);
module_exit (my_cleanup);
