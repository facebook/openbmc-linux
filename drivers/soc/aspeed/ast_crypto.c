/********************************************************************************
* File Name     : ast_crypto.c
* Author         : Ryan Chen
* Description   : AST Crypto Controller
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
*
*   Version      : 1.0
*   History      : 
*      1. 2013/01/30 Ryan Chen create this file 
*    
********************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>

#include <linux/completion.h>
#include <linux/slab.h>

#include <linux/sched.h>  
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>


#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/platform.h>
#include <mach/hardware.h>
#include <plat/ast-scu.h>
#include "aes.h"

/*****************************************************************************************************************/
#define HAC_REG_BASE 				0x1e6e3000

#define CRYPTO_SRC_ADDR			0x00			/*Base addr of crypto data source : 8 byte aligned */
#define CRYPTO_DEST_ADDR			0x04			/*Base addr of crypto data destination : 8 byte aligned */
#define CRYPTO_CONTEXT_ADDR		0x08			/*Base addr of crypto context buffer : 8 byte aligned */
#define CRYPTO_DATA_LEN				0x0C
#define CRYPTO_ENG_CMD				0x10
/* 0x14 */
#define HAC_ENG_TAG					0x18
#define HAC_ENG_STS					0x1C
#define HASH_SRC_ADDR				0x20
#define HASH_DEST_ADDR				0x24
#define HASH_HMAC_KEY_ADDR			0x28
#define HASH_DATA_LEN				0x2C
#define HASH_ENG_CMD				0x30

#define RSA_EXPONENT_VAL			0x40
#define RSA_ENG_CMD					0x4C
#define CMD_QUEUE_ADDR				0x50
#define CMD_QUEUE_END				0x54
#define CMD_QUEUE_WRITE			0x58
#define CMD_QUEUE_READ				0x5C
#define HAC_ENG_FEATURE				0x60

/*****************************************************************************************************************/

#define HASH_BUSY			0x01
#define CRYPTO_BUSY			0x02

#define CRYPTO_SYNC_MODE_MASK		0x03
#define CRYPTO_SYNC_MODE_ASYNC		0x00
#define CRYPTO_SYNC_MODE_PASSIVE	0x02
#define CRYPTO_SYNC_MODE_ACTIVE		0x03

#define CRYPTO_AES128			0x00
#define CRYPTO_AES192			0x04
#define CRYPTO_AES256			0x08

#define CRYPTO_AES_ECB			0x00
#define CRYPTO_AES_CBC			0x10
#define CRYPTO_AES_CFB			0x20
#define CRYPTO_AES_OFB			0x30
#define CRYPTO_AES_CTR			0x40

#define CRYPTO_ENCRYPTO			0x80
#define CRYPTO_DECRYPTO			0x00

#define CRYPTO_AES			0x000
#define CRYPTO_RC4			0x100

#define CRYPTO_ENABLE_RW		0x000
#define CRYPTO_ENABLE_CONTEXT_LOAD	0x000
#define CRYPTO_ENABLE_CONTEXT_SAVE	0x000

#define HASH_SYNC_MODE_MASK		0x03
#define HASH_SYNC_MODE_ASYNC		0x00
#define HASH_SYNC_MODE_PASSIVE		0x02
#define HASH_SYNC_MODE_ACTIVE		0x03

#define HASH_READ_SWAP_ENABLE		0x04
#define HMAC_SWAP_CONTROL_ENABLE	0x08

#define HASH_ALG_SELECT_MASK		0x70
#define HASH_ALG_SELECT_MD5		0x00
#define HASH_ALG_SELECT_SHA1		0x20
#define HASH_ALG_SELECT_SHA224		0x40
#define HASH_ALG_SELECT_SHA256		0x50

#define HAC_ENABLE			0x80
#define HAC_DIGEST_CAL_ENABLE		0x180
#define HASH_INT_ENABLE			0x200
////////////////////////////////////////////////////////////////////////////
/* IOCTL */
#define  IOCTL_AES_KEY       	0x1103
#define  IOCTL_AES_IV       	0x1104
#define  IOCTL_AES_ENCRYPTO	0x1105
#define  IOCTL_AES_DECRYPTO	0x1106
#define  IOCTL_RC4_KEY       	0x1107
#define  IOCTL_RC4_CRYPTO    	0x1108

/* General */
#define MAX_AESTEXTLENGTH		(256 + 8)
#define MAX_RC4TEXTLENGTH		(256 + 8)
#define MAX_RC4KEYLENGTH		256 

#define CRYPTOMODE_ECB			0x00
#define CRYPTOMODE_CBC			0x01
#define CRYPTOMODE_CFB			0x02
#define CRYPTOMODE_OFB			0x03
#define CRYPTOMODE_CTR			0x04

#define HASHMODE_MD5			0x00
#define HASHMODE_SHA1			0x01
#define HASHMODE_SHA256			0x02
#define HASHMODE_SHA224			0x03

#define MIXMODE_DISABLE                 0x00
#define MIXMODE_CRYPTO                  0x02
#define MIXMODE_HASH                    0x03

struct ast_aes_context
{
    u32 erk[64];     /* encryption round keys */
    u32 drk[64];     /* decryption round keys */
    int nr;             /* number of rounds */
};

struct ast_aes_key {
    u32  keylength;	
    u8  aes_key[32];
};

struct ast_aes_iv {
    u8 aes_iv[16];
};

struct ast_aes_crypto {
    u32 aes_mode;	
    u32 aes_msglength;	
    u8 aes_plaintext[MAX_AESTEXTLENGTH];
    u8 aes_ciphertext[MAX_AESTEXTLENGTH];
    	
};

struct rc4_crypto_data {	
    u32 rc4_msglength;	
    u8 rc4_plaintext[MAX_AESTEXTLENGTH];
    u8 rc4_ciphertext[MAX_AESTEXTLENGTH];
    	
};

struct rc4_key_data {
    u32  keylength;	
    u8  rc4_key[MAX_RC4KEYLENGTH];
};

//  RC4 structure
struct rc4_state
{
    int x;
    int y;
    int m[256];
};

//#define CONFIG_AST_CRYPTO_DEBUG

#ifdef CONFIG_AST_CRYPTO_DEBUG
	#define CRYPTO_DBG(fmt, args...) printk("%s(): " fmt, __FUNCTION__, ## args)
#else
	#define CRYPTO_DBG(fmt, args...)
#endif

/***********************************************************************/
//#define AST_CRYPTO_DEBUG

#ifdef AST_CRYPTO_DEBUG
#define CRYPTO_DBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define CRYPTO_DBUG(fmt, args...)
#endif


struct ast_crypto_data {
	struct platform_device 	*pdev;
	void __iomem			*reg;			/* virtual */	
	int 					irq;					//LPC IRQ number 
	bool 					is_open;	
	u8					*src_virt_buff;
	dma_addr_t 			src_phy_buff;
	u8					*dest_virt_buff;
	dma_addr_t 			dest_phy_buff;
	u8					*context_virt_buff;
	dma_addr_t 			context_phy_buff;	
};



/******************************************************************************/
static DEFINE_SPINLOCK(crypto_state_lock);
/******************************************************************************/

/*********************************************************************************************************************************/
static inline u32 
ast_crypto_read(struct ast_crypto_data *ast_crypto, u32 reg)
{
#if 0
	u32 val;
	val = readl(ast_crypto->reg + reg);
	CRYPTO_DBUG("ast_crypto_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	return readl(ast_crypto->reg + reg);
#endif
}

static inline void
ast_crypto_write(struct ast_crypto_data *ast_crypto, u32 val, u32 reg) 
{
//	CRYPTO_DBUG("ast_crypto_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_crypto->reg + reg);
}
/*********************************************************************************************************************************/

/* AES */

void aes_key_ast2100(struct ast_crypto_data *ast_crypto, struct ast_aes_context *ctx, struct ast_aes_key *aes_key_info)
{
    u32 i, ulTemp;
    u32 contextbase;

    contextbase = ast_crypto->context_virt_buff;
    
    aes_set_key(ctx, aes_key_info->aes_key, aes_key_info->keylength);

    /* Set Expansion Key */   
    for (i=0; i<(4*(ctx->nr+1)); i++)
    {
        ulTemp = ((ctx->erk[i] & 0xFF) << 24) + ((ctx->erk[i] & 0xFF00) << 8) + ((ctx->erk[i] & 0xFF0000) >> 8) + ((ctx->erk[i] & 0xFF000000) >> 24);
    	*(u32 *) (contextbase + i*4 + 16) = ulTemp;    	
    }
	
}

void aes_iv_ast2100(struct ast_crypto_data *ast_crypto, struct ast_aes_iv *aes_iv_info)
{
    u32 i;
    u32 contextbase;
    u8 ch;

    contextbase = ast_crypto->context_virt_buff;

    for (i=0; i<16; i++) {
        ch = aes_iv_info->aes_iv[i];
    	*(u8 *) (contextbase + i) = ch;            	
    }
    	
}

void aes_enc_ast2100(struct ast_crypto_data *ast_crypto, struct ast_aes_context *ctx, struct ast_aes_crypto *aes_crypto_info)
{
	u32 i, ulTemp, ulCommand;
	u8 ch;
	u32 srcbase, dstbase;

	srcbase = ast_crypto->src_virt_buff;
	dstbase = ast_crypto->dest_virt_buff;

	ulCommand = CRYPTO_ENABLE_RW | CRYPTO_ENABLE_CONTEXT_LOAD | CRYPTO_ENABLE_CONTEXT_SAVE | \
	        CRYPTO_AES | CRYPTO_ENCRYPTO | CRYPTO_SYNC_MODE_ASYNC;

	switch (ctx->nr)
	{
		case 10:
		    ulCommand |= CRYPTO_AES128;
		    break;
		case 12:
		    ulCommand |= CRYPTO_AES192;
		    break;    
		case 14:
		    ulCommand |= CRYPTO_AES256;
		    break;    
	}

	switch (aes_crypto_info->aes_mode)
	{
		case CRYPTOMODE_ECB:
		    ulCommand |= CRYPTO_AES_ECB;
		    break;
		case CRYPTOMODE_CBC:
		    ulCommand |= CRYPTO_AES_CBC;
		    break;
		case CRYPTOMODE_CFB:
		    ulCommand |= CRYPTO_AES_CFB;
		    break;
		case CRYPTOMODE_OFB:
		    ulCommand |= CRYPTO_AES_OFB;
		    break;
		case CRYPTOMODE_CTR:
		    ulCommand |= CRYPTO_AES_CTR;
		    break;                
	}
	
	/* Init HW */
	ast_crypto_write(ast_crypto, aes_crypto_info->aes_msglength, CRYPTO_DATA_LEN);
          	
	/* Set source */
	for (i=0; i< aes_crypto_info->aes_msglength; i++) {
		ch = aes_crypto_info->aes_plaintext[i];
		*(u8 *) (srcbase + i) = ch;
	}
     
    	/* fire cmd */
	ast_crypto_write(ast_crypto, ulCommand, CRYPTO_ENG_CMD);
	do {
		ulTemp = ast_crypto_read(ast_crypto, HAC_ENG_STS);
	} while (ulTemp & CRYPTO_BUSY);

	/* Output */
	for (i=0; i<aes_crypto_info->aes_msglength; i++) 	{
		ch = *(u8 *) (dstbase + i);
		aes_crypto_info->aes_ciphertext[i] = ch;    
	}
    
} /* aes_ecb_enc_ast2100 */

void aes_dec_ast2100(struct ast_crypto_data *ast_crypto,  struct ast_aes_context *ctx, struct ast_aes_crypto *aes_crypto_info)
{
	u32 i, ulTemp, ulCommand;
	u8 ch;
	u32 srcbase, dstbase;
		
	srcbase = ast_crypto->src_virt_buff;
	dstbase = ast_crypto->dest_virt_buff;

	ulCommand = CRYPTO_ENABLE_RW | CRYPTO_ENABLE_CONTEXT_LOAD | CRYPTO_ENABLE_CONTEXT_SAVE |
	            CRYPTO_AES | CRYPTO_DECRYPTO | CRYPTO_SYNC_MODE_ASYNC;

	switch (ctx->nr)
	{
		case 10:
		    ulCommand |= CRYPTO_AES128;
		    break;
		case 12:
		    ulCommand |= CRYPTO_AES192;
		    break;    
		case 14:
		    ulCommand |= CRYPTO_AES256;
		    break;    
	}

	switch (aes_crypto_info->aes_mode)
	{
		case CRYPTOMODE_ECB:
		    ulCommand |= CRYPTO_AES_ECB;
		    break;
		case CRYPTOMODE_CBC:
		    ulCommand |= CRYPTO_AES_CBC;
		    break;
		case CRYPTOMODE_CFB:
		    ulCommand |= CRYPTO_AES_CFB;
		    break;
		case CRYPTOMODE_OFB:
		    ulCommand |= CRYPTO_AES_OFB;
		    break;
		case CRYPTOMODE_CTR:
		    ulCommand |= CRYPTO_AES_CTR;
		    break;                
	}
	
	/* Init HW */
	ast_crypto_write(ast_crypto, aes_crypto_info->aes_msglength, CRYPTO_DATA_LEN);

	/* Set source */
	for (i=0; i< aes_crypto_info->aes_msglength; i++) {
		ch = aes_crypto_info->aes_ciphertext[i];
		*(u8 *) (srcbase + i) = ch;
	}

    	/* fire cmd */
	ast_crypto_write(ast_crypto, ulCommand, CRYPTO_ENG_CMD);
	do {
		ulTemp = ast_crypto_read(ast_crypto, HAC_ENG_STS);
	} while (ulTemp & CRYPTO_BUSY);

	/* Output */
	for (i=0; i< aes_crypto_info->aes_msglength; i++)	{
		ch = *(u8 *) (dstbase + i);
		aes_crypto_info->aes_plaintext[i] = ch;    
	}

} /* aes_ecb_dec_ast2100 */

/* RC4 */
void rc4_setup( struct rc4_state *s, unsigned char *key,  int length )
{
	int i, j, k, *m, a;

	s->x = 0;
	s->y = 0;
	m = s->m;

	for( i = 0; i < 256; i++ )
	{
		m[i] = i;
	}

	j = k = 0;

	for( i = 0; i < 256; i++ )
	{
		a = m[i];
		j = (unsigned char) ( j + a + key[k] );
		m[i] = m[j]; m[j] = a;
		if( ++k >= length ) k = 0;
	}
}

void ast_rc4_set_key(struct ast_crypto_data *ast_crypto, struct rc4_key_data *rc4_key_info)
{
	struct rc4_state s;
	u32 i, ulTemp;

	rc4_setup( &s, rc4_key_info->rc4_key, rc4_key_info->keylength);

	/* Set Context */
	
	/* Set i, j */
	*(u32 *) (ast_crypto->context_virt_buff + 8) = 0x0001;

	/* Set Expansion Key */   
	for (i=0; i<(256/4); i++)
	{
		ulTemp = (s.m[i * 4] & 0xFF) + ((s.m[i * 4 + 1] & 0xFF) << 8) + ((s.m[i * 4 + 2] & 0xFF) << 16) + ((s.m[i * 4+ 3] & 0xFF) << 24);
		*(u32 *) (ast_crypto->context_virt_buff + i*4 + 16) = ulTemp; 
	}
	
} /* ast_rc4_set_key */

	
void ast_rc4_encrypto(struct ast_crypto_data *ast_crypto, struct rc4_crypto_data *rc4_crypto_info)
{
	u32 i, ulTemp;
		
	/* Init HW */
	ast_crypto_write(ast_crypto, rc4_crypto_info->rc4_msglength, CRYPTO_DATA_LEN);
          	
	/* Set source */
	for (i=0; i< rc4_crypto_info->rc4_msglength; i++)
		ast_crypto->src_virt_buff[i] = rc4_crypto_info->rc4_plaintext[i];
     
	/* fire cmd */
	ast_crypto_write(ast_crypto, CRYPTO_ENABLE_RW | CRYPTO_ENABLE_CONTEXT_LOAD | CRYPTO_ENABLE_CONTEXT_SAVE | \
	            CRYPTO_RC4 | CRYPTO_SYNC_MODE_ASYNC, CRYPTO_ENG_CMD);
	
	do {
		ulTemp = ast_crypto_read(ast_crypto, HAC_ENG_STS);
	} while (ulTemp & CRYPTO_BUSY);	

	/* Output */
	for (i=0; i< rc4_crypto_info->rc4_msglength; i++)
		rc4_crypto_info->rc4_ciphertext[i] = ast_crypto->dest_virt_buff[i];
  
} 


/***************************************************************************/
struct ast_aes_context aes_ctx;

static long crypto_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *c = file->private_data;
	struct ast_crypto_data *ast_crypto = dev_get_drvdata(c->this_device);
	struct rc4_crypto_data *rc4_crypto;
	struct rc4_key_data *rc4_key;
	struct ast_aes_key *ast_aes_key_info;
	struct ast_aes_iv  *ast_aes_iv_info;
	struct ast_aes_crypto *ast_aes_crypto_info;

	long ret = 0;
	u32 i, j, msglen; 
	switch (cmd) {
		case IOCTL_AES_KEY:
			ast_aes_key_info = (struct ast_aes_key *)arg;
			aes_key_ast2100(ast_crypto, &aes_ctx, ast_aes_key_info);
		break;
		case IOCTL_AES_IV:
			ast_aes_iv_info = (struct ast_aes_iv *)arg;
			aes_iv_ast2100(ast_crypto, ast_aes_iv_info);
			break;

		case IOCTL_AES_ENCRYPTO:
			ast_aes_crypto_info = (struct ast_aes_crypto *)arg;

			/* 16bytes Align */
			msglen = ast_aes_crypto_info->aes_msglength;
			j = ( (ast_aes_crypto_info->aes_msglength + 15) >> 4) << 4;
			for (i=ast_aes_crypto_info->aes_msglength; i<j; i++)
				ast_aes_crypto_info->aes_plaintext[i] = 0;
			ast_aes_crypto_info->aes_msglength = j;    

			aes_enc_ast2100(ast_crypto, &aes_ctx, ast_aes_crypto_info);

			ast_aes_crypto_info->aes_msglength = msglen;
			if (copy_to_user((void *)arg, ast_aes_crypto_info, sizeof(struct ast_aes_crypto)))
				ret = -1;
			break;

		case IOCTL_AES_DECRYPTO:
			ast_aes_crypto_info = (struct ast_aes_crypto *)arg;

			/* 16bytes Align */
			msglen = ast_aes_crypto_info->aes_msglength;
			j = ( (ast_aes_crypto_info->aes_msglength + 15) >> 4) << 4;
			for (i=ast_aes_crypto_info->aes_msglength; i<j; i++)
				ast_aes_crypto_info->aes_ciphertext[i] = 0;
			
			ast_aes_crypto_info->aes_msglength = j;    

			aes_dec_ast2100(ast_crypto, &aes_ctx, ast_aes_crypto_info);

			ast_aes_crypto_info->aes_msglength = msglen;
			if (copy_to_user((void *)arg, ast_aes_crypto_info, sizeof(struct ast_aes_crypto)))
				ret = -1;
			break;
		case IOCTL_RC4_KEY:
			rc4_key = (struct rc4_key_data *)arg;
			ast_rc4_set_key(ast_crypto, rc4_key);
			break;
	
		case IOCTL_RC4_CRYPTO:
			rc4_crypto = (struct rc4_crypto_data *)arg;

			/* 16bytes Align */
			msglen = rc4_crypto->rc4_msglength;
			j = ( (rc4_crypto->rc4_msglength + 15) >> 4) << 4;
			
			for (i=rc4_crypto->rc4_msglength; i<j; i++)
				rc4_crypto->rc4_plaintext[i] = 0;
			
			rc4_crypto->rc4_msglength = j;    

			ast_rc4_encrypto(ast_crypto, rc4_crypto);

			rc4_crypto->rc4_msglength = msglen;
			if (copy_to_user((void *)arg, rc4_crypto, sizeof(struct rc4_crypto_data)))
				ret = -1;
			break;

		default:
			ret = -1;
			break;
	} /* cmd */

	return ret;
}

static int crypto_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_crypto_data *ast_crypto = dev_get_drvdata(c->this_device);

	CRYPTO_DBUG("\n");	
	spin_lock(&crypto_state_lock);

	if (ast_crypto->is_open) {
		spin_unlock(&crypto_state_lock);
		return -EBUSY;
	}

	ast_crypto->is_open = true;

	spin_unlock(&crypto_state_lock);

	return 0;
}

static int crypto_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_crypto_data *ast_crypto = dev_get_drvdata(c->this_device);

	CRYPTO_DBUG("\n");	
	spin_lock(&crypto_state_lock);

	ast_crypto->is_open = false;
	spin_unlock(&crypto_state_lock);

	return 0;
}	
static const struct file_operations ast_crypto_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl 	= crypto_ioctl,
	.open			= crypto_open,
	.release			= crypto_release,
};

struct miscdevice ast_crypto_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-crypto",
	.fops = &ast_crypto_fops,
};

static irqreturn_t ast_crypto_isr (int this_irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static int __init ast_crypto_probe(struct platform_device *pdev)
{
	static struct ast_crypto_data *ast_crypto;
	struct resource *res;
	int ret = 0;
	CRYPTO_DBUG("\n");	
	
	ast_scu_init_hace();
	
	ast_crypto = kzalloc(sizeof(struct ast_crypto_data), GFP_KERNEL);
	if (ast_crypto == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	ast_crypto->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free;
	}

	ast_crypto->reg = ioremap(res->start, resource_size(res));
	if (ast_crypto->reg == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	ast_crypto->irq = platform_get_irq(pdev, 0);
	if (ast_crypto->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto err_free_mem;
	}
#if 0
	ret = request_irq(ast_crypto->irq, ast_crypto_isr, IRQF_SHARED, "ast-crypto", ast_crypto);
	if (ret) {
		printk("AST ESPI Unable to get IRQ");
		goto err_free_mem;
	}
#endif
	// 8-byte aligned

	ast_crypto->src_virt_buff = dma_alloc_coherent(&pdev->dev,
					 PAGE_SIZE,
					 &ast_crypto->src_phy_buff, GFP_KERNEL);

	ast_crypto->dest_virt_buff = (u8 *) (ast_crypto->src_virt_buff + 0x200);
	ast_crypto->context_virt_buff = (u8 *) (ast_crypto->dest_virt_buff + 0x200);
	
//	ast_crypto->dest_phy_buff = ast_crypto->src_phy_buff + 0x200;
//	ast_crypto->context_phy_buff = ast_crypto->dest_phy_buff + 0x200;

//	printk("dma src %x, dest %x, context %x \n", ast_crypto->src_phy_buff, ast_crypto->dest_phy_buff, ast_crypto->context_phy_buff);

	/* Init H/W Engines */
	ast_crypto_write(ast_crypto, ast_crypto->src_phy_buff, CRYPTO_SRC_ADDR);
	ast_crypto_write(ast_crypto, ast_crypto->src_phy_buff + 0x200, CRYPTO_DEST_ADDR);
	ast_crypto_write(ast_crypto, ast_crypto->src_phy_buff + 0x400, CRYPTO_CONTEXT_ADDR);

	ret = misc_register(&ast_crypto_misc);
	if (ret){		
		printk(KERN_ERR "Crypto : failed misc_register\n");
		goto err_free_irq;
	}

	platform_set_drvdata(pdev, ast_crypto);
	dev_set_drvdata(ast_crypto_misc.this_device, ast_crypto);

	return 0;

	err_free_irq:
	free_irq(ast_crypto->irq, pdev);

	err_free_mem:
	release_mem_region(res->start, resource_size(res));
	err_free:
	kfree(ast_crypto);

	return ret;
}


static int __exit ast_crypto_remove(struct platform_device *pdev)
{
	struct ast_crypto_data *ast_crypto;
	struct resource *res;

	CRYPTO_DBUG("\n");
	ast_crypto = platform_get_drvdata(pdev);
	if (ast_crypto == NULL)
		return -ENODEV;

	iounmap(ast_crypto->reg);

	/****************************************************/
//	dma_free_coherent(pdev->dev, s,c,h)
	/****************************************************/

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	kfree(ast_crypto);

	return 0;
}

static const struct platform_device_id ast_crypto_idtable[] = {
	{
		.name = "ast-crypto",
//		.driver_data = ast_video_data,
		/* sentinel */
	},
	{
	}
};
MODULE_DEVICE_TABLE(platform, ast_crypto_idtable);

static struct platform_driver ast_crypto_driver = {
	.driver		= {
		.name	= "ast-crypto",
		.owner	= THIS_MODULE,
	},
	.remove		= ast_crypto_remove,
	.id_table	= ast_crypto_idtable,		
};

module_platform_driver_probe(ast_crypto_driver, ast_crypto_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST Crypto driver");
MODULE_LICENSE("GPL");

