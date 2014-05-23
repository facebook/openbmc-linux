/*
 * AST20000 Engines
 */
void EnableHMAC2000(_CRYPTO_HW *crypto_hwinfo)
{
    ULONG srcbase, dstbase, contextbase;
    	
    srcbase = ((ULONG) virt_to_phys(crypto_hwinfo->pjsrcvirt) + 7) & 0xFFFFFFF8;
    dstbase = ((ULONG) virt_to_phys(crypto_hwinfo->pjdstvirt) + 7) & 0xFFFFFFF8;
    contextbase = ((ULONG) virt_to_phys(crypto_hwinfo->pjcontextvirt) + 7) & 0xFFFFFFF8;

    printk(KERN_INFO "[Crypto] Crypto Src Base Address: %x \n", (ULONG) srcbase);
    printk(KERN_INFO "[Crypto] Crypto Dst Base Address: %x \n", (ULONG) dstbase);
    printk(KERN_INFO "[Crypto] Crypto Context Base Address: %x \n", (ULONG) contextbase);
    
    /* init SCU */
    	
    /* Init HW */
    WriteDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_SRC_BASE_OFFSET, srcbase);
    WriteDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_DST_BASE_OFFSET, dstbase);
    WriteDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_CONTEXT_BASE_OFFSET, contextbase);
    WriteDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_ENABLE_OFFSET, 0x01);
    	
}

void DisableHMAC2000(void)
{
    /* Init HW */
}

/* AES */
void aes_key_ast2000(aes_context *ctx, _AES_KEY *aes_key_info, _CRYPTO_HW *crypto_hwinfo)
{
    ULONG i, ulTemp;
    ULONG contextbase;

    contextbase = ((ULONG) crypto_hwinfo->pjcontextvirt + 7) & 0xFFFFFFF8;
    
    aes_set_key(ctx, aes_key_info->aes_key, aes_key_info->keylength);

    /* Set Expansion Key */   
    for (i=0; i<(4*(ctx->nr+1)); i++)
    {
        ulTemp = ((ctx->erk[i] & 0xFF) << 24) + ((ctx->erk[i] & 0xFF00) << 8) + ((ctx->erk[i] & 0xFF0000) >> 8) + ((ctx->erk[i] & 0xFF000000) >> 24);
    	*(ULONG *) (contextbase + i*4) = ulTemp;    	
    }
	
}

void aes_iv_ast2000(_AES_IV *aes_iv_info, _CRYPTO_HW *crypto_hwinfo)
{
    ULONG i, ulTemp;
    ULONG *pjivinfo;

    pjivinfo = (ULONG *) aes_iv_info->aes_iv;
    
    for (i=0; i< 4; i++)
    {
        ulTemp = *(ULONG *) pjivinfo++;
        WriteDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_IV_OFFSET + i*4, ulTemp);
    }
    	
}
	
void aes_enc_ast2000(aes_context *ctx, _AES_CRYPTO *aes_crypto_info, _CRYPTO_HW *crypto_hwinfo)
{
    ULONG i, ulTemp, ulCommand;
    UCHAR ch;
    ULONG srcbase, dstbase;
    	
    srcbase = ((ULONG) crypto_hwinfo->pjsrcvirt + 7) & 0xFFFFFFF8;
    dstbase = ((ULONG) crypto_hwinfo->pjdstvirt + 7) & 0xFFFFFFF8;
    
    ulCommand = CRYPTO2000_ENABLE_IV | CRYPTO2000_EXPANDKEY | CRYPTO2000_AES | CRYPTO2000_ENCRYPTO;

    switch (ctx->nr)
    {
    case 10:
        ulCommand |= CRYPTO2000_AES128;
        break;
    case 12:
        ulCommand |= CRYPTO2000_AES192;
        break;    
    case 14:
        ulCommand |= CRYPTO2000_AES256;
        break;    
    }

    switch (aes_crypto_info->aes_mode)
    {
    case CRYPTOMODE_ECB:
        ulCommand |= CRYPTO2000_AES_ECB;
        break;
    case CRYPTOMODE_CBC:
        ulCommand |= CRYPTO2000_AES_CBC;
        break;
    case CRYPTOMODE_CFB:
        ulCommand |= CRYPTO2000_AES_CFB;
        break;
    case CRYPTOMODE_OFB:
        ulCommand |= CRYPTO2000_AES_OFB;
        break;
    case CRYPTOMODE_CTR:
        ulCommand |= CRYPTO2000_AES_CTR;
        break;                
    }
	
    /* Init HW */
    WriteDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_LEN_OFFSET, (aes_crypto_info->aes_msglength / 16));	
          	
    /* Set source */
    for (i=0; i< aes_crypto_info->aes_msglength; i++)
    {
    	ch = aes_crypto_info->aes_plaintext[i];
    	*(UCHAR *) (srcbase + i) = ch;
    }
     
    /* fire cmd */
    WriteDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) +  REG_CRYPTO2000_CMD_BASE_OFFSET, ulCommand);	
    do {
        ReadDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_STATUS_OFFSET, ulTemp);
    } while (ulTemp & CRYPTO2000_BUSY);

    /* Output */
    for (i=0; i<aes_crypto_info->aes_msglength; i++)
    {
    	ch = *(UCHAR *) (dstbase + i);
        aes_crypto_info->aes_ciphertext[i] = ch;    
    }
    
} /* aes_ecb_enc_ast2000 */

void aes_dec_ast2000(aes_context *ctx, _AES_CRYPTO *aes_crypto_info, _CRYPTO_HW *crypto_hwinfo)
{
    ULONG i, ulTemp, ulCommand;
    UCHAR ch;
    ULONG srcbase, dstbase;
    	
    srcbase = ((ULONG) crypto_hwinfo->pjsrcvirt + 7) & 0xFFFFFFF8;
    dstbase = ((ULONG) crypto_hwinfo->pjdstvirt + 7) & 0xFFFFFFF8;

    ulCommand = CRYPTO2000_ENABLE_IV | CRYPTO2000_EXPANDKEY | CRYPTO2000_AES | CRYPTO2000_DECRYPTO;

    switch (ctx->nr)
    {
    case 10:
        ulCommand |= CRYPTO2000_AES128;
        break;
    case 12:
        ulCommand |= CRYPTO2000_AES192;
        break;    
    case 14:
        ulCommand |= CRYPTO2000_AES256;
        break;    
    }

    switch (aes_crypto_info->aes_mode)
    {
    case CRYPTOMODE_ECB:
        ulCommand |= CRYPTO2000_AES_ECB;
        break;
    case CRYPTOMODE_CBC:
        ulCommand |= CRYPTO2000_AES_CBC;
        break;
    case CRYPTOMODE_CFB:
        ulCommand |= CRYPTO2000_AES_CFB;
        break;
    case CRYPTOMODE_OFB:
        ulCommand |= CRYPTO2000_AES_OFB;
        break;
    case CRYPTOMODE_CTR:
        ulCommand |= CRYPTO2000_AES_CTR;
        break;                
    }
	
    /* Init HW */
    WriteDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_LEN_OFFSET, (aes_crypto_info->aes_msglength / 16));

    /* Set source */
    for (i=0; i< aes_crypto_info->aes_msglength; i++)
    {
    	ch = aes_crypto_info->aes_ciphertext[i];
    	*(UCHAR *) (srcbase + i) = ch;
    }

    /* fire cmd */
    WriteDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_CMD_BASE_OFFSET, ulCommand);	
    do {
        ReadDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_STATUS_OFFSET,ulTemp);
    } while (ulTemp & CRYPTO2000_BUSY);	

    /* Output */
    for (i=0; i< aes_crypto_info->aes_msglength; i++)
    {
    	ch = *(UCHAR *) (dstbase + i);
        aes_crypto_info->aes_plaintext[i] = ch;    
    }

} /* aes_ecb_dec_ast2000 */

/* RC4 */
void rc4_key_ast2000(_RC4_KEY *rc4_key_info, _CRYPTO_HW *crypto_hwinfo)
{
    struct rc4_state s;
    ULONG i, ulTemp;
    ULONG contextbase;

    contextbase = ((ULONG) crypto_hwinfo->pjcontextvirt + 7) & 0xFFFFFFF8;
	
    rc4_setup( &s, rc4_key_info->rc4_key, rc4_key_info->keylength );

    /* Set Context */       
    /* Set Expansion Key */   
    for (i=0; i<(256/4); i++)
    {
       ulTemp = (s.m[i * 4] & 0xFF) + ((s.m[i * 4 + 1] & 0xFF) << 8) + ((s.m[i * 4 + 2] & 0xFF) << 16) + ((s.m[i * 4+ 3] & 0xFF) << 24);    	
       *(ULONG *) (contextbase + i*4) = ulTemp; 
    }
	
} /* rc4_key_ast2000 */

	
void rc4_crypt_ast2000(_RC4_CRYPTO *rc4_crypto_info, _CRYPTO_HW *crypto_hwinfo)
{
    ULONG i, ulTemp, ulCommand;
    UCHAR ch;
    ULONG srcbase, dstbase;
    	
    srcbase = ((ULONG) crypto_hwinfo->pjsrcvirt + 7) & 0xFFFFFFF8;
    dstbase = ((ULONG) crypto_hwinfo->pjdstvirt + 7) & 0xFFFFFFF8;
    
    ulCommand = CRYPTO2000_RC4 | CRYPTO2000_EXPANDKEY;

    /* Init HW */
    WriteDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_LEN_OFFSET, (rc4_crypto_info->rc4_msglength / 16));
          	
    /* Set source */
    for (i=0; i< rc4_crypto_info->rc4_msglength; i++)
    {
    	ch = rc4_crypto_info->rc4_plaintext[i];
    	*(UCHAR *) (srcbase + i) = ch;
    }
     
    /* fire cmd */
    WriteDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_CMD_BASE_OFFSET, ulCommand);	
    do {
        ReadDD_SOC(IO_ADDRESS(HAC2000_REG_BASE) + REG_CRYPTO2000_STATUS_OFFSET,ulTemp);
    } while (ulTemp & CRYPTO2000_BUSY);	

    /* Output */
    for (i=0; i< rc4_crypto_info->rc4_msglength; i++)
    {
    	ch = *(UCHAR *) (dstbase + i);
        rc4_crypto_info->rc4_ciphertext[i] = ch;    
    }
  
} /* rc4_crypt_ast2000 */

/*
 * AST1100/2050/2100 Engines
 */
void EnableHMAC(_CRYPTO_HW *crypto_hwinfo)
{
    ULONG ulData;
    ULONG srcbase, dstbase, contextbase;
    	
    srcbase = ((ULONG) virt_to_phys(crypto_hwinfo->pjsrcvirt) + 7) & 0xFFFFFFF8;
    dstbase = ((ULONG) virt_to_phys(crypto_hwinfo->pjdstvirt) + 7) & 0xFFFFFFF8;
    contextbase = ((ULONG) virt_to_phys(crypto_hwinfo->pjcontextvirt) + 7) & 0xFFFFFFF8;

    printk(KERN_INFO "[Crypto] Crypto Src Base Address: %x \n", (ULONG) srcbase);
    printk(KERN_INFO "[Crypto] Crypto Dst Base Address: %x \n", (ULONG) dstbase);
    printk(KERN_INFO "[Crypto] Crypto Context Base Address: %x \n", (ULONG) contextbase);
    
    /* init SCU */
    WriteDD_SOC(IO_ADDRESS(SCU_REG_BASE), 0x1688A8A8);	
    ReadDD_SOC(IO_ADDRESS(SCU_REG_BASE) + 0x000c, ulData);	
    ulData &= 0xfdfff;
    WriteDD_SOC(IO_ADDRESS(SCU_REG_BASE) + 0x000c, ulData);	
    mdelay(1);
    ReadDD_SOC(IO_ADDRESS(SCU_REG_BASE) + 0x0004, ulData);	
    ulData &= 0xfffef;
    WriteDD_SOC(IO_ADDRESS(SCU_REG_BASE) + 0x0004, ulData);
    	
    /* Init HW */
    WriteDD_SOC(IO_ADDRESS(HAC_REG_BASE) + 0x0100, 0xA8);
    WriteDD_SOC(IO_ADDRESS(HAC_REG_BASE) + REG_CRYPTO_SRC_BASE_OFFSET, srcbase);
    WriteDD_SOC(IO_ADDRESS(HAC_REG_BASE) + REG_CRYPTO_DST_BASE_OFFSET, dstbase);
    WriteDD_SOC(IO_ADDRESS(HAC_REG_BASE) + REG_CRYPTO_CONTEXT_BASE_OFFSET, contextbase);
    	
}

void DisableHMAC(void)
{
    /* Init HW */
}

/* AES */
void aes_key_ast2100(aes_context *ctx, _AES_KEY *aes_key_info, _CRYPTO_HW *crypto_hwinfo)
{
    ULONG i, ulTemp;
    ULONG contextbase;

    contextbase = ((ULONG) crypto_hwinfo->pjcontextvirt + 7) & 0xFFFFFFF8;
    
    aes_set_key(ctx, aes_key_info->aes_key, aes_key_info->keylength);

    /* Set Expansion Key */   
    for (i=0; i<(4*(ctx->nr+1)); i++)
    {
        ulTemp = ((ctx->erk[i] & 0xFF) << 24) + ((ctx->erk[i] & 0xFF00) << 8) + ((ctx->erk[i] & 0xFF0000) >> 8) + ((ctx->erk[i] & 0xFF000000) >> 24);
    	*(ULONG *) (contextbase + i*4 + 16) = ulTemp;    	
    }
	
}

void aes_iv_ast2100(_AES_IV *aes_iv_info, _CRYPTO_HW *crypto_hwinfo)
{
    ULONG i;
    ULONG contextbase;
    UCHAR ch;

    contextbase = ((ULONG) crypto_hwinfo->pjcontextvirt + 7) & 0xFFFFFFF8;

    for (i=0; i<16; i++)
    {
        ch = aes_iv_info->aes_iv[i];
    	*(UCHAR *) (contextbase + i) = ch;            	
    }
    	
}
	
void aes_enc_ast2100(aes_context *ctx, _AES_CRYPTO *aes_crypto_info, _CRYPTO_HW *crypto_hwinfo)
{
    ULONG i, ulTemp, ulCommand;
    UCHAR ch;
    ULONG srcbase, dstbase;
    	
    srcbase = ((ULONG) crypto_hwinfo->pjsrcvirt + 7) & 0xFFFFFFF8;
    dstbase = ((ULONG) crypto_hwinfo->pjdstvirt + 7) & 0xFFFFFFF8;
    
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
    WriteDD_SOC(IO_ADDRESS(HAC_REG_BASE) + REG_CRYPTO_LEN_OFFSET, aes_crypto_info->aes_msglength);	
          	
    /* Set source */
    for (i=0; i< aes_crypto_info->aes_msglength; i++)
    {
    	ch = aes_crypto_info->aes_plaintext[i];
    	*(UCHAR *) (srcbase + i) = ch;
    }
     
    /* fire cmd */
    WriteDD_SOC(IO_ADDRESS(HAC_REG_BASE) +  REG_CRYPTO_CMD_BASE_OFFSET, ulCommand);	
    do {
        ReadDD_SOC(IO_ADDRESS(HAC_REG_BASE) + REG_CRYPTO_STATUS_OFFSET, ulTemp);
    } while (ulTemp & CRYPTO_BUSY);

    /* Output */
    for (i=0; i<aes_crypto_info->aes_msglength; i++)
    {
    	ch = *(UCHAR *) (dstbase + i);
        aes_crypto_info->aes_ciphertext[i] = ch;    
    }
    
} /* aes_ecb_enc_ast2100 */

void aes_dec_ast2100(aes_context *ctx, _AES_CRYPTO *aes_crypto_info, _CRYPTO_HW *crypto_hwinfo)
{
    ULONG i, ulTemp, ulCommand;
    UCHAR ch;
    ULONG srcbase, dstbase;
    	
    srcbase = ((ULONG) crypto_hwinfo->pjsrcvirt + 7) & 0xFFFFFFF8;
    dstbase = ((ULONG) crypto_hwinfo->pjdstvirt + 7) & 0xFFFFFFF8;

    ulCommand = CRYPTO_ENABLE_RW | CRYPTO_ENABLE_CONTEXT_LOAD | CRYPTO_ENABLE_CONTEXT_SAVE | \
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
    WriteDD_SOC(IO_ADDRESS(HAC_REG_BASE) + REG_CRYPTO_LEN_OFFSET, aes_crypto_info->aes_msglength);	

    /* Set source */
    for (i=0; i< aes_crypto_info->aes_msglength; i++)
    {
    	ch = aes_crypto_info->aes_ciphertext[i];
    	*(UCHAR *) (srcbase + i) = ch;
    }

    /* fire cmd */
    WriteDD_SOC(IO_ADDRESS(HAC_REG_BASE) + REG_CRYPTO_CMD_BASE_OFFSET, ulCommand);	
    do {
        ReadDD_SOC(IO_ADDRESS(HAC_REG_BASE) + REG_CRYPTO_STATUS_OFFSET,ulTemp);
    } while (ulTemp & CRYPTO_BUSY);	

    /* Output */
    for (i=0; i< aes_crypto_info->aes_msglength; i++)
    {
    	ch = *(UCHAR *) (dstbase + i);
        aes_crypto_info->aes_plaintext[i] = ch;    
    }

} /* aes_ecb_dec_ast2100 */

/* RC4 */
void rc4_key_ast2100(_RC4_KEY *rc4_key_info, _CRYPTO_HW *crypto_hwinfo)
{
    struct rc4_state s;
    ULONG i, ulTemp;
    ULONG contextbase;

    contextbase = ((ULONG) crypto_hwinfo->pjcontextvirt + 7) & 0xFFFFFFF8;
	
    rc4_setup( &s, rc4_key_info->rc4_key, rc4_key_info->keylength );

    /* Set Context */
    /* Set i, j */
    *(ULONG *) (contextbase + 8) = 0x0001;
       
    /* Set Expansion Key */   
    for (i=0; i<(256/4); i++)
    {
       ulTemp = (s.m[i * 4] & 0xFF) + ((s.m[i * 4 + 1] & 0xFF) << 8) + ((s.m[i * 4 + 2] & 0xFF) << 16) + ((s.m[i * 4+ 3] & 0xFF) << 24);    	
       *(ULONG *) (contextbase + i*4 + 16) = ulTemp; 
    }
	
} /* rc4_key_ast2100 */

	
void rc4_crypt_ast2100(_RC4_CRYPTO *rc4_crypto_info, _CRYPTO_HW *crypto_hwinfo)
{
    ULONG i, ulTemp, ulCommand;
    UCHAR ch;
    ULONG srcbase, dstbase;
    	
    srcbase = ((ULONG) crypto_hwinfo->pjsrcvirt + 7) & 0xFFFFFFF8;
    dstbase = ((ULONG) crypto_hwinfo->pjdstvirt + 7) & 0xFFFFFFF8;
    
    ulCommand = CRYPTO_ENABLE_RW | CRYPTO_ENABLE_CONTEXT_LOAD | CRYPTO_ENABLE_CONTEXT_SAVE | \
                CRYPTO_RC4 | CRYPTO_SYNC_MODE_ASYNC;

    /* Init HW */
    WriteDD_SOC(IO_ADDRESS(HAC_REG_BASE) + REG_CRYPTO_LEN_OFFSET, rc4_crypto_info->rc4_msglength);	
          	
    /* Set source */
    for (i=0; i< rc4_crypto_info->rc4_msglength; i++)
    {
    	ch = rc4_crypto_info->rc4_plaintext[i];
    	*(UCHAR *) (srcbase + i) = ch;
    }
     
    /* fire cmd */
    WriteDD_SOC(IO_ADDRESS(HAC_REG_BASE) + REG_CRYPTO_CMD_BASE_OFFSET, ulCommand);	
    do {
        ReadDD_SOC(IO_ADDRESS(HAC_REG_BASE) + REG_CRYPTO_STATUS_OFFSET,ulTemp);
    } while (ulTemp & CRYPTO_BUSY);	

    /* Output */
    for (i=0; i< rc4_crypto_info->rc4_msglength; i++)
    {
    	ch = *(UCHAR *) (dstbase + i);
        rc4_crypto_info->rc4_ciphertext[i] = ch;    
    }
  
} /* rc4_crypt_ast2100 */
