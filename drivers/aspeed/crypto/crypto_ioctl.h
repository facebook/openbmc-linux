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

typedef struct {

    ULONG  keylength;	
    UCHAR  aes_key[32];

} _AES_KEY;

typedef struct {
	
    UCHAR aes_iv[16];

} _AES_IV;
	
typedef struct {
	
    ULONG aes_mode;	
    ULONG aes_msglength;	
    UCHAR aes_plaintext[MAX_AESTEXTLENGTH];
    UCHAR aes_ciphertext[MAX_AESTEXTLENGTH];
    	
} _AES_CRYPTO;

typedef struct {

    ULONG  keylength;	
    UCHAR  rc4_key[MAX_RC4KEYLENGTH];

} _RC4_KEY;

typedef struct {
	
    ULONG rc4_msglength;	
    UCHAR rc4_plaintext[MAX_AESTEXTLENGTH];
    UCHAR rc4_ciphertext[MAX_AESTEXTLENGTH];
    	
} _RC4_CRYPTO;
