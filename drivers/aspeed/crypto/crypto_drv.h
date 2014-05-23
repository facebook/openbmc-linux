#include "typedef.h"
#include "aes.h"
#include "rc4.h"
#include "crypto_hw.h"
#include "crypto_io.h"
#include "crypto_ioctl.h"

/* Reg. Definition */
typedef enum _CHIP_ID {
    AST2000,
    AST2100,
} CHIP_ID;
