#define    AC_TIMING        0x77743335
#define    ALL_CLEAR        0xFFFFFFFF
#define    MASTER_ENABLE    0x01
#define    SLAVE_ENABLE     0x02
#define    LOOP_COUNT       0x10000


#define    I2C_BASE                         0x1e78A000
#define    I2C_FUNCTION_CONTROL_REGISTER    0x00
#define    I2C_AC_TIMING_REGISTER_1         0x04
#define    I2C_AC_TIMING_REGISTER_2         0x08
#define    I2C_INTERRUPT_CONTROL_REGISTER   0x0C
#define    I2C_INTERRUPT_STATUS_REGISTER    0x10
#define    I2C_COMMAND_REGISTER             0x14
#define    I2C_BYTE_BUFFER_REGISTER         0x20


#define    MASTER_START_COMMAND    (1 << 0)
#define    MASTER_TX_COMMAND       (1 << 1)
#define    MASTER_RX_COMMAND       (1 << 3)
#define    RX_COMMAND_LIST         (1 << 4)
#define    MASTER_STOP_COMMAND     (1 << 5)

#define    TX_ACK       (1 << 0)
#define    TX_NACK      (1 << 1)
#define    RX_DONE      (1 << 2)
#define    STOP_DONE    (1 << 4)

