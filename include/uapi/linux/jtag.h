#ifndef __UAPI_LINUX_JTAG_H
#define __UAPI_LINUX_JTAG_H

#include <linux/types.h>
/*
 * JTAG_XFER_HW_MODE: JTAG hardware mode. Used to set HW drived or bitbang
 * mode. This is bitmask param of ioctl JTAG_SIOCMODE command
 */
#define  JTAG_XFER_HW_MODE 1

/**
 * enum jtag_endstate:
 *
 * @JTAG_STATE_IDLE: JTAG state machine IDLE state
 * @JTAG_STATE_PAUSEIR: JTAG state machine PAUSE_IR state
 * @JTAG_STATE_PAUSEDR: JTAG state machine PAUSE_DR state
 */
enum jtag_endstate {
    JTAG_STATE_IDLE,
    JTAG_STATE_PAUSEIR,
    JTAG_STATE_PAUSEDR,
};

/**
 * enum jtag_xfer_type:
 *
 * @JTAG_SIR_XFER: SIR transfer
 * @JTAG_SDR_XFER: SDR transfer
 */
enum jtag_xfer_type {
    JTAG_SIR_XFER,
    JTAG_SDR_XFER,
};

/**
 * enum jtag_xfer_direction:
 *
 * @JTAG_READ_XFER: read transfer
 * @JTAG_WRITE_XFER: write transfer
 */
enum jtag_xfer_direction {
    JTAG_READ_XFER,
    JTAG_WRITE_XFER,
};

/**
 * struct jtag_run_test_idle - forces JTAG state machine to
 * RUN_TEST/IDLE state
 *
 * @reset: 0 - run IDLE/PAUSE from current state
 *         1 - go through TEST_LOGIC/RESET state before  IDLE/PAUSE
 * @end: completion flag
 * @tck: clock counter
 *
 * Structure represents interface to JTAG device for jtag idle
 * execution.
 */
struct jtag_run_test_idle {
    __u8    reset;
    __u8    endstate;
    __u8    tck;
};

/**
 * struct jtag_xfer - jtag xfer:
 *
 * @type: transfer type
 * @direction: xfer direction
 * @length: xfer bits len
 * @tdio : xfer data array
 * @endir: xfer end state
 *
 * Structure represents interface to JTAG device for jtag sdr xfer
 * execution.
 */
struct jtag_xfer {
    __u8    type;
    __u8    direction;
    __u8    endstate;
    __u32   length;
    __u32   *tdio;
};

/**
 * struct run_cycle_param - parameters for jtag running cycle
 * @tdi: TDI bit value to write
 * @tms: TMS bit value to write
 * @tck: TCK bit value to write
 * @tdo: TDO bit value to read
 *
 * Structure represents interface to JTAG device set tdi tms tck,
 * and return tdo in one transaction, thus saving multiple system calls
 */
struct run_cycle_param {
    unsigned char     tdi;        // TDI bit value to write
    unsigned char     tms;        // TMS bit value to write
    unsigned char     tck;        // TCK bit value to write
    unsigned char     tdo;        // TDO bit value to read
};

/* ioctl interface */
#define __JTAG_IOCTL_MAGIC  0xb2

#define JTAG_IOCRUNTEST _IOW(__JTAG_IOCTL_MAGIC, 0,\
                 struct jtag_run_test_idle)
#define JTAG_SIOCFREQ   _IOW(__JTAG_IOCTL_MAGIC, 1, unsigned int)
#define JTAG_GIOCFREQ   _IOR(__JTAG_IOCTL_MAGIC, 2, unsigned int)
#define JTAG_IOCXFER    _IOWR(__JTAG_IOCTL_MAGIC, 3, struct jtag_xfer)
#define JTAG_GIOCSTATUS _IOWR(__JTAG_IOCTL_MAGIC, 4, enum jtag_endstate)
#define JTAG_SIOCMODE   _IOW(__JTAG_IOCTL_MAGIC, 5, unsigned int)
#define JTAG_RUN_CYCLE  _IOR(__JTAG_IOCTL_MAGIC, 11, struct run_cycle_param)

#define JTAG_FIRST_MINOR 0
#define JTAG_MAX_DEVICES 32

#endif /* __UAPI_LINUX_JTAG_H */
