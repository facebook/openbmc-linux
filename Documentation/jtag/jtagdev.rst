.. SPDX-License-Identifier: GPL-2.0

==================
JTAG userspace API
==================
JTAG master devices can be accessed through a character misc-device.

Each JTAG master interface can be accessed by using /dev/jtagN.

JTAG system calls set:
 * SIR (Scan Instruction Register, IEEE 1149.1 Instruction Register scan);
 * SDR (Scan Data Register, IEEE 1149.1 Data Register scan);
 * RUNTEST (Forces the IEEE 1149.1 bus to a run state for a specified number of clocks.

open(), close()
---------------
Open/Close  device:
::

	jtag_fd = open("/dev/jtag0", O_RDWR);
	close(jtag_fd);

ioctl()
-------
All access operations to JTAG devices are performed through ioctl interface.
The IOCTL interface supports these requests:
::

	JTAG_SIOCSTATE - Force JTAG state machine to go into a TAPC state
	JTAG_SIOCFREQ - Set JTAG TCK frequency
	JTAG_GIOCFREQ - Get JTAG TCK frequency
	JTAG_IOCXFER - send/receive JTAG data Xfer
	JTAG_GIOCSTATUS - get current JTAG TAP state
	JTAG_SIOCMODE - set JTAG mode flags.
	JTAG_IOCBITBANG - JTAG bitbang low level control.

JTAG_SIOCFREQ
~~~~~~~~~~~~~
Set JTAG clock speed:
::

	unsigned int jtag_fd;
	ioctl(jtag_fd, JTAG_SIOCFREQ, &frq);

JTAG_GIOCFREQ
~~~~~~~~~~~~~
Get JTAG clock speed:
::

	unsigned int jtag_fd;
	ioctl(jtag_fd, JTAG_GIOCFREQ, &frq);

JTAG_SIOCSTATE
~~~~~~~~~~~~~~
Force JTAG state machine to go into a TAPC state
::

	struct jtag_end_tap_state {
		__u8	reset;
		__u8	endstate;
		__u8	tck;
	};

reset: one of below options
::

	JTAG_NO_RESET - go through selected endstate from current state
	JTAG_FORCE_RESET - go through TEST_LOGIC/RESET state before selected endstate

endstate: any state listed in jtag_endstate enum
::

	enum jtag_endstate {
		JTAG_STATE_TLRESET,
		JTAG_STATE_IDLE,
		JTAG_STATE_SELECTDR,
		JTAG_STATE_CAPTUREDR,
		JTAG_STATE_SHIFTDR,
		JTAG_STATE_EXIT1DR,
		JTAG_STATE_PAUSEDR,
		JTAG_STATE_EXIT2DR,
		JTAG_STATE_UPDATEDR,
		JTAG_STATE_SELECTIR,
		JTAG_STATE_CAPTUREIR,
		JTAG_STATE_SHIFTIR,
		JTAG_STATE_EXIT1IR,
		JTAG_STATE_PAUSEIR,
		JTAG_STATE_EXIT2IR,
		JTAG_STATE_UPDATEIR
	};

tck: clock counter

Example:
::

	struct jtag_end_tap_state end_state;

	end_state.endstate = JTAG_STATE_IDLE;
	end_state.reset = 0;
	end_state.tck = data_p->tck;
	usleep(25 * 1000);
	ioctl(jtag_fd, JTAG_SIOCSTATE, &end_state);

JTAG_GIOCSTATUS
~~~~~~~~~~~~~~~
Get JTAG TAPC current machine state
::

	unsigned int jtag_fd;
	jtag_endstate endstate;
	ioctl(jtag_fd, JTAG_GIOCSTATUS, &endstate);

JTAG_IOCXFER
~~~~~~~~~~~~
Send SDR/SIR transaction
::

	struct jtag_xfer {
		__u8	type;
		__u8	direction;
		__u8	endstate;
		__u8	padding;
		__u32	length;
		__u64	tdio;
	};

type: transfer type - JTAG_SIR_XFER/JTAG_SDR_XFER

direction: xfer direction - JTAG_READ_XFER/JTAG_WRITE_XFER/JTAG_READ_WRITE_XFER

length: xfer data length in bits

tdio : xfer data array

endstate: end state after transaction finish any of jtag_endstate enum

Example:
::

	struct jtag_xfer xfer;
	static char buf[64];
	static unsigned int buf_len = 0;
	[...]
	xfer.type = JTAG_SDR_XFER;
	xfer.tdio = (__u64)buf;
	xfer.length = buf_len;
	xfer.endstate = JTAG_STATE_IDLE;

	if (is_read)
		xfer.direction = JTAG_READ_XFER;
	else if (is_write)
		xfer.direction = JTAG_WRITE_XFER;
	else
		xfer.direction = JTAG_READ_WRITE_XFER;

	ioctl(jtag_fd, JTAG_IOCXFER, &xfer);

JTAG_SIOCMODE
~~~~~~~~~~~~~
If hardware driver can support different running modes you can change it.

Example:
::

	struct jtag_mode mode;
	mode.feature = JTAG_XFER_MODE;
	mode.mode = JTAG_XFER_HW_MODE;
	ioctl(jtag_fd, JTAG_SIOCMODE, &mode);

JTAG_IOCBITBANG
~~~~~~~~~~~~~~~
JTAG Bitbang low level operation.

Example:
::

	struct tck_bitbang bitbang
	bitbang.tms = 1;
	bitbang.tdi = 0;
	ioctl(jtag_fd, JTAG_IOCBITBANG, &bitbang);
	tdo = bitbang.tdo;


THANKS TO
---------
Contributors to Linux-JTAG discussions include (in alphabetical order,
by last name):

- Ernesto Corona
- Jiri Pirko
- Oleksandr Shamray
- Steven Filary
- Vadim Pasternak
