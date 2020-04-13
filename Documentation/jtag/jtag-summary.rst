.. SPDX-License-Identifier: GPL-2.0

====================================
Linux kernel JTAG support
====================================

Introduction to JTAG
====================

JTAG is an industry standard for verifying hardware. JTAG provides access to
many logic signals of a complex integrated circuit, including the device pins.

A JTAG interface is a special interface added to a chip.
Depending on the version of JTAG, two, four, or five pins are added.

The connector pins are:
 * TDI (Test Data In)
 * TDO (Test Data Out)
 * TCK (Test Clock)
 * TMS (Test Mode Select)
 * TRST (Test Reset) optional

JTAG interface is designed to have two parts - basic core driver and
hardware specific driver. The basic driver introduces a general interface
which is not dependent of specific hardware. It provides communication
between user space and hardware specific driver.
Each JTAG device is represented as a char device from (jtag0, jtag1, ...).
Access to a JTAG device is performed through IOCTL calls.

Call flow example:
::

	User: open  -> /dev/jatgX -> JTAG core driver -> JTAG hardware specific driver
	User: ioctl -> /dev/jtagX -> JTAG core driver -> JTAG hardware specific driver
	User: close -> /dev/jatgX -> JTAG core driver -> JTAG hardware specific driver


THANKS TO
---------
Contributors to Linux-JTAG discussions include (in alphabetical order,
by last name):

- Ernesto Corona
- Jiri Pirko
- Oleksandr Shamray
- Steven Filary
- Vadim Pasternak
