.. SPDX-License-Identifier: GPL-2.0

=====================================
Linux I2C slave message queue backend
=====================================

:Author: Haiyue Wang <haiyue.wang@linux.intel.com>

Some protocols over I2C/SMBus are designed for bi-directional transferring
messages by using I2C Master Write protocol. This requires that both sides
of the communication have slave addresses.

Like MCTP (Management Component Transport Protocol) and IPMB (Intelligent
Platform Management Bus), they both require that the userspace can receive
messages from i2c dirvers under slave mode.

This I2C slave mqueue (message queue) backend is used to receive and queue
messages from the remote i2c intelligent device; and it will add the target
slave address (with R/W# bit is always 0) into the message at the first byte,
so that userspace can use this byte to dispatch the messages into different
handling modules. Also, like IPMB, the address byte is in its message format,
it needs it to do checksum.

For messages are time related, so this backend will flush the oldest message
to queue the newest one.

Link
----
`Intelligent Platform Management Bus
Communications Protocol Specification
<https://www.intel.com/content/dam/www/public/us/en/documents/product-briefs/ipmp-spec-v1.0.pdf>`_

`Management Component Transport Protocol (MCTP)
SMBus/I2C Transport Binding Specification
<https://www.dmtf.org/sites/default/files/standards/documents/DSP0237_1.1.0.pdf>`_

How to use
----------
For example, the I2C5 bus has slave address 0x10, the below command will create
the related message queue interface:

    echo slave-mqueue 0x1010 > /sys/bus/i2c/devices/i2c-5/new_device

Then you can dump the messages like this:

    hexdump -C /sys/bus/i2c/devices/5-1010/slave-mqueue

Code Example
------------
*Note: call 'lseek' before 'read', this is a requirement from kernfs' design.*

::

  #include <sys/types.h>
  #include <sys/stat.h>
  #include <unistd.h>
  #include <poll.h>
  #include <time.h>
  #include <fcntl.h>
  #include <stdio.h>

  int main(int argc, char *argv[])
  {
          int i, r;
          struct pollfd pfd;
          struct timespec ts;
          unsigned char data[256];

          pfd.fd = open(argv[1], O_RDONLY | O_NONBLOCK);
          if (pfd.fd < 0)
                  return -1;

          pfd.events = POLLPRI;

          while (1) {
                  r = poll(&pfd, 1, 5000);

                  if (r < 0)
                          break;

                  if (r == 0 || !(pfd.revents & POLLPRI))
                          continue;

                  lseek(pfd.fd, 0, SEEK_SET);
                  r = read(pfd.fd, data, sizeof(data));
                  if (r <= 0)
                          continue;

                  clock_gettime(CLOCK_MONOTONIC, &ts);
                  printf("[%ld.%.9ld] :", ts.tv_sec, ts.tv_nsec);
                  for (i = 0; i < r; i++)
                          printf(" %02x", data[i]);
                  printf("\n");
          }

          close(pfd.fd);

          return 0;
  }

Result
------
*./a.out "/sys/bus/i2c/devices/5-1010/slave-mqueue"*

::

  [10183.232500449] : 20 18 c8 2c 78 01 5b
  [10183.479358348] : 20 18 c8 2c 78 01 5b
  [10183.726556812] : 20 18 c8 2c 78 01 5b
  [10183.972605863] : 20 18 c8 2c 78 01 5b
  [10184.220124772] : 20 18 c8 2c 78 01 5b
  [10184.467764166] : 20 18 c8 2c 78 01 5b
  [10193.233421784] : 20 18 c8 2c 7c 01 57
  [10193.480273460] : 20 18 c8 2c 7c 01 57
  [10193.726788733] : 20 18 c8 2c 7c 01 57
  [10193.972781945] : 20 18 c8 2c 7c 01 57
  [10194.220487360] : 20 18 c8 2c 7c 01 57
  [10194.468089259] : 20 18 c8 2c 7c 01 57
  [10203.233433099] : 20 18 c8 2c 80 01 53
  [10203.481058715] : 20 18 c8 2c 80 01 53
  [10203.727610472] : 20 18 c8 2c 80 01 53
  [10203.974044856] : 20 18 c8 2c 80 01 53
  [10204.220734634] : 20 18 c8 2c 80 01 53
  [10204.468461664] : 20 18 c8 2c 80 01 53

