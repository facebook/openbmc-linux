#include <linux/socket.h>
#include <linux/tcp.h>

#define  IOCTL_IO_READ			0x1103
#define  IOCTL_IO_WRITE			0x1104
#define  IOCTL_REAL_IO_READ		0x1106
#define  IOCTL_REAL_IO_WRITE		0x1107
#define  IOCTL_BIT_STREAM_BASE		0x1108
#define  IOCTL_TX_BIT_STREAM		0x1109
#define  IOCTL_GET_SOCKET		0x1110
#define  IOCTL_TRIGGER			0x1111
#define  IOCTL_USB20_MOUSE_PACKET	0x1112
#define  IOCTL_MEMORY_OVER_READ		0x1116
#define  IOCTL_MEMORY_OVER_WRITE	0x1117
#define  IOCTL_IO_SCRATCH_SD		0x1118
#define  IOCTL_IO_SCRATCH_USBHOST	0x1119
#define  IOCTL_REMAP      0x1105
#define  RELOCATE_OFFSET  0x380

typedef struct _IO_ACCESS_DATA {
    unsigned char Type;
    unsigned long Address;
    unsigned long Data;
    unsigned long Value;
    int      kernel_socket;
//    struct sockaddr_in address_svr;
} IO_ACCESS_DATA, *PIO_ACCESS_DATA;

