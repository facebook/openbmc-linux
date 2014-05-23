#define ReadDD_SOC(addr, data)		\
{					\
 data = *(volatile ULONG *) (addr);		\
}

#define WriteDD_SOC(addr, data)		\
{					\
 *(ULONG *) (addr) = data;		\
}
