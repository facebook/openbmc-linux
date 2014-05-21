ifeq ($(CONFIG_ARCH_AST1510),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

ifeq ($(CONFIG_ARCH_AST2300),y)
   zreladdr-y	:= 0x40008000
params_phys-y	:= 0x40000100
initrd_phys-y	:= 0x40800000
endif

ifeq ($(CONFIG_ARCH_AST2400),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

ifeq ($(CONFIG_ARCH_AST2500),y)
   zreladdr-y   := 0x80008000
params_phys-y   := 0x80000100
initrd_phys-y   := 0x80800000
endif

ifeq ($(CONFIG_ARCH_AST3100),y)
   zreladdr-y   := 0x40008000
params_phys-y   := 0x40000100
initrd_phys-y   := 0x40800000
endif

ifeq ($(CONFIG_ARCH_AST1520),y)
   zreladdr-y   := 0x80008000
params_phys-y   := 0x80000100
initrd_phys-y   := 0x80800000
endif

ifeq ($(CONFIG_ARCH_AST3200),y)
   zreladdr-y   := 0x80008000
params_phys-y   := 0x80000100
initrd_phys-y   := 0x80800000
endif

