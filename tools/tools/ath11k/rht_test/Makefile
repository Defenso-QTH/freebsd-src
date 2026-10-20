SYSDIR?=	${SRCTOP}/sys

KMOD=	rht_test
SRCS=	rht_test.c bus_if.h device_if.h pci_if.h vnode_if.h opt_wlan.h

CFLAGS+=	-I${SYSDIR}/compat/linuxkpi/common/include
CFLAGS+=	-I${SYSDIR}/compat/linuxkpi/dummy/include
CFLAGS+=	-include ${SYSDIR}/compat/linuxkpi/common/include/linux/kconfig.h

.include <bsd.kmod.mk>
