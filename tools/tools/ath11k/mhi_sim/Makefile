# Test-only: an MHI device model driving the MHI host core, QRTR and QMI.
# kldload linuxkpi_mhi, then this, then sysctl debug.mhisim.run=1.

SRCTOP?=	${.CURDIR:H:H:H:H}

.PATH:	${.CURDIR}

KMOD=	mhi_sim
SRCS=	mhi_sim.c
SRCS+=	${LINUXKPI_GENSRCS}
SRCS+=	opt_wlan.h

CFLAGS+=	${LINUXKPI_INCLUDES}
CFLAGS+=	-I${SRCTOP}/sys
CFLAGS+=	-DLINUXKPI_VERSION=70000

.include <bsd.kmod.mk>
