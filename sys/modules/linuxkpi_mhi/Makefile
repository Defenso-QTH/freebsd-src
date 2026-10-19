LKPISRCDIR=	${SRCTOP}/sys/compat/linuxkpi/common/src

.PATH:	${LKPISRCDIR}

KMOD=	linuxkpi_mhi

# QRTR over MHI and the QMI client interface.
SRCS=	linux_qrtr.c

SRCS+=	${LINUXKPI_GENSRCS}
SRCS+=	opt_wlan.h

CFLAGS+=	-I${LKPISRCDIR}
CFLAGS+=	${LINUXKPI_INCLUDES}
CFLAGS+=	-DKBUILD_MODNAME='"mhi"'
CFLAGS+=	-DLINUXKPI_VERSION=70000
CWARNFLAGS.clang+=	-Wno-pointer-arith -Wno-cast-qual

EXPORT_SYMS=	YES

.include <bsd.kmod.mk>
