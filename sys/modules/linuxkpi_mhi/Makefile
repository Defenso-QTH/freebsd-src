LKPISRCDIR=	${SRCTOP}/sys/compat/linuxkpi/common/src

.PATH:	${LKPISRCDIR}

KMOD=	linuxkpi_mhi

# MHI host core, from OpenBSD's qwx(4).
SRCS=	linux_mhi.c
# Interrupts, sysctls and the module handler.
SRCS+=	linux_mhi_bus.c
# QRTR over MHI and the QMI client interface.
SRCS+=	linux_qmi_encdec.c linux_qrtr.c

SRCS+=	${LINUXKPI_GENSRCS}
SRCS+=	opt_wlan.h

CFLAGS+=	-I${LKPISRCDIR}
CFLAGS+=	${LINUXKPI_INCLUDES}
CFLAGS+=	-DKBUILD_MODNAME='"mhi"'
CFLAGS+=	-DLINUXKPI_VERSION=70000
CWARNFLAGS.clang+=	-Wno-pointer-arith -Wno-cast-qual

EXPORT_SYMS=	YES

.include <bsd.kmod.mk>
