DEVMHIDIR=	${SRCTOP}/sys/contrib/dev/mhi
LKPISRCDIR=	${SRCTOP}/sys/compat/linuxkpi/common/src

.PATH:	${DEVMHIDIR}/host ${SRCTOP}/sys/contrib/dev/qcom ${LKPISRCDIR}

KMOD=	linuxkpi_mhi

# MHI host core, from Linux.
SRCS=	boot.c init.c main.c pm.c
# The MHI bus on FreeBSD.
SRCS+=	mhi_freebsd.c
# QRTR over MHI and the QMI client interface; QMI encoding from Linux.
SRCS+=	linux_qrtr.c qmi_encdec.c

SRCS+=	${LINUXKPI_GENSRCS}
SRCS+=	opt_wlan.h

CFLAGS+=	-I${DEVMHIDIR}/host
CFLAGS+=	-I${LKPISRCDIR}
CFLAGS+=	${LINUXKPI_INCLUDES}
CFLAGS+=	-DKBUILD_MODNAME='"mhi"'
CFLAGS+=	-DLINUXKPI_VERSION=70000
CWARNFLAGS.clang+=	-Wno-pointer-arith -Wno-cast-qual

EXPORT_SYMS=	YES

.include <bsd.kmod.mk>
