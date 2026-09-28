.PATH: ${SRCTOP}/sys/security/mac_netbylabel

KMOD=	mac_netbylabel
SRCS=	mac_netbylabel.c
SRCS+=	opt_global.h opt_mac.h vnode_if.h

.if defined(KERNBUILDDIR)
MKDEP=	-include ${KERNBUILDDIR}/opt_global.h
.else
CFLAGS+= -include opt_global.h
MKDEP=	-include opt_global.h
opt_mac.h:
	echo "#define MAC_DEBUG 1" >> ${.TARGET}
opt_global.h:
	echo "#define MAC 1" > ${.TARGET}
.endif

.include <bsd.kmod.mk>
