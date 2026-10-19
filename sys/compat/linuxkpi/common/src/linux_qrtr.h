/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Quentin Thebault
 */

#ifndef	_LINUXKPI_SRC_LINUX_QRTR_H_
#define	_LINUXKPI_SRC_LINUX_QRTR_H_

/* Set up and tear down QRTR, from the linuxkpi_mhi module handler. */
int	lkpi_qrtr_init(void);
void	lkpi_qrtr_uninit(void);
bool	lkpi_qrtr_busy(void);

#endif	/* _LINUXKPI_SRC_LINUX_QRTR_H_ */
