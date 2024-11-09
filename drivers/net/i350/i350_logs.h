/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2010-2014 Intel Corporation
 */

#ifndef _I350_LOGS_H_
#define _I350_LOGS_H_

#include <rte_log.h>

extern int i350_logtype_init;
#define RTE_LOGTYPE_I350_INIT i350_logtype_init

#define PMD_INIT_LOG(level, ...) \
	RTE_LOG_LINE_PREFIX(level, I350_INIT, "%s(): ", __func__, __VA_ARGS__)

#define PMD_INIT_FUNC_TRACE() PMD_INIT_LOG(DEBUG, " >>")

#ifdef RTE_ETHDEV_DEBUG_RX
extern int i350_logtype_rx;
#define RTE_LOGTYPE_I350_RX i350_logtype_rx
#define PMD_RX_LOG(level, ...) \
	RTE_LOG_LINE_PREFIX(level, I350_RX, "%s(): ", __func__, __VA_ARGS__)
#else
#define PMD_RX_LOG(...) do { } while (0)
#endif

#ifdef RTE_ETHDEV_DEBUG_TX
extern int i350_logtype_tx;
#define RTE_LOGTYPE_I350_TX i350_logtype_tx
#define PMD_TX_LOG(level, ...) \
	RTE_LOG_LINE_PREFIX(level, I350_TX, "%s(): ", __func__, __VA_ARGS__)
#else
#define PMD_TX_LOG(...) do { } while (0)
#endif

extern int i350_logtype_driver;
#define RTE_LOGTYPE_I350_DRIVER i350_logtype_driver
#define PMD_DRV_LOG(level, ...) \
	RTE_LOG_LINE_PREFIX(level, I350_DRIVER, "%s(): ", __func__, __VA_ARGS__)

/* log init function shared by e1000 and igb drivers */
void i350_i350_init_log(void);

#endif /* _E1000_LOGS_H_ */
