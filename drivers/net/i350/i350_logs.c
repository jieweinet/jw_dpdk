/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Intel Corporation
 */

#include <rte_ethdev.h>
#include "i350_logs.h"

RTE_LOG_REGISTER_SUFFIX(i350_logtype_init, init, NOTICE)
RTE_LOG_REGISTER_SUFFIX(i350_logtype_driver, driver, NOTICE)
#ifdef RTE_ETHDEV_DEBUG_RX
RTE_LOG_REGISTER_SUFFIX(i350_logtype_rx, rx, DEBUG)
#endif
#ifdef RTE_ETHDEV_DEBUG_TX
RTE_LOG_REGISTER_SUFFIX(i350_logtype_tx, tx, DEBUG)
#endif
