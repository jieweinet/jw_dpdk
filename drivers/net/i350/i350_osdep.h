/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2001 - 2015 Intel Corporation
 */
/*$FreeBSD$*/

#ifndef _I350_OSDEP_H_
#define _I350_OSDEP_H_

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#include <rte_common.h>
#include <rte_cycles.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_byteorder.h>
#include <rte_io.h>

#include "i350_logs.h"

#define DELAY(x) rte_delay_us_sleep(x)
#define usec_delay(x) DELAY(x)
#define usec_delay_irq(x) DELAY(x)
#define msec_delay(x) DELAY(1000*(x))
#define msec_delay_irq(x) DELAY(1000*(x))

#define DEBUGFUNC(F)            DEBUGOUT(F "\n");
#define DEBUGOUT(S, ...)        RTE_LOG(DEBUG, I350_DRIVER, "%s(): " S, __func__, ## __VA_ARGS__)
#define DEBUGOUT1(S, args...)   DEBUGOUT(S, ##args)
#define DEBUGOUT2(S, args...)   DEBUGOUT(S, ##args)
#define DEBUGOUT3(S, args...)   DEBUGOUT(S, ##args)
#define DEBUGOUT6(S, args...)   DEBUGOUT(S, ##args)
#define DEBUGOUT7(S, args...)   DEBUGOUT(S, ##args)

#ifndef UNREFERENCED_PARAMETER
#define UNREFERENCED_PARAMETER(_p)
#endif
#define UNREFERENCED_1PARAMETER(_p)
#define UNREFERENCED_2PARAMETER(_p, _q)
#define UNREFERENCED_3PARAMETER(_p, _q, _r)
#define UNREFERENCED_4PARAMETER(_p, _q, _r, _s)

#define FALSE			0
#define TRUE			1

#define	CMD_MEM_WRT_INVALIDATE	0x0010  /* BIT_4 */

/* Mutex used in the shared code */
#define I350_MUTEX                     uintptr_t
#define I350_MUTEX_INIT(mutex)         (*(mutex) = 0)
#define I350_MUTEX_LOCK(mutex)         (*(mutex) = 1)
#define I350_MUTEX_UNLOCK(mutex)       (*(mutex) = 0)

typedef uint64_t	u64;
typedef uint32_t	u32;
typedef uint16_t	u16;
typedef uint8_t		u8;
typedef int64_t		s64;
typedef int32_t		s32;
typedef int16_t		s16;
typedef int8_t		s8;

#define __le16		u16
#define __le32		u32
#define __le64		u64

#define I350_WRITE_FLUSH(a) I350_READ_REG(a, I350_STATUS)

#define I350_PCI_REG(reg)	rte_read32(reg)

#define I350_PCI_REG16(reg)	rte_read16(reg)

#define I350_PCI_REG_WRITE(reg, value)			\
	rte_write32((rte_cpu_to_le_32(value)), reg)

#define I350_PCI_REG_WRITE_RELAXED(reg, value)		\
	rte_write32_relaxed((rte_cpu_to_le_32(value)), reg)

#define I350_PCI_REG_WRITE16(reg, value)		\
	rte_write16((rte_cpu_to_le_16(value)), reg)

#define I350_PCI_REG_ADDR(hw, reg) \
	((volatile uint32_t *)((char *)(hw)->hw_addr + (reg)))

#define I350_PCI_REG_ARRAY_ADDR(hw, reg, index) \
	I350_PCI_REG_ADDR((hw), (reg) + ((index) << 2))

#define I350_PCI_REG_FLASH_ADDR(hw, reg) \
	((volatile uint32_t *)((char *)(hw)->flash_address + (reg)))

static inline uint32_t i350_read_addr(volatile void *addr)
{
	return rte_le_to_cpu_32(I350_PCI_REG(addr));
}

static inline uint16_t i350_read_addr16(volatile void *addr)
{
	return rte_le_to_cpu_16(I350_PCI_REG16(addr));
}

/* Necessary defines */
#define I350_MRQC_ENABLE_MASK                  0x00000007
#define I350_MRQC_RSS_FIELD_IPV6_EX		0x00080000
#define I350_ALL_FULL_DUPLEX   ( \
        ADVERTISE_10_FULL | ADVERTISE_100_FULL | ADVERTISE_1000_FULL)

#define M88E1543_E_PHY_ID    0x01410EA0
#define ULP_SUPPORT

#define I350_RCTL_DTYP_MASK	0x00000C00 /* Descriptor type mask */
#define I350_MRQC_RSS_FIELD_IPV6_EX            0x00080000

/* Register READ/WRITE macros */

#define I350_READ_REG(hw, reg) \
	i350_read_addr(I350_PCI_REG_ADDR((hw), (reg)))

#define I350_WRITE_REG(hw, reg, value) \
	I350_PCI_REG_WRITE(I350_PCI_REG_ADDR((hw), (reg)), (value))

#define I350_READ_REG_ARRAY(hw, reg, index) \
	I350_PCI_REG(I350_PCI_REG_ARRAY_ADDR((hw), (reg), (index)))

#define I350_WRITE_REG_ARRAY(hw, reg, index, value) \
	I350_PCI_REG_WRITE(I350_PCI_REG_ARRAY_ADDR((hw), (reg), (index)), (value))

#define I350_READ_REG_ARRAY_DWORD I350_READ_REG_ARRAY
#define I350_WRITE_REG_ARRAY_DWORD I350_WRITE_REG_ARRAY

#define	I350_ACCESS_PANIC(x, hw, reg, value) \
	rte_panic("%s:%u\t" RTE_STR(x) "(%p, 0x%x, 0x%x)", \
		__FILE__, __LINE__, (hw), (reg), (unsigned int)(value))

/*
 * To be able to do IO write, we need to map IO BAR
 * (bar 2/4 depending on device).
 * Right now mapping multiple BARs is not supported by DPDK.
 * Fortunatelly we need it only for legacy hw support.
 */

#define I350_WRITE_REG_IO(hw, reg, value) \
	I350_WRITE_REG(hw, reg, value)

/*
 * Tested on I217/I218 chipset.
 */

#define I350_READ_FLASH_REG(hw, reg) \
	i350_read_addr(I350_PCI_REG_FLASH_ADDR((hw), (reg)))

#define I350_READ_FLASH_REG16(hw, reg)  \
	i350_read_addr16(I350_PCI_REG_FLASH_ADDR((hw), (reg)))

#define I350_WRITE_FLASH_REG(hw, reg, value)  \
	I350_PCI_REG_WRITE(I350_PCI_REG_FLASH_ADDR((hw), (reg)), (value))

#define I350_WRITE_FLASH_REG16(hw, reg, value) \
	I350_PCI_REG_WRITE16(I350_PCI_REG_FLASH_ADDR((hw), (reg)), (value))

#define STATIC static

#ifndef ETH_ADDR_LEN
#define ETH_ADDR_LEN                  6
#endif

#endif /* _I350_OSDEP_H_ */
