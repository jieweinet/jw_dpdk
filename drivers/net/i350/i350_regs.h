/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2015 Intel Corporation
 */
#ifndef _I350_REGS_H_
#define _I350_REGS_H_

#include "i350_ethdev.h"

#define I350_CTRL	0x00000  /* Device Control - RW */
#define I350_CTRL_DUP	0x00004  /* Device Control Duplicate (Shadow) - RW */
#define I350_STATUS	0x00008  /* Device Status - RO */
#define I350_EECD	0x00010  /* EEPROM/Flash Control - RW */
#define I350_EERD	0x00014  /* EEPROM Read - RW */
#define I350_CTRL_EXT	0x00018  /* Extended Device Control - RW */
#define I350_FLA	0x0001C  /* Flash Access - RW */
#define I350_MDIC	0x00020  /* MDI Control - RW */
#define I350_MDICNFG	0x00E04  /* MDI Config - RW */
#define I350_REGISTER_SET_SIZE		0x20000 /* CSR Size */
#define I350_EEPROM_INIT_CTRL_WORD_2	0x0F /* EEPROM Init Ctrl Word 2 */
#define I350_EEPROM_PCIE_CTRL_WORD_2	0x28 /* EEPROM PCIe Ctrl Word 2 */
#define I350_BARCTRL			0x5BBC /* BAR ctrl reg */
#define I350_BARCTRL_FLSIZE		0x0700 /* BAR ctrl Flsize */
#define I350_BARCTRL_CSRSIZE		0x2000 /* BAR ctrl CSR size */
#define I350_MPHY_ADDR_CTRL	0x0024 /* GbE MPHY Address Control */
#define I350_MPHY_DATA		0x0E10 /* GBE MPHY Data */
#define I350_MPHY_STAT		0x0E0C /* GBE MPHY Statistics */
#define I350_PPHY_CTRL		0x5b48 /* PCIe PHY Control */
#define I350_I350_BARCTRL		0x5BFC /* BAR ctrl reg */
#define I350_I350_DTXMXPKTSZ		0x355C /* Maximum sent packet size reg*/
#define I350_SCTL	0x00024  /* SerDes Control - RW */
#define I350_FCAL	0x00028  /* Flow Control Address Low - RW */
#define I350_FCAH	0x0002C  /* Flow Control Address High -RW */
#define I350_FEXT	0x0002C  /* Future Extended - RW */
#define I350_FEXTNVM	0x00028  /* Future Extended NVM - RW */
#define I350_FEXTNVM3	0x0003C  /* Future Extended NVM 3 - RW */
#define I350_FEXTNVM4	0x00024  /* Future Extended NVM 4 - RW */
#define I350_FEXTNVM5	0x00014  /* Future Extended NVM 5 - RW */
#define I350_FEXTNVM6	0x00010  /* Future Extended NVM 6 - RW */
#define I350_FEXTNVM7	0x000E4  /* Future Extended NVM 7 - RW */
#define I350_FEXTNVM8	0x5BB0  /* Future Extended NVM 8 - RW */
#define I350_FEXTNVM9	0x5BB4  /* Future Extended NVM 9 - RW */
#define I350_FEXTNVM11	0x5BBC  /* Future Extended NVM 11 - RW */
#define I350_FEXTNVM12	0x5BC0  /* Future Extended NVM 12 - RW */
#define I350_PCIEANACFG	0x00F18 /* PCIE Analog Config */
#define I350_DPGFR	0x00FAC	/* Dynamic Power Gate Force Control Register */
#define I350_FCT	0x00030  /* Flow Control Type - RW */
#define I350_CONNSW	0x00034  /* Copper/Fiber switch control - RW */
#define I350_VET	0x00038  /* VLAN Ether Type - RW */
#define I350_ICR	0x000C0  /* Interrupt Cause Read - R/clr */
#define I350_ITR	0x000C4  /* Interrupt Throttling Rate - RW */
#define I350_ICS	0x000C8  /* Interrupt Cause Set - WO */
#define I350_IMS	0x000D0  /* Interrupt Mask Set - RW */
#define I350_IMC	0x000D8  /* Interrupt Mask Clear - WO */
#define I350_IAM	0x000E0  /* Interrupt Acknowledge Auto Mask */
#define I350_IVAR	0x000E4  /* Interrupt Vector Allocation Register - RW */
#define I350_SVCR	0x000F0
#define I350_SVT	0x000F4
#define I350_LPIC	0x000FC  /* Low Power IDLE control */
#define I350_RCTL	0x00100  /* Rx Control - RW */
#define I350_FCTTV	0x00170  /* Flow Control Transmit Timer Value - RW */
#define I350_TXCW	0x00178  /* Tx Configuration Word - RW */
#define I350_RXCW	0x00180  /* Rx Configuration Word - RO */
#define I350_PBA_ECC	0x01100  /* PBA ECC Register */
#define I350_EICR	0x01580  /* Ext. Interrupt Cause Read - R/clr */
#define I350_EITR(_n)	(0x01680 + (0x4 * (_n)))
#define I350_EICS	0x01520  /* Ext. Interrupt Cause Set - W0 */
#define I350_EIMS	0x01524  /* Ext. Interrupt Mask Set/Read - RW */
#define I350_EIMC	0x01528  /* Ext. Interrupt Mask Clear - WO */
#define I350_EIAC	0x0152C  /* Ext. Interrupt Auto Clear - RW */
#define I350_EIAM	0x01530  /* Ext. Interrupt Ack Auto Clear Mask - RW */
#define I350_GPIE	0x01514  /* General Purpose Interrupt Enable - RW */
#define I350_IVAR0	0x01700  /* Interrupt Vector Allocation (array) - RW */
#define I350_IVAR_MISC	0x01740 /* IVAR for "other" causes - RW */
#define I350_TCTL	0x00400  /* Tx Control - RW */
#define I350_TCTL_EXT	0x00404  /* Extended Tx Control - RW */
#define I350_TIPG	0x00410  /* Tx Inter-packet gap -RW */
#define I350_TBT	0x00448  /* Tx Burst Timer - RW */
#define I350_AIT	0x00458  /* Adaptive Interframe Spacing Throttle - RW */
#define I350_LEDCTL	0x00E00  /* LED Control - RW */
#define I350_LEDMUX	0x08130  /* LED MUX Control */
#define I350_EXTCNF_CTRL	0x00F00  /* Extended Configuration Control */
#define I350_EXTCNF_SIZE	0x00F08  /* Extended Configuration Size */
#define I350_PHY_CTRL	0x00F10  /* PHY Control Register in CSR */
#define I350_POEMB	I350_PHY_CTRL /* PHY OEM Bits */
#define I350_PBA	0x01000  /* Packet Buffer Allocation - RW */
#define I350_PBS	0x01008  /* Packet Buffer Size */
#define I350_PBECCSTS	0x0100C  /* Packet Buffer ECC Status - RW */
#define I350_IOSFPC	0x00F28  /* TX corrupted data  */
#define I350_EEMNGCTL	0x01010  /* MNG EEprom Control */
#define I350_EEMNGCTL_I210	0x01010  /* i210 MNG EEprom Mode Control */
#define I350_EEARBC	0x01024  /* EEPROM Auto Read Bus Control */
#define I350_EEARBC_I210	0x12024 /* EEPROM Auto Read Bus Control */
#define I350_FLASHT	0x01028  /* FLASH Timer Register */
#define I350_EEWR	0x0102C  /* EEPROM Write Register - RW */
#define I350_FLSWCTL	0x01030  /* FLASH control register */
#define I350_FLSWDATA	0x01034  /* FLASH data register */
#define I350_FLSWCNT	0x01038  /* FLASH Access Counter */
#define I350_FLOP	0x0103C  /* FLASH Opcode Register */
#define I350_I2CCMD	0x01028  /* SFPI2C Command Register - RW */
#define I350_I2CPARAMS	0x0102C /* SFPI2C Parameters Register - RW */
#define I350_I2CBB_EN	0x00000100  /* I2C - Bit Bang Enable */
#define I350_I2C_CLK_OUT	0x00000200  /* I2C- Clock */
#define I350_I2C_DATA_OUT	0x00000400  /* I2C- Data Out */
#define I350_I2C_DATA_OE_N	0x00000800  /* I2C- Data Output Enable */
#define I350_I2C_DATA_IN	0x00001000  /* I2C- Data In */
#define I350_I2C_CLK_OE_N	0x00002000  /* I2C- Clock Output Enable */
#define I350_I2C_CLK_IN	0x00004000  /* I2C- Clock In */
#define I350_I2C_CLK_STRETCH_DIS	0x00008000 /* I2C- Dis Clk Stretching */
#define I350_WDSTP	0x01040  /* Watchdog Setup - RW */
#define I350_SWDSTS	0x01044  /* SW Device Status - RW */
#define I350_FRTIMER	0x01048  /* Free Running Timer - RW */
#define I350_TCPTIMER	0x0104C  /* TCP Timer - RW */
#define I350_VPDDIAG	0x01060  /* VPD Diagnostic - RO */
#define I350_ICR_V2	0x01500  /* Intr Cause - new location - RC */
#define I350_ICS_V2	0x01504  /* Intr Cause Set - new location - WO */
#define I350_IMS_V2	0x01508  /* Intr Mask Set/Read - new location - RW */
#define I350_IMC_V2	0x0150C  /* Intr Mask Clear - new location - WO */
#define I350_IAM_V2	0x01510  /* Intr Ack Auto Mask - new location - RW */
#define I350_ERT	0x02008  /* Early Rx Threshold - RW */
#define I350_FCRTL	0x02160  /* Flow Control Receive Threshold Low - RW */
#define I350_FCRTH	0x02168  /* Flow Control Receive Threshold High - RW */
#define I350_PSRCTL	0x02170  /* Packet Split Receive Control - RW */
#define I350_RDFH	0x02410  /* Rx Data FIFO Head - RW */
#define I350_RDFT	0x02418  /* Rx Data FIFO Tail - RW */
#define I350_RDFHS	0x02420  /* Rx Data FIFO Head Saved - RW */
#define I350_RDFTS	0x02428  /* Rx Data FIFO Tail Saved - RW */
#define I350_RDFPC	0x02430  /* Rx Data FIFO Packet Count - RW */
#define I350_PBRTH	0x02458  /* PB Rx Arbitration Threshold - RW */
#define I350_FCRTV	0x02460  /* Flow Control Refresh Timer Value - RW */
/* Split and Replication Rx Control - RW */
#define I350_RDPUMB	0x025CC  /* DMA Rx Descriptor uC Mailbox - RW */
#define I350_RDPUAD	0x025D0  /* DMA Rx Descriptor uC Addr Command - RW */
#define I350_RDPUWD	0x025D4  /* DMA Rx Descriptor uC Data Write - RW */
#define I350_RDPURD	0x025D8  /* DMA Rx Descriptor uC Data Read - RW */
#define I350_RDPUCTL	0x025DC  /* DMA Rx Descriptor uC Control - RW */
#define I350_PBDIAG	0x02458  /* Packet Buffer Diagnostic - RW */
#define I350_RXPBS	0x02404  /* Rx Packet Buffer Size - RW */
#define I350_IRPBS	0x02404 /* Same as RXPBS, renamed for newer Si - RW */
#define I350_PBRWAC	0x024E8 /* Rx packet buffer wrap around counter - RO */
#define I350_RDTR	0x02820  /* Rx Delay Timer - RW */
#define I350_RADV	0x0282C  /* Rx Interrupt Absolute Delay Timer - RW */
#define I350_EMIADD	0x10     /* Extended Memory Indirect Address */
#define I350_EMIDATA	0x11     /* Extended Memory Indirect Data */
/* Shadow Ram Write Register - RW */
#define I350_SRWR		0x12018
#define I350_EEC_REG		0x12010

#define I350_I210_FLMNGCTL	0x12038
#define I350_I210_FLMNGDATA	0x1203C
#define I350_I210_FLMNGCNT	0x12040

#define I350_I210_FLSWCTL	0x12048
#define I350_I210_FLSWDATA	0x1204C
#define I350_I210_FLSWCNT	0x12050

#define I350_I210_FLA		0x1201C

#define I350_SHADOWINF		0x12068
#define I350_FLFWUPDATE	0x12108

#define I350_INVM_DATA_REG(_n)	(0x12120 + 4*(_n))
#define I350_INVM_SIZE		64 /* Number of INVM Data Registers */

/* QAV Tx mode control register */
#define I350_I210_TQAVCTRL	0x3570
#define I350_I210_LAUNCH_OS0 0x3578

/* QAV Tx mode control register bitfields masks */
/* QAV enable */
#define I350_TQAVCTRL_MODE			(1 << 0)
/* Fetching arbitration type */
#define I350_TQAVCTRL_FETCH_ARB		(1 << 4)
/* Fetching timer enable */
#define I350_TQAVCTRL_FETCH_TIMER_ENABLE	(1 << 5)
/* Launch arbitration type */
#define I350_TQAVCTRL_LAUNCH_ARB		(1 << 8)
/* Launch timer enable */
#define I350_TQAVCTRL_LAUNCH_TIMER_ENABLE	(1 << 9)
/* SP waits for SR enable */
#define I350_TQAVCTRL_SP_WAIT_SR		(1 << 10)
/* Fetching timer correction */
#define I350_TQAVCTRL_FETCH_TIMER_DELTA_OFFSET	16
#define I350_TQAVCTRL_FETCH_TIMER_DELTA	\
			(0xFFFF << I350_TQAVCTRL_FETCH_TIMER_DELTA_OFFSET)

/* High credit registers where _n can be 0 or 1. */
#define I350_I210_TQAVHC(_n)			(0x300C + 0x40 * (_n))

/* Queues fetch arbitration priority control register */
#define I350_I210_TQAVARBCTRL			0x3574
/* Queues priority masks where _n and _p can be 0-3. */
#define I350_TQAVARBCTRL_QUEUE_PRI(_n, _p)	((_p) << (2 * (_n)))
/* QAV Tx mode control registers where _n can be 0 or 1. */
#define I350_I210_TQAVCC(_n)			(0x3004 + 0x40 * (_n))

/* QAV Tx mode control register bitfields masks */
#define I350_TQAVCC_IDLE_SLOPE		0xFFFF /* Idle slope */
#define I350_TQAVCC_KEEP_CREDITS	(1 << 30) /* Keep credits opt enable */
#define I350_TQAVCC_QUEUE_MODE		(1 << 31) /* SP vs. SR Tx mode */

/* Good transmitted packets counter registers */
#define I350_PQGPTC(_n)		(0x010014 + (0x100 * (_n)))

/* Queues packet buffer size masks where _n can be 0-3 and _s 0-63 [kB] */
#define I350_I210_TXPBS_SIZE(_n, _s)	((_s) << (6 * (_n)))

#define I350_MMDAC			13 /* MMD Access Control */
#define I350_MMDAAD			14 /* MMD Access Address/Data */

/* Convenience macros
 *
 * Note: "_n" is the queue number of the register to be written to.
 *
 * Example usage:
 * I350_RDBAL_REG(current_rx_queue)
 */
#define I350_RDBAL(_n)	((_n) < 4 ? (0x02800 + ((_n) * 0x100)) : \
			 (0x0C000 + ((_n) * 0x40)))
#define I350_RDBAH(_n)	((_n) < 4 ? (0x02804 + ((_n) * 0x100)) : \
			 (0x0C004 + ((_n) * 0x40)))
#define I350_RDLEN(_n)	((_n) < 4 ? (0x02808 + ((_n) * 0x100)) : \
			 (0x0C008 + ((_n) * 0x40)))
#define I350_SRRCTL(_n)	((_n) < 4 ? (0x0280C + ((_n) * 0x100)) : \
				 (0x0C00C + ((_n) * 0x40)))
#define I350_RDH(_n)	((_n) < 4 ? (0x02810 + ((_n) * 0x100)) : \
			 (0x0C010 + ((_n) * 0x40)))
#define I350_RXCTL(_n)	((_n) < 4 ? (0x02814 + ((_n) * 0x100)) : \
			 (0x0C014 + ((_n) * 0x40)))
#define I350_DCA_RXCTRL(_n)	I350_RXCTL(_n)
#define I350_RDT(_n)	((_n) < 4 ? (0x02818 + ((_n) * 0x100)) : \
			 (0x0C018 + ((_n) * 0x40)))
#define I350_RXDCTL(_n)	((_n) < 4 ? (0x02828 + ((_n) * 0x100)) : \
				 (0x0C028 + ((_n) * 0x40)))
#define I350_RQDPC(_n)	((_n) < 4 ? (0x02830 + ((_n) * 0x100)) : \
			 (0x0C030 + ((_n) * 0x40)))
#define I350_TDBAL(_n)	((_n) < 4 ? (0x03800 + ((_n) * 0x100)) : \
			 (0x0E000 + ((_n) * 0x40)))
#define I350_TDBAH(_n)	((_n) < 4 ? (0x03804 + ((_n) * 0x100)) : \
			 (0x0E004 + ((_n) * 0x40)))
#define I350_TDLEN(_n)	((_n) < 4 ? (0x03808 + ((_n) * 0x100)) : \
			 (0x0E008 + ((_n) * 0x40)))
#define I350_TDH(_n)	((_n) < 4 ? (0x03810 + ((_n) * 0x100)) : \
			 (0x0E010 + ((_n) * 0x40)))
#define I350_TXCTL(_n)	((_n) < 4 ? (0x03814 + ((_n) * 0x100)) : \
			 (0x0E014 + ((_n) * 0x40)))
#define I350_DCA_TXCTRL(_n) I350_TXCTL(_n)
#define I350_TDT(_n)	((_n) < 4 ? (0x03818 + ((_n) * 0x100)) : \
			 (0x0E018 + ((_n) * 0x40)))
#define I350_TXDCTL(_n)	((_n) < 4 ? (0x03828 + ((_n) * 0x100)) : \
				 (0x0E028 + ((_n) * 0x40)))
#define I350_TDWBAL(_n)	((_n) < 4 ? (0x03838 + ((_n) * 0x100)) : \
				 (0x0E038 + ((_n) * 0x40)))
#define I350_TDWBAH(_n)	((_n) < 4 ? (0x0383C + ((_n) * 0x100)) : \
				 (0x0E03C + ((_n) * 0x40)))
#define I350_TARC(_n)		(0x03840 + ((_n) * 0x100))
#define I350_RSRPD		0x02C00  /* Rx Small Packet Detect - RW */
#define I350_RAID		0x02C08  /* Receive Ack Interrupt Delay - RW */
#define I350_TXDMAC		0x03000  /* Tx DMA Control - RW */
#define I350_KABGTXD		0x03004  /* AFE Band Gap Transmit Ref Data */
#define I350_PSRTYPE(_i)	(0x05480 + ((_i) * 4))
#define I350_RAL(_i)		(((_i) <= 15) ? (0x05400 + ((_i) * 8)) : \
				 (0x054E0 + ((_i - 16) * 8)))
#define I350_RAH(_i)		(((_i) <= 15) ? (0x05404 + ((_i) * 8)) : \
				 (0x054E4 + ((_i - 16) * 8)))
#define I350_SHRAL(_i)		(0x05438 + ((_i) * 8))
#define I350_SHRAH(_i)		(0x0543C + ((_i) * 8))
#define I350_IP4AT_REG(_i)	(0x05840 + ((_i) * 8))
#define I350_IP6AT_REG(_i)	(0x05880 + ((_i) * 4))
#define I350_WUPM_REG(_i)	(0x05A00 + ((_i) * 4))
#define I350_FFMT_REG(_i)	(0x09000 + ((_i) * 8))
#define I350_FFVT_REG(_i)	(0x09800 + ((_i) * 8))
#define I350_FFLT_REG(_i)	(0x05F00 + ((_i) * 8))
#define I350_PBSLAC		0x03100  /* Pkt Buffer Slave Access Control */
#define I350_PBSLAD(_n)	(0x03110 + (0x4 * (_n)))  /* Pkt Buffer DWORD */
#define I350_TXPBS		0x03404  /* Tx Packet Buffer Size - RW */
/* Same as TXPBS, renamed for newer Si - RW */
#define I350_ITPBS		0x03404
#define I350_TDFH		0x03410  /* Tx Data FIFO Head - RW */
#define I350_TDFT		0x03418  /* Tx Data FIFO Tail - RW */
#define I350_TDFHS		0x03420  /* Tx Data FIFO Head Saved - RW */
#define I350_TDFTS		0x03428  /* Tx Data FIFO Tail Saved - RW */
#define I350_TDFPC		0x03430  /* Tx Data FIFO Packet Count - RW */
#define I350_TDPUMB		0x0357C  /* DMA Tx Desc uC Mail Box - RW */
#define I350_TDPUAD		0x03580  /* DMA Tx Desc uC Addr Command - RW */
#define I350_TDPUWD		0x03584  /* DMA Tx Desc uC Data Write - RW */
#define I350_TDPURD		0x03588  /* DMA Tx Desc uC Data  Read  - RW */
#define I350_TDPUCTL		0x0358C  /* DMA Tx Desc uC Control - RW */
#define I350_DTXCTL		0x03590  /* DMA Tx Control - RW */
#define I350_DTXTCPFLGL	0x0359C /* DMA Tx Control flag low - RW */
#define I350_DTXTCPFLGH	0x035A0 /* DMA Tx Control flag high - RW */
/* DMA Tx Max Total Allow Size Reqs - RW */
#define I350_DTXMXSZRQ		0x03540
#define I350_TIDV	0x03820  /* Tx Interrupt Delay Value - RW */
#define I350_TADV	0x0382C  /* Tx Interrupt Absolute Delay Val - RW */
#define I350_TSPMT	0x03830  /* TCP Segmentation PAD & Min Threshold - RW */
/* Statistics Register Descriptions */
#define I350_CRCERRS	0x04000  /* CRC Error Count - R/clr */
#define I350_ALGNERRC	0x04004  /* Alignment Error Count - R/clr */
#define I350_SYMERRS	0x04008  /* Symbol Error Count - R/clr */
#define I350_RXERRC	0x0400C  /* Receive Error Count - R/clr */
#define I350_MPC	0x04010  /* Missed Packet Count - R/clr */
#define I350_SCC	0x04014  /* Single Collision Count - R/clr */
#define I350_ECOL	0x04018  /* Excessive Collision Count - R/clr */
#define I350_MCC	0x0401C  /* Multiple Collision Count - R/clr */
#define I350_LATECOL	0x04020  /* Late Collision Count - R/clr */
#define I350_COLC	0x04028  /* Collision Count - R/clr */
#define I350_DC	0x04030  /* Defer Count - R/clr */
#define I350_TNCRS	0x04034  /* Tx-No CRS - R/clr */
#define I350_SEC	0x04038  /* Sequence Error Count - R/clr */
#define I350_CEXTERR	0x0403C  /* Carrier Extension Error Count - R/clr */
#define I350_RLEC	0x04040  /* Receive Length Error Count - R/clr */
#define I350_XONRXC	0x04048  /* XON Rx Count - R/clr */
#define I350_XONTXC	0x0404C  /* XON Tx Count - R/clr */
#define I350_XOFFRXC	0x04050  /* XOFF Rx Count - R/clr */
#define I350_XOFFTXC	0x04054  /* XOFF Tx Count - R/clr */
#define I350_FCRUC	0x04058  /* Flow Control Rx Unsupported Count- R/clr */
#define I350_PRC64	0x0405C  /* Packets Rx (64 bytes) - R/clr */
#define I350_PRC127	0x04060  /* Packets Rx (65-127 bytes) - R/clr */
#define I350_PRC255	0x04064  /* Packets Rx (128-255 bytes) - R/clr */
#define I350_PRC511	0x04068  /* Packets Rx (255-511 bytes) - R/clr */
#define I350_PRC1023	0x0406C  /* Packets Rx (512-1023 bytes) - R/clr */
#define I350_PRC1522	0x04070  /* Packets Rx (1024-1522 bytes) - R/clr */
#define I350_GPRC	0x04074  /* Good Packets Rx Count - R/clr */
#define I350_BPRC	0x04078  /* Broadcast Packets Rx Count - R/clr */
#define I350_MPRC	0x0407C  /* Multicast Packets Rx Count - R/clr */
#define I350_GPTC	0x04080  /* Good Packets Tx Count - R/clr */
#define I350_GORCL	0x04088  /* Good Octets Rx Count Low - R/clr */
#define I350_GORCH	0x0408C  /* Good Octets Rx Count High - R/clr */
#define I350_GOTCL	0x04090  /* Good Octets Tx Count Low - R/clr */
#define I350_GOTCH	0x04094  /* Good Octets Tx Count High - R/clr */
#define I350_RNBC	0x040A0  /* Rx No Buffers Count - R/clr */
#define I350_RUC	0x040A4  /* Rx Undersize Count - R/clr */
#define I350_RFC	0x040A8  /* Rx Fragment Count - R/clr */
#define I350_ROC	0x040AC  /* Rx Oversize Count - R/clr */
#define I350_RJC	0x040B0  /* Rx Jabber Count - R/clr */
#define I350_MGTPRC	0x040B4  /* Management Packets Rx Count - R/clr */
#define I350_MGTPDC	0x040B8  /* Management Packets Dropped Count - R/clr */
#define I350_MGTPTC	0x040BC  /* Management Packets Tx Count - R/clr */
#define I350_TORL	0x040C0  /* Total Octets Rx Low - R/clr */
#define I350_TORH	0x040C4  /* Total Octets Rx High - R/clr */
#define I350_TOTL	0x040C8  /* Total Octets Tx Low - R/clr */
#define I350_TOTH	0x040CC  /* Total Octets Tx High - R/clr */
#define I350_TPR	0x040D0  /* Total Packets Rx - R/clr */
#define I350_TPT	0x040D4  /* Total Packets Tx - R/clr */
#define I350_PTC64	0x040D8  /* Packets Tx (64 bytes) - R/clr */
#define I350_PTC127	0x040DC  /* Packets Tx (65-127 bytes) - R/clr */
#define I350_PTC255	0x040E0  /* Packets Tx (128-255 bytes) - R/clr */
#define I350_PTC511	0x040E4  /* Packets Tx (256-511 bytes) - R/clr */
#define I350_PTC1023	0x040E8  /* Packets Tx (512-1023 bytes) - R/clr */
#define I350_PTC1522	0x040EC  /* Packets Tx (1024-1522 Bytes) - R/clr */
#define I350_MPTC	0x040F0  /* Multicast Packets Tx Count - R/clr */
#define I350_BPTC	0x040F4  /* Broadcast Packets Tx Count - R/clr */
#define I350_TSCTC	0x040F8  /* TCP Segmentation Context Tx - R/clr */
#define I350_TSCTFC	0x040FC  /* TCP Segmentation Context Tx Fail - R/clr */
#define I350_IAC	0x04100  /* Interrupt Assertion Count */
/* Interrupt Cause */
#define I350_ICRXPTC	0x04104  /* Interrupt Cause Rx Pkt Timer Expire Count */
#define I350_ICRXATC	0x04108  /* Interrupt Cause Rx Abs Timer Expire Count */
#define I350_ICTXPTC	0x0410C  /* Interrupt Cause Tx Pkt Timer Expire Count */
#define I350_ICTXATC	0x04110  /* Interrupt Cause Tx Abs Timer Expire Count */
#define I350_ICTXQEC	0x04118  /* Interrupt Cause Tx Queue Empty Count */
#define I350_ICTXQMTC	0x0411C  /* Interrupt Cause Tx Queue Min Thresh Count */
#define I350_ICRXDMTC	0x04120  /* Interrupt Cause Rx Desc Min Thresh Count */
#define I350_ICRXOC	0x04124  /* Interrupt Cause Receiver Overrun Count */
#define I350_CRC_OFFSET	0x05F50  /* CRC Offset register */

#define I350_VFGPRC	0x00F10
#define I350_VFGORC	0x00F18
#define I350_VFMPRC	0x00F3C
#define I350_VFGPTC	0x00F14
#define I350_VFGOTC	0x00F34
#define I350_VFGOTLBC	0x00F50
#define I350_VFGPTLBC	0x00F44
#define I350_VFGORLBC	0x00F48
#define I350_VFGPRLBC	0x00F40
/* Virtualization statistical counters */
#define I350_PFVFGPRC(_n)	(0x010010 + (0x100 * (_n)))
#define I350_PFVFGPTC(_n)	(0x010014 + (0x100 * (_n)))
#define I350_PFVFGORC(_n)	(0x010018 + (0x100 * (_n)))
#define I350_PFVFGOTC(_n)	(0x010034 + (0x100 * (_n)))
#define I350_PFVFMPRC(_n)	(0x010038 + (0x100 * (_n)))
#define I350_PFVFGPRLBC(_n)	(0x010040 + (0x100 * (_n)))
#define I350_PFVFGPTLBC(_n)	(0x010044 + (0x100 * (_n)))
#define I350_PFVFGORLBC(_n)	(0x010048 + (0x100 * (_n)))
#define I350_PFVFGOTLBC(_n)	(0x010050 + (0x100 * (_n)))

/* LinkSec */
#define I350_LSECTXUT		0x04300  /* Tx Untagged Pkt Cnt */
#define I350_LSECTXPKTE	0x04304  /* Encrypted Tx Pkts Cnt */
#define I350_LSECTXPKTP	0x04308  /* Protected Tx Pkt Cnt */
#define I350_LSECTXOCTE	0x0430C  /* Encrypted Tx Octets Cnt */
#define I350_LSECTXOCTP	0x04310  /* Protected Tx Octets Cnt */
#define I350_LSECRXUT		0x04314  /* Untagged non-Strict Rx Pkt Cnt */
#define I350_LSECRXOCTD	0x0431C  /* Rx Octets Decrypted Count */
#define I350_LSECRXOCTV	0x04320  /* Rx Octets Validated */
#define I350_LSECRXBAD		0x04324  /* Rx Bad Tag */
#define I350_LSECRXNOSCI	0x04328  /* Rx Packet No SCI Count */
#define I350_LSECRXUNSCI	0x0432C  /* Rx Packet Unknown SCI Count */
#define I350_LSECRXUNCH	0x04330  /* Rx Unchecked Packets Count */
#define I350_LSECRXDELAY	0x04340  /* Rx Delayed Packet Count */
#define I350_LSECRXLATE	0x04350  /* Rx Late Packets Count */
#define I350_LSECRXOK(_n)	(0x04360 + (0x04 * (_n))) /* Rx Pkt OK Cnt */
#define I350_LSECRXINV(_n)	(0x04380 + (0x04 * (_n))) /* Rx Invalid Cnt */
#define I350_LSECRXNV(_n)	(0x043A0 + (0x04 * (_n))) /* Rx Not Valid Cnt */
#define I350_LSECRXUNSA	0x043C0  /* Rx Unused SA Count */
#define I350_LSECRXNUSA	0x043D0  /* Rx Not Using SA Count */
#define I350_LSECTXCAP		0x0B000  /* Tx Capabilities Register - RO */
#define I350_LSECRXCAP		0x0B300  /* Rx Capabilities Register - RO */
#define I350_LSECTXCTRL	0x0B004  /* Tx Control - RW */
#define I350_LSECRXCTRL	0x0B304  /* Rx Control - RW */
#define I350_LSECTXSCL		0x0B008  /* Tx SCI Low - RW */
#define I350_LSECTXSCH		0x0B00C  /* Tx SCI High - RW */
#define I350_LSECTXSA		0x0B010  /* Tx SA0 - RW */
#define I350_LSECTXPN0		0x0B018  /* Tx SA PN 0 - RW */
#define I350_LSECTXPN1		0x0B01C  /* Tx SA PN 1 - RW */
#define I350_LSECRXSCL		0x0B3D0  /* Rx SCI Low - RW */
#define I350_LSECRXSCH		0x0B3E0  /* Rx SCI High - RW */
/* LinkSec Tx 128-bit Key 0 - WO */
#define I350_LSECTXKEY0(_n)	(0x0B020 + (0x04 * (_n)))
/* LinkSec Tx 128-bit Key 1 - WO */
#define I350_LSECTXKEY1(_n)	(0x0B030 + (0x04 * (_n)))
#define I350_LSECRXSA(_n)	(0x0B310 + (0x04 * (_n))) /* Rx SAs - RW */
#define I350_LSECRXPN(_n)	(0x0B330 + (0x04 * (_n))) /* Rx SAs - RW */
/* LinkSec Rx Keys  - where _n is the SA no. and _m the 4 dwords of the 128 bit
 * key - RW.
 */
#define I350_LSECRXKEY(_n, _m)	(0x0B350 + (0x10 * (_n)) + (0x04 * (_m)))

#define I350_SSVPC		0x041A0 /* Switch Security Violation Pkt Cnt */
#define I350_IPSCTRL		0xB430  /* IpSec Control Register */
#define I350_IPSRXCMD		0x0B408 /* IPSec Rx Command Register - RW */
#define I350_IPSRXIDX		0x0B400 /* IPSec Rx Index - RW */
/* IPSec Rx IPv4/v6 Address - RW */
#define I350_IPSRXIPADDR(_n)	(0x0B420 + (0x04 * (_n)))
/* IPSec Rx 128-bit Key - RW */
#define I350_IPSRXKEY(_n)	(0x0B410 + (0x04 * (_n)))
#define I350_IPSRXSALT		0x0B404  /* IPSec Rx Salt - RW */
#define I350_IPSRXSPI		0x0B40C  /* IPSec Rx SPI - RW */
/* IPSec Tx 128-bit Key - RW */
#define I350_IPSTXKEY(_n)	(0x0B460 + (0x04 * (_n)))
#define I350_IPSTXSALT		0x0B454  /* IPSec Tx Salt - RW */
#define I350_IPSTXIDX		0x0B450  /* IPSec Tx SA IDX - RW */
#define I350_PCS_CFG0	0x04200  /* PCS Configuration 0 - RW */
#define I350_PCS_LCTL	0x04208  /* PCS Link Control - RW */
#define I350_PCS_LSTAT	0x0420C  /* PCS Link Status - RO */
#define I350_CBTMPC	0x0402C  /* Circuit Breaker Tx Packet Count */
#define I350_HTDPMC	0x0403C  /* Host Transmit Discarded Packets */
#define I350_CBRDPC	0x04044  /* Circuit Breaker Rx Dropped Count */
#define I350_CBRMPC	0x040FC  /* Circuit Breaker Rx Packet Count */
#define I350_RPTHC	0x04104  /* Rx Packets To Host */
#define I350_HGPTC	0x04118  /* Host Good Packets Tx Count */
#define I350_HTCBDPC	0x04124  /* Host Tx Circuit Breaker Dropped Count */
#define I350_HGORCL	0x04128  /* Host Good Octets Received Count Low */
#define I350_HGORCH	0x0412C  /* Host Good Octets Received Count High */
#define I350_HGOTCL	0x04130  /* Host Good Octets Transmit Count Low */
#define I350_HGOTCH	0x04134  /* Host Good Octets Transmit Count High */
#define I350_LENERRS	0x04138  /* Length Errors Count */
#define I350_SCVPC	0x04228  /* SerDes/SGMII Code Violation Pkt Count */
#define I350_HRMPC	0x0A018  /* Header Redirection Missed Packet Count */
#define I350_PCS_ANADV	0x04218  /* AN advertisement - RW */
#define I350_PCS_LPAB	0x0421C  /* Link Partner Ability - RW */
#define I350_PCS_NPTX	0x04220  /* AN Next Page Transmit - RW */
#define I350_PCS_LPABNP	0x04224 /* Link Partner Ability Next Pg - RW */
#define I350_RXCSUM	0x05000  /* Rx Checksum Control - RW */
#define I350_RLPML	0x05004  /* Rx Long Packet Max Length */
#define I350_RFCTL	0x05008  /* Receive Filter Control*/
#define I350_MTA	0x05200  /* Multicast Table Array - RW Array */
#define I350_RA	0x05400  /* Receive Address - RW Array */
#define I350_RA2	0x054E0  /* 2nd half of Rx address array - RW Array */
#define I350_VFTA	0x05600  /* VLAN Filter Table Array - RW Array */
#define I350_VT_CTL	0x0581C  /* VMDq Control - RW */
#define I350_CIAA	0x05B88  /* Config Indirect Access Address - RW */
#define I350_CIAD	0x05B8C  /* Config Indirect Access Data - RW */
#define I350_VFQA0	0x0B000  /* VLAN Filter Queue Array 0 - RW Array */
#define I350_VFQA1	0x0B200  /* VLAN Filter Queue Array 1 - RW Array */
#define I350_WUC	0x05800  /* Wakeup Control - RW */
#define I350_WUFC	0x05808  /* Wakeup Filter Control - RW */
#define I350_WUS	0x05810  /* Wakeup Status - RO */
/* Management registers */
#define I350_MANC	0x05820  /* Management Control - RW */
#define I350_IPAV	0x05838  /* IP Address Valid - RW */
#define I350_IP4AT	0x05840  /* IPv4 Address Table - RW Array */
#define I350_IP6AT	0x05880  /* IPv6 Address Table - RW Array */
#define I350_WUPL	0x05900  /* Wakeup Packet Length - RW */
#define I350_WUPM	0x05A00  /* Wakeup Packet Memory - RO A */
/* MSI-X Table Register Descriptions */
#define I350_PBACL	0x05B68  /* MSIx PBA Clear - Read/Write 1's to clear */
#define I350_FFLT	0x05F00  /* Flexible Filter Length Table - RW Array */
#define I350_HOST_IF	0x08800  /* Host Interface */
#define I350_HIBBA	0x8F40   /* Host Interface Buffer Base Address */
/* Flexible Host Filter Table */
#define I350_FHFT(_n)	(0x09000 + ((_n) * 0x100))
/* Ext Flexible Host Filter Table */
#define I350_FHFT_EXT(_n)	(0x09A00 + ((_n) * 0x100))


#define I350_KMRNCTRLSTA	0x00034 /* MAC-PHY interface - RW */
#define I350_MANC2H		0x05860 /* Management Control To Host - RW */
/* Management Decision Filters */
#define I350_MDEF(_n)		(0x05890 + (4 * (_n)))
/* Semaphore registers */
#define I350_SW_FW_SYNC	0x05B5C /* SW-FW Synchronization - RW */
#define I350_CCMCTL	0x05B48 /* CCM Control Register */
#define I350_GIOCTL	0x05B44 /* GIO Analog Control Register */
#define I350_SCCTL	0x05B4C /* PCIc PLL Configuration Register */
/* PCIe Register Description */
#define I350_GCR	0x05B00 /* PCI-Ex Control */
#define I350_GCR2	0x05B64 /* PCI-Ex Control #2 */
#define I350_GSCL_1	0x05B10 /* PCI-Ex Statistic Control #1 */
#define I350_GSCL_2	0x05B14 /* PCI-Ex Statistic Control #2 */
#define I350_GSCL_3	0x05B18 /* PCI-Ex Statistic Control #3 */
#define I350_GSCL_4	0x05B1C /* PCI-Ex Statistic Control #4 */
/* Function Active and Power State to MNG */
#define I350_FACTPS	0x05B30
#define I350_SWSM	0x05B50 /* SW Semaphore */
#define I350_FWSM	0x05B54 /* FW Semaphore */
/* Driver-only SW semaphore (not used by BOOT agents) */
#define I350_SWSM2	0x05B58
#define I350_DCA_ID	0x05B70 /* DCA Requester ID Information - RO */
#define I350_DCA_CTRL	0x05B74 /* DCA Control - RW */
#define I350_UFUSE	0x05B78 /* UFUSE - RO */
#define I350_FFLT_DBG	0x05F04 /* Debug Register */
#define I350_HICR	0x08F00 /* Host Interface Control */
#define I350_FWSTS	0x08F0C /* FW Status */

/* RSS registers */
#define I350_CPUVEC	0x02C10 /* CPU Vector Register - RW */
#define I350_MRQC	0x05818 /* Multiple Receive Control - RW */
#define I350_IMIR(_i)	(0x05A80 + ((_i) * 4))  /* Immediate Interrupt */
#define I350_IMIREXT(_i)	(0x05AA0 + ((_i) * 4)) /* Immediate INTR Ext*/
#define I350_IMIRVP		0x05AC0 /* Immediate INT Rx VLAN Priority -RW */
#define I350_MSIXBM(_i)	(0x01600 + ((_i) * 4)) /* MSI-X Alloc Reg -RW */
/* Redirection Table - RW Array */
#define I350_RETA(_i)	(0x05C00 + ((_i) * 4))
/* RSS Random Key - RW Array */
#define I350_RSSRK(_i)	(0x05C80 + ((_i) * 4))
#define I350_RSSIM	0x05864 /* RSS Interrupt Mask */
#define I350_RSSIR	0x05868 /* RSS Interrupt Request */
/* VT Registers */
#define I350_SWPBS	0x03004 /* Switch Packet Buffer Size - RW */
#define I350_MBVFICR	0x00C80 /* Mailbox VF Cause - RWC */
#define I350_MBVFIMR	0x00C84 /* Mailbox VF int Mask - RW */
#define I350_VFLRE	0x00C88 /* VF Register Events - RWC */
#define I350_VFRE	0x00C8C /* VF Receive Enables */
#define I350_VFTE	0x00C90 /* VF Transmit Enables */
#define I350_QDE	0x02408 /* Queue Drop Enable - RW */
#define I350_DTXSWC	0x03500 /* DMA Tx Switch Control - RW */
#define I350_WVBR	0x03554 /* VM Wrong Behavior - RWS */
#define I350_RPLOLR	0x05AF0 /* Replication Offload - RW */
#define I350_UTA	0x0A000 /* Unicast Table Array - RW */
#define I350_IOVTCL	0x05BBC /* IOV Control Register */
#define I350_VMRCTL	0X05D80 /* Virtual Mirror Rule Control */
#define I350_VMRVLAN	0x05D90 /* Virtual Mirror Rule VLAN */
#define I350_VMRVM	0x05DA0 /* Virtual Mirror Rule VM */
#define I350_MDFB	0x03558 /* Malicious Driver free block */
#define I350_LVMMC	0x03548 /* Last VM Misbehavior cause */
#define I350_TXSWC	0x05ACC /* Tx Switch Control */
#define I350_SCCRL	0x05DB0 /* Storm Control Control */
#define I350_BSCTRH	0x05DB8 /* Broadcast Storm Control Threshold */
#define I350_MSCTRH	0x05DBC /* Multicast Storm Control Threshold */
/* These act per VF so an array friendly macro is used */
#define I350_V2PMAILBOX(_n)	(0x00C40 + (4 * (_n)))
#define I350_P2VMAILBOX(_n)	(0x00C00 + (4 * (_n)))
#define I350_VMBMEM(_n)	(0x00800 + (64 * (_n)))
#define I350_VFVMBMEM(_n)	(0x00800 + (_n))
#define I350_VMOLR(_n)		(0x05AD0 + (4 * (_n)))
/* VLAN Virtual Machine Filter - RW */
#define I350_VLVF(_n)		(0x05D00 + (4 * (_n)))
#define I350_VMVIR(_n)		(0x03700 + (4 * (_n)))
#define I350_DVMOLR(_n)	(0x0C038 + (0x40 * (_n))) /* DMA VM offload */
#define I350_VTCTRL(_n)	(0x10000 + (0x100 * (_n))) /* VT Control */
#define I350_TSYNCRXCTL	0x0B620 /* Rx Time Sync Control register - RW */
#define I350_TSYNCTXCTL	0x0B614 /* Tx Time Sync Control register - RW */
#define I350_TSYNCRXCFG	0x05F50 /* Time Sync Rx Configuration - RW */
#define I350_RXSTMPL	0x0B624 /* Rx timestamp Low - RO */
#define I350_RXSTMPH	0x0B628 /* Rx timestamp High - RO */
#define I350_RXSATRL	0x0B62C /* Rx timestamp attribute low - RO */
#define I350_RXSATRH	0x0B630 /* Rx timestamp attribute high - RO */
#define I350_TXSTMPL	0x0B618 /* Tx timestamp value Low - RO */
#define I350_TXSTMPH	0x0B61C /* Tx timestamp value High - RO */
#define I350_SYSTIML	0x0B600 /* System time register Low - RO */
#define I350_SYSTIMH	0x0B604 /* System time register High - RO */
#define I350_TIMINCA	0x0B608 /* Increment attributes register - RW */
#define I350_TIMADJL	0x0B60C /* Time sync time adjustment offset Low - RW */
#define I350_TIMADJH	0x0B610 /* Time sync time adjustment offset High - RW */
#define I350_TSAUXC	0x0B640 /* Timesync Auxiliary Control register */
#define	I350_SYSSTMPL	0x0B648 /* HH Timesync system stamp low register */
#define	I350_SYSSTMPH	0x0B64C /* HH Timesync system stamp hi register */
#define	I350_PLTSTMPL	0x0B640 /* HH Timesync platform stamp low register */
#define	I350_PLTSTMPH	0x0B644 /* HH Timesync platform stamp hi register */
#define I350_SYSTIMR	0x0B6F8 /* System time register Residue */
#define I350_TSICR	0x0B66C /* Interrupt Cause Register */
#define I350_TSIM	0x0B674 /* Interrupt Mask Register */
#define I350_RXMTRL	0x0B634 /* Time sync Rx EtherType and Msg Type - RW */
#define I350_RXUDP	0x0B638 /* Time Sync Rx UDP Port - RW */

/* Filtering Registers */
#define I350_SAQF(_n)	(0x05980 + (4 * (_n))) /* Source Address Queue Fltr */
#define I350_DAQF(_n)	(0x059A0 + (4 * (_n))) /* Dest Address Queue Fltr */
#define I350_SPQF(_n)	(0x059C0 + (4 * (_n))) /* Source Port Queue Fltr */
#define I350_FTQF(_n)	(0x059E0 + (4 * (_n))) /* 5-tuple Queue Fltr */
#define I350_TTQF(_n)	(0x059E0 + (4 * (_n))) /* 2-tuple Queue Fltr */
#define I350_SYNQF(_n)	(0x055FC + (4 * (_n))) /* SYN Packet Queue Fltr */
#define I350_ETQF(_n)	(0x05CB0 + (4 * (_n))) /* EType Queue Fltr */

/* ETQF register bit definitions */
#define I350_ETQF_FILTER_ENABLE	(1 << 26)
#define I350_ETQF_IMM_INT		(1 << 29)
#define I350_ETQF_QUEUE_ENABLE		(1 << 31)
#define I350_ETQF_QUEUE_SHIFT		16
#define I350_ETQF_QUEUE_MASK		0x00070000
#define I350_ETQF_ETYPE_MASK		0x0000FFFF

#define I350_RTTDCS	0x3600 /* Reedtown Tx Desc plane control and status */
#define I350_RTTPCS	0x3474 /* Reedtown Tx Packet Plane control and status */
#define I350_RTRPCS	0x2474 /* Rx packet plane control and status */
#define I350_RTRUP2TC	0x05AC4 /* Rx User Priority to Traffic Class */
#define I350_RTTUP2TC	0x0418 /* Transmit User Priority to Traffic Class */
/* Tx Desc plane TC Rate-scheduler config */
#define I350_RTTDTCRC(_n)	(0x3610 + ((_n) * 4))
/* Tx Packet plane TC Rate-Scheduler Config */
#define I350_RTTPTCRC(_n)	(0x3480 + ((_n) * 4))
/* Rx Packet plane TC Rate-Scheduler Config */
#define I350_RTRPTCRC(_n)	(0x2480 + ((_n) * 4))
/* Tx Desc Plane TC Rate-Scheduler Status */
#define I350_RTTDTCRS(_n)	(0x3630 + ((_n) * 4))
/* Tx Desc Plane TC Rate-Scheduler MMW */
#define I350_RTTDTCRM(_n)	(0x3650 + ((_n) * 4))
/* Tx Packet plane TC Rate-Scheduler Status */
#define I350_RTTPTCRS(_n)	(0x34A0 + ((_n) * 4))
/* Tx Packet plane TC Rate-scheduler MMW */
#define I350_RTTPTCRM(_n)	(0x34C0 + ((_n) * 4))
/* Rx Packet plane TC Rate-Scheduler Status */
#define I350_RTRPTCRS(_n)	(0x24A0 + ((_n) * 4))
/* Rx Packet plane TC Rate-Scheduler MMW */
#define I350_RTRPTCRM(_n)	(0x24C0 + ((_n) * 4))
/* Tx Desc plane VM Rate-Scheduler MMW*/
#define I350_RTTDVMRM(_n)	(0x3670 + ((_n) * 4))
/* Tx BCN Rate-Scheduler MMW */
#define I350_RTTBCNRM(_n)	(0x3690 + ((_n) * 4))
#define I350_RTTDQSEL	0x3604  /* Tx Desc Plane Queue Select */
#define I350_RTTDVMRC	0x3608  /* Tx Desc Plane VM Rate-Scheduler Config */
#define I350_RTTDVMRS	0x360C  /* Tx Desc Plane VM Rate-Scheduler Status */
#define I350_RTTBCNRC	0x36B0  /* Tx BCN Rate-Scheduler Config */
#define I350_RTTBCNRS	0x36B4  /* Tx BCN Rate-Scheduler Status */
#define I350_RTTBCNCR	0xB200  /* Tx BCN Control Register */
#define I350_RTTBCNTG	0x35A4  /* Tx BCN Tagging */
#define I350_RTTBCNCP	0xB208  /* Tx BCN Congestion point */
#define I350_RTRBCNCR	0xB20C  /* Rx BCN Control Register */
#define I350_RTTBCNRD	0x36B8  /* Tx BCN Rate Drift */
#define I350_PFCTOP	0x1080  /* Priority Flow Control Type and Opcode */
#define I350_RTTBCNIDX	0xB204  /* Tx BCN Congestion Point */
#define I350_RTTBCNACH	0x0B214 /* Tx BCN Control High */
#define I350_RTTBCNACL	0x0B210 /* Tx BCN Control Low */

/* DMA Coalescing registers */
#define I350_DMACR	0x02508 /* Control Register */
#define I350_DMCTXTH	0x03550 /* Transmit Threshold */
#define I350_DMCTLX	0x02514 /* Time to Lx Request */
#define I350_DMCRTRH	0x05DD0 /* Receive Packet Rate Threshold */
#define I350_DMCCNT	0x05DD4 /* Current Rx Count */
#define I350_FCRTC	0x02170 /* Flow Control Rx high watermark */
#define I350_PCIEMISC	0x05BB8 /* PCIE misc config register */

/* PCIe Parity Status Register */
#define I350_PCIEERRSTS	0x05BA8

#define I350_PROXYS	0x5F64 /* Proxying Status */
#define I350_PROXYFC	0x5F60 /* Proxying Filter Control */
/* Thermal sensor configuration and status registers */
#define I350_THMJT	0x08100 /* Junction Temperature */
#define I350_THLOWTC	0x08104 /* Low Threshold Control */
#define I350_THMIDTC	0x08108 /* Mid Threshold Control */
#define I350_THHIGHTC	0x0810C /* High Threshold Control */
#define I350_THSTAT	0x08110 /* Thermal Sensor Status */

/* Energy Efficient Ethernet "EEE" registers */
#define I350_IPCNFG	0x0E38 /* Internal PHY Configuration */
#define I350_LTRC	0x01A0 /* Latency Tolerance Reporting Control */
#define I350_EEER	0x0E30 /* Energy Efficient Ethernet "EEE"*/
#define I350_EEE_SU	0x0E34 /* EEE Setup */
#define I350_TLPIC	0x4148 /* EEE Tx LPI Count - TLPIC */
#define I350_RLPIC	0x414C /* EEE Rx LPI Count - RLPIC */

/* OS2BMC Registers */
#define I350_B2OSPC	0x08FE0 /* BMC2OS packets sent by BMC */
#define I350_B2OGPRC	0x04158 /* BMC2OS packets received by host */
#define I350_O2BGPTC	0x08FE4 /* OS2BMC packets received by BMC */
#define I350_O2BSPC	0x0415C /* OS2BMC packets transmitted by host */


struct reg_info {
	uint32_t base_addr;
	uint32_t count;
	uint32_t stride;
	const char *name;
};

static const struct reg_info i350_regs_general[] = {
	{I350_CTRL, 1, 1, "I350_CTRL"},
	{I350_STATUS, 1, 1, "I350_STATUS"},
	{I350_CTRL_EXT, 1, 1, "I350_CTRL_EXT"},
	{I350_MDIC, 1, 1, "I350_MDIC"},
	{I350_SCTL, 1, 1, "I350_SCTL"},
	{I350_CONNSW, 1, 1, "I350_CONNSW"},
	{I350_VET, 1, 1, "I350_VET"},
	{I350_LEDCTL, 1, 1, "I350_LEDCTL"},
	{I350_PBA, 1, 1, "I350_PBA"},
	{I350_PBS, 1, 1, "I350_PBS"},
	{I350_FRTIMER, 1, 1, "I350_FRTIMER"},
	{I350_TCPTIMER, 1, 1, "I350_TCPTIMER"},
	{0, 0, 0, ""}
};

static const struct reg_info i350_regs_nvm[] = {
	{I350_EECD, 1, 1, "I350_EECD"},
	{0, 0, 0, ""}
};

static const struct reg_info i350_regs_interrupt[] = {
	{I350_EICS, 1, 1, "I350_EICS"},
	{I350_EIMS, 1, 1, "I350_EIMS"},
	{I350_EIMC, 1, 1, "I350_EIMC"},
	{I350_EIAC, 1, 1, "I350_EIAC"},
	{I350_EIAM, 1, 1, "I350_EIAM"},
	{I350_ICS, 1, 1, "I350_ICS"},
	{I350_IMS, 1, 1, "I350_IMS"},
	{I350_IMC, 1, 1, "I350_IMC"},
	{I350_IAC, 1, 1, "I350_IAC"},
	{I350_IAM,  1, 1, "I350_IAM"},
	{I350_IMIRVP, 1, 1, "I350_IMIRVP"},
	{I350_EITR(0), 10, 4, "I350_EITR"},
	{I350_IMIR(0), 8, 4, "I350_IMIR"},
	{I350_IMIREXT(0), 8, 4, "I350_IMIREXT"},
	{0, 0, 0, ""}
};

static const struct reg_info i350_regs_fctl[] = {
	{I350_FCAL, 1, 1, "I350_FCAL"},
	{I350_FCAH, 1, 1, "I350_FCAH"},
	{I350_FCTTV, 1, 1, "I350_FCTTV"},
	{I350_FCRTL, 1, 1, "I350_FCRTL"},
	{I350_FCRTH, 1, 1, "I350_FCRTH"},
	{I350_FCRTV, 1, 1, "I350_FCRTV"},
	{0, 0, 0, ""}
};

static const struct reg_info i350_regs_rxdma[] = {
	{I350_RDBAL(0), 4, 0x100, "I350_RDBAL"},
	{I350_RDBAH(0), 4, 0x100, "I350_RDBAH"},
	{I350_RDLEN(0), 4, 0x100, "I350_RDLEN"},
	{I350_RDH(0), 4, 0x100, "I350_RDH"},
	{I350_RDT(0), 4, 0x100, "I350_RDT"},
	{I350_RXCTL(0), 4, 0x100, "I350_RXCTL"},
	{I350_SRRCTL(0), 4, 0x100, "I350_SRRCTL"},
	{I350_DCA_RXCTRL(0), 4, 0x100, "I350_DCA_RXCTRL"},
	{0, 0, 0, ""}
};

static const struct reg_info i350_regs_rx[] = {
	{I350_RCTL, 1, 1, "I350_RCTL"},
	{I350_RXCSUM, 1, 1, "I350_RXCSUM"},
	{I350_RLPML, 1, 1, "I350_RLPML"},
	{I350_RFCTL, 1, 1, "I350_RFCTL"},
	{I350_MRQC, 1, 1, "I350_MRQC"},
	{I350_VT_CTL, 1, 1, "I350_VT_CTL"},
	{I350_RAL(0), 16, 8, "I350_RAL"},
	{I350_RAH(0), 16, 8, "I350_RAH"},
	{0, 0, 0, ""}
};

static const struct reg_info i350_regs_tx[] = {
	{I350_TCTL, 1, 1, "I350_TCTL"},
	{I350_TCTL_EXT, 1, 1, "I350_TCTL_EXT"},
	{I350_TIPG, 1, 1, "I350_TIPG"},
	{I350_DTXCTL, 1, 1, "I350_DTXCTL"},
	{I350_TDBAL(0), 4, 0x100, "I350_TDBAL"},
	{I350_TDBAH(0), 4, 0x100, "I350_TDBAH"},
	{I350_TDLEN(0), 4, 0x100, "I350_TDLEN"},
	{I350_TDH(0), 4, 0x100, "I350_TDLEN"},
	{I350_TDT(0), 4, 0x100, "I350_TDT"},
	{I350_TXDCTL(0), 4, 0x100, "I350_TXDCTL"},
	{I350_TDWBAL(0), 4, 0x100, "I350_TDWBAL"},
	{I350_TDWBAH(0), 4, 0x100, "I350_TDWBAH"},
	{I350_DCA_TXCTRL(0), 4, 0x100, "I350_DCA_TXCTRL"},
	{I350_TDFH, 1, 1, "I350_TDFH"},
	{I350_TDFT, 1, 1, "I350_TDFT"},
	{I350_TDFHS, 1, 1, "I350_TDFHS"},
	{I350_TDFPC, 1, 1, "I350_TDFPC"},
	{0, 0, 0, ""}
};

static const struct reg_info i350_regs_wakeup[] = {
	{I350_WUC, 1, 1, "I350_WUC"},
	{I350_WUFC, 1, 1, "I350_WUFC"},
	{I350_WUS, 1, 1, "I350_WUS"},
	{I350_IPAV, 1, 1, "I350_IPAV"},
	{I350_WUPL, 1, 1, "I350_WUPL"},
	{I350_IP4AT_REG(0), 4, 8, "I350_IP4AT_REG"},
	{I350_IP6AT_REG(0), 4, 4, "I350_IP6AT_REG"},
	{I350_WUPM_REG(0), 4, 4, "I350_WUPM_REG"},
	{I350_FFMT_REG(0), 4, 8, "I350_FFMT_REG"},
	{I350_FFVT_REG(0), 4, 8, "I350_FFVT_REG"},
	{I350_FFLT_REG(0), 4, 8, "I350_FFLT_REG"},
	{0, 0, 0, ""}
};

static const struct reg_info i350_regs_mac[] = {
	{I350_PCS_CFG0, 1, 1, "I350_PCS_CFG0"},
	{I350_PCS_LCTL, 1, 1, "I350_PCS_LCTL"},
	{I350_PCS_LSTAT, 1, 1, "I350_PCS_LSTAT"},
	{I350_PCS_ANADV, 1, 1, "I350_PCS_ANADV"},
	{I350_PCS_LPAB, 1, 1, "I350_PCS_LPAB"},
	{I350_PCS_NPTX, 1, 1, "I350_PCS_NPTX"},
	{I350_PCS_LPABNP, 1, 1, "I350_PCS_LPABNP"},
	{0, 0, 0, ""}
};

static const struct reg_info *i350_regs[] = {
				i350_regs_general,
				i350_regs_nvm,
				i350_regs_interrupt,
				i350_regs_fctl,
				i350_regs_rxdma,
				i350_regs_rx,
				i350_regs_tx,
				i350_regs_wakeup,
				i350_regs_mac,
				NULL};

/* FIXME: reading igb_regs_interrupt results side-effect which doesn't
 * work with VFIO; re-install igb_regs_interrupt once issue is resolved.
 */
static const struct reg_info *i350vf_regs[] = {
				i350_regs_general,
				i350_regs_rxdma,
				i350_regs_tx,
				NULL};

static inline int
i350_read_regs(struct i350_hw *hw, const struct reg_info *reg,
	uint32_t *reg_buf)
{
	unsigned int i;

	for (i = 0; i < reg->count; i++) {
		reg_buf[i] = I350_READ_REG(hw,
				reg->base_addr + i * reg->stride);
	}
	return reg->count;
};

static inline int
i350_reg_group_count(const struct reg_info *regs)
{
	int count = 0;
	int i = 0;

	while (regs[i].count)
		count += regs[i++].count;
	return count;
};

static inline int
i350_read_regs_group(struct rte_eth_dev *dev, uint32_t *reg_buf,
		const struct reg_info *regs)
{
	int count = 0;
	int i = 0;
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	while (regs[i].count)
		count += i350_read_regs(hw, &regs[i++], &reg_buf[count]);
	return count;
};

#endif /* _IGB_REGS_H_ */
