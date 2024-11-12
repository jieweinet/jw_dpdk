/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2001-2020 Intel Corporation
 */

#ifndef _I350_MBX_H_
#define _I350_MBX_H_

#include "i350_api.h"

/* Define mailbox register bits */
#define I350_V2PMAILBOX_REQ	0x00000001 /* Request for PF Ready bit */
#define I350_V2PMAILBOX_ACK	0x00000002 /* Ack PF message received */
#define I350_V2PMAILBOX_VFU	0x00000004 /* VF owns the mailbox buffer */
#define I350_V2PMAILBOX_PFU	0x00000008 /* PF owns the mailbox buffer */
#define I350_V2PMAILBOX_PFSTS	0x00000010 /* PF wrote a message in the MB */
#define I350_V2PMAILBOX_PFACK	0x00000020 /* PF ack the previous VF msg */
#define I350_V2PMAILBOX_RSTI	0x00000040 /* PF has reset indication */
#define I350_V2PMAILBOX_RSTD	0x00000080 /* PF has indicated reset done */
#define I350_V2PMAILBOX_R2C_BITS 0x000000B0 /* All read to clear bits */

#define I350_P2VMAILBOX_STS	0x00000001 /* Initiate message send to VF */
#define I350_P2VMAILBOX_ACK	0x00000002 /* Ack message recv'd from VF */
#define I350_P2VMAILBOX_VFU	0x00000004 /* VF owns the mailbox buffer */
#define I350_P2VMAILBOX_PFU	0x00000008 /* PF owns the mailbox buffer */
#define I350_P2VMAILBOX_RVFU	0x00000010 /* Reset VFU - used when VF stuck */

#define I350_MBVFICR_VFREQ_MASK 0x000000FF /* bits for VF messages */
#define I350_MBVFICR_VFREQ_VF1	0x00000001 /* bit for VF 1 message */
#define I350_MBVFICR_VFACK_MASK 0x00FF0000 /* bits for VF acks */
#define I350_MBVFICR_VFACK_VF1	0x00010000 /* bit for VF 1 ack */

#define I350_VFMAILBOX_SIZE	16 /* 16 32 bit words - 64 bytes */

/* If it's a I350_VF_* msg then it originates in the VF and is sent to the
 * PF.  The reverse is true if it is I350_PF_*.
 * Message ACK's are the value or'd with 0xF0000000
 */
/* Msgs below or'd with this are the ACK */
#define I350_VT_MSGTYPE_ACK	0x80000000
/* Msgs below or'd with this are the NACK */
#define I350_VT_MSGTYPE_NACK	0x40000000
/* Indicates that VF is still clear to send requests */
#define I350_VT_MSGTYPE_CTS	0x20000000
#define I350_VT_MSGINFO_SHIFT	16
/* bits 23:16 are used for extra info for certain messages */
#define I350_VT_MSGINFO_MASK	(0xFF << I350_VT_MSGINFO_SHIFT)

#define I350_VF_RESET			0x01 /* VF requests reset */
#define I350_VF_SET_MAC_ADDR		0x02 /* VF requests to set MAC addr */
#define I350_VF_SET_MULTICAST		0x03 /* VF requests to set MC addr */
#define I350_VF_SET_MULTICAST_COUNT_MASK (0x1F << I350_VT_MSGINFO_SHIFT)
#define I350_VF_SET_MULTICAST_OVERFLOW	(0x80 << I350_VT_MSGINFO_SHIFT)
#define I350_VF_SET_VLAN		0x04 /* VF requests to set VLAN */
#define I350_VF_SET_VLAN_ADD		(0x01 << I350_VT_MSGINFO_SHIFT)
#define I350_VF_SET_LPE		0x05 /* reqs to set VMOLR.LPE */
#define I350_VF_SET_PROMISC		0x06 /* reqs to clear VMOLR.ROPE/MPME*/
#define I350_VF_SET_PROMISC_UNICAST	(0x01 << I350_VT_MSGINFO_SHIFT)
#define I350_VF_SET_PROMISC_MULTICAST	(0x02 << I350_VT_MSGINFO_SHIFT)

#define I350_PF_CONTROL_MSG		0x0100 /* PF control message */

#define I350_VF_MBX_INIT_TIMEOUT	2000 /* number of retries on mailbox */
#define I350_VF_MBX_INIT_DELAY		500  /* microseconds between retries */

s32 i350_read_mbx(struct i350_hw *, u32 *, u16, u16);
s32 i350_write_mbx(struct i350_hw *, u32 *, u16, u16);
s32 i350_read_posted_mbx(struct i350_hw *, u32 *, u16, u16);
s32 i350_write_posted_mbx(struct i350_hw *, u32 *, u16, u16);
s32 i350_check_for_msg(struct i350_hw *, u16);
s32 i350_check_for_ack(struct i350_hw *, u16);
s32 i350_check_for_rst(struct i350_hw *, u16);
void i350_init_mbx_ops_generic(struct i350_hw *hw);
s32 i350_init_mbx_params_vf(struct i350_hw *);
s32 i350_init_mbx_params_pf(struct i350_hw *);

#endif /* _I350_MBX_H_ */
