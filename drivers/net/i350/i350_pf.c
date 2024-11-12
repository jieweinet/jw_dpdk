/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2010-2016 Intel Corporation
 */

#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <inttypes.h>

#include <bus_pci_driver.h>
#include <rte_interrupts.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_eal.h>
#include <rte_ether.h>
#include <ethdev_driver.h>
#include <rte_memcpy.h>
#include <rte_malloc.h>
#include <rte_random.h>

#include "i350_defines.h"
#include "i350_regs.h"
#include "i350_hw.h"
#include "i350_ethdev.h"

static inline uint16_t
dev_num_vf(struct rte_eth_dev *eth_dev)
{
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(eth_dev);

	return pci_dev->max_vfs;
}

static inline
int i350_vf_perm_addr_gen(struct rte_eth_dev *dev, uint16_t vf_num)
{
	unsigned char vf_mac_addr[RTE_ETHER_ADDR_LEN];
	struct i350_vf_info *vfinfo =
		*I350_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private);
	uint16_t vfn;

	for (vfn = 0; vfn < vf_num; vfn++) {
		rte_eth_random_addr(vf_mac_addr);
		/* keep the random address as default */
		memcpy(vfinfo[vfn].vf_mac_addresses, vf_mac_addr,
				RTE_ETHER_ADDR_LEN);
	}

	return 0;
}

static inline int
i350_mb_intr_setup(struct rte_eth_dev *dev)
{
	struct i350_interrupt *intr =
		I350_DEV_PRIVATE_TO_INTR(dev->data->dev_private);

	intr->mask |= I350_ICR_VMMB;

	return 0;
}

void i350_pf_host_init(struct rte_eth_dev *eth_dev)
{
	struct i350_vf_info **vfinfo =
		I350_DEV_PRIVATE_TO_P_VFDATA(eth_dev->data->dev_private);
	struct i350_hw *hw =
		I350_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);
	uint16_t vf_num;
	uint8_t nb_queue;

	RTE_ETH_DEV_SRIOV(eth_dev).active = 0;
	if (0 == (vf_num = dev_num_vf(eth_dev)))
		return;

	nb_queue = 1;

	*vfinfo = rte_zmalloc("vf_info", sizeof(struct i350_vf_info) * vf_num, 0);
	if (*vfinfo == NULL)
		rte_panic("Cannot allocate memory for private VF data\n");

	RTE_ETH_DEV_SRIOV(eth_dev).active = RTE_ETH_8_POOLS;
	RTE_ETH_DEV_SRIOV(eth_dev).nb_q_per_pool = nb_queue;
	RTE_ETH_DEV_SRIOV(eth_dev).def_vmdq_idx = vf_num;
	RTE_ETH_DEV_SRIOV(eth_dev).def_pool_q_idx = (uint16_t)(vf_num * nb_queue);

	i350_vf_perm_addr_gen(eth_dev, vf_num);

	/* set mb interrupt mask */
	i350_mb_intr_setup(eth_dev);

	return;
}

void i350_pf_host_uninit(struct rte_eth_dev *dev)
{
	struct i350_vf_info **vfinfo;
	uint16_t vf_num;

	PMD_INIT_FUNC_TRACE();

	vfinfo = I350_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private);

	RTE_ETH_DEV_SRIOV(dev).active = 0;
	RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool = 0;
	RTE_ETH_DEV_SRIOV(dev).def_vmdq_idx = 0;
	RTE_ETH_DEV_SRIOV(dev).def_pool_q_idx = 0;

	vf_num = dev_num_vf(dev);
	if (vf_num == 0)
		return;

	rte_free(*vfinfo);
	*vfinfo = NULL;
}

#define I350_RAH_POOLSEL_SHIFT    (18)
int i350_pf_host_configure(struct rte_eth_dev *eth_dev)
{
	uint32_t vtctl;
	uint16_t vf_num;
	struct i350_hw *hw =
		I350_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);
	uint32_t vlanctrl;
	int i;
	uint32_t rah;

	if (0 == (vf_num = dev_num_vf(eth_dev)))
		return -1;

	/* enable VMDq and set the default pool for PF */
	vtctl = I350_READ_REG(hw, I350_VT_CTL);
	vtctl &= ~I350_VT_CTL_DEFAULT_POOL_MASK;
	vtctl |= RTE_ETH_DEV_SRIOV(eth_dev).def_vmdq_idx
		<< I350_VT_CTL_DEFAULT_POOL_SHIFT;
	vtctl |= I350_VT_CTL_VM_REPL_EN;
	I350_WRITE_REG(hw, I350_VT_CTL, vtctl);

	/* Enable pools reserved to PF only */
	I350_WRITE_REG(hw, I350_VFRE, (~0U) << vf_num);
	I350_WRITE_REG(hw, I350_VFTE, (~0U) << vf_num);

	/* PFDMA Tx General Switch Control Enables VMDQ loopback */
	I350_WRITE_REG(hw, I350_TXSWC, I350_DTXSWC_VMDQ_LOOPBACK_EN);


	/* clear VMDq map to permanent rar 0 */
	rah = I350_READ_REG(hw, I350_RAH(0));
	rah &= ~ (0xFF << I350_RAH_POOLSEL_SHIFT);
	I350_WRITE_REG(hw, I350_RAH(0), rah);

	/* clear VMDq map to scan rar 32 */
	rah = I350_READ_REG(hw, I350_RAH(hw->mac.rar_entry_count));
	rah &= ~ (0xFF << I350_RAH_POOLSEL_SHIFT);
	I350_WRITE_REG(hw, I350_RAH(hw->mac.rar_entry_count), rah);

	/* set VMDq map to default PF pool */
	rah = I350_READ_REG(hw, I350_RAH(0));
	rah |= (0x1 << (RTE_ETH_DEV_SRIOV(eth_dev).def_vmdq_idx +
			I350_RAH_POOLSEL_SHIFT));
	I350_WRITE_REG(hw, I350_RAH(0), rah);

	/*
	 * enable vlan filtering and allow all vlan tags through
	 */
	vlanctrl = I350_READ_REG(hw, I350_RCTL);
	vlanctrl |= I350_RCTL_VFE ; /* enable vlan filters */
	I350_WRITE_REG(hw, I350_RCTL, vlanctrl);

	/* VFTA - enable all vlan filters */
	for (i = 0; i < I350_VFTA_SIZE; i++) {
		I350_WRITE_REG_ARRAY(hw, I350_VFTA, i, 0xFFFFFFFF);
	}

	/* Enable/Disable MAC Anti-Spoofing */
	i350_vmdq_set_anti_spoofing_pf(hw, FALSE, vf_num);

	return 0;
}

static void
set_rx_mode(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_data *dev_data = dev->data;
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t fctrl, vmolr = I350_VMOLR_BAM | I350_VMOLR_AUPE;
	uint16_t vfn = dev_num_vf(dev);

	/* Check for Promiscuous and All Multicast modes */
	fctrl = I350_READ_REG(hw, I350_RCTL);

	/* set all bits that we expect to always be set */
	fctrl &= ~I350_RCTL_SBP; /* disable store-bad-packets */
	fctrl |= I350_RCTL_BAM;

	/* clear the bits we are changing the status of */
	fctrl &= ~(I350_RCTL_UPE | I350_RCTL_MPE);

	if (dev_data->promiscuous) {
		fctrl |= (I350_RCTL_UPE | I350_RCTL_MPE);
		vmolr |= (I350_VMOLR_ROPE | I350_VMOLR_MPME);
	} else {
		if (dev_data->all_multicast) {
			fctrl |= I350_RCTL_MPE;
			vmolr |= I350_VMOLR_MPME;
		} else {
			vmolr |= I350_VMOLR_ROMPE;
		}
	}


	vmolr |= I350_READ_REG(hw, I350_VMOLR(vfn)) &
		 ~(I350_VMOLR_MPME | I350_VMOLR_ROMPE |
		   I350_VMOLR_ROPE);
	I350_WRITE_REG(hw, I350_VMOLR(vfn), vmolr);


	I350_WRITE_REG(hw, I350_RCTL, fctrl);
}

static inline void
i350_vf_reset_event(struct rte_eth_dev *dev, uint16_t vf)
{
	struct i350_hw *hw =
		I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct i350_vf_info *vfinfo =
		*(I350_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	uint32_t vmolr = I350_READ_REG(hw, I350_VMOLR(vf));

	vmolr |= (I350_VMOLR_ROPE | I350_VMOLR_ROMPE |
			I350_VMOLR_BAM | I350_VMOLR_AUPE);
	I350_WRITE_REG(hw, I350_VMOLR(vf), vmolr);

	I350_WRITE_REG(hw, I350_VMVIR(vf), 0);

	/* reset multicast table array for vf */
	vfinfo[vf].num_vf_mc_hashes = 0;

	/* reset rx mode */
	set_rx_mode(dev);
}

static inline void
i350_vf_reset_msg(struct rte_eth_dev *dev, uint16_t vf)
{
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t reg;

	/* enable transmit and receive for vf */
	reg = I350_READ_REG(hw, I350_VFTE);
	reg |= (reg | (1 << vf));
	I350_WRITE_REG(hw, I350_VFTE, reg);

	reg = I350_READ_REG(hw, I350_VFRE);
	reg |= (reg | (1 << vf));
	I350_WRITE_REG(hw, I350_VFRE, reg);

	i350_vf_reset_event(dev, vf);
}

static int
i350_vf_reset(struct rte_eth_dev *dev, uint16_t vf, uint32_t *msgbuf)
{
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct i350_vf_info *vfinfo =
		*(I350_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	unsigned char *vf_mac = vfinfo[vf].vf_mac_addresses;
	int rar_entry = hw->mac.rar_entry_count - (vf + 1);
	uint8_t *new_mac = (uint8_t *)(&msgbuf[1]);
	uint32_t rah;

	i350_vf_reset_msg(dev, vf);

	hw->mac.ops.rar_set(hw, vf_mac, rar_entry);
	rah = I350_READ_REG(hw, I350_RAH(rar_entry));
	rah |= (0x1 << (vf + I350_RAH_POOLSEL_SHIFT));
	I350_WRITE_REG(hw, I350_RAH(rar_entry), rah);

	/* reply to reset with ack and vf mac address */
	msgbuf[0] = I350_VF_RESET | I350_VT_MSGTYPE_ACK;
	rte_memcpy(new_mac, vf_mac, RTE_ETHER_ADDR_LEN);
	i350_write_mbx(hw, msgbuf, 3, vf);

	return 0;
}

static int
i350_vf_set_mac_addr(struct rte_eth_dev *dev, uint32_t vf, uint32_t *msgbuf)
{
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct i350_vf_info *vfinfo =
		*(I350_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	int rar_entry = hw->mac.rar_entry_count - (vf + 1);
	uint8_t *new_mac = (uint8_t *)(&msgbuf[1]);
	int rah;

	if (rte_is_unicast_ether_addr((struct rte_ether_addr *)new_mac)) {
		if (!rte_is_zero_ether_addr((struct rte_ether_addr *)new_mac))
			rte_memcpy(vfinfo[vf].vf_mac_addresses, new_mac,
				sizeof(vfinfo[vf].vf_mac_addresses));
		hw->mac.ops.rar_set(hw, new_mac, rar_entry);
		rah = I350_READ_REG(hw, I350_RAH(rar_entry));
		rah |= (0x1 << (I350_RAH_POOLSEL_SHIFT + vf));
		I350_WRITE_REG(hw, I350_RAH(rar_entry), rah);
		return 0;
	}
	return -1;
}

static int
i350_vf_set_multicast(struct rte_eth_dev *dev, __rte_unused uint32_t vf, uint32_t *msgbuf)
{
	int i;
	uint32_t vector_bit;
	uint32_t vector_reg;
	uint32_t mta_reg;
	int entries = (msgbuf[0] & I350_VT_MSGINFO_MASK) >>
		I350_VT_MSGINFO_SHIFT;
	uint16_t *hash_list = (uint16_t *)&msgbuf[1];
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct i350_vf_info *vfinfo =
		*(I350_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));

	/* only so many hash values supported */
	entries = RTE_MIN(entries, I350_MAX_VF_MC_ENTRIES);

	/*
	 * salt away the number of multi cast addresses assigned
	 * to this VF for later use to restore when the PF multi cast
	 * list changes
	 */
	vfinfo->num_vf_mc_hashes = (uint16_t)entries;

	/*
	 * VFs are limited to using the MTA hash table for their multicast
	 * addresses
	 */
	for (i = 0; i < entries; i++) {
		vfinfo->vf_mc_hashes[i] = hash_list[i];
	}

	for (i = 0; i < vfinfo->num_vf_mc_hashes; i++) {
		vector_reg = (vfinfo->vf_mc_hashes[i] >> 5) & 0x7F;
		vector_bit = vfinfo->vf_mc_hashes[i] & 0x1F;
		mta_reg = I350_READ_REG_ARRAY(hw, I350_MTA, vector_reg);
		mta_reg |= (1 << vector_bit);
		I350_WRITE_REG_ARRAY(hw, I350_MTA, vector_reg, mta_reg);
	}

	return 0;
}

static int
i350_vf_set_vlan(struct rte_eth_dev *dev, uint32_t vf, uint32_t *msgbuf)
{
	int add, vid;
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct i350_vf_info *vfinfo =
		*(I350_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	uint32_t vid_idx, vid_bit, vfta;

	add = (msgbuf[0] & I350_VT_MSGINFO_MASK)
		>> I350_VT_MSGINFO_SHIFT;
	vid = (msgbuf[1] & I350_VLVF_VLANID_MASK);

	if (add)
		vfinfo[vf].vlan_count++;
	else if (vfinfo[vf].vlan_count)
		vfinfo[vf].vlan_count--;

	vid_idx = (uint32_t)((vid >> I350_VFTA_ENTRY_SHIFT) &
			     I350_VFTA_ENTRY_MASK);
	vid_bit = (uint32_t)(1 << (vid & I350_VFTA_ENTRY_BIT_SHIFT_MASK));
	vfta = I350_READ_REG_ARRAY(hw, I350_VFTA, vid_idx);
	if (add)
		vfta |= vid_bit;
	else
		vfta &= ~vid_bit;

	I350_WRITE_REG_ARRAY(hw, I350_VFTA, vid_idx, vfta);
	I350_WRITE_FLUSH(hw);

	return 0;
}

static int
i350_vf_set_rlpml(struct rte_eth_dev *dev, uint32_t vf, uint32_t *msgbuf)
{
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint16_t rlpml = msgbuf[1] & I350_VMOLR_RLPML_MASK;
	uint32_t max_frame = rlpml + RTE_ETHER_HDR_LEN + RTE_ETHER_CRC_LEN;
	uint32_t vmolr;

	if (max_frame < RTE_ETHER_MIN_LEN ||
			max_frame > RTE_ETHER_MAX_JUMBO_FRAME_LEN)
		return -1;

	vmolr = I350_READ_REG(hw, I350_VMOLR(vf));

	vmolr &= ~I350_VMOLR_RLPML_MASK;
	vmolr |= rlpml;

	/* Enable Long Packet support */
	vmolr |= I350_VMOLR_LPE;

	I350_WRITE_REG(hw, I350_VMOLR(vf), vmolr);
	I350_WRITE_FLUSH(hw);

	return 0;
}

static int
i350_rcv_msg_from_vf(struct rte_eth_dev *dev, uint16_t vf)
{
	uint16_t mbx_size = I350_VFMAILBOX_SIZE;
	uint32_t msgbuf[I350_VFMAILBOX_SIZE];
	int32_t retval;
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	retval = i350_read_mbx(hw, msgbuf, mbx_size, vf);
	if (retval) {
		PMD_INIT_LOG(ERR, "Error mbx recv msg from VF %d", vf);
		return retval;
	}

	/* do nothing with the message already processed */
	if (msgbuf[0] & (I350_VT_MSGTYPE_ACK | I350_VT_MSGTYPE_NACK))
		return retval;

	/* flush the ack before we write any messages back */
	I350_WRITE_FLUSH(hw);

	/* perform VF reset */
	if (msgbuf[0] == I350_VF_RESET) {
		return i350_vf_reset(dev, vf, msgbuf);
	}

	/* check & process VF to PF mailbox message */
	switch ((msgbuf[0] & 0xFFFF)) {
	case I350_VF_SET_MAC_ADDR:
		retval = i350_vf_set_mac_addr(dev, vf, msgbuf);
		break;
	case I350_VF_SET_MULTICAST:
		retval = i350_vf_set_multicast(dev, vf, msgbuf);
		break;
	case I350_VF_SET_LPE:
		retval = i350_vf_set_rlpml(dev, vf, msgbuf);
		break;
	case I350_VF_SET_VLAN:
		retval = i350_vf_set_vlan(dev, vf, msgbuf);
		break;
	default:
		PMD_INIT_LOG(DEBUG, "Unhandled Msg %8.8x",
			     (unsigned) msgbuf[0]);
		retval = I350_ERR_MBX;
		break;
	}

	/* response the VF according to the message process result */
	if (retval)
		msgbuf[0] |= I350_VT_MSGTYPE_NACK;
	else
		msgbuf[0] |= I350_VT_MSGTYPE_ACK;

	msgbuf[0] |= I350_VT_MSGTYPE_CTS;

	i350_write_mbx(hw, msgbuf, 1, vf);

	return retval;
}

static inline void
i350_rcv_ack_from_vf(struct rte_eth_dev *dev, uint16_t vf)
{
	uint32_t msg = I350_VT_MSGTYPE_NACK;
	struct i350_hw *hw =
		I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	i350_write_mbx(hw, &msg, 1, vf);
}

void i350_pf_mbx_process(struct rte_eth_dev *eth_dev)
{
	uint16_t vf;
	struct i350_hw *hw =
		I350_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);

	for (vf = 0; vf < dev_num_vf(eth_dev); vf++) {
		/* check & process vf function level reset */
		if (!i350_check_for_rst(hw, vf))
			i350_vf_reset_event(eth_dev, vf);

		/* check & process vf mailbox messages */
		if (!i350_check_for_msg(hw, vf))
			i350_rcv_msg_from_vf(eth_dev, vf);

		/* check & process acks from vf */
		if (!i350_check_for_ack(hw, vf))
			i350_rcv_ack_from_vf(eth_dev, vf);
	}
}
