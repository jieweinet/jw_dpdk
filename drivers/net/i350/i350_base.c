/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2001-2020 Intel Corporation
 */

#include "i350_hw.h"
#include "i350_82575.h"
#include "i350_mac.h"
#include "i350_base.h"
#include "i350_manage.h"

/**
 *  i350_acquire_phy_base - Acquire rights to access PHY
 *  @hw: pointer to the HW structure
 *
 *  Acquire access rights to the correct PHY.
 **/
s32 i350_acquire_phy_base(struct i350_hw *hw)
{
	u16 mask = I350_SWFW_PHY0_SM;

	DEBUGFUNC("i350_acquire_phy_base");

	if (hw->bus.func == I350_FUNC_1)
		mask = I350_SWFW_PHY1_SM;
	else if (hw->bus.func == I350_FUNC_2)
		mask = I350_SWFW_PHY2_SM;
	else if (hw->bus.func == I350_FUNC_3)
		mask = I350_SWFW_PHY3_SM;

	return hw->mac.ops.acquire_swfw_sync(hw, mask);
}

/**
 *  i350_release_phy_base - Release rights to access PHY
 *  @hw: pointer to the HW structure
 *
 *  A wrapper to release access rights to the correct PHY.
 **/
void i350_release_phy_base(struct i350_hw *hw)
{
	u16 mask = I350_SWFW_PHY0_SM;

	DEBUGFUNC("i350_release_phy_base");

	if (hw->bus.func == I350_FUNC_1)
		mask = I350_SWFW_PHY1_SM;
	else if (hw->bus.func == I350_FUNC_2)
		mask = I350_SWFW_PHY2_SM;
	else if (hw->bus.func == I350_FUNC_3)
		mask = I350_SWFW_PHY3_SM;

	hw->mac.ops.release_swfw_sync(hw, mask);
}

/**
 *  i350_init_hw_base - Initialize hardware
 *  @hw: pointer to the HW structure
 *
 *  This inits the hardware readying it for operation.
 **/
s32 i350_init_hw_base(struct i350_hw *hw)
{
	struct i350_mac_info *mac = &hw->mac;
	s32 ret_val;
	u16 i, rar_count = mac->rar_entry_count;

	DEBUGFUNC("i350_init_hw_base");

	/* Setup the receive address */
	i350_init_rx_addrs_generic(hw, rar_count);

	/* Zero out the Multicast HASH table */
	DEBUGOUT("Zeroing the MTA\n");
	for (i = 0; i < mac->mta_reg_count; i++)
		I350_WRITE_REG_ARRAY(hw, I350_MTA, i, 0);

	/* Zero out the Unicast HASH table */
	DEBUGOUT("Zeroing the UTA\n");
	for (i = 0; i < mac->uta_reg_count; i++)
		I350_WRITE_REG_ARRAY(hw, I350_UTA, i, 0);

	/* Setup link and flow control */
	ret_val = mac->ops.setup_link(hw);

	/* Clear all of the statistics registers (clear on read).  It is
	 * important that we do this after we have tried to establish link
	 * because the symbol error count will increment wildly if there
	 * is no link.
	 */
	i350_clear_hw_cntrs_base_generic(hw);

	return ret_val;
}

/**
 * i350_power_down_phy_copper_base - Remove link during PHY power down
 * @hw: pointer to the HW structure
 *
 * In the case of a PHY power down to save power, or to turn off link during a
 * driver unload, or wake on lan is not enabled, remove the link.
 **/
void i350_power_down_phy_copper_base(struct i350_hw *hw)
{
	struct i350_phy_info *phy = &hw->phy;

	if (!(phy->ops.check_reset_block))
		return;

	/* If the management interface is not enabled, then power down */
	if (!(i350_enable_mng_pass_thru(hw) || phy->ops.check_reset_block(hw)))
		i350_power_down_phy_copper(hw);
}

/**
 *  i350_rx_fifo_flush_base - Clean Rx FIFO after Rx enable
 *  @hw: pointer to the HW structure
 *
 *  After Rx enable, if manageability is enabled then there is likely some
 *  bad data at the start of the FIFO and possibly in the DMA FIFO.  This
 *  function clears the FIFOs and flushes any packets that came in as Rx was
 *  being enabled.
 **/
void i350_rx_fifo_flush_base(struct i350_hw *hw)
{
	u32 rctl, rlpml, rxdctl[4], rfctl, temp_rctl, rx_enabled;
	int i, ms_wait;

	DEBUGFUNC("i350_rx_fifo_flush_base");

	/* disable IPv6 options as per hardware errata */
	rfctl = I350_READ_REG(hw, I350_RFCTL);
	rfctl |= I350_RFCTL_IPV6_EX_DIS;
	I350_WRITE_REG(hw, I350_RFCTL, rfctl);

	if (!(I350_READ_REG(hw, I350_MANC) & I350_MANC_RCV_TCO_EN))
		return;

	/* Disable all Rx queues */
	for (i = 0; i < 4; i++) {
		rxdctl[i] = I350_READ_REG(hw, I350_RXDCTL(i));
		I350_WRITE_REG(hw, I350_RXDCTL(i),
				rxdctl[i] & ~I350_RXDCTL_QUEUE_ENABLE);
	}
	/* Poll all queues to verify they have shut down */
	for (ms_wait = 0; ms_wait < 10; ms_wait++) {
		msec_delay(1);
		rx_enabled = 0;
		for (i = 0; i < 4; i++)
			rx_enabled |= I350_READ_REG(hw, I350_RXDCTL(i));
		if (!(rx_enabled & I350_RXDCTL_QUEUE_ENABLE))
			break;
	}

	if (ms_wait == 10)
		DEBUGOUT("Queue disable timed out after 10ms\n");

	/* Clear RLPML, RCTL.SBP, RFCTL.LEF, and set RCTL.LPE so that all
	 * incoming packets are rejected.  Set enable and wait 2ms so that
	 * any packet that was coming in as RCTL.EN was set is flushed
	 */
	I350_WRITE_REG(hw, I350_RFCTL, rfctl & ~I350_RFCTL_LEF);

	rlpml = I350_READ_REG(hw, I350_RLPML);
	I350_WRITE_REG(hw, I350_RLPML, 0);

	rctl = I350_READ_REG(hw, I350_RCTL);
	temp_rctl = rctl & ~(I350_RCTL_EN | I350_RCTL_SBP);
	temp_rctl |= I350_RCTL_LPE;

	I350_WRITE_REG(hw, I350_RCTL, temp_rctl);
	I350_WRITE_REG(hw, I350_RCTL, temp_rctl | I350_RCTL_EN);
	I350_WRITE_FLUSH(hw);
	msec_delay(2);

	/* Enable Rx queues that were previously enabled and restore our
	 * previous state
	 */
	for (i = 0; i < 4; i++)
		I350_WRITE_REG(hw, I350_RXDCTL(i), rxdctl[i]);
	I350_WRITE_REG(hw, I350_RCTL, rctl);
	I350_WRITE_FLUSH(hw);

	I350_WRITE_REG(hw, I350_RLPML, rlpml);
	I350_WRITE_REG(hw, I350_RFCTL, rfctl);

	/* Flush receive errors generated by workaround */
	I350_READ_REG(hw, I350_ROC);
	I350_READ_REG(hw, I350_RNBC);
	I350_READ_REG(hw, I350_MPC);
}
