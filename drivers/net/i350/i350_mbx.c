/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2001-2020 Intel Corporation
 */

#include "i350_mbx.h"

/**
 *  i350_null_mbx_check_for_flag - No-op function, return 0
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to read
 **/
STATIC s32 i350_null_mbx_check_for_flag(struct i350_hw I350_UNUSEDARG *hw,
					 u16 I350_UNUSEDARG mbx_id)
{
	DEBUGFUNC("i350_null_mbx_check_flag");
	UNREFERENCED_2PARAMETER(hw, mbx_id);

	return I350_SUCCESS;
}

/**
 *  i350_null_mbx_transact - No-op function, return 0
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to read
 **/
STATIC s32 i350_null_mbx_transact(struct i350_hw I350_UNUSEDARG *hw,
				   u32 I350_UNUSEDARG *msg,
				   u16 I350_UNUSEDARG size,
				   u16 I350_UNUSEDARG mbx_id)
{
	DEBUGFUNC("i350_null_mbx_rw_msg");
	UNREFERENCED_4PARAMETER(hw, msg, size, mbx_id);

	return I350_SUCCESS;
}

/**
 *  i350_read_mbx - Reads a message from the mailbox
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to read
 *
 *  returns SUCCESS if it successfully read message from buffer
 **/
s32 i350_read_mbx(struct i350_hw *hw, u32 *msg, u16 size, u16 mbx_id)
{
	struct i350_mbx_info *mbx = &hw->mbx;
	s32 ret_val = -I350_ERR_MBX;

	DEBUGFUNC("i350_read_mbx");

	/* limit read to size of mailbox */
	if (size > mbx->size)
		size = mbx->size;

	if (mbx->ops.read)
		ret_val = mbx->ops.read(hw, msg, size, mbx_id);

	return ret_val;
}

/**
 *  i350_write_mbx - Write a message to the mailbox
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully copied message into the buffer
 **/
s32 i350_write_mbx(struct i350_hw *hw, u32 *msg, u16 size, u16 mbx_id)
{
	struct i350_mbx_info *mbx = &hw->mbx;
	s32 ret_val = I350_SUCCESS;

	DEBUGFUNC("i350_write_mbx");

	if (size > mbx->size)
		ret_val = -I350_ERR_MBX;

	else if (mbx->ops.write)
		ret_val = mbx->ops.write(hw, msg, size, mbx_id);

	return ret_val;
}

/**
 *  i350_check_for_msg - checks to see if someone sent us mail
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns SUCCESS if the Status bit was found or else ERR_MBX
 **/
s32 i350_check_for_msg(struct i350_hw *hw, u16 mbx_id)
{
	struct i350_mbx_info *mbx = &hw->mbx;
	s32 ret_val = -I350_ERR_MBX;

	DEBUGFUNC("i350_check_for_msg");

	if (mbx->ops.check_for_msg)
		ret_val = mbx->ops.check_for_msg(hw, mbx_id);

	return ret_val;
}

/**
 *  i350_check_for_ack - checks to see if someone sent us ACK
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns SUCCESS if the Status bit was found or else ERR_MBX
 **/
s32 i350_check_for_ack(struct i350_hw *hw, u16 mbx_id)
{
	struct i350_mbx_info *mbx = &hw->mbx;
	s32 ret_val = -I350_ERR_MBX;

	DEBUGFUNC("i350_check_for_ack");

	if (mbx->ops.check_for_ack)
		ret_val = mbx->ops.check_for_ack(hw, mbx_id);

	return ret_val;
}

/**
 *  i350_check_for_rst - checks to see if other side has reset
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns SUCCESS if the Status bit was found or else ERR_MBX
 **/
s32 i350_check_for_rst(struct i350_hw *hw, u16 mbx_id)
{
	struct i350_mbx_info *mbx = &hw->mbx;
	s32 ret_val = -I350_ERR_MBX;

	DEBUGFUNC("i350_check_for_rst");

	if (mbx->ops.check_for_rst)
		ret_val = mbx->ops.check_for_rst(hw, mbx_id);

	return ret_val;
}

/**
 *  i350_poll_for_msg - Wait for message notification
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully received a message notification
 **/
STATIC s32 i350_poll_for_msg(struct i350_hw *hw, u16 mbx_id)
{
	struct i350_mbx_info *mbx = &hw->mbx;
	int countdown = mbx->timeout;

	DEBUGFUNC("i350_poll_for_msg");

	if (!countdown || !mbx->ops.check_for_msg)
		goto out;

	while (countdown && mbx->ops.check_for_msg(hw, mbx_id)) {
		countdown--;
		if (!countdown)
			break;
		usec_delay(mbx->usec_delay);
	}

	/* if we failed, all future posted messages fail until reset */
	if (!countdown)
		mbx->timeout = 0;
out:
	return countdown ? I350_SUCCESS : -I350_ERR_MBX;
}

/**
 *  i350_poll_for_ack - Wait for message acknowledgement
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully received a message acknowledgement
 **/
STATIC s32 i350_poll_for_ack(struct i350_hw *hw, u16 mbx_id)
{
	struct i350_mbx_info *mbx = &hw->mbx;
	int countdown = mbx->timeout;

	DEBUGFUNC("i350_poll_for_ack");

	if (!countdown || !mbx->ops.check_for_ack)
		goto out;

	while (countdown && mbx->ops.check_for_ack(hw, mbx_id)) {
		countdown--;
		if (!countdown)
			break;
		usec_delay(mbx->usec_delay);
	}

	/* if we failed, all future posted messages fail until reset */
	if (!countdown)
		mbx->timeout = 0;
out:
	return countdown ? I350_SUCCESS : -I350_ERR_MBX;
}

/**
 *  i350_read_posted_mbx - Wait for message notification and receive message
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully received a message notification and
 *  copied it into the receive buffer.
 **/
s32 i350_read_posted_mbx(struct i350_hw *hw, u32 *msg, u16 size, u16 mbx_id)
{
	struct i350_mbx_info *mbx = &hw->mbx;
	s32 ret_val = -I350_ERR_MBX;

	DEBUGFUNC("i350_read_posted_mbx");

	if (!mbx->ops.read)
		goto out;

	ret_val = i350_poll_for_msg(hw, mbx_id);

	/* if ack received read message, otherwise we timed out */
	if (!ret_val)
		ret_val = mbx->ops.read(hw, msg, size, mbx_id);
out:
	return ret_val;
}

/**
 *  i350_write_posted_mbx - Write a message to the mailbox, wait for ack
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully copied message into the buffer and
 *  received an ack to that message within delay * timeout period
 **/
s32 i350_write_posted_mbx(struct i350_hw *hw, u32 *msg, u16 size, u16 mbx_id)
{
	struct i350_mbx_info *mbx = &hw->mbx;
	s32 ret_val = -I350_ERR_MBX;

	DEBUGFUNC("i350_write_posted_mbx");

	/* exit if either we can't write or there isn't a defined timeout */
	if (!mbx->ops.write || !mbx->timeout)
		goto out;

	/* send msg */
	ret_val = mbx->ops.write(hw, msg, size, mbx_id);

	/* if msg sent wait until we receive an ack */
	if (!ret_val)
		ret_val = i350_poll_for_ack(hw, mbx_id);
out:
	return ret_val;
}

/**
 *  i350_init_mbx_ops_generic - Initialize mbx function pointers
 *  @hw: pointer to the HW structure
 *
 *  Sets the function pointers to no-op functions
 **/
void i350_init_mbx_ops_generic(struct i350_hw *hw)
{
	struct i350_mbx_info *mbx = &hw->mbx;
	mbx->ops.init_params = i350_null_ops_generic;
	mbx->ops.read = i350_null_mbx_transact;
	mbx->ops.write = i350_null_mbx_transact;
	mbx->ops.check_for_msg = i350_null_mbx_check_for_flag;
	mbx->ops.check_for_ack = i350_null_mbx_check_for_flag;
	mbx->ops.check_for_rst = i350_null_mbx_check_for_flag;
	mbx->ops.read_posted = i350_read_posted_mbx;
	mbx->ops.write_posted = i350_write_posted_mbx;
}

/**
 *  i350_read_v2p_mailbox - read v2p mailbox
 *  @hw: pointer to the HW structure
 *
 *  This function is used to read the v2p mailbox without losing the read to
 *  clear status bits.
 **/
STATIC u32 i350_read_v2p_mailbox(struct i350_hw *hw)
{
	u32 v2p_mailbox = I350_READ_REG(hw, I350_V2PMAILBOX(0));

	v2p_mailbox |= hw->dev_spec.vf.v2p_mailbox;
	hw->dev_spec.vf.v2p_mailbox |= v2p_mailbox & I350_V2PMAILBOX_R2C_BITS;

	return v2p_mailbox;
}

/**
 *  i350_check_for_bit_vf - Determine if a status bit was set
 *  @hw: pointer to the HW structure
 *  @mask: bitmask for bits to be tested and cleared
 *
 *  This function is used to check for the read to clear bits within
 *  the V2P mailbox.
 **/
STATIC s32 i350_check_for_bit_vf(struct i350_hw *hw, u32 mask)
{
	u32 v2p_mailbox = i350_read_v2p_mailbox(hw);
	s32 ret_val = -I350_ERR_MBX;

	if (v2p_mailbox & mask)
		ret_val = I350_SUCCESS;

	hw->dev_spec.vf.v2p_mailbox &= ~mask;

	return ret_val;
}

/**
 *  i350_check_for_msg_vf - checks to see if the PF has sent mail
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns SUCCESS if the PF has set the Status bit or else ERR_MBX
 **/
STATIC s32 i350_check_for_msg_vf(struct i350_hw *hw,
				  u16 I350_UNUSEDARG mbx_id)
{
	s32 ret_val = -I350_ERR_MBX;

	UNREFERENCED_1PARAMETER(mbx_id);
	DEBUGFUNC("i350_check_for_msg_vf");

	if (!i350_check_for_bit_vf(hw, I350_V2PMAILBOX_PFSTS)) {
		ret_val = I350_SUCCESS;
		hw->mbx.stats.reqs++;
	}

	return ret_val;
}

/**
 *  i350_check_for_ack_vf - checks to see if the PF has ACK'd
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns SUCCESS if the PF has set the ACK bit or else ERR_MBX
 **/
STATIC s32 i350_check_for_ack_vf(struct i350_hw *hw,
				  u16 I350_UNUSEDARG mbx_id)
{
	s32 ret_val = -I350_ERR_MBX;

	UNREFERENCED_1PARAMETER(mbx_id);
	DEBUGFUNC("i350_check_for_ack_vf");

	if (!i350_check_for_bit_vf(hw, I350_V2PMAILBOX_PFACK)) {
		ret_val = I350_SUCCESS;
		hw->mbx.stats.acks++;
	}

	return ret_val;
}

/**
 *  i350_check_for_rst_vf - checks to see if the PF has reset
 *  @hw: pointer to the HW structure
 *  @mbx_id: id of mailbox to check
 *
 *  returns true if the PF has set the reset done bit or else false
 **/
STATIC s32 i350_check_for_rst_vf(struct i350_hw *hw,
				  u16 I350_UNUSEDARG mbx_id)
{
	s32 ret_val = -I350_ERR_MBX;

	UNREFERENCED_1PARAMETER(mbx_id);
	DEBUGFUNC("i350_check_for_rst_vf");

	if (!i350_check_for_bit_vf(hw, (I350_V2PMAILBOX_RSTD |
					 I350_V2PMAILBOX_RSTI))) {
		ret_val = I350_SUCCESS;
		hw->mbx.stats.rsts++;
	}

	return ret_val;
}

/**
 *  i350_obtain_mbx_lock_vf - obtain mailbox lock
 *  @hw: pointer to the HW structure
 *
 *  return SUCCESS if we obtained the mailbox lock
 **/
STATIC s32 i350_obtain_mbx_lock_vf(struct i350_hw *hw)
{
	s32 ret_val = -I350_ERR_MBX;
	int count = 10;

	DEBUGFUNC("i350_obtain_mbx_lock_vf");

	do {
		/* Take ownership of the buffer */
		I350_WRITE_REG(hw, I350_V2PMAILBOX(0), I350_V2PMAILBOX_VFU);

		/* reserve mailbox for vf use */
		if (i350_read_v2p_mailbox(hw) & I350_V2PMAILBOX_VFU) {
			ret_val = I350_SUCCESS;
			break;
		}
		usec_delay(1000);
	} while (count-- > 0);

	return ret_val;
}

/**
 *  i350_write_mbx_vf - Write a message to the mailbox
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to write
 *
 *  returns SUCCESS if it successfully copied message into the buffer
 **/
STATIC s32 i350_write_mbx_vf(struct i350_hw *hw, u32 *msg, u16 size,
			      u16 I350_UNUSEDARG mbx_id)
{
	s32 ret_val;
	u16 i;

	UNREFERENCED_1PARAMETER(mbx_id);

	DEBUGFUNC("i350_write_mbx_vf");

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = i350_obtain_mbx_lock_vf(hw);
	if (ret_val)
		goto out_no_write;

	/* flush msg and acks as we are overwriting the message buffer */
	i350_check_for_msg_vf(hw, 0);
	i350_check_for_ack_vf(hw, 0);

	/* copy the caller specified message to the mailbox memory buffer */
	for (i = 0; i < size; i++)
		I350_WRITE_REG_ARRAY(hw, I350_VMBMEM(0), i, msg[i]);

	/* update stats */
	hw->mbx.stats.msgs_tx++;

	/* Drop VFU and interrupt the PF to tell it a message has been sent */
	I350_WRITE_REG(hw, I350_V2PMAILBOX(0), I350_V2PMAILBOX_REQ);

out_no_write:
	return ret_val;
}

/**
 *  i350_read_mbx_vf - Reads a message from the inbox intended for vf
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @mbx_id: id of mailbox to read
 *
 *  returns SUCCESS if it successfully read message from buffer
 **/
STATIC s32 i350_read_mbx_vf(struct i350_hw *hw, u32 *msg, u16 size,
			     u16 I350_UNUSEDARG mbx_id)
{
	s32 ret_val = I350_SUCCESS;
	u16 i;

	DEBUGFUNC("i350_read_mbx_vf");
	UNREFERENCED_1PARAMETER(mbx_id);

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = i350_obtain_mbx_lock_vf(hw);
	if (ret_val)
		goto out_no_read;

	/* copy the message from the mailbox memory buffer */
	for (i = 0; i < size; i++)
		msg[i] = I350_READ_REG_ARRAY(hw, I350_VMBMEM(0), i);

	/* Acknowledge receipt and release mailbox, then we're done */
	I350_WRITE_REG(hw, I350_V2PMAILBOX(0), I350_V2PMAILBOX_ACK);

	/* update stats */
	hw->mbx.stats.msgs_rx++;

out_no_read:
	return ret_val;
}

/**
 *  i350_init_mbx_params_vf - set initial values for vf mailbox
 *  @hw: pointer to the HW structure
 *
 *  Initializes the hw->mbx struct to correct values for vf mailbox
 */
s32 i350_init_mbx_params_vf(struct i350_hw *hw)
{
	struct i350_mbx_info *mbx = &hw->mbx;

	/* start mailbox as timed out and let the reset_hw call set the timeout
	 * value to begin communications */
	mbx->timeout = 0;
	mbx->usec_delay = I350_VF_MBX_INIT_DELAY;

	mbx->size = I350_VFMAILBOX_SIZE;

	mbx->ops.read = i350_read_mbx_vf;
	mbx->ops.write = i350_write_mbx_vf;
	mbx->ops.read_posted = i350_read_posted_mbx;
	mbx->ops.write_posted = i350_write_posted_mbx;
	mbx->ops.check_for_msg = i350_check_for_msg_vf;
	mbx->ops.check_for_ack = i350_check_for_ack_vf;
	mbx->ops.check_for_rst = i350_check_for_rst_vf;

	mbx->stats.msgs_tx = 0;
	mbx->stats.msgs_rx = 0;
	mbx->stats.reqs = 0;
	mbx->stats.acks = 0;
	mbx->stats.rsts = 0;

	return I350_SUCCESS;
}

STATIC s32 i350_check_for_bit_pf(struct i350_hw *hw, u32 mask)
{
	u32 mbvficr = I350_READ_REG(hw, I350_MBVFICR);
	s32 ret_val = -I350_ERR_MBX;

	if (mbvficr & mask) {
		ret_val = I350_SUCCESS;
		I350_WRITE_REG(hw, I350_MBVFICR, mask);
	}

	return ret_val;
}

/**
 *  i350_check_for_msg_pf - checks to see if the VF has sent mail
 *  @hw: pointer to the HW structure
 *  @vf_number: the VF index
 *
 *  returns SUCCESS if the VF has set the Status bit or else ERR_MBX
 **/
STATIC s32 i350_check_for_msg_pf(struct i350_hw *hw, u16 vf_number)
{
	s32 ret_val = -I350_ERR_MBX;

	DEBUGFUNC("i350_check_for_msg_pf");

	if (!i350_check_for_bit_pf(hw, I350_MBVFICR_VFREQ_VF1 << vf_number)) {
		ret_val = I350_SUCCESS;
		hw->mbx.stats.reqs++;
	}

	return ret_val;
}

/**
 *  i350_check_for_ack_pf - checks to see if the VF has ACKed
 *  @hw: pointer to the HW structure
 *  @vf_number: the VF index
 *
 *  returns SUCCESS if the VF has set the Status bit or else ERR_MBX
 **/
STATIC s32 i350_check_for_ack_pf(struct i350_hw *hw, u16 vf_number)
{
	s32 ret_val = -I350_ERR_MBX;

	DEBUGFUNC("i350_check_for_ack_pf");

	if (!i350_check_for_bit_pf(hw, I350_MBVFICR_VFACK_VF1 << vf_number)) {
		ret_val = I350_SUCCESS;
		hw->mbx.stats.acks++;
	}

	return ret_val;
}

/**
 *  i350_check_for_rst_pf - checks to see if the VF has reset
 *  @hw: pointer to the HW structure
 *  @vf_number: the VF index
 *
 *  returns SUCCESS if the VF has set the Status bit or else ERR_MBX
 **/
STATIC s32 i350_check_for_rst_pf(struct i350_hw *hw, u16 vf_number)
{
	u32 vflre = I350_READ_REG(hw, I350_VFLRE);
	s32 ret_val = -I350_ERR_MBX;

	DEBUGFUNC("i350_check_for_rst_pf");

	if (vflre & (1 << vf_number)) {
		ret_val = I350_SUCCESS;
		I350_WRITE_REG(hw, I350_VFLRE, (1 << vf_number));
		hw->mbx.stats.rsts++;
	}

	return ret_val;
}

/**
 *  i350_obtain_mbx_lock_pf - obtain mailbox lock
 *  @hw: pointer to the HW structure
 *  @vf_number: the VF index
 *
 *  return SUCCESS if we obtained the mailbox lock
 **/
STATIC s32 i350_obtain_mbx_lock_pf(struct i350_hw *hw, u16 vf_number)
{
	s32 ret_val = -I350_ERR_MBX;
	u32 p2v_mailbox;
	int count = 10;

	DEBUGFUNC("i350_obtain_mbx_lock_pf");

	do {
		/* Take ownership of the buffer */
		I350_WRITE_REG(hw, I350_P2VMAILBOX(vf_number),
				I350_P2VMAILBOX_PFU);

		/* reserve mailbox for pf use */
		p2v_mailbox = I350_READ_REG(hw, I350_P2VMAILBOX(vf_number));
		if (p2v_mailbox & I350_P2VMAILBOX_PFU) {
			ret_val = I350_SUCCESS;
			break;
		}
		usec_delay(1000);
	} while (count-- > 0);

	return ret_val;

}

/**
 *  i350_write_mbx_pf - Places a message in the mailbox
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @vf_number: the VF index
 *
 *  returns SUCCESS if it successfully copied message into the buffer
 **/
STATIC s32 i350_write_mbx_pf(struct i350_hw *hw, u32 *msg, u16 size,
			      u16 vf_number)
{
	s32 ret_val;
	u16 i;

	DEBUGFUNC("i350_write_mbx_pf");

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = i350_obtain_mbx_lock_pf(hw, vf_number);
	if (ret_val)
		goto out_no_write;

	/* flush msg and acks as we are overwriting the message buffer */
	i350_check_for_msg_pf(hw, vf_number);
	i350_check_for_ack_pf(hw, vf_number);

	/* copy the caller specified message to the mailbox memory buffer */
	for (i = 0; i < size; i++)
		I350_WRITE_REG_ARRAY(hw, I350_VMBMEM(vf_number), i, msg[i]);

	/* Interrupt VF to tell it a message has been sent and release buffer*/
	I350_WRITE_REG(hw, I350_P2VMAILBOX(vf_number), I350_P2VMAILBOX_STS);

	/* update stats */
	hw->mbx.stats.msgs_tx++;

out_no_write:
	return ret_val;

}

/**
 *  i350_read_mbx_pf - Read a message from the mailbox
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *  @vf_number: the VF index
 *
 *  This function copies a message from the mailbox buffer to the caller's
 *  memory buffer.  The presumption is that the caller knows that there was
 *  a message due to a VF request so no polling for message is needed.
 **/
STATIC s32 i350_read_mbx_pf(struct i350_hw *hw, u32 *msg, u16 size,
			     u16 vf_number)
{
	s32 ret_val;
	u16 i;

	DEBUGFUNC("i350_read_mbx_pf");

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = i350_obtain_mbx_lock_pf(hw, vf_number);
	if (ret_val)
		goto out_no_read;

	/* copy the message to the mailbox memory buffer */
	for (i = 0; i < size; i++)
		msg[i] = I350_READ_REG_ARRAY(hw, I350_VMBMEM(vf_number), i);

	/* Acknowledge the message and release buffer */
	I350_WRITE_REG(hw, I350_P2VMAILBOX(vf_number), I350_P2VMAILBOX_ACK);

	/* update stats */
	hw->mbx.stats.msgs_rx++;

out_no_read:
	return ret_val;
}

/**
 *  i350_init_mbx_params_pf - set initial values for pf mailbox
 *  @hw: pointer to the HW structure
 *
 *  Initializes the hw->mbx struct to correct values for pf mailbox
 */
s32 i350_init_mbx_params_pf(struct i350_hw *hw)
{
	struct i350_mbx_info *mbx = &hw->mbx;

	switch (hw->mac.type) {
	case i350_82576:
	case i350_i350:
	case i350_i354:
		mbx->timeout = 0;
		mbx->usec_delay = 0;

		mbx->size = I350_VFMAILBOX_SIZE;

		mbx->ops.read = i350_read_mbx_pf;
		mbx->ops.write = i350_write_mbx_pf;
		mbx->ops.read_posted = i350_read_posted_mbx;
		mbx->ops.write_posted = i350_write_posted_mbx;
		mbx->ops.check_for_msg = i350_check_for_msg_pf;
		mbx->ops.check_for_ack = i350_check_for_ack_pf;
		mbx->ops.check_for_rst = i350_check_for_rst_pf;

		mbx->stats.msgs_tx = 0;
		mbx->stats.msgs_rx = 0;
		mbx->stats.reqs = 0;
		mbx->stats.acks = 0;
		mbx->stats.rsts = 0;
		/* Fall through */
	default:
		return I350_SUCCESS;
	}
}

