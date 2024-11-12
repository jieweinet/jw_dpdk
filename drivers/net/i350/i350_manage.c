/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2001-2020 Intel Corporation
 */

#include "i350_api.h"
#include "i350_manage.h"

/**
 *  i350_calculate_checksum - Calculate checksum for buffer
 *  @buffer: pointer to EEPROM
 *  @length: size of EEPROM to calculate a checksum for
 *
 *  Calculates the checksum for some buffer on a specified length.  The
 *  checksum calculated is returned.
 **/
u8 i350_calculate_checksum(u8 *buffer, u32 length)
{
	u32 i;
	u8 sum = 0;

	DEBUGFUNC("i350_calculate_checksum");

	if (!buffer)
		return 0;

	for (i = 0; i < length; i++)
		sum += buffer[i];

	return (u8) (0 - sum);
}

/**
 *  i350_mng_enable_host_if_generic - Checks host interface is enabled
 *  @hw: pointer to the HW structure
 *
 *  Returns I350_success upon success, else I350_ERR_HOST_INTERFACE_COMMAND
 *
 *  This function checks whether the HOST IF is enabled for command operation
 *  and also checks whether the previous command is completed.  It busy waits
 *  in case of previous command is not completed.
 **/
s32 i350_mng_enable_host_if_generic(struct i350_hw *hw)
{
	u32 hicr;
	u8 i;

	DEBUGFUNC("i350_mng_enable_host_if_generic");

	if (!hw->mac.arc_subsystem_valid) {
		DEBUGOUT("ARC subsystem not valid.\n");
		return -I350_ERR_HOST_INTERFACE_COMMAND;
	}

	/* Check that the host interface is enabled. */
	hicr = I350_READ_REG(hw, I350_HICR);
	if (!(hicr & I350_HICR_EN)) {
		DEBUGOUT("I350_HOST_EN bit disabled.\n");
		return -I350_ERR_HOST_INTERFACE_COMMAND;
	}
	/* check the previous command is completed */
	for (i = 0; i < I350_MNG_DHCP_COMMAND_TIMEOUT; i++) {
		hicr = I350_READ_REG(hw, I350_HICR);
		if (!(hicr & I350_HICR_C))
			break;
		msec_delay_irq(1);
	}

	if (i == I350_MNG_DHCP_COMMAND_TIMEOUT) {
		DEBUGOUT("Previous command timeout failed .\n");
		return -I350_ERR_HOST_INTERFACE_COMMAND;
	}

	return I350_SUCCESS;
}

/**
 *  i350_check_mng_mode_generic - Generic check management mode
 *  @hw: pointer to the HW structure
 *
 *  Reads the firmware semaphore register and returns true (>0) if
 *  manageability is enabled, else false (0).
 **/
bool i350_check_mng_mode_generic(struct i350_hw *hw)
{
	u32 fwsm = I350_READ_REG(hw, I350_FWSM);

	DEBUGFUNC("i350_check_mng_mode_generic");


	return (fwsm & I350_FWSM_MODE_MASK) ==
		(I350_MNG_IAMT_MODE << I350_FWSM_MODE_SHIFT);
}

/**
 *  i350_enable_tx_pkt_filtering_generic - Enable packet filtering on Tx
 *  @hw: pointer to the HW structure
 *
 *  Enables packet filtering on transmit packets if manageability is enabled
 *  and host interface is enabled.
 **/
bool i350_enable_tx_pkt_filtering_generic(struct i350_hw *hw)
{
	struct i350_host_mng_dhcp_cookie *hdr = &hw->mng_cookie;
	u32 *buffer = (u32 *)&hw->mng_cookie;
	u32 offset;
	s32 ret_val, hdr_csum, csum;
	u8 i, len;

	DEBUGFUNC("i350_enable_tx_pkt_filtering_generic");

	hw->mac.tx_pkt_filtering = true;

	/* No manageability, no filtering */
	if (!hw->mac.ops.check_mng_mode(hw)) {
		hw->mac.tx_pkt_filtering = false;
		return hw->mac.tx_pkt_filtering;
	}

	/* If we can't read from the host interface for whatever
	 * reason, disable filtering.
	 */
	ret_val = i350_mng_enable_host_if_generic(hw);
	if (ret_val != I350_SUCCESS) {
		hw->mac.tx_pkt_filtering = false;
		return hw->mac.tx_pkt_filtering;
	}

	/* Read in the header.  Length and offset are in dwords. */
	len    = I350_MNG_DHCP_COOKIE_LENGTH >> 2;
	offset = I350_MNG_DHCP_COOKIE_OFFSET >> 2;
	for (i = 0; i < len; i++)
		*(buffer + i) = I350_READ_REG_ARRAY_DWORD(hw, I350_HOST_IF,
							   offset + i);
	hdr_csum = hdr->checksum;
	hdr->checksum = 0;
	csum = i350_calculate_checksum((u8 *)hdr,
					I350_MNG_DHCP_COOKIE_LENGTH);
	/* If either the checksums or signature don't match, then
	 * the cookie area isn't considered valid, in which case we
	 * take the safe route of assuming Tx filtering is enabled.
	 */
	if ((hdr_csum != csum) || (hdr->signature != I350_IAMT_SIGNATURE)) {
		hw->mac.tx_pkt_filtering = true;
		return hw->mac.tx_pkt_filtering;
	}

	/* Cookie area is valid, make the final check for filtering. */
	if (!(hdr->status & I350_MNG_DHCP_COOKIE_STATUS_PARSING))
		hw->mac.tx_pkt_filtering = false;

	return hw->mac.tx_pkt_filtering;
}

/**
 *  i350_mng_write_cmd_header_generic - Writes manageability command header
 *  @hw: pointer to the HW structure
 *  @hdr: pointer to the host interface command header
 *
 *  Writes the command header after does the checksum calculation.
 **/
s32 i350_mng_write_cmd_header_generic(struct i350_hw *hw,
				      struct i350_host_mng_command_header *hdr)
{
	u16 i, length = sizeof(struct i350_host_mng_command_header);

	DEBUGFUNC("i350_mng_write_cmd_header_generic");

	/* Write the whole command header structure with new checksum. */

	hdr->checksum = i350_calculate_checksum((u8 *)hdr, length);

	length >>= 2;
	/* Write the relevant command block into the ram area. */
	for (i = 0; i < length; i++) {
		I350_WRITE_REG_ARRAY_DWORD(hw, I350_HOST_IF, i,
					    *((u32 *) hdr + i));
		I350_WRITE_FLUSH(hw);
	}

	return I350_SUCCESS;
}

/**
 *  i350_mng_host_if_write_generic - Write to the manageability host interface
 *  @hw: pointer to the HW structure
 *  @buffer: pointer to the host interface buffer
 *  @length: size of the buffer
 *  @offset: location in the buffer to write to
 *  @sum: sum of the data (not checksum)
 *
 *  This function writes the buffer content at the offset given on the host if.
 *  It also does alignment considerations to do the writes in most efficient
 *  way.  Also fills up the sum of the buffer in *buffer parameter.
 **/
s32 i350_mng_host_if_write_generic(struct i350_hw *hw, u8 *buffer,
				    u16 length, u16 offset, u8 *sum)
{
	u8 *tmp;
	u8 *bufptr = buffer;
	u32 data = 0;
	u16 remaining, i, j, prev_bytes;

	DEBUGFUNC("i350_mng_host_if_write_generic");

	/* sum = only sum of the data and it is not checksum */

	if (length == 0 || offset + length > I350_HI_MAX_MNG_DATA_LENGTH)
		return -I350_ERR_PARAM;

	tmp = (u8 *)&data;
	prev_bytes = offset & 0x3;
	offset >>= 2;

	if (prev_bytes) {
		data = I350_READ_REG_ARRAY_DWORD(hw, I350_HOST_IF, offset);
		for (j = prev_bytes; j < sizeof(u32); j++) {
			*(tmp + j) = *bufptr++;
			*sum += *(tmp + j);
		}
		I350_WRITE_REG_ARRAY_DWORD(hw, I350_HOST_IF, offset, data);
		length -= j - prev_bytes;
		offset++;
	}

	remaining = length & 0x3;
	length -= remaining;

	/* Calculate length in DWORDs */
	length >>= 2;

	/* The device driver writes the relevant command block into the
	 * ram area.
	 */
	for (i = 0; i < length; i++) {
		for (j = 0; j < sizeof(u32); j++) {
			*(tmp + j) = *bufptr++;
			*sum += *(tmp + j);
		}

		I350_WRITE_REG_ARRAY_DWORD(hw, I350_HOST_IF, offset + i,
					    data);
	}
	if (remaining) {
		for (j = 0; j < sizeof(u32); j++) {
			if (j < remaining)
				*(tmp + j) = *bufptr++;
			else
				*(tmp + j) = 0;

			*sum += *(tmp + j);
		}
		I350_WRITE_REG_ARRAY_DWORD(hw, I350_HOST_IF, offset + i,
					    data);
	}

	return I350_SUCCESS;
}

/**
 *  i350_mng_write_dhcp_info_generic - Writes DHCP info to host interface
 *  @hw: pointer to the HW structure
 *  @buffer: pointer to the host interface
 *  @length: size of the buffer
 *
 *  Writes the DHCP information to the host interface.
 **/
s32 i350_mng_write_dhcp_info_generic(struct i350_hw *hw, u8 *buffer,
				      u16 length)
{
	struct i350_host_mng_command_header hdr;
	s32 ret_val;
	u32 hicr;

	DEBUGFUNC("i350_mng_write_dhcp_info_generic");

	hdr.command_id = I350_MNG_DHCP_TX_PAYLOAD_CMD;
	hdr.command_length = length;
	hdr.reserved1 = 0;
	hdr.reserved2 = 0;
	hdr.checksum = 0;

	/* Enable the host interface */
	ret_val = i350_mng_enable_host_if_generic(hw);
	if (ret_val)
		return ret_val;

	/* Populate the host interface with the contents of "buffer". */
	ret_val = i350_mng_host_if_write_generic(hw, buffer, length,
						  sizeof(hdr), &(hdr.checksum));
	if (ret_val)
		return ret_val;

	/* Write the manageability command header */
	ret_val = i350_mng_write_cmd_header_generic(hw, &hdr);
	if (ret_val)
		return ret_val;

	/* Tell the ARC a new command is pending. */
	hicr = I350_READ_REG(hw, I350_HICR);
	I350_WRITE_REG(hw, I350_HICR, hicr | I350_HICR_C);

	return I350_SUCCESS;
}

/**
 *  i350_enable_mng_pass_thru - Check if management passthrough is needed
 *  @hw: pointer to the HW structure
 *
 *  Verifies the hardware needs to leave interface enabled so that frames can
 *  be directed to and from the management interface.
 **/
bool i350_enable_mng_pass_thru(struct i350_hw *hw)
{
	u32 manc;
	u32 fwsm, factps;

	DEBUGFUNC("i350_enable_mng_pass_thru");

	if (!hw->mac.asf_firmware_present)
		return false;

	manc = I350_READ_REG(hw, I350_MANC);

	if (!(manc & I350_MANC_RCV_TCO_EN))
		return false;

	if (hw->mac.has_fwsm) {
		fwsm = I350_READ_REG(hw, I350_FWSM);
		factps = I350_READ_REG(hw, I350_FACTPS);

		if (!(factps & I350_FACTPS_MNGCG) &&
		    ((fwsm & I350_FWSM_MODE_MASK) ==
		     (i350_mng_mode_pt << I350_FWSM_MODE_SHIFT)))
			return true;
	} else if ((manc & I350_MANC_SMBUS_EN) &&
		   !(manc & I350_MANC_ASF_EN)) {
		return true;
	}

	return false;
}

/**
 *  i350_host_interface_command - Writes buffer to host interface
 *  @hw: pointer to the HW structure
 *  @buffer: contains a command to write
 *  @length: the byte length of the buffer, must be multiple of 4 bytes
 *
 *  Writes a buffer to the Host Interface.  Upon success, returns I350_SUCCESS
 *  else returns I350_ERR_HOST_INTERFACE_COMMAND.
 **/
s32 i350_host_interface_command(struct i350_hw *hw, u8 *buffer, u32 length)
{
	u32 hicr, i;

	DEBUGFUNC("i350_host_interface_command");

	if (!(hw->mac.arc_subsystem_valid)) {
		DEBUGOUT("Hardware doesn't support host interface command.\n");
		return I350_SUCCESS;
	}

	if (!hw->mac.asf_firmware_present) {
		DEBUGOUT("Firmware is not present.\n");
		return I350_SUCCESS;
	}

	if (length == 0 || length & 0x3 ||
	    length > I350_HI_MAX_BLOCK_BYTE_LENGTH) {
		DEBUGOUT("Buffer length failure.\n");
		return -I350_ERR_HOST_INTERFACE_COMMAND;
	}

	/* Check that the host interface is enabled. */
	hicr = I350_READ_REG(hw, I350_HICR);
	if (!(hicr & I350_HICR_EN)) {
		DEBUGOUT("I350_HOST_EN bit disabled.\n");
		return -I350_ERR_HOST_INTERFACE_COMMAND;
	}

	/* Calculate length in DWORDs */
	length >>= 2;

	/* The device driver writes the relevant command block
	 * into the ram area.
	 */
	for (i = 0; i < length; i++)
		I350_WRITE_REG_ARRAY_DWORD(hw, I350_HOST_IF, i,
					    *((u32 *)buffer + i));

	/* Setting this bit tells the ARC that a new command is pending. */
	I350_WRITE_REG(hw, I350_HICR, hicr | I350_HICR_C);

	for (i = 0; i < I350_HI_COMMAND_TIMEOUT; i++) {
		hicr = I350_READ_REG(hw, I350_HICR);
		if (!(hicr & I350_HICR_C))
			break;
		msec_delay(1);
	}

	/* Check command successful completion. */
	if (i == I350_HI_COMMAND_TIMEOUT ||
	    (!(I350_READ_REG(hw, I350_HICR) & I350_HICR_SV))) {
		DEBUGOUT("Command has failed with no status valid.\n");
		return -I350_ERR_HOST_INTERFACE_COMMAND;
	}

	for (i = 0; i < length; i++)
		*((u32 *)buffer + i) = I350_READ_REG_ARRAY_DWORD(hw,
								  I350_HOST_IF,
								  i);

	return I350_SUCCESS;
}

/**
 *  i350_load_firmware - Writes proxy FW code buffer to host interface
 *                        and execute.
 *  @hw: pointer to the HW structure
 *  @buffer: contains a firmware to write
 *  @length: the byte length of the buffer, must be multiple of 4 bytes
 *
 *  Upon success returns I350_SUCCESS, returns I350_ERR_CONFIG if not enabled
 *  in HW else returns I350_ERR_HOST_INTERFACE_COMMAND.
 **/
s32 i350_load_firmware(struct i350_hw *hw, u8 *buffer, u32 length)
{
	u32 hicr, hibba, fwsm, icr, i;

	DEBUGFUNC("i350_load_firmware");

	if (hw->mac.type < i350_i210) {
		DEBUGOUT("Hardware doesn't support loading FW by the driver\n");
		return -I350_ERR_CONFIG;
	}

	/* Check that the host interface is enabled. */
	hicr = I350_READ_REG(hw, I350_HICR);
	if (!(hicr & I350_HICR_EN)) {
		DEBUGOUT("I350_HOST_EN bit disabled.\n");
		return -I350_ERR_CONFIG;
	}
	if (!(hicr & I350_HICR_MEMORY_BASE_EN)) {
		DEBUGOUT("I350_HICR_MEMORY_BASE_EN bit disabled.\n");
		return -I350_ERR_CONFIG;
	}

	if (length == 0 || length & 0x3 || length > I350_HI_FW_MAX_LENGTH) {
		DEBUGOUT("Buffer length failure.\n");
		return -I350_ERR_INVALID_ARGUMENT;
	}

	/* Clear notification from ROM-FW by reading ICR register */
	icr = I350_READ_REG(hw, I350_ICR_V2);

	/* Reset ROM-FW */
	hicr = I350_READ_REG(hw, I350_HICR);
	hicr |= I350_HICR_FW_RESET_ENABLE;
	I350_WRITE_REG(hw, I350_HICR, hicr);
	hicr |= I350_HICR_FW_RESET;
	I350_WRITE_REG(hw, I350_HICR, hicr);
	I350_WRITE_FLUSH(hw);

	/* Wait till MAC notifies about its readiness after ROM-FW reset */
	for (i = 0; i < (I350_HI_COMMAND_TIMEOUT * 2); i++) {
		icr = I350_READ_REG(hw, I350_ICR_V2);
		if (icr & I350_ICR_MNG)
			break;
		msec_delay(1);
	}

	/* Check for timeout */
	if (i == I350_HI_COMMAND_TIMEOUT) {
		DEBUGOUT("FW reset failed.\n");
		return -I350_ERR_HOST_INTERFACE_COMMAND;
	}

	/* Wait till MAC is ready to accept new FW code */
	for (i = 0; i < I350_HI_COMMAND_TIMEOUT; i++) {
		fwsm = I350_READ_REG(hw, I350_FWSM);
		if ((fwsm & I350_FWSM_FW_VALID) &&
		    ((fwsm & I350_FWSM_MODE_MASK) >> I350_FWSM_MODE_SHIFT ==
		    I350_FWSM_HI_EN_ONLY_MODE))
			break;
		msec_delay(1);
	}

	/* Check for timeout */
	if (i == I350_HI_COMMAND_TIMEOUT) {
		DEBUGOUT("FW reset failed.\n");
		return -I350_ERR_HOST_INTERFACE_COMMAND;
	}

	/* Calculate length in DWORDs */
	length >>= 2;

	/* The device driver writes the relevant FW code block
	 * into the ram area in DWORDs via 1kB ram addressing window.
	 */
	for (i = 0; i < length; i++) {
		if (!(i % I350_HI_FW_BLOCK_DWORD_LENGTH)) {
			/* Point to correct 1kB ram window */
			hibba = I350_HI_FW_BASE_ADDRESS +
				((I350_HI_FW_BLOCK_DWORD_LENGTH << 2) *
				(i / I350_HI_FW_BLOCK_DWORD_LENGTH));

			I350_WRITE_REG(hw, I350_HIBBA, hibba);
		}

		I350_WRITE_REG_ARRAY_DWORD(hw, I350_HOST_IF,
					    i % I350_HI_FW_BLOCK_DWORD_LENGTH,
					    *((u32 *)buffer + i));
	}

	/* Setting this bit tells the ARC that a new FW is ready to execute. */
	hicr = I350_READ_REG(hw, I350_HICR);
	I350_WRITE_REG(hw, I350_HICR, hicr | I350_HICR_C);

	for (i = 0; i < I350_HI_COMMAND_TIMEOUT; i++) {
		hicr = I350_READ_REG(hw, I350_HICR);
		if (!(hicr & I350_HICR_C))
			break;
		msec_delay(1);
	}

	/* Check for successful FW start. */
	if (i == I350_HI_COMMAND_TIMEOUT) {
		DEBUGOUT("New FW did not start within timeout period.\n");
		return -I350_ERR_HOST_INTERFACE_COMMAND;
	}

	return I350_SUCCESS;
}
