/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2001-2020 Intel Corporation
 */

#include "i350_api.h"

STATIC void i350_reload_nvm_generic(struct i350_hw *hw);

/**
 *  i350_init_nvm_ops_generic - Initialize NVM function pointers
 *  @hw: pointer to the HW structure
 *
 *  Setups up the function pointers to no-op functions
 **/
void i350_init_nvm_ops_generic(struct i350_hw *hw)
{
	struct i350_nvm_info *nvm = &hw->nvm;
	DEBUGFUNC("i350_init_nvm_ops_generic");

	/* Initialize function pointers */
	nvm->ops.init_params = i350_null_ops_generic;
	nvm->ops.acquire = i350_null_ops_generic;
	nvm->ops.read = i350_null_read_nvm;
	nvm->ops.release = i350_null_nvm_generic;
	nvm->ops.reload = i350_reload_nvm_generic;
	nvm->ops.update = i350_null_ops_generic;
	nvm->ops.valid_led_default = i350_null_led_default;
	nvm->ops.validate = i350_null_ops_generic;
	nvm->ops.write = i350_null_write_nvm;
}

/**
 *  i350_null_nvm_read - No-op function, return 0
 *  @hw: pointer to the HW structure
 *  @a: dummy variable
 *  @b: dummy variable
 *  @c: dummy variable
 **/
s32 i350_null_read_nvm(struct i350_hw I350_UNUSEDARG *hw,
			u16 I350_UNUSEDARG a, u16 I350_UNUSEDARG b,
			u16 I350_UNUSEDARG *c)
{
	DEBUGFUNC("i350_null_read_nvm");
	UNREFERENCED_4PARAMETER(hw, a, b, c);
	return I350_SUCCESS;
}

/**
 *  i350_null_nvm_generic - No-op function, return void
 *  @hw: pointer to the HW structure
 **/
void i350_null_nvm_generic(struct i350_hw I350_UNUSEDARG *hw)
{
	DEBUGFUNC("i350_null_nvm_generic");
	UNREFERENCED_1PARAMETER(hw);
	return;
}

/**
 *  i350_null_led_default - No-op function, return 0
 *  @hw: pointer to the HW structure
 *  @data: dummy variable
 **/
s32 i350_null_led_default(struct i350_hw I350_UNUSEDARG *hw,
			   u16 I350_UNUSEDARG *data)
{
	DEBUGFUNC("i350_null_led_default");
	UNREFERENCED_2PARAMETER(hw, data);
	return I350_SUCCESS;
}

/**
 *  i350_null_write_nvm - No-op function, return 0
 *  @hw: pointer to the HW structure
 *  @a: dummy variable
 *  @b: dummy variable
 *  @c: dummy variable
 **/
s32 i350_null_write_nvm(struct i350_hw I350_UNUSEDARG *hw,
			 u16 I350_UNUSEDARG a, u16 I350_UNUSEDARG b,
			 u16 I350_UNUSEDARG *c)
{
	DEBUGFUNC("i350_null_write_nvm");
	UNREFERENCED_4PARAMETER(hw, a, b, c);
	return I350_SUCCESS;
}

/**
 *  i350_raise_eec_clk - Raise EEPROM clock
 *  @hw: pointer to the HW structure
 *  @eecd: pointer to the EEPROM
 *
 *  Enable/Raise the EEPROM clock bit.
 **/
STATIC void i350_raise_eec_clk(struct i350_hw *hw, u32 *eecd)
{
	*eecd = *eecd | I350_EECD_SK;
	I350_WRITE_REG(hw, I350_EECD, *eecd);
	I350_WRITE_FLUSH(hw);
	usec_delay(hw->nvm.delay_usec);
}

/**
 *  i350_lower_eec_clk - Lower EEPROM clock
 *  @hw: pointer to the HW structure
 *  @eecd: pointer to the EEPROM
 *
 *  Clear/Lower the EEPROM clock bit.
 **/
STATIC void i350_lower_eec_clk(struct i350_hw *hw, u32 *eecd)
{
	*eecd = *eecd & ~I350_EECD_SK;
	I350_WRITE_REG(hw, I350_EECD, *eecd);
	I350_WRITE_FLUSH(hw);
	usec_delay(hw->nvm.delay_usec);
}

/**
 *  i350_shift_out_eec_bits - Shift data bits our to the EEPROM
 *  @hw: pointer to the HW structure
 *  @data: data to send to the EEPROM
 *  @count: number of bits to shift out
 *
 *  We need to shift 'count' bits out to the EEPROM.  So, the value in the
 *  "data" parameter will be shifted out to the EEPROM one bit at a time.
 *  In order to do this, "data" must be broken down into bits.
 **/
STATIC void i350_shift_out_eec_bits(struct i350_hw *hw, u16 data, u16 count)
{
	struct i350_nvm_info *nvm = &hw->nvm;
	u32 eecd = I350_READ_REG(hw, I350_EECD);
	u32 mask;

	DEBUGFUNC("i350_shift_out_eec_bits");

	mask = 0x01 << (count - 1);
	if (nvm->type == i350_nvm_eeprom_microwire)
		eecd &= ~I350_EECD_DO;
	else
	if (nvm->type == i350_nvm_eeprom_spi)
		eecd |= I350_EECD_DO;

	do {
		eecd &= ~I350_EECD_DI;

		if (data & mask)
			eecd |= I350_EECD_DI;

		I350_WRITE_REG(hw, I350_EECD, eecd);
		I350_WRITE_FLUSH(hw);

		usec_delay(nvm->delay_usec);

		i350_raise_eec_clk(hw, &eecd);
		i350_lower_eec_clk(hw, &eecd);

		mask >>= 1;
	} while (mask);

	eecd &= ~I350_EECD_DI;
	I350_WRITE_REG(hw, I350_EECD, eecd);
}

/**
 *  i350_shift_in_eec_bits - Shift data bits in from the EEPROM
 *  @hw: pointer to the HW structure
 *  @count: number of bits to shift in
 *
 *  In order to read a register from the EEPROM, we need to shift 'count' bits
 *  in from the EEPROM.  Bits are "shifted in" by raising the clock input to
 *  the EEPROM (setting the SK bit), and then reading the value of the data out
 *  "DO" bit.  During this "shifting in" process the data in "DI" bit should
 *  always be clear.
 **/
STATIC u16 i350_shift_in_eec_bits(struct i350_hw *hw, u16 count)
{
	u32 eecd;
	u32 i;
	u16 data;

	DEBUGFUNC("i350_shift_in_eec_bits");

	eecd = I350_READ_REG(hw, I350_EECD);

	eecd &= ~(I350_EECD_DO | I350_EECD_DI);
	data = 0;

	for (i = 0; i < count; i++) {
		data <<= 1;
		i350_raise_eec_clk(hw, &eecd);

		eecd = I350_READ_REG(hw, I350_EECD);

		eecd &= ~I350_EECD_DI;
		if (eecd & I350_EECD_DO)
			data |= 1;

		i350_lower_eec_clk(hw, &eecd);
	}

	return data;
}

/**
 *  i350_poll_eerd_eewr_done - Poll for EEPROM read/write completion
 *  @hw: pointer to the HW structure
 *  @ee_reg: EEPROM flag for polling
 *
 *  Polls the EEPROM status bit for either read or write completion based
 *  upon the value of 'ee_reg'.
 **/
s32 i350_poll_eerd_eewr_done(struct i350_hw *hw, int ee_reg)
{
	u32 attempts = 100000;
	u32 i, reg = 0;

	DEBUGFUNC("i350_poll_eerd_eewr_done");

	for (i = 0; i < attempts; i++) {
		if (ee_reg == I350_NVM_POLL_READ)
			reg = I350_READ_REG(hw, I350_EERD);
		else
			reg = I350_READ_REG(hw, I350_EEWR);

		if (reg & I350_NVM_RW_REG_DONE)
			return I350_SUCCESS;

		usec_delay(5);
	}

	return -I350_ERR_NVM;
}

/**
 *  i350_acquire_nvm_generic - Generic request for access to EEPROM
 *  @hw: pointer to the HW structure
 *
 *  Set the EEPROM access request bit and wait for EEPROM access grant bit.
 *  Return successful if access grant bit set, else clear the request for
 *  EEPROM access and return -I350_ERR_NVM (-1).
 **/
s32 i350_acquire_nvm_generic(struct i350_hw *hw)
{
	u32 eecd = I350_READ_REG(hw, I350_EECD);
	s32 timeout = I350_NVM_GRANT_ATTEMPTS;

	DEBUGFUNC("i350_acquire_nvm_generic");

	I350_WRITE_REG(hw, I350_EECD, eecd | I350_EECD_REQ);
	eecd = I350_READ_REG(hw, I350_EECD);

	while (timeout) {
		if (eecd & I350_EECD_GNT)
			break;
		usec_delay(5);
		eecd = I350_READ_REG(hw, I350_EECD);
		timeout--;
	}

	if (!timeout) {
		eecd &= ~I350_EECD_REQ;
		I350_WRITE_REG(hw, I350_EECD, eecd);
		DEBUGOUT("Could not acquire NVM grant\n");
		return -I350_ERR_NVM;
	}

	return I350_SUCCESS;
}

/**
 *  i350_standby_nvm - Return EEPROM to standby state
 *  @hw: pointer to the HW structure
 *
 *  Return the EEPROM to a standby state.
 **/
STATIC void i350_standby_nvm(struct i350_hw *hw)
{
	struct i350_nvm_info *nvm = &hw->nvm;
	u32 eecd = I350_READ_REG(hw, I350_EECD);

	DEBUGFUNC("i350_standby_nvm");

	if (nvm->type == i350_nvm_eeprom_microwire) {
		eecd &= ~(I350_EECD_CS | I350_EECD_SK);
		I350_WRITE_REG(hw, I350_EECD, eecd);
		I350_WRITE_FLUSH(hw);
		usec_delay(nvm->delay_usec);

		i350_raise_eec_clk(hw, &eecd);

		/* Select EEPROM */
		eecd |= I350_EECD_CS;
		I350_WRITE_REG(hw, I350_EECD, eecd);
		I350_WRITE_FLUSH(hw);
		usec_delay(nvm->delay_usec);

		i350_lower_eec_clk(hw, &eecd);
	} else if (nvm->type == i350_nvm_eeprom_spi) {
		/* Toggle CS to flush commands */
		eecd |= I350_EECD_CS;
		I350_WRITE_REG(hw, I350_EECD, eecd);
		I350_WRITE_FLUSH(hw);
		usec_delay(nvm->delay_usec);
		eecd &= ~I350_EECD_CS;
		I350_WRITE_REG(hw, I350_EECD, eecd);
		I350_WRITE_FLUSH(hw);
		usec_delay(nvm->delay_usec);
	}
}

/**
 *  i350_stop_nvm - Terminate EEPROM command
 *  @hw: pointer to the HW structure
 *
 *  Terminates the current command by inverting the EEPROM's chip select pin.
 **/
void i350_stop_nvm(struct i350_hw *hw)
{
	u32 eecd;

	DEBUGFUNC("i350_stop_nvm");

	eecd = I350_READ_REG(hw, I350_EECD);
	if (hw->nvm.type == i350_nvm_eeprom_spi) {
		/* Pull CS high */
		eecd |= I350_EECD_CS;
		i350_lower_eec_clk(hw, &eecd);
	} else if (hw->nvm.type == i350_nvm_eeprom_microwire) {
		/* CS on Microwire is active-high */
		eecd &= ~(I350_EECD_CS | I350_EECD_DI);
		I350_WRITE_REG(hw, I350_EECD, eecd);
		i350_raise_eec_clk(hw, &eecd);
		i350_lower_eec_clk(hw, &eecd);
	}
}

/**
 *  i350_release_nvm_generic - Release exclusive access to EEPROM
 *  @hw: pointer to the HW structure
 *
 *  Stop any current commands to the EEPROM and clear the EEPROM request bit.
 **/
void i350_release_nvm_generic(struct i350_hw *hw)
{
	u32 eecd;

	DEBUGFUNC("i350_release_nvm_generic");

	i350_stop_nvm(hw);

	eecd = I350_READ_REG(hw, I350_EECD);
	eecd &= ~I350_EECD_REQ;
	I350_WRITE_REG(hw, I350_EECD, eecd);
}

/**
 *  i350_ready_nvm_eeprom - Prepares EEPROM for read/write
 *  @hw: pointer to the HW structure
 *
 *  Setups the EEPROM for reading and writing.
 **/
STATIC s32 i350_ready_nvm_eeprom(struct i350_hw *hw)
{
	struct i350_nvm_info *nvm = &hw->nvm;
	u32 eecd = I350_READ_REG(hw, I350_EECD);
	u8 spi_stat_reg;

	DEBUGFUNC("i350_ready_nvm_eeprom");

	if (nvm->type == i350_nvm_eeprom_microwire) {
		/* Clear SK and DI */
		eecd &= ~(I350_EECD_DI | I350_EECD_SK);
		I350_WRITE_REG(hw, I350_EECD, eecd);
		/* Set CS */
		eecd |= I350_EECD_CS;
		I350_WRITE_REG(hw, I350_EECD, eecd);
	} else if (nvm->type == i350_nvm_eeprom_spi) {
		u16 timeout = NVM_MAX_RETRY_SPI;

		/* Clear SK and CS */
		eecd &= ~(I350_EECD_CS | I350_EECD_SK);
		I350_WRITE_REG(hw, I350_EECD, eecd);
		I350_WRITE_FLUSH(hw);
		usec_delay(1);

		/* Read "Status Register" repeatedly until the LSB is cleared.
		 * The EEPROM will signal that the command has been completed
		 * by clearing bit 0 of the internal status register.  If it's
		 * not cleared within 'timeout', then error out.
		 */
		while (timeout) {
			i350_shift_out_eec_bits(hw, NVM_RDSR_OPCODE_SPI,
						 hw->nvm.opcode_bits);
			spi_stat_reg = (u8)i350_shift_in_eec_bits(hw, 8);
			if (!(spi_stat_reg & NVM_STATUS_RDY_SPI))
				break;

			usec_delay(5);
			i350_standby_nvm(hw);
			timeout--;
		}

		if (!timeout) {
			DEBUGOUT("SPI NVM Status error\n");
			return -I350_ERR_NVM;
		}
	}

	return I350_SUCCESS;
}

/**
 *  i350_read_nvm_spi - Read EEPROM's using SPI
 *  @hw: pointer to the HW structure
 *  @offset: offset of word in the EEPROM to read
 *  @words: number of words to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM.
 **/
s32 i350_read_nvm_spi(struct i350_hw *hw, u16 offset, u16 words, u16 *data)
{
	struct i350_nvm_info *nvm = &hw->nvm;
	u32 i = 0;
	s32 ret_val;
	u16 word_in;
	u8 read_opcode = NVM_READ_OPCODE_SPI;

	DEBUGFUNC("i350_read_nvm_spi");

	/* A check for invalid values:  offset too large, too many words,
	 * and not enough words.
	 */
	if ((offset >= nvm->word_size) || (words > (nvm->word_size - offset)) ||
	    (words == 0)) {
		DEBUGOUT("nvm parameter(s) out of bounds\n");
		return -I350_ERR_NVM;
	}

	ret_val = nvm->ops.acquire(hw);
	if (ret_val)
		return ret_val;

	ret_val = i350_ready_nvm_eeprom(hw);
	if (ret_val)
		goto release;

	i350_standby_nvm(hw);

	if ((nvm->address_bits == 8) && (offset >= 128))
		read_opcode |= NVM_A8_OPCODE_SPI;

	/* Send the READ command (opcode + addr) */
	i350_shift_out_eec_bits(hw, read_opcode, nvm->opcode_bits);
	i350_shift_out_eec_bits(hw, (u16)(offset*2), nvm->address_bits);

	/* Read the data.  SPI NVMs increment the address with each byte
	 * read and will roll over if reading beyond the end.  This allows
	 * us to read the whole NVM from any offset
	 */
	for (i = 0; i < words; i++) {
		word_in = i350_shift_in_eec_bits(hw, 16);
		data[i] = (word_in >> 8) | (word_in << 8);
	}

release:
	nvm->ops.release(hw);

	return ret_val;
}

/**
 *  i350_read_nvm_microwire - Reads EEPROM's using microwire
 *  @hw: pointer to the HW structure
 *  @offset: offset of word in the EEPROM to read
 *  @words: number of words to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM.
 **/
s32 i350_read_nvm_microwire(struct i350_hw *hw, u16 offset, u16 words,
			     u16 *data)
{
	struct i350_nvm_info *nvm = &hw->nvm;
	u32 i = 0;
	s32 ret_val;
	u8 read_opcode = NVM_READ_OPCODE_MICROWIRE;

	DEBUGFUNC("i350_read_nvm_microwire");

	/* A check for invalid values:  offset too large, too many words,
	 * and not enough words.
	 */
	if ((offset >= nvm->word_size) || (words > (nvm->word_size - offset)) ||
	    (words == 0)) {
		DEBUGOUT("nvm parameter(s) out of bounds\n");
		return -I350_ERR_NVM;
	}

	ret_val = nvm->ops.acquire(hw);
	if (ret_val)
		return ret_val;

	ret_val = i350_ready_nvm_eeprom(hw);
	if (ret_val)
		goto release;

	for (i = 0; i < words; i++) {
		/* Send the READ command (opcode + addr) */
		i350_shift_out_eec_bits(hw, read_opcode, nvm->opcode_bits);
		i350_shift_out_eec_bits(hw, (u16)(offset + i),
					nvm->address_bits);

		/* Read the data.  For microwire, each word requires the
		 * overhead of setup and tear-down.
		 */
		data[i] = i350_shift_in_eec_bits(hw, 16);
		i350_standby_nvm(hw);
	}

release:
	nvm->ops.release(hw);

	return ret_val;
}

/**
 *  i350_read_nvm_eerd - Reads EEPROM using EERD register
 *  @hw: pointer to the HW structure
 *  @offset: offset of word in the EEPROM to read
 *  @words: number of words to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM using the EERD register.
 **/
s32 i350_read_nvm_eerd(struct i350_hw *hw, u16 offset, u16 words, u16 *data)
{
	struct i350_nvm_info *nvm = &hw->nvm;
	u32 i, eerd = 0;
	s32 ret_val = I350_SUCCESS;

	DEBUGFUNC("i350_read_nvm_eerd");

	/* A check for invalid values:  offset too large, too many words,
	 * too many words for the offset, and not enough words.
	 */
	if ((offset >= nvm->word_size) || (words > (nvm->word_size - offset)) ||
	    (words == 0)) {
		DEBUGOUT("nvm parameter(s) out of bounds\n");
		return -I350_ERR_NVM;
	}

	for (i = 0; i < words; i++) {
		eerd = ((offset + i) << I350_NVM_RW_ADDR_SHIFT) +
		       I350_NVM_RW_REG_START;

		I350_WRITE_REG(hw, I350_EERD, eerd);
		ret_val = i350_poll_eerd_eewr_done(hw, I350_NVM_POLL_READ);
		if (ret_val)
			break;

		data[i] = (I350_READ_REG(hw, I350_EERD) >>
			   I350_NVM_RW_REG_DATA);
	}

	if (ret_val)
		DEBUGOUT1("NVM read error: %d\n", ret_val);

	return ret_val;
}

/**
 *  i350_write_nvm_spi - Write to EEPROM using SPI
 *  @hw: pointer to the HW structure
 *  @offset: offset within the EEPROM to be written to
 *  @words: number of words to write
 *  @data: 16 bit word(s) to be written to the EEPROM
 *
 *  Writes data to EEPROM at offset using SPI interface.
 *
 *  If i350_update_nvm_checksum is not called after this function , the
 *  EEPROM will most likely contain an invalid checksum.
 **/
s32 i350_write_nvm_spi(struct i350_hw *hw, u16 offset, u16 words, u16 *data)
{
	struct i350_nvm_info *nvm = &hw->nvm;
	s32 ret_val = -I350_ERR_NVM;
	u16 widx = 0;

	DEBUGFUNC("i350_write_nvm_spi");

	/* A check for invalid values:  offset too large, too many words,
	 * and not enough words.
	 */
	if ((offset >= nvm->word_size) || (words > (nvm->word_size - offset)) ||
	    (words == 0)) {
		DEBUGOUT("nvm parameter(s) out of bounds\n");
		return -I350_ERR_NVM;
	}

	while (widx < words) {
		u8 write_opcode = NVM_WRITE_OPCODE_SPI;

		ret_val = nvm->ops.acquire(hw);
		if (ret_val)
			return ret_val;

		ret_val = i350_ready_nvm_eeprom(hw);
		if (ret_val) {
			nvm->ops.release(hw);
			return ret_val;
		}

		i350_standby_nvm(hw);

		/* Send the WRITE ENABLE command (8 bit opcode) */
		i350_shift_out_eec_bits(hw, NVM_WREN_OPCODE_SPI,
					 nvm->opcode_bits);

		i350_standby_nvm(hw);

		/* Some SPI eeproms use the 8th address bit embedded in the
		 * opcode
		 */
		if ((nvm->address_bits == 8) && (offset >= 128))
			write_opcode |= NVM_A8_OPCODE_SPI;

		/* Send the Write command (8-bit opcode + addr) */
		i350_shift_out_eec_bits(hw, write_opcode, nvm->opcode_bits);
		i350_shift_out_eec_bits(hw, (u16)((offset + widx) * 2),
					 nvm->address_bits);

		/* Loop to allow for up to whole page write of eeprom */
		while (widx < words) {
			u16 word_out = data[widx];
			word_out = (word_out >> 8) | (word_out << 8);
			i350_shift_out_eec_bits(hw, word_out, 16);
			widx++;

			if ((((offset + widx) * 2) % nvm->page_size) == 0) {
				i350_standby_nvm(hw);
				break;
			}
		}
		msec_delay(10);
		nvm->ops.release(hw);
	}

	return ret_val;
}

/**
 *  i350_write_nvm_microwire - Writes EEPROM using microwire
 *  @hw: pointer to the HW structure
 *  @offset: offset within the EEPROM to be written to
 *  @words: number of words to write
 *  @data: 16 bit word(s) to be written to the EEPROM
 *
 *  Writes data to EEPROM at offset using microwire interface.
 *
 *  If i350_update_nvm_checksum is not called after this function , the
 *  EEPROM will most likely contain an invalid checksum.
 **/
s32 i350_write_nvm_microwire(struct i350_hw *hw, u16 offset, u16 words,
			      u16 *data)
{
	struct i350_nvm_info *nvm = &hw->nvm;
	s32  ret_val;
	u32 eecd;
	u16 words_written = 0;
	u16 widx = 0;

	DEBUGFUNC("i350_write_nvm_microwire");

	/* A check for invalid values:  offset too large, too many words,
	 * and not enough words.
	 */
	if ((offset >= nvm->word_size) || (words > (nvm->word_size - offset)) ||
	    (words == 0)) {
		DEBUGOUT("nvm parameter(s) out of bounds\n");
		return -I350_ERR_NVM;
	}

	ret_val = nvm->ops.acquire(hw);
	if (ret_val)
		return ret_val;

	ret_val = i350_ready_nvm_eeprom(hw);
	if (ret_val)
		goto release;

	i350_shift_out_eec_bits(hw, NVM_EWEN_OPCODE_MICROWIRE,
				 (u16)(nvm->opcode_bits + 2));

	i350_shift_out_eec_bits(hw, 0, (u16)(nvm->address_bits - 2));

	i350_standby_nvm(hw);

	while (words_written < words) {
		i350_shift_out_eec_bits(hw, NVM_WRITE_OPCODE_MICROWIRE,
					 nvm->opcode_bits);

		i350_shift_out_eec_bits(hw, (u16)(offset + words_written),
					 nvm->address_bits);

		i350_shift_out_eec_bits(hw, data[words_written], 16);

		i350_standby_nvm(hw);

		for (widx = 0; widx < 200; widx++) {
			eecd = I350_READ_REG(hw, I350_EECD);
			if (eecd & I350_EECD_DO)
				break;
			usec_delay(50);
		}

		if (widx == 200) {
			DEBUGOUT("NVM Write did not complete\n");
			ret_val = -I350_ERR_NVM;
			goto release;
		}

		i350_standby_nvm(hw);

		words_written++;
	}

	i350_shift_out_eec_bits(hw, NVM_EWDS_OPCODE_MICROWIRE,
				 (u16)(nvm->opcode_bits + 2));

	i350_shift_out_eec_bits(hw, 0, (u16)(nvm->address_bits - 2));

release:
	nvm->ops.release(hw);

	return ret_val;
}

/**
 *  i350_read_pba_string_generic - Read device part number
 *  @hw: pointer to the HW structure
 *  @pba_num: pointer to device part number
 *  @pba_num_size: size of part number buffer
 *
 *  Reads the product board assembly (PBA) number from the EEPROM and stores
 *  the value in pba_num.
 **/
s32 i350_read_pba_string_generic(struct i350_hw *hw, u8 *pba_num,
				  u32 pba_num_size)
{
	s32 ret_val;
	u16 nvm_data;
	u16 pba_ptr;
	u16 offset;
	u16 length;

	DEBUGFUNC("i350_read_pba_string_generic");

	if ((hw->mac.type == i350_i210 ||
	     hw->mac.type == i350_i211) &&
	     !i350_get_flash_presence_i210(hw)) {
		DEBUGOUT("Flashless no PBA string\n");
		return -I350_ERR_NVM_PBA_SECTION;
	}

	if (pba_num == NULL) {
		DEBUGOUT("PBA string buffer was null\n");
		return -I350_ERR_INVALID_ARGUMENT;
	}

	ret_val = hw->nvm.ops.read(hw, NVM_PBA_OFFSET_0, 1, &nvm_data);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	}

	ret_val = hw->nvm.ops.read(hw, NVM_PBA_OFFSET_1, 1, &pba_ptr);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	}

	/* if nvm_data is not ptr guard the PBA must be in legacy format which
	 * means pba_ptr is actually our second data word for the PBA number
	 * and we can decode it into an ascii string
	 */
	if (nvm_data != NVM_PBA_PTR_GUARD) {
		DEBUGOUT("NVM PBA number is not stored as string\n");

		/* make sure callers buffer is big enough to store the PBA */
		if (pba_num_size < I350_PBANUM_LENGTH) {
			DEBUGOUT("PBA string buffer too small\n");
			return I350_ERR_NO_SPACE;
		}

		/* extract hex string from data and pba_ptr */
		pba_num[0] = (nvm_data >> 12) & 0xF;
		pba_num[1] = (nvm_data >> 8) & 0xF;
		pba_num[2] = (nvm_data >> 4) & 0xF;
		pba_num[3] = nvm_data & 0xF;
		pba_num[4] = (pba_ptr >> 12) & 0xF;
		pba_num[5] = (pba_ptr >> 8) & 0xF;
		pba_num[6] = '-';
		pba_num[7] = 0;
		pba_num[8] = (pba_ptr >> 4) & 0xF;
		pba_num[9] = pba_ptr & 0xF;

		/* put a null character on the end of our string */
		pba_num[10] = '\0';

		/* switch all the data but the '-' to hex char */
		for (offset = 0; offset < 10; offset++) {
			if (pba_num[offset] < 0xA)
				pba_num[offset] += '0';
			else if (pba_num[offset] < 0x10)
				pba_num[offset] += 'A' - 0xA;
		}

		return I350_SUCCESS;
	}

	ret_val = hw->nvm.ops.read(hw, pba_ptr, 1, &length);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	}

	if (length == 0xFFFF || length == 0) {
		DEBUGOUT("NVM PBA number section invalid length\n");
		return -I350_ERR_NVM_PBA_SECTION;
	}
	/* check if pba_num buffer is big enough */
	if (pba_num_size < (((u32)length * 2) - 1)) {
		DEBUGOUT("PBA string buffer too small\n");
		return -I350_ERR_NO_SPACE;
	}

	/* trim pba length from start of string */
	pba_ptr++;
	length--;

	for (offset = 0; offset < length; offset++) {
		ret_val = hw->nvm.ops.read(hw, pba_ptr + offset, 1, &nvm_data);
		if (ret_val) {
			DEBUGOUT("NVM Read Error\n");
			return ret_val;
		}
		pba_num[offset * 2] = (u8)(nvm_data >> 8);
		pba_num[(offset * 2) + 1] = (u8)(nvm_data & 0xFF);
	}
	pba_num[offset * 2] = '\0';

	return I350_SUCCESS;
}

/**
 *  i350_read_pba_length_generic - Read device part number length
 *  @hw: pointer to the HW structure
 *  @pba_num_size: size of part number buffer
 *
 *  Reads the product board assembly (PBA) number length from the EEPROM and
 *  stores the value in pba_num_size.
 **/
s32 i350_read_pba_length_generic(struct i350_hw *hw, u32 *pba_num_size)
{
	s32 ret_val;
	u16 nvm_data;
	u16 pba_ptr;
	u16 length;

	DEBUGFUNC("i350_read_pba_length_generic");

	if (pba_num_size == NULL) {
		DEBUGOUT("PBA buffer size was null\n");
		return -I350_ERR_INVALID_ARGUMENT;
	}

	ret_val = hw->nvm.ops.read(hw, NVM_PBA_OFFSET_0, 1, &nvm_data);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	}

	ret_val = hw->nvm.ops.read(hw, NVM_PBA_OFFSET_1, 1, &pba_ptr);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	}

	 /* if data is not ptr guard the PBA must be in legacy format */
	if (nvm_data != NVM_PBA_PTR_GUARD) {
		*pba_num_size = I350_PBANUM_LENGTH;
		return I350_SUCCESS;
	}

	ret_val = hw->nvm.ops.read(hw, pba_ptr, 1, &length);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	}

	if (length == 0xFFFF || length == 0) {
		DEBUGOUT("NVM PBA number section invalid length\n");
		return -I350_ERR_NVM_PBA_SECTION;
	}

	/* Convert from length in u16 values to u8 chars, add 1 for NULL,
	 * and subtract 2 because length field is included in length.
	 */
	*pba_num_size = ((u32)length * 2) - 1;

	return I350_SUCCESS;
}

/**
 *  i350_read_pba_num_generic - Read device part number
 *  @hw: pointer to the HW structure
 *  @pba_num: pointer to device part number
 *
 *  Reads the product board assembly (PBA) number from the EEPROM and stores
 *  the value in pba_num.
 **/
s32 i350_read_pba_num_generic(struct i350_hw *hw, u32 *pba_num)
{
	s32 ret_val;
	u16 nvm_data;

	DEBUGFUNC("i350_read_pba_num_generic");

	ret_val = hw->nvm.ops.read(hw, NVM_PBA_OFFSET_0, 1, &nvm_data);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	} else if (nvm_data == NVM_PBA_PTR_GUARD) {
		DEBUGOUT("NVM Not Supported\n");
		return -I350_NOT_IMPLEMENTED;
	}
	*pba_num = (u32)(nvm_data << 16);

	ret_val = hw->nvm.ops.read(hw, NVM_PBA_OFFSET_1, 1, &nvm_data);
	if (ret_val) {
		DEBUGOUT("NVM Read Error\n");
		return ret_val;
	}
	*pba_num |= nvm_data;

	return I350_SUCCESS;
}


/**
 *  i350_read_pba_raw
 *  @hw: pointer to the HW structure
 *  @eeprom_buf: optional pointer to EEPROM image
 *  @eeprom_buf_size: size of EEPROM image in words
 *  @max_pba_block_size: PBA block size limit
 *  @pba: pointer to output PBA structure
 *
 *  Reads PBA from EEPROM image when eeprom_buf is not NULL.
 *  Reads PBA from physical EEPROM device when eeprom_buf is NULL.
 *
 **/
s32 i350_read_pba_raw(struct i350_hw *hw, u16 *eeprom_buf,
		       u32 eeprom_buf_size, u16 max_pba_block_size,
		       struct i350_pba *pba)
{
	s32 ret_val;
	u16 pba_block_size;

	if (pba == NULL)
		return -I350_ERR_PARAM;

	if (eeprom_buf == NULL) {
		ret_val = i350_read_nvm(hw, NVM_PBA_OFFSET_0, 2,
					 &pba->word[0]);
		if (ret_val)
			return ret_val;
	} else {
		if (eeprom_buf_size > NVM_PBA_OFFSET_1) {
			pba->word[0] = eeprom_buf[NVM_PBA_OFFSET_0];
			pba->word[1] = eeprom_buf[NVM_PBA_OFFSET_1];
		} else {
			return -I350_ERR_PARAM;
		}
	}

	if (pba->word[0] == NVM_PBA_PTR_GUARD) {
		if (pba->pba_block == NULL)
			return -I350_ERR_PARAM;

		ret_val = i350_get_pba_block_size(hw, eeprom_buf,
						   eeprom_buf_size,
						   &pba_block_size);
		if (ret_val)
			return ret_val;

		if (pba_block_size > max_pba_block_size)
			return -I350_ERR_PARAM;

		if (eeprom_buf == NULL) {
			ret_val = i350_read_nvm(hw, pba->word[1],
						 pba_block_size,
						 pba->pba_block);
			if (ret_val)
				return ret_val;
		} else {
			if (eeprom_buf_size > (u32)(pba->word[1] +
					      pba_block_size)) {
				memcpy(pba->pba_block,
				       &eeprom_buf[pba->word[1]],
				       pba_block_size * sizeof(u16));
			} else {
				return -I350_ERR_PARAM;
			}
		}
	}

	return I350_SUCCESS;
}

/**
 *  i350_write_pba_raw
 *  @hw: pointer to the HW structure
 *  @eeprom_buf: optional pointer to EEPROM image
 *  @eeprom_buf_size: size of EEPROM image in words
 *  @pba: pointer to PBA structure
 *
 *  Writes PBA to EEPROM image when eeprom_buf is not NULL.
 *  Writes PBA to physical EEPROM device when eeprom_buf is NULL.
 *
 **/
s32 i350_write_pba_raw(struct i350_hw *hw, u16 *eeprom_buf,
			u32 eeprom_buf_size, struct i350_pba *pba)
{
	s32 ret_val;

	if (pba == NULL)
		return -I350_ERR_PARAM;

	if (eeprom_buf == NULL) {
		ret_val = i350_write_nvm(hw, NVM_PBA_OFFSET_0, 2,
					  &pba->word[0]);
		if (ret_val)
			return ret_val;
	} else {
		if (eeprom_buf_size > NVM_PBA_OFFSET_1) {
			eeprom_buf[NVM_PBA_OFFSET_0] = pba->word[0];
			eeprom_buf[NVM_PBA_OFFSET_1] = pba->word[1];
		} else {
			return -I350_ERR_PARAM;
		}
	}

	if (pba->word[0] == NVM_PBA_PTR_GUARD) {
		if (pba->pba_block == NULL)
			return -I350_ERR_PARAM;

		if (eeprom_buf == NULL) {
			ret_val = i350_write_nvm(hw, pba->word[1],
						  pba->pba_block[0],
						  pba->pba_block);
			if (ret_val)
				return ret_val;
		} else {
			if (eeprom_buf_size > (u32)(pba->word[1] +
					      pba->pba_block[0])) {
				memcpy(&eeprom_buf[pba->word[1]],
				       pba->pba_block,
				       pba->pba_block[0] * sizeof(u16));
			} else {
				return -I350_ERR_PARAM;
			}
		}
	}

	return I350_SUCCESS;
}

/**
 *  i350_get_pba_block_size
 *  @hw: pointer to the HW structure
 *  @eeprom_buf: optional pointer to EEPROM image
 *  @eeprom_buf_size: size of EEPROM image in words
 *  @pba_data_size: pointer to output variable
 *
 *  Returns the size of the PBA block in words. Function operates on EEPROM
 *  image if the eeprom_buf pointer is not NULL otherwise it accesses physical
 *  EEPROM device.
 *
 **/
s32 i350_get_pba_block_size(struct i350_hw *hw, u16 *eeprom_buf,
			     u32 eeprom_buf_size, u16 *pba_block_size)
{
	s32 ret_val;
	u16 pba_word[2];
	u16 length;

	DEBUGFUNC("i350_get_pba_block_size");

	if (eeprom_buf == NULL) {
		ret_val = i350_read_nvm(hw, NVM_PBA_OFFSET_0, 2, &pba_word[0]);
		if (ret_val)
			return ret_val;
	} else {
		if (eeprom_buf_size > NVM_PBA_OFFSET_1) {
			pba_word[0] = eeprom_buf[NVM_PBA_OFFSET_0];
			pba_word[1] = eeprom_buf[NVM_PBA_OFFSET_1];
		} else {
			return -I350_ERR_PARAM;
		}
	}

	if (pba_word[0] == NVM_PBA_PTR_GUARD) {
		if (eeprom_buf == NULL) {
			ret_val = i350_read_nvm(hw, pba_word[1] + 0, 1,
						 &length);
			if (ret_val)
				return ret_val;
		} else {
			if (eeprom_buf_size > pba_word[1])
				length = eeprom_buf[pba_word[1] + 0];
			else
				return -I350_ERR_PARAM;
		}

		if (length == 0xFFFF || length == 0)
			return -I350_ERR_NVM_PBA_SECTION;
	} else {
		/* PBA number in legacy format, there is no PBA Block. */
		length = 0;
	}

	if (pba_block_size != NULL)
		*pba_block_size = length;

	return I350_SUCCESS;
}

/**
 *  i350_read_mac_addr_generic - Read device MAC address
 *  @hw: pointer to the HW structure
 *
 *  Reads the device MAC address from the EEPROM and stores the value.
 *  Since devices with two ports use the same EEPROM, we increment the
 *  last bit in the MAC address for the second port.
 **/
s32 i350_read_mac_addr_generic(struct i350_hw *hw)
{
	u32 rar_high;
	u32 rar_low;
	u16 i;

	rar_high = I350_READ_REG(hw, I350_RAH(0));
	rar_low = I350_READ_REG(hw, I350_RAL(0));

	for (i = 0; i < I350_RAL_MAC_ADDR_LEN; i++)
		hw->mac.perm_addr[i] = (u8)(rar_low >> (i*8));

	for (i = 0; i < I350_RAH_MAC_ADDR_LEN; i++)
		hw->mac.perm_addr[i+4] = (u8)(rar_high >> (i*8));

	for (i = 0; i < ETH_ADDR_LEN; i++)
		hw->mac.addr[i] = hw->mac.perm_addr[i];

	return I350_SUCCESS;
}

/**
 *  i350_validate_nvm_checksum_generic - Validate EEPROM checksum
 *  @hw: pointer to the HW structure
 *
 *  Calculates the EEPROM checksum by reading/adding each word of the EEPROM
 *  and then verifies that the sum of the EEPROM is equal to 0xBABA.
 **/
s32 i350_validate_nvm_checksum_generic(struct i350_hw *hw)
{
	s32 ret_val;
	u16 checksum = 0;
	u16 i, nvm_data;

	DEBUGFUNC("i350_validate_nvm_checksum_generic");

	for (i = 0; i < (NVM_CHECKSUM_REG + 1); i++) {
		ret_val = hw->nvm.ops.read(hw, i, 1, &nvm_data);
		if (ret_val) {
			DEBUGOUT("NVM Read Error\n");
			return ret_val;
		}
		checksum += nvm_data;
	}

	if (checksum != (u16) NVM_SUM) {
		DEBUGOUT("NVM Checksum Invalid\n");
		return -I350_ERR_NVM;
	}

	return I350_SUCCESS;
}

/**
 *  i350_update_nvm_checksum_generic - Update EEPROM checksum
 *  @hw: pointer to the HW structure
 *
 *  Updates the EEPROM checksum by reading/adding each word of the EEPROM
 *  up to the checksum.  Then calculates the EEPROM checksum and writes the
 *  value to the EEPROM.
 **/
s32 i350_update_nvm_checksum_generic(struct i350_hw *hw)
{
	s32 ret_val;
	u16 checksum = 0;
	u16 i, nvm_data;

	DEBUGFUNC("i350_update_nvm_checksum");

	for (i = 0; i < NVM_CHECKSUM_REG; i++) {
		ret_val = hw->nvm.ops.read(hw, i, 1, &nvm_data);
		if (ret_val) {
			DEBUGOUT("NVM Read Error while updating checksum.\n");
			return ret_val;
		}
		checksum += nvm_data;
	}
	checksum = (u16) NVM_SUM - checksum;
	ret_val = hw->nvm.ops.write(hw, NVM_CHECKSUM_REG, 1, &checksum);
	if (ret_val)
		DEBUGOUT("NVM Write Error while updating checksum.\n");

	return ret_val;
}

/**
 *  i350_reload_nvm_generic - Reloads EEPROM
 *  @hw: pointer to the HW structure
 *
 *  Reloads the EEPROM by setting the "Reinitialize from EEPROM" bit in the
 *  extended control register.
 **/
STATIC void i350_reload_nvm_generic(struct i350_hw *hw)
{
	u32 ctrl_ext;

	DEBUGFUNC("i350_reload_nvm_generic");

	usec_delay(10);
	ctrl_ext = I350_READ_REG(hw, I350_CTRL_EXT);
	ctrl_ext |= I350_CTRL_EXT_EE_RST;
	I350_WRITE_REG(hw, I350_CTRL_EXT, ctrl_ext);
	I350_WRITE_FLUSH(hw);
}

/**
 *  i350_get_fw_version - Get firmware version information
 *  @hw: pointer to the HW structure
 *  @fw_vers: pointer to output version structure
 *
 *  unsupported/not present features return 0 in version structure
 **/
void i350_get_fw_version(struct i350_hw *hw, struct i350_fw_version *fw_vers)
{
	u16 eeprom_verh, eeprom_verl, etrack_test, fw_version;
	u8 q, hval, rem, result;
	u16 comb_verh, comb_verl, comb_offset;

	memset(fw_vers, 0, sizeof(struct i350_fw_version));

	/* basic eeprom version numbers, bits used vary by part and by tool
	 * used to create the nvm images */
	/* Check which data format we have */
	switch (hw->mac.type) {
	case i350_i211:
		i350_read_invm_version(hw, fw_vers);
		return;
	case i350_82575:
	case i350_82576:
	case i350_82580:
	case i350_i354:
		hw->nvm.ops.read(hw, NVM_ETRACK_HIWORD, 1, &etrack_test);
		/* Use this format, unless EETRACK ID exists,
		 * then use alternate format
		 */
		if ((etrack_test &  NVM_MAJOR_MASK) != NVM_ETRACK_VALID) {
			hw->nvm.ops.read(hw, NVM_VERSION, 1, &fw_version);
			fw_vers->eep_major = (fw_version & NVM_MAJOR_MASK)
					      >> NVM_MAJOR_SHIFT;
			fw_vers->eep_minor = (fw_version & NVM_MINOR_MASK)
					      >> NVM_MINOR_SHIFT;
			fw_vers->eep_build = (fw_version & NVM_IMAGE_ID_MASK);
			goto etrack_id;
		}
		break;
	case i350_i210:
		if (!(i350_get_flash_presence_i210(hw))) {
			i350_read_invm_version(hw, fw_vers);
			return;
		}
		/* fall through */
	case i350_i350:
		hw->nvm.ops.read(hw, NVM_ETRACK_HIWORD, 1, &etrack_test);
		/* find combo image version */
		hw->nvm.ops.read(hw, NVM_COMB_VER_PTR, 1, &comb_offset);
		if ((comb_offset != 0x0) &&
		    (comb_offset != NVM_VER_INVALID)) {

			hw->nvm.ops.read(hw, (NVM_COMB_VER_OFF + comb_offset
					 + 1), 1, &comb_verh);
			hw->nvm.ops.read(hw, (NVM_COMB_VER_OFF + comb_offset),
					 1, &comb_verl);

			/* get Option Rom version if it exists and is valid */
			if ((comb_verh && comb_verl) &&
			    ((comb_verh != NVM_VER_INVALID) &&
			     (comb_verl != NVM_VER_INVALID))) {

				fw_vers->or_valid = true;
				fw_vers->or_major =
					comb_verl >> NVM_COMB_VER_SHFT;
				fw_vers->or_build =
					(comb_verl << NVM_COMB_VER_SHFT)
					| (comb_verh >> NVM_COMB_VER_SHFT);
				fw_vers->or_patch =
					comb_verh & NVM_COMB_VER_MASK;
			}
		}
		break;
	default:
		hw->nvm.ops.read(hw, NVM_ETRACK_HIWORD, 1, &etrack_test);
		return;
	}
	hw->nvm.ops.read(hw, NVM_VERSION, 1, &fw_version);
	fw_vers->eep_major = (fw_version & NVM_MAJOR_MASK)
			      >> NVM_MAJOR_SHIFT;

	/* check for old style version format in newer images*/
	if ((fw_version & NVM_NEW_DEC_MASK) == 0x0) {
		eeprom_verl = (fw_version & NVM_COMB_VER_MASK);
	} else {
		eeprom_verl = (fw_version & NVM_MINOR_MASK)
				>> NVM_MINOR_SHIFT;
	}
	/* Convert minor value to hex before assigning to output struct
	 * Val to be converted will not be higher than 99, per tool output
	 */
	q = eeprom_verl / NVM_HEX_CONV;
	hval = q * NVM_HEX_TENS;
	rem = eeprom_verl % NVM_HEX_CONV;
	result = hval + rem;
	fw_vers->eep_minor = result;

etrack_id:
	if ((etrack_test &  NVM_MAJOR_MASK) == NVM_ETRACK_VALID) {
		hw->nvm.ops.read(hw, NVM_ETRACK_WORD, 1, &eeprom_verl);
		hw->nvm.ops.read(hw, (NVM_ETRACK_WORD + 1), 1, &eeprom_verh);
		fw_vers->etrack_id = (eeprom_verh << NVM_ETRACK_SHIFT)
			| eeprom_verl;
	} else if ((etrack_test & NVM_ETRACK_VALID) == 0) {
		hw->nvm.ops.read(hw, NVM_ETRACK_WORD, 1, &eeprom_verh);
		hw->nvm.ops.read(hw, (NVM_ETRACK_WORD + 1), 1, &eeprom_verl);
		fw_vers->etrack_id = (eeprom_verh << NVM_ETRACK_SHIFT) |
				     eeprom_verl;
	}
}


