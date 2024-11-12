/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2001-2020 Intel Corporation
 */

#ifndef _I350_MANAGE_H_
#define _I350_MANAGE_H_

bool i350_check_mng_mode_generic(struct i350_hw *hw);
bool i350_enable_tx_pkt_filtering_generic(struct i350_hw *hw);
s32  i350_mng_enable_host_if_generic(struct i350_hw *hw);
s32  i350_mng_host_if_write_generic(struct i350_hw *hw, u8 *buffer,
				     u16 length, u16 offset, u8 *sum);
s32  i350_mng_write_cmd_header_generic(struct i350_hw *hw,
				     struct i350_host_mng_command_header *hdr);
s32  i350_mng_write_dhcp_info_generic(struct i350_hw *hw,
				       u8 *buffer, u16 length);
bool i350_enable_mng_pass_thru(struct i350_hw *hw);
u8 i350_calculate_checksum(u8 *buffer, u32 length);
s32 i350_host_interface_command(struct i350_hw *hw, u8 *buffer, u32 length);
s32 i350_load_firmware(struct i350_hw *hw, u8 *buffer, u32 length);

enum i350_mng_mode {
	i350_mng_mode_none = 0,
	i350_mng_mode_asf,
	i350_mng_mode_pt,
	i350_mng_mode_ipmi,
	i350_mng_mode_host_if_only
};

#define I350_FACTPS_MNGCG			0x20000000

#define I350_FWSM_MODE_MASK			0xE
#define I350_FWSM_MODE_SHIFT			1
#define I350_FWSM_FW_VALID			0x00008000
#define I350_FWSM_HI_EN_ONLY_MODE		0x4

#define I350_MNG_IAMT_MODE			0x3
#define I350_MNG_DHCP_COOKIE_LENGTH		0x10
#define I350_MNG_DHCP_COOKIE_OFFSET		0x6F0
#define I350_MNG_DHCP_COMMAND_TIMEOUT		10
#define I350_MNG_DHCP_TX_PAYLOAD_CMD		64
#define I350_MNG_DHCP_COOKIE_STATUS_PARSING	0x1
#define I350_MNG_DHCP_COOKIE_STATUS_VLAN	0x2

#define I350_VFTA_ENTRY_SHIFT			5
#define I350_VFTA_ENTRY_MASK			0x7F
#define I350_VFTA_ENTRY_BIT_SHIFT_MASK		0x1F

#define I350_HI_MAX_BLOCK_BYTE_LENGTH		1792 /* Num of bytes in range */
#define I350_HI_MAX_BLOCK_DWORD_LENGTH		448 /* Num of dwords in range */
#define I350_HI_COMMAND_TIMEOUT		500 /* Process HI cmd limit */
#define I350_HI_FW_BASE_ADDRESS		0x10000
#define I350_HI_FW_MAX_LENGTH			(64 * 1024) /* Num of bytes */
#define I350_HI_FW_BLOCK_DWORD_LENGTH		256 /* Num of DWORDs per page */
#define I350_HICR_MEMORY_BASE_EN		0x200 /* MB Enable bit - RO */
#define I350_HICR_EN			0x01  /* Enable bit - RO */
/* Driver sets this bit when done to put command in RAM */
#define I350_HICR_C			0x02
#define I350_HICR_SV			0x04  /* Status Validity */
#define I350_HICR_FW_RESET_ENABLE	0x40
#define I350_HICR_FW_RESET		0x80

/* Intel(R) Active Management Technology signature */
#define I350_IAMT_SIGNATURE		0x544D4149
#endif
