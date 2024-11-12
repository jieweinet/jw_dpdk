/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2001-2020 Intel Corporation
 */

#ifndef _I350_API_H_
#define _I350_API_H_

#include "i350_hw.h"

extern void i350_init_function_pointers_82575(struct i350_hw *hw);
extern void i350_init_function_pointers_vf(struct i350_hw *hw);
extern void i350_power_up_fiber_serdes_link(struct i350_hw *hw);
extern void i350_shutdown_fiber_serdes_link(struct i350_hw *hw);
extern void i350_init_function_pointers_i210(struct i350_hw *hw);

s32 i350_set_obff_timer(struct i350_hw *hw, u32 itr);
s32 i350_set_mac_type(struct i350_hw *hw);
s32 i350_setup_init_funcs(struct i350_hw *hw, bool init_device);
s32 i350_init_mac_params(struct i350_hw *hw);
s32 i350_init_nvm_params(struct i350_hw *hw);
s32 i350_init_phy_params(struct i350_hw *hw);
s32 i350_init_mbx_params(struct i350_hw *hw);
s32 i350_get_bus_info(struct i350_hw *hw);
void i350_clear_vfta(struct i350_hw *hw);
void i350_write_vfta(struct i350_hw *hw, u32 offset, u32 value);
s32 i350_force_mac_fc(struct i350_hw *hw);
s32 i350_check_for_link(struct i350_hw *hw);
s32 i350_reset_hw(struct i350_hw *hw);
s32 i350_init_hw(struct i350_hw *hw);
s32 i350_setup_link(struct i350_hw *hw);
s32 i350_get_speed_and_duplex(struct i350_hw *hw, u16 *speed, u16 *duplex);
s32 i350_disable_pcie_master(struct i350_hw *hw);
void i350_config_collision_dist(struct i350_hw *hw);
int i350_rar_set(struct i350_hw *hw, u8 *addr, u32 index);
u32 i350_hash_mc_addr(struct i350_hw *hw, u8 *mc_addr);
void i350_update_mc_addr_list(struct i350_hw *hw, u8 *mc_addr_list,
			       u32 mc_addr_count);
s32 i350_setup_led(struct i350_hw *hw);
s32 i350_cleanup_led(struct i350_hw *hw);
s32 i350_check_reset_block(struct i350_hw *hw);
s32 i350_blink_led(struct i350_hw *hw);
s32 i350_led_on(struct i350_hw *hw);
s32 i350_led_off(struct i350_hw *hw);
s32 i350_id_led_init(struct i350_hw *hw);
void i350_reset_adaptive(struct i350_hw *hw);
void i350_update_adaptive(struct i350_hw *hw);
s32 i350_get_cable_length(struct i350_hw *hw);
s32 i350_validate_mdi_setting(struct i350_hw *hw);
s32 i350_read_phy_reg(struct i350_hw *hw, u32 offset, u16 *data);
s32 i350_write_phy_reg(struct i350_hw *hw, u32 offset, u16 data);
s32 i350_write_8bit_ctrl_reg(struct i350_hw *hw, u32 reg, u32 offset,
			      u8 data);
s32 i350_get_phy_info(struct i350_hw *hw);
void i350_release_phy(struct i350_hw *hw);
s32 i350_acquire_phy(struct i350_hw *hw);
s32 i350_cfg_on_link_up(struct i350_hw *hw);
s32 i350_phy_hw_reset(struct i350_hw *hw);
s32 i350_phy_commit(struct i350_hw *hw);
void i350_power_up_phy(struct i350_hw *hw);
void i350_power_down_phy(struct i350_hw *hw);
s32 i350_read_mac_addr(struct i350_hw *hw);
s32 i350_read_pba_num(struct i350_hw *hw, u32 *part_num);
s32 i350_read_pba_string(struct i350_hw *hw, u8 *pba_num, u32 pba_num_size);
s32 i350_read_pba_length(struct i350_hw *hw, u32 *pba_num_size);
void i350_reload_nvm(struct i350_hw *hw);
s32 i350_update_nvm_checksum(struct i350_hw *hw);
s32 i350_validate_nvm_checksum(struct i350_hw *hw);
s32 i350_read_nvm(struct i350_hw *hw, u16 offset, u16 words, u16 *data);
s32 i350_read_kmrn_reg(struct i350_hw *hw, u32 offset, u16 *data);
s32 i350_write_kmrn_reg(struct i350_hw *hw, u32 offset, u16 data);
s32 i350_write_nvm(struct i350_hw *hw, u16 offset, u16 words, u16 *data);
s32 i350_set_d3_lplu_state(struct i350_hw *hw, bool active);
s32 i350_set_d0_lplu_state(struct i350_hw *hw, bool active);
bool i350_check_mng_mode(struct i350_hw *hw);
bool i350_enable_tx_pkt_filtering(struct i350_hw *hw);
s32 i350_mng_enable_host_if(struct i350_hw *hw);
s32 i350_mng_host_if_write(struct i350_hw *hw, u8 *buffer, u16 length,
			    u16 offset, u8 *sum);
s32 i350_mng_write_cmd_header(struct i350_hw *hw,
			       struct i350_host_mng_command_header *hdr);
s32 i350_mng_write_dhcp_info(struct i350_hw *hw, u8 *buffer, u16 length);
u32  i350_translate_register_82542(u32 reg);



/*
 * TBI_ACCEPT macro definition:
 *
 * This macro requires:
 *      a = a pointer to struct i350_hw
 *      status = the 8 bit status field of the Rx descriptor with EOP set
 *      errors = the 8 bit error field of the Rx descriptor with EOP set
 *      length = the sum of all the length fields of the Rx descriptors that
 *               make up the current frame
 *      last_byte = the last byte of the frame DMAed by the hardware
 *      min_frame_size = the minimum frame length we want to accept.
 *      max_frame_size = the maximum frame length we want to accept.
 *
 * This macro is a conditional that should be used in the interrupt
 * handler's Rx processing routine when RxErrors have been detected.
 *
 * Typical use:
 *  ...
 *  if (TBI_ACCEPT) {
 *      accept_frame = true;
 *      i350_tbi_adjust_stats(adapter, MacAddress);
 *      frame_length--;
 *  } else {
 *      accept_frame = false;
 *  }
 *  ...
 */

/* The carrier extension symbol, as received by the NIC. */
#define CARRIER_EXTENSION   0x0F

#define TBI_ACCEPT(a, status, errors, length, last_byte, \
		   min_frame_size, max_frame_size) \
	(i350_tbi_sbp_enabled_82543(a) && \
	 (((errors) & I350_RXD_ERR_FRAME_ERR_MASK) == I350_RXD_ERR_CE) && \
	 ((last_byte) == CARRIER_EXTENSION) && \
	 (((status) & I350_RXD_STAT_VP) ? \
	  (((length) > ((min_frame_size) - VLAN_TAG_SIZE)) && \
	  ((length) <= ((max_frame_size) + 1))) : \
	  (((length) > (min_frame_size)) && \
	  ((length) <= ((max_frame_size) + VLAN_TAG_SIZE + 1)))))

#define I350_MAX(a, b) ((a) > (b) ? (a) : (b))
#define I350_DIVIDE_ROUND_UP(a, b)	(((a) + (b) - 1) / (b)) /* ceil(a/b) */
#endif /* _I350_API_H_ */
