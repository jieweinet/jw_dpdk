/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2001-2020 Intel Corporation
 */

#ifndef _I350_HW_H_
#define _I350_HW_H_

#include "i350_osdep.h"
#include "i350_regs.h"
#include "i350_defines.h"

struct i350_hw;

#define I350_DEV_ID_I350_COPPER		0x1521
#define I350_DEV_ID_I350_FIBER			0x1522
#define I350_DEV_ID_I350_SERDES		0x1523
#define I350_DEV_ID_I350_SGMII			0x1524
#define I350_DEV_ID_I350_DA4			0x1546

#define I350_REVISION_0	0
#define I350_REVISION_1	1
#define I350_REVISION_2	2
#define I350_REVISION_3	3
#define I350_REVISION_4	4

#define I350_FUNC_0		0
#define I350_FUNC_1		1
#define I350_FUNC_2		2
#define I350_FUNC_3		3

#define I350_ALT_MAC_ADDRESS_OFFSET_LAN0	0
#define I350_ALT_MAC_ADDRESS_OFFSET_LAN1	3
#define I350_ALT_MAC_ADDRESS_OFFSET_LAN2	6
#define I350_ALT_MAC_ADDRESS_OFFSET_LAN3	9

enum i350_mac_type {
	i350_i350,
	i350_vfadapt,
	i350_vfadapt_i350,
	i350_num_macs  /* List is 1-based, so subtract 1 for true count. */
};

enum i350_media_type {
	i350_media_type_unknown = 0,
	i350_media_type_copper = 1,
	i350_media_type_fiber = 2,
	i350_media_type_internal_serdes = 3,
	i350_num_media_types
};

enum i350_nvm_type {
	i350_nvm_unknown = 0,
	i350_nvm_none,
	i350_nvm_eeprom_spi,
	i350_nvm_eeprom_microwire,
	i350_nvm_flash_hw,
	i350_nvm_invm,
	i350_nvm_flash_sw
};

enum i350_nvm_override {
	i350_nvm_override_none = 0,
	i350_nvm_override_spi_small,
	i350_nvm_override_spi_large,
	i350_nvm_override_microwire_small,
	i350_nvm_override_microwire_large
};

enum i350_phy_type {
	i350_phy_82580,
	i350_phy_vf,
};

enum i350_bus_type {
	i350_bus_type_unknown = 0,
	i350_bus_type_pci_express,
	i350_bus_type_reserved
};

enum i350_bus_speed {
	i350_bus_speed_unknown = 0,
	i350_bus_speed_33,
	i350_bus_speed_66,
	i350_bus_speed_100,
	i350_bus_speed_120,
	i350_bus_speed_133,
	i350_bus_speed_2500,
	i350_bus_speed_5000,
	i350_bus_speed_reserved
};

enum i350_bus_width {
	i350_bus_width_unknown = 0,
	i350_bus_width_pcie_x1,
	i350_bus_width_pcie_x2,
	i350_bus_width_pcie_x4 = 4,
	i350_bus_width_pcie_x8 = 8,
	i350_bus_width_32,
	i350_bus_width_64,
	i350_bus_width_reserved
};

enum i350_1000t_rx_status {
	i350_1000t_rx_status_not_ok = 0,
	i350_1000t_rx_status_ok,
	i350_1000t_rx_status_undefined = 0xFF
};

enum i350_rev_polarity {
	i350_rev_polarity_normal = 0,
	i350_rev_polarity_reversed,
	i350_rev_polarity_undefined = 0xFF
};

enum i350_fc_mode {
	i350_fc_none = 0,
	i350_fc_rx_pause,
	i350_fc_tx_pause,
	i350_fc_full,
	i350_fc_default = 0xFF
};

enum i350_ffe_config {
	i350_ffe_config_enabled = 0,
	i350_ffe_config_active,
	i350_ffe_config_blocked
};

enum i350_dsp_config {
	i350_dsp_config_disabled = 0,
	i350_dsp_config_enabled,
	i350_dsp_config_activated,
	i350_dsp_config_undefined = 0xFF
};

enum i350_ms_type {
	i350_ms_hw_default = 0,
	i350_ms_force_master,
	i350_ms_force_slave,
	i350_ms_auto
};

enum i350_smart_speed {
	i350_smart_speed_default = 0,
	i350_smart_speed_on,
	i350_smart_speed_off
};

enum i350_serdes_link_state {
	i350_serdes_link_down = 0,
	i350_serdes_link_autoneg_progress,
	i350_serdes_link_autoneg_complete,
	i350_serdes_link_forced_up
};

#define __le16 u16
#define __le32 u32
#define __le64 u64
/* Receive Descriptor */
struct i350_rx_desc {
	__le64 buffer_addr; /* Address of the descriptor's data buffer */
	__le16 length;      /* Length of data DMAed into data buffer */
	__le16 csum; /* Packet checksum */
	u8  status;  /* Descriptor status */
	u8  errors;  /* Descriptor Errors */
	__le16 special;
};

/* Receive Descriptor - Extended */
union i350_rx_desc_extended {
	struct {
		__le64 buffer_addr;
		__le64 reserved;
	} read;
	struct {
		struct {
			__le32 mrq; /* Multiple Rx Queues */
			union {
				__le32 rss; /* RSS Hash */
				struct {
					__le16 ip_id;  /* IP id */
					__le16 csum;   /* Packet Checksum */
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			__le32 status_error;  /* ext status/error */
			__le16 length;
			__le16 vlan; /* VLAN tag */
		} upper;
	} wb;  /* writeback */
};

#define MAX_PS_BUFFERS 4

/* Number of packet split data buffers (not including the header buffer) */
#define PS_PAGE_BUFFERS	(MAX_PS_BUFFERS - 1)

/* Receive Descriptor - Packet Split */
union i350_rx_desc_packet_split {
	struct {
		/* one buffer for protocol header(s), three data buffers */
		__le64 buffer_addr[MAX_PS_BUFFERS];
	} read;
	struct {
		struct {
			__le32 mrq;  /* Multiple Rx Queues */
			union {
				__le32 rss; /* RSS Hash */
				struct {
					__le16 ip_id;    /* IP id */
					__le16 csum;     /* Packet Checksum */
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			__le32 status_error;  /* ext status/error */
			__le16 length0;  /* length of buffer 0 */
			__le16 vlan;  /* VLAN tag */
		} middle;
		struct {
			__le16 header_status;
			/* length of buffers 1-3 */
			__le16 length[PS_PAGE_BUFFERS];
		} upper;
		__le64 reserved;
	} wb; /* writeback */
};

/* Transmit Descriptor */
struct i350_tx_desc {
	__le64 buffer_addr;   /* Address of the descriptor's data buffer */
	union {
		__le32 data;
		struct {
			__le16 length;  /* Data buffer length */
			u8 cso;  /* Checksum offset */
			u8 cmd;  /* Descriptor control */
		} flags;
	} lower;
	union {
		__le32 data;
		struct {
			u8 status; /* Descriptor status */
			u8 css;  /* Checksum start */
			__le16 special;
		} fields;
	} upper;
};

/* Offload Context Descriptor */
struct i350_context_desc {
	union {
		__le32 ip_config;
		struct {
			u8 ipcss;  /* IP checksum start */
			u8 ipcso;  /* IP checksum offset */
			__le16 ipcse;  /* IP checksum end */
		} ip_fields;
	} lower_setup;
	union {
		__le32 tcp_config;
		struct {
			u8 tucss;  /* TCP checksum start */
			u8 tucso;  /* TCP checksum offset */
			__le16 tucse;  /* TCP checksum end */
		} tcp_fields;
	} upper_setup;
	__le32 cmd_and_length;
	union {
		__le32 data;
		struct {
			u8 status;  /* Descriptor status */
			u8 hdr_len;  /* Header length */
			__le16 mss;  /* Maximum segment size */
		} fields;
	} tcp_seg_setup;
};

/* Offload data descriptor */
struct i350_data_desc {
	__le64 buffer_addr;  /* Address of the descriptor's buffer address */
	union {
		__le32 data;
		struct {
			__le16 length;  /* Data buffer length */
			u8 typ_len_ext;
			u8 cmd;
		} flags;
	} lower;
	union {
		__le32 data;
		struct {
			u8 status;  /* Descriptor status */
			u8 popts;  /* Packet Options */
			__le16 special;
		} fields;
	} upper;
};

/* Statistics counters collected by the MAC */
struct i350_hw_stats {
	u64 crcerrs;
	u64 algnerrc;
	u64 symerrs;
	u64 rxerrc;
	u64 mpc;
	u64 scc;
	u64 ecol;
	u64 mcc;
	u64 latecol;
	u64 colc;
	u64 dc;
	u64 tncrs;
	u64 sec;
	u64 cexterr;
	u64 rlec;
	u64 xonrxc;
	u64 xontxc;
	u64 xoffrxc;
	u64 xofftxc;
	u64 fcruc;
	u64 prc64;
	u64 prc127;
	u64 prc255;
	u64 prc511;
	u64 prc1023;
	u64 prc1522;
	u64 gprc;
	u64 bprc;
	u64 mprc;
	u64 gptc;
	u64 gorc;
	u64 gotc;
	u64 rnbc;
	u64 ruc;
	u64 rfc;
	u64 roc;
	u64 rjc;
	u64 mgprc;
	u64 mgpdc;
	u64 mgptc;
	u64 tor;
	u64 tot;
	u64 tpr;
	u64 tpt;
	u64 ptc64;
	u64 ptc127;
	u64 ptc255;
	u64 ptc511;
	u64 ptc1023;
	u64 ptc1522;
	u64 mptc;
	u64 bptc;
	u64 tsctc;
	u64 tsctfc;
	u64 iac;
	u64 icrxptc;
	u64 icrxatc;
	u64 ictxptc;
	u64 ictxatc;
	u64 ictxqec;
	u64 ictxqmtc;
	u64 icrxdmtc;
	u64 icrxoc;
	u64 cbtmpc;
	u64 htdpmc;
	u64 cbrdpc;
	u64 cbrmpc;
	u64 rpthc;
	u64 hgptc;
	u64 htcbdpc;
	u64 hgorc;
	u64 hgotc;
	u64 lenerrs;
	u64 scvpc;
	u64 hrmpc;
	u64 doosync;
	u64 o2bgptc;
	u64 o2bspc;
	u64 b2ospc;
	u64 b2ogprc;
};

struct i350_vf_stats {
	u64 base_gprc;
	u64 base_gptc;
	u64 base_gorc;
	u64 base_gotc;
	u64 base_mprc;
	u64 base_gotlbc;
	u64 base_gptlbc;
	u64 base_gorlbc;
	u64 base_gprlbc;

	u32 last_gprc;
	u32 last_gptc;
	u32 last_gorc;
	u32 last_gotc;
	u32 last_mprc;
	u32 last_gotlbc;
	u32 last_gptlbc;
	u32 last_gorlbc;
	u32 last_gprlbc;

	u64 gprc;
	u64 gptc;
	u64 gorc;
	u64 gotc;
	u64 mprc;
	u64 gotlbc;
	u64 gptlbc;
	u64 gorlbc;
	u64 gprlbc;
};

struct i350_phy_stats {
	u32 idle_errors;
	u32 receive_errors;
};

struct i350_host_mng_dhcp_cookie {
	u32 signature;
	u8  status;
	u8  reserved0;
	u16 vlan_id;
	u32 reserved1;
	u16 reserved2;
	u8  reserved3;
	u8  checksum;
};

/* Host Interface "Rev 1" */
struct i350_host_command_header {
	u8 command_id;
	u8 command_length;
	u8 command_options;
	u8 checksum;
};

#define I350_HI_MAX_DATA_LENGTH	252
struct i350_host_command_info {
	struct i350_host_command_header command_header;
	u8 command_data[I350_HI_MAX_DATA_LENGTH];
};

/* Host Interface "Rev 2" */
struct i350_host_mng_command_header {
	u8  command_id;
	u8  checksum;
	u16 reserved1;
	u16 reserved2;
	u16 command_length;
};

#define I350_HI_MAX_MNG_DATA_LENGTH	0x6F8
struct i350_host_mng_command_info {
	struct i350_host_mng_command_header command_header;
	u8 command_data[I350_HI_MAX_MNG_DATA_LENGTH];
};

#include "i350_mac.h"
#include "i350_phy.h"
#include "i350_nvm.h"
#include "i350_manage.h"
#include "i350_mbx.h"

/* Function pointers for the MAC. */
struct i350_mac_operations {
	s32  (*init_params)(struct i350_hw *);
	s32  (*id_led_init)(struct i350_hw *);
	s32  (*blink_led)(struct i350_hw *);
	bool (*check_mng_mode)(struct i350_hw *);
	s32  (*check_for_link)(struct i350_hw *);
	s32  (*cleanup_led)(struct i350_hw *);
	void (*clear_hw_cntrs)(struct i350_hw *);
	void (*clear_vfta)(struct i350_hw *);
	s32  (*get_bus_info)(struct i350_hw *);
	void (*set_lan_id)(struct i350_hw *);
	s32  (*get_link_up_info)(struct i350_hw *, u16 *, u16 *);
	s32  (*led_on)(struct i350_hw *);
	s32  (*led_off)(struct i350_hw *);
	void (*update_mc_addr_list)(struct i350_hw *, u8 *, u32);
	s32  (*reset_hw)(struct i350_hw *);
	s32  (*init_hw)(struct i350_hw *);
	void (*shutdown_serdes)(struct i350_hw *);
	void (*power_up_serdes)(struct i350_hw *);
	s32  (*setup_link)(struct i350_hw *);
	s32  (*setup_physical_interface)(struct i350_hw *);
	s32  (*setup_led)(struct i350_hw *);
	void (*write_vfta)(struct i350_hw *, u32, u32);
	void (*config_collision_dist)(struct i350_hw *);
	int  (*rar_set)(struct i350_hw *, u8*, u32);
	s32  (*read_mac_addr)(struct i350_hw *);
	s32  (*validate_mdi_setting)(struct i350_hw *);
	s32  (*acquire_swfw_sync)(struct i350_hw *, u16);
	void (*release_swfw_sync)(struct i350_hw *, u16);
};

/* When to use various PHY register access functions:
 *
 *                 Func   Caller
 *   Function      Does   Does    When to use
 *   ~~~~~~~~~~~~  ~~~~~  ~~~~~~  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *   X_reg         L,P,A  n/a     for simple PHY reg accesses
 *   X_reg_locked  P,A    L       for multiple accesses of different regs
 *                                on different pages
 *   X_reg_page    A      L,P     for multiple accesses of different regs
 *                                on the same page
 *
 * Where X=[read|write], L=locking, P=sets page, A=register access
 *
 */
struct i350_phy_operations {
	s32  (*init_params)(struct i350_hw *);
	s32  (*acquire)(struct i350_hw *);
	s32  (*cfg_on_link_up)(struct i350_hw *);
	s32  (*check_polarity)(struct i350_hw *);
	s32  (*check_reset_block)(struct i350_hw *);
	s32  (*commit)(struct i350_hw *);
	s32  (*force_speed_duplex)(struct i350_hw *);
	s32  (*get_cfg_done)(struct i350_hw *hw);
	s32  (*get_cable_length)(struct i350_hw *);
	s32  (*get_info)(struct i350_hw *);
	s32  (*set_page)(struct i350_hw *, u16);
	s32  (*read_reg)(struct i350_hw *, u32, u16 *);
	s32  (*read_reg_locked)(struct i350_hw *, u32, u16 *);
	s32  (*read_reg_page)(struct i350_hw *, u32, u16 *);
	void (*release)(struct i350_hw *);
	s32  (*reset)(struct i350_hw *);
	s32  (*set_d0_lplu_state)(struct i350_hw *, bool);
	s32  (*set_d3_lplu_state)(struct i350_hw *, bool);
	s32  (*write_reg)(struct i350_hw *, u32, u16);
	s32  (*write_reg_locked)(struct i350_hw *, u32, u16);
	s32  (*write_reg_page)(struct i350_hw *, u32, u16);
	void (*power_up)(struct i350_hw *);
	void (*power_down)(struct i350_hw *);
	s32 (*read_i2c_byte)(struct i350_hw *, u8, u8, u8 *);
	s32 (*write_i2c_byte)(struct i350_hw *, u8, u8, u8);
};

/* Function pointers for the NVM. */
struct i350_nvm_operations {
	s32  (*init_params)(struct i350_hw *);
	s32  (*acquire)(struct i350_hw *);
	s32  (*read)(struct i350_hw *, u16, u16, u16 *);
	void (*release)(struct i350_hw *);
	void (*reload)(struct i350_hw *);
	s32  (*update)(struct i350_hw *);
	s32  (*valid_led_default)(struct i350_hw *, u16 *);
	s32  (*validate)(struct i350_hw *);
	s32  (*write)(struct i350_hw *, u16, u16, u16 *);
};

struct i350_mac_info {
	struct i350_mac_operations ops;
	u8 addr[ETH_ADDR_LEN];
	u8 perm_addr[ETH_ADDR_LEN];

	enum i350_mac_type type;

	u32 collision_delta;
	u32 ledctl_default;
	u32 ledctl_mode1;
	u32 ledctl_mode2;
	u32 mc_filter_type;
	u32 tx_packet_delta;
	u32 txcw;

	u16 current_ifs_val;
	u16 ifs_max_val;
	u16 ifs_min_val;
	u16 ifs_ratio;
	u16 ifs_step_size;
	u16 mta_reg_count;
	u16 uta_reg_count;

	/* Maximum size of the MTA register table in all supported adapters */
#define MAX_MTA_REG 128
	u32 mta_shadow[MAX_MTA_REG];
	u16 rar_entry_count;

	u8  forced_speed_duplex;

	bool adaptive_ifs;
	bool has_fwsm;
	bool arc_subsystem_valid;
	bool asf_firmware_present;
	bool autoneg;
	bool autoneg_failed;
	bool get_link_status;
	bool in_ifs_mode;
	bool report_tx_early;
	enum i350_serdes_link_state serdes_link_state;
	bool serdes_has_link;
	bool tx_pkt_filtering;
};

struct i350_phy_info {
	struct i350_phy_operations ops;
	enum i350_phy_type type;

	enum i350_1000t_rx_status local_rx;
	enum i350_1000t_rx_status remote_rx;
	enum i350_ms_type ms_type;
	enum i350_ms_type original_ms_type;
	enum i350_rev_polarity cable_polarity;
	enum i350_smart_speed smart_speed;

	u32 addr;
	u32 id;
	u32 reset_delay_us; /* in usec */
	u32 revision;

	enum i350_media_type media_type;

	u16 autoneg_advertised;
	u16 autoneg_mask;
	u16 cable_length;
	u16 max_cable_length;
	u16 min_cable_length;

	u8 mdix;

	bool disable_polarity_correction;
	bool is_mdix;
	bool polarity_correction;
	bool speed_downgraded;
	bool autoneg_wait_to_complete;
};

struct i350_nvm_info {
	struct i350_nvm_operations ops;
	enum i350_nvm_type type;
	enum i350_nvm_override override;

	u32 flash_bank_size;
	u32 flash_base_addr;

	u16 word_size;
	u16 delay_usec;
	u16 address_bits;
	u16 opcode_bits;
	u16 page_size;
};

struct i350_bus_info {
	enum i350_bus_type type;
	enum i350_bus_speed speed;
	enum i350_bus_width width;

	u16 func;
	u16 pci_cmd_word;
};

struct i350_fc_info {
	u32 high_water;  /* Flow control high-water mark */
	u32 low_water;  /* Flow control low-water mark */
	u16 pause_time;  /* Flow control pause timer */
	u16 refresh_time;  /* Flow control refresh timer */
	bool send_xon;  /* Flow control send XON */
	bool strict_ieee;  /* Strict IEEE mode */
	enum i350_fc_mode current_mode;  /* FC mode in effect */
	enum i350_fc_mode requested_mode;  /* FC mode requested by caller */
};

struct i350_shadow_ram {
	u16  value;
	bool modified;
};

#define I350_SHADOW_RAM_WORDS		2048

/* I218 PHY Ultra Low Power (ULP) states */
enum i350_ulp_state {
	i350_ulp_state_unknown,
	i350_ulp_state_off,
	i350_ulp_state_on,
};

struct i350_dev_spec_ich8lan {
	bool kmrn_lock_loss_workaround_enabled;
	struct i350_shadow_ram shadow_ram[I350_SHADOW_RAM_WORDS];
	I350_MUTEX nvm_mutex;
	I350_MUTEX swflag_mutex;
	bool nvm_k1_enabled;
	bool disable_k1_off;
	bool eee_disable;
	u16 eee_lp_ability;
	enum i350_ulp_state ulp_state;
	bool ulp_capability_disabled;
	bool during_suspend_flow;
	bool during_dpg_exit;
	u16 lat_enc;
	u16 max_ltr_enc;
	bool smbus_disable;
};

struct i350_dev_spec_82575 {
	bool sgmii_active;
	bool global_device_reset;
	bool eee_disable;
	bool module_plugged;
	bool clear_semaphore_once;
	u32 mtu;
	struct sfp_i350_flags eth_flags;
	u8 media_port;
	bool media_changed;
};

struct i350_dev_spec_vf {
	u32 vf_number;
	u32 v2p_mailbox;
};

struct i350_hw {
	void *back;

	u8 *hw_addr;
	u8 *flash_address;
	unsigned long io_base;

	struct i350_mac_info  mac;
	struct i350_fc_info   fc;
	struct i350_phy_info  phy;
	struct i350_nvm_info  nvm;
	struct i350_bus_info  bus;
	struct i350_mbx_info mbx;
	struct i350_host_mng_dhcp_cookie mng_cookie;

	union {
		struct i350_dev_spec_82541 _82541;
		struct i350_dev_spec_82542 _82542;
		struct i350_dev_spec_82543 _82543;
		struct i350_dev_spec_82571 _82571;
		struct i350_dev_spec_80003es2lan _80003es2lan;
		struct i350_dev_spec_ich8lan ich8lan;
		struct i350_dev_spec_82575 _82575;
		struct i350_dev_spec_vf vf;
	} dev_spec;

	u16 device_id;
	u16 subsystem_vendor_id;
	u16 subsystem_device_id;
	u16 vendor_id;

	u8  revision_id;
};

#include "i350_82541.h"
#include "i350_82543.h"
#include "i350_82571.h"
#include "i350_80003es2lan.h"
#include "i350_ich8lan.h"
#include "i350_82575.h"
#include "i350_base.h"

/* These functions must be implemented by drivers */
void i350_pci_clear_mwi(struct i350_hw *hw);
void i350_pci_set_mwi(struct i350_hw *hw);
s32  i350_read_pcie_cap_reg(struct i350_hw *hw, u32 reg, u16 *value);
s32  i350_write_pcie_cap_reg(struct i350_hw *hw, u32 reg, u16 *value);
void i350_read_pci_cfg(struct i350_hw *hw, u32 reg, u16 *value);
void i350_write_pci_cfg(struct i350_hw *hw, u32 reg, u16 *value);

#endif
