/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2010-2015 Intel Corporation
 */

#ifndef _I350_ETHDEV_H_
#define _I350_ETHDEV_H_

#include <stdint.h>
#include <sys/queue.h>

#include <rte_flow.h>
#include <rte_time.h>
#include <rte_pci.h>

#define I350_INTEL_VENDOR_ID 0x8086

/* need update link, bit flag */
#define I350_FLAG_NEED_LINK_UPDATE (uint32_t)(1 << 0)
#define I350_FLAG_MAILBOX          (uint32_t)(1 << 1)

/*
 * Defines that were not part of i350_hw.h as they are not used by the FreeBSD
 * driver.
 */
#define I350_ADVTXD_POPTS_TXSM     0x00000200 /* L4 Checksum offload request */
#define I350_ADVTXD_POPTS_IXSM     0x00000100 /* IP Checksum offload request */
#define I350_ADVTXD_TUCMD_L4T_RSV  0x00001800 /* L4 Packet TYPE of Reserved */
#define I350_RXD_STAT_TMST         0x10000    /* Timestamped Packet indication */
#define I350_RXD_ERR_CKSUM_BIT     29
#define I350_RXD_ERR_CKSUM_MSK     3
#define I350_ADVTXD_MACLEN_SHIFT   9          /* Bit shift for l2_len */
#define I350_CTRL_EXT_EXTEND_VLAN  (1<<26)    /* EXTENDED VLAN */
#define I350_VFTA_SIZE 128

#define I350_HKEY_MAX_INDEX             10
#define I350_MAX_RX_QUEUE_NUM           8
#define I350_MAX_RX_QUEUE_NUM_82576     16

#define I350_I219_MAX_RX_QUEUE_NUM		2
#define I350_I219_MAX_TX_QUEUE_NUM		2

#define I350_SYN_FILTER_ENABLE        0x00000001 /* syn filter enable field */
#define I350_SYN_FILTER_QUEUE         0x0000000E /* syn filter queue field */
#define I350_SYN_FILTER_QUEUE_SHIFT   1          /* syn filter queue field */
#define I350_RFCTL_SYNQFP             0x00080000 /* SYNQFP in RFCTL register */

#define I350_ETQF_ETHERTYPE           0x0000FFFF
#define I350_ETQF_QUEUE               0x00070000
#define I350_ETQF_QUEUE_SHIFT         16
#define I350_MAX_ETQF_FILTERS         8

#define I350_IMIR_DSTPORT             0x0000FFFF
#define I350_IMIR_PRIORITY            0xE0000000
#define I350_MAX_TTQF_FILTERS         8
#define I350_2TUPLE_MAX_PRI           7

#define I350_MAX_FLEX_FILTERS           8
#define I350_MAX_FHFT                   4
#define I350_MAX_FHFT_EXT               4
#define I350_FHFT_SIZE_IN_DWD           64
#define I350_MAX_FLEX_FILTER_PRI        7
#define I350_MAX_FLEX_FILTER_LEN        128
#define I350_MAX_FLEX_FILTER_DWDS \
	(I350_MAX_FLEX_FILTER_LEN / sizeof(uint32_t))
#define I350_FLEX_FILTERS_MASK_SIZE \
	(I350_MAX_FLEX_FILTER_DWDS / 2)
#define I350_FHFT_QUEUEING_LEN          0x0000007F
#define I350_FHFT_QUEUEING_QUEUE        0x00000700
#define I350_FHFT_QUEUEING_PRIO         0x00070000
#define I350_FHFT_QUEUEING_OFFSET       0xFC
#define I350_FHFT_QUEUEING_QUEUE_SHIFT  8
#define I350_FHFT_QUEUEING_PRIO_SHIFT   16
#define I350_WUFC_FLEX_HQ               0x00004000

#define I350_SPQF_SRCPORT               0x0000FFFF

#define I350_MAX_FTQF_FILTERS           8
#define I350_FTQF_PROTOCOL_MASK         0x000000FF
#define I350_FTQF_5TUPLE_MASK_SHIFT     28
#define I350_FTQF_QUEUE_MASK            0x03ff0000
#define I350_FTQF_QUEUE_SHIFT           16
#define I350_FTQF_QUEUE_ENABLE          0x00000100

#define I350_RSS_OFFLOAD_ALL ( \
	RTE_ETH_RSS_IPV4 | \
	RTE_ETH_RSS_NONFRAG_IPV4_TCP | \
	RTE_ETH_RSS_NONFRAG_IPV4_UDP | \
	RTE_ETH_RSS_IPV6 | \
	RTE_ETH_RSS_NONFRAG_IPV6_TCP | \
	RTE_ETH_RSS_NONFRAG_IPV6_UDP | \
	RTE_ETH_RSS_IPV6_EX | \
	RTE_ETH_RSS_IPV6_TCP_EX | \
	RTE_ETH_RSS_IPV6_UDP_EX)

/*
 * The overhead from MTU to max frame size.
 * Considering VLAN so a tag needs to be counted.
 */
#define I350_ETH_OVERHEAD (RTE_ETHER_HDR_LEN + RTE_ETHER_CRC_LEN + \
				VLAN_TAG_SIZE)
#define I350_ETH_MAX_LEN (RTE_ETHER_MTU + I350_ETH_OVERHEAD)
/*
 * Maximum number of Ring Descriptors.
 *
 * Since RDLEN/TDLEN should be multiple of 128 bytes, the number of ring
 * descriptors should meet the following condition:
 * (num_ring_desc * sizeof(struct i350_rx/tx_desc)) % 128 == 0
 */
#define	I350_MIN_RING_DESC	32
#define	I350_MAX_RING_DESC	4096

/*
 * TDBA/RDBA should be aligned on 16 byte boundary. But TDLEN/RDLEN should be
 * multiple of 128 bytes. So we align TDBA/RDBA on 128 byte boundary.
 * This will also optimize cache line size effect.
 * H/W supports up to cache line size 128.
 */
#define	I350_ALIGN	128

#define	I350_RXD_ALIGN	(I350_ALIGN / sizeof(union i350_adv_rx_desc))
#define	I350_TXD_ALIGN	(I350_ALIGN / sizeof(union i350_adv_tx_desc))

#define	EM_RXD_ALIGN	(I350_ALIGN / sizeof(struct i350_rx_desc))
#define	EM_TXD_ALIGN	(I350_ALIGN / sizeof(struct i350_data_desc))

#define I350_MISC_VEC_ID               RTE_INTR_VEC_ZERO_OFFSET
#define I350_RX_VEC_START              RTE_INTR_VEC_RXTX_OFFSET

#define I350_TX_MAX_SEG     UINT8_MAX
#define I350_TX_MAX_MTU_SEG UINT8_MAX
#define EM_TX_MAX_SEG      UINT8_MAX
#define EM_TX_MAX_MTU_SEG  UINT8_MAX

#define MAC_TYPE_FILTER_SUP(type)    do {\
	if ((type) != i350_82580 && (type) != i350_i350 &&\
		(type) != i350_82576 && (type) != i350_i210 &&\
		(type) != i350_i211)\
		return -ENOTSUP;\
} while (0)

#define MAC_TYPE_FILTER_SUP_EXT(type)    do {\
	if ((type) != i350_82580 && (type) != i350_i350 &&\
		(type) != i350_i210 && (type) != i350_i211)\
		return -ENOTSUP; \
} while (0)

/* structure for interrupt relative data */
struct i350_interrupt {
	uint32_t flags;
	uint32_t mask;
};

/* local vfta copy */
struct i350_vfta {
	uint32_t vfta[I350_VFTA_SIZE];
};

/*
 * VF data which used by PF host only
 */
#define I350_MAX_VF_MC_ENTRIES         30
struct i350_vf_info {
	uint8_t vf_mac_addresses[RTE_ETHER_ADDR_LEN];
	uint16_t vf_mc_hashes[I350_MAX_VF_MC_ENTRIES];
	uint16_t num_vf_mc_hashes;
	uint16_t default_vf_vlan_id;
	uint16_t vlans_enabled;
	uint16_t pf_qos;
	uint16_t vlan_count;
	uint16_t tx_rate;
};

TAILQ_HEAD(i350_flex_filter_list, i350_flex_filter);

struct i350_flex_filter_info {
	uint16_t len;
	uint32_t dwords[I350_MAX_FLEX_FILTER_DWDS]; /* flex bytes in dword. */
	/* if mask bit is 1b, do not compare corresponding byte in dwords. */
	uint8_t mask[I350_FLEX_FILTERS_MASK_SIZE];
	uint8_t priority;
};

/* Flex filter structure */
struct i350_flex_filter {
	TAILQ_ENTRY(i350_flex_filter) entries;
	uint16_t index; /* index of flex filter */
	struct i350_flex_filter_info filter_info;
	uint16_t queue; /* rx queue assigned to */
};

TAILQ_HEAD(i350_5tuple_filter_list, i350_5tuple_filter);
TAILQ_HEAD(i350_2tuple_filter_list, i350_2tuple_filter);

struct i350_5tuple_filter_info {
	uint32_t dst_ip;
	uint32_t src_ip;
	uint16_t dst_port;
	uint16_t src_port;
	uint8_t proto;           /* l4 protocol. */
	/* the packet matched above 5tuple and contain any set bit will hit this filter. */
	uint8_t tcp_flags;
	uint8_t priority;        /* seven levels (001b-111b), 111b is highest,
				      used when more than one filter matches. */
	uint8_t dst_ip_mask:1,   /* if mask is 1b, do not compare dst ip. */
		src_ip_mask:1,   /* if mask is 1b, do not compare src ip. */
		dst_port_mask:1, /* if mask is 1b, do not compare dst port. */
		src_port_mask:1, /* if mask is 1b, do not compare src port. */
		proto_mask:1;    /* if mask is 1b, do not compare protocol. */
};

struct i350_2tuple_filter_info {
	uint16_t dst_port;
	uint8_t proto;           /* l4 protocol. */
	/* the packet matched above 2tuple and contain any set bit will hit this filter. */
	uint8_t tcp_flags;
	uint8_t priority;        /* seven levels (001b-111b), 111b is highest,
				      used when more than one filter matches. */
	uint8_t dst_ip_mask:1,   /* if mask is 1b, do not compare dst ip. */
		src_ip_mask:1,   /* if mask is 1b, do not compare src ip. */
		dst_port_mask:1, /* if mask is 1b, do not compare dst port. */
		src_port_mask:1, /* if mask is 1b, do not compare src port. */
		proto_mask:1;    /* if mask is 1b, do not compare protocol. */
};

/* 5tuple filter structure */
struct i350_5tuple_filter {
	TAILQ_ENTRY(i350_5tuple_filter) entries;
	uint16_t index;       /* the index of 5tuple filter */
	struct i350_5tuple_filter_info filter_info;
	uint16_t queue;       /* rx queue assigned to */
};

/* 2tuple filter structure */
struct i350_2tuple_filter {
	TAILQ_ENTRY(i350_2tuple_filter) entries;
	uint16_t index;         /* the index of 2tuple filter */
	struct i350_2tuple_filter_info filter_info;
	uint16_t queue;       /* rx queue assigned to */
};

/* ethertype filter structure */
struct i350_ethertype_filter {
	uint16_t ethertype;
	uint32_t etqf;
};

struct i350_rte_flow_rss_conf {
	struct rte_flow_action_rss conf; /**< RSS parameters. */
	uint8_t key[I350_HKEY_MAX_INDEX * sizeof(uint32_t)]; /* Hash key. */
	/* Queues indices to use. */
	uint16_t queue[I350_MAX_RX_QUEUE_NUM_82576];
};

/*
 * Structure to store filters' info.
 */
struct i350_filter_info {
	uint8_t ethertype_mask; /* Bit mask for every used ethertype filter */
	/* store used ethertype filters*/
	struct i350_ethertype_filter ethertype_filters[I350_MAX_ETQF_FILTERS];
	uint8_t flex_mask;	/* Bit mask for every used flex filter */
	struct i350_flex_filter_list flex_list;
	/* Bit mask for every used 5tuple filter */
	uint8_t fivetuple_mask;
	struct i350_5tuple_filter_list fivetuple_list;
	/* Bit mask for every used 2tuple filter */
	uint8_t twotuple_mask;
	struct i350_2tuple_filter_list twotuple_list;
	/* store the SYN filter info */
	uint32_t syn_info;
	/* store the rss filter info */
	struct i350_rte_flow_rss_conf rss_info;
};

/*
 * Structure to store private data for each driver instance (for each port).
 */
struct i350_adapter {
	struct i350_hw         hw;
	struct i350_hw_stats   stats;
	struct i350_interrupt  intr;
	struct i350_vfta       shadow_vfta;
	struct i350_vf_info    *vfdata;
	struct i350_filter_info filter;
	bool stopped;
	struct rte_timecounter  systime_tc;
	struct rte_timecounter  rx_tstamp_tc;
	struct rte_timecounter  tx_tstamp_tc;
};

#define I350_DEV_PRIVATE(adapter) \
	((struct i350_adapter *)adapter)

#define I350_DEV_PRIVATE_TO_HW(adapter) \
	(&((struct i350_adapter *)adapter)->hw)

#define I350_DEV_PRIVATE_TO_STATS(adapter) \
	(&((struct i350_adapter *)adapter)->stats)

#define I350_DEV_PRIVATE_TO_INTR(adapter) \
	(&((struct i350_adapter *)adapter)->intr)

#define I350_DEV_PRIVATE_TO_VFTA(adapter) \
	(&((struct i350_adapter *)adapter)->shadow_vfta)

#define I350_DEV_PRIVATE_TO_P_VFDATA(adapter) \
        (&((struct i350_adapter *)adapter)->vfdata)

#define I350_DEV_PRIVATE_TO_FILTER_INFO(adapter) \
	(&((struct i350_adapter *)adapter)->filter)

struct rte_flow {
	enum rte_filter_type filter_type;
	void *rule;
};

/* ntuple filter list structure */
struct i350_ntuple_filter_ele {
	TAILQ_ENTRY(i350_ntuple_filter_ele) entries;
	struct rte_eth_ntuple_filter filter_info;
};

/* ethertype filter list structure */
struct i350_ethertype_filter_ele {
	TAILQ_ENTRY(i350_ethertype_filter_ele) entries;
	struct rte_eth_ethertype_filter filter_info;
};

/* syn filter list structure */
struct i350_eth_syn_filter_ele {
	TAILQ_ENTRY(i350_eth_syn_filter_ele) entries;
	struct rte_eth_syn_filter filter_info;
};

#define I350_FLEX_FILTER_MAXLEN	128	/**< bytes to use in flex filter. */
#define I350_FLEX_FILTER_MASK_SIZE	\
	(RTE_ALIGN(I350_FLEX_FILTER_MAXLEN, CHAR_BIT) / CHAR_BIT)
					/**< mask bytes in flex filter. */

/**
 * A structure used to define the flex filter entry
 * to support RTE_ETH_FILTER_FLEXIBLE data representation.
 */
struct i350_flex_filter {
	uint16_t len;
	uint8_t bytes[I350_FLEX_FILTER_MAXLEN]; /**< flex bytes in big endian. */
	uint8_t mask[I350_FLEX_FILTER_MASK_SIZE];
		/**< if mask bit is 1b, do not compare corresponding byte. */
	uint8_t priority;
	uint16_t queue;       /**< Queue assigned to when match. */
};

/* flex filter list structure */
struct i350_flex_filter_ele {
	TAILQ_ENTRY(i350_flex_filter_ele) entries;
	struct i350_flex_filter filter_info;
};

/* rss filter  list structure */
struct i350_rss_conf_ele {
	TAILQ_ENTRY(i350_rss_conf_ele) entries;
	struct i350_rte_flow_rss_conf filter_info;
};

/* igb_flow memory list structure */
struct i350_flow_mem {
	TAILQ_ENTRY(i350_flow_mem) entries;
	struct rte_flow *flow;
	struct rte_eth_dev *dev;
};

TAILQ_HEAD(i350_ntuple_filter_list, i350_ntuple_filter_ele);
extern struct i350_ntuple_filter_list i350_filter_ntuple_list;
TAILQ_HEAD(i350_ethertype_filter_list, i350_ethertype_filter_ele);
extern struct i350_ethertype_filter_list i350_filter_ethertype_list;
TAILQ_HEAD(i350_syn_filter_list, i350_eth_syn_filter_ele);
extern struct i350_syn_filter_list i350_filter_syn_list;
TAILQ_HEAD(i350_flex_filter_list, i350_flex_filter_ele);
extern struct i350_flex_filter_list i350_filter_flex_list;
TAILQ_HEAD(i350_rss_filter_list, i350_rss_conf_ele);
extern struct i350_rss_filter_list i350_filter_rss_list;
TAILQ_HEAD(i350_flow_mem_list, i350_flow_mem);
extern struct i350_flow_mem_list i350_flow_list;

/*
 * Macros to compensate the constant latency observed in i210 for launch time
 *
 * launch time = (offset_speed - offset_base + txtime) * 32
 * offset_speed is speed dependent, set in E1000_I210_LAUNCH_OS0
 */
#define I350_I210_TX_OFFSET_BASE				0xffe0
#define I350_I210_TX_OFFSET_SPEED_10			0xc7a0
#define I350_I210_TX_OFFSET_SPEED_100		0x86e0
#define I350_I210_TX_OFFSET_SPEED_1000		0xbe00

extern uint64_t i350_tx_timestamp_dynflag;
extern int i350_tx_timestamp_dynfield_offset;

extern const struct rte_flow_ops i350_flow_ops;

/*
 * RX/TX IGB function prototypes
 */
void eth_i350_tx_queue_release(struct rte_eth_dev *dev, uint16_t qid);
void eth_i350_rx_queue_release(struct rte_eth_dev *dev, uint16_t qid);
void i350_dev_clear_queues(struct rte_eth_dev *dev);
void i350_dev_free_queues(struct rte_eth_dev *dev);

uint64_t i350_get_rx_port_offloads_capa(struct rte_eth_dev *dev);
uint64_t i350_get_rx_queue_offloads_capa(struct rte_eth_dev *dev);

int eth_i350_rx_queue_setup(struct rte_eth_dev *dev, uint16_t rx_queue_id,
		uint16_t nb_rx_desc, unsigned int socket_id,
		const struct rte_eth_rxconf *rx_conf,
		struct rte_mempool *mb_pool);

uint32_t eth_i350_rx_queue_count(void *rx_queue);

int eth_i350_rx_descriptor_status(void *rx_queue, uint16_t offset);
int eth_i350_tx_descriptor_status(void *tx_queue, uint16_t offset);

uint64_t i350_get_tx_port_offloads_capa(struct rte_eth_dev *dev);
uint64_t i350_get_tx_queue_offloads_capa(struct rte_eth_dev *dev);

int eth_i350_tx_queue_setup(struct rte_eth_dev *dev, uint16_t tx_queue_id,
		uint16_t nb_tx_desc, unsigned int socket_id,
		const struct rte_eth_txconf *tx_conf);

int eth_i350_tx_done_cleanup(void *txq, uint32_t free_cnt);

int eth_i350_rx_init(struct rte_eth_dev *dev);

void eth_i350_tx_init(struct rte_eth_dev *dev);

uint16_t eth_i350_xmit_pkts(void *txq, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts);

uint16_t eth_i350_prep_pkts(void *txq, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts);

uint16_t eth_i350_recv_pkts(void *rxq, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts);

uint16_t eth_i350_recv_scattered_pkts(void *rxq,
		struct rte_mbuf **rx_pkts, uint16_t nb_pkts);

int eth_i350_rss_hash_update(struct rte_eth_dev *dev,
			    struct rte_eth_rss_conf *rss_conf);

int eth_i350_rss_hash_conf_get(struct rte_eth_dev *dev,
			      struct rte_eth_rss_conf *rss_conf);

int eth_i350vf_rx_init(struct rte_eth_dev *dev);

void eth_i350vf_tx_init(struct rte_eth_dev *dev);

/*
 * misc function prototypes
 */
void i350_pf_host_init(struct rte_eth_dev *eth_dev);

void i350_pf_mbx_process(struct rte_eth_dev *eth_dev);

int i350_pf_host_configure(struct rte_eth_dev *eth_dev);

void i350_rxq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_rxq_info *qinfo);

void i350_txq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_txq_info *qinfo);

uint32_t em_get_max_pktlen(struct rte_eth_dev *dev);

/*
 * RX/TX EM function prototypes
 */
void eth_em_tx_queue_release(struct rte_eth_dev *dev, uint16_t qid);
void eth_em_rx_queue_release(struct rte_eth_dev *dev, uint16_t qid);

void em_dev_clear_queues(struct rte_eth_dev *dev);
void em_dev_free_queues(struct rte_eth_dev *dev);

uint64_t em_get_rx_port_offloads_capa(void);
uint64_t em_get_rx_queue_offloads_capa(void);

int eth_em_rx_queue_setup(struct rte_eth_dev *dev, uint16_t rx_queue_id,
		uint16_t nb_rx_desc, unsigned int socket_id,
		const struct rte_eth_rxconf *rx_conf,
		struct rte_mempool *mb_pool);

uint32_t eth_em_rx_queue_count(void *rx_queue);

int eth_em_rx_descriptor_status(void *rx_queue, uint16_t offset);
int eth_em_tx_descriptor_status(void *tx_queue, uint16_t offset);

uint64_t em_get_tx_port_offloads_capa(struct rte_eth_dev *dev);
uint64_t em_get_tx_queue_offloads_capa(struct rte_eth_dev *dev);

int eth_em_tx_queue_setup(struct rte_eth_dev *dev, uint16_t tx_queue_id,
		uint16_t nb_tx_desc, unsigned int socket_id,
		const struct rte_eth_txconf *tx_conf);

int eth_em_rx_init(struct rte_eth_dev *dev);

void eth_em_tx_init(struct rte_eth_dev *dev);

uint16_t eth_em_xmit_pkts(void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts);

uint16_t eth_em_prep_pkts(void *txq, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts);

uint16_t eth_em_recv_pkts(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts);

uint16_t eth_em_recv_scattered_pkts(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts);

void em_rxq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_rxq_info *qinfo);

void em_txq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_txq_info *qinfo);

void i350_pf_host_uninit(struct rte_eth_dev *dev);

void i350_filterlist_flush(struct rte_eth_dev *dev);
int i350_delete_5tuple_filter_82576(struct rte_eth_dev *dev,
		struct i350_5tuple_filter *filter);
int i350_delete_2tuple_filter(struct rte_eth_dev *dev,
		struct i350_2tuple_filter *filter);
void i350_remove_flex_filter(struct rte_eth_dev *dev,
			struct i350_flex_filter *filter);
int i350_ethertype_filter_remove(struct i350_filter_info *filter_info,
	uint8_t idx);
int i350_add_del_ntuple_filter(struct rte_eth_dev *dev,
		struct rte_eth_ntuple_filter *ntuple_filter, bool add);
int i350_add_del_ethertype_filter(struct rte_eth_dev *dev,
			struct rte_eth_ethertype_filter *filter,
			bool add);
int eth_i350_syn_filter_set(struct rte_eth_dev *dev,
			struct rte_eth_syn_filter *filter,
			bool add);
int eth_i350_add_del_flex_filter(struct rte_eth_dev *dev,
			struct i350_flex_filter *filter,
			bool add);
int i350_rss_conf_init(struct rte_eth_dev *dev,
		      struct i350_rte_flow_rss_conf *out,
		      const struct rte_flow_action_rss *in);
int i350_action_rss_same(const struct rte_flow_action_rss *comp,
			const struct rte_flow_action_rss *with);
int i350_config_rss_filter(struct rte_eth_dev *dev,
			struct i350_rte_flow_rss_conf *conf,
			bool add);
void em_flush_desc_rings(struct rte_eth_dev *dev);

#endif /* _E1000_ETHDEV_H_ */
