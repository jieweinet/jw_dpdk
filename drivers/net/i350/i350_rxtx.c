/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2010-2016 Intel Corporation
 */

#include <sys/queue.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdarg.h>
#include <inttypes.h>

#include <rte_interrupts.h>
#include <rte_byteorder.h>
#include <rte_common.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_pci.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_launch.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_mempool.h>
#include <rte_malloc.h>
#include <rte_mbuf.h>
#include <rte_ether.h>
#include <ethdev_driver.h>
#include <rte_prefetch.h>
#include <rte_udp.h>
#include <rte_tcp.h>
#include <rte_sctp.h>
#include <rte_net.h>
#include <rte_string_fns.h>

#include "i350_logs.h"
#include "i350_api.h"
#include "i350_ethdev.h"

#ifdef RTE_LIBRTE_IEEE1588
#define I350_TX_IEEE1588_TMST RTE_MBUF_F_TX_IEEE1588_TMST
#else
#define I350_TX_IEEE1588_TMST 0
#endif
/* Bit Mask to indicate what bits required for building TX context */
#define I350_TX_OFFLOAD_MASK (RTE_MBUF_F_TX_OUTER_IPV6 |	 \
		RTE_MBUF_F_TX_OUTER_IPV4 |	 \
		RTE_MBUF_F_TX_IPV6 |		 \
		RTE_MBUF_F_TX_IPV4 |		 \
		RTE_MBUF_F_TX_VLAN |		 \
		RTE_MBUF_F_TX_IP_CKSUM |		 \
		RTE_MBUF_F_TX_L4_MASK |		 \
		RTE_MBUF_F_TX_TCP_SEG |		 \
		I350_TX_IEEE1588_TMST)

#define I350_TX_OFFLOAD_NOTSUP_MASK \
		(RTE_MBUF_F_TX_OFFLOAD_MASK ^ I350_TX_OFFLOAD_MASK)

/**
 * Structure associated with each descriptor of the RX ring of a RX queue.
 */
struct i350_rx_entry {
	struct rte_mbuf *mbuf; /**< mbuf associated with RX descriptor. */
};

/**
 * Structure associated with each descriptor of the TX ring of a TX queue.
 */
struct i350_tx_entry {
	struct rte_mbuf *mbuf; /**< mbuf associated with TX desc, if any. */
	uint16_t next_id; /**< Index of next descriptor in ring. */
	uint16_t last_id; /**< Index of last scattered descriptor. */
};

/**
 * rx queue flags
 */
enum i350_rxq_flags {
	I350_RXQ_FLAG_LB_BSWAP_VLAN = 0x01,
};

/**
 * Structure associated with each RX queue.
 */
struct i350_rx_queue {
	struct rte_mempool  *mb_pool;   /**< mbuf pool to populate RX ring. */
	volatile union i350_adv_rx_desc *rx_ring; /**< RX ring virtual address. */
	uint64_t            rx_ring_phys_addr; /**< RX ring DMA address. */
	volatile uint32_t   *rdt_reg_addr; /**< RDT register address. */
	volatile uint32_t   *rdh_reg_addr; /**< RDH register address. */
	struct i350_rx_entry *sw_ring;   /**< address of RX software ring. */
	struct rte_mbuf *pkt_first_seg; /**< First segment of current packet. */
	struct rte_mbuf *pkt_last_seg;  /**< Last segment of current packet. */
	uint16_t            nb_rx_desc; /**< number of RX descriptors. */
	uint16_t            rx_tail;    /**< current value of RDT register. */
	uint16_t            nb_rx_hold; /**< number of held free RX desc. */
	uint16_t            rx_free_thresh; /**< max free RX desc to hold. */
	uint16_t            queue_id;   /**< RX queue index. */
	uint16_t            reg_idx;    /**< RX queue register index. */
	uint16_t            port_id;    /**< Device port identifier. */
	uint8_t             pthresh;    /**< Prefetch threshold register. */
	uint8_t             hthresh;    /**< Host threshold register. */
	uint8_t             wthresh;    /**< Write-back threshold register. */
	uint8_t             crc_len;    /**< 0 if CRC stripped, 4 otherwise. */
	uint8_t             drop_en;  /**< If not 0, set SRRCTL.Drop_En. */
	uint32_t            flags;      /**< RX flags. */
	uint64_t	    offloads;   /**< offloads of RTE_ETH_RX_OFFLOAD_* */
	const struct rte_memzone *mz;
};

/**
 * Hardware context number
 */
enum i350_advctx_num {
	I350_CTX_0    = 0, /**< CTX0    */
	I350_CTX_1    = 1, /**< CTX1    */
	I350_CTX_NUM  = 2, /**< CTX_NUM */
};

/** Offload features */
union i350_tx_offload {
	uint64_t data;
	struct {
		uint64_t l3_len:9; /**< L3 (IP) Header Length. */
		uint64_t l2_len:7; /**< L2 (MAC) Header Length. */
		uint64_t vlan_tci:16;  /**< VLAN Tag Control Identifier(CPU order). */
		uint64_t l4_len:8; /**< L4 (TCP/UDP) Header Length. */
		uint64_t tso_segsz:16; /**< TCP TSO segment size. */

		/* uint64_t unused:8; */
	};
};

/*
 * Compare mask for igb_tx_offload.data,
 * should be in sync with igb_tx_offload layout.
 * */
#define TX_MACIP_LEN_CMP_MASK	0x000000000000FFFFULL /**< L2L3 header mask. */
#define TX_VLAN_CMP_MASK		0x00000000FFFF0000ULL /**< Vlan mask. */
#define TX_TCP_LEN_CMP_MASK		0x000000FF00000000ULL /**< TCP header mask. */
#define TX_TSO_MSS_CMP_MASK		0x00FFFF0000000000ULL /**< TSO segsz mask. */
/** Mac + IP + TCP + Mss mask. */
#define TX_TSO_CMP_MASK	\
	(TX_MACIP_LEN_CMP_MASK | TX_TCP_LEN_CMP_MASK | TX_TSO_MSS_CMP_MASK)

/**
 * Structure to check if new context need be built
 */
struct i350_advctx_info {
	uint64_t flags;           /**< ol_flags related to context build. */
	/** tx offload: vlan, tso, l2-l3-l4 lengths. */
	union i350_tx_offload tx_offload;
	/** compare mask for tx offload. */
	union i350_tx_offload tx_offload_mask;
};

/**
 * Structure associated with each TX queue.
 */
struct i350_tx_queue {
	volatile union i350_adv_tx_desc *tx_ring; /**< TX ring address */
	uint64_t               tx_ring_phys_addr; /**< TX ring DMA address. */
	struct i350_tx_entry    *sw_ring; /**< virtual address of SW ring. */
	volatile uint32_t      *tdt_reg_addr; /**< Address of TDT register. */
	uint32_t               txd_type;      /**< Device-specific TXD type */
	uint16_t               nb_tx_desc;    /**< number of TX descriptors. */
	uint16_t               tx_tail; /**< Current value of TDT register. */
	uint16_t               tx_head;
	/**< Index of first used TX descriptor. */
	uint16_t               queue_id; /**< TX queue index. */
	uint16_t               reg_idx;  /**< TX queue register index. */
	uint16_t               port_id;  /**< Device port identifier. */
	uint8_t                pthresh;  /**< Prefetch threshold register. */
	uint8_t                hthresh;  /**< Host threshold register. */
	uint8_t                wthresh;  /**< Write-back threshold register. */
	uint32_t               ctx_curr;
	/**< Current used hardware descriptor. */
	uint32_t               ctx_start;
	/**< Start context position for transmit queue. */
	struct i350_advctx_info ctx_cache[I350_CTX_NUM];
	/**< Hardware context history.*/
	uint64_t	       offloads; /**< offloads of RTE_ETH_TX_OFFLOAD_* */
	const struct rte_memzone *mz;
};

#if 1
#define RTE_PMD_USE_PREFETCH
#endif

#ifdef RTE_PMD_USE_PREFETCH
#define rte_i350_prefetch(p)	rte_prefetch0(p)
#else
#define rte_i350_prefetch(p)	do {} while(0)
#endif

#ifdef RTE_PMD_PACKET_PREFETCH
#define rte_packet_prefetch(p) rte_prefetch1(p)
#else
#define rte_packet_prefetch(p)	do {} while(0)
#endif

/*
 * Macro for VMDq feature for 1 GbE NIC.
 */
#define I350_VMOLR_SIZE			(8)
#define I350_TSO_MAX_HDRLEN			(512)
#define I350_TSO_MAX_MSS				(9216)

/*********************************************************************
 *
 *  TX function
 *
 **********************************************************************/

/*
 *There're some limitations in hardware for TCP segmentation offload. We
 *should check whether the parameters are valid.
 */
static inline uint64_t
check_tso_para(uint64_t ol_req, union i350_tx_offload ol_para)
{
	if (!(ol_req & RTE_MBUF_F_TX_TCP_SEG))
		return ol_req;
	if ((ol_para.tso_segsz > I350_TSO_MAX_MSS) || (ol_para.l2_len +
			ol_para.l3_len + ol_para.l4_len > I350_TSO_MAX_HDRLEN)) {
		ol_req &= ~RTE_MBUF_F_TX_TCP_SEG;
		ol_req |= RTE_MBUF_F_TX_TCP_CKSUM;
	}
	return ol_req;
}

/*
 * Advanced context descriptor are almost same between igb/ixgbe
 * This is a separate function, looking for optimization opportunity here
 * Rework required to go with the pre-defined values.
 */

static inline void
i350e_set_xmit_ctx(struct i350_tx_queue* txq,
		volatile struct i350_adv_tx_context_desc *ctx_txd,
		uint64_t ol_flags, union i350_tx_offload tx_offload, uint64_t txtime)
{
	uint32_t type_tucmd_mlhl;
	uint32_t mss_l4len_idx;
	uint32_t ctx_idx, ctx_curr;
	uint32_t vlan_macip_lens;
	uint32_t launch_time;
	union i350_tx_offload tx_offload_mask;

	ctx_curr = txq->ctx_curr;
	ctx_idx = ctx_curr + txq->ctx_start;

	tx_offload_mask.data = 0;
	type_tucmd_mlhl = 0;

	/* Specify which HW CTX to upload. */
	mss_l4len_idx = (ctx_idx << I350_ADVTXD_IDX_SHIFT);

	if (ol_flags & RTE_MBUF_F_TX_VLAN)
		tx_offload_mask.data |= TX_VLAN_CMP_MASK;

	/* check if TCP segmentation required for this packet */
	if (ol_flags & RTE_MBUF_F_TX_TCP_SEG) {
		/* implies IP cksum in IPv4 */
		if (ol_flags & RTE_MBUF_F_TX_IP_CKSUM)
			type_tucmd_mlhl = I350_ADVTXD_TUCMD_IPV4 |
				I350_ADVTXD_TUCMD_L4T_TCP |
				I350_ADVTXD_DTYP_CTXT | I350_ADVTXD_DCMD_DEXT;
		else
			type_tucmd_mlhl = I350_ADVTXD_TUCMD_IPV6 |
				I350_ADVTXD_TUCMD_L4T_TCP |
				I350_ADVTXD_DTYP_CTXT | I350_ADVTXD_DCMD_DEXT;

		tx_offload_mask.data |= TX_TSO_CMP_MASK;
		mss_l4len_idx |= tx_offload.tso_segsz << I350_ADVTXD_MSS_SHIFT;
		mss_l4len_idx |= tx_offload.l4_len << I350_ADVTXD_L4LEN_SHIFT;
	} else { /* no TSO, check if hardware checksum is needed */
		if (ol_flags & (RTE_MBUF_F_TX_IP_CKSUM | RTE_MBUF_F_TX_L4_MASK))
			tx_offload_mask.data |= TX_MACIP_LEN_CMP_MASK;

		if (ol_flags & RTE_MBUF_F_TX_IP_CKSUM)
			type_tucmd_mlhl = I350_ADVTXD_TUCMD_IPV4;

		switch (ol_flags & RTE_MBUF_F_TX_L4_MASK) {
		case RTE_MBUF_F_TX_UDP_CKSUM:
			type_tucmd_mlhl |= I350_ADVTXD_TUCMD_L4T_UDP |
				I350_ADVTXD_DTYP_CTXT | I350_ADVTXD_DCMD_DEXT;
			mss_l4len_idx |= sizeof(struct rte_udp_hdr)
				<< I350_ADVTXD_L4LEN_SHIFT;
			break;
		case RTE_MBUF_F_TX_TCP_CKSUM:
			type_tucmd_mlhl |= I350_ADVTXD_TUCMD_L4T_TCP |
				I350_ADVTXD_DTYP_CTXT | I350_ADVTXD_DCMD_DEXT;
			mss_l4len_idx |= sizeof(struct rte_tcp_hdr)
				<< I350_ADVTXD_L4LEN_SHIFT;
			break;
		case RTE_MBUF_F_TX_SCTP_CKSUM:
			type_tucmd_mlhl |= I350_ADVTXD_TUCMD_L4T_SCTP |
				I350_ADVTXD_DTYP_CTXT | I350_ADVTXD_DCMD_DEXT;
			mss_l4len_idx |= sizeof(struct rte_sctp_hdr)
				<< I350_ADVTXD_L4LEN_SHIFT;
			break;
		default:
			type_tucmd_mlhl |= I350_ADVTXD_TUCMD_L4T_RSV |
				I350_ADVTXD_DTYP_CTXT | I350_ADVTXD_DCMD_DEXT;
			break;
		}
	}

	if (!txtime) {
		txq->ctx_cache[ctx_curr].flags = ol_flags;
		txq->ctx_cache[ctx_curr].tx_offload.data =
			tx_offload_mask.data & tx_offload.data;
		txq->ctx_cache[ctx_curr].tx_offload_mask = tx_offload_mask;
	}

	ctx_txd->type_tucmd_mlhl = rte_cpu_to_le_32(type_tucmd_mlhl);
	vlan_macip_lens = (uint32_t)tx_offload.data;
	ctx_txd->vlan_macip_lens = rte_cpu_to_le_32(vlan_macip_lens);
	ctx_txd->mss_l4len_idx = rte_cpu_to_le_32(mss_l4len_idx);
	ctx_txd->u.seqnum_seed = 0;

	if (txtime) {
		launch_time = (txtime - I350_I210_TX_OFFSET_BASE) % NSEC_PER_SEC;
		ctx_txd->u.launch_time = rte_cpu_to_le_32(launch_time / 32);
	} else {
		ctx_txd->u.launch_time = 0;
	}
}

/*
 * Check which hardware context can be used. Use the existing match
 * or create a new context descriptor.
 */
static inline uint32_t
what_advctx_update(struct i350_tx_queue *txq, uint64_t flags,
		union i350_tx_offload tx_offload)
{
	/* If match with the current context */
	if (likely((txq->ctx_cache[txq->ctx_curr].flags == flags) &&
		(txq->ctx_cache[txq->ctx_curr].tx_offload.data ==
		(txq->ctx_cache[txq->ctx_curr].tx_offload_mask.data & tx_offload.data)))) {
			return txq->ctx_curr;
	}

	/* If match with the second context */
	txq->ctx_curr ^= 1;
	if (likely((txq->ctx_cache[txq->ctx_curr].flags == flags) &&
		(txq->ctx_cache[txq->ctx_curr].tx_offload.data ==
		(txq->ctx_cache[txq->ctx_curr].tx_offload_mask.data & tx_offload.data)))) {
			return txq->ctx_curr;
	}

	/* Mismatch, use the previous context */
	return I350_CTX_NUM;
}

static inline uint32_t
tx_desc_cksum_flags_to_olinfo(uint64_t ol_flags)
{
	static const uint32_t l4_olinfo[2] = {0, I350_ADVTXD_POPTS_TXSM};
	static const uint32_t l3_olinfo[2] = {0, I350_ADVTXD_POPTS_IXSM};
	uint32_t tmp;

	tmp  = l4_olinfo[(ol_flags & RTE_MBUF_F_TX_L4_MASK)  != RTE_MBUF_F_TX_L4_NO_CKSUM];
	tmp |= l3_olinfo[(ol_flags & RTE_MBUF_F_TX_IP_CKSUM) != 0];
	tmp |= l4_olinfo[(ol_flags & RTE_MBUF_F_TX_TCP_SEG) != 0];
	return tmp;
}

static inline uint32_t
tx_desc_vlan_flags_to_cmdtype(uint64_t ol_flags)
{
	uint32_t cmdtype;
	static uint32_t vlan_cmd[2] = {0, I350_ADVTXD_DCMD_VLE};
	static uint32_t tso_cmd[2] = {0, I350_ADVTXD_DCMD_TSE};
	cmdtype = vlan_cmd[(ol_flags & RTE_MBUF_F_TX_VLAN) != 0];
	cmdtype |= tso_cmd[(ol_flags & RTE_MBUF_F_TX_TCP_SEG) != 0];
	return cmdtype;
}

uint16_t
eth_i350_xmit_pkts(void *tx_queue, struct rte_mbuf **tx_pkts,
	       uint16_t nb_pkts)
{
	struct i350_tx_queue *txq;
	struct i350_tx_entry *sw_ring;
	struct i350_tx_entry *txe, *txn;
	volatile union i350_adv_tx_desc *txr;
	volatile union i350_adv_tx_desc *txd;
	struct rte_mbuf     *tx_pkt;
	struct rte_mbuf     *m_seg;
	uint64_t buf_dma_addr;
	uint32_t olinfo_status;
	uint32_t cmd_type_len;
	uint32_t pkt_len;
	uint16_t slen;
	uint64_t ol_flags;
	uint16_t tx_end;
	uint16_t tx_id;
	uint16_t tx_last;
	uint16_t nb_tx;
	uint64_t tx_ol_req;
	uint32_t new_ctx = 0;
	uint32_t ctx = 0;
	union i350_tx_offload tx_offload = {0};
	uint64_t ts;

	txq = tx_queue;
	sw_ring = txq->sw_ring;
	txr     = txq->tx_ring;
	tx_id   = txq->tx_tail;
	txe = &sw_ring[tx_id];

	for (nb_tx = 0; nb_tx < nb_pkts; nb_tx++) {
		tx_pkt = *tx_pkts++;
		pkt_len = tx_pkt->pkt_len;

		RTE_MBUF_PREFETCH_TO_FREE(txe->mbuf);

		/*
		 * The number of descriptors that must be allocated for a
		 * packet is the number of segments of that packet, plus 1
		 * Context Descriptor for the VLAN Tag Identifier, if any.
		 * Determine the last TX descriptor to allocate in the TX ring
		 * for the packet, starting from the current position (tx_id)
		 * in the ring.
		 */
		tx_last = (uint16_t) (tx_id + tx_pkt->nb_segs - 1);

		ol_flags = tx_pkt->ol_flags;
		tx_ol_req = ol_flags & I350_TX_OFFLOAD_MASK;

		/* If a Context Descriptor need be built . */
		if (tx_ol_req) {
			tx_offload.l2_len = tx_pkt->l2_len;
			tx_offload.l3_len = tx_pkt->l3_len;
			tx_offload.l4_len = tx_pkt->l4_len;
			tx_offload.vlan_tci = tx_pkt->vlan_tci;
			tx_offload.tso_segsz = tx_pkt->tso_segsz;
			tx_ol_req = check_tso_para(tx_ol_req, tx_offload);

			ctx = what_advctx_update(txq, tx_ol_req, tx_offload);
			/* Only allocate context descriptor if required*/
			new_ctx = (ctx == I350_CTX_NUM);
			ctx = txq->ctx_curr + txq->ctx_start;
			tx_last = (uint16_t) (tx_last + new_ctx);
		}
		if (tx_last >= txq->nb_tx_desc)
			tx_last = (uint16_t) (tx_last - txq->nb_tx_desc);

		PMD_TX_LOG(DEBUG, "port_id=%u queue_id=%u pktlen=%u"
			   " tx_first=%u tx_last=%u",
			   (unsigned) txq->port_id,
			   (unsigned) txq->queue_id,
			   (unsigned) pkt_len,
			   (unsigned) tx_id,
			   (unsigned) tx_last);

		/*
		 * Check if there are enough free descriptors in the TX ring
		 * to transmit the next packet.
		 * This operation is based on the two following rules:
		 *
		 *   1- Only check that the last needed TX descriptor can be
		 *      allocated (by construction, if that descriptor is free,
		 *      all intermediate ones are also free).
		 *
		 *      For this purpose, the index of the last TX descriptor
		 *      used for a packet (the "last descriptor" of a packet)
		 *      is recorded in the TX entries (the last one included)
		 *      that are associated with all TX descriptors allocated
		 *      for that packet.
		 *
		 *   2- Avoid to allocate the last free TX descriptor of the
		 *      ring, in order to never set the TDT register with the
		 *      same value stored in parallel by the NIC in the TDH
		 *      register, which makes the TX engine of the NIC enter
		 *      in a deadlock situation.
		 *
		 *      By extension, avoid to allocate a free descriptor that
		 *      belongs to the last set of free descriptors allocated
		 *      to the same packet previously transmitted.
		 */

		/*
		 * The "last descriptor" of the previously sent packet, if any,
		 * which used the last descriptor to allocate.
		 */
		tx_end = sw_ring[tx_last].last_id;

		/*
		 * The next descriptor following that "last descriptor" in the
		 * ring.
		 */
		tx_end = sw_ring[tx_end].next_id;

		/*
		 * The "last descriptor" associated with that next descriptor.
		 */
		tx_end = sw_ring[tx_end].last_id;

		/*
		 * Check that this descriptor is free.
		 */
		if (! (txr[tx_end].wb.status & I350_TXD_STAT_DD)) {
			if (nb_tx == 0)
				return 0;
			goto end_of_tx;
		}

		/*
		 * Set common flags of all TX Data Descriptors.
		 *
		 * The following bits must be set in all Data Descriptors:
		 *   - E1000_ADVTXD_DTYP_DATA
		 *   - E1000_ADVTXD_DCMD_DEXT
		 *
		 * The following bits must be set in the first Data Descriptor
		 * and are ignored in the other ones:
		 *   - E1000_ADVTXD_DCMD_IFCS
		 *   - E1000_ADVTXD_MAC_1588
		 *   - E1000_ADVTXD_DCMD_VLE
		 *
		 * The following bits must only be set in the last Data
		 * Descriptor:
		 *   - E1000_TXD_CMD_EOP
		 *
		 * The following bits can be set in any Data Descriptor, but
		 * are only set in the last Data Descriptor:
		 *   - E1000_TXD_CMD_RS
		 */
		cmd_type_len = txq->txd_type |
			I350_ADVTXD_DCMD_IFCS | I350_ADVTXD_DCMD_DEXT;
		if (tx_ol_req & RTE_MBUF_F_TX_TCP_SEG)
			pkt_len -= (tx_pkt->l2_len + tx_pkt->l3_len + tx_pkt->l4_len);
		olinfo_status = (pkt_len << I350_ADVTXD_PAYLEN_SHIFT);
#if defined(RTE_LIBRTE_IEEE1588)
		if (ol_flags & RTE_MBUF_F_TX_IEEE1588_TMST)
			cmd_type_len |= I350_ADVTXD_MAC_TSTAMP;
#endif
		if (tx_ol_req) {
			/* Setup TX Advanced context descriptor if required */
			if (new_ctx) {
				volatile struct i350_adv_tx_context_desc *
				    ctx_txd;

				ctx_txd = (volatile struct
				    i350_adv_tx_context_desc *)
				    &txr[tx_id];

				txn = &sw_ring[txe->next_id];
				RTE_MBUF_PREFETCH_TO_FREE(txn->mbuf);

				if (txe->mbuf != NULL) {
					rte_pktmbuf_free_seg(txe->mbuf);
					txe->mbuf = NULL;
				}

				if (i350_tx_timestamp_dynflag > 0) {
					ts = *RTE_MBUF_DYNFIELD(tx_pkt,
						i350_tx_timestamp_dynfield_offset, uint64_t *);
					i350e_set_xmit_ctx(txq, ctx_txd, tx_ol_req, tx_offload, ts);
				} else {
					i350e_set_xmit_ctx(txq, ctx_txd, tx_ol_req, tx_offload, 0);
				}

				txe->last_id = tx_last;
				tx_id = txe->next_id;
				txe = txn;
			}

			/* Setup the TX Advanced Data Descriptor */
			cmd_type_len  |= tx_desc_vlan_flags_to_cmdtype(tx_ol_req);
			olinfo_status |= tx_desc_cksum_flags_to_olinfo(tx_ol_req);
			olinfo_status |= (ctx << I350_ADVTXD_IDX_SHIFT);
		}

		m_seg = tx_pkt;
		do {
			txn = &sw_ring[txe->next_id];
			txd = &txr[tx_id];

			if (txe->mbuf != NULL)
				rte_pktmbuf_free_seg(txe->mbuf);
			txe->mbuf = m_seg;

			/*
			 * Set up transmit descriptor.
			 */
			slen = (uint16_t) m_seg->data_len;
			buf_dma_addr = rte_mbuf_data_iova(m_seg);
			txd->read.buffer_addr =
				rte_cpu_to_le_64(buf_dma_addr);
			txd->read.cmd_type_len =
				rte_cpu_to_le_32(cmd_type_len | slen);
			txd->read.olinfo_status =
				rte_cpu_to_le_32(olinfo_status);
			txe->last_id = tx_last;
			tx_id = txe->next_id;
			txe = txn;
			m_seg = m_seg->next;
		} while (m_seg != NULL);

		/*
		 * The last packet data descriptor needs End Of Packet (EOP)
		 * and Report Status (RS).
		 */
		txd->read.cmd_type_len |=
			rte_cpu_to_le_32(I350_TXD_CMD_EOP | I350_TXD_CMD_RS);
	}
 end_of_tx:
	rte_wmb();

	/*
	 * Set the Transmit Descriptor Tail (TDT).
	 */
	I350_PCI_REG_WRITE_RELAXED(txq->tdt_reg_addr, tx_id);
	PMD_TX_LOG(DEBUG, "port_id=%u queue_id=%u tx_tail=%u nb_tx=%u",
		   (unsigned) txq->port_id, (unsigned) txq->queue_id,
		   (unsigned) tx_id, (unsigned) nb_tx);
	txq->tx_tail = tx_id;

	return nb_tx;
}

/*********************************************************************
 *
 *  TX prep functions
 *
 **********************************************************************/
uint16_t
eth_i350_prep_pkts(__rte_unused void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts)
{
	int i, ret;
	struct rte_mbuf *m;

	for (i = 0; i < nb_pkts; i++) {
		m = tx_pkts[i];

		/* Check some limitations for TSO in hardware */
		if (m->ol_flags & RTE_MBUF_F_TX_TCP_SEG)
			if ((m->tso_segsz > I350_TSO_MAX_MSS) ||
					(m->l2_len + m->l3_len + m->l4_len >
					I350_TSO_MAX_HDRLEN)) {
				rte_errno = EINVAL;
				return i;
			}

		if (m->ol_flags & I350_TX_OFFLOAD_NOTSUP_MASK) {
			rte_errno = ENOTSUP;
			return i;
		}

#ifdef RTE_ETHDEV_DEBUG_TX
		ret = rte_validate_tx_offload(m);
		if (ret != 0) {
			rte_errno = -ret;
			return i;
		}
#endif
		ret = rte_net_intel_cksum_prepare(m);
		if (ret != 0) {
			rte_errno = -ret;
			return i;
		}
	}

	return i;
}

/*********************************************************************
 *
 *  RX functions
 *
 **********************************************************************/
#define I350_PACKET_TYPE_IPV4              0X01
#define I350_PACKET_TYPE_IPV4_TCP          0X11
#define I350_PACKET_TYPE_IPV4_UDP          0X21
#define I350_PACKET_TYPE_IPV4_SCTP         0X41
#define I350_PACKET_TYPE_IPV4_EXT          0X03
#define I350_PACKET_TYPE_IPV4_EXT_SCTP     0X43
#define I350_PACKET_TYPE_IPV6              0X04
#define I350_PACKET_TYPE_IPV6_TCP          0X14
#define I350_PACKET_TYPE_IPV6_UDP          0X24
#define I350_PACKET_TYPE_IPV6_EXT          0X0C
#define I350_PACKET_TYPE_IPV6_EXT_TCP      0X1C
#define I350_PACKET_TYPE_IPV6_EXT_UDP      0X2C
#define I350_PACKET_TYPE_IPV4_IPV6         0X05
#define I350_PACKET_TYPE_IPV4_IPV6_TCP     0X15
#define I350_PACKET_TYPE_IPV4_IPV6_UDP     0X25
#define I350_PACKET_TYPE_IPV4_IPV6_EXT     0X0D
#define I350_PACKET_TYPE_IPV4_IPV6_EXT_TCP 0X1D
#define I350_PACKET_TYPE_IPV4_IPV6_EXT_UDP 0X2D
#define I350_PACKET_TYPE_MAX               0X80
#define I350_PACKET_TYPE_MASK              0X7F
#define I350_PACKET_TYPE_SHIFT             0X04
static inline uint32_t
i350_rxd_pkt_info_to_pkt_type(uint16_t pkt_info)
{
	static const alignas(RTE_CACHE_LINE_SIZE) uint32_t
		ptype_table[I350_PACKET_TYPE_MAX] = {
		[I350_PACKET_TYPE_IPV4] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4,
		[I350_PACKET_TYPE_IPV4_EXT] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4_EXT,
		[I350_PACKET_TYPE_IPV6] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6,
		[I350_PACKET_TYPE_IPV4_IPV6] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6,
		[I350_PACKET_TYPE_IPV6_EXT] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6_EXT,
		[I350_PACKET_TYPE_IPV4_IPV6_EXT] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6_EXT,
		[I350_PACKET_TYPE_IPV4_TCP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_TCP,
		[I350_PACKET_TYPE_IPV6_TCP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_TCP,
		[I350_PACKET_TYPE_IPV4_IPV6_TCP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_TCP,
		[I350_PACKET_TYPE_IPV6_EXT_TCP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6_EXT | RTE_PTYPE_L4_TCP,
		[I350_PACKET_TYPE_IPV4_IPV6_EXT_TCP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_TCP,
		[I350_PACKET_TYPE_IPV4_UDP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_UDP,
		[I350_PACKET_TYPE_IPV6_UDP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_UDP,
		[I350_PACKET_TYPE_IPV4_IPV6_UDP] =  RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_UDP,
		[I350_PACKET_TYPE_IPV6_EXT_UDP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6_EXT | RTE_PTYPE_L4_UDP,
		[I350_PACKET_TYPE_IPV4_IPV6_EXT_UDP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_UDP,
		[I350_PACKET_TYPE_IPV4_SCTP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_SCTP,
		[I350_PACKET_TYPE_IPV4_EXT_SCTP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_L4_SCTP,
	};
	if (unlikely(pkt_info & I350_RXDADV_PKTTYPE_ETQF))
		return RTE_PTYPE_UNKNOWN;

	pkt_info = (pkt_info >> I350_PACKET_TYPE_SHIFT) & I350_PACKET_TYPE_MASK;

	return ptype_table[pkt_info];
}

static inline uint64_t
rx_desc_hlen_type_rss_to_pkt_flags(struct i350_rx_queue *rxq, uint32_t hl_tp_rs)
{
	uint64_t pkt_flags = ((hl_tp_rs & 0x0F) == 0) ?  0 : RTE_MBUF_F_RX_RSS_HASH;

#if defined(RTE_LIBRTE_IEEE1588)
	static uint32_t ip_pkt_etqf_map[8] = {
		0, 0, 0, RTE_MBUF_F_RX_IEEE1588_PTP,
		0, 0, 0, 0,
	};

	struct rte_eth_dev dev = rte_eth_devices[rxq->port_id];
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev.data->dev_private);

	/* EtherType is in bits 8:10 in Packet Type, and not in the default 0:2 */
	pkt_flags |= ip_pkt_etqf_map[(hl_tp_rs >> 4) & 0x07];
#else
	RTE_SET_USED(rxq);
#endif

	return pkt_flags;
}

static inline uint64_t
rx_desc_status_to_pkt_flags(uint32_t rx_status)
{
	uint64_t pkt_flags;

	/* Check if VLAN present */
	pkt_flags = ((rx_status & I350_RXD_STAT_VP) ?
		RTE_MBUF_F_RX_VLAN | RTE_MBUF_F_RX_VLAN_STRIPPED : 0);

#if defined(RTE_LIBRTE_IEEE1588)
	if (rx_status & I350_RXD_STAT_TMST)
		pkt_flags = pkt_flags | RTE_MBUF_F_RX_IEEE1588_TMST;
#endif
	return pkt_flags;
}

static inline uint64_t
rx_desc_error_to_pkt_flags(uint32_t rx_status)
{
	/*
	 * Bit 30: IPE, IPv4 checksum error
	 * Bit 29: L4I, L4I integrity error
	 */

	static uint64_t error_to_pkt_flags_map[4] = {
		RTE_MBUF_F_RX_IP_CKSUM_GOOD | RTE_MBUF_F_RX_L4_CKSUM_GOOD,
		RTE_MBUF_F_RX_IP_CKSUM_GOOD | RTE_MBUF_F_RX_L4_CKSUM_BAD,
		RTE_MBUF_F_RX_IP_CKSUM_BAD | RTE_MBUF_F_RX_L4_CKSUM_GOOD,
		RTE_MBUF_F_RX_IP_CKSUM_BAD | RTE_MBUF_F_RX_L4_CKSUM_BAD
	};
	return error_to_pkt_flags_map[(rx_status >>
		I350_RXD_ERR_CKSUM_BIT) & I350_RXD_ERR_CKSUM_MSK];
}

uint16_t
eth_i350_recv_pkts(void *rx_queue, struct rte_mbuf **rx_pkts,
	       uint16_t nb_pkts)
{
	struct i350_rx_queue *rxq;
	volatile union i350_adv_rx_desc *rx_ring;
	volatile union i350_adv_rx_desc *rxdp;
	struct i350_rx_entry *sw_ring;
	struct i350_rx_entry *rxe;
	struct rte_mbuf *rxm;
	struct rte_mbuf *nmb;
	union i350_adv_rx_desc rxd;
	uint64_t dma_addr;
	uint32_t staterr;
	uint32_t hlen_type_rss;
	uint16_t pkt_len;
	uint16_t rx_id;
	uint16_t nb_rx;
	uint16_t nb_hold;
	uint64_t pkt_flags;

	nb_rx = 0;
	nb_hold = 0;
	rxq = rx_queue;
	rx_id = rxq->rx_tail;
	rx_ring = rxq->rx_ring;
	sw_ring = rxq->sw_ring;
	while (nb_rx < nb_pkts) {
		/*
		 * The order of operations here is important as the DD status
		 * bit must not be read after any other descriptor fields.
		 * rx_ring and rxdp are pointing to volatile data so the order
		 * of accesses cannot be reordered by the compiler. If they were
		 * not volatile, they could be reordered which could lead to
		 * using invalid descriptor fields when read from rxd.
		 */
		rxdp = &rx_ring[rx_id];
		staterr = rxdp->wb.upper.status_error;
		if (! (staterr & rte_cpu_to_le_32(I350_RXD_STAT_DD)))
			break;
		rxd = *rxdp;

		/*
		 * End of packet.
		 *
		 * If the E1000_RXD_STAT_EOP flag is not set, the RX packet is
		 * likely to be invalid and to be dropped by the various
		 * validation checks performed by the network stack.
		 *
		 * Allocate a new mbuf to replenish the RX ring descriptor.
		 * If the allocation fails:
		 *    - arrange for that RX descriptor to be the first one
		 *      being parsed the next time the receive function is
		 *      invoked [on the same queue].
		 *
		 *    - Stop parsing the RX ring and return immediately.
		 *
		 * This policy do not drop the packet received in the RX
		 * descriptor for which the allocation of a new mbuf failed.
		 * Thus, it allows that packet to be later retrieved if
		 * mbuf have been freed in the mean time.
		 * As a side effect, holding RX descriptors instead of
		 * systematically giving them back to the NIC may lead to
		 * RX ring exhaustion situations.
		 * However, the NIC can gracefully prevent such situations
		 * to happen by sending specific "back-pressure" flow control
		 * frames to its peer(s).
		 */
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_id=%u "
			   "staterr=0x%x pkt_len=%u",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) staterr,
			   (unsigned) rte_le_to_cpu_16(rxd.wb.upper.length));

		nmb = rte_mbuf_raw_alloc(rxq->mb_pool);
		if (nmb == NULL) {
			PMD_RX_LOG(DEBUG, "RX mbuf alloc failed port_id=%u "
				   "queue_id=%u", (unsigned) rxq->port_id,
				   (unsigned) rxq->queue_id);
			rte_eth_devices[rxq->port_id].data->rx_mbuf_alloc_failed++;
			break;
		}

		nb_hold++;
		rxe = &sw_ring[rx_id];
		rx_id++;
		if (rx_id == rxq->nb_rx_desc)
			rx_id = 0;

		/* Prefetch next mbuf while processing current one. */
		rte_i350_prefetch(sw_ring[rx_id].mbuf);

		/*
		 * When next RX descriptor is on a cache-line boundary,
		 * prefetch the next 4 RX descriptors and the next 8 pointers
		 * to mbufs.
		 */
		if ((rx_id & 0x3) == 0) {
			rte_i350_prefetch(&rx_ring[rx_id]);
			rte_i350_prefetch(&sw_ring[rx_id]);
		}

		rxm = rxe->mbuf;
		rxe->mbuf = nmb;
		dma_addr =
			rte_cpu_to_le_64(rte_mbuf_data_iova_default(nmb));
		rxdp->read.hdr_addr = 0;
		rxdp->read.pkt_addr = dma_addr;

		/*
		 * Initialize the returned mbuf.
		 * 1) setup generic mbuf fields:
		 *    - number of segments,
		 *    - next segment,
		 *    - packet length,
		 *    - RX port identifier.
		 * 2) integrate hardware offload data, if any:
		 *    - RSS flag & hash,
		 *    - IP checksum flag,
		 *    - VLAN TCI, if any,
		 *    - error flags.
		 */
		pkt_len = (uint16_t) (rte_le_to_cpu_16(rxd.wb.upper.length) -
				      rxq->crc_len);
		rxm->data_off = RTE_PKTMBUF_HEADROOM;
		rte_packet_prefetch((char *)rxm->buf_addr + rxm->data_off);
		rxm->nb_segs = 1;
		rxm->next = NULL;
		rxm->pkt_len = pkt_len;
		rxm->data_len = pkt_len;
		rxm->port = rxq->port_id;

		rxm->hash.rss = rxd.wb.lower.hi_dword.rss;
		hlen_type_rss = rte_le_to_cpu_32(rxd.wb.lower.lo_dword.data);

		/*
		 * The vlan_tci field is only valid when RTE_MBUF_F_RX_VLAN is
		 * set in the pkt_flags field and must be in CPU byte order.
		 */
		if ((staterr & rte_cpu_to_le_32(I350_RXDEXT_STATERR_LB)) &&
				(rxq->flags & I350_RXQ_FLAG_LB_BSWAP_VLAN)) {
			rxm->vlan_tci = rte_be_to_cpu_16(rxd.wb.upper.vlan);
		} else {
			rxm->vlan_tci = rte_le_to_cpu_16(rxd.wb.upper.vlan);
		}
		pkt_flags = rx_desc_hlen_type_rss_to_pkt_flags(rxq, hlen_type_rss);
		pkt_flags = pkt_flags | rx_desc_status_to_pkt_flags(staterr);
		pkt_flags = pkt_flags | rx_desc_error_to_pkt_flags(staterr);
		rxm->ol_flags = pkt_flags;
		rxm->packet_type = i350_rxd_pkt_info_to_pkt_type(rxd.wb.lower.
						lo_dword.hs_rss.pkt_info);

		/*
		 * Store the mbuf address into the next entry of the array
		 * of returned packets.
		 */
		rx_pkts[nb_rx++] = rxm;
	}
	rxq->rx_tail = rx_id;

	/*
	 * If the number of free RX descriptors is greater than the RX free
	 * threshold of the queue, advance the Receive Descriptor Tail (RDT)
	 * register.
	 * Update the RDT with the value of the last processed RX descriptor
	 * minus 1, to guarantee that the RDT register is never equal to the
	 * RDH register, which creates a "full" ring situation from the
	 * hardware point of view...
	 */
	nb_hold = (uint16_t) (nb_hold + rxq->nb_rx_hold);
	if (nb_hold > rxq->rx_free_thresh) {
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_tail=%u "
			   "nb_hold=%u nb_rx=%u",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) nb_hold,
			   (unsigned) nb_rx);
		rx_id = (uint16_t) ((rx_id == 0) ?
				     (rxq->nb_rx_desc - 1) : (rx_id - 1));
		I350_PCI_REG_WRITE(rxq->rdt_reg_addr, rx_id);
		nb_hold = 0;
	}
	rxq->nb_rx_hold = nb_hold;
	return nb_rx;
}

uint16_t
eth_i350_recv_scattered_pkts(void *rx_queue, struct rte_mbuf **rx_pkts,
			 uint16_t nb_pkts)
{
	struct i350_rx_queue *rxq;
	volatile union i350_adv_rx_desc *rx_ring;
	volatile union i350_adv_rx_desc *rxdp;
	struct i350_rx_entry *sw_ring;
	struct i350_rx_entry *rxe;
	struct rte_mbuf *first_seg;
	struct rte_mbuf *last_seg;
	struct rte_mbuf *rxm;
	struct rte_mbuf *nmb;
	union i350_adv_rx_desc rxd;
	uint64_t dma; /* Physical address of mbuf data buffer */
	uint32_t staterr;
	uint32_t hlen_type_rss;
	uint16_t rx_id;
	uint16_t nb_rx;
	uint16_t nb_hold;
	uint16_t data_len;
	uint64_t pkt_flags;

	nb_rx = 0;
	nb_hold = 0;
	rxq = rx_queue;
	rx_id = rxq->rx_tail;
	rx_ring = rxq->rx_ring;
	sw_ring = rxq->sw_ring;

	/*
	 * Retrieve RX context of current packet, if any.
	 */
	first_seg = rxq->pkt_first_seg;
	last_seg = rxq->pkt_last_seg;

	while (nb_rx < nb_pkts) {
	next_desc:
		/*
		 * The order of operations here is important as the DD status
		 * bit must not be read after any other descriptor fields.
		 * rx_ring and rxdp are pointing to volatile data so the order
		 * of accesses cannot be reordered by the compiler. If they were
		 * not volatile, they could be reordered which could lead to
		 * using invalid descriptor fields when read from rxd.
		 */
		rxdp = &rx_ring[rx_id];
		staterr = rxdp->wb.upper.status_error;
		if (! (staterr & rte_cpu_to_le_32(I350_RXD_STAT_DD)))
			break;
		rxd = *rxdp;

		/*
		 * Descriptor done.
		 *
		 * Allocate a new mbuf to replenish the RX ring descriptor.
		 * If the allocation fails:
		 *    - arrange for that RX descriptor to be the first one
		 *      being parsed the next time the receive function is
		 *      invoked [on the same queue].
		 *
		 *    - Stop parsing the RX ring and return immediately.
		 *
		 * This policy does not drop the packet received in the RX
		 * descriptor for which the allocation of a new mbuf failed.
		 * Thus, it allows that packet to be later retrieved if
		 * mbuf have been freed in the mean time.
		 * As a side effect, holding RX descriptors instead of
		 * systematically giving them back to the NIC may lead to
		 * RX ring exhaustion situations.
		 * However, the NIC can gracefully prevent such situations
		 * to happen by sending specific "back-pressure" flow control
		 * frames to its peer(s).
		 */
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_id=%u "
			   "staterr=0x%x data_len=%u",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) staterr,
			   (unsigned) rte_le_to_cpu_16(rxd.wb.upper.length));

		nmb = rte_mbuf_raw_alloc(rxq->mb_pool);
		if (nmb == NULL) {
			PMD_RX_LOG(DEBUG, "RX mbuf alloc failed port_id=%u "
				   "queue_id=%u", (unsigned) rxq->port_id,
				   (unsigned) rxq->queue_id);
			rte_eth_devices[rxq->port_id].data->rx_mbuf_alloc_failed++;
			break;
		}

		nb_hold++;
		rxe = &sw_ring[rx_id];
		rx_id++;
		if (rx_id == rxq->nb_rx_desc)
			rx_id = 0;

		/* Prefetch next mbuf while processing current one. */
		rte_i350_prefetch(sw_ring[rx_id].mbuf);

		/*
		 * When next RX descriptor is on a cache-line boundary,
		 * prefetch the next 4 RX descriptors and the next 8 pointers
		 * to mbufs.
		 */
		if ((rx_id & 0x3) == 0) {
			rte_i350_prefetch(&rx_ring[rx_id]);
			rte_i350_prefetch(&sw_ring[rx_id]);
		}

		/*
		 * Update RX descriptor with the physical address of the new
		 * data buffer of the new allocated mbuf.
		 */
		rxm = rxe->mbuf;
		rxe->mbuf = nmb;
		dma = rte_cpu_to_le_64(rte_mbuf_data_iova_default(nmb));
		rxdp->read.pkt_addr = dma;
		rxdp->read.hdr_addr = 0;

		/*
		 * Set data length & data buffer address of mbuf.
		 */
		data_len = rte_le_to_cpu_16(rxd.wb.upper.length);
		rxm->data_len = data_len;
		rxm->data_off = RTE_PKTMBUF_HEADROOM;

		/*
		 * If this is the first buffer of the received packet,
		 * set the pointer to the first mbuf of the packet and
		 * initialize its context.
		 * Otherwise, update the total length and the number of segments
		 * of the current scattered packet, and update the pointer to
		 * the last mbuf of the current packet.
		 */
		if (first_seg == NULL) {
			first_seg = rxm;
			first_seg->pkt_len = data_len;
			first_seg->nb_segs = 1;
		} else {
			first_seg->pkt_len += data_len;
			first_seg->nb_segs++;
			last_seg->next = rxm;
		}

		/*
		 * If this is not the last buffer of the received packet,
		 * update the pointer to the last mbuf of the current scattered
		 * packet and continue to parse the RX ring.
		 */
		if (! (staterr & I350_RXD_STAT_EOP)) {
			last_seg = rxm;
			goto next_desc;
		}

		/*
		 * This is the last buffer of the received packet.
		 * If the CRC is not stripped by the hardware:
		 *   - Subtract the CRC	length from the total packet length.
		 *   - If the last buffer only contains the whole CRC or a part
		 *     of it, free the mbuf associated to the last buffer.
		 *     If part of the CRC is also contained in the previous
		 *     mbuf, subtract the length of that CRC part from the
		 *     data length of the previous mbuf.
		 */
		rxm->next = NULL;
		if (unlikely(rxq->crc_len > 0)) {
			first_seg->pkt_len -= RTE_ETHER_CRC_LEN;
			if (data_len <= RTE_ETHER_CRC_LEN) {
				rte_pktmbuf_free_seg(rxm);
				first_seg->nb_segs--;
				last_seg->data_len = (uint16_t)
					(last_seg->data_len -
					 (RTE_ETHER_CRC_LEN - data_len));
				last_seg->next = NULL;
			} else
				rxm->data_len = (uint16_t)
					(data_len - RTE_ETHER_CRC_LEN);
		}

		/*
		 * Initialize the first mbuf of the returned packet:
		 *    - RX port identifier,
		 *    - hardware offload data, if any:
		 *      - RSS flag & hash,
		 *      - IP checksum flag,
		 *      - VLAN TCI, if any,
		 *      - error flags.
		 */
		first_seg->port = rxq->port_id;
		first_seg->hash.rss = rxd.wb.lower.hi_dword.rss;

		/*
		 * The vlan_tci field is only valid when RTE_MBUF_F_RX_VLAN is
		 * set in the pkt_flags field and must be in CPU byte order.
		 */
		if ((staterr & rte_cpu_to_le_32(I350_RXDEXT_STATERR_LB)) &&
				(rxq->flags & I350_RXQ_FLAG_LB_BSWAP_VLAN)) {
			first_seg->vlan_tci =
				rte_be_to_cpu_16(rxd.wb.upper.vlan);
		} else {
			first_seg->vlan_tci =
				rte_le_to_cpu_16(rxd.wb.upper.vlan);
		}
		hlen_type_rss = rte_le_to_cpu_32(rxd.wb.lower.lo_dword.data);
		pkt_flags = rx_desc_hlen_type_rss_to_pkt_flags(rxq, hlen_type_rss);
		pkt_flags = pkt_flags | rx_desc_status_to_pkt_flags(staterr);
		pkt_flags = pkt_flags | rx_desc_error_to_pkt_flags(staterr);
		first_seg->ol_flags = pkt_flags;
		first_seg->packet_type = i350_rxd_pkt_info_to_pkt_type(rxd.wb.
					lower.lo_dword.hs_rss.pkt_info);

		/* Prefetch data of first segment, if configured to do so. */
		rte_packet_prefetch((char *)first_seg->buf_addr +
			first_seg->data_off);

		/*
		 * Store the mbuf address into the next entry of the array
		 * of returned packets.
		 */
		rx_pkts[nb_rx++] = first_seg;

		/*
		 * Setup receipt context for a new packet.
		 */
		first_seg = NULL;
	}

	/*
	 * Record index of the next RX descriptor to probe.
	 */
	rxq->rx_tail = rx_id;

	/*
	 * Save receive context.
	 */
	rxq->pkt_first_seg = first_seg;
	rxq->pkt_last_seg = last_seg;

	/*
	 * If the number of free RX descriptors is greater than the RX free
	 * threshold of the queue, advance the Receive Descriptor Tail (RDT)
	 * register.
	 * Update the RDT with the value of the last processed RX descriptor
	 * minus 1, to guarantee that the RDT register is never equal to the
	 * RDH register, which creates a "full" ring situation from the
	 * hardware point of view...
	 */
	nb_hold = (uint16_t) (nb_hold + rxq->nb_rx_hold);
	if (nb_hold > rxq->rx_free_thresh) {
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_tail=%u "
			   "nb_hold=%u nb_rx=%u",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) nb_hold,
			   (unsigned) nb_rx);
		rx_id = (uint16_t) ((rx_id == 0) ?
				     (rxq->nb_rx_desc - 1) : (rx_id - 1));
		I350_PCI_REG_WRITE(rxq->rdt_reg_addr, rx_id);
		nb_hold = 0;
	}
	rxq->nb_rx_hold = nb_hold;
	return nb_rx;
}

/*
 * Maximum number of Ring Descriptors.
 *
 * Since RDLEN/TDLEN should be multiple of 128bytes, the number of ring
 * descriptors should meet the following condition:
 *      (num_ring_desc * sizeof(struct e1000_rx/tx_desc)) % 128 == 0
 */

static void
i350_tx_queue_release_mbufs(struct i350_tx_queue *txq)
{
	unsigned i;

	if (txq->sw_ring != NULL) {
		for (i = 0; i < txq->nb_tx_desc; i++) {
			if (txq->sw_ring[i].mbuf != NULL) {
				rte_pktmbuf_free_seg(txq->sw_ring[i].mbuf);
				txq->sw_ring[i].mbuf = NULL;
			}
		}
	}
}

static void
i350_tx_queue_release(struct i350_tx_queue *txq)
{
	if (txq != NULL) {
		i350_tx_queue_release_mbufs(txq);
		rte_free(txq->sw_ring);
		rte_memzone_free(txq->mz);
		rte_free(txq);
	}
}

void
eth_i350_tx_queue_release(struct rte_eth_dev *dev, uint16_t qid)
{
	i350_tx_queue_release(dev->data->tx_queues[qid]);
}

static int
i350_tx_done_cleanup(struct i350_tx_queue *txq, uint32_t free_cnt)
{
	struct i350_tx_entry *sw_ring;
	volatile union i350_adv_tx_desc *txr;
	uint16_t tx_first; /* First segment analyzed. */
	uint16_t tx_id;    /* Current segment being processed. */
	uint16_t tx_last;  /* Last segment in the current packet. */
	uint16_t tx_next;  /* First segment of the next packet. */
	int count = 0;

	if (!txq)
		return -ENODEV;

	sw_ring = txq->sw_ring;
	txr = txq->tx_ring;

	/* tx_tail is the last sent packet on the sw_ring. Goto the end
	 * of that packet (the last segment in the packet chain) and
	 * then the next segment will be the start of the oldest segment
	 * in the sw_ring. This is the first packet that will be
	 * attempted to be freed.
	 */

	/* Get last segment in most recently added packet. */
	tx_first = sw_ring[txq->tx_tail].last_id;

	/* Get the next segment, which is the oldest segment in ring. */
	tx_first = sw_ring[tx_first].next_id;

	/* Set the current index to the first. */
	tx_id = tx_first;

	/* Loop through each packet. For each packet, verify that an
	 * mbuf exists and that the last segment is free. If so, free
	 * it and move on.
	 */
	while (1) {
		tx_last = sw_ring[tx_id].last_id;

		if (sw_ring[tx_last].mbuf) {
			if (txr[tx_last].wb.status &
			    I350_TXD_STAT_DD) {
				/* Increment the number of packets
				 * freed.
				 */
				count++;

				/* Get the start of the next packet. */
				tx_next = sw_ring[tx_last].next_id;

				/* Loop through all segments in a
				 * packet.
				 */
				do {
					if (sw_ring[tx_id].mbuf) {
						rte_pktmbuf_free_seg(
							sw_ring[tx_id].mbuf);
						sw_ring[tx_id].mbuf = NULL;
						sw_ring[tx_id].last_id = tx_id;
					}

					/* Move to next segment. */
					tx_id = sw_ring[tx_id].next_id;

				} while (tx_id != tx_next);

				if (unlikely(count == (int)free_cnt))
					break;
			} else {
				/* mbuf still in use, nothing left to
				 * free.
				 */
				break;
			}
		} else {
			/* There are multiple reasons to be here:
			 * 1) All the packets on the ring have been
			 *    freed - tx_id is equal to tx_first
			 *    and some packets have been freed.
			 *    - Done, exit
			 * 2) Interfaces has not sent a rings worth of
			 *    packets yet, so the segment after tail is
			 *    still empty. Or a previous call to this
			 *    function freed some of the segments but
			 *    not all so there is a hole in the list.
			 *    Hopefully this is a rare case.
			 *    - Walk the list and find the next mbuf. If
			 *      there isn't one, then done.
			 */
			if (likely(tx_id == tx_first && count != 0))
				break;

			/* Walk the list and find the next mbuf, if any. */
			do {
				/* Move to next segment. */
				tx_id = sw_ring[tx_id].next_id;

				if (sw_ring[tx_id].mbuf)
					break;

			} while (tx_id != tx_first);

			/* Determine why previous loop bailed. If there
			 * is not an mbuf, done.
			 */
			if (!sw_ring[tx_id].mbuf)
				break;
		}
	}

	return count;
}

int
eth_i350_tx_done_cleanup(void *txq, uint32_t free_cnt)
{
	return i350_tx_done_cleanup(txq, free_cnt);
}

static void
i350_reset_tx_queue_stat(struct i350_tx_queue *txq)
{
	txq->tx_head = 0;
	txq->tx_tail = 0;
	txq->ctx_curr = 0;
	memset((void*)&txq->ctx_cache, 0,
		I350_CTX_NUM * sizeof(struct i350_advctx_info));
}

static void
i350_reset_tx_queue(struct i350_tx_queue *txq, struct rte_eth_dev *dev)
{
	static const union i350_adv_tx_desc zeroed_desc = {{0}};
	struct i350_tx_entry *txe = txq->sw_ring;
	uint16_t i, prev;
	struct i350_hw *hw;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	/* Zero out HW ring memory */
	for (i = 0; i < txq->nb_tx_desc; i++) {
		txq->tx_ring[i] = zeroed_desc;
	}

	/* Initialize ring entries */
	prev = (uint16_t)(txq->nb_tx_desc - 1);
	for (i = 0; i < txq->nb_tx_desc; i++) {
		volatile union i350_adv_tx_desc *txd = &(txq->tx_ring[i]);

		txd->wb.status = I350_TXD_STAT_DD;
		txe[i].mbuf = NULL;
		txe[i].last_id = i;
		txe[prev].next_id = i;
		prev = i;
	}

	txq->txd_type = I350_ADVTXD_DTYP_DATA;

	i350_reset_tx_queue_stat(txq);
}

uint64_t
i350_get_tx_port_offloads_capa(struct rte_eth_dev *dev)
{
	uint64_t tx_offload_capa;

	RTE_SET_USED(dev);
	tx_offload_capa = RTE_ETH_TX_OFFLOAD_VLAN_INSERT |
			  RTE_ETH_TX_OFFLOAD_IPV4_CKSUM  |
			  RTE_ETH_TX_OFFLOAD_UDP_CKSUM   |
			  RTE_ETH_TX_OFFLOAD_TCP_CKSUM   |
			  RTE_ETH_TX_OFFLOAD_SCTP_CKSUM  |
			  RTE_ETH_TX_OFFLOAD_TCP_TSO     |
			  RTE_ETH_TX_OFFLOAD_MULTI_SEGS  |
			  RTE_ETH_TX_OFFLOAD_SEND_ON_TIMESTAMP;

	return tx_offload_capa;
}

uint64_t
i350_get_tx_queue_offloads_capa(struct rte_eth_dev *dev)
{
	uint64_t tx_queue_offload_capa;

	tx_queue_offload_capa = i350_get_tx_port_offloads_capa(dev);

	return tx_queue_offload_capa;
}

int
eth_i350_tx_queue_setup(struct rte_eth_dev *dev,
			 uint16_t queue_idx,
			 uint16_t nb_desc,
			 unsigned int socket_id,
			 const struct rte_eth_txconf *tx_conf)
{
	const struct rte_memzone *tz;
	struct i350_tx_queue *txq;
	struct i350_hw     *hw;
	uint32_t size;
	uint64_t offloads;

	offloads = tx_conf->offloads | dev->data->dev_conf.txmode.offloads;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/*
	 * Validate number of transmit descriptors.
	 * It must not exceed hardware maximum, and must be multiple
	 * of E1000_ALIGN.
	 */
	if (nb_desc % I350_TXD_ALIGN != 0 ||
			(nb_desc > I350_MAX_RING_DESC) ||
			(nb_desc < I350_MIN_RING_DESC)) {
		return -EINVAL;
	}

	/*
	 * The tx_free_thresh and tx_rs_thresh values are not used in the 1G
	 * driver.
	 */
	if (tx_conf->tx_free_thresh != 0)
		PMD_INIT_LOG(INFO, "The tx_free_thresh parameter is not "
			     "used for the 1G driver.");
	if (tx_conf->tx_rs_thresh != 0)
		PMD_INIT_LOG(INFO, "The tx_rs_thresh parameter is not "
			     "used for the 1G driver.");
	if (tx_conf->tx_thresh.wthresh == 0)
		PMD_INIT_LOG(INFO, "To improve 1G driver performance, "
			     "consider setting the TX WTHRESH value to 4, 8, "
			     "or 16.");

	/* Free memory prior to re-allocation if needed */
	if (dev->data->tx_queues[queue_idx] != NULL) {
		i350_tx_queue_release(dev->data->tx_queues[queue_idx]);
		dev->data->tx_queues[queue_idx] = NULL;
	}

	/* First allocate the tx queue data structure */
	txq = rte_zmalloc("ethdev TX queue", sizeof(struct i350_tx_queue),
							RTE_CACHE_LINE_SIZE);
	if (txq == NULL)
		return -ENOMEM;

	/*
	 * Allocate TX ring hardware descriptors. A memzone large enough to
	 * handle the maximum ring size is allocated in order to allow for
	 * resizing in later calls to the queue setup function.
	 */
	size = sizeof(union i350_adv_tx_desc) * I350_MAX_RING_DESC;
	tz = rte_eth_dma_zone_reserve(dev, "tx_ring", queue_idx, size,
				      I350_ALIGN, socket_id);
	if (tz == NULL) {
		i350_tx_queue_release(txq);
		return -ENOMEM;
	}

	txq->mz = tz;
	txq->nb_tx_desc = nb_desc;
	txq->pthresh = tx_conf->tx_thresh.pthresh;
	txq->hthresh = tx_conf->tx_thresh.hthresh;
	txq->wthresh = tx_conf->tx_thresh.wthresh;
	txq->queue_id = queue_idx;
	txq->reg_idx = (uint16_t)((RTE_ETH_DEV_SRIOV(dev).active == 0) ?
		queue_idx : RTE_ETH_DEV_SRIOV(dev).def_pool_q_idx + queue_idx);
	txq->port_id = dev->data->port_id;

	txq->tdt_reg_addr = I350_PCI_REG_ADDR(hw, I350_TDT(txq->reg_idx));
	txq->tx_ring_phys_addr = tz->iova;

	txq->tx_ring = (union i350_adv_tx_desc *) tz->addr;
	/* Allocate software ring */
	txq->sw_ring = rte_zmalloc("txq->sw_ring",
				   sizeof(struct i350_tx_entry) * nb_desc,
				   RTE_CACHE_LINE_SIZE);
	if (txq->sw_ring == NULL) {
		i350_tx_queue_release(txq);
		return -ENOMEM;
	}
	PMD_INIT_LOG(DEBUG, "sw_ring=%p hw_ring=%p dma_addr=0x%"PRIx64,
		     txq->sw_ring, txq->tx_ring, txq->tx_ring_phys_addr);

	i350_reset_tx_queue(txq, dev);
	dev->tx_pkt_burst = eth_i350_xmit_pkts;
	dev->tx_pkt_prepare = &eth_i350_prep_pkts;
	dev->data->tx_queues[queue_idx] = txq;
	txq->offloads = offloads;

	return 0;
}

static void
i350_rx_queue_release_mbufs(struct i350_rx_queue *rxq)
{
	unsigned i;

	if (rxq->sw_ring != NULL) {
		for (i = 0; i < rxq->nb_rx_desc; i++) {
			if (rxq->sw_ring[i].mbuf != NULL) {
				rte_pktmbuf_free_seg(rxq->sw_ring[i].mbuf);
				rxq->sw_ring[i].mbuf = NULL;
			}
		}
	}
}

static void
i350_rx_queue_release(struct i350_rx_queue *rxq)
{
	if (rxq != NULL) {
		i350_rx_queue_release_mbufs(rxq);
		rte_free(rxq->sw_ring);
		rte_memzone_free(rxq->mz);
		rte_free(rxq);
	}
}

void
eth_i350_rx_queue_release(struct rte_eth_dev *dev, uint16_t qid)
{
	i350_rx_queue_release(dev->data->rx_queues[qid]);
}

static void
i350_reset_rx_queue(struct i350_rx_queue *rxq)
{
	static const union i350_adv_rx_desc zeroed_desc = {{0}};
	unsigned i;

	/* Zero out HW ring memory */
	for (i = 0; i < rxq->nb_rx_desc; i++) {
		rxq->rx_ring[i] = zeroed_desc;
	}

	rxq->rx_tail = 0;
	rxq->pkt_first_seg = NULL;
	rxq->pkt_last_seg = NULL;
}

uint64_t
i350_get_rx_port_offloads_capa(struct rte_eth_dev *dev)
{
	uint64_t rx_offload_capa;
	struct i350_hw *hw;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	rx_offload_capa = RTE_ETH_RX_OFFLOAD_VLAN_STRIP  |
			  RTE_ETH_RX_OFFLOAD_VLAN_FILTER |
			  RTE_ETH_RX_OFFLOAD_IPV4_CKSUM  |
			  RTE_ETH_RX_OFFLOAD_UDP_CKSUM   |
			  RTE_ETH_RX_OFFLOAD_TCP_CKSUM   |
			  RTE_ETH_RX_OFFLOAD_KEEP_CRC    |
			  RTE_ETH_RX_OFFLOAD_SCATTER     |
			  RTE_ETH_RX_OFFLOAD_RSS_HASH;

	rx_offload_capa |= RTE_ETH_RX_OFFLOAD_VLAN_EXTEND;

	return rx_offload_capa;
}

uint64_t
i350_get_rx_queue_offloads_capa(struct rte_eth_dev *dev)
{
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint64_t rx_queue_offload_capa;

	switch (hw->mac.type) {
	case i350_vfadapt_i350:
		/*
		 * As only one Rx queue can be used, let per queue offloading
		 * capability be same to per port queue offloading capability
		 * for better convenience.
		 */
		rx_queue_offload_capa = i350_get_rx_port_offloads_capa(dev);
		break;
	default:
		rx_queue_offload_capa = 0;
	}
	return rx_queue_offload_capa;
}

int
eth_i350_rx_queue_setup(struct rte_eth_dev *dev,
			 uint16_t queue_idx,
			 uint16_t nb_desc,
			 unsigned int socket_id,
			 const struct rte_eth_rxconf *rx_conf,
			 struct rte_mempool *mp)
{
	const struct rte_memzone *rz;
	struct i350_rx_queue *rxq;
	struct i350_hw     *hw;
	unsigned int size;
	uint64_t offloads;

	offloads = rx_conf->offloads | dev->data->dev_conf.rxmode.offloads;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/*
	 * Validate number of receive descriptors.
	 * It must not exceed hardware maximum, and must be multiple
	 * of E1000_ALIGN.
	 */
	if (nb_desc % I350_RXD_ALIGN != 0 ||
			(nb_desc > I350_MAX_RING_DESC) ||
			(nb_desc < I350_MIN_RING_DESC)) {
		return -EINVAL;
	}

	/* Free memory prior to re-allocation if needed */
	if (dev->data->rx_queues[queue_idx] != NULL) {
		i350_rx_queue_release(dev->data->rx_queues[queue_idx]);
		dev->data->rx_queues[queue_idx] = NULL;
	}

	/* First allocate the RX queue data structure. */
	rxq = rte_zmalloc("ethdev RX queue", sizeof(struct i350_rx_queue),
			  RTE_CACHE_LINE_SIZE);
	if (rxq == NULL)
		return -ENOMEM;
	rxq->offloads = offloads;
	rxq->mb_pool = mp;
	rxq->nb_rx_desc = nb_desc;
	rxq->pthresh = rx_conf->rx_thresh.pthresh;
	rxq->hthresh = rx_conf->rx_thresh.hthresh;
	rxq->wthresh = rx_conf->rx_thresh.wthresh;
	if (rxq->wthresh > 0 &&
	    (hw->mac.type == i350_82576 || hw->mac.type == i350_vfadapt_i350))
		rxq->wthresh = 1;
	rxq->drop_en = rx_conf->rx_drop_en;
	rxq->rx_free_thresh = rx_conf->rx_free_thresh;
	rxq->queue_id = queue_idx;
	rxq->reg_idx = (uint16_t)((RTE_ETH_DEV_SRIOV(dev).active == 0) ?
		queue_idx : RTE_ETH_DEV_SRIOV(dev).def_pool_q_idx + queue_idx);
	rxq->port_id = dev->data->port_id;
	if (dev->data->dev_conf.rxmode.offloads & RTE_ETH_RX_OFFLOAD_KEEP_CRC)
		rxq->crc_len = RTE_ETHER_CRC_LEN;
	else
		rxq->crc_len = 0;

	/*
	 *  Allocate RX ring hardware descriptors. A memzone large enough to
	 *  handle the maximum ring size is allocated in order to allow for
	 *  resizing in later calls to the queue setup function.
	 */
	size = sizeof(union i350_adv_rx_desc) * I350_MAX_RING_DESC;
	rz = rte_eth_dma_zone_reserve(dev, "rx_ring", queue_idx, size,
				      I350_ALIGN, socket_id);
	if (rz == NULL) {
		i350_rx_queue_release(rxq);
		return -ENOMEM;
	}

	rxq->mz = rz;
	rxq->rdt_reg_addr = I350_PCI_REG_ADDR(hw, I350_RDT(rxq->reg_idx));
	rxq->rdh_reg_addr = I350_PCI_REG_ADDR(hw, I350_RDH(rxq->reg_idx));
	rxq->rx_ring_phys_addr = rz->iova;
	rxq->rx_ring = (union i350_adv_rx_desc *) rz->addr;

	/* Allocate software ring. */
	rxq->sw_ring = rte_zmalloc("rxq->sw_ring",
				   sizeof(struct i350_rx_entry) * nb_desc,
				   RTE_CACHE_LINE_SIZE);
	if (rxq->sw_ring == NULL) {
		i350_rx_queue_release(rxq);
		return -ENOMEM;
	}
	PMD_INIT_LOG(DEBUG, "sw_ring=%p hw_ring=%p dma_addr=0x%"PRIx64,
		     rxq->sw_ring, rxq->rx_ring, rxq->rx_ring_phys_addr);

	dev->data->rx_queues[queue_idx] = rxq;
	i350_reset_rx_queue(rxq);

	return 0;
}

uint32_t
eth_i350_rx_queue_count(void *rx_queue)
{
#define I350_RXQ_SCAN_INTERVAL 4
	volatile union i350_adv_rx_desc *rxdp;
	struct i350_rx_queue *rxq;
	uint32_t desc = 0;

	rxq = rx_queue;
	rxdp = &(rxq->rx_ring[rxq->rx_tail]);

	while ((desc < rxq->nb_rx_desc) &&
		(rxdp->wb.upper.status_error & I350_RXD_STAT_DD)) {
		desc += I350_RXQ_SCAN_INTERVAL;
		rxdp += I350_RXQ_SCAN_INTERVAL;
		if (rxq->rx_tail + desc >= rxq->nb_rx_desc)
			rxdp = &(rxq->rx_ring[rxq->rx_tail +
				desc - rxq->nb_rx_desc]);
	}

	return desc;
}

int
eth_i350_rx_descriptor_status(void *rx_queue, uint16_t offset)
{
	struct i350_rx_queue *rxq = rx_queue;
	volatile uint32_t *status;
	uint32_t desc;

	if (unlikely(offset >= rxq->nb_rx_desc))
		return -EINVAL;

	if (offset >= rxq->nb_rx_desc - rxq->nb_rx_hold)
		return RTE_ETH_RX_DESC_UNAVAIL;

	desc = rxq->rx_tail + offset;
	if (desc >= rxq->nb_rx_desc)
		desc -= rxq->nb_rx_desc;

	status = &rxq->rx_ring[desc].wb.upper.status_error;
	if (*status & rte_cpu_to_le_32(I350_RXD_STAT_DD))
		return RTE_ETH_RX_DESC_DONE;

	return RTE_ETH_RX_DESC_AVAIL;
}

int
eth_i350_tx_descriptor_status(void *tx_queue, uint16_t offset)
{
	struct i350_tx_queue *txq = tx_queue;
	volatile uint32_t *status;
	uint32_t desc;

	if (unlikely(offset >= txq->nb_tx_desc))
		return -EINVAL;

	desc = txq->tx_tail + offset;
	if (desc >= txq->nb_tx_desc)
		desc -= txq->nb_tx_desc;

	status = &txq->tx_ring[desc].wb.status;
	if (*status & rte_cpu_to_le_32(I350_TXD_STAT_DD))
		return RTE_ETH_TX_DESC_DONE;

	return RTE_ETH_TX_DESC_FULL;
}

void
i350_dev_clear_queues(struct rte_eth_dev *dev)
{
	uint16_t i;
	struct i350_tx_queue *txq;
	struct i350_rx_queue *rxq;

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		txq = dev->data->tx_queues[i];
		if (txq != NULL) {
			i350_tx_queue_release_mbufs(txq);
			i350_reset_tx_queue(txq, dev);
			dev->data->tx_queue_state[i] = RTE_ETH_QUEUE_STATE_STOPPED;
		}
	}

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		rxq = dev->data->rx_queues[i];
		if (rxq != NULL) {
			i350_rx_queue_release_mbufs(rxq);
			i350_reset_rx_queue(rxq);
			dev->data->rx_queue_state[i] = RTE_ETH_QUEUE_STATE_STOPPED;
		}
	}
}

void
i350_dev_free_queues(struct rte_eth_dev *dev)
{
	uint16_t i;

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		eth_i350_rx_queue_release(dev, i);
		dev->data->rx_queues[i] = NULL;
	}
	dev->data->nb_rx_queues = 0;

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		eth_i350_tx_queue_release(dev, i);
		dev->data->tx_queues[i] = NULL;
	}
	dev->data->nb_tx_queues = 0;
}

/**
 * Receive Side Scaling (RSS).
 * See section 7.1.1.7 in the following document:
 *     "Intel 82576 GbE Controller Datasheet" - Revision 2.45 October 2009
 *
 * Principles:
 * The source and destination IP addresses of the IP header and the source and
 * destination ports of TCP/UDP headers, if any, of received packets are hashed
 * against a configurable random key to compute a 32-bit RSS hash result.
 * The seven (7) LSBs of the 32-bit hash result are used as an index into a
 * 128-entry redirection table (RETA).  Each entry of the RETA provides a 3-bit
 * RSS output index which is used as the RX queue index where to store the
 * received packets.
 * The following output is supplied in the RX write-back descriptor:
 *     - 32-bit result of the Microsoft RSS hash function,
 *     - 4-bit RSS type field.
 */

/*
 * RSS random key supplied in section 7.1.1.7.3 of the Intel 82576 datasheet.
 * Used as the default key.
 */
static uint8_t rss_intel_key[40] = {
	0x6D, 0x5A, 0x56, 0xDA, 0x25, 0x5B, 0x0E, 0xC2,
	0x41, 0x67, 0x25, 0x3D, 0x43, 0xA3, 0x8F, 0xB0,
	0xD0, 0xCA, 0x2B, 0xCB, 0xAE, 0x7B, 0x30, 0xB4,
	0x77, 0xCB, 0x2D, 0xA3, 0x80, 0x30, 0xF2, 0x0C,
	0x6A, 0x42, 0xB7, 0x3B, 0xBE, 0xAC, 0x01, 0xFA,
};

static void
i350_rss_disable(struct rte_eth_dev *dev)
{
	struct i350_hw *hw;
	uint32_t mrqc;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	mrqc = I350_READ_REG(hw, I350_MRQC);
	mrqc &= ~I350_MRQC_ENABLE_MASK;
	I350_WRITE_REG(hw, I350_MRQC, mrqc);
}

static void
i350_hw_rss_hash_set(struct i350_hw *hw, struct rte_eth_rss_conf *rss_conf)
{
	uint8_t  *hash_key;
	uint32_t rss_key;
	uint32_t mrqc;
	uint64_t rss_hf;
	uint16_t i;

	hash_key = rss_conf->rss_key;
	if (hash_key != NULL) {
		/* Fill in RSS hash key */
		for (i = 0; i < 10; i++) {
			rss_key  = hash_key[(i * 4)];
			rss_key |= hash_key[(i * 4) + 1] << 8;
			rss_key |= hash_key[(i * 4) + 2] << 16;
			rss_key |= hash_key[(i * 4) + 3] << 24;
			I350_WRITE_REG_ARRAY(hw, I350_RSSRK(0), i, rss_key);
		}
	}

	/* Set configured hashing protocols in MRQC register */
	rss_hf = rss_conf->rss_hf;
	mrqc = I350_MRQC_ENABLE_RSS_4Q; /* RSS enabled. */
	if (rss_hf & RTE_ETH_RSS_IPV4)
		mrqc |= I350_MRQC_RSS_FIELD_IPV4;
	if (rss_hf & RTE_ETH_RSS_NONFRAG_IPV4_TCP)
		mrqc |= I350_MRQC_RSS_FIELD_IPV4_TCP;
	if (rss_hf & RTE_ETH_RSS_IPV6)
		mrqc |= I350_MRQC_RSS_FIELD_IPV6;
	if (rss_hf & RTE_ETH_RSS_IPV6_EX)
		mrqc |= I350_MRQC_RSS_FIELD_IPV6_EX;
	if (rss_hf & RTE_ETH_RSS_NONFRAG_IPV6_TCP)
		mrqc |= I350_MRQC_RSS_FIELD_IPV6_TCP;
	if (rss_hf & RTE_ETH_RSS_IPV6_TCP_EX)
		mrqc |= I350_MRQC_RSS_FIELD_IPV6_TCP_EX;
	if (rss_hf & RTE_ETH_RSS_NONFRAG_IPV4_UDP)
		mrqc |= I350_MRQC_RSS_FIELD_IPV4_UDP;
	if (rss_hf & RTE_ETH_RSS_NONFRAG_IPV6_UDP)
		mrqc |= I350_MRQC_RSS_FIELD_IPV6_UDP;
	if (rss_hf & RTE_ETH_RSS_IPV6_UDP_EX)
		mrqc |= I350_MRQC_RSS_FIELD_IPV6_UDP_EX;
	I350_WRITE_REG(hw, I350_MRQC, mrqc);
}

int
eth_i350_rss_hash_update(struct rte_eth_dev *dev,
			struct rte_eth_rss_conf *rss_conf)
{
	struct i350_hw *hw;
	uint32_t mrqc;
	uint64_t rss_hf;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/*
	 * Before changing anything, first check that the update RSS operation
	 * does not attempt to disable RSS, if RSS was enabled at
	 * initialization time, or does not attempt to enable RSS, if RSS was
	 * disabled at initialization time.
	 */
	rss_hf = rss_conf->rss_hf & I350_RSS_OFFLOAD_ALL;
	mrqc = I350_READ_REG(hw, I350_MRQC);
	if (!(mrqc & I350_MRQC_ENABLE_MASK)) { /* RSS disabled */
		if (rss_hf != 0) /* Enable RSS */
			return -(EINVAL);
		return 0; /* Nothing to do */
	}
	/* RSS enabled */
	if (rss_hf == 0) /* Disable RSS */
		return -(EINVAL);
	i350_hw_rss_hash_set(hw, rss_conf);
	return 0;
}

int eth_i350_rss_hash_conf_get(struct rte_eth_dev *dev,
			      struct rte_eth_rss_conf *rss_conf)
{
	struct i350_hw *hw;
	uint8_t *hash_key;
	uint32_t rss_key;
	uint32_t mrqc;
	uint64_t rss_hf;
	uint16_t i;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	hash_key = rss_conf->rss_key;
	if (hash_key != NULL) {
		/* Return RSS hash key */
		for (i = 0; i < 10; i++) {
			rss_key = I350_READ_REG_ARRAY(hw, I350_RSSRK(0), i);
			hash_key[(i * 4)] = rss_key & 0x000000FF;
			hash_key[(i * 4) + 1] = (rss_key >> 8) & 0x000000FF;
			hash_key[(i * 4) + 2] = (rss_key >> 16) & 0x000000FF;
			hash_key[(i * 4) + 3] = (rss_key >> 24) & 0x000000FF;
		}
	}

	/* Get RSS functions configured in MRQC register */
	mrqc = I350_READ_REG(hw, I350_MRQC);
	if ((mrqc & I350_MRQC_ENABLE_RSS_4Q) == 0) { /* RSS is disabled */
		rss_conf->rss_hf = 0;
		return 0;
	}
	rss_hf = 0;
	if (mrqc & I350_MRQC_RSS_FIELD_IPV4)
		rss_hf |= RTE_ETH_RSS_IPV4;
	if (mrqc & I350_MRQC_RSS_FIELD_IPV4_TCP)
		rss_hf |= RTE_ETH_RSS_NONFRAG_IPV4_TCP;
	if (mrqc & I350_MRQC_RSS_FIELD_IPV6)
		rss_hf |= RTE_ETH_RSS_IPV6;
	if (mrqc & I350_MRQC_RSS_FIELD_IPV6_EX)
		rss_hf |= RTE_ETH_RSS_IPV6_EX;
	if (mrqc & I350_MRQC_RSS_FIELD_IPV6_TCP)
		rss_hf |= RTE_ETH_RSS_NONFRAG_IPV6_TCP;
	if (mrqc & I350_MRQC_RSS_FIELD_IPV6_TCP_EX)
		rss_hf |= RTE_ETH_RSS_IPV6_TCP_EX;
	if (mrqc & I350_MRQC_RSS_FIELD_IPV4_UDP)
		rss_hf |= RTE_ETH_RSS_NONFRAG_IPV4_UDP;
	if (mrqc & I350_MRQC_RSS_FIELD_IPV6_UDP)
		rss_hf |= RTE_ETH_RSS_NONFRAG_IPV6_UDP;
	if (mrqc & I350_MRQC_RSS_FIELD_IPV6_UDP_EX)
		rss_hf |= RTE_ETH_RSS_IPV6_UDP_EX;
	rss_conf->rss_hf = rss_hf;
	return 0;
}

static void
i350_rss_configure(struct rte_eth_dev *dev)
{
	struct rte_eth_rss_conf rss_conf;
	struct i350_hw *hw;
	uint32_t shift;
	uint16_t i;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Fill in redirection table. */
	shift = (hw->mac.type == i350_82575) ? 6 : 0;
	for (i = 0; i < 128; i++) {
		union i350_reta {
			uint32_t dword;
			uint8_t  bytes[4];
		} reta;
		uint8_t q_idx;

		q_idx = (uint8_t) ((dev->data->nb_rx_queues > 1) ?
				   i % dev->data->nb_rx_queues : 0);
		reta.bytes[i & 3] = (uint8_t) (q_idx << shift);
		if ((i & 3) == 3)
			I350_WRITE_REG(hw, I350_RETA(i >> 2), reta.dword);
	}

	/*
	 * Configure the RSS key and the RSS protocols used to compute
	 * the RSS hash of input packets.
	 */
	rss_conf = dev->data->dev_conf.rx_adv_conf.rss_conf;
	if ((rss_conf.rss_hf & I350_RSS_OFFLOAD_ALL) == 0) {
		i350_rss_disable(dev);
		return;
	}
	if (rss_conf.rss_key == NULL)
		rss_conf.rss_key = rss_intel_key; /* Default hash key */
	i350_hw_rss_hash_set(hw, &rss_conf);
}

/*
 * Check if the mac type support VMDq or not.
 * Return 1 if it supports, otherwise, return 0.
 */
static int
i350_is_vmdq_supported(const struct rte_eth_dev *dev)
{
	const struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	switch (hw->mac.type) {
	case i350_82576:
	case i350_82580:
	case i350_i350:
		return 1;
	case i350_82540:
	case i350_82541:
	case i350_82542:
	case i350_82543:
	case i350_82544:
	case i350_82545:
	case i350_82546:
	case i350_82547:
	case i350_82571:
	case i350_82572:
	case i350_82573:
	case i350_82574:
	case i350_82583:
	case i350_i210:
	case i350_i211:
	default:
		PMD_INIT_LOG(ERR, "Cannot support VMDq feature");
		return 0;
	}
}

static int
i350_vmdq_rx_hw_configure(struct rte_eth_dev *dev)
{
	struct rte_eth_vmdq_rx_conf *cfg;
	struct i350_hw *hw;
	uint32_t mrqc, vt_ctl, vmolr, rctl;
	int i;

	PMD_INIT_FUNC_TRACE();

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	cfg = &dev->data->dev_conf.rx_adv_conf.vmdq_rx_conf;

	/* Check if mac type can support VMDq, return value of 0 means NOT support */
	if (i350_is_vmdq_supported(dev) == 0)
		return -1;

	i350_rss_disable(dev);

	/* RCTL: enable VLAN filter */
	rctl = I350_READ_REG(hw, I350_RCTL);
	rctl |= I350_RCTL_VFE;
	I350_WRITE_REG(hw, I350_RCTL, rctl);

	/* MRQC: enable vmdq */
	mrqc = I350_READ_REG(hw, I350_MRQC);
	mrqc |= I350_MRQC_ENABLE_VMDQ;
	I350_WRITE_REG(hw, I350_MRQC, mrqc);

	/* VTCTL:  pool selection according to VLAN tag */
	vt_ctl = I350_READ_REG(hw, I350_VT_CTL);
	if (cfg->enable_default_pool)
		vt_ctl |= (cfg->default_pool << I350_VT_CTL_DEFAULT_POOL_SHIFT);
	vt_ctl |= I350_VT_CTL_IGNORE_MAC;
	I350_WRITE_REG(hw, I350_VT_CTL, vt_ctl);

	for (i = 0; i < I350_VMOLR_SIZE; i++) {
		vmolr = I350_READ_REG(hw, I350_VMOLR(i));
		vmolr &= ~(I350_VMOLR_AUPE | I350_VMOLR_ROMPE |
			I350_VMOLR_ROPE | I350_VMOLR_BAM |
			I350_VMOLR_MPME);

		if (cfg->rx_mode & RTE_ETH_VMDQ_ACCEPT_UNTAG)
			vmolr |= I350_VMOLR_AUPE;
		if (cfg->rx_mode & RTE_ETH_VMDQ_ACCEPT_HASH_MC)
			vmolr |= I350_VMOLR_ROMPE;
		if (cfg->rx_mode & RTE_ETH_VMDQ_ACCEPT_HASH_UC)
			vmolr |= I350_VMOLR_ROPE;
		if (cfg->rx_mode & RTE_ETH_VMDQ_ACCEPT_BROADCAST)
			vmolr |= I350_VMOLR_BAM;
		if (cfg->rx_mode & RTE_ETH_VMDQ_ACCEPT_MULTICAST)
			vmolr |= I350_VMOLR_MPME;

		I350_WRITE_REG(hw, I350_VMOLR(i), vmolr);
	}

	/*
	 * VMOLR: set STRVLAN as 1 if IGMAC in VTCTL is set as 1
	 * Both 82576 and 82580 support it
	 */
	if (hw->mac.type != i350_i350) {
		for (i = 0; i < I350_VMOLR_SIZE; i++) {
			vmolr = I350_READ_REG(hw, I350_VMOLR(i));
			vmolr |= I350_VMOLR_STRVLAN;
			I350_WRITE_REG(hw, I350_VMOLR(i), vmolr);
		}
	}

	/* VFTA - enable all vlan filters */
	for (i = 0; i < I350_VFTA_SIZE; i++)
		I350_WRITE_REG(hw, (I350_VFTA+(i*4)), UINT32_MAX);

	/* VFRE: 8 pools enabling for rx, both 82576 and i350 support it */
	if (hw->mac.type != i350_82580)
		I350_WRITE_REG(hw, I350_VFRE, I350_MBVFICR_VFREQ_MASK);

	/*
	 * RAH/RAL - allow pools to read specific mac addresses
	 * In this case, all pools should be able to read from mac addr 0
	 */
	I350_WRITE_REG(hw, I350_RAH(0), (I350_RAH_AV | UINT16_MAX));
	I350_WRITE_REG(hw, I350_RAL(0), UINT32_MAX);

	/* VLVF: set up filters for vlan tags as configured */
	for (i = 0; i < cfg->nb_pool_maps; i++) {
		/* set vlan id in VF register and set the valid bit */
		I350_WRITE_REG(hw, I350_VLVF(i), (I350_VLVF_VLANID_ENABLE |
			(cfg->pool_map[i].vlan_id & RTE_ETH_VLAN_ID_MAX) |
			((cfg->pool_map[i].pools << I350_VLVF_POOLSEL_SHIFT) &
			I350_VLVF_POOLSEL_MASK)));
	}

	I350_WRITE_FLUSH(hw);

	return 0;
}


/*********************************************************************
 *
 *  Enable receive unit.
 *
 **********************************************************************/

static int
i350_alloc_rx_queue_mbufs(struct i350_rx_queue *rxq)
{
	struct i350_rx_entry *rxe = rxq->sw_ring;
	uint64_t dma_addr;
	unsigned i;

	/* Initialize software ring entries. */
	for (i = 0; i < rxq->nb_rx_desc; i++) {
		volatile union i350_adv_rx_desc *rxd;
		struct rte_mbuf *mbuf = rte_mbuf_raw_alloc(rxq->mb_pool);

		if (mbuf == NULL) {
			PMD_INIT_LOG(ERR, "RX mbuf alloc failed "
				     "queue_id=%hu", rxq->queue_id);
			return -ENOMEM;
		}
		dma_addr =
			rte_cpu_to_le_64(rte_mbuf_data_iova_default(mbuf));
		rxd = &rxq->rx_ring[i];
		rxd->read.hdr_addr = 0;
		rxd->read.pkt_addr = dma_addr;
		rxe[i].mbuf = mbuf;
	}

	return 0;
}

#define I350_MRQC_DEF_Q_SHIFT               (3)
static int
i350_dev_mq_rx_configure(struct rte_eth_dev *dev)
{
	struct i350_hw *hw =
		I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t mrqc;

	if (RTE_ETH_DEV_SRIOV(dev).active == RTE_ETH_8_POOLS) {
		/*
		 * SRIOV active scheme
		 * FIXME if support RSS together with VMDq & SRIOV
		 */
		mrqc = I350_MRQC_ENABLE_VMDQ;
		/* 011b Def_Q ignore, according to VT_CTL.DEF_PL */
		mrqc |= 0x3 << I350_MRQC_DEF_Q_SHIFT;
		I350_WRITE_REG(hw, I350_MRQC, mrqc);
	} else if(RTE_ETH_DEV_SRIOV(dev).active == 0) {
		/*
		 * SRIOV inactive scheme
		 */
		switch (dev->data->dev_conf.rxmode.mq_mode) {
			case RTE_ETH_MQ_RX_RSS:
				i350_rss_configure(dev);
				break;
			case RTE_ETH_MQ_RX_VMDQ_ONLY:
				/*Configure general VMDQ only RX parameters*/
				i350_vmdq_rx_hw_configure(dev);
				break;
			case RTE_ETH_MQ_RX_NONE:
				/* if mq_mode is none, disable rss mode.*/
			default:
				i350_rss_disable(dev);
				break;
		}
	}

	return 0;
}

int
eth_i350_rx_init(struct rte_eth_dev *dev)
{
	struct rte_eth_rxmode *rxmode;
	struct i350_hw     *hw;
	struct i350_rx_queue *rxq;
	uint32_t rctl;
	uint32_t rxcsum;
	uint32_t srrctl;
	uint16_t buf_size;
	uint16_t rctl_bsize;
	uint32_t max_len;
	uint16_t i;
	int ret;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	srrctl = 0;

	/*
	 * Make sure receives are disabled while setting
	 * up the descriptor ring.
	 */
	rctl = I350_READ_REG(hw, I350_RCTL);
	I350_WRITE_REG(hw, I350_RCTL, rctl & ~I350_RCTL_EN);

	rxmode = &dev->data->dev_conf.rxmode;

	/*
	 * Configure support of jumbo frames, if any.
	 */
	max_len = dev->data->mtu + I350_ETH_OVERHEAD;
	if (dev->data->mtu > RTE_ETHER_MTU) {
		rctl |= I350_RCTL_LPE;

		/*
		 * Set maximum packet length by default, and might be updated
		 * together with enabling/disabling dual VLAN.
		 */
		if (rxmode->offloads & RTE_ETH_RX_OFFLOAD_VLAN_EXTEND)
			max_len += VLAN_TAG_SIZE;

		I350_WRITE_REG(hw, I350_RLPML, max_len);
	} else
		rctl &= ~I350_RCTL_LPE;

	/* Configure and enable each RX queue. */
	rctl_bsize = 0;
	dev->rx_pkt_burst = eth_i350_recv_pkts;
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		uint64_t bus_addr;
		uint32_t rxdctl;

		rxq = dev->data->rx_queues[i];

		rxq->flags = 0;
		/*
		 * i350 and i354 vlan packets have vlan tags byte swapped.
		 */

		rxq->flags |= I350_RXQ_FLAG_LB_BSWAP_VLAN;
		PMD_INIT_LOG(DEBUG, "I350 rx vlan bswap required");


		/* Allocate buffers for descriptor rings and set up queue */
		ret = i350_alloc_rx_queue_mbufs(rxq);
		if (ret)
			return ret;

		/*
		 * Reset crc_len in case it was changed after queue setup by a
		 *  call to configure
		 */
		if (dev->data->dev_conf.rxmode.offloads & RTE_ETH_RX_OFFLOAD_KEEP_CRC)
			rxq->crc_len = RTE_ETHER_CRC_LEN;
		else
			rxq->crc_len = 0;

		bus_addr = rxq->rx_ring_phys_addr;
		I350_WRITE_REG(hw, I350_RDLEN(rxq->reg_idx),
				rxq->nb_rx_desc *
				sizeof(union i350_adv_rx_desc));
		I350_WRITE_REG(hw, I350_RDBAH(rxq->reg_idx),
				(uint32_t)(bus_addr >> 32));
		I350_WRITE_REG(hw, I350_RDBAL(rxq->reg_idx), (uint32_t)bus_addr);

		srrctl = I350_SRRCTL_DESCTYPE_ADV_ONEBUF;

		/*
		 * Configure RX buffer size.
		 */
		buf_size = (uint16_t)(rte_pktmbuf_data_room_size(rxq->mb_pool) -
			RTE_PKTMBUF_HEADROOM);
		if (buf_size >= 1024) {
			/*
			 * Configure the BSIZEPACKET field of the SRRCTL
			 * register of the queue.
			 * Value is in 1 KB resolution, from 1 KB to 127 KB.
			 * If this field is equal to 0b, then RCTL.BSIZE
			 * determines the RX packet buffer size.
			 */
			srrctl |= ((buf_size >> I350_SRRCTL_BSIZEPKT_SHIFT) &
				   I350_SRRCTL_BSIZEPKT_MASK);
			buf_size = (uint16_t) ((srrctl &
						I350_SRRCTL_BSIZEPKT_MASK) <<
					       I350_SRRCTL_BSIZEPKT_SHIFT);

			/* It adds dual VLAN length for supporting dual VLAN */
			if ((max_len + 2 * VLAN_TAG_SIZE) > buf_size) {
				if (!dev->data->scattered_rx)
					PMD_INIT_LOG(DEBUG,
						     "forcing scatter mode");
				dev->rx_pkt_burst = eth_i350_recv_scattered_pkts;
				dev->data->scattered_rx = 1;
			}
		} else {
			/*
			 * Use BSIZE field of the device RCTL register.
			 */
			if ((rctl_bsize == 0) || (rctl_bsize > buf_size))
				rctl_bsize = buf_size;
			if (!dev->data->scattered_rx)
				PMD_INIT_LOG(DEBUG, "forcing scatter mode");
			dev->rx_pkt_burst = eth_i350_recv_scattered_pkts;
			dev->data->scattered_rx = 1;
		}

		/* Set if packets are dropped when no descriptors available */
		if (rxq->drop_en)
			srrctl |= I350_SRRCTL_DROP_EN;

		I350_WRITE_REG(hw, I350_SRRCTL(rxq->reg_idx), srrctl);

		/* Enable this RX queue. */
		rxdctl = I350_READ_REG(hw, I350_RXDCTL(rxq->reg_idx));
		rxdctl |= I350_RXDCTL_QUEUE_ENABLE;
		rxdctl &= 0xFFF00000;
		rxdctl |= (rxq->pthresh & 0x1F);
		rxdctl |= ((rxq->hthresh & 0x1F) << 8);
		rxdctl |= ((rxq->wthresh & 0x1F) << 16);
		I350_WRITE_REG(hw, I350_RXDCTL(rxq->reg_idx), rxdctl);
		dev->data->rx_queue_state[i] = RTE_ETH_QUEUE_STATE_STARTED;
	}

	if (dev->data->dev_conf.rxmode.offloads & RTE_ETH_RX_OFFLOAD_SCATTER) {
		if (!dev->data->scattered_rx)
			PMD_INIT_LOG(DEBUG, "forcing scatter mode");
		dev->rx_pkt_burst = eth_i350_recv_scattered_pkts;
		dev->data->scattered_rx = 1;
	}

	/*
	 * Setup BSIZE field of RCTL register, if needed.
	 * Buffer sizes >= 1024 are not [supposed to be] setup in the RCTL
	 * register, since the code above configures the SRRCTL register of
	 * the RX queue in such a case.
	 * All configurable sizes are:
	 * 16384: rctl |= (E1000_RCTL_SZ_16384 | E1000_RCTL_BSEX);
	 *  8192: rctl |= (E1000_RCTL_SZ_8192  | E1000_RCTL_BSEX);
	 *  4096: rctl |= (E1000_RCTL_SZ_4096  | E1000_RCTL_BSEX);
	 *  2048: rctl |= E1000_RCTL_SZ_2048;
	 *  1024: rctl |= E1000_RCTL_SZ_1024;
	 *   512: rctl |= E1000_RCTL_SZ_512;
	 *   256: rctl |= E1000_RCTL_SZ_256;
	 */
	if (rctl_bsize > 0) {
		if (rctl_bsize >= 512) /* 512 <= buf_size < 1024 - use 512 */
			rctl |= I350_RCTL_SZ_512;
		else /* 256 <= buf_size < 512 - use 256 */
			rctl |= I350_RCTL_SZ_256;
	}

	/*
	 * Configure RSS if device configured with multiple RX queues.
	 */
	i350_dev_mq_rx_configure(dev);

	/* Update the rctl since igb_dev_mq_rx_configure may change its value */
	rctl |= I350_READ_REG(hw, I350_RCTL);

	/*
	 * Setup the Checksum Register.
	 * Receive Full-Packet Checksum Offload is mutually exclusive with RSS.
	 */
	rxcsum = I350_READ_REG(hw, I350_RXCSUM);
	rxcsum |= I350_RXCSUM_PCSD;

	/* Enable both L3/L4 rx checksum offload */
	if (rxmode->offloads & RTE_ETH_RX_OFFLOAD_IPV4_CKSUM)
		rxcsum |= I350_RXCSUM_IPOFL;
	else
		rxcsum &= ~I350_RXCSUM_IPOFL;
	if (rxmode->offloads &
		(RTE_ETH_RX_OFFLOAD_TCP_CKSUM | RTE_ETH_RX_OFFLOAD_UDP_CKSUM))
		rxcsum |= I350_RXCSUM_TUOFL;
	else
		rxcsum &= ~I350_RXCSUM_TUOFL;
	if (rxmode->offloads & RTE_ETH_RX_OFFLOAD_CHECKSUM)
		rxcsum |= I350_RXCSUM_CRCOFL;
	else
		rxcsum &= ~I350_RXCSUM_CRCOFL;

	I350_WRITE_REG(hw, I350_RXCSUM, rxcsum);

	/* Setup the Receive Control Register. */
	if (dev->data->dev_conf.rxmode.offloads & RTE_ETH_RX_OFFLOAD_KEEP_CRC) {
		rctl &= ~I350_RCTL_SECRC; /* Do not Strip Ethernet CRC. */

		for (i = 0; i < dev->data->nb_rx_queues; i++) {
			rxq = dev->data->rx_queues[i];
			uint32_t dvmolr = I350_READ_REG(hw,
				I350_DVMOLR(rxq->reg_idx));
			dvmolr &= ~I350_DVMOLR_STRCRC;
			I350_WRITE_REG(hw, I350_DVMOLR(rxq->reg_idx), dvmolr);
		}
	} else {
		rctl |= I350_RCTL_SECRC; /* Strip Ethernet CRC. */

		/* set STRCRC bit in all queues */
		for (i = 0; i < dev->data->nb_rx_queues; i++) {
			rxq = dev->data->rx_queues[i];
			uint32_t dvmolr = I350_READ_REG(hw,
				I350_DVMOLR(rxq->reg_idx));
			dvmolr |= I350_DVMOLR_STRCRC;
			I350_WRITE_REG(hw, I350_DVMOLR(rxq->reg_idx), dvmolr);
		}
	}

	rctl &= ~(3 << I350_RCTL_MO_SHIFT);
	rctl |= I350_RCTL_EN | I350_RCTL_BAM | I350_RCTL_LBM_NO |
		I350_RCTL_RDMTS_HALF |
		(hw->mac.mc_filter_type << I350_RCTL_MO_SHIFT);

	/* Make sure VLAN Filters are off. */
	if (dev->data->dev_conf.rxmode.mq_mode != RTE_ETH_MQ_RX_VMDQ_ONLY)
		rctl &= ~I350_RCTL_VFE;
	/* Don't store bad packets. */
	rctl &= ~I350_RCTL_SBP;

	/* Enable Receives. */
	I350_WRITE_REG(hw, I350_RCTL, rctl);

	/*
	 * Setup the HW Rx Head and Tail Descriptor Pointers.
	 * This needs to be done after enable.
	 */
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		rxq = dev->data->rx_queues[i];
		I350_WRITE_REG(hw, I350_RDH(rxq->reg_idx), 0);
		I350_WRITE_REG(hw, I350_RDT(rxq->reg_idx), rxq->nb_rx_desc - 1);
	}

	return 0;
}

/*********************************************************************
 *
 *  Enable transmit unit.
 *
 **********************************************************************/
void
eth_i350_tx_init(struct rte_eth_dev *dev)
{
	struct i350_hw     *hw;
	struct i350_tx_queue *txq;
	uint64_t offloads = dev->data->dev_conf.txmode.offloads;
	uint32_t tctl;
	uint32_t txdctl;
	uint16_t i;
	int err;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Setup the Base and Length of the Tx Descriptor Rings. */
	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		uint64_t bus_addr;
		txq = dev->data->tx_queues[i];
		bus_addr = txq->tx_ring_phys_addr;

		I350_WRITE_REG(hw, I350_TDLEN(txq->reg_idx),
				txq->nb_tx_desc *
				sizeof(union i350_adv_tx_desc));
		I350_WRITE_REG(hw, I350_TDBAH(txq->reg_idx),
				(uint32_t)(bus_addr >> 32));
		I350_WRITE_REG(hw, I350_TDBAL(txq->reg_idx), (uint32_t)bus_addr);

		/* Setup the HW Tx Head and Tail descriptor pointers. */
		I350_WRITE_REG(hw, I350_TDT(txq->reg_idx), 0);
		I350_WRITE_REG(hw, I350_TDH(txq->reg_idx), 0);

		/* Setup Transmit threshold registers. */
		txdctl = I350_READ_REG(hw, I350_TXDCTL(txq->reg_idx));
		txdctl |= txq->pthresh & 0x1F;
		txdctl |= ((txq->hthresh & 0x1F) << 8);
		txdctl |= ((txq->wthresh & 0x1F) << 16);
		txdctl |= I350_TXDCTL_QUEUE_ENABLE;
		I350_WRITE_REG(hw, I350_TXDCTL(txq->reg_idx), txdctl);
		dev->data->tx_queue_state[i] = RTE_ETH_QUEUE_STATE_STARTED;
	}

	if (offloads & RTE_ETH_TX_OFFLOAD_SEND_ON_TIMESTAMP) {
		err = rte_mbuf_dyn_tx_timestamp_register(
			&i350_tx_timestamp_dynfield_offset,
			&i350_tx_timestamp_dynflag);
		if (err)
			PMD_DRV_LOG(ERR, "Failed to register tx timestamp dynamic field");
	}

	/* Program the Transmit Control Register. */
	tctl = I350_READ_REG(hw, I350_TCTL);
	tctl &= ~I350_TCTL_CT;
	tctl |= (I350_TCTL_PSP | I350_TCTL_RTLC | I350_TCTL_EN |
		 (I350_COLLISION_THRESHOLD << I350_CT_SHIFT));

	i350_config_collision_dist(hw);

	/* This write will effectively turn on the transmit unit. */
	I350_WRITE_REG(hw, I350_TCTL, tctl);
}

/*********************************************************************
 *
 *  Enable VF receive unit.
 *
 **********************************************************************/
int
eth_i350vf_rx_init(struct rte_eth_dev *dev)
{
	struct i350_hw     *hw;
	struct i350_rx_queue *rxq;
	uint32_t srrctl;
	uint16_t buf_size;
	uint16_t rctl_bsize;
	uint32_t max_len;
	uint16_t i;
	int ret;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* setup MTU */
	max_len = dev->data->mtu + I350_ETH_OVERHEAD;
	i350_rlpml_set_vf(hw, (uint16_t)(max_len + VLAN_TAG_SIZE));

	/* Configure and enable each RX queue. */
	rctl_bsize = 0;
	dev->rx_pkt_burst = eth_i350_recv_pkts;
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		uint64_t bus_addr;
		uint32_t rxdctl;

		rxq = dev->data->rx_queues[i];

		rxq->flags = 0;
		/*
		 * i350VF LB vlan packets have vlan tags byte swapped.
		 */
		if (hw->mac.type == i350_vfadapt_i350) {
			rxq->flags |= I350_RXQ_FLAG_LB_BSWAP_VLAN;
			PMD_INIT_LOG(DEBUG, "I350 rx vlan bswap required");
		} else {
			PMD_INIT_LOG(DEBUG, "I350 rx vlan bswap not required");
		}

		/* Allocate buffers for descriptor rings and set up queue */
		ret = i350_alloc_rx_queue_mbufs(rxq);
		if (ret)
			return ret;

		bus_addr = rxq->rx_ring_phys_addr;
		I350_WRITE_REG(hw, I350_RDLEN(i),
				rxq->nb_rx_desc *
				sizeof(union i350_adv_rx_desc));
		I350_WRITE_REG(hw, I350_RDBAH(i),
				(uint32_t)(bus_addr >> 32));
		I350_WRITE_REG(hw, I350_RDBAL(i), (uint32_t)bus_addr);

		srrctl = I350_SRRCTL_DESCTYPE_ADV_ONEBUF;

		/*
		 * Configure RX buffer size.
		 */
		buf_size = (uint16_t)(rte_pktmbuf_data_room_size(rxq->mb_pool) -
			RTE_PKTMBUF_HEADROOM);
		if (buf_size >= 1024) {
			/*
			 * Configure the BSIZEPACKET field of the SRRCTL
			 * register of the queue.
			 * Value is in 1 KB resolution, from 1 KB to 127 KB.
			 * If this field is equal to 0b, then RCTL.BSIZE
			 * determines the RX packet buffer size.
			 */
			srrctl |= ((buf_size >> I350_SRRCTL_BSIZEPKT_SHIFT) &
				   I350_SRRCTL_BSIZEPKT_MASK);
			buf_size = (uint16_t) ((srrctl &
						I350_SRRCTL_BSIZEPKT_MASK) <<
					       I350_SRRCTL_BSIZEPKT_SHIFT);

			/* It adds dual VLAN length for supporting dual VLAN */
			if ((max_len + 2 * VLAN_TAG_SIZE) > buf_size) {
				if (!dev->data->scattered_rx)
					PMD_INIT_LOG(DEBUG,
						     "forcing scatter mode");
				dev->rx_pkt_burst = eth_i350_recv_scattered_pkts;
				dev->data->scattered_rx = 1;
			}
		} else {
			/*
			 * Use BSIZE field of the device RCTL register.
			 */
			if ((rctl_bsize == 0) || (rctl_bsize > buf_size))
				rctl_bsize = buf_size;
			if (!dev->data->scattered_rx)
				PMD_INIT_LOG(DEBUG, "forcing scatter mode");
			dev->rx_pkt_burst = eth_i350_recv_scattered_pkts;
			dev->data->scattered_rx = 1;
		}

		/* Set if packets are dropped when no descriptors available */
		if (rxq->drop_en)
			srrctl |= I350_SRRCTL_DROP_EN;

		I350_WRITE_REG(hw, I350_SRRCTL(i), srrctl);

		/* Enable this RX queue. */
		rxdctl = I350_READ_REG(hw, I350_RXDCTL(i));
		rxdctl |= I350_RXDCTL_QUEUE_ENABLE;
		rxdctl &= 0xFFF00000;
		rxdctl |= (rxq->pthresh & 0x1F);
		rxdctl |= ((rxq->hthresh & 0x1F) << 8);
		if (hw->mac.type == i350_vfadapt) {
			/*
			 * Workaround of 82576 VF Erratum
			 * force set WTHRESH to 1
			 * to avoid Write-Back not triggered sometimes
			 */
			rxdctl |= 0x10000;
			PMD_INIT_LOG(DEBUG, "Force set RX WTHRESH to 1 !");
		}
		else
			rxdctl |= ((rxq->wthresh & 0x1F) << 16);
		I350_WRITE_REG(hw, I350_RXDCTL(i), rxdctl);

		dev->data->rx_queue_state[i] = RTE_ETH_QUEUE_STATE_STARTED;
	}

	if (dev->data->dev_conf.rxmode.offloads & RTE_ETH_RX_OFFLOAD_SCATTER) {
		if (!dev->data->scattered_rx)
			PMD_INIT_LOG(DEBUG, "forcing scatter mode");
		dev->rx_pkt_burst = eth_i350_recv_scattered_pkts;
		dev->data->scattered_rx = 1;
	}

	/*
	 * Setup the HW Rx Head and Tail Descriptor Pointers.
	 * This needs to be done after enable.
	 */
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		rxq = dev->data->rx_queues[i];
		I350_WRITE_REG(hw, I350_RDH(i), 0);
		I350_WRITE_REG(hw, I350_RDT(i), rxq->nb_rx_desc - 1);
	}

	return 0;
}

/*********************************************************************
 *
 *  Enable VF transmit unit.
 *
 **********************************************************************/
void
eth_i350vf_tx_init(struct rte_eth_dev *dev)
{
	struct i350_hw     *hw;
	struct i350_tx_queue *txq;
	uint32_t txdctl;
	uint16_t i;

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Setup the Base and Length of the Tx Descriptor Rings. */
	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		uint64_t bus_addr;

		txq = dev->data->tx_queues[i];
		bus_addr = txq->tx_ring_phys_addr;
		I350_WRITE_REG(hw, I350_TDLEN(i),
				txq->nb_tx_desc *
				sizeof(union i350_adv_tx_desc));
		I350_WRITE_REG(hw, I350_TDBAH(i),
				(uint32_t)(bus_addr >> 32));
		I350_WRITE_REG(hw, I350_TDBAL(i), (uint32_t)bus_addr);

		/* Setup the HW Tx Head and Tail descriptor pointers. */
		I350_WRITE_REG(hw, I350_TDT(i), 0);
		I350_WRITE_REG(hw, I350_TDH(i), 0);

		/* Setup Transmit threshold registers. */
		txdctl = I350_READ_REG(hw, I350_TXDCTL(i));
		txdctl |= txq->pthresh & 0x1F;
		txdctl |= ((txq->hthresh & 0x1F) << 8);
		if (hw->mac.type == i350_82576) {
			/*
			 * Workaround of 82576 VF Erratum
			 * force set WTHRESH to 1
			 * to avoid Write-Back not triggered sometimes
			 */
			txdctl |= 0x10000;
			PMD_INIT_LOG(DEBUG, "Force set TX WTHRESH to 1 !");
		}
		else
			txdctl |= ((txq->wthresh & 0x1F) << 16);
		txdctl |= I350_TXDCTL_QUEUE_ENABLE;
		I350_WRITE_REG(hw, I350_TXDCTL(i), txdctl);

		dev->data->tx_queue_state[i] = RTE_ETH_QUEUE_STATE_STARTED;
	}

}

void
i350_rxq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_rxq_info *qinfo)
{
	struct i350_rx_queue *rxq;

	rxq = dev->data->rx_queues[queue_id];

	qinfo->mp = rxq->mb_pool;
	qinfo->scattered_rx = dev->data->scattered_rx;
	qinfo->nb_desc = rxq->nb_rx_desc;

	qinfo->conf.rx_free_thresh = rxq->rx_free_thresh;
	qinfo->conf.rx_drop_en = rxq->drop_en;
	qinfo->conf.offloads = rxq->offloads;
}

void
i350_txq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_txq_info *qinfo)
{
	struct i350_tx_queue *txq;

	txq = dev->data->tx_queues[queue_id];

	qinfo->nb_desc = txq->nb_tx_desc;

	qinfo->conf.tx_thresh.pthresh = txq->pthresh;
	qinfo->conf.tx_thresh.hthresh = txq->hthresh;
	qinfo->conf.tx_thresh.wthresh = txq->wthresh;
	qinfo->conf.offloads = txq->offloads;
}

int
i350_rss_conf_init(struct rte_eth_dev *dev,
		  struct i350_rte_flow_rss_conf *out,
		  const struct rte_flow_action_rss *in)
{
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (in->key_len > RTE_DIM(out->key) ||
	    ((hw->mac.type == i350_82576) &&
	     (in->queue_num > I350_MAX_RX_QUEUE_NUM_82576)) ||
	    ((hw->mac.type != i350_82576) &&
	     (in->queue_num > I350_MAX_RX_QUEUE_NUM)))
		return -EINVAL;
	out->conf = (struct rte_flow_action_rss){
		.func = in->func,
		.level = in->level,
		.types = in->types,
		.key_len = in->key_len,
		.queue_num = in->queue_num,
		.key = memcpy(out->key, in->key, in->key_len),
		.queue = memcpy(out->queue, in->queue,
				sizeof(*in->queue) * in->queue_num),
	};
	return 0;
}

int
i350_action_rss_same(const struct rte_flow_action_rss *comp,
		    const struct rte_flow_action_rss *with)
{
	return (comp->func == with->func &&
		comp->level == with->level &&
		comp->types == with->types &&
		comp->key_len == with->key_len &&
		comp->queue_num == with->queue_num &&
		!memcmp(comp->key, with->key, with->key_len) &&
		!memcmp(comp->queue, with->queue,
			sizeof(*with->queue) * with->queue_num));
}

int
i350_config_rss_filter(struct rte_eth_dev *dev,
		struct i350_rte_flow_rss_conf *conf, bool add)
{
	uint32_t shift;
	uint16_t i, j;
	struct rte_eth_rss_conf rss_conf = {
		.rss_key = conf->conf.key_len ?
			(void *)(uintptr_t)conf->conf.key : NULL,
		.rss_key_len = conf->conf.key_len,
		.rss_hf = conf->conf.types,
	};
	struct i350_filter_info *filter_info =
		I350_DEV_PRIVATE_TO_FILTER_INFO(dev->data->dev_private);
	struct i350_hw *hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	hw = I350_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (!add) {
		if (i350_action_rss_same(&filter_info->rss_info.conf,
					&conf->conf)) {
			i350_rss_disable(dev);
			memset(&filter_info->rss_info, 0,
				sizeof(struct i350_rte_flow_rss_conf));
			return 0;
		}
		return -EINVAL;
	}

	if (filter_info->rss_info.conf.queue_num)
		return -EINVAL;

	/* Fill in redirection table. */
	shift = (hw->mac.type == i350_82575) ? 6 : 0;
	for (i = 0, j = 0; i < 128; i++, j++) {
		union i350_reta {
			uint32_t dword;
			uint8_t  bytes[4];
		} reta;
		uint8_t q_idx;

		if (j == conf->conf.queue_num)
			j = 0;
		q_idx = conf->conf.queue[j];
		reta.bytes[i & 3] = (uint8_t)(q_idx << shift);
		if ((i & 3) == 3)
			I350_WRITE_REG(hw, I350_RETA(i >> 2), reta.dword);
	}

	/* Configure the RSS key and the RSS protocols used to compute
	 * the RSS hash of input packets.
	 */
	if ((rss_conf.rss_hf & I350_RSS_OFFLOAD_ALL) == 0) {
		i350_rss_disable(dev);
		return 0;
	}
	if (rss_conf.rss_key == NULL)
		rss_conf.rss_key = rss_intel_key; /* Default hash key */
	i350_hw_rss_hash_set(hw, &rss_conf);

	if (i350_rss_conf_init(dev, &filter_info->rss_info, &conf->conf))
		return -EINVAL;

	return 0;
}
