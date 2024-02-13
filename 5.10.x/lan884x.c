// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/net/phy/micrel.c
 *
 * Driver for Micrel PHYs
 *
 * Author: David J. Choi
 *
 * Copyright (c) 2010-2013 Micrel, Inc.
 * Copyright (c) 2014 Johan Hovold <johan@kernel.org>
 *
 * Support : Micrel Phys:
 *		Giga phys: ksz9021, ksz9031, ksz9131
 *		100/10 Phys : ksz8001, ksz8721, ksz8737, ksz8041
 *			   ksz8021, ksz8031, ksz8051,
 *			   ksz8081, ksz8091,
 *			   ksz8061,
 *		Switch : ksz8873, ksz886x
 *			 ksz9477
 */

#include <linux/bitfield.h>
#include <linux/ethtool_netlink.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/micrel_phy.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/ptp_clock.h>
#include <linux/ptp_classify.h>
#include <linux/net_tstamp.h>

#define PHY_ID_LAN8841		0x00221650

/* LinkMD Control/Status */
#define KSZ8081_LMD_ENABLE_TEST			BIT(15)
#define KSZ8081_LMD_STAT_NORMAL			0
#define KSZ8081_LMD_STAT_OPEN			1
#define KSZ8081_LMD_STAT_SHORT			2
#define KSZ8081_LMD_STAT_FAIL			3

#define LAN8814_CABLE_DIAG			0x12
#define LAN8814_CABLE_DIAG_STAT_MASK		GENMASK(9, 8)
#define LAN8814_CABLE_DIAG_VCT_DATA_MASK	GENMASK(7, 0)
#define LAN8814_PAIR_BIT_SHIFT			12

#define LAN8814_WIRE_PAIR_MASK			0xF

/* Lan8814 general Interrupt control/status reg in GPHY specific block. */
#define LAN8814_INTC				0x18
#define LAN8814_INTC_LINK_DOWN			BIT(2)
#define LAN8814_INTC_LINK_UP			BIT(0)
#define LAN8814_INTC_LINK			(LAN8814_INTC_LINK_UP |\
						 LAN8814_INTC_LINK_DOWN)

#define LAN8814_INTS				0x1B
#define LAN8814_INTS_LINK_DOWN			BIT(2)
#define LAN8814_INTS_LINK_UP			BIT(0)
#define LAN8814_INTS_LINK			(LAN8814_INTS_LINK_UP |\
						 LAN8814_INTS_LINK_DOWN)

#define LAN8841_1PPM_FORMAT			34360

#define PTP_TIMESTAMP_EN_SYNC_			BIT(0)
#define PTP_TIMESTAMP_EN_DREQ_			BIT(1)
#define PTP_TIMESTAMP_EN_PDREQ_			BIT(2)
#define PTP_TIMESTAMP_EN_PDRES_			BIT(3)

#define PTP_TX_MOD_TX_PTP_SYNC_TS_INSERT_	BIT(12)

#define PTP_RX_PARSE_CONFIG_LAYER2_EN_		BIT(0)
#define PTP_RX_PARSE_CONFIG_IPV4_EN_		BIT(1)
#define PTP_RX_PARSE_CONFIG_IPV6_EN_		BIT(2)

#define PTP_TX_PARSE_CONFIG_LAYER2_EN_		BIT(0)
#define PTP_TX_PARSE_CONFIG_IPV4_EN_		BIT(1)
#define PTP_TX_PARSE_CONFIG_IPV6_EN_		BIT(2)

#define FIFO_SIZE				8

/* LAN7431 & LAN8841 custom drivers
 * Add macros and a function from newer kernel.h due to the back porting
 * this file from kernel 5.15 to 5.10
 */
/**
 * upper_16_bits - return bits 16-31 of a number
 * @n: the number we're accessing
 */
#define upper_16_bits(n) ((u16)((n) >> 16))

/**
 * lower_16_bits - return bits 0-15 of a number
 * @n: the number we're accessing
 */
#define lower_16_bits(n) ((u16)((n) & 0xffff))

/**
 * phy_trigger_machine - Trigger the state machine to run now
 *
 * @phydev: the phy_device struct
 */
void phy_trigger_machine(struct phy_device *phydev)
{
	phy_queue_state_machine(phydev, 0);
}

struct kszphy_hw_stat {
	const char *string;
	u8 reg;
	u8 bits;
};

static struct kszphy_hw_stat kszphy_hw_stats[] = {
	{ "phy_receive_errors", 21, 16},
	{ "phy_idle_errors", 10, 8 },
};

struct kszphy_type {
	u32 led_mode_reg;
	u16 disable_dll_rx_bit;
	u16 disable_dll_tx_bit;
	u16 disable_dll_mask;
	u16 interrupt_level_mask;
	u16 cable_diag_reg;
	unsigned long pair_mask;
	bool has_broadcast_disable;
	bool has_nand_tree_disable;
	bool has_rmii_ref_clk_sel;
};

struct lan8814_ptp_rx_ts {
	struct list_head list;
	u32 seconds;
	u32 nsec;
	u16 seq_id;
};

struct kszphy_ptp_priv {
	struct mii_timestamper mii_ts;
	struct phy_device *phydev;

	struct sk_buff_head tx_queue;
	struct sk_buff_head rx_queue;

	struct list_head rx_ts_list;
	/* Lock for Rx ts fifo */
	spinlock_t rx_ts_lock;

	int hwts_tx_type;
	enum hwtstamp_rx_filters rx_filter;
	int layer;
	int version;

	struct ptp_clock *ptp_clock;
	struct ptp_clock_info ptp_clock_info;
	struct mutex ptp_lock;
	struct ptp_pin_desc *pin_config;
	/* Contains the pin on which event is active. If the event is not active
	 * then contains negative value
	 */
	s8 event_a_pin;
	s8 event_b_pin;
};

struct kszphy_priv {
	struct kszphy_ptp_priv ptp_priv;
	const struct kszphy_type *type;
	int led_mode;
	bool rmii_ref_clk_sel;
	bool rmii_ref_clk_sel_val;
	u64 stats[ARRAY_SIZE(kszphy_hw_stats)];

	int rev;
};

static const struct kszphy_type lan8841_type = {
	.disable_dll_tx_bit	= BIT(14),
	.disable_dll_rx_bit	= BIT(14),
	.disable_dll_mask	= BIT_MASK(14),
	.cable_diag_reg		= LAN8814_CABLE_DIAG,
	.pair_mask		= LAN8814_WIRE_PAIR_MASK,
};

#define MII_KSZ9031RN_RX_DATA_PAD_SKEW	5
#define MII_KSZ9031RN_RXD3		GENMASK(15, 12)
#define MII_KSZ9031RN_RXD2		GENMASK(11, 8)
#define MII_KSZ9031RN_RXD1		GENMASK(7, 4)
#define MII_KSZ9031RN_RXD0		GENMASK(3, 0)

#define MII_KSZ9031RN_TX_DATA_PAD_SKEW	6
#define MII_KSZ9031RN_TXD3		GENMASK(15, 12)
#define MII_KSZ9031RN_TXD2		GENMASK(11, 8)
#define MII_KSZ9031RN_TXD1		GENMASK(7, 4)
#define MII_KSZ9031RN_TXD0		GENMASK(3, 0)

#define MII_KSZ9031RN_CLK_PAD_SKEW	8
#define MII_KSZ9031RN_GTX_CLK		GENMASK(9, 5)
#define MII_KSZ9031RN_RX_CLK		GENMASK(4, 0)

#define KSZ9131_SKEW_5BIT_MAX	2400
#define KSZ9131_SKEW_4BIT_MAX	800
#define KSZ9131_OFFSET		700
#define KSZ9131_STEP		100

static int ksz9131_of_load_skew_values(struct phy_device *phydev,
				       struct device_node *of_node,
				       u16 reg, size_t field_sz,
				       char *field[], u8 numfields)
{
	int val[4] = {-(1 + KSZ9131_OFFSET), -(2 + KSZ9131_OFFSET),
		      -(3 + KSZ9131_OFFSET), -(4 + KSZ9131_OFFSET)};
	int skewval, skewmax = 0;
	int matches = 0;
	u16 maxval;
	u16 newval;
	u16 mask;
	int i;

	/* psec properties in dts should mean x pico seconds */
	if (field_sz == 5)
		skewmax = KSZ9131_SKEW_5BIT_MAX;
	else
		skewmax = KSZ9131_SKEW_4BIT_MAX;

	for (i = 0; i < numfields; i++)
		if (!of_property_read_s32(of_node, field[i], &skewval)) {
			if (skewval < -KSZ9131_OFFSET)
				skewval = -KSZ9131_OFFSET;
			else if (skewval > skewmax)
				skewval = skewmax;

			val[i] = skewval + KSZ9131_OFFSET;
			matches++;
		}

	if (!matches)
		return 0;

	newval = phy_read_mmd(phydev, 2, reg);

	maxval = (field_sz == 4) ? 0xf : 0x1f;
	for (i = 0; i < numfields; i++)
		if (val[i] != -(i + 1 + KSZ9131_OFFSET)) {
			mask = 0xffff;
			mask ^= maxval << (field_sz * i);
			newval = (newval & mask) |
				(((val[i] / KSZ9131_STEP) & maxval)
					<< (field_sz * i));
		}

	return phy_write_mmd(phydev, 2, reg, newval);
}

#define KSZ9131RN_MMD_COMMON_CTRL_REG	2
#define KSZ9131RN_RXC_DLL_CTRL		76
#define KSZ9131RN_TXC_DLL_CTRL		77
#define KSZ9131RN_DLL_ENABLE_DELAY	0

static int ksz9131_config_rgmii_delay(struct phy_device *phydev)
{
	const struct kszphy_type *type = phydev->drv->driver_data;
	u16 rxcdll_val, txcdll_val;
	int ret;

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_RGMII:
		rxcdll_val = type->disable_dll_rx_bit;
		txcdll_val = type->disable_dll_tx_bit;
		break;
	case PHY_INTERFACE_MODE_RGMII_ID:
		rxcdll_val = KSZ9131RN_DLL_ENABLE_DELAY;
		txcdll_val = KSZ9131RN_DLL_ENABLE_DELAY;
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		rxcdll_val = KSZ9131RN_DLL_ENABLE_DELAY;
		txcdll_val = type->disable_dll_tx_bit;
		break;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		rxcdll_val = type->disable_dll_rx_bit;
		txcdll_val = KSZ9131RN_DLL_ENABLE_DELAY;
		break;
	default:
		return 0;
	}

	ret = phy_modify_mmd(phydev, KSZ9131RN_MMD_COMMON_CTRL_REG,
			     KSZ9131RN_RXC_DLL_CTRL,
			     type->disable_dll_mask,
			     rxcdll_val);
	if (ret < 0)
		return ret;

	return phy_modify_mmd(phydev, KSZ9131RN_MMD_COMMON_CTRL_REG,
			      KSZ9131RN_TXC_DLL_CTRL,
			      type->disable_dll_mask,
			      txcdll_val);
}

static int kszphy_get_sset_count(struct phy_device *phydev)
{
	return ARRAY_SIZE(kszphy_hw_stats);
}

static void kszphy_get_strings(struct phy_device *phydev, u8 *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(kszphy_hw_stats); i++) {
		strlcpy(data + i * ETH_GSTRING_LEN,
			kszphy_hw_stats[i].string, ETH_GSTRING_LEN);
	}
}

static u64 kszphy_get_stat(struct phy_device *phydev, int i)
{
	struct kszphy_hw_stat stat = kszphy_hw_stats[i];
	struct kszphy_priv *priv = phydev->priv;
	int val;
	u64 ret;

	val = phy_read(phydev, stat.reg);
	if (val < 0) {
		ret = U64_MAX;
	} else {
		val = val & ((1 << stat.bits) - 1);
		priv->stats[i] += val;
		ret = priv->stats[i];
	}

	return ret;
}

static void kszphy_get_stats(struct phy_device *phydev,
			     struct ethtool_stats *stats, u64 *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(kszphy_hw_stats); i++)
		data[i] = kszphy_get_stat(phydev, i);
}

static void kszphy_parse_led_mode(struct phy_device *phydev)
{
	const struct kszphy_type *type = phydev->drv->driver_data;
	const struct device_node *np = phydev->mdio.dev.of_node;
	struct kszphy_priv *priv = phydev->priv;
	int ret;

	if (type && type->led_mode_reg) {
		ret = of_property_read_u32(np, "micrel,led-mode",
					   &priv->led_mode);

		if (ret)
			priv->led_mode = -1;

		if (priv->led_mode > 3) {
			phydev_err(phydev, "invalid led mode: 0x%02x\n",
				   priv->led_mode);
			priv->led_mode = -1;
		}
	} else {
		priv->led_mode = -1;
	}
}

static int kszphy_probe(struct phy_device *phydev)
{
	const struct kszphy_type *type = phydev->drv->driver_data;
	const struct device_node *np = phydev->mdio.dev.of_node;
	struct kszphy_priv *priv;
	struct clk *clk;

	priv = devm_kzalloc(&phydev->mdio.dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;

	priv->type = type;

	kszphy_parse_led_mode(phydev);

	clk = devm_clk_get(&phydev->mdio.dev, "rmii-ref");
	/* NOTE: clk may be NULL if building without CONFIG_HAVE_CLK */
	if (!IS_ERR_OR_NULL(clk)) {
		unsigned long rate = clk_get_rate(clk);
		bool rmii_ref_clk_sel_25_mhz;

		if (type)
			priv->rmii_ref_clk_sel = type->has_rmii_ref_clk_sel;
		rmii_ref_clk_sel_25_mhz = of_property_read_bool(np,
				"micrel,rmii-reference-clock-select-25-mhz");

		if (rate > 24500000 && rate < 25500000) {
			priv->rmii_ref_clk_sel_val = rmii_ref_clk_sel_25_mhz;
		} else if (rate > 49500000 && rate < 50500000) {
			priv->rmii_ref_clk_sel_val = !rmii_ref_clk_sel_25_mhz;
		} else {
			phydev_err(phydev, "Clock rate out of range: %ld\n",
				   rate);
			return -EINVAL;
		}
	}

	/* Support legacy board-file configuration */
	if (phydev->dev_flags & MICREL_PHY_50MHZ_CLK) {
		priv->rmii_ref_clk_sel = true;
		priv->rmii_ref_clk_sel_val = true;
	}

	return 0;
}

static int lan8814_cable_test_start(struct phy_device *phydev)
{
	/* If autoneg is enabled, we won't be able to test cross pair
	 * short. In this case, the PHY will "detect" a link and
	 * confuse the internal state machine - disable auto neg here.
	 * Set the speed to 1000mbit and full duplex.
	 */
	return phy_modify(phydev, MII_BMCR, BMCR_ANENABLE | BMCR_SPEED100,
			  BMCR_SPEED1000 | BMCR_FULLDPLX);
}

static int ksz886x_cable_test_result_trans(u16 status, u16 mask)
{
	switch (FIELD_GET(mask, status)) {
	case KSZ8081_LMD_STAT_NORMAL:
		return ETHTOOL_A_CABLE_RESULT_CODE_OK;
	case KSZ8081_LMD_STAT_SHORT:
		return ETHTOOL_A_CABLE_RESULT_CODE_SAME_SHORT;
	case KSZ8081_LMD_STAT_OPEN:
		return ETHTOOL_A_CABLE_RESULT_CODE_OPEN;
	case KSZ8081_LMD_STAT_FAIL:
		fallthrough;
	default:
		return ETHTOOL_A_CABLE_RESULT_CODE_UNSPEC;
	}
}

static bool ksz886x_cable_test_failed(u16 status, u16 mask)
{
	return FIELD_GET(mask, status) ==
		KSZ8081_LMD_STAT_FAIL;
}

static bool ksz886x_cable_test_fault_length_valid(u16 status, u16 mask)
{
	switch (FIELD_GET(mask, status)) {
	case KSZ8081_LMD_STAT_OPEN:
		fallthrough;
	case KSZ8081_LMD_STAT_SHORT:
		return true;
	}
	return false;
}

static int ksz886x_cable_test_fault_length(struct phy_device *phydev, u16 status, u16 data_mask)
{
	int dt;

	/* According to the data sheet the distance to the fault is
	 * DELTA_TIME * 0.4 meters for ksz phys.
	 * (DELTA_TIME - 22) * 0.8 for lan8814 phy.
	 */
	dt = FIELD_GET(data_mask, status);

	if ((phydev->phy_id & MICREL_PHY_ID_MASK) == PHY_ID_LAN8814)
		return ((dt - 22) * 800) / 10;
	else
		return (dt * 400) / 10;
}

static int ksz886x_cable_test_wait_for_completion(struct phy_device *phydev)
{
	const struct kszphy_type *type = phydev->drv->driver_data;
	int val, ret;

	ret = phy_read_poll_timeout(phydev, type->cable_diag_reg, val,
				    !(val & KSZ8081_LMD_ENABLE_TEST),
				    30000, 100000, true);

	return ret < 0 ? ret : 0;
}

static int lan8814_cable_test_one_pair(struct phy_device *phydev, int pair)
{
	static const int ethtool_pair[] = { ETHTOOL_A_CABLE_PAIR_A,
					    ETHTOOL_A_CABLE_PAIR_B,
					    ETHTOOL_A_CABLE_PAIR_C,
					    ETHTOOL_A_CABLE_PAIR_D,
					  };
	u32 fault_length;
	int ret;
	int val;

	val = KSZ8081_LMD_ENABLE_TEST;
	val = val | (pair << LAN8814_PAIR_BIT_SHIFT);

	ret = phy_write(phydev, LAN8814_CABLE_DIAG, val);
	if (ret < 0)
		return ret;

	ret = ksz886x_cable_test_wait_for_completion(phydev);
	if (ret)
		return ret;

	val = phy_read(phydev, LAN8814_CABLE_DIAG);
	if (val < 0)
		return val;

	if (ksz886x_cable_test_failed(val, LAN8814_CABLE_DIAG_STAT_MASK))
		return -EAGAIN;

	ret = ethnl_cable_test_result(phydev, ethtool_pair[pair],
				      ksz886x_cable_test_result_trans(val,
								      LAN8814_CABLE_DIAG_STAT_MASK
								      ));
	if (ret)
		return ret;

	if (!ksz886x_cable_test_fault_length_valid(val, LAN8814_CABLE_DIAG_STAT_MASK))
		return 0;

	fault_length = ksz886x_cable_test_fault_length(phydev, val,
						       LAN8814_CABLE_DIAG_VCT_DATA_MASK);

	return ethnl_cable_test_fault_length(phydev, ethtool_pair[pair], fault_length);
}


static int ksz886x_cable_test_get_status(struct phy_device *phydev,
					 bool *finished)
{
	const struct kszphy_type *type = phydev->drv->driver_data;
	unsigned long pair_mask = type->pair_mask;
	int retries = 20;
	int pair, ret;

	*finished = false;

	/* Try harder if link partner is active */
	while (pair_mask && retries--) {
		for_each_set_bit(pair, &pair_mask, 4) {
			if (type->cable_diag_reg == LAN8814_CABLE_DIAG)
				ret = lan8814_cable_test_one_pair(phydev, pair);
			if (ret == -EAGAIN)
				continue;
			if (ret < 0)
				return ret;
			clear_bit(pair, &pair_mask);
		}
		/* If link partner is in autonegotiation mode it will send 2ms
		 * of FLPs with at least 6ms of silence.
		 * Add 2ms sleep to have better chances to hit this silence.
		 */
		if (pair_mask)
			msleep(2);
	}

	*finished = true;

	return ret;
}

#define LAN8814_DCQ_CTRL				0xe6
#define LAN8814_DCQ_CTRL_READ_CAPTURE_			BIT(15)
#define LAN8814_DCQ_CTRL_CHANNEL_MASK			GENMASK(1, 0)
#define LAN8814_DCQ_SQI					0xe4
#define LAN8814_DCQ_SQI_MAX				7
#define LAN8814_DCQ_SQI_VAL_MASK			GENMASK(3, 1)

static void lan88xx_ts_info(struct ethtool_ts_info *info)
{
	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;

	info->tx_types =
		(1 << HWTSTAMP_TX_OFF) |
		(1 << HWTSTAMP_TX_ON) |
		(1 << HWTSTAMP_TX_ONESTEP_SYNC);

	info->rx_filters =
		(1 << HWTSTAMP_FILTER_NONE) |
		(1 << HWTSTAMP_FILTER_PTP_V1_L4_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L2_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_EVENT);
}

static bool is_sync(struct sk_buff *skb, int type)
{
	struct ptp_header *hdr;

	hdr = ptp_parse_header(skb, type);
	if (!hdr)
		return false;

	return ((ptp_get_msgtype(hdr, type) & 0xf) == 0);
}

static void lan8814_txtstamp(struct mii_timestamper *mii_ts,
			     struct sk_buff *skb, int type)
{
	struct kszphy_ptp_priv *ptp_priv = container_of(mii_ts, struct kszphy_ptp_priv, mii_ts);

	switch (ptp_priv->hwts_tx_type) {
	case HWTSTAMP_TX_ONESTEP_SYNC:
		if (is_sync(skb, type)) {
			kfree_skb(skb);
			return;
		}
		fallthrough;
	case HWTSTAMP_TX_ON:
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		skb_queue_tail(&ptp_priv->tx_queue, skb);
		break;
	case HWTSTAMP_TX_OFF:
	default:
		kfree_skb(skb);
		break;
	}
}

static void lan8814_get_sig_rx(struct sk_buff *skb, u16 *sig)
{
	struct ptp_header *ptp_header;
	u32 type;

	skb_push(skb, ETH_HLEN);
	type = ptp_classify_raw(skb);
	ptp_header = ptp_parse_header(skb, type);
	skb_pull_inline(skb, ETH_HLEN);

	*sig = ntohs(ptp_header->sequence_id);
}

static bool lan8814_match_rx_skb(struct kszphy_ptp_priv *ptp_priv,
				 struct sk_buff *skb)
{
	struct skb_shared_hwtstamps *shhwtstamps;
	struct lan8814_ptp_rx_ts *rx_ts, *tmp;
	unsigned long flags;
	bool ret = false;
	u16 skb_sig;

	lan8814_get_sig_rx(skb, &skb_sig);

	/* Iterate over all RX timestamps and match it with the received skbs */
	spin_lock_irqsave(&ptp_priv->rx_ts_lock, flags);
	list_for_each_entry_safe(rx_ts, tmp, &ptp_priv->rx_ts_list, list) {
		/* Check if we found the signature we were looking for. */
		if (memcmp(&skb_sig, &rx_ts->seq_id, sizeof(rx_ts->seq_id)))
			continue;

		shhwtstamps = skb_hwtstamps(skb);
		memset(shhwtstamps, 0, sizeof(*shhwtstamps));
		shhwtstamps->hwtstamp = ktime_set(rx_ts->seconds,
						  rx_ts->nsec);
		netif_rx_ni(skb);

		list_del(&rx_ts->list);
		kfree(rx_ts);

		ret = true;
		break;
	}
	spin_unlock_irqrestore(&ptp_priv->rx_ts_lock, flags);

	return ret;
}

static bool lan8814_rxtstamp(struct mii_timestamper *mii_ts, struct sk_buff *skb, int type)
{
	struct kszphy_ptp_priv *ptp_priv = container_of(mii_ts, struct kszphy_ptp_priv, mii_ts);

	if (ptp_priv->rx_filter == HWTSTAMP_FILTER_NONE ||
	    type == PTP_CLASS_NONE)
		return false;

	if ((type & ptp_priv->version) == 0 || (type & ptp_priv->layer) == 0)
		return false;

	/* If we failed to match then add it to the queue for when the timestamp
	 * will come
	 */
	if (!lan8814_match_rx_skb(ptp_priv, skb))
		skb_queue_tail(&ptp_priv->rx_queue, skb);

	return true;
}

#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_200MS_	13
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_100MS_	12
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_50MS_	11
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_10MS_	10
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_5MS_	9
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_1MS_	8
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_500US_	7
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_100US_	6
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_50US_	5
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_10US_	4
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_5US_	3
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_1US_	2
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_500NS_	1
#define LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_100NS_	0
static int lan88xx_get_pulsewidth(struct phy_device *phydev,
				  struct ptp_perout_request *perout_request,
				  int *pulse_width)
{
	struct timespec64 ts_period;
	s64 ts_on_nsec, period_nsec;
	struct timespec64 ts_on;

	ts_period.tv_sec = perout_request->period.sec;
	ts_period.tv_nsec = perout_request->period.nsec;

	ts_on.tv_sec = perout_request->on.sec;
	ts_on.tv_nsec = perout_request->on.nsec;
	ts_on_nsec = timespec64_to_ns(&ts_on);
	period_nsec = timespec64_to_ns(&ts_period);

	if (period_nsec < 200) {
		phydev_warn(phydev, "perout period too small, minimum is 200ns\n");
		return -EOPNOTSUPP;
	}

	if (ts_on_nsec >= period_nsec) {
		phydev_warn(phydev, "pulse width must be smaller than period\n");
		return -EINVAL;
	}

	switch (ts_on_nsec) {
	case 200000000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_200MS_;
		break;
	case 100000000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_100MS_;
		break;
	case 50000000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_50MS_;
		break;
	case 10000000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_10MS_;
		break;
	case 5000000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_5MS_;
		break;
	case 1000000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_1MS_;
		break;
	case 500000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_500US_;
		break;
	case 100000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_100US_;
		break;
	case 50000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_50US_;
		break;
	case 10000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_10US_;
		break;
	case 5000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_5US_;
		break;
	case 1000:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_1US_;
		break;
	case 500:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_500NS_;
		break;
	case 100:
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_100NS_;
		break;
	default:
		phydev_warn(phydev, "Using default pulse width of 100ns\n");
		*pulse_width = LAN88XX_PTP_GENERAL_CONFIG_LTC_EVENT_100NS_;
		break;
	}
	return 0;
}

static void lan8814_get_sig_tx(struct sk_buff *skb, u16 *sig)
{
	struct ptp_header *ptp_header;
	u32 type;

	type = ptp_classify_raw(skb);
	ptp_header = ptp_parse_header(skb, type);

	*sig = htons(ptp_header->sequence_id);
}

static void lan8814_match_tx_skb(struct kszphy_ptp_priv *ptp_priv,
				 u32 seconds, u32 nsec, u16 seq_id)
{
	struct skb_shared_hwtstamps shhwtstamps;
	struct sk_buff *skb, *skb_tmp;
	unsigned long flags;
	bool ret = false;
	u16 skb_sig;

	spin_lock_irqsave(&ptp_priv->tx_queue.lock, flags);
	skb_queue_walk_safe(&ptp_priv->tx_queue, skb, skb_tmp) {
		lan8814_get_sig_tx(skb, &skb_sig);

		if (memcmp(&skb_sig, &seq_id, sizeof(seq_id)))
			continue;

		__skb_unlink(skb, &ptp_priv->tx_queue);
		ret = true;
		break;
	}
	spin_unlock_irqrestore(&ptp_priv->tx_queue.lock, flags);

	if (ret) {
		memset(&shhwtstamps, 0, sizeof(shhwtstamps));
		shhwtstamps.hwtstamp = ktime_set(seconds, nsec);
		skb_complete_tx_timestamp(skb, &shhwtstamps);
	}
}

static bool lan8814_match_skb(struct kszphy_ptp_priv *ptp_priv,
			      struct lan8814_ptp_rx_ts *rx_ts)
{
	struct skb_shared_hwtstamps *shhwtstamps;
	struct sk_buff *skb, *skb_tmp;
	unsigned long flags;
	bool ret = false;
	u16 skb_sig;

	spin_lock_irqsave(&ptp_priv->rx_queue.lock, flags);
	skb_queue_walk_safe(&ptp_priv->rx_queue, skb, skb_tmp) {
		lan8814_get_sig_rx(skb, &skb_sig);

		if (memcmp(&skb_sig, &rx_ts->seq_id, sizeof(rx_ts->seq_id)))
			continue;

		__skb_unlink(skb, &ptp_priv->rx_queue);

		ret = true;
		break;
	}
	spin_unlock_irqrestore(&ptp_priv->rx_queue.lock, flags);

	if (ret) {
		shhwtstamps = skb_hwtstamps(skb);
		memset(shhwtstamps, 0, sizeof(*shhwtstamps));
		shhwtstamps->hwtstamp = ktime_set(rx_ts->seconds, rx_ts->nsec);
		netif_rx_ni(skb);
	}

	return ret;
}

static void lan8814_match_rx_ts(struct kszphy_ptp_priv *ptp_priv,
				struct lan8814_ptp_rx_ts *rx_ts)
{
	unsigned long flags;

	/* If we failed to match the skb add it to the queue for when
	 * the frame will come
	 */
	if (!lan8814_match_skb(ptp_priv, rx_ts)) {
		spin_lock_irqsave(&ptp_priv->rx_ts_lock, flags);
		list_add(&rx_ts->list, &ptp_priv->rx_ts_list);
		spin_unlock_irqrestore(&ptp_priv->rx_ts_lock, flags);
	} else {
		kfree(rx_ts);
	}
}

static int lan8814_get_sqi_max(struct phy_device *phydev)
{
	return LAN8814_DCQ_SQI_MAX;
}

static int lan8841_get_sqi(struct phy_device *phydev)
{
	int rc, val;

	val = phy_read_mmd(phydev, 1, LAN8814_DCQ_CTRL);
	if (val < 0)
		return val;

	/* TODO 
	 * so far, only report the first pair as the uAPI takes one pair only.
	 */
//	val &= ~LAN8814_DCQ_CTRL_CHANNEL_MASK;
//	val |= LAN8814_DCQ_CTRL_READ_CAPTURE_;
	val = LAN8814_DCQ_CTRL_READ_CAPTURE_;
	rc = phy_write_mmd(phydev, 1, LAN8814_DCQ_CTRL, val);
	if (rc < 0)
		return rc;

	rc = phy_read_mmd(phydev, 1, LAN8814_DCQ_SQI);
	if (rc < 0)
		return rc;

	return FIELD_GET(LAN8814_DCQ_SQI_VAL_MASK, rc);
}

static void lan8841_ptp_update_target(struct kszphy_ptp_priv *ptp_priv,
				      const struct timespec64 *ts);
static void lan8841_gpio_process_cap(struct kszphy_ptp_priv *ptp_priv);

#define LAN8841_PTP_RX_PARSE_L2_ADDR_EN		370
#define LAN8841_PTP_RX_PARSE_IP_ADDR_EN		371
#define LAN8841_PTP_RX_VERSION			374
#define LAN8841_PTP_TX_PARSE_L2_ADDR_EN		434
#define LAN8841_PTP_TX_PARSE_IP_ADDR_EN		435
#define LAN8841_PTP_TX_VERSION			438
#define LAN8841_PTP_CMD_CTL			256
#define LAN8841_PTP_CMD_CTL_PTP_ENABLE		BIT(2)
#define LAN8841_PTP_CMD_CTL_PTP_DISABLE		BIT(1)
#define LAN8841_PTP_CMD_CTL_PTP_RESET		BIT(0)
#define LAN8841_PTP_RX_PARSE_CONFIG		368
#define LAN8841_PTP_TX_PARSE_CONFIG		432
#define LAN8841_ANALOG_CONTROL_1		1
#define LAN8841_ANALOG_CONTROL_10		13
#define LAN8841_ANALOG_CONTROL_11		14
#define LAN8841_TX_LOW_I_CH_C_POWER_MANAGMENT	69
#define LAN8841_BTRX_POWER_DOWN			70
#define LAN8841_MMD0_REGISTER_17		17
#define LAN8841_ADC_CHANNEL_MASK		198
#define LAN8841_CTRL_REG			0
#define LAN8841_CTRL_REG_CR_TRANS_DIS		BIT(14)
static int lan8841_config_init(struct phy_device *phydev)
{
	char *rx_data_skews[4] = {"rxd0-skew-psec", "rxd1-skew-psec",
				  "rxd2-skew-psec", "rxd3-skew-psec"};
	char *tx_data_skews[4] = {"txd0-skew-psec", "txd1-skew-psec",
				  "txd2-skew-psec", "txd3-skew-psec"};
	char *clk_skews[2] = {"rxc-skew-psec", "txc-skew-psec"};
	struct device_node *of_node;
	int ret;

	if (phy_interface_is_rgmii(phydev)) {
		ret = ksz9131_config_rgmii_delay(phydev);
		if (ret < 0)
			return ret;
	}

	of_node = phydev->mdio.dev.of_node;
	if (!of_node)
		goto hw_init;

	ret = ksz9131_of_load_skew_values(phydev, of_node,
					  MII_KSZ9031RN_CLK_PAD_SKEW, 5,
					  clk_skews, 2);
	if (ret < 0)
		return ret;

	ret = ksz9131_of_load_skew_values(phydev, of_node,
					  MII_KSZ9031RN_RX_DATA_PAD_SKEW, 4,
					  rx_data_skews, 4);
	if (ret < 0)
		return ret;

	ret = ksz9131_of_load_skew_values(phydev, of_node,
					  MII_KSZ9031RN_TX_DATA_PAD_SKEW, 4,
					  tx_data_skews, 4);
	if (ret < 0)
		return ret;

hw_init:
	/* Initialize the HW by resetting everything */
	phy_modify_mmd(phydev, 2, LAN8841_PTP_CMD_CTL,
		       LAN8841_PTP_CMD_CTL_PTP_RESET,
		       LAN8841_PTP_CMD_CTL_PTP_RESET);

	phy_modify_mmd(phydev, 2, LAN8841_PTP_CMD_CTL,
		       LAN8841_PTP_CMD_CTL_PTP_ENABLE,
		       LAN8841_PTP_CMD_CTL_PTP_ENABLE);

	/* Don't process any frames */
	phy_write_mmd(phydev, 2, LAN8841_PTP_RX_PARSE_CONFIG, 0);
	phy_write_mmd(phydev, 2, LAN8841_PTP_TX_PARSE_CONFIG, 0);
	phy_write_mmd(phydev, 2, LAN8841_PTP_TX_PARSE_L2_ADDR_EN, 0);
	phy_write_mmd(phydev, 2, LAN8841_PTP_RX_PARSE_L2_ADDR_EN, 0);
	phy_write_mmd(phydev, 2, LAN8841_PTP_TX_PARSE_IP_ADDR_EN, 0);
	phy_write_mmd(phydev, 2, LAN8841_PTP_RX_PARSE_IP_ADDR_EN, 0);

	/* Disable checking for minorVersionPTP field */
	phy_write_mmd(phydev, 2, LAN8841_PTP_RX_VERSION, 0xff00);
	phy_write_mmd(phydev, 2, LAN8841_PTP_TX_VERSION, 0xff00);

	/* 100BT Clause 40 improvenent errata */
	phy_write_mmd(phydev, 28, LAN8841_ANALOG_CONTROL_1, 0x40);
	phy_write_mmd(phydev, 28, LAN8841_ANALOG_CONTROL_10, 0x1);

	/* 10M/100M Ethernet Signal Tuning Errata for Shorted-Center Tap
	 * Magnetics
	 */
	ret = phy_read_mmd(phydev, 2, 0x2);
	if ((ret & BIT(14)) == BIT(14)) {
		phy_write_mmd(phydev, 28,
			      LAN8841_TX_LOW_I_CH_C_POWER_MANAGMENT, 0xbffc);
		phy_write_mmd(phydev, 28,
			      LAN8841_BTRX_POWER_DOWN, 0xaf);
	}

	/* LDO Adjustment errata */
	phy_write_mmd(phydev, 28, LAN8841_ANALOG_CONTROL_11, 0x1000);

	/* 100BT RGMII latency tuning errata */
	phy_write_mmd(phydev, 1, LAN8841_ADC_CHANNEL_MASK, 0x0);
	phy_write_mmd(phydev, 0, LAN8841_MMD0_REGISTER_17, 0xa);

	/* Enable Cr_trans_dis bit to avoid LTC register reset */
	phy_modify_mmd(phydev, KSZ9131RN_MMD_COMMON_CTRL_REG, LAN8841_CTRL_REG,
		       LAN8841_CTRL_REG_CR_TRANS_DIS,
		       LAN8841_CTRL_REG_CR_TRANS_DIS);

	return 0;
}

#define LAN8841_OUTPUT_CTRL			25
#define LAN8841_OUTPUT_CTRL_INT_BUFFER		BIT(14)
#define LAN8841_CTRL				31
#define LAN8841_CTRL_INTR_POLARITY		BIT(14)
#define LAN8841_INTC_PTP			BIT(9)
#define LAN8841_INTC_GPIO			BIT(8)
static int lan8841_config_intr(struct phy_device *phydev)
{
	struct irq_data *irq_data;
	int tmp = 0;

	irq_data = irq_get_irq_data(phydev->irq);
	if (!irq_data)
		return 0;

	if (irqd_get_trigger_type(irq_data) & IRQ_TYPE_LEVEL_HIGH) {
		/* Change polarity of the interrupt */
		phy_modify(phydev, LAN8841_OUTPUT_CTRL,
			   LAN8841_OUTPUT_CTRL_INT_BUFFER,
			   LAN8841_OUTPUT_CTRL_INT_BUFFER);
		phy_modify(phydev, LAN8841_CTRL,
			   LAN8841_CTRL_INTR_POLARITY,
			   LAN8841_CTRL_INTR_POLARITY);
	} else {
		/* It is enought to set INT buffer to open-drain because then
		 * the interrupt will be active low.
		 */
		phy_modify(phydev, LAN8841_OUTPUT_CTRL,
			   LAN8841_OUTPUT_CTRL_INT_BUFFER, 0);
	}

	/* Enable / disable interrupts. It is OK to enable PTP interrupt even if
	 * it PTP is not enabled. Because the underneath blocks will not enable
	 * the PTP so we will never get the PTP interrupt
	 */
	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		tmp = LAN8814_INTC_LINK | LAN8841_INTC_PTP | LAN8841_INTC_GPIO;

	return phy_write(phydev, LAN8814_INTC, tmp);
}

#define LAN8841_PTP_TX_EGRESS_SEC_LO			453
#define LAN8841_PTP_TX_EGRESS_SEC_HI			452
#define LAN8841_PTP_TX_EGRESS_NS_LO			451
#define LAN8841_PTP_TX_EGRESS_NS_HI			450
#define LAN8841_PTP_TX_EGRESS_NSEC_HI_VALID		BIT(15)
#define LAN8841_PTP_TX_MSG_HEADER2			455
static bool lan8841_ptp_get_tx_ts(struct kszphy_ptp_priv *ptp_priv,
				  u32 *sec, u32 *nsec, u16 *seq)
{
	struct phy_device *phydev = ptp_priv->phydev;

	*nsec = phy_read_mmd(phydev, 2, LAN8841_PTP_TX_EGRESS_NS_HI);
	if (!(*nsec & LAN8841_PTP_TX_EGRESS_NSEC_HI_VALID))
		return false;

	*nsec = ((*nsec & 0x3fff) << 16);
	*nsec = *nsec | phy_read_mmd(phydev, 2, LAN8841_PTP_TX_EGRESS_NS_LO);

	*sec = phy_read_mmd(phydev, 2, LAN8841_PTP_TX_EGRESS_SEC_HI);
	*sec = *sec << 16;
	*sec = *sec | phy_read_mmd(phydev, 2, LAN8841_PTP_TX_EGRESS_SEC_LO);

	*seq = phy_read_mmd(phydev, 2, LAN8841_PTP_TX_MSG_HEADER2);

	return true;
}

static void lan8841_ptp_process_tx_ts(struct kszphy_ptp_priv *ptp_priv)
{
	u32 sec, nsec;
	u16 seq;

	while (lan8841_ptp_get_tx_ts(ptp_priv, &sec, &nsec, &seq))
		lan8814_match_tx_skb(ptp_priv, sec, nsec, seq);
}

#define LAN8841_PTP_RX_INGRESS_SEC_LO			389
#define LAN8841_PTP_RX_INGRESS_SEC_HI			388
#define LAN8841_PTP_RX_INGRESS_NS_LO			387
#define LAN8841_PTP_RX_INGRESS_NS_HI			386
#define LAN8841_PTP_RX_INGRESS_NSEC_HI_VALID		BIT(15)
#define LAN8841_PTP_RX_MSG_HEADER2			391
static struct lan8814_ptp_rx_ts *lan8841_ptp_get_rx_ts(struct kszphy_ptp_priv *ptp_priv)
{
	struct phy_device *phydev = ptp_priv->phydev;
	struct lan8814_ptp_rx_ts *rx_ts;
	u32 sec, nsec;
	u16 seq;

	nsec = phy_read_mmd(phydev, 2, LAN8841_PTP_RX_INGRESS_NS_HI);
	if (!(nsec & LAN8841_PTP_RX_INGRESS_NSEC_HI_VALID))
		return NULL;

	nsec = ((nsec & 0x3fff) << 16);
	nsec = nsec | phy_read_mmd(phydev, 2, LAN8841_PTP_RX_INGRESS_NS_LO);

	sec = phy_read_mmd(phydev, 2, LAN8841_PTP_RX_INGRESS_SEC_HI);
	sec = sec << 16;
	sec = sec | phy_read_mmd(phydev, 2, LAN8841_PTP_RX_INGRESS_SEC_LO);

	seq = phy_read_mmd(phydev, 2, LAN8841_PTP_RX_MSG_HEADER2);

	rx_ts = kzalloc(sizeof(*rx_ts), GFP_KERNEL);
	if (!rx_ts)
		return NULL;

	rx_ts->seconds = sec;
	rx_ts->nsec = nsec;
	rx_ts->seq_id = seq;

	return rx_ts;
}

static void lan8841_ptp_process_rx_ts(struct kszphy_ptp_priv *ptp_priv)
{
	struct lan8814_ptp_rx_ts *rx_ts;

	while ((rx_ts = lan8841_ptp_get_rx_ts(ptp_priv)) != NULL)
		lan8814_match_rx_ts(ptp_priv, rx_ts);
}

#define LAN8841_PTP_INT_STS			259
#define LAN8841_PTP_INT_STS_PTP_TX_TS_OVRFL_INT	BIT(13)
#define LAN8841_PTP_INT_STS_PTP_TX_TS_INT	BIT(12)
#define LAN8841_PTP_INT_STS_PTP_RX_TS_OVRFL_INT	BIT(9)
#define LAN8841_PTP_INT_STS_PTP_RX_TS_INT	BIT(8)
#define LAN8841_PTP_INT_STS_PTP_GPIO_CAP_INT	BIT(2)
static void lan8841_ptp_flush_fifo(struct kszphy_ptp_priv *ptp_priv, bool egress)
{
	struct phy_device *phydev = ptp_priv->phydev;
	int i;

	for (i = 0; i < FIFO_SIZE; ++i)
		phy_read_mmd(phydev, 2,
			     egress ? LAN8841_PTP_TX_MSG_HEADER2 :
				      LAN8841_PTP_RX_MSG_HEADER2);

	phy_read_mmd(phydev, 2, LAN8841_PTP_INT_STS);
}

static void lan8841_handle_ptp_interrupt(struct phy_device *phydev)
{
	struct kszphy_priv *priv = phydev->priv;
	struct kszphy_ptp_priv *ptp_priv = &priv->ptp_priv;
	u16 status;

	do {
		status = phy_read_mmd(phydev, 2, LAN8841_PTP_INT_STS);
		if (status & LAN8841_PTP_INT_STS_PTP_TX_TS_INT)
			lan8841_ptp_process_tx_ts(ptp_priv);

		if (status & LAN8841_PTP_INT_STS_PTP_RX_TS_INT)
			lan8841_ptp_process_rx_ts(ptp_priv);

		if (status & LAN8841_PTP_INT_STS_PTP_TX_TS_OVRFL_INT) {
			lan8841_ptp_flush_fifo(ptp_priv, true);
			skb_queue_purge(&ptp_priv->tx_queue);
		}

		if (status & LAN8841_PTP_INT_STS_PTP_RX_TS_OVRFL_INT) {
			lan8841_ptp_flush_fifo(ptp_priv, false);
			skb_queue_purge(&ptp_priv->rx_queue);
		}

		if (status & LAN8841_PTP_INT_STS_PTP_GPIO_CAP_INT)
			lan8841_gpio_process_cap(ptp_priv);
	} while (status);
}

#define LAN8841_INTS_PTP		BIT(9)
#define LAN8841_INTS_GPIO		BIT(8)
static irqreturn_t lan8841_handle_interrupt(struct phy_device *phydev)
{
	int irq_status;

	irq_status = phy_read(phydev, LAN8814_INTS);
	if (irq_status < 0) {
		return IRQ_NONE;
	}

	if (irq_status & LAN8814_INTS_LINK)
		phy_trigger_machine(phydev);

	if (irq_status & LAN8841_INTS_PTP ||
	    irq_status & LAN8841_INTS_GPIO)
		lan8841_handle_ptp_interrupt(phydev);

	return IRQ_HANDLED;
}

static int lan8841_ts_info(struct mii_timestamper *mii_ts, struct ethtool_ts_info *info)
{
	struct kszphy_ptp_priv *ptp_priv = container_of(mii_ts, struct kszphy_ptp_priv, mii_ts);

	info->phc_index = ptp_priv->ptp_clock ? ptp_clock_index(ptp_priv->ptp_clock) : -1;
	if (info->phc_index == -1) {
		info->so_timestamping |= SOF_TIMESTAMPING_TX_SOFTWARE |
					 SOF_TIMESTAMPING_RX_SOFTWARE |
					 SOF_TIMESTAMPING_SOFTWARE;
		return 0;
	}

	lan88xx_ts_info(info);
	return 0;
}

#define LAN8841_PTP_INT_EN			260
#define LAN8841_PTP_INT_EN_PTP_TX_TS_OVRFL_EN	BIT(13)
#define LAN8841_PTP_INT_EN_PTP_TX_TS_EN		BIT(12)
#define LAN8841_PTP_INT_EN_PTP_RX_TS_OVRFL_EN	BIT(9)
#define LAN8841_PTP_INT_EN_PTP_RX_TS_EN		BIT(8)
static void lan8841_ptp_enable_int(struct kszphy_ptp_priv *ptp_priv,
				   bool enable)
{
	struct phy_device *phydev = ptp_priv->phydev;

	if (enable)
		/* Enable interrupts */
		phy_modify_mmd(phydev, 2, LAN8841_PTP_INT_EN,
			       LAN8841_PTP_INT_EN_PTP_TX_TS_OVRFL_EN |
			       LAN8841_PTP_INT_EN_PTP_RX_TS_OVRFL_EN |
			       LAN8841_PTP_INT_EN_PTP_TX_TS_EN |
			       LAN8841_PTP_INT_EN_PTP_RX_TS_EN,
			       LAN8841_PTP_INT_EN_PTP_TX_TS_OVRFL_EN |
			       LAN8841_PTP_INT_EN_PTP_RX_TS_OVRFL_EN |
			       LAN8841_PTP_INT_EN_PTP_TX_TS_EN |
			       LAN8841_PTP_INT_EN_PTP_RX_TS_EN);
	else
		/* Disable interrupts */
		phy_modify_mmd(phydev, 2, LAN8841_PTP_INT_EN,
			       LAN8841_PTP_INT_EN_PTP_TX_TS_OVRFL_EN |
			       LAN8841_PTP_INT_EN_PTP_RX_TS_OVRFL_EN |
			       LAN8841_PTP_INT_EN_PTP_TX_TS_EN |
			       LAN8841_PTP_INT_EN_PTP_RX_TS_EN, 0);
}

#define LAN8841_PTP_RX_TIMESTAMP_EN		379
#define LAN8841_PTP_TX_TIMESTAMP_EN		443
#define LAN8841_PTP_TX_MOD 			445
static int lan8841_hwtstamp(struct mii_timestamper *mii_ts, struct ifreq *ifr)
{
	struct kszphy_ptp_priv *ptp_priv = container_of(mii_ts, struct kszphy_ptp_priv, mii_ts);
	struct phy_device *phydev = ptp_priv->phydev;
	struct lan8814_ptp_rx_ts *rx_ts, *tmp;
	struct hwtstamp_config config;
	int txcfg = 0, rxcfg = 0;
	int pkt_ts_enable;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	ptp_priv->hwts_tx_type = config.tx_type;
	ptp_priv->rx_filter = config.rx_filter;

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		ptp_priv->layer = 0;
		ptp_priv->version = 0;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		ptp_priv->layer = PTP_CLASS_L4;
		ptp_priv->version = PTP_CLASS_V2;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		ptp_priv->layer = PTP_CLASS_L2;
		ptp_priv->version = PTP_CLASS_V2;
		break;
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		ptp_priv->layer = PTP_CLASS_L4 | PTP_CLASS_L2;
		ptp_priv->version = PTP_CLASS_V2;
		break;
	default:
		return -ERANGE;
	}

	/* Setup parsing of the frames and enable the timestamping for ptp
	 * frames
	 */
	if (ptp_priv->layer & PTP_CLASS_L2) {
		rxcfg = PTP_RX_PARSE_CONFIG_LAYER2_EN_;
		txcfg = PTP_TX_PARSE_CONFIG_LAYER2_EN_;
	} else if (ptp_priv->layer & PTP_CLASS_L4) {
		rxcfg |= PTP_RX_PARSE_CONFIG_IPV4_EN_ | PTP_RX_PARSE_CONFIG_IPV6_EN_;
		txcfg |= PTP_TX_PARSE_CONFIG_IPV4_EN_ | PTP_TX_PARSE_CONFIG_IPV6_EN_;
	}
	phy_write_mmd(phydev, 2, LAN8841_PTP_RX_PARSE_CONFIG, rxcfg);
	phy_write_mmd(phydev, 2, LAN8841_PTP_TX_PARSE_CONFIG, txcfg);

	pkt_ts_enable = PTP_TIMESTAMP_EN_SYNC_ | PTP_TIMESTAMP_EN_DREQ_ |
			PTP_TIMESTAMP_EN_PDREQ_ | PTP_TIMESTAMP_EN_PDRES_;
	phy_write_mmd(phydev, 2, LAN8841_PTP_RX_TIMESTAMP_EN, pkt_ts_enable);
	phy_write_mmd(phydev, 2, LAN8841_PTP_TX_TIMESTAMP_EN, pkt_ts_enable);

	/* Enable / disable of the TX timestamp in the SYNC frames */
	phy_modify_mmd(phydev, 2, LAN8841_PTP_TX_MOD,
		       PTP_TX_MOD_TX_PTP_SYNC_TS_INSERT_,
		       ptp_priv->hwts_tx_type == HWTSTAMP_TX_ONESTEP_SYNC ?
				PTP_TX_MOD_TX_PTP_SYNC_TS_INSERT_ : 0);

	/* Now enable the timestamping */
	lan8841_ptp_enable_int(ptp_priv,
			       config.rx_filter != HWTSTAMP_FILTER_NONE);

	/* In case of multiple starts and stops, these needs to be cleared */
	list_for_each_entry_safe(rx_ts, tmp, &ptp_priv->rx_ts_list, list) {
		list_del(&rx_ts->list);
		kfree(rx_ts);
	}
	skb_queue_purge(&ptp_priv->rx_queue);
	skb_queue_purge(&ptp_priv->tx_queue);

	lan8841_ptp_flush_fifo(ptp_priv, false);
	lan8841_ptp_flush_fifo(ptp_priv, true);

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ? -EFAULT : 0;
}

#define LAN8841_PTP_LTC_SET_SEC_HI	262
#define LAN8841_PTP_LTC_SET_SEC_MID	263
#define LAN8841_PTP_LTC_SET_SEC_LO	264
#define LAN8841_PTP_LTC_SET_NS_HI	265
#define LAN8841_PTP_LTC_SET_NS_LO	266
#define LAN8841_PTP_CMD_CTL_PTP_LTC_LOAD	BIT(4)
static int lan8841_ptp_settime64(struct ptp_clock_info *ptp,
				 const struct timespec64 *ts)
{
	struct kszphy_ptp_priv *ptp_priv = container_of(ptp, struct kszphy_ptp_priv,
							ptp_clock_info);
	struct phy_device *phydev = ptp_priv->phydev;

	/* Set the value to be stored */
	mutex_lock(&ptp_priv->ptp_lock);
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_SET_SEC_LO, lower_16_bits(ts->tv_sec));
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_SET_SEC_MID, upper_16_bits(ts->tv_sec));
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_SET_SEC_HI, upper_32_bits(ts->tv_sec) & 0xffff);
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_SET_NS_LO, lower_16_bits(ts->tv_nsec));
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_SET_NS_HI, upper_16_bits(ts->tv_nsec) & 0x3fff);

	/* Set the command to load the LTC */
	phy_write_mmd(phydev, 2, LAN8841_PTP_CMD_CTL,
		      LAN8841_PTP_CMD_CTL_PTP_LTC_LOAD);

	lan8841_ptp_update_target(ptp_priv, ts);
	mutex_unlock(&ptp_priv->ptp_lock);

	return 0;
}

#define LAN8841_PTP_LTC_RD_SEC_HI	358
#define LAN8841_PTP_LTC_RD_SEC_MID	359
#define LAN8841_PTP_LTC_RD_SEC_LO	360
#define LAN8841_PTP_LTC_RD_NS_HI	361
#define LAN8841_PTP_LTC_RD_NS_LO	362
#define LAN8841_PTP_CMD_CTL_PTP_LTC_READ	BIT(3)
static int lan8841_ptp_gettime64(struct ptp_clock_info *ptp,
				 struct timespec64 *ts)
{
	struct kszphy_ptp_priv *ptp_priv = container_of(ptp, struct kszphy_ptp_priv,
							ptp_clock_info);
	struct phy_device *phydev = ptp_priv->phydev;
	time64_t s;
	s64 ns;

	mutex_lock(&ptp_priv->ptp_lock);
	/* Issue the command to read the LTC */
	phy_write_mmd(phydev, 2, LAN8841_PTP_CMD_CTL,
		      LAN8841_PTP_CMD_CTL_PTP_LTC_READ);

	/* Read the LTC */
	s = phy_read_mmd(phydev, 2, LAN8841_PTP_LTC_RD_SEC_HI);
	s <<= 16;
	s |= phy_read_mmd(phydev, 2, LAN8841_PTP_LTC_RD_SEC_MID);
	s <<= 16;
	s |= phy_read_mmd(phydev, 2, LAN8841_PTP_LTC_RD_SEC_LO);

	ns = phy_read_mmd(phydev, 2, LAN8841_PTP_LTC_RD_NS_HI) & 0x3fff;
	ns <<= 16;
	ns |= phy_read_mmd(phydev, 2, LAN8841_PTP_LTC_RD_NS_LO);
	mutex_unlock(&ptp_priv->ptp_lock);

	set_normalized_timespec64(ts, s, ns);
	return 0;
}

#define LAN8841_PTP_LTC_STEP_ADJ_LO			276
#define LAN8841_PTP_LTC_STEP_ADJ_HI			275
#define LAN8841_PTP_LTC_STEP_ADJ_DIR			BIT(15)
#define LAN8841_PTP_CMD_CTL_PTP_LTC_STEP_SECONDS	BIT(5)
#define LAN8841_PTP_CMD_CTL_PTP_LTC_STEP_NANOSECONDS	BIT(6)
static int lan8841_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct kszphy_ptp_priv *ptp_priv = container_of(ptp, struct kszphy_ptp_priv,
							ptp_clock_info);
	struct phy_device *phydev = ptp_priv->phydev;
	struct timespec64 ts;
	bool add = true;
	u32 nsec;
	s32 sec;

	/* The HW allows up to 15 sec to adjust the time, but here we limit to
	 * 10 sec the adjustment. The reason is, in case the adjustment is 14
	 * sec and 999999999 nsec, then we add 8ns to compansate the actual
	 * increment so the value can be bigger than 15 sec. Therefore limit the
	 * possible adjustments so we will not have these corner cases
	 */
	if (delta > 10000000000LL || delta < -10000000000LL) {
		/* The timeadjustment is too big, so fall back using set time */
		u64 now;

		ptp->gettime64(ptp, &ts);

		now = ktime_to_ns(timespec64_to_ktime(ts));
		ts = ns_to_timespec64(now + delta);

		ptp->settime64(ptp, &ts);
		return 0;
	}

	sec = div_u64_rem(delta < 0 ? -delta : delta, NSEC_PER_SEC, &nsec);
	if (delta < 0 && nsec != 0) {
		/* It is not allowed to adjust low the nsec part, therefore
		 * substract more from second part and add to nanosecond such
		 * that would roll over, so the second part will increase
		 */
		sec--;
		nsec = NSEC_PER_SEC - nsec;
	}

	/* Calculate the adjustments and the direction */
	if (delta < 0)
		add = false;

	if (nsec > 0)
		/* add 8 ns to cover the likely normal increment */
		nsec += 8;

	if (nsec >= NSEC_PER_SEC) {
		/* carry into seconds */
		sec++;
		nsec -= NSEC_PER_SEC;
	}

	mutex_lock(&ptp_priv->ptp_lock);
	if (sec) {
		phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_STEP_ADJ_LO, sec);
		phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_STEP_ADJ_HI,
			      add ? LAN8841_PTP_LTC_STEP_ADJ_DIR : 0);
		phy_write_mmd(phydev, 2, LAN8841_PTP_CMD_CTL,
			      LAN8841_PTP_CMD_CTL_PTP_LTC_STEP_SECONDS);
	}

	if (nsec) {
		phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_STEP_ADJ_LO,
			      nsec & 0xffff);
		phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_STEP_ADJ_HI,
			      (nsec >> 16) & 0x3fff);
		phy_write_mmd(phydev, 2, LAN8841_PTP_CMD_CTL,
			      LAN8841_PTP_CMD_CTL_PTP_LTC_STEP_NANOSECONDS);
	}
	mutex_unlock(&ptp_priv->ptp_lock);

	/* Update the target clock */
	ptp->gettime64(ptp, &ts);
	mutex_lock(&ptp_priv->ptp_lock);
	lan8841_ptp_update_target(ptp_priv, &ts);
	mutex_unlock(&ptp_priv->ptp_lock);

	return 0;
}

#define LAN8841_PTP_LTC_RATE_ADJ_HI		269
#define LAN8841_PTP_LTC_RATE_ADJ_HI_DIR		BIT(15)
#define LAN8841_PTP_LTC_RATE_ADJ_LO		270
static int lan8841_ptp_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct kszphy_ptp_priv *ptp_priv = container_of(ptp, struct kszphy_ptp_priv,
							ptp_clock_info);
	struct phy_device *phydev = ptp_priv->phydev;
	bool faster = true;
	u32 rate;

	if (!scaled_ppm)
		return 0;

	if (scaled_ppm < 0) {
		scaled_ppm = -scaled_ppm;
		faster = false;
	}

	rate = LAN8841_1PPM_FORMAT * (upper_16_bits(scaled_ppm));
	rate += (LAN8841_1PPM_FORMAT * (lower_16_bits(scaled_ppm))) >> 16;

	mutex_lock(&ptp_priv->ptp_lock);
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_RATE_ADJ_HI,
		      faster ? LAN8841_PTP_LTC_RATE_ADJ_HI_DIR | (upper_16_bits(rate) & 0x3fff)
			     : upper_16_bits(rate) & 0x3fff);
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_RATE_ADJ_LO, lower_16_bits(rate));
	mutex_unlock(&ptp_priv->ptp_lock);

	return 0;
}

#define LAN8841_PTP_GPIO_NUM	10
#define LAN8841_PTP_GPIO_MASK   GENMASK(LAN8841_PTP_GPIO_NUM, 0)
static int lan8841_ptp_verify(struct ptp_clock_info *ptp, unsigned int pin,
			      enum ptp_pin_function func, unsigned int chan)
{
	struct kszphy_ptp_priv *ptp_priv = container_of(ptp, struct kszphy_ptp_priv,
							ptp_clock_info);

	/* Even if there are more then 2 pins, only 2 events can be active at
	 * the same time
	 */
	if (ptp_priv->event_a_pin >= 0 && ptp_priv->event_b_pin >= 0)
		return -1;

	/* It is not possible to have the same event on the same pin */
	if ((ptp_priv->event_a_pin == pin || ptp_priv->event_b_pin == pin) &&
	     func != PTP_PF_NONE)
		return -1;

	switch (func) {
	case PTP_PF_NONE:
	case PTP_PF_PEROUT:
	case PTP_PF_EXTTS:
		break;
	default:
		return -1;
	}

	return 0;
}

#define LAN8841_GPIO_EN		128
#define LAN8841_GPIO_DIR	129
#define LAN8841_GPIO_BUF	130
static void lan8841_ptp_perout_off(struct kszphy_ptp_priv *ptp_priv, int pin)
{
	struct phy_device *phydev = ptp_priv->phydev;
	u16 tmp;

	tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_EN) & LAN8841_PTP_GPIO_MASK;
	tmp &= ~BIT(pin);
	phy_write_mmd(phydev, 2, LAN8841_GPIO_EN, tmp);

	tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_DIR) & LAN8841_PTP_GPIO_MASK;
	tmp &= ~BIT(pin);
	phy_write_mmd(phydev, 2, LAN8841_GPIO_DIR, tmp);

	tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_BUF) & LAN8841_PTP_GPIO_MASK;
	tmp &= ~BIT(pin);
	phy_write_mmd(phydev, 2, LAN8841_GPIO_BUF, tmp);
}

static void lan8841_ptp_perout_on(struct kszphy_ptp_priv *ptp_priv, int pin)
{
	struct phy_device *phydev = ptp_priv->phydev;
	u16 tmp;

	tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_EN) & LAN8841_PTP_GPIO_MASK;
	tmp |= BIT(pin);
	phy_write_mmd(phydev, 2, LAN8841_GPIO_EN, tmp);

	tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_DIR) & LAN8841_PTP_GPIO_MASK;
	tmp |= BIT(pin);
	phy_write_mmd(phydev, 2, LAN8841_GPIO_DIR, tmp);

	tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_BUF) & LAN8841_PTP_GPIO_MASK;
	tmp |= BIT(pin);
	phy_write_mmd(phydev, 2, LAN8841_GPIO_BUF, tmp);
}

#define LAN8841_EVENT_A		0
#define LAN8841_EVENT_B		1
static s8 lan8841_ptp_get_event(struct kszphy_ptp_priv *ptp_priv, int pin)
{
	if (ptp_priv->event_a_pin < 0 || ptp_priv->event_a_pin == pin) {
		ptp_priv->event_a_pin = pin;
		return LAN8841_EVENT_A;
	}

	if (ptp_priv->event_b_pin < 0 || ptp_priv->event_b_pin == pin) {
		ptp_priv->event_b_pin = pin;
		return LAN8841_EVENT_B;
	}

	return -1;
}

#define LAN8841_GPIO_DATA_SEL1		131
#define LAN8841_GPIO_DATA_SEL2		132
#define LAN8841_GPIO_DATA_SEL_GPIO_DATA_SEL_EVENT_MASK	GENMASK(2,0)
#define LAN8841_GPIO_DATA_SEL_GPIO_DATA_SEL_EVENT_A	1
#define LAN8841_GPIO_DATA_SEL_GPIO_DATA_SEL_EVENT_B	2
#define LAN8841_PTP_GENERAL_CONFIG		257
#define LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_POL_A		BIT(1)
#define LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_POL_B		BIT(3)
#define LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_A_MASK		GENMASK(7,4)
#define LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_B_MASK		GENMASK(11,8)
#define LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_A		4
#define LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_B		8
static void lan8841_ptp_remove_event(struct kszphy_ptp_priv *ptp_priv, int pin)
{
	struct phy_device *phydev = ptp_priv->phydev;
	u8 offset;
	u16 tmp;

	/* Nowt remove pin from the event. GPIO_DATA_SEL1 contains the GPIO
	 * pins 0-4 while GPIO_DATA_SEL2 contains GPIO pins 5-9, therefore
	 * depending on the pin, it requires to read a different register
	 */
	if (pin < 5) {
		tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_DATA_SEL1);
		offset = pin;
	} else {
		tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_DATA_SEL2);
		offset = pin - 5;
	}
	tmp &= ~(LAN8841_GPIO_DATA_SEL_GPIO_DATA_SEL_EVENT_MASK << (3 * offset));
	if (pin < 5)
		phy_write_mmd(phydev, 2, LAN8841_GPIO_DATA_SEL1, tmp);
	else
		phy_write_mmd(phydev, 2, LAN8841_GPIO_DATA_SEL2, tmp);

	/* Disable the event */
	tmp = phy_read_mmd(phydev, 2, LAN8841_PTP_GENERAL_CONFIG);
	if (ptp_priv->event_a_pin == pin) {
		tmp &= ~LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_POL_A;
		tmp &= ~LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_A_MASK;
		ptp_priv->event_a_pin = -1;
	}

	if (ptp_priv->event_b_pin == pin) {
		tmp &= ~LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_POL_B;
		tmp &= ~LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_B_MASK;
		ptp_priv->event_b_pin = -1;
	}
	phy_write_mmd(phydev, 2, LAN8841_PTP_GENERAL_CONFIG, tmp);
}

static void lan8841_ptp_enable_event(struct kszphy_ptp_priv *ptp_priv, int pin,
				     s8 event, int pulse_width)
{
	struct phy_device *phydev = ptp_priv->phydev;
	u8 offset;
	u16 tmp;

	/* Enable the event */
	tmp = phy_read_mmd(phydev, 2, LAN8841_PTP_GENERAL_CONFIG);
	tmp |= event == LAN8841_EVENT_A ? LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_POL_A :
					  LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_POL_B;
	tmp &= event == LAN8841_EVENT_A ? ~LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_A_MASK :
					  ~LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_B_MASK;
	tmp |= event == LAN8841_EVENT_A ? pulse_width << LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_A :
					  pulse_width << LAN8841_PTP_GENERAL_CONFIG_LTC_EVENT_B;
	phy_write_mmd(phydev, 2, LAN8841_PTP_GENERAL_CONFIG, tmp);

	/* Now connect the pin to the event. GPIO_DATA_SEL1 contains the GPIO
	 * pins 0-4 while GPIO_DATA_SEL2 contains GPIO pins 5-9, therefore
	 * depending on the pin, it requires to read a different register
	 */
	if (pin < 5) {
		tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_DATA_SEL1);
		offset = pin;
	} else {
		tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_DATA_SEL2);
		offset = pin - 5;
	}
	tmp |= (event == LAN8841_EVENT_A ? LAN8841_GPIO_DATA_SEL_GPIO_DATA_SEL_EVENT_A :
					   LAN8841_GPIO_DATA_SEL_GPIO_DATA_SEL_EVENT_B) << (3 * offset);
	if (pin < 5)
		phy_write_mmd(phydev, 2, LAN8841_GPIO_DATA_SEL1, tmp);
	else
		phy_write_mmd(phydev, 2, LAN8841_GPIO_DATA_SEL2, tmp);
}

#define LAN8841_PTP_LTC_TARGET_SEC_HI(event)	((event) == LAN8841_EVENT_A ? 278 : 288)
#define LAN8841_PTP_LTC_TARGET_SEC_LO(event)	((event) == LAN8841_EVENT_A ? 279 : 289)
#define LAN8841_PTP_LTC_TARGET_NS_HI(event)	((event) == LAN8841_EVENT_A ? 280 : 290)
#define LAN8841_PTP_LTC_TARGET_NS_LO(event)	((event) == LAN8841_EVENT_A ? 281 : 291)
static void lan8841_ptp_set_target(struct kszphy_ptp_priv *ptp_priv, s8 event,
				   s64 sec, u32 nsec)
{
	struct phy_device *phydev = ptp_priv->phydev;

	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_TARGET_SEC_HI(event),
		      upper_16_bits(sec));
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_TARGET_SEC_LO(event),
		      lower_16_bits(sec));
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_TARGET_NS_HI(event),
		      upper_16_bits(nsec) & 0x3fff);
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_TARGET_NS_LO(event),
		      lower_16_bits(nsec));
}

#define LAN8841_BUFFER_TIME	2
static void lan8841_ptp_update_target(struct kszphy_ptp_priv *ptp_priv,
				      const struct timespec64 *ts)
{
	if (ptp_priv->event_a_pin >= 0)
		lan8841_ptp_set_target(ptp_priv, LAN8841_EVENT_A,
				       ts->tv_sec + LAN8841_BUFFER_TIME, 0);
	if (ptp_priv->event_b_pin >= 0)
		lan8841_ptp_set_target(ptp_priv, LAN8841_EVENT_B,
				       ts->tv_sec + LAN8841_BUFFER_TIME, 0);
}

#define LAN8841_PTP_LTC_TARGET_RELOAD_SEC_HI(event)	((event) == LAN8841_EVENT_A ? 282 : 292)
#define LAN8841_PTP_LTC_TARGET_RELOAD_SEC_LO(event)	((event) == LAN8841_EVENT_A ? 283 : 293)
#define LAN8841_PTP_LTC_TARGET_RELOAD_NS_HI(event)	((event) == LAN8841_EVENT_A ? 284 : 294)
#define LAN8841_PTP_LTC_TARGET_RELOAD_NS_LO(event)	((event) == LAN8841_EVENT_A ? 285 : 295)
static void lan8841_ptp_set_reload(struct kszphy_ptp_priv *ptp_priv, s8 event,
				   s64 sec, u32 nsec)
{
	struct phy_device *phydev = ptp_priv->phydev;

	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_TARGET_RELOAD_SEC_HI(event),
		      upper_16_bits(sec));
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_TARGET_RELOAD_SEC_LO(event),
		      lower_16_bits(sec));
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_TARGET_RELOAD_NS_HI(event),
		      upper_16_bits(nsec) & 0x3fff);
	phy_write_mmd(phydev, 2, LAN8841_PTP_LTC_TARGET_RELOAD_NS_LO(event),
		      lower_16_bits(nsec));
}

static int lan8841_ptp_perout(struct ptp_clock_info *ptp,
			      struct ptp_clock_request *rq, int on)
{
	struct kszphy_ptp_priv *ptp_priv = container_of(ptp, struct kszphy_ptp_priv,
							ptp_clock_info);
	struct phy_device *phydev = ptp_priv->phydev;
	int pulse_width;
	s8 event;
	int pin;
	int ret;

	if (rq->perout.flags & ~PTP_PEROUT_DUTY_CYCLE)
		return -EOPNOTSUPP;

	pin = ptp_find_pin(ptp_priv->ptp_clock, PTP_PF_PEROUT, rq->perout.index);
	if (pin == -1 || pin >= LAN8841_PTP_GPIO_NUM)
		return -EINVAL;

	if (!on) {
		lan8841_ptp_perout_off(ptp_priv, pin);
		lan8841_ptp_remove_event(ptp_priv, pin);
		return 0;
	}

	ret = lan88xx_get_pulsewidth(phydev, &rq->perout, &pulse_width);
	if (ret < 0)
		return ret;

	/* Don't need to check the event as it already is checked in verify */
	event = lan8841_ptp_get_event(ptp_priv, pin);

	mutex_lock(&ptp_priv->ptp_lock);
	lan8841_ptp_set_target(ptp_priv, event, rq->perout.start.sec,
			       rq->perout.start.nsec);
	mutex_unlock(&ptp_priv->ptp_lock);
	lan8841_ptp_set_reload(ptp_priv, event, rq->perout.period.sec,
			       rq->perout.period.nsec);
	lan8841_ptp_enable_event(ptp_priv, pin, event, pulse_width);
	lan8841_ptp_perout_on(ptp_priv, pin);

	return 0;
}

#define LAN8841_PTP_GPIO_CAP_STS	506
#define LAN8841_PTP_GPIO_CAP_STS_PTP_GPIO_RE_STS(gpio)	(BIT(gpio))
#define LAN8841_PTP_GPIO_CAP_STS_PTP_GPIO_FE_STS(gpio)	(BIT(gpio) << 8)
#define LAN8841_PTP_GPIO_SEL		327
#define LAN8841_PTP_GPIO_SEL_GPIO_SEL(gpio)	((gpio) << 8)
#define LAN8841_PTP_GPIO_RE_LTC_SEC_HI_CAP	498
#define LAN8841_PTP_GPIO_RE_LTC_SEC_LO_CAP	499
#define LAN8841_PTP_GPIO_RE_LTC_NS_HI_CAP	500
#define LAN8841_PTP_GPIO_RE_LTC_NS_LO_CAP	501
#define LAN8841_PTP_GPIO_FE_LTC_SEC_HI_CAP	502
#define LAN8841_PTP_GPIO_FE_LTC_SEC_LO_CAP	503
#define LAN8841_PTP_GPIO_FE_LTC_NS_HI_CAP	504
#define LAN8841_PTP_GPIO_FE_LTC_NS_LO_CAP	505
static void lan8841_gpio_process_cap(struct kszphy_ptp_priv *ptp_priv)
{
	struct phy_device *phydev = ptp_priv->phydev;
	struct ptp_clock_event ptp_event = {0};
	s32 sec, nsec;
	int pin;
	u16 tmp;

	pin = ptp_find_pin_unlocked(ptp_priv->ptp_clock, PTP_PF_EXTTS, 0);
	if (pin == -1)
		return;

	tmp = phy_read_mmd(phydev, 2, LAN8841_PTP_GPIO_CAP_STS);
	if (!(tmp & LAN8841_PTP_GPIO_CAP_STS_PTP_GPIO_RE_STS(pin)) &&
	    !(tmp & LAN8841_PTP_GPIO_CAP_STS_PTP_GPIO_FE_STS(pin)))
		return;

	phy_write_mmd(phydev, 2, LAN8841_PTP_GPIO_SEL,
		      LAN8841_PTP_GPIO_SEL_GPIO_SEL(pin));

	mutex_lock(&ptp_priv->ptp_lock);
	if (tmp & BIT(pin)) {
		sec = phy_read_mmd(phydev, 2, LAN8841_PTP_GPIO_RE_LTC_SEC_HI_CAP);
		sec <<= 16;
		sec |= phy_read_mmd(phydev, 2, LAN8841_PTP_GPIO_RE_LTC_SEC_LO_CAP);

		nsec = phy_read_mmd(phydev, 2, LAN8841_PTP_GPIO_RE_LTC_NS_HI_CAP) & 0x3fff;
		nsec <<= 16;
		nsec |= phy_read_mmd(phydev, 2, LAN8841_PTP_GPIO_RE_LTC_NS_LO_CAP);
	} else {
		sec = phy_read_mmd(phydev, 2, LAN8841_PTP_GPIO_RE_LTC_SEC_HI_CAP);
		sec <<= 16;
		sec |= phy_read_mmd(phydev, 2, LAN8841_PTP_GPIO_RE_LTC_SEC_LO_CAP);

		nsec = phy_read_mmd(phydev, 2, LAN8841_PTP_GPIO_RE_LTC_NS_HI_CAP) & 0x3fff;
		nsec <<= 16;
		nsec |= phy_read_mmd(phydev, 2, LAN8841_PTP_GPIO_RE_LTC_NS_LO_CAP);
	}
	mutex_unlock(&ptp_priv->ptp_lock);

	ptp_event.index = 0;
	ptp_event.timestamp = ktime_set(sec, nsec);
	ptp_event.type = PTP_CLOCK_EXTTS;
	ptp_clock_event(ptp_priv->ptp_clock, &ptp_event);

	phy_write_mmd(phydev, 2, LAN8841_PTP_GPIO_SEL, 0);
}

#define LAN8841_PTP_GPIO_CAP_EN		496
#define LAN8841_PTP_GPIO_CAP_EN_GPIO_RE_CAPTURE_ENABLE(gpio)	(BIT(gpio))
#define LAN8841_PTP_GPIO_CAP_EN_GPIO_FE_CAPTURE_ENABLE(gpio)	(BIT(gpio) << 8)
#define LAN8841_PTP_INT_EN_PTP_GPIO_CAP_EN	BIT(2)
static void lan8841_ptp_extts_on(struct kszphy_ptp_priv *ptp_priv, int pin,
				 u32 flags)
{
	struct phy_device *phydev = ptp_priv->phydev;
	u16 tmp;

	/* Set GPIO to be input */
	tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_EN) & LAN8841_PTP_GPIO_MASK;
	tmp |= BIT(pin);
	phy_write_mmd(phydev, 2, LAN8841_GPIO_EN, tmp);

	tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_DIR) & LAN8841_PTP_GPIO_MASK;
	tmp &= ~BIT(pin);
	phy_write_mmd(phydev, 2, LAN8841_GPIO_DIR, tmp);

	/* Enable capture on the edges of the pin */
	tmp = phy_read_mmd(phydev, 2, LAN8841_PTP_GPIO_CAP_EN);
	if (flags & PTP_RISING_EDGE)
		tmp |= LAN8841_PTP_GPIO_CAP_EN_GPIO_RE_CAPTURE_ENABLE(pin);
	if (flags & PTP_FALLING_EDGE)
		tmp |= LAN8841_PTP_GPIO_CAP_EN_GPIO_FE_CAPTURE_ENABLE(pin);
	phy_write_mmd(phydev, 2, LAN8841_PTP_GPIO_CAP_EN, tmp);

	/* Enable interrupt */
	phy_modify_mmd(phydev, 2, LAN8841_PTP_INT_EN,
		       LAN8841_PTP_INT_EN_PTP_GPIO_CAP_EN,
		       LAN8841_PTP_INT_EN_PTP_GPIO_CAP_EN);

}

static void lan8841_ptp_extts_off(struct kszphy_ptp_priv *ptp_priv, int pin)
{
	struct phy_device *phydev = ptp_priv->phydev;
	u16 tmp;

	/* Set GPIO to be input */
	tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_EN) & LAN8841_PTP_GPIO_MASK;
	tmp &= ~BIT(pin);
	phy_write_mmd(phydev, 2, LAN8841_GPIO_EN, tmp);

	tmp = phy_read_mmd(phydev, 2, LAN8841_GPIO_DIR) & LAN8841_PTP_GPIO_MASK;
	tmp &= ~BIT(pin);
	phy_write_mmd(phydev, 2, LAN8841_GPIO_DIR, tmp);

	/* Disable capture on both of the edges */
	tmp = phy_read_mmd(phydev, 2, LAN8841_PTP_GPIO_CAP_EN);
	tmp &= ~LAN8841_PTP_GPIO_CAP_EN_GPIO_RE_CAPTURE_ENABLE(pin);
	tmp &= ~LAN8841_PTP_GPIO_CAP_EN_GPIO_FE_CAPTURE_ENABLE(pin);
	phy_write_mmd(phydev, 2, LAN8841_PTP_GPIO_CAP_EN, tmp);

	/* Disable interrupt */
	phy_modify_mmd(phydev, 2, LAN8841_PTP_INT_EN,
		       LAN8841_PTP_INT_EN_PTP_GPIO_CAP_EN,
		       0);
}

static int lan8841_ptp_extts(struct ptp_clock_info *ptp,
			     struct ptp_clock_request *rq, int on)
{
	struct kszphy_ptp_priv *ptp_priv = container_of(ptp, struct kszphy_ptp_priv,
							ptp_clock_info);
	int pin;

	/* Reject requests with unsupported flags */
	if (rq->extts.flags & ~(PTP_ENABLE_FEATURE |
				PTP_EXTTS_EDGES |
				PTP_STRICT_FLAGS))
		return -EOPNOTSUPP;

	pin = ptp_find_pin(ptp_priv->ptp_clock, PTP_PF_EXTTS, rq->extts.index);
	if (pin == -1 || pin >= LAN8841_PTP_GPIO_NUM)
		return -EINVAL;

	mutex_lock(&ptp_priv->ptp_lock);
	if (on) {
		lan8841_ptp_extts_on(ptp_priv, pin, rq->extts.flags);
	} else {
		lan8841_ptp_extts_off(ptp_priv, pin);
	}
	mutex_unlock(&ptp_priv->ptp_lock);

	return 0;
}

static int lan8841_ptp_enable(struct ptp_clock_info *ptp,
			      struct ptp_clock_request *rq, int on)
{
	switch (rq->type) {
	case PTP_CLK_REQ_PEROUT:
		return lan8841_ptp_perout(ptp, rq, on);
	case PTP_CLK_REQ_EXTTS:
		return lan8841_ptp_extts(ptp, rq, on);
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static struct ptp_clock_info lan8841_ptp_clock_info = {
	.owner		= THIS_MODULE,
	.name		= "lan8841 ptp",
	.max_adj	= 31249999,
	.gettime64	= lan8841_ptp_gettime64,
	.settime64	= lan8841_ptp_settime64,
	.adjtime	= lan8841_ptp_adjtime,
	.adjfine	= lan8841_ptp_adjfine,
	.verify		= lan8841_ptp_verify,
	.enable		= lan8841_ptp_enable,
	.n_per_out	= LAN8841_PTP_GPIO_NUM,
	.n_ext_ts	= LAN8841_PTP_GPIO_NUM,
	.n_pins		= LAN8841_PTP_GPIO_NUM,
};

static int lan8841_soft_reset(struct phy_device *phydev)
{
	phy_write(phydev, MII_BMCR,
		  BMCR_RESET | BMCR_ANENABLE | BMCR_FULLDPLX | BMCR_SPEED1000);
	lan8841_config_init(phydev);
	phy_read(phydev, MII_BMCR);

	return 0;
}

#define LAN8841_OPERATION_MODE_STRAP_LOW_REGISTER 3
#define LAN8841_OPERATION_MODE_STRAP_LOW_REGISTER_STRAP_RGMII_EN BIT(0)
static int lan8841_probe(struct phy_device *phydev)
{
	struct kszphy_ptp_priv *ptp_priv;
	struct kszphy_priv *priv;
	int err;
	int i;

	err = kszphy_probe(phydev);
	if (err)
		return err;

	if (phy_read_mmd(phydev, 2, LAN8841_OPERATION_MODE_STRAP_LOW_REGISTER) &
	    LAN8841_OPERATION_MODE_STRAP_LOW_REGISTER_STRAP_RGMII_EN)
		phydev->interface = PHY_INTERFACE_MODE_RGMII_RXID;

	/* Skip if PTP is not enabled */
	if (!IS_ENABLED(CONFIG_PTP_1588_CLOCK) ||
	    !IS_ENABLED(CONFIG_NETWORK_PHY_TIMESTAMPING))
		return 0;

	/* Register the clock */
	priv = phydev->priv;
	ptp_priv = &priv->ptp_priv;

	ptp_priv->pin_config = devm_kmalloc_array(&phydev->mdio.dev,
						  LAN8841_PTP_GPIO_NUM,
						  sizeof(*ptp_priv->pin_config),
						  GFP_KERNEL);
	if (!ptp_priv->pin_config)
		return -ENOMEM;

	for (i = 0; i < LAN8841_PTP_GPIO_NUM; ++i) {
		struct ptp_pin_desc *p = &ptp_priv->pin_config[i];

		memset(p, 0, sizeof(*p));
		snprintf(p->name, sizeof(p->name), "pin%d", i);
		p->index = i;
		p->func = PTP_PF_NONE;
	}

	ptp_priv->event_a_pin = -1;
	ptp_priv->event_b_pin = -1;
	ptp_priv->ptp_clock_info = lan8841_ptp_clock_info;
	ptp_priv->ptp_clock_info.pin_config = ptp_priv->pin_config;
	ptp_priv->ptp_clock = ptp_clock_register(&ptp_priv->ptp_clock_info,
						 &phydev->mdio.dev);
	if (IS_ERR_OR_NULL(ptp_priv->ptp_clock)) {
		phydev_err(phydev, "ptp_clock_register failed: %lu\n",
			   PTR_ERR(ptp_priv->ptp_clock));
		return -EINVAL;
	}

	/* Initialize the SW */
	skb_queue_head_init(&ptp_priv->tx_queue);
	skb_queue_head_init(&ptp_priv->rx_queue);
	INIT_LIST_HEAD(&ptp_priv->rx_ts_list);
	spin_lock_init(&ptp_priv->rx_ts_lock);
	ptp_priv->phydev = phydev;
	mutex_init(&ptp_priv->ptp_lock);

	ptp_priv->mii_ts.rxtstamp = lan8814_rxtstamp;
	ptp_priv->mii_ts.txtstamp = lan8814_txtstamp;
	ptp_priv->mii_ts.hwtstamp = lan8841_hwtstamp;
	ptp_priv->mii_ts.ts_info = lan8841_ts_info;

	phydev->mii_ts = &ptp_priv->mii_ts;

	return 0;
}

static struct phy_driver ksphy_driver[] = {
{
	.phy_id		= PHY_ID_LAN8841,
	.phy_id_mask	= MICREL_PHY_ID_MASK,
	.name		= "Microchip LAN8841 Gigabit PHY",
	.flags		= PHY_POLL_CABLE_TEST,
	.driver_data	= &lan8841_type,
	.config_init	= lan8841_config_init,
	.probe		= lan8841_probe,
	.config_intr	= lan8841_config_intr,
	.handle_interrupt = lan8841_handle_interrupt,
	.get_sqi	= lan8841_get_sqi,
	.get_sqi_max	= lan8814_get_sqi_max,
	.soft_reset	= lan8841_soft_reset,
	.get_sset_count = kszphy_get_sset_count,
	.get_strings	= kszphy_get_strings,
	.get_stats	= kszphy_get_stats,
	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
	/* cable test and sqi are almost compatible between lan8814 and lan8841 */
	.cable_test_start	= lan8814_cable_test_start,
	.cable_test_get_status	= ksz886x_cable_test_get_status,
}
};

module_phy_driver(ksphy_driver);

static struct mdio_device_id __maybe_unused micrel_tbl[] = {
	{ PHY_ID_LAN8841, MICREL_PHY_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, micrel_tbl);
MODULE_LICENSE("GPL");
