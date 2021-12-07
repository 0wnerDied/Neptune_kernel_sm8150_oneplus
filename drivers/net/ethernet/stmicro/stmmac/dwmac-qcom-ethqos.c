// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018-19, Linaro Limited
// Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mii.h>
#include <linux/of_mdio.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/debugfs.h>
#include <asm/dma-iommu.h>
#include <linux/iommu.h>
#include <linux/micrel_phy.h>
#include <linux/tcp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/rtnetlink.h>
#include <linux/if_vlan.h>
#include <linux/msm_eth.h>
#include <soc/qcom/sb_notification.h>

#include "stmmac.h"
#include "stmmac_platform.h"
#include "dwmac-qcom-ethqos.h"
#include "stmmac_ptp.h"
#include "dwmac-qcom-ipa-offload.h"

static struct msm_bus_scale_pdata *emac_bus_scale_vec;

#define PHY_LOOPBACK_1000 0x4140
#define PHY_LOOPBACK_100 0x6100
#define PHY_LOOPBACK_10 0x4100
#define MDIO_RD_WR_OPS_CLOCK 1

#define DMA_TX_SIZE_CV2X 128
#define DMA_RX_SIZE_CV2X 128

static void __iomem *tlmm_central_base_addr;
static void ethqos_rgmii_io_macro_loopback(struct qcom_ethqos *ethqos,
					   int mode);
static int phy_digital_loopback_config(
	struct qcom_ethqos *ethqos, int speed, int config);

static struct ethqos_emac_por emac_por[] = {
	{ .offset = RGMII_IO_MACRO_CONFIG,	.value = 0x0 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG,	.value = 0x0 },
	{ .offset = SDCC_HC_REG_DDR_CONFIG,	.value = 0x0 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG2,	.value = 0x0 },
	{ .offset = SDCC_USR_CTL,		.value = 0x0 },
	{ .offset = RGMII_IO_MACRO_CONFIG2,	.value = 0x0},
};

static struct ethqos_emac_driver_data emac_por_data = {
	.por = emac_por,
	.num_por = ARRAY_SIZE(emac_por),
};

static char err_names[10][14] = {"PHY_RW_ERR",
	"PHY_DET_ERR",
	"CRC_ERR",
	"RECEIVE_ERR",
	"OVERFLOW_ERR",
	"FBE_ERR",
	"RBU_ERR",
	"TDU_ERR",
	"DRIBBLE_ERR",
	"WDT_ERR",
};
struct qcom_ethqos *pethqos;

struct stmmac_emb_smmu_cb_ctx stmmac_emb_smmu_ctx = {0};
static unsigned char dev_addr[ETH_ALEN] = {
	0, 0x55, 0x7b, 0xb5, 0x7d, 0xf7};
static unsigned char config_dev_addr[ETH_ALEN] = {0};

void *ipc_stmmac_log_ctxt;
void *ipc_stmmac_log_ctxt_low;
int stmmac_enable_ipc_low;
#define MAX_PROC_SIZE 1024
char tmp_buff[MAX_PROC_SIZE];
static struct qmp_pkt pkt;
static char qmp_buf[MAX_QMP_MSG_SIZE + 1] = {0};
static struct ip_params pparams;

static DECLARE_WAIT_QUEUE_HEAD(mac_rec_wq);
static bool mac_rec_wq_flag;

struct qcom_ethqos *get_pethqos(void)
{
	return pethqos;
}

static int qcom_ethqos_get_bus_config(struct platform_device *pdev)
{
	int o, i;

	emac_bus_scale_vec = msm_bus_cl_get_pdata(pdev);
	if (!emac_bus_scale_vec) {
		ETHQOSERR("unable to get bus scaling vector\n");
		return -EPERM;
	}

	ETHQOSDBG("bus name: %s\n", emac_bus_scale_vec->name);
	ETHQOSDBG("num of paths: %d\n", emac_bus_scale_vec->usecase->num_paths);
	ETHQOSDBG("num of use cases: %d\n", emac_bus_scale_vec->num_usecases);

	for (o = 0; o < emac_bus_scale_vec->num_usecases; o++) {
		ETHQOSDBG("use case[%d] parameters:\n", o);
		for (i = 0; i < emac_bus_scale_vec->usecase->num_paths; i++)
			ETHQOSDBG("src_prt:%d dst_prt:%d ab:%llu ib:%llu\n",
				  emac_bus_scale_vec->usecase[o].vectors[i].src,
				  emac_bus_scale_vec->usecase[o].vectors[i].dst,
				  emac_bus_scale_vec->usecase[o].vectors[i].ab,
				  emac_bus_scale_vec->usecase[o].vectors[i].ib);
	}
	return 0;
}

static void qcom_ethqos_read_iomacro_por_values(struct qcom_ethqos *ethqos)
{
	int i;

	ethqos->por = emac_por_data.por;
	ethqos->num_por = emac_por_data.num_por;

	/* Read to POR values and enable clk */
	for (i = 0; i < ethqos->num_por; i++)
		ethqos->por[i].value =
			readl_relaxed(
				ethqos->rgmii_base +
				ethqos->por[i].offset);
}

unsigned int dwmac_qcom_get_eth_type(unsigned char *buf)
{
	return
		((((u16)buf[QTAG_ETH_TYPE_OFFSET] << 8) |
		  buf[QTAG_ETH_TYPE_OFFSET + 1]) == ETH_P_8021Q) ?
		(((u16)buf[QTAG_VLAN_ETH_TYPE_OFFSET] << 8) |
		 buf[QTAG_VLAN_ETH_TYPE_OFFSET + 1]) :
		 (((u16)buf[QTAG_ETH_TYPE_OFFSET] << 8) |
		  buf[QTAG_ETH_TYPE_OFFSET + 1]);
}

static inline unsigned int dwmac_qcom_get_vlan_ucp(unsigned char  *buf)
{
	return
		(((u16)buf[QTAG_UCP_FIELD_OFFSET] << 8)
		 | buf[QTAG_UCP_FIELD_OFFSET + 1]);
}

u16 dwmac_qcom_select_queue(
	struct net_device *dev,
	struct sk_buff *skb,
	void *accel_priv,
	select_queue_fallback_t fallback)
{
	u16 txqueue_select = ALL_OTHER_TRAFFIC_TX_CHANNEL;
	unsigned int eth_type, priority, vlan_id;
	bool ipa_enabled = pethqos->ipa_enabled;

	/* Retrieve ETH type */
	eth_type = dwmac_qcom_get_eth_type(skb->data);

	if (eth_type == ETH_P_TSN && pethqos->cv2x_mode == CV2X_MODE_DISABLE) {
		/* Read VLAN priority field from skb->data */
		priority = dwmac_qcom_get_vlan_ucp(skb->data);

		priority >>= VLAN_TAG_UCP_SHIFT;
		if (priority == CLASS_A_TRAFFIC_UCP) {
			txqueue_select = CLASS_A_TRAFFIC_TX_CHANNEL;
		} else if (priority == CLASS_B_TRAFFIC_UCP) {
			txqueue_select = CLASS_B_TRAFFIC_TX_CHANNEL;
		} else {
			if (ipa_enabled)
				txqueue_select = ALL_OTHER_TRAFFIC_TX_CHANNEL;
			else
				txqueue_select =
				ALL_OTHER_TX_TRAFFIC_IPA_DISABLED;
		}
	} else if (eth_type == ETH_P_1588) {
		/*gPTP seelct tx queue 1*/
		txqueue_select = NON_TAGGED_IP_TRAFFIC_TX_CHANNEL;
	} else if (skb_vlan_tag_present(skb)) {
		vlan_id = skb_vlan_tag_get_id(skb);

		if (pethqos->cv2x_mode == CV2X_MODE_AP &&
		    vlan_id == pethqos->cv2x_vlan.vlan_id) {
			txqueue_select = CV2X_TAG_TX_CHANNEL;
		} else if (pethqos->qoe_mode &&
			 vlan_id == pethqos->qoe_vlan.vlan_id){
			txqueue_select = QMI_TAG_TX_CHANNEL;
		} else {
			if (ipa_enabled)
				txqueue_select = ALL_OTHER_TRAFFIC_TX_CHANNEL;
			else
				txqueue_select =
				ALL_OTHER_TX_TRAFFIC_IPA_DISABLED;
		}
	} else {
		/* VLAN tagged IP packet or any other non vlan packets (PTP)*/
		if (ipa_enabled)
			txqueue_select = ALL_OTHER_TRAFFIC_TX_CHANNEL;
		else
			txqueue_select = ALL_OTHER_TX_TRAFFIC_IPA_DISABLED;
	}

	/* use better macro, cannot afford function call here */
	if (ipa_enabled && (txqueue_select == IPA_DMA_TX_CH_BE ||
			    (pethqos->cv2x_mode != CV2X_MODE_DISABLE &&
			     txqueue_select == IPA_DMA_TX_CH_CV2X))) {
		ETHQOSERR("TX Channel [%d] is not a valid for SW path\n",
			  txqueue_select);
		WARN_ON(1);
	}

	return txqueue_select;
}

int dwmac_qcom_program_avb_algorithm(
	struct stmmac_priv *priv, struct ifr_data_struct *req)
{
	struct dwmac_qcom_avb_algorithm l_avb_struct, *u_avb_struct =
		(struct dwmac_qcom_avb_algorithm *)req->ptr;
	struct dwmac_qcom_avb_algorithm_params *avb_params;
	int ret = 0;

	ETHQOSDBG("\n");

	if (copy_from_user(&l_avb_struct, (void __user *)u_avb_struct,
			   sizeof(struct dwmac_qcom_avb_algorithm))) {
		ETHQOSERR("Failed to fetch AVB Struct\n");
		return -EFAULT;
	}

	if (priv->speed == SPEED_1000)
		avb_params = &l_avb_struct.speed1000params;
	else
		avb_params = &l_avb_struct.speed100params;

	/* Application uses 1 for CLASS A traffic and
	 * 2 for CLASS B traffic
	 * Configure right channel accordingly
	 */
	if (l_avb_struct.qinx == 1) {
		l_avb_struct.qinx = CLASS_A_TRAFFIC_TX_CHANNEL;
	} else if (l_avb_struct.qinx == 2) {
		l_avb_struct.qinx = CLASS_B_TRAFFIC_TX_CHANNEL;
	} else {
		ETHQOSERR("Invalid index [%u] in AVB struct from user\n",
			  l_avb_struct.qinx);
		return -EFAULT;
	}

	priv->plat->tx_queues_cfg[l_avb_struct.qinx].mode_to_use =
		MTL_QUEUE_AVB;
	priv->plat->tx_queues_cfg[l_avb_struct.qinx].send_slope =
		avb_params->send_slope,
	priv->plat->tx_queues_cfg[l_avb_struct.qinx].idle_slope =
		avb_params->idle_slope,
	priv->plat->tx_queues_cfg[l_avb_struct.qinx].high_credit =
		avb_params->hi_credit,
	priv->plat->tx_queues_cfg[l_avb_struct.qinx].low_credit =
		avb_params->low_credit,

	priv->hw->mac->config_cbs(
	   priv->hw, priv->plat->tx_queues_cfg[l_avb_struct.qinx].send_slope,
	   priv->plat->tx_queues_cfg[l_avb_struct.qinx].idle_slope,
	   priv->plat->tx_queues_cfg[l_avb_struct.qinx].high_credit,
	   priv->plat->tx_queues_cfg[l_avb_struct.qinx].low_credit,
	   l_avb_struct.qinx);

	ETHQOSDBG("\n");
	return ret;
}

unsigned int dwmac_qcom_get_plat_tx_coal_frames(
	struct sk_buff *skb)
{
	bool is_udp;
	unsigned int eth_type;

	eth_type = dwmac_qcom_get_eth_type(skb->data);

#ifdef CONFIG_PTPSUPPORT_OBJ
	if (eth_type == ETH_P_1588)
		return PTP_INT_MOD;
#endif

	if (eth_type == ETH_P_TSN)
		return AVB_INT_MOD;
	if (eth_type == ETH_P_IP || eth_type == ETH_P_IPV6) {
#ifdef CONFIG_PTPSUPPORT_OBJ
		is_udp = (((eth_type == ETH_P_IP) &&
				   (ip_hdr(skb)->protocol ==
					IPPROTO_UDP)) ||
				  ((eth_type == ETH_P_IPV6) &&
				   (ipv6_hdr(skb)->nexthdr ==
					IPPROTO_UDP)));

		if (is_udp && ((udp_hdr(skb)->dest ==
			htons(PTP_UDP_EV_PORT)) ||
			(udp_hdr(skb)->dest ==
			  htons(PTP_UDP_GEN_PORT))))
			return PTP_INT_MOD;
#endif
		return IP_PKT_INT_MOD;
	}
	return DEFAULT_INT_MOD;
}

int ethqos_handle_prv_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct stmmac_priv *pdata = netdev_priv(dev);
	struct ifr_data_struct req;
	struct pps_cfg eth_pps_cfg;
	int ret = 0;

	if (copy_from_user(&req, ifr->ifr_ifru.ifru_data,
			   sizeof(struct ifr_data_struct)))
		return -EFAULT;

	switch (req.cmd) {
	case ETHQOS_CONFIG_PPSOUT_CMD:
		if (copy_from_user(&eth_pps_cfg, (void __user *)req.ptr,
				   sizeof(struct pps_cfg)))
			return -EFAULT;

		if (eth_pps_cfg.ppsout_ch < 0 ||
		    eth_pps_cfg.ppsout_ch >= pdata->dma_cap.pps_out_num)
			ret = -EOPNOTSUPP;
		else if ((eth_pps_cfg.ppsout_align == 1) &&
			 ((eth_pps_cfg.ppsout_ch != DWC_ETH_QOS_PPS_CH_0) &&
			 (eth_pps_cfg.ppsout_ch != DWC_ETH_QOS_PPS_CH_3)))
			ret = -EOPNOTSUPP;
		else
			ret = ppsout_config(pdata, &eth_pps_cfg);
		break;
	case ETHQOS_AVB_ALGORITHM:
		ret = dwmac_qcom_program_avb_algorithm(pdata, &req);
		break;
	default:
		break;
	}
	return ret;
}

static void ethqos_update_ahb_bus_cfg(struct stmmac_priv *priv,
				      bool ahb_bus_cfg)
{
	static int vote_idx;
	struct qcom_ethqos *ethqos;
	int ret;

	if (!priv->plat->bsp_priv)
		return;

	ethqos = (struct qcom_ethqos *)priv->plat->bsp_priv;

	if (ethqos->skip_mdio_vote)
		return;

	if (ethqos->bus_hdl && ahb_bus_cfg == ETH_AHB_BUS_CFG_NOMINAL) {
		vote_idx = ethqos->vote_idx;
		ethqos->vote_idx = VOTE_IDX_MDIO_NOM_CLK;
		ret = msm_bus_scale_client_update_request(ethqos->bus_hdl,
							  ethqos->vote_idx);
		WARN_ON(ret);
	}

	else if (ethqos->bus_hdl && ahb_bus_cfg == ETH_AHB_BUS_CFG_RECOVER) {
		ethqos->vote_idx = vote_idx;
		ret = msm_bus_scale_client_update_request(ethqos->bus_hdl,
							  ethqos->vote_idx);
		WARN_ON(ret);
	}
}

static void ethqos_update_ahb_clk_cfg(void *priv_n, bool ahb_bus_cfg,
				      bool skip_mdio_vote)
{
	struct stmmac_priv *priv = priv_n;
	struct qcom_ethqos *ethqos;

	if (!priv->plat->bsp_priv)
		return;

	ethqos = (struct qcom_ethqos *)priv->plat->bsp_priv;
	ethqos->skip_mdio_vote = skip_mdio_vote;

	ethqos_update_ahb_bus_cfg(priv, ahb_bus_cfg);
}

static int __init set_early_ethernet_ipv4(char *ipv4_addr_in)
{
	int ret = 1;

	pparams.is_valid_ipv4_addr = false;

	if (!ipv4_addr_in)
		return ret;

	strlcpy(pparams.ipv4_addr_str,
		ipv4_addr_in, sizeof(pparams.ipv4_addr_str));
	ETHQOSDBG("Early ethernet IPv4 addr: %s\n", pparams.ipv4_addr_str);

	ret = in4_pton(pparams.ipv4_addr_str, -1,
		       (u8 *)&pparams.ipv4_addr.s_addr, -1, NULL);
	if (ret != 1 || pparams.ipv4_addr.s_addr == 0) {
		ETHQOSERR("Invalid ipv4 address programmed: %s\n",
			  ipv4_addr_in);
		return ret;
	}

	pparams.is_valid_ipv4_addr = true;
	return ret;
}

__setup("eipv4=", set_early_ethernet_ipv4);

static int __init set_early_ethernet_ipv6(char *ipv6_addr_in)
{
	int ret = 1;

	pparams.is_valid_ipv6_addr = false;

	if (!ipv6_addr_in)
		return ret;

	strlcpy(pparams.ipv6_addr_str,
		ipv6_addr_in, sizeof(pparams.ipv6_addr_str));
	ETHQOSDBG("Early ethernet IPv6 addr: %s\n", pparams.ipv6_addr_str);

	ret = in6_pton(pparams.ipv6_addr_str, -1,
		       (u8 *)&pparams.ipv6_addr.ifr6_addr.s6_addr32, -1, NULL);
	if (ret != 1 || !pparams.ipv6_addr.ifr6_addr.s6_addr32)  {
		ETHQOSERR("Invalid ipv6 address programmed: %s\n",
			  ipv6_addr_in);
		return ret;
	}

	pparams.is_valid_ipv6_addr = true;
	return ret;
}

__setup("eipv6=", set_early_ethernet_ipv6);

static int __init set_early_ethernet_mac(char *mac_addr)
{
	int ret = 1;
	bool valid_mac = false;

	pparams.is_valid_mac_addr = false;
	if (!mac_addr)
		return ret;

	valid_mac = mac_pton(mac_addr, pparams.mac_addr);
	if (!valid_mac)
		goto fail;

	valid_mac = is_valid_ether_addr(pparams.mac_addr);
	if (!valid_mac)
		goto fail;

	pparams.is_valid_mac_addr = true;
	return ret;

fail:
	ETHQOSERR("Invalid Mac address programmed: %s\n", mac_addr);
	return ret;
}

__setup("ermac=", set_early_ethernet_mac);

static int qcom_ethqos_add_ipaddr(struct ip_params *ip_info,
				  struct net_device *dev)
{
	int res = 0;
	struct ifreq ir;
	struct sockaddr_in *sin = (void *)&ir.ifr_ifru.ifru_addr;
	struct net *net = dev_net(dev);

	if (!net || !net->genl_sock || !net->genl_sock->sk_socket) {
		ETHQOSERR("Sock is null, unable to assign ipv4 address\n");
		return res;
	}

	if (!net->ipv4.devconf_dflt) {
		ETHQOSERR("ipv4.devconf_dflt is null, schedule wq\n");
		schedule_delayed_work(&pethqos->ipv4_addr_assign_wq,
				      msecs_to_jiffies(1000));
		return res;
	}
	/*For valid Ipv4 address*/
	memset(&ir, 0, sizeof(ir));
	memcpy(&sin->sin_addr.s_addr, &ip_info->ipv4_addr,
	       sizeof(sin->sin_addr.s_addr));

	strlcpy(ir.ifr_ifrn.ifrn_name,
		dev->name, sizeof(ir.ifr_ifrn.ifrn_name));
	sin->sin_family = AF_INET;
	sin->sin_port = 0;

	res = inet_ioctl(net->genl_sock->sk_socket,
			 SIOCSIFADDR, (unsigned long)(void *)&ir);
		if (res) {
			ETHQOSERR("can't setup IPv4 address!: %d\r\n", res);
		} else {
			ETHQOSINFO("Assigned IPv4 address: %s\r\n",
				   ip_info->ipv4_addr_str);
#ifdef CONFIG_MSM_BOOT_TIME_MARKER
place_marker("M - Etherent Assigned IPv4 address");
#endif
		}
	return res;
}

#ifdef CONFIG_IPV6
static int qcom_ethqos_add_ipv6addr(struct ip_params *ip_info,
				    struct net_device *dev)
{
	int ret = -EFAULT;
	struct in6_ifreq ir6;
	char *prefix;
	struct net *net = dev_net(dev);
	/*For valid IPv6 address*/

	if (!net || !net->genl_sock || !net->genl_sock->sk_socket) {
		ETHQOSERR("Sock is null, unable to assign ipv6 address\n");
		return -EFAULT;
	}

	if (!net->ipv6.devconf_dflt) {
		ETHQOSERR("ipv6.devconf_dflt is null, schedule wq\n");
		schedule_delayed_work(&pethqos->ipv6_addr_assign_wq,
				      msecs_to_jiffies(1000));
		return ret;
	}
	memset(&ir6, 0, sizeof(ir6));
	memcpy(&ir6, &ip_info->ipv6_addr, sizeof(struct in6_ifreq));
	ir6.ifr6_ifindex = dev->ifindex;

	prefix = strnchr(ip_info->ipv6_addr_str,
			 strlen(ip_info->ipv6_addr_str), '/');

	if (!prefix) {
		ir6.ifr6_prefixlen = 0;
	} else {
		kstrtoul(prefix + 1, 0, (unsigned long *)&ir6.ifr6_prefixlen);
		if (ir6.ifr6_prefixlen > 128)
			ir6.ifr6_prefixlen = 0;
	}
	ret = inet6_ioctl(net->genl_sock->sk_socket,
			  SIOCSIFADDR, (unsigned long)(void *)&ir6);
		if (ret) {
			ETHQOSDBG("Can't setup IPv6 address!\r\n");
		} else {
			ETHQOSDBG("Assigned IPv6 address: %s\r\n",
				  ip_info->ipv6_addr_str);
#ifdef CONFIG_MSM_BOOT_TIME_MARKER
		place_marker("M - Ethernet Assigned IPv6 address");
#endif
		}
	return ret;
}
#endif

static int rgmii_readl(struct qcom_ethqos *ethqos, unsigned int offset)
{
	return readl(ethqos->rgmii_base + offset);
}

static void rgmii_writel(struct qcom_ethqos *ethqos,
			 int value, unsigned int offset)
{
	writel(value, ethqos->rgmii_base + offset);
}

static void rgmii_updatel(struct qcom_ethqos *ethqos,
			  int mask, int val, unsigned int offset)
{
	unsigned int temp;

	temp =  rgmii_readl(ethqos, offset);
	temp = (temp & ~(mask)) | val;
	rgmii_writel(ethqos, temp, offset);
}

static void rgmii_loopback_config(void *priv_n, int loopback_en)
{
	struct stmmac_priv *priv = priv_n;
	struct qcom_ethqos *ethqos;

	if (!priv->plat->bsp_priv)
		return;

	ethqos = (struct qcom_ethqos *)priv->plat->bsp_priv;

	if (loopback_en) {
		rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
			      RGMII_CONFIG_LOOPBACK_EN,
			      RGMII_IO_MACRO_CONFIG);
		ETHQOSINFO("Loopback EN Enabled\n");
	} else {
		rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
			      0, RGMII_IO_MACRO_CONFIG);
		ETHQOSINFO("Loopback EN Disabled\n");
	}
}

static void rgmii_dump(struct qcom_ethqos *ethqos)
{
	dev_dbg(&ethqos->pdev->dev, "Rgmii register dump\n");
	dev_dbg(&ethqos->pdev->dev, "RGMII_IO_MACRO_CONFIG: %x\n",
		rgmii_readl(ethqos, RGMII_IO_MACRO_CONFIG));
	dev_dbg(&ethqos->pdev->dev, "SDCC_HC_REG_DLL_CONFIG: %x\n",
		rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG));
	dev_dbg(&ethqos->pdev->dev, "SDCC_HC_REG_DDR_CONFIG: %x\n",
		rgmii_readl(ethqos, SDCC_HC_REG_DDR_CONFIG));
	dev_dbg(&ethqos->pdev->dev, "SDCC_HC_REG_DLL_CONFIG2: %x\n",
		rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG2));
	dev_dbg(&ethqos->pdev->dev, "SDC4_STATUS: %x\n",
		rgmii_readl(ethqos, SDC4_STATUS));
	dev_dbg(&ethqos->pdev->dev, "SDCC_USR_CTL: %x\n",
		rgmii_readl(ethqos, SDCC_USR_CTL));
	dev_dbg(&ethqos->pdev->dev, "RGMII_IO_MACRO_CONFIG2: %x\n",
		rgmii_readl(ethqos, RGMII_IO_MACRO_CONFIG2));
	dev_dbg(&ethqos->pdev->dev, "RGMII_IO_MACRO_DEBUG1: %x\n",
		rgmii_readl(ethqos, RGMII_IO_MACRO_DEBUG1));
	dev_dbg(&ethqos->pdev->dev, "EMAC_SYSTEM_LOW_POWER_DEBUG: %x\n",
		rgmii_readl(ethqos, EMAC_SYSTEM_LOW_POWER_DEBUG));
}

static void
ethqos_update_rgmii_clk_and_bus_cfg(struct qcom_ethqos *ethqos,
				    unsigned int speed)
{
	int ret;

	switch (speed) {
	case SPEED_1000:
		ethqos->rgmii_clk_rate =  RGMII_1000_NOM_CLK_FREQ;
		break;

	case SPEED_100:
		ethqos->rgmii_clk_rate =  RGMII_ID_MODE_100_LOW_SVS_CLK_FREQ;
		break;

	case SPEED_10:
		ethqos->rgmii_clk_rate =  RGMII_ID_MODE_10_LOW_SVS_CLK_FREQ;
		break;
	}

	switch (speed) {
	case MDIO_RD_WR_OPS_CLOCK:
		ethqos->vote_idx = VOTE_IDX_MDIO_NOM_CLK;
		break;
	case SPEED_1000:
		ethqos->vote_idx = VOTE_IDX_1000MBPS;
		break;
	case SPEED_100:
		ethqos->vote_idx = VOTE_IDX_100MBPS;
		break;
	case SPEED_10:
		ethqos->vote_idx = VOTE_IDX_10MBPS;
		break;
	case 0:
		ethqos->vote_idx = VOTE_IDX_0MBPS;
		ethqos->rgmii_clk_rate = 0;
		break;
	}

	if (ethqos->bus_hdl) {
		ret = msm_bus_scale_client_update_request(ethqos->bus_hdl,
							  ethqos->vote_idx);
		WARN_ON(ret);
	}

	clk_set_rate(ethqos->rgmii_clk, ethqos->rgmii_clk_rate);
}

static int qcom_ethqos_qmp_mailbox_init(struct qcom_ethqos *ethqos)
{
	ethqos->qmp_mbox_client = devm_kzalloc(
	&ethqos->pdev->dev, sizeof(*ethqos->qmp_mbox_client), GFP_KERNEL);

	if (!ethqos->qmp_mbox_client || IS_ERR(ethqos->qmp_mbox_client)) {
		ETHQOSERR("qmp alloc client failed\n");
		return -EINVAL;
	}

	ethqos->qmp_mbox_client->dev = &ethqos->pdev->dev;
	ethqos->qmp_mbox_client->tx_block = true;
	ethqos->qmp_mbox_client->tx_tout = 1000;
	ethqos->qmp_mbox_client->knows_txdone = false;

	ethqos->qmp_mbox_chan = mbox_request_channel(ethqos->qmp_mbox_client,
						     0);

	if (IS_ERR(ethqos->qmp_mbox_chan)) {
		ETHQOSERR("qmp request channel failed\n");
		return -EINVAL;
	}

	return 0;
}

static int qcom_ethqos_qmp_mailbox_send_message(struct qcom_ethqos *ethqos)
{
	int ret = 0;

	memset(&qmp_buf[0], 0, MAX_QMP_MSG_SIZE + 1);

	snprintf(qmp_buf, MAX_QMP_MSG_SIZE, "{class:ctile, pc:0}");

	pkt.size = ((size_t)strlen(qmp_buf) + 0x3) & ~0x3;
	pkt.data = qmp_buf;

	ret = mbox_send_message(ethqos->qmp_mbox_chan, (void *)&pkt);

	ETHQOSDBG("qmp mbox_send_message ret = %d\n", ret);

	if (ret < 0) {
		ETHQOSERR("Disabling c-tile power collapse failed\n");
		return ret;
	}

	ETHQOSDBG("Disabling c-tile power collapse succeded");

	return 0;
}

/**
 *  DWC_ETH_QOS_qmp_mailbox_work - Scheduled from probe
 *  @work: work_struct
 */
static void qcom_ethqos_qmp_mailbox_work(struct work_struct *work)
{
	struct qcom_ethqos *ethqos =
		container_of(work, struct qcom_ethqos, qmp_mailbox_work);

	ETHQOSDBG("Enter\n");

	/* Send QMP message to disable c-tile power collapse */
	qcom_ethqos_qmp_mailbox_send_message(ethqos);

	ETHQOSDBG("Exit\n");
}

static void ethqos_set_func_clk_en(struct qcom_ethqos *ethqos)
{
	rgmii_updatel(ethqos, RGMII_CONFIG_FUNC_CLK_EN,
		      RGMII_CONFIG_FUNC_CLK_EN, RGMII_IO_MACRO_CONFIG);
}

static int ethqos_dll_configure(struct qcom_ethqos *ethqos)
{
	unsigned int val;
	int retry = 1000;

	/* Set CDR_EN */
	if (ethqos->emac_ver == EMAC_HW_v2_3_2 ||
	    ethqos->emac_ver == EMAC_HW_v2_1_2)
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CDR_EN,
			      0, SDCC_HC_REG_DLL_CONFIG);
	else
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CDR_EN,
			      SDCC_DLL_CONFIG_CDR_EN, SDCC_HC_REG_DLL_CONFIG);

	if (ethqos->io_macro.clear_cdt_ext_en)
		/* Clear CDR_EXT_EN */
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CDR_EXT_EN,
			      0, SDCC_HC_REG_DLL_CONFIG);
	else
		/* Set CDR_EXT_EN */
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CDR_EXT_EN,
			      SDCC_DLL_CONFIG_CDR_EXT_EN,
			      SDCC_HC_REG_DLL_CONFIG);

	/* Clear CK_OUT_EN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CK_OUT_EN,
		      0, SDCC_HC_REG_DLL_CONFIG);

	if (!ethqos->io_macro.rx_dll_bypass)
		/* Set DLL_EN */
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_EN,
			      SDCC_DLL_CONFIG_DLL_EN, SDCC_HC_REG_DLL_CONFIG);

	if (ethqos->emac_ver != EMAC_HW_v2_3_2 &&
	    ethqos->emac_ver != EMAC_HW_v2_1_2) {
		rgmii_updatel(ethqos, SDCC_DLL_MCLK_GATING_EN,
			      0, SDCC_HC_REG_DLL_CONFIG);

		rgmii_updatel(ethqos, SDCC_DLL_CDR_FINE_PHASE,
			      0, SDCC_HC_REG_DLL_CONFIG);
	}
	/* Wait for CK_OUT_EN clear */
	do {
		val = rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG);
		val &= SDCC_DLL_CONFIG_CK_OUT_EN;
		if (!val)
			break;
		mdelay(1);
		retry--;
	} while (retry > 0);
	if (!retry)
		dev_err(&ethqos->pdev->dev, "Clear CK_OUT_EN timedout\n");

	/* Set CK_OUT_EN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CK_OUT_EN,
		      SDCC_DLL_CONFIG_CK_OUT_EN, SDCC_HC_REG_DLL_CONFIG);

	/* Wait for CK_OUT_EN set */
	retry = 1000;
	do {
		val = rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG);
		val &= SDCC_DLL_CONFIG_CK_OUT_EN;
		if (val)
			break;
		mdelay(1);
		retry--;
	} while (retry > 0);
	if (!retry)
		dev_err(&ethqos->pdev->dev, "Set CK_OUT_EN timedout\n");

	/* Set DDR_CAL_EN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_CAL_EN,
		      SDCC_DLL_CONFIG2_DDR_CAL_EN, SDCC_HC_REG_DLL_CONFIG2);

	if (ethqos->emac_ver != EMAC_HW_v2_3_2 &&
	    ethqos->emac_ver != EMAC_HW_v2_1_2) {
		if (!ethqos->io_macro.rx_dll_bypass)
			rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DLL_CLOCK_DIS,
				      0, SDCC_HC_REG_DLL_CONFIG2);

		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_MCLK_FREQ_CALC,
			      0x1A << 10, SDCC_HC_REG_DLL_CONFIG2);

		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SEL,
			      BIT(2), SDCC_HC_REG_DLL_CONFIG2);

		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SW,
			      SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SW,
			      SDCC_HC_REG_DLL_CONFIG2);
	}

	return 0;
}

static int ethqos_rgmii_macro_init(struct qcom_ethqos *ethqos)
{
	/* Disable loopback mode */
	rgmii_updatel(ethqos, RGMII_CONFIG2_TX_TO_RX_LOOPBACK_EN,
		      0, RGMII_IO_MACRO_CONFIG2);

	/* Select RGMII, write 0 to interface select */
	rgmii_updatel(ethqos, RGMII_CONFIG_INTF_SEL,
		      0, RGMII_IO_MACRO_CONFIG);

	switch (ethqos->speed) {
	case SPEED_1000:
		rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
			      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
			      0, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
			      RGMII_CONFIG_POS_NEG_DATA_SEL,
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
			      RGMII_CONFIG_PROG_SWAP, RGMII_IO_MACRO_CONFIG);
		if (ethqos->emac_ver != EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
				      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
			      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
			      RGMII_CONFIG2_RX_PROG_SWAP,
			      RGMII_IO_MACRO_CONFIG2);

		if (!ethqos->io_macro.rx_dll_bypass) {
			rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
				      BIT(21), SDCC_HC_REG_DDR_CONFIG);
			rgmii_updatel(ethqos,
				      SDCC_DDR_CONFIG_TCXO_CYCLES_DLY_LINE,
				      BIT(18), SDCC_HC_REG_DDR_CONFIG);
			rgmii_updatel(ethqos, SDCC_DDR_CONFIG_TCXO_CYCLES_CNT,
				      BIT(11), SDCC_HC_REG_DDR_CONFIG);

			rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_RCLK_DLY,
				      ethqos->io_macro.prg_rclk_dly,
				      SDCC_HC_REG_DDR_CONFIG);

			rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_DLY_EN,
				      SDCC_DDR_CONFIG_PRG_DLY_EN,
				      SDCC_HC_REG_DDR_CONFIG);
		}

		if (ethqos->emac_ver == EMAC_HW_v2_3_2 ||
		    ethqos->emac_ver == EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      0, RGMII_IO_MACRO_CONFIG);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_IO_MACRO_CONFIG);
		break;

	case SPEED_100:
		rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
			      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
			      0, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
			      0, RGMII_IO_MACRO_CONFIG);
		if (ethqos->emac_ver != EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
				      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG_MAX_SPD_PRG_2,
			      BIT(6), RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
			      0, RGMII_IO_MACRO_CONFIG2);
		if (ethqos->io_macro.rx_prog_swap)
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_IO_MACRO_CONFIG2);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      0, RGMII_IO_MACRO_CONFIG2);
		/* Write 0x5 to PRG_RCLK_DLY_CODE */
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_CODE,
			      (BIT(29) | BIT(27)), SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_HC_REG_DDR_CONFIG);
		if (ethqos->emac_ver == EMAC_HW_v2_3_2 ||
		    ethqos->emac_ver == EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      0, RGMII_IO_MACRO_CONFIG);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_IO_MACRO_CONFIG);
		break;

	case SPEED_10:
		rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
			      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
			      0, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
			      0, RGMII_IO_MACRO_CONFIG);
		if (ethqos->emac_ver != EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
				      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos,
			      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG_MAX_SPD_PRG_9,
			      BIT(12) | GENMASK(9, 8),
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
			      0, RGMII_IO_MACRO_CONFIG2);
		if (ethqos->io_macro.rx_prog_swap)
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_IO_MACRO_CONFIG2);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      0, RGMII_IO_MACRO_CONFIG2);
		/* Write 0x5 to PRG_RCLK_DLY_CODE */
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_CODE,
			      (BIT(29) | BIT(27)), SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_HC_REG_DDR_CONFIG);
		if (ethqos->emac_ver == EMAC_HW_v2_3_2 ||
		    ethqos->emac_ver == EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      0, RGMII_IO_MACRO_CONFIG);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_IO_MACRO_CONFIG);
		break;
	default:
		dev_err(&ethqos->pdev->dev,
			"Invalid speed %d\n", ethqos->speed);
		return -EINVAL;
	}

	return 0;
}

static int ethqos_configure(struct qcom_ethqos *ethqos)
{
	volatile unsigned int dll_lock;
	unsigned int i, retry = 1000;

	/* Reset to POR values and enable clk */
	for (i = 0; i < ethqos->num_por; i++)
		rgmii_writel(ethqos, ethqos->por[i].value,
			     ethqos->por[i].offset);
	ethqos_set_func_clk_en(ethqos);

	/* Initialize the DLL first */

	/* Set DLL_RST */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_RST,
		      SDCC_DLL_CONFIG_DLL_RST, SDCC_HC_REG_DLL_CONFIG);

	/* Set PDN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_PDN,
		      SDCC_DLL_CONFIG_PDN, SDCC_HC_REG_DLL_CONFIG);

	/* Clear DLL_RST */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_RST, 0,
		      SDCC_HC_REG_DLL_CONFIG);

	/* Clear PDN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_PDN, 0,
		      SDCC_HC_REG_DLL_CONFIG);

	if (ethqos->speed != SPEED_100 && ethqos->speed != SPEED_10) {
		if (ethqos->io_macro.rx_dll_bypass) {
			/* Set DLL_CLOCK_DISABLE */
			rgmii_updatel(ethqos,
				      SDCC_DLL_CONFIG2_DLL_CLOCK_DIS,
				      SDCC_DLL_CONFIG2_DLL_CLOCK_DIS,
				      SDCC_HC_REG_DLL_CONFIG2);

			/* Clear DLL_EN */
			rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_EN,
				      0, SDCC_HC_REG_DLL_CONFIG);

			/* Set PDN */
			rgmii_updatel(ethqos,
				      SDCC_DLL_CONFIG_PDN,
				      SDCC_DLL_CONFIG_PDN,
				      SDCC_HC_REG_DLL_CONFIG);

			if (ethqos->io_macro.usr_ctl_set)
				rgmii_updatel(ethqos, GENMASK(31, 0),
					      ethqos->io_macro.usr_ctl,
					      SDCC_USR_CTL);
			/* Set PDN bit 30 in USR_CTL */
			rgmii_updatel(ethqos, BIT(30), BIT(30), SDCC_USR_CTL);
		} else {
			/* Set DLL_EN */
			rgmii_updatel(ethqos,
				      SDCC_DLL_CONFIG_DLL_EN,
				      SDCC_DLL_CONFIG_DLL_EN,
				      SDCC_HC_REG_DLL_CONFIG);

			/* Set CK_OUT_EN */
			rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CK_OUT_EN,
				      SDCC_DLL_CONFIG_CK_OUT_EN,
				      SDCC_HC_REG_DLL_CONFIG);

			if (ethqos->io_macro.usr_ctl_set)
				/* Set USR_CTL value from dts */
				rgmii_updatel(ethqos, GENMASK(31, 0),
					      ethqos->io_macro.usr_ctl,
					      SDCC_USR_CTL);
			else
				/* Set USR_CTL bit 26 with mask of 3 bits */
				rgmii_updatel(ethqos, GENMASK(26, 24),
					      BIT(26), SDCC_USR_CTL);

			/* wait for DLL LOCK */
			do {
				mdelay(1);
				dll_lock = rgmii_readl(ethqos, SDC4_STATUS);
				if (dll_lock & SDC4_STATUS_DLL_LOCK)
					break;
				retry--;
			} while (retry > 0);
			if (!retry)
				dev_err(&ethqos->pdev->dev,
					"Timeout while waiting for DLL lock\n");
		}

		ethqos_dll_configure(ethqos);
	}

	ethqos_rgmii_macro_init(ethqos);

	return 0;
}

static void ethqos_fix_mac_speed(void *priv, unsigned int speed)
{
	struct qcom_ethqos *ethqos = priv;

	ethqos->speed = speed;
	ethqos_update_rgmii_clk_and_bus_cfg(ethqos, speed);
	ethqos_configure(ethqos);
}

static int ethqos_mdio_read(struct stmmac_priv  *priv, int phyaddr, int phyreg)
{
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;
	u32 v;
	int data;
	u32 value = MII_BUSY;
	struct qcom_ethqos *ethqos = priv->plat->bsp_priv;

	if (ethqos->phy_state == PHY_IS_OFF) {
		ETHQOSINFO("Phy is in off state reading is not possible\n");
		return -EOPNOTSUPP;
	}

	value |= (phyaddr << priv->hw->mii.addr_shift)
		& priv->hw->mii.addr_mask;
	value |= (phyreg << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;
	value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
		& priv->hw->mii.clk_csr_mask;
	if (priv->plat->has_gmac4)
		value |= MII_GMAC4_READ;

	if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       100, 10000)) {
		if (priv->plat->handle_mac_err)
			priv->plat->handle_mac_err(priv, PHY_RW_ERR, 0);
		return -EBUSY;
	}
	writel_relaxed(value, priv->ioaddr + mii_address);

	if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       100, 10000)) {
		if (priv->plat->handle_mac_err)
			priv->plat->handle_mac_err(priv, PHY_RW_ERR, 0);
		return -EBUSY;
	}
	/* Read the data from the MII data register */
	data = (int)readl_relaxed(priv->ioaddr + mii_data);

	return data;
}

static int ethqos_phy_intr_config(struct qcom_ethqos *ethqos)
{
	int ret = 0;
	struct platform_device *pdev = ethqos->pdev;
	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);

	ethqos->phy_intr = platform_get_irq_byname(ethqos->pdev, "phy-intr");

	if (ethqos->phy_intr < 0) {
		if (ethqos->phy_intr != -EPROBE_DEFER) {
			dev_err(&ethqos->pdev->dev,
				"PHY IRQ configuration information not found\n");
		}
		ret = 1;
	} else {
		priv->wol_irq = ethqos->phy_intr;
	}

	return ret;
}

static void ethqos_handle_phy_interrupt(struct qcom_ethqos *ethqos)
{
	int phy_intr_status = 0;
	struct platform_device *pdev = ethqos->pdev;

	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);
	int micrel_intr_status = 0;

	if (dev->phydev && ((dev->phydev->phy_id &
	    dev->phydev->drv->phy_id_mask) == MICREL_PHY_ID)) {
		phy_intr_status = ethqos_mdio_read(
			priv, priv->plat->phy_addr, DWC_ETH_QOS_BASIC_STATUS);
		ETHQOSDBG(
			"Basic Status Reg (%#x) = %#x\n",
			DWC_ETH_QOS_BASIC_STATUS, phy_intr_status);
		micrel_intr_status = ethqos_mdio_read(
			priv, priv->plat->phy_addr,
			DWC_ETH_QOS_MICREL_PHY_INTCS);
		ETHQOSDBG(
			"MICREL PHY Intr EN Reg (%#x) = %#x\n",
			DWC_ETH_QOS_MICREL_PHY_INTCS, micrel_intr_status);
		/* Call ack interrupt to clear the WOL status fields */
		if (dev->phydev->drv->ack_interrupt)
			dev->phydev->drv->ack_interrupt(dev->phydev);

		/* Interrupt received for link state change */
		if (phy_intr_status & LINK_STATE_MASK) {
			if (micrel_intr_status & MICREL_LINK_UP_INTR_STATUS)
				ETHQOSDBG("Intr for link UP state\n");
			phy_mac_interrupt(dev->phydev, LINK_UP);
		} else if (!(phy_intr_status & LINK_STATE_MASK)) {
			ETHQOSDBG("Intr for link DOWN state\n");
			phy_mac_interrupt(dev->phydev, LINK_DOWN);
		} else if (!(phy_intr_status & AUTONEG_STATE_MASK)) {
			ETHQOSDBG("Intr for link down with auto-neg err\n");
		} else if (phy_intr_status & PHY_WOL) {
			ETHQOSDBG("Interrupt received for WoL packet\n");
		}
	} else {
		phy_intr_status =
		 ethqos_mdio_read(
		    priv, priv->plat->phy_addr, DWC_ETH_QOS_PHY_INTR_STATUS);
		if (dev->phydev && (phy_intr_status & LINK_UP_STATE))
			phy_mac_interrupt(dev->phydev, LINK_UP);
		else if (dev->phydev && (phy_intr_status & LINK_DOWN_STATE))
			phy_mac_interrupt(dev->phydev, LINK_DOWN);
	}
}

static void ethqos_defer_phy_isr_work(struct work_struct *work)
{
	struct qcom_ethqos *ethqos =
		container_of(work, struct qcom_ethqos, emac_phy_work);

	if (ethqos->clks_suspended)
		wait_for_completion(&ethqos->clk_enable_done);

	ethqos_handle_phy_interrupt(ethqos);
}

static irqreturn_t ETHQOS_PHY_ISR(int irq, void *dev_data)
{
	struct qcom_ethqos *ethqos = (struct qcom_ethqos *)dev_data;

	pm_wakeup_event(&ethqos->pdev->dev, 5000);

	queue_work(system_wq, &ethqos->emac_phy_work);

	return IRQ_HANDLED;
}

static int ethqos_phy_intr_enable(void *priv_n)
{
	int ret = 0;
	struct stmmac_priv *priv = priv_n;
	struct qcom_ethqos *ethqos = priv->plat->bsp_priv;

	if (ethqos_phy_intr_config(ethqos)) {
		ret = 1;
		return ret;
	}

	INIT_WORK(&ethqos->emac_phy_work, ethqos_defer_phy_isr_work);
	init_completion(&ethqos->clk_enable_done);

	ret = request_irq(ethqos->phy_intr, ETHQOS_PHY_ISR,
			  IRQF_SHARED, "stmmac", ethqos);
	if (ret) {
		ETHQOSERR("Unable to register PHY IRQ %d\n",
			  ethqos->phy_intr);
		return ret;
	}
	priv->plat->phy_intr_en_extn_stm = true;
	return ret;
}

static void ethqos_pps_irq_config(struct qcom_ethqos *ethqos)
{
	ethqos->pps_class_a_irq =
	platform_get_irq_byname(ethqos->pdev, "ptp_pps_irq_0");
	if (ethqos->pps_class_a_irq < 0) {
		if (ethqos->pps_class_a_irq != -EPROBE_DEFER)
			ETHQOSERR("class_a_irq config info not found\n");
	}
	ethqos->pps_class_b_irq =
	platform_get_irq_byname(ethqos->pdev, "ptp_pps_irq_1");
	if (ethqos->pps_class_b_irq < 0) {
		if (ethqos->pps_class_b_irq != -EPROBE_DEFER)
			ETHQOSERR("class_b_irq config info not found\n");
	}
}

static const struct of_device_id qcom_ethqos_match[] = {
	{ .compatible = "qcom,sdxprairie-ethqos",},
	{ .compatible = "qcom,emac-smmu-embedded", },
	{ .compatible = "qcom,stmmac-ethqos", },
	{}
};

static ssize_t read_phy_reg_dump(struct file *file, char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct qcom_ethqos *ethqos = file->private_data;
	struct platform_device *pdev;
	struct net_device *dev;
	struct stmmac_priv *priv;
	unsigned int len = 0, buf_len = 2000;
	char *buf;
	ssize_t ret_cnt;
	int phydata = 0;
	int i = 0;

	if (!ethqos) {
		ETHQOSERR("NULL Pointer\n");
		return -EINVAL;
	}

	pdev = ethqos->pdev;
	dev = platform_get_drvdata(pdev);
	priv = netdev_priv(dev);

	if (!ethqos || !dev->phydev) {
		ETHQOSERR("NULL Pointer\n");
		return -EINVAL;
	}
	if (ethqos->phy_state == PHY_IS_OFF) {
		ETHQOSINFO("Phy is in off state phy dump is not possible\n");
		return -EOPNOTSUPP;
	}

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len,
					 "\n************* PHY Reg dump *************\n");

	for (i = 0; i < 32; i++) {
		phydata = ethqos_mdio_read(priv, priv->plat->phy_addr, i);
		len += scnprintf(buf + len, buf_len - len,
					 "MII Register (%#x) = %#x\n",
					 i, phydata);
	}

	if (len > buf_len) {
		ETHQOSERR("(len > buf_len) buffer not sufficient\n");
		len = buf_len;
	}

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret_cnt;
}

static ssize_t read_rgmii_reg_dump(struct file *file,
				   char __user *user_buf, size_t count,
				   loff_t *ppos)
{
	struct qcom_ethqos *ethqos = file->private_data;
	struct platform_device *pdev;
	struct net_device *dev;
	unsigned int len = 0, buf_len = 2000;
	char *buf;
	ssize_t ret_cnt;
	int rgmii_data = 0;

	if (!ethqos) {
		ETHQOSERR("NULL Pointer\n");
		return -EINVAL;
	}

	pdev = ethqos->pdev;
	dev = platform_get_drvdata(pdev);

	if (!ethqos || !dev->phydev) {
		ETHQOSERR("NULL Pointer\n");
		return -EINVAL;
	}

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len,
					 "\n************* RGMII Reg dump *************\n");
	rgmii_data = rgmii_readl(ethqos, RGMII_IO_MACRO_CONFIG);
	len += scnprintf(buf + len, buf_len - len,
					 "RGMII_IO_MACRO_CONFIG Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG);
	len += scnprintf(buf + len, buf_len - len,
					 "SDCC_HC_REG_DLL_CONFIG Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, SDCC_HC_REG_DDR_CONFIG);
	len += scnprintf(buf + len, buf_len - len,
					 "SDCC_HC_REG_DDR_CONFIG Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG2);
	len += scnprintf(buf + len, buf_len - len,
					 "SDCC_HC_REG_DLL_CONFIG2 Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, SDC4_STATUS);
	len += scnprintf(buf + len, buf_len - len,
					 "SDC4_STATUS Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, SDCC_USR_CTL);
	len += scnprintf(buf + len, buf_len - len,
					 "SDCC_USR_CTL Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, RGMII_IO_MACRO_CONFIG2);
	len += scnprintf(buf + len, buf_len - len,
					 "RGMII_IO_MACRO_CONFIG2 Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, RGMII_IO_MACRO_DEBUG1);
	len += scnprintf(buf + len, buf_len - len,
					 "RGMII_IO_MACRO_DEBUG1 Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, EMAC_SYSTEM_LOW_POWER_DEBUG);
	len += scnprintf(buf + len, buf_len - len,
					 "EMAC_SYSTEM_LOW_POWER_DEBUG Register = %#x\n",
					 rgmii_data);

	if (len > buf_len) {
		ETHQOSERR("(len > buf_len) buffer not sufficient\n");
		len = buf_len;
	}

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret_cnt;
}

static ssize_t read_phy_off(struct file *file,
			    char __user *user_buf,
			    size_t count, loff_t *ppos)
{
	unsigned int len = 0, buf_len = 2000;
	char *buf;
	ssize_t ret_cnt;
	struct qcom_ethqos *ethqos = file->private_data;

	if (!ethqos) {
		ETHQOSERR("NULL Pointer\n");
		return -EINVAL;
	}
	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (ethqos->current_phy_mode == DISABLE_PHY_IMMEDIATELY)
		len += scnprintf(buf + len, buf_len - len,
				"Disable phy immediately enabled\n");
	else if (ethqos->current_phy_mode == ENABLE_PHY_IMMEDIATELY)
		len += scnprintf(buf + len, buf_len - len,
				 "Enable phy immediately enabled\n");
	else if (ethqos->current_phy_mode == DISABLE_PHY_AT_SUSPEND_ONLY) {
		len += scnprintf(buf + len, buf_len - len,
				 "Disable Phy at suspend\n");
		len += scnprintf(buf + len, buf_len - len,
				 " & do not enable at resume enabled\n");
	} else if (ethqos->current_phy_mode ==
		 DISABLE_PHY_SUSPEND_ENABLE_RESUME) {
		len += scnprintf(buf + len, buf_len - len,
				 "Disable Phy at suspend\n");
		len += scnprintf(buf + len, buf_len - len,
				 " & enable at resume enabled\n");
	} else if (ethqos->current_phy_mode == DISABLE_PHY_ON_OFF)
		len += scnprintf(buf + len, buf_len - len,
				 "Disable phy on/off disabled\n");
	else
		len += scnprintf(buf + len, buf_len - len,
					"Invalid Phy State\n");

	if (len > buf_len) {
		ETHQOSERR("(len > buf_len) buffer not sufficient\n");
		len = buf_len;
	}

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret_cnt;
}

static ssize_t phy_off_config(
	struct file *file, const char __user *user_buffer,
	size_t count, loff_t *position)
{
	char *in_buf;
	int buf_len = 2000;
	unsigned long ret;
	int config = 0;
	struct qcom_ethqos *ethqos = file->private_data;
	struct platform_device *pdev = ethqos->pdev;
	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = qcom_ethqos_get_priv(ethqos);
	struct plat_stmmacenet_data *plat;

	plat = priv->plat;
	in_buf = kzalloc(buf_len, GFP_KERNEL);
	if (!in_buf)
		return -ENOMEM;

	ret = copy_from_user(in_buf, user_buffer, buf_len);
	if (ret) {
		ETHQOSERR("unable to copy from user\n");
		return -EFAULT;
	}

	ret = sscanf(in_buf, "%d", &config);
	if (ret != 1) {
		ETHQOSERR("Error in reading option from user");
		return -EINVAL;
	}
	if (config > DISABLE_PHY_ON_OFF || config < DISABLE_PHY_IMMEDIATELY) {
		ETHQOSERR("Invalid option =%d", config);
		return -EINVAL;
	}
	if (config == ethqos->current_phy_mode) {
		ETHQOSERR("No effect as duplicate config");
		return -EPERM;
	}
	if (config == DISABLE_PHY_IMMEDIATELY) {
		ethqos->current_phy_mode = DISABLE_PHY_IMMEDIATELY;
	//make phy off
		if (priv->current_loopback == ENABLE_PHY_LOOPBACK) {
			/* If Phy loopback is enabled
			 *  Disabled It before phy off
			 */
			phy_digital_loopback_config(ethqos,
						    ethqos->loopback_speed, 0);
			ETHQOSDBG("Disable phy Loopback\n");
			priv->current_loopback = ENABLE_PHY_LOOPBACK;
		}
		/*Backup phy related data*/
		if (priv->phydev && priv->phydev->autoneg == AUTONEG_DISABLE) {
			ethqos->backup_autoneg = priv->phydev->autoneg;
			ethqos->backup_bmcr = ethqos_mdio_read(priv,
							       plat->phy_addr,
							       MII_BMCR);
		} else {
			ethqos->backup_autoneg = AUTONEG_ENABLE;
		}
		if (priv->phydev) {
			if (qcom_ethqos_is_phy_link_up(ethqos)) {
				ETHQOSINFO("Post Link down before PHY off\n");
				netif_carrier_off(dev);
				phy_mac_interrupt(priv->phydev, LINK_DOWN);
			}
		}
		ethqos_phy_power_off(ethqos);
	} else if (config == ENABLE_PHY_IMMEDIATELY) {
		ethqos->current_phy_mode = ENABLE_PHY_IMMEDIATELY;
		//make phy on
		ethqos_phy_power_on(ethqos);
		ethqos_reset_phy_enable_interrupt(ethqos);
		if (ethqos->backup_autoneg == AUTONEG_DISABLE) {
			priv->phydev->autoneg = ethqos->backup_autoneg;
			phy_write(priv->phydev, MII_BMCR, ethqos->backup_bmcr);
		}
		if (priv->current_loopback == ENABLE_PHY_LOOPBACK) {
			/*If Phy loopback is enabled , enabled It again*/
			phy_digital_loopback_config(ethqos,
						    ethqos->loopback_speed, 1);
			ETHQOSDBG("Enabling Phy loopback again");
		}
	} else if (config == DISABLE_PHY_AT_SUSPEND_ONLY) {
		ethqos->current_phy_mode = DISABLE_PHY_AT_SUSPEND_ONLY;
	} else if (config == DISABLE_PHY_SUSPEND_ENABLE_RESUME) {
		ethqos->current_phy_mode = DISABLE_PHY_SUSPEND_ENABLE_RESUME;
	} else if (config == DISABLE_PHY_ON_OFF) {
		ethqos->current_phy_mode = DISABLE_PHY_ON_OFF;
	} else {
		ETHQOSERR("Invalid option\n");
		return -EINVAL;
	}
	kfree(in_buf);
	return count;
}

static void ethqos_rgmii_io_macro_loopback(struct qcom_ethqos *ethqos, int mode)
{
	/* Set loopback mode */
	if (mode == 1) {
		rgmii_updatel(ethqos, RGMII_CONFIG2_TX_TO_RX_LOOPBACK_EN,
			      RGMII_CONFIG2_TX_TO_RX_LOOPBACK_EN,
			      RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
			      0, RGMII_IO_MACRO_CONFIG2);
	} else {
		rgmii_updatel(ethqos, RGMII_CONFIG2_TX_TO_RX_LOOPBACK_EN,
			      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
			      RGMII_CONFIG2_RX_PROG_SWAP,
			      RGMII_IO_MACRO_CONFIG2);
	}
}

static void ethqos_mac_loopback(struct qcom_ethqos *ethqos, int mode)
{
	u32 read_value = (u32)readl_relaxed(ethqos->ioaddr + MAC_CONFIGURATION);
	/* Set loopback mode */
	if (mode == 1)
		read_value |= MAC_LM;
	else
		read_value &= ~MAC_LM;
	writel_relaxed(read_value, ethqos->ioaddr + MAC_CONFIGURATION);
}

static int phy_digital_loopback_config(
	struct qcom_ethqos *ethqos, int speed, int config)
{
	struct platform_device *pdev = ethqos->pdev;
	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);
	int phydata = 0;

	if (config == 1) {
		ETHQOSINFO("Request for phy digital loopback enable\n");
		switch (speed) {
		case SPEED_1000:
			phydata = PHY_LOOPBACK_1000;
			break;
		case SPEED_100:
			phydata = PHY_LOOPBACK_100;
			break;
		case SPEED_10:
			phydata = PHY_LOOPBACK_10;
			break;
		default:
			ETHQOSERR("Invalid link speed\n");
			break;
		}
	} else if (config == 0) {
		ETHQOSINFO("Request for phy digital loopback disable\n");
		if (ethqos->bmcr_backup)
			phydata = ethqos->bmcr_backup;
		else
			phydata = 0x1140;
	} else {
		ETHQOSERR("Invalid option\n");
		return -EINVAL;
	}
	if (phydata != 0) {
		if (priv->phydev) {
			phy_write(priv->phydev, MII_BMCR, phydata);
			ETHQOSINFO("write done for phy loopback\n");
		} else {
			ETHQOSINFO("Phy dev is NULL\n");
		}
	}
	return 0;
}

static void print_loopback_detail(enum loopback_mode loopback)
{
	switch (loopback) {
	case DISABLE_LOOPBACK:
		ETHQOSINFO("Loopback is disabled\n");
		break;
	case ENABLE_IO_MACRO_LOOPBACK:
		ETHQOSINFO("Loopback is Enabled as IO MACRO LOOPBACK\n");
		break;
	case ENABLE_MAC_LOOPBACK:
		ETHQOSINFO("Loopback is Enabled as MAC LOOPBACK\n");
		break;
	case ENABLE_PHY_LOOPBACK:
		ETHQOSINFO("Loopback is Enabled as PHY LOOPBACK\n");
		break;
	default:
		ETHQOSINFO("Invalid Loopback=%d\n", loopback);
		break;
	}
}

static void setup_config_registers(struct qcom_ethqos *ethqos,
				   int speed, int duplex, int mode)
{
	struct platform_device *pdev = ethqos->pdev;
	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);
	u32 ctrl = 0;

	ETHQOSDBG("Speed=%d,dupex=%d,mode=%d\n", speed, duplex, mode);

	if (mode > DISABLE_LOOPBACK && !qcom_ethqos_is_phy_link_up(ethqos)) {
		/*If Link is Down & need to enable Loopback*/
		ETHQOSDBG("Enable Lower Up Flag & disable phy dev\n");
		ETHQOSDBG("IRQ so that Rx/Tx can happen beforeee Link down\n");
		netif_carrier_on(dev);
		/*Disable phy interrupt by Link/Down by cable plug in/out*/
		disable_irq(ethqos->phy_intr);
	} else if (mode > DISABLE_LOOPBACK &&
			qcom_ethqos_is_phy_link_up(ethqos)) {
		ETHQOSDBG("Only disable phy irqqq Lin is UP\n");
		/*Since link is up no need to set Lower UP flag*/
		/*Disable phy interrupt by Link/Down by cable plug in/out*/
		disable_irq(ethqos->phy_intr);
	} else if (mode == DISABLE_LOOPBACK &&
		!qcom_ethqos_is_phy_link_up(ethqos)) {
		ETHQOSDBG("Disable Lower Up as Link is down\n");
		netif_carrier_off(dev);
		enable_irq(ethqos->phy_intr);
	}
	ETHQOSDBG("Old ctrl=%d  dupex full\n", ctrl);
	ctrl = readl_relaxed(priv->ioaddr + MAC_CTRL_REG);
		ETHQOSDBG("Old ctrl=0x%x with mask with flow control\n", ctrl);

	ctrl |= priv->hw->link.duplex;
	priv->dev->phydev->duplex = duplex;
	ctrl &= ~priv->hw->link.speed_mask;
	switch (speed) {
	case SPEED_1000:
		ctrl |= priv->hw->link.speed1000;
		break;
	case SPEED_100:
		ctrl |= priv->hw->link.speed100;
		break;
	case SPEED_10:
		ctrl |= priv->hw->link.speed10;
		break;
	default:
		speed = SPEED_UNKNOWN;
		ETHQOSDBG("unkwon speed\n");
		break;
	}
	writel_relaxed(ctrl, priv->ioaddr + MAC_CTRL_REG);
	ETHQOSDBG("New ctrl=%x priv hw speeed =%d\n", ctrl,
		  priv->hw->link.speed1000);
	priv->dev->phydev->speed = speed;
	priv->speed  = speed;

	if (mode > DISABLE_LOOPBACK && pethqos->ipa_enabled)
		priv->plat->offload_event_handler(ethqos, EV_LOOPBACK_DMA_MAP);
	else
		priv->hw->mac->map_mtl_to_dma(priv->hw, EMAC_QUEUE_0,
					      EMAC_CHANNEL_0);
	if (priv->dev->phydev->speed != SPEED_UNKNOWN)
		ethqos_fix_mac_speed(ethqos, speed);

	if (mode > DISABLE_LOOPBACK) {
		if (mode == ENABLE_MAC_LOOPBACK ||
		    mode == ENABLE_IO_MACRO_LOOPBACK)
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_IO_MACRO_CONFIG);
	} else if (mode == DISABLE_LOOPBACK) {
		if (ethqos->emac_ver == EMAC_HW_v2_3_2 ||
		    ethqos->emac_ver == EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      0, RGMII_IO_MACRO_CONFIG);
	}
	ETHQOSERR("End\n");
}

static ssize_t loopback_handling_config(
	struct file *file, const char __user *user_buffer,
	size_t count, loff_t *position)
{
	char *in_buf;
	int buf_len = 2000;
	unsigned long ret;
	int config = 0;
	struct qcom_ethqos *ethqos = file->private_data;
	struct platform_device *pdev = ethqos->pdev;
	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);
	int speed = 0;

	in_buf = kzalloc(buf_len, GFP_KERNEL);
	if (!in_buf)
		return -ENOMEM;

	ret = copy_from_user(in_buf, user_buffer, buf_len);
	if (ret) {
		ETHQOSERR("unable to copy from user\n");
		return -EFAULT;
	}

	ret = sscanf(in_buf, "%d %d", &config,  &speed);
	if (config > DISABLE_LOOPBACK && ret != 2) {
		ETHQOSERR("Speed is also needed while enabling loopback\n");
		return -EINVAL;
	}
	if (config < DISABLE_LOOPBACK || config > ENABLE_PHY_LOOPBACK) {
		ETHQOSERR("Invalid config =%d\n", config);
		return -EINVAL;
	}
	if (priv->current_loopback == ENABLE_PHY_LOOPBACK &&
	    priv->plat->mac2mac_en) {
		ETHQOSINFO("Not supported with Mac2Mac enabled\n");
		return -EOPNOTSUPP;
	}
	if ((config == ENABLE_PHY_LOOPBACK  || priv->current_loopback ==
			ENABLE_PHY_LOOPBACK) &&
			ethqos->current_phy_mode == DISABLE_PHY_IMMEDIATELY) {
		ETHQOSERR("Can't enabled/disable ");
		ETHQOSERR("phy loopback when phy is off\n");
		return -EPERM;
	}

	/*Argument validation*/
	if (config == ENABLE_IO_MACRO_LOOPBACK ||
	    config == ENABLE_MAC_LOOPBACK || config == ENABLE_PHY_LOOPBACK) {
		if (speed != SPEED_1000 && speed != SPEED_100 &&
		    speed != SPEED_10)
			return -EINVAL;
	}

	if (config == priv->current_loopback) {
		switch (config) {
		case DISABLE_LOOPBACK:
			ETHQOSINFO("Loopback is already disabled\n");
			break;
		case ENABLE_IO_MACRO_LOOPBACK:
			ETHQOSINFO("Loopback is already Enabled as ");
			ETHQOSINFO("IO MACRO LOOPBACK\n");
			break;
		case ENABLE_MAC_LOOPBACK:
			ETHQOSINFO("Loopback is already Enabled as ");
			ETHQOSINFO("MAC LOOPBACK\n");
			break;
		case ENABLE_PHY_LOOPBACK:
			ETHQOSINFO("Loopback is already Enabled as ");
			ETHQOSINFO("PHY LOOPBACK\n");
			break;
		}
		return -EINVAL;
	}
	/*If request to enable loopback & some other loopback already enabled*/
	if (config != DISABLE_LOOPBACK &&
	    priv->current_loopback > DISABLE_LOOPBACK) {
		ETHQOSINFO("Loopback is already enabled\n");
		print_loopback_detail(priv->current_loopback);
		return -EINVAL;
	}
	ETHQOSINFO("enable loopback = %d with link speed = %d backup now\n",
		   config, speed);

	/*Backup speed & duplex before Enabling Loopback */
	if (priv->current_loopback == DISABLE_LOOPBACK &&
	    config > DISABLE_LOOPBACK) {
		/*Backup old speed & duplex*/
		ethqos->backup_speed = priv->speed;
		ethqos->backup_duplex = priv->dev->phydev->duplex;
	}
	/*Backup BMCR before Enabling Phy LoopbackLoopback */
	if (priv->current_loopback == DISABLE_LOOPBACK &&
	    config == ENABLE_PHY_LOOPBACK)
		ethqos->bmcr_backup = ethqos_mdio_read(priv,
						       priv->plat->phy_addr,
						       MII_BMCR);

	if (config == DISABLE_LOOPBACK)
		setup_config_registers(ethqos, ethqos->backup_speed,
				       ethqos->backup_duplex, 0);
	else
		setup_config_registers(ethqos, speed, DUPLEX_FULL, config);

	switch (config) {
	case DISABLE_LOOPBACK:
		ETHQOSINFO("Request to Disable Loopback\n");
		if (priv->current_loopback == ENABLE_IO_MACRO_LOOPBACK)
			ethqos_rgmii_io_macro_loopback(ethqos, 0);
		else if (priv->current_loopback == ENABLE_MAC_LOOPBACK)
			ethqos_mac_loopback(ethqos, 0);
		else if (priv->current_loopback == ENABLE_PHY_LOOPBACK)
			phy_digital_loopback_config(ethqos,
						    ethqos->backup_speed, 0);
		break;
	case ENABLE_IO_MACRO_LOOPBACK:
		ETHQOSINFO("Request to Enable IO MACRO LOOPBACK\n");
		ethqos_rgmii_io_macro_loopback(ethqos, 1);
		break;
	case ENABLE_MAC_LOOPBACK:
		ETHQOSINFO("Request to Enable MAC LOOPBACK\n");
		ethqos_mac_loopback(ethqos, 1);
		break;
	case ENABLE_PHY_LOOPBACK:
		ETHQOSINFO("Request to Enable PHY LOOPBACK\n");
		ethqos->loopback_speed = speed;
		phy_digital_loopback_config(ethqos, speed, 1);
		break;
	default:
		ETHQOSINFO("Invalid Loopback=%d\n", config);
		break;
	}

	priv->current_loopback = config;
	kfree(in_buf);
	return count;
}

static ssize_t read_loopback_config(struct file *file,
				    char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	unsigned int len = 0, buf_len = 2000;
	struct qcom_ethqos *ethqos = file->private_data;
	char *buf;
	ssize_t ret_cnt;
	struct platform_device *pdev = ethqos->pdev;
	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);

	if (!ethqos) {
		ETHQOSERR("NULL Pointer\n");
		return -EINVAL;
	}
	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (priv->current_loopback == DISABLE_LOOPBACK)
		len += scnprintf(buf + len, buf_len - len,
				 "Loopback is Disabled\n");
	else if (priv->current_loopback == ENABLE_IO_MACRO_LOOPBACK)
		len += scnprintf(buf + len, buf_len - len,
				 "Current Loopback is IO MACRO LOOPBACK\n");
	else if (priv->current_loopback == ENABLE_MAC_LOOPBACK)
		len += scnprintf(buf + len, buf_len - len,
				 "Current Loopback is MAC LOOPBACK\n");
	else if (priv->current_loopback == ENABLE_PHY_LOOPBACK)
		len += scnprintf(buf + len, buf_len - len,
				 "Current Loopback is PHY LOOPBACK\n");
	else
		len += scnprintf(buf + len, buf_len - len,
				 "Invalid LOOPBACK Config\n");
	if (len > buf_len)
		len = buf_len;

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret_cnt;
}

inline void *qcom_ethqos_get_priv(struct qcom_ethqos *ethqos)
{
	struct platform_device *pdev = ethqos->pdev;
	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);

	return priv;
}

static int ethqos_writeback_desc_rec(struct stmmac_priv *priv, int chan)
{
	return -EOPNOTSUPP;
}

static int ethqos_reset_phy_rec(struct stmmac_priv *priv, int int_en)
{
	struct qcom_ethqos *ethqos =   priv->plat->bsp_priv;
	int ret = 1;

	if (int_en && (priv->phydev &&
		       priv->phydev->autoneg == AUTONEG_DISABLE)) {
		ethqos->backup_autoneg = priv->phydev->autoneg;
		ethqos->backup_bmcr = ethqos_mdio_read(priv,
						       priv->plat->phy_addr,
						       MII_BMCR);
	} else {
		ethqos->backup_autoneg = AUTONEG_ENABLE;
	}

	ethqos_phy_power_off(ethqos);

	ethqos_phy_power_on(ethqos);

	if (int_en) {
		ethqos_reset_phy_enable_interrupt(ethqos);
		if (ethqos->backup_autoneg == AUTONEG_DISABLE && priv->phydev) {
			priv->phydev->autoneg = ethqos->backup_autoneg;
			phy_write(priv->phydev, MII_BMCR, ethqos->backup_bmcr);
		}
	}

	return ret;
}

static int ethqos_reset_tx_dma_rec(struct stmmac_priv *priv, int chan)
{
	struct stmmac_tx_queue *tx_q = &priv->tx_queue[chan];
	int i;
	int ret = 1;
	if (tx_q->skip_sw) {
		priv->plat->offload_event_handler(priv, EV_DEV_CLOSE);
		priv->plat->offload_event_handler(priv, EV_DEV_OPEN);
		priv->dev->stats.tx_errors++;
		return ret;
	}

	netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, chan));
	priv->hw->dma->stop_tx(priv->ioaddr, chan);

	for (i = 0; i < DMA_TX_SIZE; i++) {
		if (tx_q->tx_skbuff_dma[i].buf) {
			if (tx_q->tx_skbuff_dma[i].map_as_page)
				dma_unmap_page(GET_MEM_PDEV_DEV,
					       tx_q->tx_skbuff_dma[i].buf,
					       tx_q->tx_skbuff_dma[i].len,
					       DMA_TO_DEVICE);
			else
				dma_unmap_single(GET_MEM_PDEV_DEV,
						 tx_q->tx_skbuff_dma[i].buf,
						 tx_q->tx_skbuff_dma[i].len,
						 DMA_TO_DEVICE);
		}
		if (tx_q->tx_skbuff[i]) {
			dev_kfree_skb_any(tx_q->tx_skbuff[i]);
			tx_q->tx_skbuff[i] = NULL;
			tx_q->tx_skbuff_dma[i].buf = 0;
			tx_q->tx_skbuff_dma[i].map_as_page = false;
		}
	}

	for (i = 0; i < DMA_TX_SIZE; i++)
		if (priv->extend_desc)
			priv->hw->desc->init_tx_desc(&tx_q->dma_etx[i].basic,
						     priv->mode,
						     (i == DMA_TX_SIZE - 1));
		else
			priv->hw->desc->init_tx_desc(&tx_q->dma_tx[i],
						     priv->mode,
						     (i == DMA_TX_SIZE - 1));
	tx_q->dirty_tx = 0;
	tx_q->cur_tx = 0;
	netdev_tx_reset_queue(netdev_get_tx_queue(priv->dev, chan));
	priv->hw->dma->init_tx_chan(priv->ioaddr, priv->plat->dma_cfg,
				    tx_q->dma_tx_phy, chan);
	priv->hw->dma->start_tx(priv->ioaddr, chan);

	priv->dev->stats.tx_errors++;
	netif_tx_wake_queue(netdev_get_tx_queue(priv->dev, chan));
	return ret;
}

static int ethqos_schedule_poll(struct stmmac_priv *priv, int chan)
{
	struct stmmac_rx_queue *rx_q = &priv->rx_queue[chan];
	int ret = 1;

	if (!rx_q->skip_sw) {
		if (likely(napi_schedule_prep(&rx_q->napi))) {
			priv->hw->dma->disable_dma_irq(priv->ioaddr, chan);
			__napi_schedule(&rx_q->napi);
		}
	}
	return ret;
}

static void ethqos_tdu_rec_wq(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct stmmac_priv *priv;
	struct qcom_ethqos *ethqos;
	int ret;

	ETHQOSDBG("Enter\n");
	dwork = container_of(work, struct delayed_work, work);
	ethqos = container_of(dwork, struct qcom_ethqos, tdu_rec);

	priv = qcom_ethqos_get_priv(ethqos);
	if (!priv)
		return;

	ret = ethqos_schedule_poll(priv, ethqos->tdu_chan);
	if (!ret)
		return;

	ethqos->tdu_scheduled = false;
}

static void ethqos_mac_rec_init(struct qcom_ethqos *ethqos)
{
	int threshold[] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
	int en[] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0};
	int i;

	for (i = 0; i < MAC_ERR_CNT; i++) {
		ethqos->mac_rec_threshold[i] = threshold[i];
		ethqos->mac_rec_en[i] = en[i];
		ethqos->mac_err_cnt[i] = 0;
		ethqos->mac_rec_cnt[i] = 0;
	}
	INIT_DELAYED_WORK(&ethqos->tdu_rec,
			  ethqos_tdu_rec_wq);
}

static int dwmac_qcom_handle_mac_err(void *pdata, int type, int chan)
{
	int ret = 1;
	struct qcom_ethqos *ethqos;
	struct stmmac_priv *priv;

	if (!pdata)
		return -EINVAL;
	priv = pdata;
	ethqos = priv->plat->bsp_priv;

	if (!ethqos)
		return -EINVAL;

	ethqos->mac_err_cnt[type]++;

	if (ethqos->mac_rec_en[type]) {
		if (ethqos->mac_rec_cnt[type] >
		    ethqos->mac_rec_threshold[type]) {
			ETHQOSERR("exceeded recovery threshold for %s",
				  err_names[type]);
			ethqos->mac_rec_en[type] = false;
			ret = 0;
		} else {
			ethqos->mac_rec_cnt[type]++;
			switch (type) {
			case PHY_RW_ERR:
			{
				if (priv->plat->mac2mac_en)
					return -EOPNOTSUPP;
				ret = ethqos_reset_phy_rec(priv, true);
				if (!ret) {
					ETHQOSERR("recovery failed for %s",
						  err_names[type]);
					ethqos->mac_rec_fail[type] = true;
				}
			}
			break;
			case PHY_DET_ERR:
			{
				if (priv->plat->mac2mac_en)
					return -EOPNOTSUPP;
				ret = ethqos_reset_phy_rec(priv, false);
				if (!ret) {
					ETHQOSERR("recovery failed for %s",
						  err_names[type]);
					ethqos->mac_rec_fail[type] = true;
				}
			}
			break;
			case FBE_ERR:
			{
				ret = ethqos_reset_tx_dma_rec(priv, chan);
				if (!ret) {
					ETHQOSERR("recovery failed for %s",
						  err_names[type]);
					ethqos->mac_rec_fail[type] = true;
				}
			}
			break;
			case RBU_ERR:
			{
				ret = ethqos_schedule_poll(priv, chan);
				if (!ret) {
					ETHQOSERR("recovery failed for %s",
						  err_names[type]);
					ethqos->mac_rec_fail[type] = true;
				}
			}
			break;
			case TDU_ERR:
			{
				if (!ethqos->tdu_scheduled) {
					ethqos->tdu_chan = chan;
					schedule_delayed_work
					(&ethqos->tdu_rec,
					 msecs_to_jiffies(3000));
					ethqos->tdu_scheduled = true;
				}
			}
			break;
			case CRC_ERR:
			case RECEIVE_ERR:
			case OVERFLOW_ERR:
			case DRIBBLE_ERR:
			case WDT_ERR:
			{
				ret = ethqos_writeback_desc_rec(priv, chan);
				if (!ret) {
					ETHQOSERR("recovery failed for %s",
						  err_names[type]);
					ethqos->mac_rec_fail[type] = true;
				}
			}
			break;
			default:
			break;
			}
		}
	}
	mac_rec_wq_flag = true;
	wake_up_interruptible(&mac_rec_wq);

	return ret;
}

static const struct file_operations fops_phy_reg_dump = {
	.read = read_phy_reg_dump,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static const struct file_operations fops_rgmii_reg_dump = {
	.read = read_rgmii_reg_dump,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t write_ipc_stmmac_log_ctxt_low(struct file *file,
					     const char __user *buf,
					     size_t count, loff_t *data)
{
	int tmp = 0;

	if (count > MAX_PROC_SIZE)
		count = MAX_PROC_SIZE;
	if (copy_from_user(tmp_buff, buf, count))
		return -EFAULT;
	if (sscanf(tmp_buff, "%du", &tmp) < 0) {
		pr_err("sscanf failed\n");
	} else {
		if (tmp) {
			if (!ipc_stmmac_log_ctxt_low) {
				ipc_stmmac_log_ctxt_low =
				ipc_log_context_create(IPCLOG_STATE_PAGES,
						       "stmmac_low", 0);
			}
			if (!ipc_stmmac_log_ctxt_low) {
				pr_err("failed to create ipc stmmac low context\n");
				return -EFAULT;
			}
		} else {
			if (ipc_stmmac_log_ctxt_low) {
				ipc_log_context_destroy(
							ipc_stmmac_log_ctxt_low
						       );
		}
			ipc_stmmac_log_ctxt_low = NULL;
		}
	}

	stmmac_enable_ipc_low = tmp;
	return count;
}

static ssize_t read_mac_recovery_enable(struct file *filp, char __user *usr_buf,
					size_t count, loff_t *f_pos)
{
	char *buf;
	unsigned int len = 0, buf_len = 6000;
	ssize_t ret_cnt;

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "PHY_RW_rec", pethqos->mac_rec_en[PHY_RW_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "PHY_DET_rec", pethqos->mac_rec_en[PHY_DET_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "CRC_rec", pethqos->mac_rec_en[CRC_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "RECEIVE_rec", pethqos->mac_rec_en[RECEIVE_ERR]);
		len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "OVERFLOW_rec", pethqos->mac_rec_en[OVERFLOW_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "FBE_rec", pethqos->mac_rec_en[FBE_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "RBU_rec", pethqos->mac_rec_en[RBU_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "TDU_rec", pethqos->mac_rec_en[TDU_ERR]);
		len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "DRIBBLE_rec", pethqos->mac_rec_en[DRIBBLE_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "WDT_rec", pethqos->mac_rec_en[WDT_ERR]);

	if (len > buf_len)
		len = buf_len;

	ETHQOSDBG("%s", buf);
	ret_cnt = simple_read_from_buffer(usr_buf, count, f_pos, buf, len);
	kfree(buf);
	return ret_cnt;
}

static ssize_t ethqos_mac_recovery_enable(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	unsigned char in_buf[15] = {0};
	int i, ret;
	struct qcom_ethqos *ethqos = pethqos;

	if (sizeof(in_buf) < count) {
		ETHQOSERR("emac string is too long - count=%u\n", count);
		return -EFAULT;
	}

	memset(in_buf, 0,  sizeof(in_buf));
	ret = copy_from_user(in_buf, user_buf, count);

	for (i = 0; i < MAC_ERR_CNT; i++) {
		if (in_buf[i] == '1')
			ethqos->mac_rec_en[i] = true;
		else
			ethqos->mac_rec_en[i] = false;
	}
	return count;
}

static ssize_t ethqos_test_mac_recovery(struct file *file,
					const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	unsigned char in_buf[4] = {0};
	int ret, err, chan;
	struct qcom_ethqos *ethqos = pethqos;

	struct stmmac_priv *priv = qcom_ethqos_get_priv(ethqos);

	if (sizeof(in_buf) < count) {
		ETHQOSERR("emac string is too long - count=%u\n", count);
		return -EFAULT;
	}

	memset(in_buf, 0,  sizeof(in_buf));
	ret = copy_from_user(in_buf, user_buf, count);

	err = in_buf[0] - '0';

	chan = in_buf[2] - '0';

	if (priv->plat->handle_mac_err)
		priv->plat->handle_mac_err(priv, err, chan);

	return count;
}

static const struct file_operations fops_phy_off = {
	.read = read_phy_off,
	.write = phy_off_config,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static const struct file_operations fops_mac_rec = {
	.read = read_mac_recovery_enable,
	.write = ethqos_test_mac_recovery,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static const struct file_operations fops_loopback_config = {
	.read = read_loopback_config,
	.write = loopback_handling_config,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static const struct file_operations fops_ipc_stmmac_log_low = {
	.write = write_ipc_stmmac_log_ctxt_low,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static int ethqos_create_debugfs(struct qcom_ethqos        *ethqos)
{
	static struct dentry *phy_reg_dump;
	static struct dentry *rgmii_reg_dump;
	static struct dentry *ipc_stmmac_log_low;
	static struct dentry *phy_off;
	static struct dentry *loopback_enable_mode;
	static struct dentry *mac_rec;
	struct stmmac_priv *priv;

	if (!ethqos) {
		ETHQOSERR("Null Param %s\n", __func__);
		return -ENOMEM;
	}

	priv = qcom_ethqos_get_priv(ethqos);
	ethqos->debugfs_dir = debugfs_create_dir("eth", NULL);

	if (!ethqos->debugfs_dir || IS_ERR(ethqos->debugfs_dir)) {
		ETHQOSERR("Can't create debugfs dir\n");
		return -ENOMEM;
	}

	phy_reg_dump = debugfs_create_file("phy_reg_dump", 0400,
					   ethqos->debugfs_dir, ethqos,
					   &fops_phy_reg_dump);
	if (!phy_reg_dump || IS_ERR(phy_reg_dump)) {
		ETHQOSERR("Can't create phy_dump %d\n", (int)phy_reg_dump);
		goto fail;
	}

	rgmii_reg_dump = debugfs_create_file("rgmii_reg_dump", 0400,
					     ethqos->debugfs_dir, ethqos,
					     &fops_rgmii_reg_dump);
	if (!rgmii_reg_dump || IS_ERR(rgmii_reg_dump)) {
		ETHQOSERR("Can't create rgmii_dump %d\n", (int)rgmii_reg_dump);
		goto fail;
	}

	ipc_stmmac_log_low = debugfs_create_file("ipc_stmmac_log_low", 0220,
						 ethqos->debugfs_dir, ethqos,
						 &fops_ipc_stmmac_log_low);
	if (!ipc_stmmac_log_low || IS_ERR(ipc_stmmac_log_low)) {
		ETHQOSERR("Cannot create debugfs ipc_stmmac_log_low %x\n",
			  ipc_stmmac_log_low);
		goto fail;
	}

	if (!priv->plat->mac2mac_en) {
		phy_off = debugfs_create_file("phy_off", 0400,
					      ethqos->debugfs_dir, ethqos,
					      &fops_phy_off);
		if (!phy_off || IS_ERR(phy_off)) {
			ETHQOSERR("Can't create phy_off %x\n", phy_off);
			goto fail;
		}
	}

	loopback_enable_mode = debugfs_create_file("loopback_enable_mode", 0400,
						   ethqos->debugfs_dir, ethqos,
						   &fops_loopback_config);
	if (!loopback_enable_mode || IS_ERR(loopback_enable_mode)) {
		ETHQOSERR("Can't create loopback_enable_mode %d\n",
			  (int)loopback_enable_mode);
		goto fail;
	}

	mac_rec = debugfs_create_file("test_mac_recovery", 0400,
				      ethqos->debugfs_dir, ethqos,
				      &fops_mac_rec);
	if (!mac_rec || IS_ERR(mac_rec)) {
		ETHQOSERR("Can't create mac_rec directory");
		goto fail;
	}
	return 0;

fail:
	debugfs_remove_recursive(ethqos->debugfs_dir);
	return -ENOMEM;
}

static int ethqos_cleanup_debugfs(struct qcom_ethqos *ethqos)
{
	debugfs_remove_recursive(ethqos->debugfs_dir);
	ethqos->debugfs_dir = NULL;

	ETHQOSDBG("debugfs Deleted Successfully\n");
	return 0;
}

static void ethqos_emac_mem_base(struct qcom_ethqos *ethqos)
{
	struct resource *resource = NULL;
	int ret = 0;

	resource = platform_get_resource(ethqos->pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		ETHQOSERR("get emac-base resource failed\n");
		ret = -ENODEV;
		return;
	}
	ethqos->emac_mem_base = resource->start;
	ethqos->emac_mem_size = resource_size(resource);
}

static void stmmac_emb_smmu_exit(void)
{
	if (stmmac_emb_smmu_ctx.valid) {
		if (stmmac_emb_smmu_ctx.smmu_pdev)
			arm_iommu_detach_device
			(&stmmac_emb_smmu_ctx.smmu_pdev->dev);
		if (stmmac_emb_smmu_ctx.mapping)
			arm_iommu_release_mapping(stmmac_emb_smmu_ctx.mapping);
		stmmac_emb_smmu_ctx.valid = false;
		stmmac_emb_smmu_ctx.mapping = NULL;
		stmmac_emb_smmu_ctx.pdev_master = NULL;
		stmmac_emb_smmu_ctx.smmu_pdev = NULL;
	}
}

static int stmmac_emb_smmu_cb_probe(struct platform_device *pdev)
{
	int result;
	u32 iova_ap_mapping[2];
	struct device *dev = &pdev->dev;
	int atomic_ctx = 1;
	int fast = 1;
	int bypass = 1;
	struct iommu_domain_geometry geometry = {0};

	ETHQOSDBG("EMAC EMB SMMU CB probe: smmu pdev=%px\n", pdev);

	result = of_property_read_u32_array(dev->of_node, "qcom,iova-mapping",
					    iova_ap_mapping, 2);
	if (result) {
		ETHQOSERR("Failed to read EMB start/size iova addresses\n");
		return result;
	}
	stmmac_emb_smmu_ctx.va_start = iova_ap_mapping[0];
	stmmac_emb_smmu_ctx.va_size = iova_ap_mapping[1];
	stmmac_emb_smmu_ctx.va_end = stmmac_emb_smmu_ctx.va_start +
				   stmmac_emb_smmu_ctx.va_size;

	geometry.aperture_start = stmmac_emb_smmu_ctx.va_start;
	geometry.aperture_end =
	stmmac_emb_smmu_ctx.va_start + stmmac_emb_smmu_ctx.va_size;

	stmmac_emb_smmu_ctx.smmu_pdev = pdev;

	if (dma_set_mask(dev, DMA_BIT_MASK(32)) ||
	    dma_set_coherent_mask(dev, DMA_BIT_MASK(32))) {
		ETHQOSERR("DMA set 32bit mask failed\n");
		return -EOPNOTSUPP;
	}

	stmmac_emb_smmu_ctx.mapping = arm_iommu_create_mapping
	(dev->bus, stmmac_emb_smmu_ctx.va_start, stmmac_emb_smmu_ctx.va_size);
	if (IS_ERR_OR_NULL(stmmac_emb_smmu_ctx.mapping)) {
		ETHQOSDBG("Fail to create mapping\n");
		/* assume this failure is because iommu driver is not ready */
		return -EPROBE_DEFER;
	}
	ETHQOSDBG("Successfully Created SMMU mapping\n");
	stmmac_emb_smmu_ctx.valid = true;

	if (of_property_read_bool(dev->of_node, "qcom,smmu-s1-bypass")) {
		if (iommu_domain_set_attr(stmmac_emb_smmu_ctx.mapping->domain,
					  DOMAIN_ATTR_S1_BYPASS,
					  &bypass)) {
			ETHQOSERR("Couldn't set SMMU S1 bypass\n");
			result = -EIO;
			goto err_smmu_probe;
		}
		ETHQOSDBG("SMMU S1 BYPASS set\n");
	} else {
		if (iommu_domain_set_attr(stmmac_emb_smmu_ctx.mapping->domain,
					  DOMAIN_ATTR_ATOMIC,
					  &atomic_ctx)) {
			ETHQOSERR("Couldn't set SMMU domain as atomic\n");
			result = -EIO;
			goto err_smmu_probe;
		}
		ETHQOSDBG("SMMU atomic set\n");
		if (of_property_read_bool(dev->of_node,
					  "qcom,smmu-fastmap")) {
			ETHQOSERR("SMMU-Fastmap device tree entry detected");
			if (iommu_domain_set_attr
			    (stmmac_emb_smmu_ctx.mapping->domain,
			     DOMAIN_ATTR_FAST, &fast)) {
				ETHQOSERR("Couldn't set FAST SMMU\n");
				result = -EIO;
				goto err_smmu_probe;
			}
			ETHQOSDBG("SMMU fast map set\n");
		}
		if (of_property_read_bool(dev->of_node,
					  "qcom,smmu-geometry")) {
			if (iommu_domain_set_attr
			    (stmmac_emb_smmu_ctx.mapping->domain,
			     DOMAIN_ATTR_GEOMETRY,
			     &geometry)) {
				ETHQOSERR("Couldn't set DOMAIN_ATTR_GEOMETRY");
				result = -EIO;
				goto err_smmu_probe;
			}
			ETHQOSDBG("SMMU DOMAIN_ATTR_GEOMETRY set\n");
		}

	}

	result = arm_iommu_attach_device(&stmmac_emb_smmu_ctx.smmu_pdev->dev,
					 stmmac_emb_smmu_ctx.mapping);
	if (result) {
		ETHQOSERR("couldn't attach to IOMMU ret=%d\n", result);
		goto err_smmu_probe;
	}

	stmmac_emb_smmu_ctx.iommu_domain =
		iommu_get_domain_for_dev(&stmmac_emb_smmu_ctx.smmu_pdev->dev);

	ETHQOSDBG("Successfully attached to IOMMU\n");
	if (stmmac_emb_smmu_ctx.pdev_master)
		goto smmu_probe_done;

err_smmu_probe:
	if (stmmac_emb_smmu_ctx.mapping)
		arm_iommu_release_mapping(stmmac_emb_smmu_ctx.mapping);
	stmmac_emb_smmu_ctx.valid = false;

smmu_probe_done:
	stmmac_emb_smmu_ctx.ret = result;
	return result;
}

static int ethqos_update_rgmii_tx_drv_strength(struct qcom_ethqos *ethqos,
					       struct device_node *np)
{
	int ret = 0;
	struct resource *resource = NULL;
	unsigned long tlmm_central_base = 0;
	unsigned long tlmm_central_size = 0;
	unsigned long reg_rgmii_io_pads_voltage = 0;
	u32 tx_drv_str[3];

	resource =
	 platform_get_resource_byname(
	    ethqos->pdev, IORESOURCE_MEM, "tlmm-central-base");

	if (!resource) {
		ETHQOSERR("Resource tlmm-central-base not found\n");
		goto err_out;
	}

	tlmm_central_base = resource->start;
	tlmm_central_size = resource_size(resource);
	ETHQOSDBG("tlmm_central_base = 0x%x, size = 0x%x\n",
		  tlmm_central_base, tlmm_central_size);

	tlmm_central_base_addr = ioremap(
	   tlmm_central_base, tlmm_central_size);
	if (!tlmm_central_base_addr) {
		ETHQOSERR("cannot map dwc_tlmm_central reg memory, aborting\n");
		ret = -EIO;
		goto err_out;
	}

	ETHQOSDBG("dwc_tlmm_central = %#lx\n", tlmm_central_base_addr);

	if (ethqos->emac_ver != EMAC_HW_v2_1_2) {
		reg_rgmii_io_pads_voltage =
		regulator_get_voltage(ethqos->reg_rgmii_io_pads);
	}

	ETHQOSINFO("IOMACRO pads voltage: %u uV\n", reg_rgmii_io_pads_voltage);

	if (np && !of_property_read_u32(np, "rgmii-tx-drv-str-clk",
					&tx_drv_str[0])) {
		switch (tx_drv_str[0]) {
		case 2:
			tx_drv_str[0] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_2MA;
			break;
		case 4:
			tx_drv_str[0] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_4MA;
			break;
		case 6:
			tx_drv_str[0] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_6MA;
			break;
		case 8:
			tx_drv_str[0] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_8MA;
			break;
		case 10:
			tx_drv_str[0] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_10MA;
			break;
		case 12:
			tx_drv_str[0] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_12MA;
			break;
		case 14:
			tx_drv_str[0] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_14MA;
			break;
		case 16:
			tx_drv_str[0] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA;
			break;
		default:
			tx_drv_str[0] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA;
			break;
		}
	} else {
		tx_drv_str[0] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA;
	}

	if (np && !of_property_read_u32(np, "rgmii-tx-drv-str-data",
					&tx_drv_str[1])) {
		switch (tx_drv_str[1]) {
		case 2:
			tx_drv_str[1] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_2MA;
			break;
		case 4:
			tx_drv_str[1] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_4MA;
			break;
		case 6:
			tx_drv_str[1] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_6MA;
			break;
		case 8:
			tx_drv_str[1] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_8MA;
			break;
		case 10:
			tx_drv_str[1] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_10MA;
			break;
		case 12:
			tx_drv_str[1] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_12MA;
			break;
		case 14:
			tx_drv_str[1] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_14MA;
			break;
		case 16:
			tx_drv_str[1] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA;
			break;
		default:
			tx_drv_str[1] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA;
			break;
		}
	} else {
		tx_drv_str[1] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA;
	}

	if (np && !of_property_read_u32(np, "rgmii-tx-drv-str-ctl",
					&tx_drv_str[2])) {
		switch (tx_drv_str[2]) {
		case 2:
			tx_drv_str[2] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_2MA;
			break;
		case 4:
			tx_drv_str[2] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_4MA;
			break;
		case 6:
			tx_drv_str[2] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_6MA;
			break;
		case 8:
			tx_drv_str[2] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_8MA;
			break;
		case 10:
			tx_drv_str[2] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_10MA;
			break;
		case 12:
			tx_drv_str[2] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_12MA;
			break;
		case 14:
			tx_drv_str[2] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_14MA;
			break;
		case 16:
			tx_drv_str[2] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA;
			break;
		default:
			tx_drv_str[2] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA;
			break;
		}
	} else {
		tx_drv_str[2] = TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA;
	}

	switch (reg_rgmii_io_pads_voltage) {
	case 1500000:
	case 1800000: {
		switch (ethqos->emac_ver) {
		case EMAC_HW_v2_0_0:
		case EMAC_HW_v2_2_0:
		case EMAC_HW_v2_3_2: {
				TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_WR(
				   tx_drv_str[0],
				   tx_drv_str[1],
				   tx_drv_str[2]);
				TLMM_RGMII_RX_HV_MODE_CTL_RGWR(0x0);
		}
		break;
		default:
		break;
		}
	}
	break;
	default:
	break;
	}

err_out:
	if (tlmm_central_base_addr)
		iounmap(tlmm_central_base_addr);

	return ret;
}

static void qcom_ethqos_phy_suspend_clks(struct qcom_ethqos *ethqos)
{
	struct stmmac_priv *priv = qcom_ethqos_get_priv(ethqos);

	ETHQOSINFO("Enter\n");

	if (priv->plat->phy_intr_en_extn_stm)
		reinit_completion(&ethqos->clk_enable_done);

	ethqos->clks_suspended = 1;

	ethqos_update_rgmii_clk_and_bus_cfg(ethqos, 0);

	if (ethqos->rgmii_clk)
		clk_disable_unprepare(ethqos->rgmii_clk);

	ETHQOSINFO("Exit\n");
}

inline bool qcom_ethqos_is_phy_link_up(struct qcom_ethqos *ethqos)
{
	/* PHY driver initializes phydev->link=1.
	 * So, phydev->link is 1 even on bootup with no PHY connected.
	 * phydev->link is valid only after adjust_link is called once.
	 */
	struct stmmac_priv *priv = qcom_ethqos_get_priv(ethqos);

	if (priv->plat->mac2mac_en) {
		return priv->plat->mac2mac_link;
	} else {
		return ((priv->oldlink != -1) &&
			(priv->dev->phydev &&
			priv->dev->phydev->link));
	}
}

static void qcom_ethqos_phy_resume_clks(struct qcom_ethqos *ethqos)
{
	struct stmmac_priv *priv = qcom_ethqos_get_priv(ethqos);

	ETHQOSINFO("Enter\n");

	if (ethqos->rgmii_clk)
		clk_prepare_enable(ethqos->rgmii_clk);

	if (qcom_ethqos_is_phy_link_up(ethqos))
		ethqos_update_rgmii_clk_and_bus_cfg(ethqos, ethqos->speed);
	else
		ethqos_update_rgmii_clk_and_bus_cfg(ethqos, SPEED_10);

	ethqos->clks_suspended = 0;

	if (priv->plat->phy_intr_en_extn_stm)
		complete_all(&ethqos->clk_enable_done);

	ETHQOSINFO("Exit\n");
}

static void qcom_ethqos_bringup_iface(struct work_struct *work)
{
	struct platform_device *pdev = NULL;
	struct net_device *ndev = NULL;
	struct qcom_ethqos *ethqos =
		container_of(work, struct qcom_ethqos, early_eth);

	ETHQOSINFO("entry\n");

	if (!ethqos)
		return;

	pdev = ethqos->pdev;
	if (!pdev)
		return;

	ndev = platform_get_drvdata(pdev);

	if (!ndev || netif_running(ndev))
		return;

	rtnl_lock();

	if (dev_change_flags(ndev, ndev->flags | IFF_UP) < 0)
		ETHQOSINFO("ERROR\n");

	rtnl_unlock();

	ETHQOSINFO("exit\n");
}

static void qcom_ethqos_request_phy_wol(void *plat_n)
{
	struct plat_stmmacenet_data *plat = plat_n;
	struct qcom_ethqos *ethqos;
	struct stmmac_priv *priv;
	int ret = 0;

	if (!plat)
		return;

	ethqos = plat->bsp_priv;
	priv = qcom_ethqos_get_priv(ethqos);

	if (!priv || !priv->en_wol)
		return;

	/* Check if phydev is valid*/
	/* Check and enable Wake-on-LAN functionality in PHY*/
	if (priv->phydev) {
		struct ethtool_wolinfo wol = {.cmd = ETHTOOL_GWOL};
		phy_ethtool_get_wol(priv->phydev, &wol);

		wol.cmd = ETHTOOL_SWOL;
		wol.wolopts = wol.supported;
		ret = phy_ethtool_set_wol(priv->phydev, &wol);

		if (ret) {
			ETHQOSERR("set wol in PHY failed\n");
			return;
		}

		if (ret == EOPNOTSUPP) {
			ETHQOSERR("WOL not supported\n");
			return;
		}

		device_set_wakeup_capable(priv->device, 1);

		enable_irq_wake(ethqos->phy_intr);
		device_set_wakeup_enable(&ethqos->pdev->dev, 1);
	}
}

static void ethqos_is_ipv4_NW_stack_ready(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct qcom_ethqos *ethqos;
	struct platform_device *pdev = NULL;
	struct net_device *ndev = NULL;
	int ret;

	ETHQOSDBG("Enter\n");
	dwork = container_of(work, struct delayed_work, work);
	ethqos = container_of(dwork, struct qcom_ethqos, ipv4_addr_assign_wq);

	if (!ethqos)
		return;

	pdev = ethqos->pdev;

	if (!pdev)
		return;

	ndev = platform_get_drvdata(pdev);

	ret = qcom_ethqos_add_ipaddr(&pparams, ndev);
	if (ret)
		return;

	cancel_delayed_work_sync(&ethqos->ipv4_addr_assign_wq);
	flush_delayed_work(&ethqos->ipv4_addr_assign_wq);
}

#ifdef CONFIG_IPV6
static void ethqos_is_ipv6_NW_stack_ready(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct qcom_ethqos *ethqos;
	struct platform_device *pdev = NULL;
	struct net_device *ndev = NULL;
	int ret;

	ETHQOSDBG("Enter\n");
	dwork = container_of(work, struct delayed_work, work);
	ethqos = container_of(dwork, struct qcom_ethqos, ipv6_addr_assign_wq);

	if (!ethqos)
		return;

	pdev = ethqos->pdev;

	if (!pdev)
		return;

	ndev = platform_get_drvdata(pdev);

	ret = qcom_ethqos_add_ipv6addr(&pparams, ndev);
	if (ret)
		return;

	cancel_delayed_work_sync(&ethqos->ipv6_addr_assign_wq);
	flush_delayed_work(&ethqos->ipv6_addr_assign_wq);
}
#endif

static void ethqos_set_early_eth_param(
				struct stmmac_priv *priv,
				struct qcom_ethqos *ethqos)
{
	int ret = 0;

	if (priv->plat && priv->plat->mdio_bus_data)
		priv->plat->mdio_bus_data->phy_mask =
		 priv->plat->mdio_bus_data->phy_mask | DUPLEX_FULL | SPEED_100;

	if (priv->plat)
		priv->plat->max_speed = SPEED_100;

	if (pparams.is_valid_ipv4_addr) {
		INIT_DELAYED_WORK(&ethqos->ipv4_addr_assign_wq,
				  ethqos_is_ipv4_NW_stack_ready);
		schedule_delayed_work(&ethqos->ipv4_addr_assign_wq, 0);
	}

#ifdef CONFIG_IPV6
	if (pparams.is_valid_ipv6_addr) {
		INIT_DELAYED_WORK(&ethqos->ipv6_addr_assign_wq,
				  ethqos_is_ipv6_NW_stack_ready);
		ret = qcom_ethqos_add_ipv6addr(&pparams, priv->dev);
		if (ret)
			schedule_delayed_work(&ethqos->ipv6_addr_assign_wq,
					      msecs_to_jiffies(1000));
	}
#endif
	return;
}

bool qcom_ethqos_ipa_enabled(void)
{
#ifdef CONFIG_ETH_IPA_OFFLOAD
	return pethqos->ipa_enabled;
#endif
	return false;
}

static ssize_t ethqos_read_dev_emac(struct file *filp, char __user *buf,
				    size_t count, loff_t *f_pos)
{
	struct eth_msg_meta msg;
	u8 status = 0;
	struct stmmac_priv *priv;

	memset(&msg, 0,  sizeof(struct eth_msg_meta));
	priv = qcom_ethqos_get_priv(pethqos);

	if (!priv)
		return -ENODEV;

	if (pethqos->ipa_enabled)
		priv->plat->offload_event_handler(
			&status, EV_QTI_GET_CONN_STATUS);

	msg.msg_type = status;

	ETHQOSDBG("status %02x\n", status);
	ETHQOSDBG("msg.msg_type %02x\n", msg.msg_type);
	ETHQOSDBG("msg.rsvd %02x\n", msg.rsvd);
	ETHQOSDBG("msg.msg_len %d\n", msg.msg_len);

	return copy_to_user(buf, &msg, sizeof(struct eth_msg_meta));
}

static ssize_t ethqos_write_dev_emac(struct file *file,
				     const char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	unsigned char in_buf[300] = {0};
	unsigned long ret;
	struct qcom_ethqos *ethqos = pethqos;
	struct stmmac_priv *priv = qcom_ethqos_get_priv(pethqos);
	char mac_str[30] = {0};
	char vlan_str[30] = {0};
	char *prefix = NULL;
	u32 err, prio, queue;
	unsigned int number;

	if (sizeof(in_buf) < count) {
		ETHQOSERR("emac string is too long - count=%u\n", count);
		return -EFAULT;
	}

	memset(in_buf, 0,  sizeof(in_buf));
	ret = copy_from_user(in_buf, user_buf, count);

	if (ret)
		return -EFAULT;

	strlcpy(vlan_str, in_buf, sizeof(vlan_str));

	ETHQOSINFO("emac string is %s\n", vlan_str);

	if (strnstr(vlan_str, "QOE", sizeof(vlan_str))) {
		ethqos->qoe_vlan.available = true;
		queue = ethqos->qoe_vlan.rx_queue;
		prio = priv->plat->rx_queues_cfg[queue].prio;
		/* Convert prio to bit format */
		prio = MAC_RXQCTRL_PSRQX_PRIO_SHIFT(prio);
		priv->hw->mac->rx_queue_prio(priv->hw, prio, queue);
	}

	if (strnstr(vlan_str, "qvlanid=", sizeof(vlan_str))) {
		prefix = strnchr(vlan_str,
				 strlen(vlan_str), '=');
		ETHQOSINFO("vlanid data written is %s\n", prefix + 1);
		if (prefix) {
			err = kstrtouint(prefix + 1, 0, &number);
			if (!err)
				ethqos->qoe_vlan.vlan_id = number;
		}
	}

	if (strnstr(vlan_str, "qvlan_pcp=", strlen(vlan_str))) {
		prefix = strnchr(vlan_str, strlen(vlan_str), '=');
		ETHQOSDBG("QMI vlan_pcp data written is %s\n", prefix + 1);
		if (prefix) {
			err = kstrtouint(prefix + 1, 0, &number);
			if (!err) {
				queue = ethqos->qoe_vlan.rx_queue;
				priv->plat->rx_queues_cfg[queue].prio = number;
			}
		}
	}

	if (strnstr(vlan_str, "Cv2X", strlen(vlan_str))) {
		ETHQOSDBG("Cv2X supported mode is %u\n", ethqos->cv2x_mode);
		ethqos->cv2x_vlan.available = true;
		queue = ethqos->cv2x_vlan.rx_queue;
		prio = priv->plat->rx_queues_cfg[queue].prio;
		/* Convert prio to bit format */
		prio = MAC_RXQCTRL_PSRQX_PRIO_SHIFT(prio);
		priv->hw->mac->rx_queue_prio(priv->hw, prio, queue);
	}

	if (strnstr(vlan_str, "cvlanid=", strlen(vlan_str))) {
		prefix = strnchr(vlan_str, strlen(vlan_str), '=');
		ETHQOSDBG("Cv2X vlanid data written is %s\n", prefix + 1);
		if (prefix) {
			err = kstrtouint(prefix + 1, 0, &number);
			if (!err)
				ethqos->cv2x_vlan.vlan_id = number;
		}
	}

	if (strnstr(vlan_str, "cvlan_pcp=", strlen(vlan_str))) {
		prefix = strnchr(vlan_str, strlen(vlan_str), '=');
		ETHQOSDBG("Cv2X vlan_pcp data written is %s\n", prefix + 1);
		if (prefix) {
			err = kstrtouint(prefix + 1, 0, &number);
			if (!err) {
				queue = ethqos->cv2x_vlan.rx_queue;
				priv->plat->rx_queues_cfg[queue].prio = number;
			}
		}
	}

	if (strnstr(in_buf, "cmac_id=", strlen(in_buf))) {
		prefix = strnchr(in_buf, strlen(in_buf), '=');
		if (prefix) {
			memcpy(mac_str, (char *)prefix + 1, 30);
			mac_str[sizeof(mac_str) - 1] = '\0';

			if (!mac_pton(mac_str, config_dev_addr)) {
				ETHQOSERR("Invalid mac addr in /dev/emac\n");
				return count;
			}

			if (!is_valid_ether_addr(config_dev_addr)) {
				ETHQOSERR("Invalid/Multcast mac addr found\n");
				return count;
			}

			ether_addr_copy(dev_addr, config_dev_addr);
			memcpy(ethqos->cv2x_dev_addr, dev_addr, ETH_ALEN);
		}
	}

	return count;
}

static void ethqos_get_qoe_dt(struct qcom_ethqos *ethqos,
			      struct device_node *np)
{
	int res;

	res = of_property_read_u32(np, "qcom,qoe_mode", &ethqos->qoe_mode);
	if (res) {
		ETHQOSDBG("qoe_mode not in dtsi\n");
		ethqos->qoe_mode = 0;
	}

	if (ethqos->qoe_mode) {
		res = of_property_read_u32(np, "qcom,qoe-queue",
					   &ethqos->qoe_vlan.rx_queue);
		if (res) {
			ETHQOSERR("qoe-queue not in dtsi for qoe_mode %u\n",
				  ethqos->qoe_mode);
			ethqos->qoe_vlan.rx_queue = QMI_TAG_TX_CHANNEL;
		}

		res = of_property_read_u32(np, "qcom,qoe-vlan-offset",
					   &ethqos->qoe_vlan.vlan_offset);
		if (res) {
			ETHQOSERR("qoe-vlan-offset not in dtsi\n");
			ethqos->qoe_vlan.vlan_offset = 0;
		}
	}
}

static DECLARE_WAIT_QUEUE_HEAD(dev_emac_wait);
#ifdef CONFIG_ETH_IPA_OFFLOAD
void ethqos_wakeup_dev_emac_queue(void)
{
	ETHQOSDBG("\n");
	wake_up_interruptible(&dev_emac_wait);
}
#endif

static unsigned int ethqos_poll_dev_emac(struct file *file, poll_table *wait)
{
	int mask = 0;
	int update = 0;
	struct stmmac_priv *priv;

	ETHQOSDBG("\n");

	poll_wait(file, &dev_emac_wait, wait);

	priv = qcom_ethqos_get_priv(pethqos);

	if (pethqos && pethqos->ipa_enabled && pethqos->cv2x_mode)
		priv->plat->offload_event_handler(
			&update, EV_QTI_CHECK_CONN_UPDATE);

	if (update)
		mask = POLLIN | POLLRDNORM;

	ETHQOSDBG("mask %d\n", mask);

	return mask;
}

static const struct file_operations emac_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ethqos_read_dev_emac,
	.write = ethqos_write_dev_emac,
	.poll = ethqos_poll_dev_emac,
};

static int ethqos_create_emac_device_node(dev_t *emac_dev_t,
					  struct cdev **emac_cdev,
					  struct class **emac_class,
					  char *emac_dev_node_name)
{
	int ret;

	ret = alloc_chrdev_region(emac_dev_t, 0, 1,
				  emac_dev_node_name);
	if (ret) {
		ETHQOSERR("alloc_chrdev_region error for node %s\n",
			  emac_dev_node_name);
		goto alloc_chrdev1_region_fail;
	}

	*emac_cdev = cdev_alloc();
	if (!*emac_cdev) {
		ret = -ENOMEM;
		ETHQOSERR("failed to alloc cdev\n");
		goto fail_alloc_cdev;
	}
	cdev_init(*emac_cdev, &emac_fops);

	ret = cdev_add(*emac_cdev, *emac_dev_t, 1);
	if (ret < 0) {
		ETHQOSERR(":cdev_add err=%d\n", -ret);
		goto cdev1_add_fail;
	}

	*emac_class = class_create(THIS_MODULE, emac_dev_node_name);
	if (!*emac_class) {
		ret = -ENODEV;
		ETHQOSERR("failed to create class\n");
		goto fail_create_class;
	}

	if (!device_create(*emac_class, NULL,
			   *emac_dev_t, NULL, emac_dev_node_name)) {
		ret = -EINVAL;
		ETHQOSERR("failed to create device_create\n");
		goto fail_create_device;
	}

	return 0;

fail_create_device:
	class_destroy(*emac_class);
fail_create_class:
	cdev_del(*emac_cdev);
cdev1_add_fail:
fail_alloc_cdev:
	unregister_chrdev_region(*emac_dev_t, 1);
alloc_chrdev1_region_fail:
		return ret;
}

static void ethqos_remove_emac_device_node(struct qcom_ethqos *ethqos)
{
	device_destroy(ethqos->emac_class, ethqos->emac_dev_t);
	class_destroy(ethqos->emac_class);
	cdev_del(ethqos->emac_cdev);
	unregister_chrdev_region(ethqos->emac_dev_t, 1);
}

static void ethqos_remove_emac_rec_device_node(struct qcom_ethqos *ethqos)
{
	device_destroy(ethqos->emac_rec_class, ethqos->emac_rec_dev_t);
	class_destroy(ethqos->emac_rec_class);
	cdev_del(ethqos->emac_rec_cdev);
	unregister_chrdev_region(ethqos->emac_rec_dev_t, 1);
}

static unsigned int ethqos_poll_rec_dev_emac(struct file *file,
					     poll_table *wait)
{
	int mask = 0;

	ETHQOSDBG("\n");

	poll_wait(file, &mac_rec_wq, wait);

	if (mac_rec_wq_flag) {
		mask = POLLIN | POLLRDNORM;
		mac_rec_wq_flag = false;
	}

	ETHQOSDBG("mask %d\n", mask);

	return mask;
}

static ssize_t ethqos_read_rec_dev_emac(struct file *filp, char __user *usr_buf,
					size_t count, loff_t *f_pos)
{
	char *buf;
	unsigned int len = 0, buf_len = 6000;
	ssize_t ret_cnt;

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "PHY_RW_ERR", pethqos->mac_err_cnt[PHY_RW_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "PHY_DET_ERR", pethqos->mac_err_cnt[PHY_DET_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "CRC_ERR", pethqos->mac_err_cnt[CRC_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "RECEIVE_ERR", pethqos->mac_err_cnt[RECEIVE_ERR]);
		len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "OVERFLOW_ERR", pethqos->mac_err_cnt[OVERFLOW_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "FBE_ERR", pethqos->mac_err_cnt[FBE_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "RBU_ERR", pethqos->mac_err_cnt[RBU_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "TDU_ERR", pethqos->mac_err_cnt[TDU_ERR]);
		len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "DRIBBLE_ERR", pethqos->mac_err_cnt[DRIBBLE_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "WDT_ERR", pethqos->mac_err_cnt[WDT_ERR]);

	len += scnprintf(buf + len, buf_len - len, "\n\n%s  =  %d\n",
		 "PHY_RW_EN", pethqos->mac_rec_en[PHY_RW_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "PHY_DET_EN", pethqos->mac_rec_en[PHY_DET_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "CRC_EN", pethqos->mac_rec_en[CRC_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "RECEIVE_EN", pethqos->mac_rec_en[RECEIVE_ERR]);
		len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "OVERFLOW_EN", pethqos->mac_rec_en[OVERFLOW_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "FBE_EN", pethqos->mac_rec_en[FBE_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "RBU_EN", pethqos->mac_rec_en[RBU_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "TDU_EN", pethqos->mac_rec_en[TDU_ERR]);
		len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "DRIBBLE_EN", pethqos->mac_rec_en[DRIBBLE_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "WDT_EN", pethqos->mac_rec_en[WDT_ERR]);

	len += scnprintf(buf + len, buf_len - len, "\n\n%s  =  %d\n",
		 "PHY_RW_REC", pethqos->mac_rec_cnt[PHY_RW_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "PHY_DET_REC", pethqos->mac_rec_cnt[PHY_DET_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "CRC_REC", pethqos->mac_rec_cnt[CRC_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "RECEIVE_REC", pethqos->mac_rec_cnt[RECEIVE_ERR]);
		len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "OVERFLOW_REC", pethqos->mac_rec_cnt[OVERFLOW_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "FBE_REC", pethqos->mac_rec_cnt[FBE_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "RBU_REC", pethqos->mac_rec_cnt[RBU_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "TDU_REC", pethqos->mac_rec_cnt[TDU_ERR]);
		len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "DRIBBLE_REC", pethqos->mac_rec_cnt[DRIBBLE_ERR]);
	len += scnprintf(buf + len, buf_len - len, "%s  =  %d\n",
		 "WDT_REC", pethqos->mac_rec_cnt[WDT_ERR]);

	if (len > buf_len)
		len = buf_len;

	ETHQOSDBG("%s", buf);
	ret_cnt = simple_read_from_buffer(usr_buf, count, f_pos, buf, len);
	kfree(buf);
	return ret_cnt;
}

static const struct file_operations emac_rec_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ethqos_read_rec_dev_emac,
	.write = ethqos_mac_recovery_enable,
	.poll = ethqos_poll_rec_dev_emac,
};

static int ethqos_create_emac_rec_device_node(dev_t *emac_dev_t,
					      struct cdev **emac_cdev,
					      struct class **emac_class,
					      char *emac_dev_node_name)
{
	int ret;

	ret = alloc_chrdev_region(emac_dev_t, 0, 1,
				  emac_dev_node_name);
	if (ret) {
		ETHQOSERR("alloc_chrdev_region error for node %s\n",
			  emac_dev_node_name);
		goto alloc_chrdev1_region_fail;
	}

	*emac_cdev = cdev_alloc();
	if (!*emac_cdev) {
		ret = -ENOMEM;
		ETHQOSERR("failed to alloc cdev\n");
		goto fail_alloc_cdev;
	}
	cdev_init(*emac_cdev, &emac_rec_fops);

	ret = cdev_add(*emac_cdev, *emac_dev_t, 1);
	if (ret < 0) {
		ETHQOSERR(":cdev_add err=%d\n", -ret);
		goto cdev1_add_fail;
	}

	*emac_class = class_create(THIS_MODULE, emac_dev_node_name);
	if (!*emac_class) {
		ret = -ENODEV;
		ETHQOSERR("failed to create class\n");
		goto fail_create_class;
	}

	if (!device_create(*emac_class, NULL,
			   *emac_dev_t, NULL, emac_dev_node_name)) {
		ret = -EINVAL;
		ETHQOSERR("failed to create device_create\n");
		goto fail_create_device;
	}

	return 0;

fail_create_device:
	class_destroy(*emac_class);
fail_create_class:
	cdev_del(*emac_cdev);
cdev1_add_fail:
fail_alloc_cdev:
	unregister_chrdev_region(*emac_dev_t, 1);
alloc_chrdev1_region_fail:
		return ret;
}

static void ethqos_get_cv2x_dt(struct qcom_ethqos *ethqos,
			       struct device_node *np)
{
	int res;

	res = of_property_read_u32(np, "qcom,cv2x_mode", &ethqos->cv2x_mode);
	if (res) {
		ETHQOSDBG("cv2x_mode not in dtsi\n");
		ethqos->cv2x_mode = CV2X_MODE_DISABLE;
	}

	if (ethqos->cv2x_mode != CV2X_MODE_DISABLE) {
		res = of_property_read_u32(np, "qcom,cv2x-queue",
					   &ethqos->cv2x_vlan.rx_queue);
		if (res) {
			ETHQOSERR("cv2x-queue not in dtsi for cv2x_mode %u\n",
				  ethqos->cv2x_mode);
			ethqos->cv2x_vlan.rx_queue = CV2X_TAG_TX_CHANNEL;
		}

		res = of_property_read_u32(np, "qcom,cv2x-vlan-offset",
					   &ethqos->cv2x_vlan.vlan_offset);
		if (res) {
			ETHQOSERR("cv2x-vlan-offset not in dtsi\n");
			ethqos->cv2x_vlan.vlan_offset = 1;
		}
	}
}

inline u32 qcom_ethqos_rgmii_io_macro_num_of_regs(u32 emac_hw_version)
{
	switch (emac_hw_version) {
	case EMAC_HW_v2_0_0:
		return 27;
	case EMAC_HW_v2_1_0:
		return 27;
	case EMAC_HW_v2_1_1:
		return 27;
	case EMAC_HW_v2_1_2:
		return 27;
	case EMAC_HW_v2_2_0:
		return 27;
	case EMAC_HW_v2_3_0:
		return 28;
	case EMAC_HW_v2_3_1:
		return 27;
	case EMAC_HW_v2_3_2:
		return 29;
	case EMAC_HW_NONE:
	default:
		return 0;
	}
}

static int qcom_ethos_init_panic_notifier(struct qcom_ethqos *ethqos)
{
	u32 size_iomacro_regs;
	int ret = 1;

	if (pethqos) {
		size_iomacro_regs =
		qcom_ethqos_rgmii_io_macro_num_of_regs(pethqos->emac_ver) * 4;

		pethqos->emac_reg_base_address =
			kzalloc(pethqos->emac_mem_size, GFP_KERNEL);

		if (!pethqos->emac_reg_base_address)
			ret = 0;

		pethqos->rgmii_reg_base_address =
			kzalloc(size_iomacro_regs, GFP_KERNEL);

		if (!pethqos->rgmii_reg_base_address)
			ret = 0;
	}

	return ret;
}

static int qcom_ethos_panic_notifier(struct notifier_block *this,
				     unsigned long event, void *ptr)
{
	u32 size_iomacro_regs;
	struct stmmac_priv *priv = NULL;

	if (pethqos) {
		priv = qcom_ethqos_get_priv(pethqos);

		pr_info("qcom-ethqos: ethqos 0x%px\n", pethqos);

		pr_info("qcom-ethqos: stmmac_priv 0x%px\n", priv);

		pethqos->iommu_domain =
			priv->plat->stmmac_emb_smmu_ctx.iommu_domain;

		pr_info("qcom-ethqos: emac iommu domain 0x%px\n",
			pethqos->iommu_domain);

		pr_info("qcom-ethqos: emac register mem 0x%px\n",
			pethqos->emac_reg_base_address);
		if (pethqos->emac_reg_base_address)
			memcpy_fromio(pethqos->emac_reg_base_address,
				      pethqos->ioaddr,
				      pethqos->emac_mem_size);

		pr_info("qcom-ethqos: rgmii register mem 0x%px\n",
			pethqos->rgmii_reg_base_address);
		size_iomacro_regs =
		qcom_ethqos_rgmii_io_macro_num_of_regs(pethqos->emac_ver) * 4;
		if (pethqos->rgmii_reg_base_address)
			memcpy_fromio(pethqos->rgmii_reg_base_address,
				      pethqos->rgmii_base,
				      size_iomacro_regs);
	}
	return NOTIFY_DONE;
}

static struct notifier_block qcom_ethqos_panic_blk = {
	.notifier_call  = qcom_ethos_panic_notifier,
};

static void read_mac_addr_from_fuse_reg(struct device_node *np)
{
	int ret, i, count, x;
	u32 mac_efuse_prop, efuse_size = 8;
	unsigned long mac_addr;

	/* If the property doesn't exist or empty return */
	count = of_property_count_u32_elems(np, "mac-efuse-addr");
	if (!count || count < 0)
		return;

	/* Loop over all addresses given until we get valid address */
	for (x = 0; x < count; x++) {
		void __iomem *mac_efuse_addr;

		ret = of_property_read_u32_index(np, "mac-efuse-addr",
						 x, &mac_efuse_prop);
		if (!ret) {
			mac_efuse_addr = ioremap(mac_efuse_prop, efuse_size);
			if (!mac_efuse_addr)
				continue;

			mac_addr = readq(mac_efuse_addr);
			ETHQOSINFO("Mac address read: %llx\n", mac_addr);

			/* create byte array out of value read from efuse */
			for (i = 0; i < ETH_ALEN ; i++) {
				pparams.mac_addr[ETH_ALEN - 1 - i] =
					mac_addr & 0xff;
				mac_addr = mac_addr >> 8;
			}

			iounmap(mac_efuse_addr);

			/* if valid address is found set cookie & return */
			pparams.is_valid_mac_addr =
				is_valid_ether_addr(pparams.mac_addr);
			if (pparams.is_valid_mac_addr)
				return;
		}
	}
}

static void qcom_ethqos_handle_ssr_workqueue(struct work_struct *work)
{
	struct stmmac_priv *priv = NULL;

	priv = qcom_ethqos_get_priv(pethqos);

	ETHQOSDBG("%s is executing action: %d\n", __func__, pethqos->action);

	if (priv->hw_offload_enabled) {
		if (pethqos->action == EVENT_REMOTE_STATUS_DOWN)
			priv->plat->offload_event_handler(NULL,
							  EV_IPA_SSR_DOWN);
		else if (pethqos->action == EVENT_REMOTE_STATUS_UP)
			priv->plat->offload_event_handler(NULL, EV_IPA_SSR_UP);
	}
}

static int qcom_ethqos_qti_alert(struct notifier_block *nb,
				 unsigned long action, void *dev)
{
	struct stmmac_priv *priv = NULL;

	priv = qcom_ethqos_get_priv(pethqos);
	if (!priv) {
		ETHQOSERR("Unable to alert QTI of SSR status: %s\n", __func__);
		return NOTIFY_DONE;
	}

	switch (action) {
	case EVENT_REMOTE_STATUS_UP:
		ETHQOSDBG("Link up\n");
		pethqos->action = EVENT_REMOTE_STATUS_UP;
		break;
	case EVENT_REMOTE_STATUS_DOWN:
		ETHQOSDBG("Link down\n");
		pethqos->action = EVENT_REMOTE_STATUS_DOWN;
		break;
	default:
		ETHQOSERR("Invalid action passed: %s, %d\n", __func__, action);
		return NOTIFY_DONE;
	}

	INIT_WORK(&pethqos->eth_ssr, qcom_ethqos_handle_ssr_workqueue);
	queue_work(system_wq, &pethqos->eth_ssr);
	return NOTIFY_DONE;
}

static void qcom_ethqos_register_listener(void)
{
	int ret;

	ETHQOSDBG("Registering sb notification listener: %s\n", __func__);
	pethqos->qti_nb.notifier_call = qcom_ethqos_qti_alert;
	ret = sb_register_evt_listener(&pethqos->qti_nb);
	if (ret)
		ETHQOSERR("sb_register_evt_listener failed at: %s\n", __func__);
}

static int qcom_ethqos_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *io_macro_node = NULL;
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct qcom_ethqos *ethqos;
	struct resource *res;
	struct net_device *ndev;
	struct stmmac_priv *priv;
	unsigned int rclk_dly_read_ps;
	int ret, i;

	if (of_device_is_compatible(pdev->dev.of_node,
				    "qcom,emac-smmu-embedded"))
		return stmmac_emb_smmu_cb_probe(pdev);

#ifdef CONFIG_MSM_BOOT_TIME_MARKER
	place_marker("M - Ethernet probe start");
#endif

	ipc_stmmac_log_ctxt = ipc_log_context_create(IPCLOG_STATE_PAGES,
						     "emac", 0);
	if (!ipc_stmmac_log_ctxt)
		ETHQOSERR("Error creating logging context for emac\n");
	else
		ETHQOSDBG("IPC logging has been enabled for emac\n");
	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	ethqos = devm_kzalloc(&pdev->dev, sizeof(*ethqos), GFP_KERNEL);
	if (!ethqos) {
		return -ENOMEM;
	}

	ethqos->pdev = pdev;

	ethqos_init_reqgulators(ethqos);
	ethqos_init_gpio(ethqos);

	ethqos_get_qoe_dt(ethqos, np);
	ethqos_get_cv2x_dt(ethqos, np);

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat)) {
		dev_err(&pdev->dev, "dt configuration failed\n");
		return PTR_ERR(plat_dat);
	}

	if (plat_dat->tx_sched_algorithm == MTL_TX_ALGORITHM_WFQ ||
	    plat_dat->tx_sched_algorithm == MTL_TX_ALGORITHM_DWRR) {
		ETHQOSERR("WFO and DWRR TX Algorithm is not supported\n");
		ETHQOSDBG("Set TX Algorithm to default WRR\n");
		plat_dat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;
	}

	if (ethqos->cv2x_mode) {
		if (ethqos->cv2x_vlan.rx_queue >= plat_dat->rx_queues_to_use)
			ethqos->cv2x_vlan.rx_queue = CV2X_TAG_TX_CHANNEL;

		ret = of_property_read_u32(np, "jumbo-mtu",
					   &plat_dat->jumbo_mtu);
		if (!ret) {
			if (plat_dat->jumbo_mtu >
			    MAX_SUPPORTED_JUMBO_MTU) {
				ETHQOSDBG("jumbo mtu %u biger than max val\n",
					  plat_dat->jumbo_mtu);
				ETHQOSDBG("Set it to max supported value %u\n",
					  MAX_SUPPORTED_JUMBO_MTU);
				plat_dat->jumbo_mtu =
					MAX_SUPPORTED_JUMBO_MTU;
			}

			if (plat_dat->jumbo_mtu < MIN_JUMBO_FRAME_SIZE) {
				plat_dat->jumbo_mtu = 0;
			} else {
				/* Store and Forward mode will limit the max
				 * buffer size per rx fifo buffer size
				 * configuration. Use Receive Queue Threshold
				 * Control mode (rtc) for cv2x rx queue to
				 * support the jumbo frame up to 8K.
				 */
				i = ethqos->cv2x_vlan.rx_queue;
				plat_dat->rx_queues_cfg[i].use_rtc = true;
			}
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iomacro");
	ethqos->rgmii_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ethqos->rgmii_base)) {
		dev_err(&pdev->dev, "Can't get rgmii base\n");
		ret = PTR_ERR(ethqos->rgmii_base);
		goto err_mem;
	}

	ethqos->rgmii_clk = devm_clk_get(&pdev->dev, "rgmii");
	if (!ethqos->rgmii_clk) {
		ret = -ENOMEM;
		goto err_mem;
	}

	ret = clk_prepare_enable(ethqos->rgmii_clk);
	if (ret)
		goto err_mem;

	/* Read mac address from fuse register */
	read_mac_addr_from_fuse_reg(np);

	/*Initialize Early ethernet to false*/
	ethqos->early_eth_enabled = false;

	/*Check for valid mac, ip address to enable Early eth*/
	if (pparams.is_valid_mac_addr &&
	    (pparams.is_valid_ipv4_addr || pparams.is_valid_ipv6_addr)) {
		/* For 1000BASE-T mode, auto-negotiation is required and
		 * always used to establish a link.
		 * Configure phy and MAC in 100Mbps mode with autoneg
		 * disable as link up takes more time with autoneg
		 * enabled.
		 */
		ethqos->early_eth_enabled = 1;
		ETHQOSINFO("Early ethernet is enabled\n");
	}

	qcom_ethqos_get_bus_config(pdev);

	if (emac_bus_scale_vec)
		ethqos->bus_scale_vec = emac_bus_scale_vec;

	ethqos->bus_hdl = msm_bus_scale_register_client(ethqos->bus_scale_vec);

	if (!ethqos->bus_hdl) {
		ETHQOSERR("unable to register for bus\n");
		msm_bus_cl_clear_pdata(emac_bus_scale_vec);
	}

	ethqos->rgmii_clk_rate =  RGMII_ID_MODE_10_LOW_SVS_CLK_FREQ;
	ethqos->skip_mdio_vote = true;
	ethqos->speed = MDIO_RD_WR_OPS_CLOCK;
	ethqos_update_rgmii_clk_and_bus_cfg(ethqos, MDIO_RD_WR_OPS_CLOCK);
	ethqos_set_func_clk_en(ethqos);
	if (ethqos->emac_ver == EMAC_HW_v2_0_0)
		ethqos->disable_ctile_pc = 1;

	plat_dat->bsp_priv = ethqos;
	plat_dat->fix_mac_speed = ethqos_fix_mac_speed;
	plat_dat->tx_select_queue = dwmac_qcom_select_queue;
	plat_dat->get_plat_tx_coal_frames =  dwmac_qcom_get_plat_tx_coal_frames;
	plat_dat->has_gmac4 = 1;
	plat_dat->tso_en = of_property_read_bool(np, "snps,tso");
	plat_dat->early_eth = ethqos->early_eth_enabled;
	plat_dat->handle_mac_err = dwmac_qcom_handle_mac_err;
	plat_dat->offload_enabled = qcom_ethqos_ipa_enabled;
	plat_dat->handle_prv_ioctl = ethqos_handle_prv_ioctl;
	plat_dat->request_phy_wol = qcom_ethqos_request_phy_wol;
	plat_dat->phy_intr_enable = ethqos_phy_intr_enable;
	plat_dat->offload_event_handler = ethqos_ipa_offload_event_handler;
	plat_dat->init_pps = ethqos_init_pps;
	plat_dat->update_ahb_clk_cfg = ethqos_update_ahb_clk_cfg;
	plat_dat->rgmii_loopback_cfg = rgmii_loopback_config;
	/* Get rgmii interface speed for mac2c from device tree */
	if (of_property_read_u32(np, "mac2mac-rgmii-speed",
				 &plat_dat->mac2mac_rgmii_speed))
		plat_dat->mac2mac_rgmii_speed = -1;
	else
		ETHQOSINFO("mac2mac rgmii speed = %d\n",
			   plat_dat->mac2mac_rgmii_speed);

	if (of_property_read_bool(pdev->dev.of_node, "qcom,arm-smmu")) {
		stmmac_emb_smmu_ctx.pdev_master = pdev;
		ret = of_platform_populate(pdev->dev.of_node,
					   qcom_ethqos_match, NULL, &pdev->dev);
		if (ret)
			ETHQOSERR("Failed to populate EMAC platform\n");
		if (stmmac_emb_smmu_ctx.ret) {
			ETHQOSERR("smmu probe failed\n");
			of_platform_depopulate(&pdev->dev);
			ret = stmmac_emb_smmu_ctx.ret;
			stmmac_emb_smmu_ctx.ret = 0;
			goto err_clk;
		}
	}

	if (of_property_read_bool(pdev->dev.of_node,
				  "emac-core-version")) {
		/* Read emac core version value from dtsi */
		ret = of_property_read_u32(pdev->dev.of_node,
					   "emac-core-version",
					   &ethqos->emac_ver);
		if (ret) {
			ETHQOSDBG(":resource emac-hw-ver! not in dtsi\n");
			ethqos->emac_ver = EMAC_HW_NONE;
			WARN_ON(1);
		}
	} else {
		ethqos->emac_ver =
		rgmii_readl(ethqos, EMAC_I0_EMAC_CORE_HW_VERSION_RGOFFADDR);
	}
	ETHQOSDBG(": emac_core_version = %d\n", ethqos->emac_ver);

	if (of_property_read_bool(pdev->dev.of_node,
				  "emac-phy-off-suspend")) {
		/* Read emac core version value from dtsi */
		ret = of_property_read_u32(pdev->dev.of_node,
					   "emac-phy-off-suspend",
					   &ethqos->current_phy_mode);
		if (ret) {
			ETHQOSDBG(":resource emac-phy-off-suspend! ");
			ETHQOSDBG("not in dtsi\n");
			ethqos->current_phy_mode = 0;
		}
	}
	ETHQOSINFO("emac-phy-off-suspend = %d\n",
		   ethqos->current_phy_mode);

	/* Initialize io-macro settings to default */
	if (ethqos->emac_ver == EMAC_HW_v2_3_2 ||
	    ethqos->emac_ver == EMAC_HW_v2_1_2) {
		ethqos->io_macro.rx_prog_swap = 1;

		/* Set PRG_RCLK_DLY to 57 for 1.8 ns delay */
		if (ethqos->emac_ver == EMAC_HW_v2_3_2)
			ethqos->io_macro.prg_rclk_dly = 69;
		else if (ethqos->emac_ver == EMAC_HW_v2_1_2)
			ethqos->io_macro.prg_rclk_dly = 52;
		else
			ethqos->io_macro.prg_rclk_dly = 57;

	}

	/* Update io-macro settings from device tree */
	io_macro_node = of_find_node_by_name(pdev->dev.of_node,
					     "io-macro-info");

	if (io_macro_node) {
		if (of_property_read_bool(io_macro_node, "rx-prog-swap"))
			ethqos->io_macro.rx_prog_swap = true;
		if (of_property_read_bool(io_macro_node, "rx-dll-bypass"))
			ethqos->io_macro.rx_dll_bypass = true;
		if (of_property_read_bool(io_macro_node, "clear-cdr-ext-en"))
			ethqos->io_macro.clear_cdt_ext_en = true;
		ret = of_property_read_u32(io_macro_node, "prg-rclk-dly",
					   &rclk_dly_read_ps);
		if (!ret && rclk_dly_read_ps) {
			ethqos->io_macro.prg_rclk_dly =
			(RGMII_PRG_RCLK_CONST * 1000) / rclk_dly_read_ps;
		}
		if (of_property_read_bool(io_macro_node,
					  "sdcc-usr-ctl")) {
			ret = of_property_read_u32(io_macro_node,
						   "sdcc-usr-ctl",
						   &ethqos->io_macro.usr_ctl);
			if (!ret)
				ethqos->io_macro.usr_ctl_set = true;
		}
	}

	ethqos->ioaddr = (&stmmac_res)->addr;
	ethqos_update_rgmii_tx_drv_strength(ethqos, np);
	ethqos_mac_rec_init(ethqos);

	plat_dat->stmmac_emb_smmu_ctx = stmmac_emb_smmu_ctx;
	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_clk;

	rgmii_dump(ethqos);

	if (ethqos->emac_ver == EMAC_HW_v2_3_2) {
		ethqos_pps_irq_config(ethqos);
		create_pps_interrupt_device_node(&ethqos->avb_class_a_dev_t,
						 &ethqos->avb_class_a_cdev,
						 &ethqos->avb_class_a_class,
						 AVB_CLASS_A_POLL_DEV_NODE);

		create_pps_interrupt_device_node(&ethqos->avb_class_b_dev_t,
						 &ethqos->avb_class_b_cdev,
						 &ethqos->avb_class_b_class,
						 AVB_CLASS_B_POLL_DEV_NODE);
	}

	if (ethqos->disable_ctile_pc && !qcom_ethqos_qmp_mailbox_init(ethqos)) {
		INIT_WORK(&ethqos->qmp_mailbox_work,
			  qcom_ethqos_qmp_mailbox_work);
		queue_work(system_wq, &ethqos->qmp_mailbox_work);
	}
	ethqos_emac_mem_base(ethqos);
	pethqos = ethqos;
	ethqos_create_debugfs(ethqos);

	qcom_ethqos_read_iomacro_por_values(ethqos);

	ndev = dev_get_drvdata(&ethqos->pdev->dev);
	priv = netdev_priv(ndev);
	priv->clk_csr = STMMAC_CSR_100_150M;

	/* Read en_wol from device tree */
	priv->en_wol = of_property_read_bool(np, "enable-wol");

	if (pparams.is_valid_mac_addr) {
		ether_addr_copy(dev_addr, pparams.mac_addr);
		memcpy(priv->dev->dev_addr, dev_addr, ETH_ALEN);
	}

	for (i = 0; i < plat_dat->tx_queues_to_use; i++) {
		if (of_property_read_u32(np, "dma-tx-desc-cnt",
					 &priv->tx_queue[i].dma_tx_desc_sz))
			priv->tx_queue[i].dma_tx_desc_sz = DMA_TX_SIZE;

		if (ethqos->cv2x_mode && i == ethqos->cv2x_vlan.rx_queue)
			priv->tx_queue[i].dma_tx_desc_sz = DMA_TX_SIZE_CV2X;

		ETHQOSDBG("TX queue[%u] desc cnt = %u\n",
			  i, priv->tx_queue[i].dma_tx_desc_sz);
	}

	for (i = 0; i < plat_dat->rx_queues_to_use; i++) {
		if (of_property_read_u32(np, "dma-rx-desc-cnt",
					 &priv->rx_queue[i].dma_rx_desc_sz))
			priv->rx_queue[i].dma_rx_desc_sz = DMA_RX_SIZE;

		if (ethqos->cv2x_mode) {
			priv->rx_queue[i].en_fep = true;
			if (i == ethqos->cv2x_vlan.rx_queue) {
				priv->rx_queue[i].dis_mod = true;
				if (plat_dat->jumbo_mtu)
					priv->rx_queue[i].jumbo_en = true;
				priv->rx_queue[i].dma_rx_desc_sz =
					DMA_RX_SIZE_CV2X;
			}
		}

		ETHQOSDBG("RX queue[%u] desc cnt = %u\n",
			  i, priv->rx_queue[i].dma_rx_desc_sz);
	}

	if (ethqos->qoe_mode || ethqos->cv2x_mode) {
		ethqos_create_emac_device_node(&ethqos->emac_dev_t,
					       &ethqos->emac_cdev,
					       &ethqos->emac_class,
					       "emac");
	}

	if (priv->plat->mac2mac_en)
		priv->plat->mac2mac_link = -1;

#ifdef CONFIG_ETH_IPA_OFFLOAD
	priv->plat->offload_event_handler(ethqos, EV_PROBE_INIT);
#endif

	if (pethqos->cv2x_mode != CV2X_MODE_DISABLE)
		qcom_ethqos_register_listener();

	if (ethqos->early_eth_enabled) {
		/* Initialize work*/
		INIT_WORK(&ethqos->early_eth,
			  qcom_ethqos_bringup_iface);
		/* Queue the work*/
		queue_work(system_wq, &ethqos->early_eth);
		/*Set early eth parameters*/
		ethqos_set_early_eth_param(priv, ethqos);
	}

	if (qcom_ethos_init_panic_notifier(ethqos))
		atomic_notifier_chain_register(&panic_notifier_list,
					       &qcom_ethqos_panic_blk);

	ethqos_create_emac_rec_device_node(&ethqos->emac_rec_dev_t,
					   &ethqos->emac_rec_cdev,
					   &ethqos->emac_rec_class,
					   "emac_rec");
#ifdef CONFIG_MSM_BOOT_TIME_MARKER
	place_marker("M - Ethernet probe end");
#endif
	ethqos->skip_mdio_vote = false;
	ethqos->speed = SPEED_10;
	ethqos_update_rgmii_clk_and_bus_cfg(ethqos, SPEED_10);
	return ret;

err_clk:
	clk_disable_unprepare(ethqos->rgmii_clk);

	if (ethqos->bus_hdl)
		msm_bus_scale_unregister_client(ethqos->bus_hdl);

err_mem:
	stmmac_remove_config_dt(pdev, plat_dat);
	ethqos->skip_mdio_vote = false;
	ethqos->speed = SPEED_10;
	ethqos_update_rgmii_clk_and_bus_cfg(ethqos, SPEED_10);
	return ret;
}

static int qcom_ethqos_remove(struct platform_device *pdev)
{
	struct qcom_ethqos *ethqos;
	int ret = 0;
	struct stmmac_priv *priv;

	if (!pdev)
		return -ENODEV;

	if (of_device_is_compatible(pdev->dev.of_node,
				    "qcom,emac-smmu-embedded")) {
		of_platform_depopulate(&pdev->dev);
		return 0;
	}

	ethqos = get_stmmac_bsp_priv(&pdev->dev);

	if (!ethqos)
		return -ENODEV;

	priv = qcom_ethqos_get_priv(pethqos);

	ret = stmmac_pltfr_remove(pdev);
	clk_disable_unprepare(ethqos->rgmii_clk);

	if (ethqos->bus_hdl)
		msm_bus_scale_unregister_client(ethqos->bus_hdl);

	if (priv->plat->phy_intr_en_extn_stm)
		free_irq(ethqos->phy_intr, ethqos);

	if (priv->plat->phy_intr_en_extn_stm)
		cancel_work_sync(&ethqos->emac_phy_work);

	if (ethqos->emac_ver == EMAC_HW_v2_3_2)
		ethqos_remove_pps_dev(ethqos);

	ret = ethqos_cleanup_debugfs(ethqos);
	if (ret)
		ETHQOSERR("debugsfs cleanup failed");

	ethqos_remove_emac_rec_device_node(ethqos);

	if (ethqos->qoe_mode || ethqos->cv2x_mode)
		ethqos_remove_emac_device_node(ethqos);

	ethqos_free_gpios(ethqos);

	stmmac_emb_smmu_exit();
	ethqos_disable_regulators(ethqos);

	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &qcom_ethqos_panic_blk);
	platform_set_drvdata(pdev, NULL);
	of_platform_depopulate(&pdev->dev);

	return ret;
}

static int qcom_ethqos_suspend(struct device *dev)
{
	struct qcom_ethqos *ethqos;
	struct net_device *ndev = NULL;
	int ret;
	int allow_suspend = 1;
	struct stmmac_priv *priv;
	struct plat_stmmacenet_data *plat;

	if (of_device_is_compatible(dev->of_node, "qcom,emac-smmu-embedded")) {
		ETHQOSDBG("smmu return\n");
		return 0;
	}

	ETHQOSINFO("Ethernet Suspend Enter\n");
#ifdef CONFIG_MSM_BOOT_TIME_MARKER
	update_marker("M - Ethernet Suspend start");
#endif

	ethqos = get_stmmac_bsp_priv(dev);
	if (!ethqos)
		return -ENODEV;

	ndev = dev_get_drvdata(dev);
	priv = netdev_priv(ndev);
	plat = priv->plat;

	priv->plat->offload_event_handler(&allow_suspend, EV_DPM_SUSPEND);
	if (!allow_suspend) {
		enable_irq_wake(ndev->irq);
		ETHQOSDBG("Suspend Exit enable IRQ\n");
		return 0;
	}
	if (!ndev || !netif_running(ndev))
		return -EINVAL;
	if (ethqos->current_phy_mode == DISABLE_PHY_AT_SUSPEND_ONLY ||
	    ethqos->current_phy_mode == DISABLE_PHY_SUSPEND_ENABLE_RESUME) {
		/*Backup phy related data*/
		if (priv->phydev->autoneg == AUTONEG_DISABLE) {
			ethqos->backup_autoneg = priv->phydev->autoneg;
			ethqos->backup_bmcr = ethqos_mdio_read(priv,
							       plat->phy_addr,
							       MII_BMCR);
		} else {
			ethqos->backup_autoneg = AUTONEG_ENABLE;
		}
	}
	if (ethqos->current_phy_mode == DISABLE_PHY_AT_SUSPEND_ONLY ||
	    ethqos->current_phy_mode == DISABLE_PHY_SUSPEND_ENABLE_RESUME) {
		if (priv->phydev) {
			if (qcom_ethqos_is_phy_link_up(ethqos)) {
				ETHQOSINFO("Post Link down before PHY off\n");
				netif_carrier_off(ndev);
				phy_mac_interrupt(priv->phydev, LINK_DOWN);
			}
		}
	}
	ret = stmmac_suspend(dev);
	qcom_ethqos_phy_suspend_clks(ethqos);
	if (ethqos->current_phy_mode == DISABLE_PHY_AT_SUSPEND_ONLY ||
	    ethqos->current_phy_mode == DISABLE_PHY_SUSPEND_ENABLE_RESUME) {
		ETHQOSINFO("disable phy at suspend\n");
		ethqos_phy_power_off(ethqos);
	}

#ifdef CONFIG_MSM_BOOT_TIME_MARKER
	update_marker("M - Ethernet Suspend End");
#endif
	ETHQOSINFO("Ethernet Suspend End ret = %d\n", ret);

	return ret;
}

static int qcom_ethqos_resume(struct device *dev)
{
	struct net_device *ndev = NULL;
	struct qcom_ethqos *ethqos;
	int ret;
	struct stmmac_priv *priv;

	if (of_device_is_compatible(dev->of_node, "qcom,emac-smmu-embedded"))
		return 0;

	ETHQOSINFO("Ethernet Resume Enter\n");
#ifdef CONFIG_MSM_BOOT_TIME_MARKER
	update_marker("M - Ethernet Resume start");
#endif

	ethqos = get_stmmac_bsp_priv(dev);

	if (!ethqos)
		return -ENODEV;

	ndev = dev_get_drvdata(dev);
	priv = netdev_priv(ndev);

	if (!ndev || !netif_running(ndev)) {
		ETHQOSERR(" Resume not possible\n");
		return -EINVAL;
	}

	if (!ethqos->clks_suspended) {
		disable_irq_wake(ndev->irq);
		ETHQOSDBG("Resume Exit disable IRQ\n");
		return 0;
	}

	if (ethqos->current_phy_mode == DISABLE_PHY_SUSPEND_ENABLE_RESUME) {
		ETHQOSINFO("enable phy at resume\n");
		ethqos_phy_power_on(ethqos);
	}
	qcom_ethqos_phy_resume_clks(ethqos);

	if (ethqos->current_phy_mode == DISABLE_PHY_SUSPEND_ENABLE_RESUME) {
		ETHQOSINFO("reset phy after clock\n");
		ethqos_reset_phy_enable_interrupt(ethqos);
		if (ethqos->backup_autoneg == AUTONEG_DISABLE) {
			if (priv->phydev) {
				priv->phydev->autoneg = ethqos->backup_autoneg;
				phy_write(priv->phydev,
					  MII_BMCR,
					  ethqos->backup_bmcr);
			} else {
				ETHQOSINFO("Phy dev is NULL\n");
			}
		}
	}

	if (ethqos->phy_state == PHY_IS_OFF) {
		/* Temp Enable LOOPBACK_EN.
		 * TX clock needed for reset As Phy is off
		 */
		rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
			      RGMII_CONFIG_LOOPBACK_EN,
			      RGMII_IO_MACRO_CONFIG);
		ETHQOSINFO("Loopback EN Enabled\n");
	}
	ret = stmmac_resume(dev);
	if (ethqos->phy_state == PHY_IS_OFF) {
		//Disable  LOOPBACK_EN
		rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
			      0, RGMII_IO_MACRO_CONFIG);
		ETHQOSINFO("Loopback EN Disabled\n");
	}
	priv->plat->offload_event_handler(NULL, EV_DPM_RESUME);

#ifdef CONFIG_MSM_BOOT_TIME_MARKER
	update_marker("M - Ethernet Resume End");
#endif
	ETHQOSINFO("Ethernet Resume End ret = %d\n", ret);

	return ret;
}

MODULE_DEVICE_TABLE(of, qcom_ethqos_match);

static const struct dev_pm_ops qcom_ethqos_pm_ops = {
	.suspend = qcom_ethqos_suspend,
	.resume = qcom_ethqos_resume,
};

static struct platform_driver qcom_ethqos_driver = {
	.probe  = qcom_ethqos_probe,
	.remove = qcom_ethqos_remove,
	.driver = {
		.name           = DRV_NAME,
		.pm = &qcom_ethqos_pm_ops,
		.of_match_table = of_match_ptr(qcom_ethqos_match),
	},
};

static int __init qcom_ethqos_init_module(void)
{
	int ret = 0;

	ETHQOSDBG("Enter\n");

	ret = platform_driver_register(&qcom_ethqos_driver);
	if (ret < 0) {
		ETHQOSERR("qcom-ethqos: Driver registration failed");
		return ret;
	}

	ETHQOSDBG("Exit\n");

	return ret;
}

static void __exit qcom_ethqos_exit_module(void)
{
	ETHQOSDBG("Enter\n");

	platform_driver_unregister(&qcom_ethqos_driver);

	if (!ipc_stmmac_log_ctxt)
		ipc_log_context_destroy(ipc_stmmac_log_ctxt);

	if (!ipc_stmmac_log_ctxt_low)
		ipc_log_context_destroy(ipc_stmmac_log_ctxt_low);

	ipc_stmmac_log_ctxt = NULL;
	ipc_stmmac_log_ctxt_low = NULL;
	ETHQOSDBG("Exit\n");
}

/*!
 * \brief Macro to register the driver registration function.
 *
 * \details A module always begin with either the init_module or the function
 * you specify with module_init call. This is the entry function for modules;
 * it tells the kernel what functionality the module provides and sets up the
 * kernel to run the module's functions when they're needed. Once it does this,
 * entry function returns and the module does nothing until the kernel wants
 * to do something with the code that the module provides.
 */

module_init(qcom_ethqos_init_module)

/*!
 * \brief Macro to register the driver un-registration function.
 *
 * \details All modules end by calling either cleanup_module or the function
 * you specify with the module_exit call. This is the exit function for modules;
 * it undoes whatever entry function did. It unregisters the functionality
 * that the entry function registered.
 */

module_exit(qcom_ethqos_exit_module)

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. ETHQOS driver");
MODULE_LICENSE("GPL v2");
