// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/atomic.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/debugfs.h>
#include <linux/in.h>
#include <linux/stddef.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/msm_ipa.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/sched.h>
#include "ipa.h"
#include <linux/random.h>
#include <linux/workqueue.h>
#include <linux/version.h>
#include "ncm_ipa.h"
#include "ipa_common_i.h"
#include "ipa_pm.h"

#define CREATE_TRACE_POINTS
#include "ncm_ipa_trace.h"

#define DRV_NAME "NCM_IPA"
#define DEBUGFS_DIR_NAME "ncm_ipa"
#define DEBUGFS_AGGR_DIR_NAME "ncm_ipa_aggregation"
#define NETDEV_NAME "ncm"
#define IPV4_HDR_NAME "ncm_eth_ipv4"
#define IPV6_HDR_NAME "ncm_eth_ipv6"
#define NCM_HDR_NAME "ncm_hdr"
#define IPA_TO_USB_CLIENT IPA_CLIENT_USB_CONS
#define DEFAULT_OUTSTANDING_HIGH 64
#define DEFAULT_OUTSTANDING_LOW 32
#define DEBUGFS_TEMP_BUF_SIZE 4
#define TX_TIMEOUT (5 * HZ)
#define MIN_TX_ERROR_SLEEP_PERIOD 500
#define DEFAULT_AGGR_TIME_LIMIT 1000 /* 1ms */
#define DEFAULT_AGGR_PKT_LIMIT 0
#define NCM_IPA_DFLT_RT_HDL 0
#define IPA_NCM_IPC_LOG_PAGES 50

#define IPA_NCM_IPC_LOGGING(buf, fmt, args...) \
	do { \
		if (buf) \
			ipc_log_string((buf), fmt, __func__, __LINE__, \
				## args); \
	} while (0)

static void *ipa_ncm_logbuf;

#define NCM_IPA_DEBUG(fmt, args...) \
	do { \
		pr_debug(DRV_NAME " %s:%d " fmt, __func__, __LINE__, ## args);\
		if (ipa_ncm_logbuf) { \
			IPA_NCM_IPC_LOGGING(ipa_ncm_logbuf, \
				DRV_NAME " %s:%d " fmt, ## args); \
		} \
	} while (0)

#define NCM_IPA_DEBUG_XMIT(fmt, args...) \
	pr_debug(DRV_NAME " %s:%d " fmt, __func__, __LINE__, ## args)

#define NCM_IPA_ERROR(fmt, args...) \
	do { \
		pr_err(DRV_NAME "@%s@%d@ctx:%s: "\
			fmt, __func__, __LINE__, current->comm, ## args);\
		if (ipa_ncm_logbuf) { \
			IPA_NCM_IPC_LOGGING(ipa_ncm_logbuf, \
				DRV_NAME " %s:%d " fmt, ## args); \
		} \
	} while (0)

#define NCM_IPA_ERROR_RL(fmt, args...) \
	do { \
		pr_err_ratelimited_ipa(DRV_NAME "@%s@%d@ctx:%s: "\
			fmt, __func__, __LINE__, current->comm, ## args);\
		if (ipa_ncm_logbuf) { \
			IPA_NCM_IPC_LOGGING(ipa_ncm_logbuf, \
				DRV_NAME " %s:%d " fmt, ## args); \
		} \
	} while (0)

#define NULL_CHECK_RETVAL(ptr) \
		do { \
			if (!(ptr)) { \
				NCM_IPA_ERROR("null pointer #ptr\n"); \
				ret = -EINVAL; \
			} \
		} \
		while (0)

#define NCM_HDR_OFST(field) offsetof(struct ncm_pkt_hdr, field)
#define NCM_IPA_LOG_ENTRY() NCM_IPA_DEBUG("begin\n")
#define NCM_IPA_LOG_EXIT()  NCM_IPA_DEBUG("end\n")

#define IPV4_IS_TCP(iph) ((iph)->protocol == IPPROTO_TCP)
#define IPV4_IS_UDP(iph) ((iph)->protocol == IPPROTO_UDP)
#define IPV6_IS_TCP(iph) (((struct ipv6hdr *)iph)->nexthdr == IPPROTO_TCP)
#define IPV6_IS_UDP(iph) (((struct ipv6hdr *)iph)->nexthdr == IPPROTO_UDP)
#define IPV4_DELTA 40
#define IPV6_DELTA 60


/**
 * enum ncm_ipa_state - Driver internal state machine
 *
 * This enum represents the different states the driver can be in.
 * The state machine is used to ensure that the driver's internal state
 * is consistent and to prevent invalid state transitions.
 *
 * The states are:
 * @UNLOADED: The driver is unloaded or not initialized.
 * @INITIALIZED: The driver is initialized and the network device is registered.
 * @CONNECTED: The USB pipes are connected to IPA.
 * @UP: The interface is up, but the pipes are not connected yet.
 * @CONNECTED_AND_UP: The pipes are connected and the interface is up.
 * @INVALID: An invalid state (should not occur).
 */
enum ncm_ipa_state {
	NCM_IPA_UNLOADED          = 0,
	NCM_IPA_INITIALIZED       = 1,
	NCM_IPA_CONNECTED         = 2,
	NCM_IPA_UP                = 3,
	NCM_IPA_CONNECTED_AND_UP  = 4,
	NCM_IPA_INVALID           = 5,
};

/**
 * enum ncm_ipa_operation - Enumerations for API operations
 *
 * This enum provides a set of values that describe the different operations
 * that can be performed by the driver's API. These operations are used as
 * input to the driver's state machine to manage the internal state of the
 * driver.
 *
 * The operations are:
 * @NCM_IPA_INITIALIZE: Initialize the driver.
 * @NCM_IPA_CONNECT: Connect the USB pipes to IPA.
 * @NCM_IPA_OPEN: Open the network interface.
 * @NCM_IPA_STOP: Stop the network interface.
 * @NCM_IPA_DISCONNECT: Disconnect the USB pipes from IPA.
 * @NCM_IPA_CLEANUP: Clean up resources.
 */
enum ncm_ipa_operation {
	NCM_IPA_INITIALIZE,
	NCM_IPA_CONNECT,
	NCM_IPA_OPEN,
	NCM_IPA_STOP,
	NCM_IPA_DISCONNECT,
	NCM_IPA_CLEANUP,
};

#define NCM_IPA_STATE_DEBUG(ctx) \
	NCM_IPA_DEBUG("Driver state: %s\n",\
	ncm_ipa_state_string((ctx)->state))

/**
 * struct ncm_ipa_dev - main driver context parameters
 *
 * This structure holds the main context parameters for the NCM IPA driver.
 * It contains various fields that are used to manage the driver's internal state,
 * configure the network interface, and handle packet transmission and reception.
 *
 * @net: network interface struct implemented by this driver
 * @tx_filter: flag that enable/disable Tx path to continue to IPA
 * @tx_dropped: number of filtered out Tx packets
 * @tx_dump_enable: dump all Tx packets
 * @rx_filter: flag that enable/disable Rx path to continue to IPA
 * @rx_dropped: number of filtered out Rx packets
 * @rx_dump_enable: dump all Rx packets
 * @icmp_filter: allow all ICMP packet to pass through the filters
 * @deaggregation_enable: enable/disable IPA HW deaggregation logic
 * @during_xmit_error: flags that indicate that the driver is in a middle
 *  of error handling in Tx path
 * @directory: holds all debug flags used by the driver to allow cleanup
 *  for driver unload
 * @eth_ipv4_hdr_hdl: saved handle for ipv4 header-insertion table
 * @eth_ipv6_hdr_hdl: saved handle for ipv6 header-insertion table
 * @usb_to_ipa_hdl: save handle for IPA pipe operations
 * @ipa_to_usb_hdl: save handle for IPA pipe operations
 * @outstanding_pkts: number of packets sent to IPA without TX complete ACKed
 * @outstanding_high: number of outstanding packets allowed
 * @outstanding_low: number of outstanding packets which shall cause
 *  to netdev queue start (after stopped due to outstanding_high reached)
 * @error_msec_sleep_time: number of msec for sleeping in case of Tx error
 * @state: current state of the driver
 * @host_ethaddr: holds the tethered PC ethernet address
 * @device_ethaddr: holds the device ethernet address
 * @device_ready_notify: callback supplied by USB core driver
 * This callback shall be called by the Netdev once the Netdev internal
 * state is changed to NCM_IPA_CONNECTED_AND_UP
 * @xmit_error_delayed_work: work item for cases where IPA driver Tx fails
 * @state_lock: used to protect the state variable.
 * @pm_hdl: handle for IPA PM framework
 * @is_vlan_mode: should driver work in vlan mode?
 * @netif_rx_function: holds the correct network stack API, needed for NAPI
 * @is_ulso_mode: indicator for ulso support
 * @ncm_hdr_hdl: hdr handle of ncm header
 */
struct ncm_ipa_dev {
	struct net_device *net;
	bool tx_filter;
	u32 tx_dropped;
	bool tx_dump_enable;
	bool rx_filter;
	u32 rx_dropped;
	bool rx_dump_enable;
	bool icmp_filter;
	bool deaggregation_enable;
	bool during_xmit_error;
	struct dentry *directory;
	u32 eth_ipv4_hdr_hdl;
	u32 eth_ipv6_hdr_hdl;
	u32 usb_to_ipa_hdl;
	u32 ipa_to_usb_hdl;
	atomic_t outstanding_pkts;
	u32 outstanding_high;
	u32 outstanding_low;
	u32 error_msec_sleep_time;
	enum ncm_ipa_state state;
	u8 host_ethaddr[ETH_ALEN];
	u8 device_ethaddr[ETH_ALEN];
	void (*device_ready_notify)(void);
	struct delayed_work xmit_error_delayed_work;
	spinlock_t state_lock; /* Spinlock for the state variable.*/
	u32 pm_hdl;
	bool is_vlan_mode;
	int (*netif_rx_function)(struct sk_buff *skb);
	bool is_ulso_mode;
	u32 ncm_hdr_hdl;
};

/**
 * ncm_pkt_hdr - ncm packet header.
 * ncm16_sig - 4byte ncm signature as a header (0x304D434E).
 */
struct ncm_pkt_hdr {
	u8 ncm16_sig[4];
} __packed;

static int ncm_ipa_open(struct net_device *net);
static void ncm_ipa_packet_receive_notify
	(void *private, enum ipa_dp_evt_type evt, unsigned long data);
static void ncm_ipa_tx_complete_notify
	(void *private, enum ipa_dp_evt_type evt, unsigned long data);

#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
static void ncm_ipa_tx_timeout(struct net_device *net,
	unsigned int txqueue);
#else /* Legacy API. */
static void ncm_ipa_tx_timeout(struct net_device *net);
#endif

static int ncm_ipa_stop(struct net_device *net);
static void ncm_ipa_enable_data_path(struct ncm_ipa_dev *ncm_ipa_ctx);
static void ncm_ipa_xmit_error(struct sk_buff *skb);
static void ncm_ipa_xmit_error_aftercare_wq(struct work_struct *work);
static int ncm_ipa_hdr_cfg(struct ncm_ipa_dev *ncm_ipa_ctx, bool is_hpc);
static int ncm_ipa_hdr_destroy(struct ncm_ipa_dev *ncm_ipa_ctx);
static struct net_device_stats *ncm_ipa_get_stats(struct net_device *net);
static int ncm_ipa_register_properties(char *netdev_name, bool is_vlan_mode);
static int ncm_ipa_deregister_properties(char *netdev_name);
static int ncm_ipa_register_pm_client(struct ncm_ipa_dev *ncm_ipa_ctx);
static int ncm_ipa_deregister_pm_client(struct ncm_ipa_dev *ncm_ipa_ctx);
static bool rx_filter(struct sk_buff *skb);
static bool tx_filter(struct sk_buff *skb);
static netdev_tx_t ncm_ipa_start_xmit
	(struct sk_buff *skb, struct net_device *net);
static int ncm_ipa_debugfs_atomic_open
	(struct inode *inode, struct file *file);
static int ncm_ipa_debugfs_aggr_open
	(struct inode *inode, struct file *file);
static ssize_t ncm_ipa_debugfs_aggr_write
	(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos);
static ssize_t ncm_ipa_debugfs_atomic_read
	(struct file *file,
	char __user *ubuf, size_t count, loff_t *ppos);
static void ncm_ipa_dump_skb(struct sk_buff *skb);
static void ncm_ipa_debugfs_init(struct ncm_ipa_dev *ncm_ipa_ctx);
static void ncm_ipa_debugfs_destroy(struct ncm_ipa_dev *ncm_ipa_ctx);
static int ncm_ipa_ep_registers_cfg
	(u32 usb_to_ipa_hdl,
	u32 ipa_to_usb_hdl, u32 max_xfer_size_bytes_to_dev,
	u32 max_xfer_size_bytes_to_host, u32 mtu,
	bool deaggr_enable,
	bool is_vlan_mode);
static int ncm_ipa_set_device_ethernet_addr
	(struct net_device *net,
	u8 device_ethaddr[]);
static enum ncm_ipa_state ncm_ipa_next_state
	(enum ncm_ipa_state current_state,
	enum ncm_ipa_operation operation);
static const char *ncm_ipa_state_string(enum ncm_ipa_state state);

static struct ncm_ipa_dev *ncm_ipa;

static const struct net_device_ops ncm_ipa_netdev_ops = {
	.ndo_open				= ncm_ipa_open,
	.ndo_stop				= ncm_ipa_stop,
	.ndo_start_xmit			= ncm_ipa_start_xmit,
	.ndo_tx_timeout			= ncm_ipa_tx_timeout,
	.ndo_get_stats			= ncm_ipa_get_stats,
	.ndo_set_mac_address	= eth_mac_addr,
};

static const struct file_operations ncm_ipa_debugfs_atomic_ops = {
	.open = ncm_ipa_debugfs_atomic_open,
	.read = ncm_ipa_debugfs_atomic_read,
};

static const struct file_operations ncm_ipa_aggr_ops = {
		.open = ncm_ipa_debugfs_aggr_open,
		.write = ncm_ipa_debugfs_aggr_write,
};

static struct ipa_ep_cfg ipa_to_usb_ep_cfg = {
	.mode = {
		.mode = IPA_BASIC,
		.dst = IPA_CLIENT_APPS_LAN_CONS,
	},
	.hdr = {
		.hdr_len = sizeof(struct ncm_pkt_hdr),
		.hdr_ofst_metadata_valid = true,
		.hdr_ofst_metadata = 0,
		.hdr_additional_const_len = 0,
		.hdr_ofst_pkt_size_valid = false,
		.hdr_ofst_pkt_size = 0,
		.hdr_a5_mux = false,
		.hdr_remove_additional = false,
		.hdr_metadata_reg_valid = true,
	},
	.hdr_ext = {
		.hdr_pad_to_alignment = 2,
		.hdr_total_len_or_pad_offset = 0,
		.hdr_payload_len_inc_padding = false,
		.hdr_total_len_or_pad = 0,
		.hdr_total_len_or_pad_valid = false,
		.hdr_little_endian = true,
	},
	.aggr = {
		.aggr_en = IPA_ENABLE_AGGR,
		.aggr = 0,
		.aggr_byte_limit = 4,
		.aggr_time_limit = DEFAULT_AGGR_TIME_LIMIT,
		.aggr_pkt_limit = DEFAULT_AGGR_PKT_LIMIT,
	},
	.deaggr = {
		.deaggr_hdr_len = 0,
		.packet_offset_valid = 0,
		.packet_offset_location = 0,
		.max_packet_len = 0,
	},
	.route = {
		.rt_tbl_hdl = NCM_IPA_DFLT_RT_HDL,
	},
	.nat = {
		.nat_en = IPA_SRC_NAT,
	},
};

static struct ipa_ep_cfg usb_to_ipa_ep_cfg_deaggr_dis = {
	.mode = {
		.mode = IPA_BASIC,
		.dst  = IPA_CLIENT_APPS_LAN_CONS,
	},
	.hdr = {
		.hdr_len = ETH_HLEN,
		.hdr_ofst_metadata_valid = false,
		.hdr_ofst_metadata = 0,
		.hdr_additional_const_len = 0,
		.hdr_ofst_pkt_size_valid = false,
		.hdr_ofst_pkt_size = 0,
		.hdr_a5_mux = false,
		.hdr_remove_additional = false,
		.hdr_metadata_reg_valid = false,
	},
	.hdr_ext = {
		.hdr_pad_to_alignment = 0,
		.hdr_total_len_or_pad_offset = 0,
		.hdr_payload_len_inc_padding = false,
		.hdr_total_len_or_pad = IPA_HDR_TOTAL_LEN,
		.hdr_total_len_or_pad_valid = false,
		.hdr_little_endian = false,
	},
	.aggr = {
		.aggr_en = IPA_BYPASS_AGGR,
		.aggr = 0,
		.aggr_byte_limit = 0,
		.aggr_time_limit = 0,
		.aggr_pkt_limit  = 0,
	},
	.deaggr = {
		.deaggr_hdr_len = 0,
		.packet_offset_valid = false,
		.packet_offset_location = 0,
		.max_packet_len = 0,
	},
	.route = {
		.rt_tbl_hdl = NCM_IPA_DFLT_RT_HDL,
	},
	.nat = {
		.nat_en = IPA_BYPASS_NAT,
	},
};

static struct ipa_ep_cfg usb_to_ipa_ep_cfg_deaggr_en = {
	.mode = {
		.mode = IPA_BASIC,
		.dst  = IPA_CLIENT_APPS_LAN_CONS,
	},
	.hdr = {
		.hdr_len = ETH_HLEN,
		.hdr_ofst_metadata_valid = false,
		.hdr_ofst_metadata = 0,
		.hdr_additional_const_len = 0,
		.hdr_ofst_pkt_size_valid = false,
		.hdr_ofst_pkt_size = 0,
		.hdr_a5_mux = false,
		.hdr_remove_additional = false,
		.hdr_metadata_reg_valid = false,
	},
	.hdr_ext = {
		.hdr_pad_to_alignment = 0,
		.hdr_total_len_or_pad_offset = 0,
		.hdr_payload_len_inc_padding = false,
		.hdr_total_len_or_pad = IPA_HDR_TOTAL_LEN,
		.hdr_total_len_or_pad_valid = false,
		.hdr_little_endian = false,
	},
	.aggr = {
		.aggr_en = IPA_ENABLE_DEAGGR,
		.aggr = 0,
		.aggr_byte_limit = 0,
		.aggr_time_limit = 0,
		.aggr_pkt_limit  = 0,
	},
	.deaggr = {
		.deaggr_hdr_len = 0,
		.syspipe_err_detection = false,
		.packet_offset_valid = false,
		.packet_offset_location = 0,
		.mbim_or_ncm_flag = true,
		.ignore_min_pkt_err = false,
		.max_packet_len = 8192, /* Will be overridden*/
	},
	.route = {
		.rt_tbl_hdl = NCM_IPA_DFLT_RT_HDL,
	},
	.nat = {
		.nat_en = IPA_BYPASS_NAT,
	},
};

/**
 * ncm_template_hdr - NCM template structure for NCM_IPA SW insertion
 * ncm16_sig - 4byte ncm signature as a header (0x304D434E).
 */
static struct ncm_pkt_hdr ncm_template_hdr = {
	.ncm16_sig = {0x4E, 0x43, 0x4D, 0x30},
};

static void ncm_ipa_msg_free_cb(void *buff, u32 len, u32 type)
{
	kfree(buff);
}

/**
 * ncm_ipa_init() - create network device and initialize internal
 *  data structures
 * @params: in/out parameters required for initialization,
 *  see "struct ipa_ncm_init_params" for more details
 *
 * Shall be called prior to pipe connection.
 * Detailed description:
 *  - allocate the network device
 *  - set default values for driver internal switches and stash them inside
 *     the netdev private field
 *  - set needed headroom for NCM header
 *  - create debugfs folder and files
 *  - create IPA resource manager client
 *  - set the ethernet address for the netdev to be added on SW Tx path
 *  - add header insertion rules for IPA driver (based on host/device Ethernet
 *     addresses given in input params and on NCM data template struct)
 *  - register tx/rx properties to IPA driver (will be later used
 *    by IPA configuration manager to configure rest of the IPA rules)
 *  - set the carrier state to "off" (until connect is called)
 *  - register the network device
 *  - set the out parameters
 *  - change driver internal state to INITIALIZED
 *
 * Returns negative errno, or zero on success
 */
int ncm_ipa_init(struct ipa_ncm_init_params *params)
{
	int result = 0;
	int ret = 0;
	struct net_device *net;
	struct ncm_ipa_dev *ncm_ipa_ctx;

	NCM_IPA_LOG_ENTRY();
	NCM_IPA_DEBUG("%s initializing\n", DRV_NAME);
	NULL_CHECK_RETVAL(params);
	if (ret)
		return ret;

	NCM_IPA_DEBUG
		("host_ethaddr=%pM, device_ethaddr=%pM\n",
		params->host_ethaddr,
		params->device_ethaddr);

	net = alloc_etherdev(sizeof(struct ncm_ipa_dev));
	if (!net) {
		result = -ENOMEM;
		NCM_IPA_ERROR("Failed to allocate Ethernet device\n");
		goto fail_alloc_etherdev;
	}
	NCM_IPA_DEBUG("Network device was successfully allocated\n");

	ncm_ipa_ctx = netdev_priv(net);
	if (!ncm_ipa_ctx) {
		result = -ENOMEM;
		NCM_IPA_ERROR("Failed to extract netdev priv\n");
		goto fail_netdev_priv;
	}
	memset(ncm_ipa_ctx, 0, sizeof(*ncm_ipa_ctx));
	NCM_IPA_DEBUG("ncm_ipa_ctx (private)=%pK\n", ncm_ipa_ctx);

	spin_lock_init(&ncm_ipa_ctx->state_lock);

	ncm_ipa_ctx->net = net;
	ncm_ipa_ctx->tx_filter = false;
	ncm_ipa_ctx->rx_filter = false;
	ncm_ipa_ctx->icmp_filter = true;
	ncm_ipa_ctx->tx_dropped = 0;
	ncm_ipa_ctx->rx_dropped = 0;
	ncm_ipa_ctx->tx_dump_enable = false;
	ncm_ipa_ctx->rx_dump_enable = false;
	ncm_ipa_ctx->deaggregation_enable = false;
	ncm_ipa_ctx->outstanding_high = DEFAULT_OUTSTANDING_HIGH;
	ncm_ipa_ctx->outstanding_low = DEFAULT_OUTSTANDING_LOW;
	atomic_set(&ncm_ipa_ctx->outstanding_pkts, 0);
	memcpy
		(ncm_ipa_ctx->device_ethaddr, params->device_ethaddr,
		sizeof(ncm_ipa_ctx->device_ethaddr));
	memcpy
		(ncm_ipa_ctx->host_ethaddr, params->host_ethaddr,
		sizeof(ncm_ipa_ctx->host_ethaddr));
	INIT_DELAYED_WORK
		(&ncm_ipa_ctx->xmit_error_delayed_work,
		ncm_ipa_xmit_error_aftercare_wq);
	ncm_ipa_ctx->error_msec_sleep_time =
		MIN_TX_ERROR_SLEEP_PERIOD;
	NCM_IPA_DEBUG("Internal data structures are set\n");

	if (!params->device_ready_notify)
		NCM_IPA_DEBUG("device_ready_notify() was not supplied\n");
	ncm_ipa_ctx->device_ready_notify = params->device_ready_notify;

	snprintf(net->name, sizeof(net->name), "%s%%d", NETDEV_NAME);
	NCM_IPA_DEBUG
		("Setting network interface driver name to: %s\n",
		net->name);

	net->netdev_ops = &ncm_ipa_netdev_ops;
	net->watchdog_timeo = TX_TIMEOUT;

	net->needed_headroom = sizeof(ncm_template_hdr);
	NCM_IPA_DEBUG
		("Needed headroom for NCM header set to %d\n",
		net->needed_headroom);

	ncm_ipa_debugfs_init(ncm_ipa_ctx);

	result = ncm_ipa_set_device_ethernet_addr
		(net, ncm_ipa_ctx->device_ethaddr);
	if (result) {
		NCM_IPA_ERROR("Set device MAC failed\n");
		goto fail_set_device_ethernet;
	}
	NCM_IPA_DEBUG("Device Ethernet address set %pM\n", net->dev_addr);

	if (ipa_is_vlan_mode(IPA_VLAN_IF_NCM,
		&ncm_ipa_ctx->is_vlan_mode)) {
		NCM_IPA_ERROR_RL("couldn't acquire vlan mode, is ipa ready?\n");
		goto fail_get_vlan_mode;
	}
	NCM_IPA_DEBUG("is_vlan_mode %d\n", ncm_ipa_ctx->is_vlan_mode);

	result = ncm_ipa_hdr_cfg(ncm_ipa_ctx, false);
	if (result) {
		NCM_IPA_ERROR("Failed to set ipa hdrs\n");
		goto fail_add_ncm_hdr;
	}
	NCM_IPA_DEBUG("IPA header-insertion configured for NCM\n");

	result = ncm_ipa_register_properties(net->name,
		ncm_ipa_ctx->is_vlan_mode);
	if (result) {
		NCM_IPA_ERROR("fail on properties set\n");
		goto fail_register_tx;
	}
	NCM_IPA_DEBUG("2 TX and 2 RX properties were registered\n");

	netif_carrier_off(net);
	NCM_IPA_DEBUG("set carrier off until pipes are connected\n");

	result = register_netdev(net);
	if (result) {
		NCM_IPA_ERROR("register_netdev failed: %d\n", result);
		goto fail_register_netdev;
	}
	NCM_IPA_DEBUG
		("netdev:%s registration succeeded, index=%d\n",
		net->name, net->ifindex);

	if (ipa_get_lan_rx_napi()) {
		ncm_ipa_ctx->netif_rx_function = netif_receive_skb;
		NCM_IPA_DEBUG("LAN RX NAPI enabled = True");
	} else {
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
		ncm_ipa_ctx->netif_rx_function = netif_rx;
#else
		ncm_ipa_ctx->netif_rx_function = netif_rx_ni;
#endif
		NCM_IPA_DEBUG("LAN RX NAPI enabled = False");
	}

	ncm_ipa = ncm_ipa_ctx;
	params->ipa_rx_notify = ncm_ipa_packet_receive_notify;
	params->ipa_tx_notify = ncm_ipa_tx_complete_notify;
	params->private = ncm_ipa_ctx;
	params->skip_ep_cfg = false;
	ncm_ipa_ctx->state = NCM_IPA_INITIALIZED;
	NCM_IPA_STATE_DEBUG(ncm_ipa_ctx);
	pr_info("NCM_IPA NetDev was initialized\n");

	NCM_IPA_LOG_EXIT();
	return 0;

fail_register_netdev:
	ncm_ipa_deregister_properties(net->name);
fail_register_tx:
	ncm_ipa_hdr_destroy(ncm_ipa_ctx);
fail_add_ncm_hdr:
fail_get_vlan_mode:
fail_set_device_ethernet:
	ncm_ipa_debugfs_destroy(ncm_ipa_ctx);
fail_netdev_priv:
	free_netdev(net);
fail_alloc_etherdev:
	return result;
}
EXPORT_SYMBOL_GPL(ncm_ipa_init);

/**
 * ncm_ipa_pipe_connect_notify() - notify ncm_ipa Netdev that the USB pipes
 *  were connected
 * @usb_to_ipa_hdl: handle from IPA driver client for USB->IPA
 * @ipa_to_usb_hdl: handle from IPA driver client for IPA->USB
 * @private: same value that was set by init(), this parameter holds the
 *  network device pointer.
 * @max_transfer_byte_size: NCM protocol specific, the maximum size that
 *  the host expect
 * @max_packet_number: NCM protocol specific, the maximum packet number
 *  that the host expects
 *
 * Once USB driver finishes the pipe connection between IPA core
 * and USB core this method shall be called in order to
 * allow the driver to complete the data path configurations.
 * Detailed description:
 *  - configure the IPA end-points register
 *  - notify the Linux kernel for "carrier_on"
 *  - change the driver internal state
 *
 *  After this function is done the driver state changes to "Connected"  or
 *  Connected and Up.
 *  This API is expected to be called after initialization() or
 *  after a call to disconnect().
 *
 * Returns negative errno, or zero on success
 */
int ncm_ipa_pipe_connect_notify(
	u32 usb_to_ipa_hdl,
	u32 ipa_to_usb_hdl,
	u32 max_xfer_size_bytes_to_dev,
	u32 max_packet_number_to_dev,
	u32 max_xfer_size_bytes_to_host,
	void *private)
{
	struct ncm_ipa_dev *ncm_ipa_ctx = private;
	int next_state = 0;
	int result = 0;
	int ret = 0;
	unsigned long flags = 0;
	struct ipa_ecm_msg *ncm_msg;
	struct ipa_msg_meta msg_meta;

	NCM_IPA_LOG_ENTRY();

	NULL_CHECK_RETVAL(private);
	if (ret)
		return ret;

	NCM_IPA_DEBUG
		("usb_to_ipa_hdl=%d, ipa_to_usb_hdl=%d, private=0x%pK\n",
		usb_to_ipa_hdl, ipa_to_usb_hdl, private);
	NCM_IPA_DEBUG
		("max_xfer_sz_to_dev=%d, max_pkt_num_to_dev=%d\n",
		max_xfer_size_bytes_to_dev,
		max_packet_number_to_dev);
	NCM_IPA_DEBUG
		("max_xfer_sz_to_host=%d\n",
		max_xfer_size_bytes_to_host);

	spin_lock_irqsave(&ncm_ipa_ctx->state_lock, flags);
	next_state = ncm_ipa_next_state
		(ncm_ipa_ctx->state,
		NCM_IPA_CONNECT);
	if (next_state == NCM_IPA_INVALID) {
		spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);
		NCM_IPA_ERROR("Use init()/disconnect() before connect()\n");
		return -EPERM;
	}
	spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);

	if (usb_to_ipa_hdl >= IPA_CLIENT_MAX) {
		NCM_IPA_ERROR_RL
			("usb_to_ipa_hdl(%d) - not valid ipa handle\n",
			usb_to_ipa_hdl);
		return -EINVAL;
	}
	if (ipa_to_usb_hdl >= IPA_CLIENT_MAX) {
		NCM_IPA_ERROR_RL
			("ipa_to_usb_hdl(%d) - not valid ipa handle\n",
			ipa_to_usb_hdl);
		return -EINVAL;
	}

	result = ncm_ipa_register_pm_client(ncm_ipa_ctx);
	if (result) {
		NCM_IPA_ERROR("Failed to register PM\n");
		goto fail_register_pm;
	}
	NCM_IPA_DEBUG("PM client was registered\n");

	ncm_ipa_ctx->ipa_to_usb_hdl = ipa_to_usb_hdl;
	ncm_ipa_ctx->usb_to_ipa_hdl = usb_to_ipa_hdl;
	if (max_packet_number_to_dev > 1)
		ncm_ipa_ctx->deaggregation_enable = true;
	else
		ncm_ipa_ctx->deaggregation_enable = false;
	result = ncm_ipa_ep_registers_cfg
		(usb_to_ipa_hdl,
		ipa_to_usb_hdl,
		max_xfer_size_bytes_to_dev,
		max_xfer_size_bytes_to_host,
		ncm_ipa_ctx->net->mtu,
		ncm_ipa_ctx->deaggregation_enable,
		ncm_ipa_ctx->is_vlan_mode);
	if (result) {
		NCM_IPA_ERROR("fail on ep cfg\n");
		goto fail;
	}
	NCM_IPA_DEBUG("end-points configured\n");

	netif_stop_queue(ncm_ipa_ctx->net);
	NCM_IPA_DEBUG("netif_stop_queue() was called\n");

	netif_carrier_on(ncm_ipa_ctx->net);
	if (!netif_carrier_ok(ncm_ipa_ctx->net)) {
		NCM_IPA_ERROR_RL("netif_carrier_ok error\n");
		result = -EBUSY;
		goto fail;
	}
	NCM_IPA_DEBUG("netif_carrier_on() was called\n");

	ncm_msg = kzalloc(sizeof(*ncm_msg), GFP_KERNEL);
	if (!ncm_msg) {
		result = -ENOMEM;
		goto fail;
	}

	memset(&msg_meta, 0, sizeof(struct ipa_msg_meta));
	msg_meta.msg_type = ECM_CONNECT;
	msg_meta.msg_len = sizeof(struct ipa_ecm_msg);
	strscpy(ncm_msg->name, ncm_ipa_ctx->net->name,
		IPA_RESOURCE_NAME_MAX);
	ncm_msg->ifindex = ncm_ipa_ctx->net->ifindex;

	result = ipa_send_msg(&msg_meta, ncm_msg, ncm_ipa_msg_free_cb);
	if (result) {
		NCM_IPA_ERROR("Failed to send ECM_CONNECT for ncm\n");
		kfree(ncm_msg);
		goto fail;
	}

	spin_lock_irqsave(&ncm_ipa_ctx->state_lock, flags);
	next_state = ncm_ipa_next_state(ncm_ipa_ctx->state,
					  NCM_IPA_CONNECT);
	if (next_state == NCM_IPA_INVALID) {
		spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);
		NCM_IPA_ERROR_RL("Use init()/disconnect() before connect()\n");
		return -EPERM;
	}
	ncm_ipa_ctx->state = next_state;
	spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);

	NCM_IPA_STATE_DEBUG(ncm_ipa_ctx);

	if (next_state == NCM_IPA_CONNECTED_AND_UP)
		ncm_ipa_enable_data_path(ncm_ipa_ctx);
	else
		NCM_IPA_DEBUG("Queue shall be started after open()\n");

	pr_info("NCM_IPA NetDev pipes were connected\n");

	NCM_IPA_LOG_EXIT();
	return 0;

fail:
	ncm_ipa_deregister_pm_client(ncm_ipa_ctx);
fail_register_pm:
	return result;
}
EXPORT_SYMBOL_GPL(ncm_ipa_pipe_connect_notify);

/**
 * ncm_ipa_open() - notify Linux network stack to start sending packets
 * @net: the network interface supplied by the network stack
 *
 * Linux uses this API to notify the driver that the network interface
 * transitions to the up state.
 * The driver will instruct the Linux network stack to start
 * delivering data packets.
 * The driver internal state shall be changed to Up or Connected and Up
 *
 * Returns negative errno, or zero on success
 */
static int ncm_ipa_open(struct net_device *net)
{
	struct ncm_ipa_dev *ncm_ipa_ctx;
	int next_state = 0;
	unsigned long flags = 0;

	NCM_IPA_LOG_ENTRY();

	ncm_ipa_ctx = netdev_priv(net);

	spin_lock_irqsave(&ncm_ipa_ctx->state_lock, flags);

	next_state = ncm_ipa_next_state(ncm_ipa_ctx->state, NCM_IPA_OPEN);
	if (next_state == NCM_IPA_INVALID) {
		spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);
		NCM_IPA_ERROR_RL("can't bring driver up before initialize\n");
		return -EPERM;
	}

	ncm_ipa_ctx->state = next_state;

	spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);

	NCM_IPA_STATE_DEBUG(ncm_ipa_ctx);

	if (next_state == NCM_IPA_CONNECTED_AND_UP)
		ncm_ipa_enable_data_path(ncm_ipa_ctx);
	else
		NCM_IPA_DEBUG("queue shall be started after connect()\n");

	pr_info("NCM_IPA NetDev was opened\n");

	NCM_IPA_LOG_EXIT();
	return 0;
}

/**
 * ncm_ipa_start_xmit() - send data from APPs to USB core via IPA core
 *  using SW path (Tx data path)
 * Tx path for this Netdev is Apps-processor->IPA->USB
 * @skb: packet received from Linux network stack destined for tethered PC
 * @net: the network device being used to send this packet (ncm0)
 *
 * Several conditions needed in order to send the packet to IPA:
 * - Transmit queue for the network driver is currently
 *   in "started" state
 * - The driver internal state is in Connected and Up state.
 * - Filters Tx switch are turned off
 * - The IPA resource manager state for the driver producer client
 *   is "Granted" which implies that all the resources in the dependency
 *   graph are valid for data flow.
 * - outstanding high boundary was not reached.
 *
 * In case the outstanding packets high boundary is reached, the driver will
 * stop the send queue until enough packets are processed by
 * the IPA core (based on calls to ncm_ipa_tx_complete_notify).
 *
 * In case all of the conditions are met, the network driver shall:
 *  - encapsulate the Ethernet packet with NCM header (REMOTE_NDIS_PACKET_MSG)
 *  - send the packet by using IPA Driver SW path (IP_PACKET_INIT)
 *  - Netdev status fields shall be updated based on the current Tx packet
 *
 * Returns NETDEV_TX_BUSY if retry should be made later,
 * or NETDEV_TX_OK on success.
 */
static netdev_tx_t ncm_ipa_start_xmit(struct sk_buff *skb,
					struct net_device *net)
{
	int ret = 0;
	netdev_tx_t status = NETDEV_TX_BUSY;
	struct ncm_ipa_dev *ncm_ipa_ctx = netdev_priv(net);
	struct ipa_tx_meta meta;

	netif_trans_update(net);

	NCM_IPA_DEBUG_XMIT
		("Tx, len=%d, skb->protocol=%d, outstanding=%d\n",
		skb->len, skb->protocol,
		atomic_read(&ncm_ipa_ctx->outstanding_pkts));

	if (unlikely(netif_queue_stopped(net))) {
		NCM_IPA_ERROR_RL("Interface queue is stopped\n");
		goto out;
	}

	if (unlikely(ncm_ipa_ctx->tx_dump_enable))
		ncm_ipa_dump_skb(skb);

	if (unlikely(ncm_ipa_ctx->state != NCM_IPA_CONNECTED_AND_UP)) {
		NCM_IPA_ERROR_RL("Missing pipe connected and/or iface up\n");
		return NETDEV_TX_BUSY;
	}

	if (unlikely(tx_filter(skb))) {
		dev_kfree_skb_any(skb);
		NCM_IPA_DEBUG("Packet got filtered out on Tx path\n");
		ncm_ipa_ctx->tx_dropped++;
		status = NETDEV_TX_OK;
		goto out;
	}

	ret = ipa_pm_activate(ncm_ipa_ctx->pm_hdl);
	if (ret) {
		NCM_IPA_DEBUG("Failed to activate PM client\n");
		netif_stop_queue(net);
		goto fail_pm_activate;
	}

	if (atomic_read(&ncm_ipa_ctx->outstanding_pkts) >=
				ncm_ipa_ctx->outstanding_high) {
		NCM_IPA_DEBUG("Outstanding high boundary reached (%d)\n",
				ncm_ipa_ctx->outstanding_high);
		netif_stop_queue(net);
		NCM_IPA_DEBUG("Send queue was stopped\n");
		status = NETDEV_TX_BUSY;
		goto out;
	}

	meta.ncm_enable = true;
	trace_ncm_tx_dp(skb->protocol);
	ret = ipa_tx_dp(IPA_TO_USB_CLIENT, skb, &meta);
	if (ret) {
		NCM_IPA_ERROR("ipa transmit failed (%d)\n", ret);
		goto fail_tx_packet;
	}

	atomic_inc(&ncm_ipa_ctx->outstanding_pkts);
	status = NETDEV_TX_OK;
	goto out;

fail_tx_packet:
	ncm_ipa_xmit_error(skb);
out:
	if (atomic_read(&ncm_ipa_ctx->outstanding_pkts) == 0)
		ipa_pm_deferred_deactivate(ncm_ipa_ctx->pm_hdl);
fail_pm_activate:

	NCM_IPA_DEBUG
		("Packet Tx done - %s\n",
		(status == NETDEV_TX_OK) ? "OK" : "FAIL");

	return status;
}

/**
 * ncm_ipa_tx_complete_notify() - notification for Netdev that the
 *  last packet was successfully sent
 * @private: driver context stashed by IPA driver upon pipe connect
 * @evt: event type (expected to be write-done event)
 * @data: data provided with event (this is actually the skb that
 *  holds the sent packet)
 *
 * This function will be called on interrupt bottom halve deferred context.
 * outstanding packets counter shall be decremented.
 * Network stack send queue will be re-started in case low outstanding
 * boundary is reached and queue was stopped before.
 * At the end the skb shall be freed.
 */
static void ncm_ipa_tx_complete_notify(
	void *private,
	enum ipa_dp_evt_type evt,
	unsigned long data)
{
	struct sk_buff *skb = (struct sk_buff *)data;
	struct ncm_ipa_dev *ncm_ipa_ctx = private;
	int ret = 0;

	NULL_CHECK_RETVAL(private);
	if (ret)
		return;

	trace_ncm_status_rcvd(skb->protocol);

	NCM_IPA_DEBUG
		("Tx-complete, len=%d, skb->prot=%d, outstanding=%d\n",
		skb->len, skb->protocol,
		atomic_read(&ncm_ipa_ctx->outstanding_pkts));

	if (unlikely((evt != IPA_WRITE_DONE))) {
		NCM_IPA_ERROR_RL("unsupported event on TX call-back\n");
		return;
	}

	if (unlikely(ncm_ipa_ctx->state != NCM_IPA_CONNECTED_AND_UP)) {
		NCM_IPA_DEBUG
		("dropping Tx-complete pkt, state=%s\n",
		ncm_ipa_state_string(ncm_ipa_ctx->state));
		goto out;
	}

	ncm_ipa_ctx->net->stats.tx_packets++;
	ncm_ipa_ctx->net->stats.tx_bytes += skb->len;

	if (atomic_read(&ncm_ipa_ctx->outstanding_pkts) > 0)
		atomic_dec(&ncm_ipa_ctx->outstanding_pkts);

	if (netif_queue_stopped(ncm_ipa_ctx->net) &&
		netif_carrier_ok(ncm_ipa_ctx->net) &&
		atomic_read(&ncm_ipa_ctx->outstanding_pkts) <
					(ncm_ipa_ctx->outstanding_low)) {
		NCM_IPA_DEBUG("Outstanding low boundary reached (%d)n",
				ncm_ipa_ctx->outstanding_low);
		netif_wake_queue(ncm_ipa_ctx->net);
		NCM_IPA_DEBUG("Send queue was awaken\n");
	}

	/*Release resource only when outstanding packets are zero*/
	if (atomic_read(&ncm_ipa_ctx->outstanding_pkts) == 0)
		ipa_pm_deferred_deactivate(ncm_ipa_ctx->pm_hdl);

out:
	dev_kfree_skb_any(skb);
}

#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
static void ncm_ipa_tx_timeout(struct net_device *net,
	unsigned int txqueue)
#else /* Legacy API. */
static void ncm_ipa_tx_timeout(struct net_device *net)
#endif
{
	struct ncm_ipa_dev *ncm_ipa_ctx = netdev_priv(net);
	int outstanding = atomic_read(&ncm_ipa_ctx->outstanding_pkts);

	NCM_IPA_ERROR
		("Possible IPA stall was detected, %d outstanding\n",
		outstanding);

	net->stats.tx_errors++;
}

/**
 * ncm_ipa_packet_receive_notify() - Rx notify for packet sent from
 *  tethered PC (USB->IPA).
 *  is USB->IPA->Apps-processor
 * @private: driver context
 * @evt: event type
 * @data: data provided with event
 *
 * Once IPA driver receives a packet from USB client this callback will be
 * called from bottom-half interrupt handling context (ipa Rx workqueue).
 *
 * Packets that shall be sent to Apps processor may be of two types:
 * 1) Packets that are destined for Apps (e.g: WEBSERVER running on Apps)
 * 2) Exception packets that need special handling (based on IPA core
 *    configuration, e.g: new TCP session or any other packets that IPA core
 *    can't handle)
 * If the next conditions are met, the packet shall be sent up to the
 * Linux network stack:
 *  - Driver internal state is Connected and Up
 *  - Notification received from IPA driver meets the expected type
 *    for Rx packet
 *  -Filters Rx switch are turned off
 *
 * Prior to the sending to the network stack:
 *  - Netdev struct shall be stashed to the skb as required by the network stack
 *  - Ethernet header shall be removed (skb->data shall point to the Ethernet
 *     payload, Ethernet still stashed under MAC header).
 *  - The skb->pkt_protocol shall be set based on the ethernet destination
 *     address, Can be Broadcast, Multicast or Other-Host, The later
 *     pkt-types packets shall be dropped in case the Netdev is not
 *     in  promisc mode.
 *   - Set the skb protocol field based on the EtherType field
 *
 * Netdev status fields shall be updated based on the current Rx packet
 */
static void ncm_ipa_packet_receive_notify(
		void *private,
		enum ipa_dp_evt_type evt,
		unsigned long data)
{
	struct sk_buff *skb = (struct sk_buff *)data;
	struct ncm_ipa_dev *ncm_ipa_ctx = private;
	unsigned int packet_len = skb->len;

	NCM_IPA_DEBUG
		("packet Rx, len=%d\n",
		skb->len);

	if (unlikely(ncm_ipa_ctx == NULL)) {
		NCM_IPA_DEBUG("Private context is NULL. Drop SKB.\n");
		dev_kfree_skb_any(skb);
		return;
	}

	if (unlikely(ncm_ipa_ctx->rx_dump_enable))
		ncm_ipa_dump_skb(skb);

	if (unlikely(ncm_ipa_ctx->state != NCM_IPA_CONNECTED_AND_UP)) {
		NCM_IPA_DEBUG("Use connect()/up() before receive()\n");
		NCM_IPA_DEBUG("Packet dropped (length=%d)\n",
				skb->len);
		ncm_ipa_ctx->rx_dropped++;
		dev_kfree_skb_any(skb);
		return;
	}

	if (evt != IPA_RECEIVE)	{
		NCM_IPA_ERROR_RL("A non IPA_RECEIVE event received in driver RX\n");
		ncm_ipa_ctx->rx_dropped++;
		dev_kfree_skb_any(skb);
		return;
	}

	if (!ncm_ipa_ctx->deaggregation_enable)
		skb_pull(skb, sizeof(struct ncm_pkt_hdr));

	skb->dev = ncm_ipa_ctx->net;
	skb->protocol = eth_type_trans(skb, ncm_ipa_ctx->net);

	if (rx_filter(skb)) {
		NCM_IPA_DEBUG("Packet got filtered out on RX path\n");
		ncm_ipa_ctx->rx_dropped++;
		dev_kfree_skb_any(skb);
		return;
	}

	trace_ncm_netif_ni(skb->protocol);
	ncm_ipa_ctx->netif_rx_function(skb);

	ncm_ipa_ctx->net->stats.rx_packets++;
	ncm_ipa_ctx->net->stats.rx_bytes += packet_len;
}

/** ncm_ipa_stop() - notify the network interface to stop
 *   sending/receiving data
 *  @net: the network device being stopped.
 *
 * This API is used by Linux network stack to notify the network driver that
 * its state was changed to "down"
 * The driver will stop the "send" queue and change its internal
 * state to "Connected".
 * The Netdev shall be returned to be "Up" after ncm_ipa_open().
 */
static int ncm_ipa_stop(struct net_device *net)
{
	struct ncm_ipa_dev *ncm_ipa_ctx = netdev_priv(net);
	int next_state = 0;
	unsigned long flags = 0;

	NCM_IPA_LOG_ENTRY();

	spin_lock_irqsave(&ncm_ipa_ctx->state_lock, flags);

	next_state = ncm_ipa_next_state(ncm_ipa_ctx->state, NCM_IPA_STOP);
	if (next_state == NCM_IPA_INVALID) {
		spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);
		NCM_IPA_DEBUG("can't do network interface down without up\n");
		return -EPERM;
	}

	ncm_ipa_ctx->state = next_state;

	spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);

	netif_stop_queue(net);
	pr_info("NCM_IPA NetDev queue is stopped\n");

	NCM_IPA_STATE_DEBUG(ncm_ipa_ctx);

	NCM_IPA_LOG_EXIT();
	return 0;
}

/** ncm_ipa_disconnect() - notify ncm_ipa Netdev that the USB pipes
 *   were disconnected
 * @private: same value that was set by init(), this  parameter holds the
 *  network device pointer.
 *
 * USB shall notify the Netdev after disconnecting the pipe.
 * - The internal driver state shall returned to its previous
 *   state (Up or Initialized).
 * - Linux network stack shall be informed for carrier off to notify
 *   user space for pipe disconnect
 * - send queue shall be stopped
 * During the transition between the pipe disconnection to
 * the Netdev notification packets
 * are expected to be dropped by IPA driver or IPA core.
 */
int ncm_ipa_pipe_disconnect_notify(void *private)
{
	struct ncm_ipa_dev *ncm_ipa_ctx = private;
	int next_state = 0;
	int outstanding_dropped_pkts = 0;
	int retval = 0;
	int ret = 0;
	unsigned long flags = 0;
	struct ipa_ecm_msg *ncm_msg;
	struct ipa_msg_meta msg_meta;

	NCM_IPA_LOG_ENTRY();

	NULL_CHECK_RETVAL(ncm_ipa_ctx);
	if (ret)
		return ret;
	NCM_IPA_DEBUG("private=0x%pK\n", private);

	spin_lock_irqsave(&ncm_ipa_ctx->state_lock, flags);

	next_state = ncm_ipa_next_state
		(ncm_ipa_ctx->state,
		NCM_IPA_DISCONNECT);
	if (next_state == NCM_IPA_INVALID) {
		spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);
		NCM_IPA_ERROR_RL("can't disconnect before connect\n");
		return -EPERM;
	}
	spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);

	if (ncm_ipa_ctx->during_xmit_error) {
		NCM_IPA_DEBUG("canceling xmit-error delayed work\n");
		cancel_delayed_work_sync(
			&ncm_ipa_ctx->xmit_error_delayed_work);
		ncm_ipa_ctx->during_xmit_error = false;
	}

	netif_carrier_off(ncm_ipa_ctx->net);
	NCM_IPA_DEBUG("carrier_off notification was sent\n");

	ncm_msg = kzalloc(sizeof(*ncm_msg), GFP_KERNEL);
	if (!ncm_msg)
		return -ENOMEM;

	memset(&msg_meta, 0, sizeof(struct ipa_msg_meta));
	msg_meta.msg_type = ECM_DISCONNECT;
	msg_meta.msg_len = sizeof(struct ipa_ecm_msg);
	strscpy(ncm_msg->name, ncm_ipa_ctx->net->name,
		IPA_RESOURCE_NAME_MAX);
	ncm_msg->ifindex = ncm_ipa_ctx->net->ifindex;

	retval = ipa_send_msg(&msg_meta, ncm_msg, ncm_ipa_msg_free_cb);
	if (retval) {
		NCM_IPA_ERROR("fail to send ECM_DISCONNECT for ncm\n");
		kfree(ncm_msg);
		return -EPERM;
	}

	netif_stop_queue(ncm_ipa_ctx->net);
	NCM_IPA_DEBUG("Queue stopped\n");

	outstanding_dropped_pkts =
		atomic_read(&ncm_ipa_ctx->outstanding_pkts);

	ncm_ipa_ctx->net->stats.tx_dropped += outstanding_dropped_pkts;
	atomic_set(&ncm_ipa_ctx->outstanding_pkts, 0);

	retval = ncm_ipa_deregister_pm_client(ncm_ipa_ctx);
	if (retval) {
		NCM_IPA_ERROR("Failed to deregister PM client\n");
		return retval;
	}
	NCM_IPA_DEBUG("PM client was successfully deregistered\n");

	spin_lock_irqsave(&ncm_ipa_ctx->state_lock, flags);
	next_state = ncm_ipa_next_state(ncm_ipa_ctx->state,
					  NCM_IPA_DISCONNECT);
	if (next_state == NCM_IPA_INVALID) {
		spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);
		NCM_IPA_ERROR_RL("Can't disconnect before connect\n");
		return -EPERM;
	}
	ncm_ipa_ctx->state = next_state;
	spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);

	NCM_IPA_STATE_DEBUG(ncm_ipa_ctx);

	pr_info("NCM_IPA NetDev pipes disconnected (%d outstanding clr)\n",
		outstanding_dropped_pkts);

	NCM_IPA_LOG_EXIT();
	return 0;
}
EXPORT_SYMBOL_GPL(ncm_ipa_pipe_disconnect_notify);

/**
 * ncm_ipa_cleanup() - unregister the network interface driver and free
 *  internal data structs.
 * @private: same value that was set by init(), this
 *   parameter holds the network device pointer.
 *
 * This function shall be called once the network interface is not
 * needed anymore, e.g: when the USB composition does not support it.
 * This function shall be called after the pipes were disconnected.
 * Detailed description:
 *  - remove header-insertion headers from IPA core
 *  - delete the driver dependency defined for IPA resource manager and
 *   destroy the producer resource.
 *  -  remove the debugfs entries
 *  - deregister the network interface from Linux network stack
 *  - free all internal data structs
 *
 * It is assumed that no packets shall be sent through HW bridging
 * during cleanup to avoid packets trying to add an header that is
 * removed during cleanup (IPA configuration manager should have
 * removed them at this point)
 */
void ncm_ipa_cleanup(void *private)
{
	struct ncm_ipa_dev *ncm_ipa_ctx = private;
	int next_state = 0;
	int ret = 0;
	unsigned long flags = 0;

	NCM_IPA_LOG_ENTRY();

	NCM_IPA_DEBUG("private=0x%pK\n", private);

	NULL_CHECK_RETVAL(ncm_ipa_ctx);
	if (ret)
		return;

	spin_lock_irqsave(&ncm_ipa_ctx->state_lock, flags);
	next_state = ncm_ipa_next_state
		(ncm_ipa_ctx->state,
		NCM_IPA_CLEANUP);
	if (next_state == NCM_IPA_INVALID) {
		spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);
		NCM_IPA_ERROR_RL("Use disconnect()before clean()\n");
		return;
	}
	spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);

	NCM_IPA_STATE_DEBUG(ncm_ipa_ctx);

	ret = ncm_ipa_deregister_properties(ncm_ipa_ctx->net->name);
	if (ret) {
		NCM_IPA_ERROR("Failed to deregister Tx/Rx properties\n");
		return;
	}
	NCM_IPA_DEBUG("Deregister Tx/Rx properties was successful\n");

	ret = ncm_ipa_hdr_destroy(ncm_ipa_ctx);
	if (ret)
		NCM_IPA_ERROR(
			"Failed removing NCM headers from IPA core. Continue anyway\n");
	else
		NCM_IPA_DEBUG("NCM headers were removed from IPA core\n");

	ncm_ipa_debugfs_destroy(ncm_ipa_ctx);
	NCM_IPA_DEBUG("Debugfs remove was done\n");

	unregister_netdev(ncm_ipa_ctx->net);
	NCM_IPA_DEBUG("netdev unregistered\n");

	spin_lock_irqsave(&ncm_ipa_ctx->state_lock, flags);
	next_state = ncm_ipa_next_state(ncm_ipa_ctx->state,
					  NCM_IPA_CLEANUP);
	if (next_state == NCM_IPA_INVALID) {
		spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);
		NCM_IPA_ERROR_RL("use disconnect()before clean()\n");
		return;
	}
	ncm_ipa_ctx->state = next_state;
	spin_unlock_irqrestore(&ncm_ipa_ctx->state_lock, flags);
	free_netdev(ncm_ipa_ctx->net);
	pr_info("NCM_IPA NetDev was cleaned\n");

	NCM_IPA_LOG_EXIT();
}
EXPORT_SYMBOL_GPL(ncm_ipa_cleanup);

/**
 * ncm_ipa_enable_data_path() - Enable the data path for the NCM IPA driver.
 * @ncm_ipa_ctx: The NCM IPA driver context.
 *
 * This function is called when the driver's internal state is changed to
 * Connected and Up. It notifies the USB device that it is ready to send and
 * receive data, and starts the network queue.
 *
 * Returns: None
 */
static void ncm_ipa_enable_data_path(struct ncm_ipa_dev *ncm_ipa_ctx)
{
	if (ncm_ipa_ctx->device_ready_notify) {
		ncm_ipa_ctx->device_ready_notify();
		NCM_IPA_DEBUG("USB device_ready_notify() was called\n");
	} else {
		NCM_IPA_DEBUG("device_ready_notify() not supplied\n");
	}

	netif_start_queue(ncm_ipa_ctx->net);
	NCM_IPA_DEBUG("netif_start_queue() was called\n");
}

/**
 * ncm_ipa_xmit_error() - Handle transmit error for the NCM IPA driver.
 * @skb: The socket buffer that caused the transmit error.
 *
 * This function is called when a transmit error occurs. It stops the transmit
 * queue, pulls the NCM header from the socket buffer, increments the transmit
 * error count, and schedules a delayed work to restart the transmit queue
 * after a certain delay.
 *
 * Returns: None
 */
static void ncm_ipa_xmit_error(struct sk_buff *skb)
{
	bool retval = false;
	struct ncm_ipa_dev *ncm_ipa_ctx = netdev_priv(skb->dev);
	unsigned long delay_jiffies;
	u8 rand_dealy_msec;

	NCM_IPA_LOG_ENTRY();

	NCM_IPA_DEBUG("Starting Tx-queue backoff\n");

	netif_stop_queue(ncm_ipa_ctx->net);
	NCM_IPA_DEBUG("netif_stop_queue was called\n");

	ncm_ipa_ctx->net->stats.tx_errors++;

	get_random_bytes(&rand_dealy_msec, sizeof(rand_dealy_msec));
	delay_jiffies = msecs_to_jiffies(
		ncm_ipa_ctx->error_msec_sleep_time + rand_dealy_msec);

	retval = schedule_delayed_work(
		&ncm_ipa_ctx->xmit_error_delayed_work, delay_jiffies);
	if (!retval) {
		NCM_IPA_ERROR("fail to schedule delayed work\n");
		netif_start_queue(ncm_ipa_ctx->net);
	} else {
		NCM_IPA_DEBUG
			("work scheduled to start Tx-queue in %d msec\n",
			ncm_ipa_ctx->error_msec_sleep_time +
			rand_dealy_msec);
		ncm_ipa_ctx->during_xmit_error = true;
	}

	NCM_IPA_LOG_EXIT();
}

static void ncm_ipa_xmit_error_aftercare_wq(struct work_struct *work)
{
	struct ncm_ipa_dev *ncm_ipa_ctx;
	struct delayed_work *delayed_work;

	NCM_IPA_LOG_ENTRY();

	NCM_IPA_DEBUG("Starting queue after xmit error\n");

	delayed_work = to_delayed_work(work);
	ncm_ipa_ctx = container_of
		(delayed_work, struct ncm_ipa_dev,
		xmit_error_delayed_work);

	if (unlikely(ncm_ipa_ctx->state != NCM_IPA_CONNECTED_AND_UP)) {
		NCM_IPA_ERROR_RL
			("error aftercare handling in bad state (%d)",
			ncm_ipa_ctx->state);
		return;
	}

	ncm_ipa_ctx->during_xmit_error = false;

	netif_start_queue(ncm_ipa_ctx->net);
	NCM_IPA_DEBUG("netif_start_queue() was called\n");

	NCM_IPA_LOG_EXIT();
}

/**
 * ncm_ipa_hdr_cfg() - configure header insertion in IPA core
 * @ncm_ipa_ctx: main driver context
 *
 * This function adds headers that are used by the hpc header insertion
 * mechanism.
 *
 * Returns negative errno, or zero on success
 */
static int ncm_ipa_hdr_cfg(struct ncm_ipa_dev *ncm_ipa_ctx, bool is_hpc)
{
	struct ipa_ioc_add_hdr *hdrs;
	struct ipa_hdr_add *ncm_hdr;
	struct ipa_pkt_init_ex_hdr_ofst_set lookup;
	int result = 0;

	NCM_IPA_LOG_ENTRY();

	hdrs = kzalloc(sizeof(*hdrs) + sizeof(*ncm_hdr), GFP_KERNEL);
	if (!hdrs) {
		result = -ENOMEM;
		goto fail_mem;
	}
	ncm_hdr = &hdrs->hdr[0];
	strscpy(ncm_hdr->name, NCM_HDR_NAME, sizeof(ncm_hdr->name));
	memcpy(ncm_hdr->hdr, &ncm_template_hdr, sizeof(ncm_template_hdr));
	ncm_hdr->hdr_len = sizeof(ncm_template_hdr);
	ncm_hdr->hdr_hdl = -1;
	ncm_hdr->is_partial = false;
	ncm_hdr->status = -1;
	hdrs->num_hdrs = 1;
	hdrs->commit = 1;
	NCM_IPA_DEBUG("is hpc %d\n", is_hpc);
	if (is_hpc)
		result = ipa3_add_hdr_hpc(hdrs);
	else
		result = ipa_add_hdr(hdrs);

	if (result) {
		NCM_IPA_ERROR("Fail on Header-Insertion(%d)\n", result);
		goto fail_add_hdr;
	}
	if (ncm_hdr->status) {
		NCM_IPA_ERROR("Fail on Header-Insertion ncm(%d)\n",
			ncm_hdr->status);
		result = ncm_hdr->status;
		goto fail_add_hdr;
	}

	ncm_ipa_ctx->ncm_hdr_hdl = ncm_hdr->hdr_hdl;
	lookup.ep = IPA_TO_USB_CLIENT;
	strscpy(lookup.name, NCM_HDR_NAME, sizeof(lookup.name));
	if (ipa_set_pkt_init_ex_hdr_ofst(&lookup, is_hpc))
		goto fail_add_hdr;

	NCM_IPA_LOG_EXIT();

fail_add_hdr:
	kfree(hdrs);
fail_mem:
	return result;
}


/**
 * ncm_ipa_hdr_destroy() - remove the IPA core configuration done for
 *  the driver data path bridging.
 * @ncm_ipa_ctx: the driver context
 *
 *  Revert the work done on ncm_ipa_hdrs_cfg(), which is,
 * remove 2 headers for Ethernet+NCM.
 */
static int ncm_ipa_hdr_destroy(struct ncm_ipa_dev *ncm_ipa_ctx)
{
	struct ipa_ioc_del_hdr *del_wrapper;
	struct ipa_hdr_del *hdr_del;
	int result = 0;

	del_wrapper = kzalloc(sizeof(*del_wrapper) + sizeof(*hdr_del), GFP_KERNEL);
	if (!del_wrapper)
		return -ENOMEM;

	del_wrapper->commit = 1;
	del_wrapper->num_hdls = 1;
	hdr_del = &del_wrapper->hdl[0];
	hdr_del->hdl = ncm_ipa_ctx->ncm_hdr_hdl;

	result = ipa_del_hdr(del_wrapper);
	if (result || hdr_del->status)
		NCM_IPA_ERROR("ipa_del_hdr failed\n");
	kfree(del_wrapper);

	return result;
}

static struct net_device_stats *ncm_ipa_get_stats(struct net_device *net)
{
	return &net->stats;
}

/**
 * ncm_ipa_register_properties() - set Tx/Rx properties needed
 *  by IPA configuration manager
 * @netdev_name: a string with the name of the network interface device
 * @is_vlan_mode: should driver work in vlan mode?
 *
 * Register Tx/Rx properties to allow user space configuration (IPA
 * Configuration Manager):
 *
 * - Two Tx properties (IPA->USB): specify the header names and pipe number
 *   that shall be used by user space for header-addition configuration
 *   for ipv4/ipv6 packets flowing from IPA to USB for HW bridging data.
 *   That header-addition header is added by the Netdev and used by user
 *   space to close the HW bridge by adding filtering and routing rules
 *   that point to this header.
 *
 * - Two Rx properties (USB->IPA): these properties shall be used by user space
 *   to configure the IPA core to identify the packets destined
 *   for Apps-processor by configuring the unicast rules destined for
 *   the Netdev IP address.
 *   This rules shall be added based on the attribute mask supplied at
 *   this function, that is, always hit rule.
 */
static int ncm_ipa_register_properties(char *netdev_name, bool is_vlan_mode)
{
	struct ipa_tx_intf tx_properties = {0};
	struct ipa_ioc_tx_intf_prop properties[2] = { {0}, {0} };
	struct ipa_ioc_tx_intf_prop *ipv4_property;
	struct ipa_ioc_tx_intf_prop *ipv6_property;
	struct ipa_ioc_rx_intf_prop rx_ioc_properties[2] = { {0}, {0} };
	struct ipa_rx_intf rx_properties = {0};
	struct ipa_ioc_rx_intf_prop *rx_ipv4_property;
	struct ipa_ioc_rx_intf_prop *rx_ipv6_property;
	enum ipa_hdr_l2_type hdr_l2_type = IPA_HDR_L2_ETHERNET_II;
	int result = 0;

	NCM_IPA_LOG_ENTRY();

	if (is_vlan_mode)
		hdr_l2_type = IPA_HDR_L2_802_1Q;

	tx_properties.prop = properties;
	ipv4_property = &tx_properties.prop[0];
	ipv4_property->ip = IPA_IP_v4;
	ipv4_property->dst_pipe = IPA_TO_USB_CLIENT;
	strscpy
		(ipv4_property->hdr_name, IPV4_HDR_NAME,
		IPA_RESOURCE_NAME_MAX);
	ipv4_property->hdr_l2_type = hdr_l2_type;
	ipv6_property = &tx_properties.prop[1];
	ipv6_property->ip = IPA_IP_v6;
	ipv6_property->dst_pipe = IPA_TO_USB_CLIENT;
	strscpy
		(ipv6_property->hdr_name, IPV6_HDR_NAME,
		IPA_RESOURCE_NAME_MAX);
	ipv6_property->hdr_l2_type = hdr_l2_type;
	tx_properties.num_props = 2;

	rx_properties.prop = rx_ioc_properties;
	rx_ipv4_property = &rx_properties.prop[0];
	rx_ipv4_property->ip = IPA_IP_v4;
	rx_ipv4_property->attrib.attrib_mask = 0;
	rx_ipv4_property->src_pipe = IPA_CLIENT_USB_PROD;
	rx_ipv4_property->hdr_l2_type = hdr_l2_type;
	rx_ipv6_property = &rx_properties.prop[1];
	rx_ipv6_property->ip = IPA_IP_v6;
	rx_ipv6_property->attrib.attrib_mask = 0;
	rx_ipv6_property->src_pipe = IPA_CLIENT_USB_PROD;
	rx_ipv6_property->hdr_l2_type = hdr_l2_type;
	rx_properties.num_props = 2;

	result = ipa_register_intf("ncm0", &tx_properties, &rx_properties);
	if (result)
		NCM_IPA_ERROR("Failed during Tx/Rx properties registration\n");
	else
		NCM_IPA_DEBUG("Tx/Rx properties registration done\n");

	NCM_IPA_LOG_EXIT();

	return result;
}

/**
 * ncm_ipa_deregister_properties() - remove the 2 Tx and 2 Rx properties
 * @netdev_name: a string with the name of the network interface device
 *
 * This function revert the work done on ncm_ipa_register_properties().
 */
static int  ncm_ipa_deregister_properties(char *netdev_name)
{
	int result = 0;

	NCM_IPA_LOG_ENTRY();

	result = ipa_deregister_intf(netdev_name);
	if (result) {
		NCM_IPA_DEBUG("Failed during Tx prop deregistration\n");
		return result;
	}
	NCM_IPA_LOG_EXIT();

	return 0;
}

/**
 * ncm_ipa_pm_cb() - Power management callback function for the NCM IPA driver.
 * @p: Pointer to the NCM IPA driver context.
 * @event: The power management event that occurred.
 *
 * This function is called by the IPA power management framework to notify the
 * NCM IPA driver of power management events. It is responsible for handling
 * the event and taking any necessary actions.
 *
 * Returns: None
 */
static void ncm_ipa_pm_cb(void *p, enum ipa_pm_cb_event event)
{
	struct ncm_ipa_dev *ncm_ipa_ctx = p;

	NCM_IPA_LOG_ENTRY();

	if (event != IPA_PM_CLIENT_ACTIVATED) {
		NCM_IPA_ERROR_RL("unexpected event %d\n", event);
		WARN_ON(1);
		return;
	}
	NCM_IPA_DEBUG("Resource Granted\n");

	if (netif_queue_stopped(ncm_ipa_ctx->net)) {
		NCM_IPA_DEBUG("Starting queue\n");
		netif_start_queue(ncm_ipa_ctx->net);
	} else {
		NCM_IPA_DEBUG("Queue already awake\n");
	}

	NCM_IPA_LOG_EXIT();
}

/**
 * ncm_ipa_register_pm_client() - Register the NCM IPA driver as a PM client.
 * @ncm_ipa_ctx: The NCM IPA driver context.
 *
 * This function registers the NCM IPA driver as a power management client with
 * the IPA power management framework. It provides a callback function to handle
 * power management events and sets up the client's parameters.
 *
 * Returns: 0 on success, negative error code on failure.
 */
static int ncm_ipa_register_pm_client(struct ncm_ipa_dev *ncm_ipa_ctx)
{
	int result = 0;
	struct ipa_pm_register_params pm_reg;

	memset(&pm_reg, 0, sizeof(pm_reg));

	pm_reg.name = ncm_ipa_ctx->net->name;
	pm_reg.user_data = ncm_ipa_ctx;
	pm_reg.callback = ncm_ipa_pm_cb;
	pm_reg.group = IPA_PM_GROUP_APPS;
	result = ipa_pm_register(&pm_reg, &ncm_ipa_ctx->pm_hdl);
	if (result) {
		NCM_IPA_ERROR("Failed to create IPA PM client %d\n", result);
		return result;
	}
	return 0;
}

/**
 * ncm_ipa_deregister_pm_client() - Deregister the NCM IPA driver as a PM client.
 * @ncm_ipa_ctx: The NCM IPA driver context.
 *
 * This function deregisters the NCM IPA driver as a power management client with
 * the IPA power management framework. It is responsible for releasing any resources
 * allocated during registration and ensuring a clean shutdown.
 *
 * Returns: 0 on success.
 */
static int ncm_ipa_deregister_pm_client(struct ncm_ipa_dev *ncm_ipa_ctx)
{
	ipa_pm_deactivate_sync(ncm_ipa_ctx->pm_hdl);
	ipa_pm_deregister(ncm_ipa_ctx->pm_hdl);
	ncm_ipa_ctx->pm_hdl = ~0;
	return 0;
}

/**
 * rx_filter() - logic that decide if the current skb is to be filtered out
 * @skb: skb that may be sent up to the network stack
 *
 * This function shall do Rx packet filtering on the Netdev level.
 */
static bool rx_filter(struct sk_buff *skb)
{
	struct ncm_ipa_dev *ncm_ipa_ctx = netdev_priv(skb->dev);

	return ncm_ipa_ctx->rx_filter;
}

/**
 * tx_filter() - logic that decide if the current skb is to be filtered out
 * @skb: skb that may be sent to the USB core
 *
 * This function shall do Tx packet filtering on the Netdev level.
 * ICMP filter bypass is possible to allow only ICMP packet to be
 * sent (pings and etc)
 */

static bool tx_filter(struct sk_buff *skb)
{
	struct ncm_ipa_dev *ncm_ipa_ctx = netdev_priv(skb->dev);
	bool is_icmp;

	if (likely(!ncm_ipa_ctx->tx_filter))
		return false;

	is_icmp = (skb->protocol == htons(ETH_P_IP)	&&
		ip_hdr(skb)->protocol == IPPROTO_ICMP);

	if ((!ncm_ipa_ctx->icmp_filter) && is_icmp)
		return false;

	return true;
}


/**
 * ncm_ipa_ep_registers_cfg() - configure the USB endpoints
 * @usb_to_ipa_hdl: handle received from ipa_connect which represents
 *  the USB to IPA end-point
 * @ipa_to_usb_hdl: handle received from ipa_connect which represents
 *  the IPA to USB end-point
 * @max_xfer_size_bytes_to_dev: the maximum size, in bytes, that the device
 *  expects to receive from the host. supplied on REMOTE_NDIS_INITIALIZE_CMPLT.
 * @max_xfer_size_bytes_to_host: the maximum size, in bytes, that the host
 *  expects to receive from the device. supplied on REMOTE_NDIS_INITIALIZE_MSG.
 * @mtu: the netdev MTU size, in bytes
 * @deaggr_enable: should deaggregation be enabled?
 * @is_vlan_mode: should driver work in vlan mode?
 *
 * USB to IPA pipe:
 *  - de-aggregation
 *  - Remove Ethernet header
 *  - Remove NCM header
 *  - SRC NAT
 *  - Default routing(0)
 * IPA to USB Pipe:
 *  - aggregation
 *  - Add Ethernet header
 *  - Add NCM header
 */
static int ncm_ipa_ep_registers_cfg(
	u32 usb_to_ipa_hdl,
	u32 ipa_to_usb_hdl,
	u32 max_xfer_size_bytes_to_dev,
	u32 max_xfer_size_bytes_to_host,
	u32 mtu,
	bool deaggr_enable,
	bool is_vlan_mode)
{
	int result;
	struct ipa_ep_cfg *usb_to_ipa_ep_cfg;
	int add = sizeof(struct ncm_pkt_hdr);

	if (deaggr_enable) {
		usb_to_ipa_ep_cfg = &usb_to_ipa_ep_cfg_deaggr_en;
		NCM_IPA_DEBUG("deaggregation enabled\n");
	} else {
		usb_to_ipa_ep_cfg = &usb_to_ipa_ep_cfg_deaggr_dis;
		NCM_IPA_DEBUG("deaggregation disabled\n");
		add = sizeof(struct ncm_pkt_hdr);
	}

	if (is_vlan_mode) {
		usb_to_ipa_ep_cfg->hdr.hdr_len = VLAN_ETH_HLEN;
		ipa_to_usb_ep_cfg.hdr.hdr_len = add;
	} else {
		usb_to_ipa_ep_cfg->hdr.hdr_len = ETH_HLEN;
		ipa_to_usb_ep_cfg.hdr.hdr_len = add;
	}

	usb_to_ipa_ep_cfg->deaggr.max_packet_len = max_xfer_size_bytes_to_dev;

	result = ipa3_cfg_ep(usb_to_ipa_hdl, usb_to_ipa_ep_cfg);
	if (result) {
		pr_err("failed to configure USB to IPA point\n");
		return result;
	}
	NCM_IPA_DEBUG("IPA<-USB end-point configured\n");

	ipa_to_usb_ep_cfg.aggr.aggr_byte_limit =
		(max_xfer_size_bytes_to_host - mtu) / 1024;

	if (ipa_to_usb_ep_cfg.aggr.aggr_byte_limit == 0) {
		ipa_to_usb_ep_cfg.aggr.aggr_time_limit = 0;
		ipa_to_usb_ep_cfg.aggr.aggr_pkt_limit = 1;
	} else {
		ipa_to_usb_ep_cfg.aggr.aggr_time_limit =
			DEFAULT_AGGR_TIME_LIMIT;
		ipa_to_usb_ep_cfg.aggr.aggr_pkt_limit = DEFAULT_AGGR_PKT_LIMIT;
	}

	NCM_IPA_DEBUG(
		"NCM aggregation param: en=%d byte_limit=%d time_limit=%d pkt_limit=%d\n"
		, ipa_to_usb_ep_cfg.aggr.aggr_en,
		ipa_to_usb_ep_cfg.aggr.aggr_byte_limit,
		ipa_to_usb_ep_cfg.aggr.aggr_time_limit,
		ipa_to_usb_ep_cfg.aggr.aggr_pkt_limit);

	/* enable hdr_metadata_reg_valid */
	usb_to_ipa_ep_cfg->hdr.hdr_metadata_reg_valid = true;

	/*xlat config in vlan mode */
	if (is_vlan_mode) {
		usb_to_ipa_ep_cfg->hdr.hdr_ofst_metadata_valid = 1;
		usb_to_ipa_ep_cfg->hdr.hdr_ofst_metadata =
			sizeof(struct ncm_pkt_hdr) + ETH_HLEN;
		usb_to_ipa_ep_cfg->hdr.hdr_metadata_reg_valid = false;
	}

	result = ipa3_cfg_ep(ipa_to_usb_hdl, &ipa_to_usb_ep_cfg);
	if (result) {
		pr_err("Failed to configure IPA to USB end-point\n");
		return result;
	}

	NCM_IPA_DEBUG("IPA->USB end-point configured\n");
	return 0;
}

/**
 * ncm_ipa_set_device_ethernet_addr() - set device Ethernet address
 * @dev_ethaddr: device Ethernet address
 *
 * Returns 0 for success, negative otherwise
 */
static int ncm_ipa_set_device_ethernet_addr(
	struct net_device *net,
	u8 device_ethaddr[])
{
	if (!is_valid_ether_addr(device_ethaddr))
		return -EINVAL;

#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
	net->addr_len = ETH_ALEN;
	dev_addr_set(net, device_ethaddr);
#else
	memcpy((u8 *)net->dev_addr, device_ethaddr, ETH_ALEN);
#endif

	return 0;
}

/** ncm_ipa_next_state - return the next state of the driver
 * @current_state: the current state of the driver
 * @operation: an enum which represent the operation being made on the driver
 *  by its API.
 *
 * This function implements the driver internal state machine.
 * Its decisions are based on the driver current state and the operation
 * being made.
 * In case the operation is invalid this state machine will return
 * the value NCM_IPA_INVALID to inform the caller for a forbidden sequence.
 */
static enum ncm_ipa_state ncm_ipa_next_state(
		enum ncm_ipa_state current_state,
		enum ncm_ipa_operation operation)
{
	int next_state = NCM_IPA_INVALID;

	switch (current_state) {
	case NCM_IPA_UNLOADED:
		if (operation == NCM_IPA_INITIALIZE)
			next_state = NCM_IPA_INITIALIZED;
		break;
	case NCM_IPA_INITIALIZED:
		if (operation == NCM_IPA_CONNECT)
			next_state = NCM_IPA_CONNECTED;
		else if (operation == NCM_IPA_OPEN)
			next_state = NCM_IPA_UP;
		else if (operation == NCM_IPA_CLEANUP)
			next_state = NCM_IPA_UNLOADED;
		break;
	case NCM_IPA_CONNECTED:
		if (operation == NCM_IPA_DISCONNECT)
			next_state = NCM_IPA_INITIALIZED;
		else if (operation == NCM_IPA_OPEN)
			next_state = NCM_IPA_CONNECTED_AND_UP;
		break;
	case NCM_IPA_UP:
		if (operation == NCM_IPA_STOP)
			next_state = NCM_IPA_INITIALIZED;
		else if (operation == NCM_IPA_CONNECT)
			next_state = NCM_IPA_CONNECTED_AND_UP;
		else if (operation == NCM_IPA_CLEANUP)
			next_state = NCM_IPA_UNLOADED;
		break;
	case NCM_IPA_CONNECTED_AND_UP:
		if (operation == NCM_IPA_STOP)
			next_state = NCM_IPA_CONNECTED;
		else if (operation == NCM_IPA_DISCONNECT)
			next_state = NCM_IPA_UP;
		break;
	default:
		NCM_IPA_ERROR_RL("State is not supported\n");
		break;
	}

	NCM_IPA_DEBUG
		("State transition ( %s -> %s )- %s\n",
		ncm_ipa_state_string(current_state),
		ncm_ipa_state_string(next_state),
		next_state == NCM_IPA_INVALID ?
		"Forbidden" : "Allowed");

	return next_state;
}

/**
 * ncm_ipa_state_string - return the state string representation
 * @state: enum which describe the state
 */
static const char *ncm_ipa_state_string(enum ncm_ipa_state state)
{
	switch (state) {
	case NCM_IPA_UNLOADED:
		return "NCM_IPA_UNLOADED";
	case NCM_IPA_INITIALIZED:
		return "NCM_IPA_INITIALIZED";
	case NCM_IPA_CONNECTED:
		return "NCM_IPA_CONNECTED";
	case NCM_IPA_UP:
		return "NCM_IPA_UP";
	case NCM_IPA_CONNECTED_AND_UP:
		return "NCM_IPA_CONNECTED_AND_UP";
	default:
		return "Not supported";
	}
}

static void ncm_ipa_dump_skb(struct sk_buff *skb)
{
	int i;
	u32 *cur = (u32 *)skb->data;
	u8 *byte;

	NCM_IPA_DEBUG
		("packet dump start for skb->len=%d\n",
		skb->len);

	for (i = 0; i < (skb->len / 4); i++) {
		byte = (u8 *)(cur + i);
		pr_info
			("%2d %08x   %02x %02x %02x %02x\n",
			i, *(cur + i),
			byte[0], byte[1], byte[2], byte[3]);
	}
	NCM_IPA_DEBUG
		("packet dump ended for skb->len=%d\n", skb->len);
}

#ifdef CONFIG_DEBUG_FS
/**
 * Creates the root folder for the driver
 */
static void ncm_ipa_debugfs_init(struct ncm_ipa_dev *ncm_ipa_ctx)
{
	const mode_t flags_read_write = 0666;
	const mode_t flags_read_only = 0444;
	const mode_t  flags_write_only = 0222;
	struct dentry *file;
	struct dentry *aggr_directory;

	NCM_IPA_LOG_ENTRY();

	if (!ncm_ipa_ctx)
		return;

	ncm_ipa_ctx->directory = debugfs_create_dir(DEBUGFS_DIR_NAME, NULL);
	if (!ncm_ipa_ctx->directory) {
		NCM_IPA_ERROR("could not create debugfs directory entry\n");
		goto fail_directory;
	}

	debugfs_create_bool
		("tx_filter", flags_read_write,
		ncm_ipa_ctx->directory, &ncm_ipa_ctx->tx_filter);

	debugfs_create_bool
		("rx_filter", flags_read_write,
		ncm_ipa_ctx->directory, &ncm_ipa_ctx->rx_filter);

	debugfs_create_bool
		("icmp_filter", flags_read_write,
		ncm_ipa_ctx->directory, &ncm_ipa_ctx->icmp_filter);

	debugfs_create_u32
		("outstanding_high", flags_read_write,
		ncm_ipa_ctx->directory,
		&ncm_ipa_ctx->outstanding_high);

	debugfs_create_u32
		("outstanding_low", flags_read_write,
		ncm_ipa_ctx->directory,
		&ncm_ipa_ctx->outstanding_low);

	file = debugfs_create_file
		("outstanding", flags_read_only,
		ncm_ipa_ctx->directory,
		ncm_ipa_ctx, &ncm_ipa_debugfs_atomic_ops);
	if (!file) {
		NCM_IPA_ERROR("could not create outstanding file\n");
		goto fail_file;
	}

	debugfs_create_u8
		("state", flags_read_only,
		ncm_ipa_ctx->directory, (u8 *)&ncm_ipa_ctx->state);

	debugfs_create_u32
		("tx_dropped", flags_read_only,
		ncm_ipa_ctx->directory, &ncm_ipa_ctx->tx_dropped);

	debugfs_create_u32
		("rx_dropped", flags_read_only,
		ncm_ipa_ctx->directory, &ncm_ipa_ctx->rx_dropped);

	aggr_directory = debugfs_create_dir
		(DEBUGFS_AGGR_DIR_NAME,
		ncm_ipa_ctx->directory);
	if (!aggr_directory) {
		NCM_IPA_ERROR("could not create debugfs aggr entry\n");
		goto fail_directory;
	}

	file = debugfs_create_file
		("aggr_value_set", flags_write_only,
		aggr_directory,
		ncm_ipa_ctx, &ncm_ipa_aggr_ops);
	if (!file) {
		NCM_IPA_ERROR("could not create aggr_value_set file\n");
		goto fail_file;
	}

	debugfs_create_u8
		("aggr_enable", flags_read_write,
		aggr_directory, (u8 *)&ipa_to_usb_ep_cfg.aggr.aggr_en);

	debugfs_create_u8
		("aggr_type", flags_read_write,
		aggr_directory, (u8 *)&ipa_to_usb_ep_cfg.aggr.aggr);

	debugfs_create_u32
		("aggr_byte_limit", flags_read_write,
		aggr_directory,
		&ipa_to_usb_ep_cfg.aggr.aggr_byte_limit);

	debugfs_create_u32
		("aggr_time_limit", flags_read_write,
		aggr_directory,
		&ipa_to_usb_ep_cfg.aggr.aggr_time_limit);

	debugfs_create_u32
		("aggr_pkt_limit", flags_read_write,
		aggr_directory,
		&ipa_to_usb_ep_cfg.aggr.aggr_pkt_limit);

	debugfs_create_bool
		("tx_dump_enable", flags_read_write,
		ncm_ipa_ctx->directory,
		&ncm_ipa_ctx->tx_dump_enable);

	debugfs_create_bool
		("rx_dump_enable", flags_read_write,
		ncm_ipa_ctx->directory,
		&ncm_ipa_ctx->rx_dump_enable);

	debugfs_create_bool
		("deaggregation_enable", flags_read_write,
		ncm_ipa_ctx->directory,
		&ncm_ipa_ctx->deaggregation_enable);

	debugfs_create_u32
		("error_msec_sleep_time", flags_read_write,
		ncm_ipa_ctx->directory,
		&ncm_ipa_ctx->error_msec_sleep_time);

	debugfs_create_bool
		("during_xmit_error", flags_read_only,
		ncm_ipa_ctx->directory,
		&ncm_ipa_ctx->during_xmit_error);

	debugfs_create_bool("is_vlan_mode", flags_read_only,
		ncm_ipa_ctx->directory,
		&ncm_ipa_ctx->is_vlan_mode);

	NCM_IPA_DEBUG("debugfs entries were created\n");
	NCM_IPA_LOG_EXIT();

	return;
fail_file:
	debugfs_remove_recursive(ncm_ipa_ctx->directory);
fail_directory:
	return;
}

static void ncm_ipa_debugfs_destroy(struct ncm_ipa_dev *ncm_ipa_ctx)
{
	debugfs_remove_recursive(ncm_ipa_ctx->directory);
}

#else /* !CONFIG_DEBUG_FS */

static void ncm_ipa_debugfs_init(struct ncm_ipa_dev *ncm_ipa_ctx) {}

static void ncm_ipa_debugfs_destroy(struct ncm_ipa_dev *ncm_ipa_ctx) {}

#endif /* CONFIG_DEBUG_FS*/

static int ncm_ipa_debugfs_aggr_open
		(struct inode *inode,
		struct file *file)
{
	struct ncm_ipa_dev *ncm_ipa_ctx = inode->i_private;

	file->private_data = ncm_ipa_ctx;

	return 0;
}

static ssize_t ncm_ipa_debugfs_aggr_write
	(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	struct ncm_ipa_dev *ncm_ipa_ctx = NULL;
	int result;

	if (file == NULL)
		return -EFAULT;
	ncm_ipa_ctx = file->private_data;

	result = ipa3_cfg_ep(ncm_ipa_ctx->usb_to_ipa_hdl, &ipa_to_usb_ep_cfg);
	if (result) {
		pr_err("failed to re-configure USB to IPA point\n");
		return result;
	}
	pr_info("IPA<-USB end-point re-configured\n");

	return count;
}

static int ncm_ipa_debugfs_atomic_open(struct inode *inode, struct file *file)
{
	struct ncm_ipa_dev *ncm_ipa_ctx = inode->i_private;

	NCM_IPA_LOG_ENTRY();

	file->private_data = &ncm_ipa_ctx->outstanding_pkts;

	NCM_IPA_LOG_EXIT();

	return 0;
}

static ssize_t ncm_ipa_debugfs_atomic_read
	(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes;
	u8 atomic_str[DEBUGFS_TEMP_BUF_SIZE] = {0};
	atomic_t *atomic_var = file->private_data;

	NCM_IPA_LOG_ENTRY();

	nbytes = scnprintf
		(atomic_str, sizeof(atomic_str), "%d\n",
		atomic_read(atomic_var));

	NCM_IPA_LOG_EXIT();

	return simple_read_from_buffer(ubuf, count, ppos, atomic_str, nbytes);
}

/**
 * ncm_ipa_init_module() - Initialize the NCM IPA module.
 *
 * This function is called when the module is loaded. It creates an IPC log
 * context and logs a message indicating that the module is loaded.
 *
 * Returns: 0 on success.
 */
int ncm_ipa_init_module(void)
{
	ipa_ncm_logbuf = ipc_log_context_create(IPA_NCM_IPC_LOG_PAGES,
		"ipa_ncm", MINIDUMP_MASK);
	if (ipa_ncm_logbuf == NULL)
		NCM_IPA_ERROR("Failed to create IPC log, continue...\n");

	pr_info("NCM_IPA module is loaded.\n");
	return 0;
}
EXPORT_SYMBOL_GPL(ncm_ipa_init_module);

/**
 * ncm_ipa_cleanup_module() - Clean up the NCM IPA module.
 *
 * This function is called when the module is unloaded. It destroys the IPC log
 * context and logs a message indicating that the module is unloaded.
 *
 * Returns: None
 */
void ncm_ipa_cleanup_module(void)
{
	if (ipa_ncm_logbuf)
		ipc_log_context_destroy(ipa_ncm_logbuf);
	ipa_ncm_logbuf = NULL;

	pr_info("NCM_IPA module is unloaded.\n");
}
EXPORT_SYMBOL_GPL(ncm_ipa_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NCM_IPA network interface");
