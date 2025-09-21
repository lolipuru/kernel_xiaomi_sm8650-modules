/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM ncm_ipa
#define TRACE_INCLUDE_FILE ncm_ipa_trace

#if !defined(_NCM_IPA_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _NCM_IPA_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(
	ncm_netif_ni,

	TP_PROTO(unsigned long proto),

	TP_ARGS(proto),

	TP_STRUCT__entry(
		__field(unsigned long,	proto)
	),

	TP_fast_assign(
		__entry->proto = proto;
	),

	TP_printk("proto =%lu\n", __entry->proto)
);

TRACE_EVENT(
	ncm_tx_dp,

	TP_PROTO(unsigned long proto),

	TP_ARGS(proto),

	TP_STRUCT__entry(
		__field(unsigned long,	proto)
	),

	TP_fast_assign(
		__entry->proto = proto;
	),

	TP_printk("proto =%lu\n", __entry->proto)
);

TRACE_EVENT(
	ncm_status_rcvd,

	TP_PROTO(unsigned long proto),

	TP_ARGS(proto),

	TP_STRUCT__entry(
		__field(unsigned long,	proto)
	),

	TP_fast_assign(
		__entry->proto = proto;
	),

	TP_printk("proto =%lu\n", __entry->proto)
);

#endif /* _NCM_IPA_TRACE_H */

/* This part must be outside protection */
#ifndef NCM_TRACE_INCLUDE_PATH
#ifdef CONFIG_IPA_VENDOR_DLKM
#define NCM_TRACE_INCLUDE_PATH \
("../../../../vendor/qcom/opensource/dataipa/drivers/platform/msm/ipa/ipa_clients")
#else
#define NCM_TRACE_INCLUDE_PATH \
("../../techpack/dataipa/drivers/platform/msm/ipa/ipa_clients")
#endif
#endif

#define TRACE_INCLUDE_PATH NCM_TRACE_INCLUDE_PATH
#include <trace/define_trace.h>
