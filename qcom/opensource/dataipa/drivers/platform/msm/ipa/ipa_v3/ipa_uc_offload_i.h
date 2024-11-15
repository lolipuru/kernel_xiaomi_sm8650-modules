/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _IPA_UC_OFFLOAD_I_H_
#define _IPA_UC_OFFLOAD_I_H_

#include "ipa.h"
#include "ipa_i.h"

/*
 * Neutrino protocol related data structures
 */

#define IPA_UC_MAX_NTN_TX_CHANNELS 2
#define IPA_UC_MAX_NTN_RX_CHANNELS 2

#define IPA_NTN_TX_DIR 1
#define IPA_NTN_RX_DIR 2

#define MAX_CH_STATS_SUPPORTED 5
#define DIR_CONSUMER 0
#define DIR_PRODUCER 1

#define MAX_AQC_CHANNELS 2
#define MAX_RTK_CHANNELS 2
#define MAX_NTN_CHANNELS 2
#define MAX_11AD_CHANNELS 5
#define MAX_WDI2_CHANNELS 2
#define MAX_WDI3_CHANNELS 3
#define MAX_MHIP_CHANNELS 4
#define MAX_USB_CHANNELS 2

#define BW_QUOTA_MONITORING_MAX_ADDR_OFFSET 8
#define BW_MONITORING_MAX_THRESHOLD 3
/**
 *  @brief   Enum value determined based on the feature it
 *           corresponds to
 *  +----------------+----------------+
 *  |    3 bits      |     5 bits     |
 *  +----------------+----------------+
 *  |   HW_FEATURE   |     OPCODE     |
 *  +----------------+----------------+
 *
 */
#define FEATURE_ENUM_VAL(feature, opcode) ((feature << 5) | opcode)
#define EXTRACT_UC_FEATURE(value) (value >> 5)

#define IPA_HW_NUM_FEATURES 0x8

/**
 * enum ipa3_hw_features - Values that represent the features supported
 * in IPA HW
 * @IPA_HW_FEATURE_COMMON : Feature related to common operation of IPA HW
 * @IPA_HW_FEATURE_MHI : Feature related to MHI operation in IPA HW
 * @IPA_HW_FEATURE_POWER_COLLAPSE: Feature related to IPA Power collapse
 * @IPA_HW_FEATURE_WDI : Feature related to WDI operation in IPA HW
 * @IPA_HW_FEATURE_NTN : Feature related to NTN operation in IPA HW
 * @IPA_HW_FEATURE_OFFLOAD : Feature related to several protocols operation in
 *				IPA HW. use protocol field to
 *				 determine (e.g. IPA_HW_PROTOCOL_11ad).
 */
enum ipa3_hw_features {
	IPA_HW_FEATURE_COMMON		=	0x0,
	IPA_HW_FEATURE_MHI		=	0x1,
	IPA_HW_FEATURE_POWER_COLLAPSE	=	0x2,
	IPA_HW_FEATURE_WDI		=	0x3,
	IPA_HW_FEATURE_ZIP		=	0x4,
	IPA_HW_FEATURE_NTN		=	0x5,
	IPA_HW_FEATURE_OFFLOAD		=	0x6,
	IPA_HW_FEATURE_RTP		=	0x8,
	IPA_HW_FEATURE_MAX		=	IPA_HW_NUM_FEATURES
};

/**
 * enum ipa4_hw_protocol - Values that represent the protocols supported
 * in IPA HW when using the IPA_HW_FEATURE_OFFLOAD feature.
 * @IPA_HW_FEATURE_COMMON : protocol related to common operation of IPA HW
 * @IPA_HW_PROTOCOL_AQC : protocol related to AQC operation in IPA HW
 * @IPA_HW_PROTOCOL_11ad: protocol related to 11ad operation in IPA HW
 * @IPA_HW_PROTOCOL_WDI : protocol related to WDI operation in IPA HW
 * @IPA_HW_PROTOCOL_WDI3: protocol related to WDI3 operation in IPA HW
 * @IPA_HW_PROTOCOL_ETH : protocol related to ETH operation in IPA HW
 * @IPA_HW_PROTOCOL_MHIP: protocol related to MHIP operation in IPA HW
 * @IPA_HW_PROTOCOL_USB : protocol related to USB operation in IPA HW
 * @IPA_HW_PROTOCOL_RTK : protocol related to RTK operation in IPA HW
 * @IPA_HW_PROTOCOL_NTN3 : protocol related to NTN3 operation in IPA HW
 */
enum ipa4_hw_protocol {
	IPA_HW_PROTOCOL_COMMON = 0x0,
	IPA_HW_PROTOCOL_AQC = 0x1,
	IPA_HW_PROTOCOL_11ad = 0x2,
	IPA_HW_PROTOCOL_WDI = 0x3,
	IPA_HW_PROTOCOL_WDI3 = 0x4,
	IPA_HW_PROTOCOL_ETH = 0x5,
	IPA_HW_PROTOCOL_MHIP = 0x6,
	IPA_HW_PROTOCOL_USB = 0x7,
	IPA_HW_PROTOCOL_RTK = 0x9,
	IPA_HW_PROTOCOL_NTN3 = 0xA,
	IPA_HW_PROTOCOL_MAX
};

/**
 * enum ipa3_hw_2_cpu_events - Values that represent HW event to be sent to CPU.
 * @IPA_HW_2_CPU_EVENT_NO_OP : No event present
 * @IPA_HW_2_CPU_EVENT_ERROR : Event specify a system error is detected by the
 *  device
 * @IPA_HW_2_CPU_EVENT_LOG_INFO : Event providing logging specific information
 * @IPA_HW_2_CPU_POST_EVNT_RING_NOTIFICAITON : Event to notify APPS
 */
enum ipa3_hw_2_cpu_events {
	IPA_HW_2_CPU_EVENT_NO_OP     =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 0),
	IPA_HW_2_CPU_EVENT_ERROR     =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 1),
	IPA_HW_2_CPU_EVENT_LOG_INFO  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 2),
	IPA_HW_2_CPU_EVNT_RING_NOTIFY  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 3),
};

/**
 * enum ipa3_hw_errors - Common error types.
 * @IPA_HW_ERROR_NONE : No error persists
 * @IPA_HW_INVALID_DOORBELL_ERROR : Invalid data read from doorbell
 * @IPA_HW_DMA_ERROR : Unexpected DMA error
 * @IPA_HW_FATAL_SYSTEM_ERROR : HW has crashed and requires reset.
 * @IPA_HW_INVALID_OPCODE : Invalid opcode sent
 * @IPA_HW_INVALID_PARAMS : Invalid params for the requested command
 * @IPA_HW_GSI_CH_NOT_EMPTY_FAILURE : GSI channel emptiness validation failed
 * @IPA_HW_CONS_STOP_FAILURE : NTN/ETH CONS stop failed
 * @IPA_HW_PROD_STOP_FAILURE : NTN/ETH PROD stop failed
 */
enum ipa3_hw_errors {
	IPA_HW_ERROR_NONE              =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 0),
	IPA_HW_INVALID_DOORBELL_ERROR  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 1),
	IPA_HW_DMA_ERROR               =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 2),
	IPA_HW_FATAL_SYSTEM_ERROR      =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 3),
	IPA_HW_INVALID_OPCODE          =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 4),
	IPA_HW_INVALID_PARAMS        =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 5),
	IPA_HW_CONS_DISABLE_CMD_GSI_STOP_FAILURE =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 6),
	IPA_HW_PROD_DISABLE_CMD_GSI_STOP_FAILURE =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 7),
	IPA_HW_GSI_CH_NOT_EMPTY_FAILURE =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 8),
	IPA_HW_CONS_STOP_FAILURE =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 9),
	IPA_HW_PROD_STOP_FAILURE =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_COMMON, 10)
};

/**
 * struct IpaHwSharedMemCommonMapping_t - Structure referring to the common
 * section in 128B shared memory located in offset zero of SW Partition in IPA
 * SRAM.
 * @cmdOp : CPU->HW command opcode. See IPA_CPU_2_HW_COMMANDS
 * @cmdParams : CPU->HW command parameter lower 32bit.
 * @cmdParams_hi : CPU->HW command parameter higher 32bit.
 * of parameters (immediate parameters) and point on structure in system memory
 * (in such case the address must be accessible for HW)
 * @responseOp : HW->CPU response opcode. See IPA_HW_2_CPU_RESPONSES
 * @responseParams : HW->CPU response parameter. The parameter filed can hold 32
 * bits of parameters (immediate parameters) and point on structure in system
 * memory
 * @eventOp : HW->CPU event opcode. See IPA_HW_2_CPU_EVENTS
 * @eventParams : HW->CPU event parameter. The parameter filed can hold 32
 *		bits of parameters (immediate parameters) and point on
 *		structure in system memory
 * @firstErrorAddress : Contains the address of first error-source on SNOC
 * @hwState : State of HW. The state carries information regarding the
 *				error type.
 * @warningCounter : The warnings counter. The counter carries information
 *						regarding non fatal errors in HW
 * @interfaceVersionCommon : The Common interface version as reported by HW
 * @responseParams_1: offset addr for uC stats
 *
 * The shared memory is used for communication between IPA HW and CPU.
 */
struct IpaHwSharedMemCommonMapping_t {
	u8  cmdOp;
	u8  reserved_01;
	u16 reserved_03_02;
	u32 cmdParams;
	u32 cmdParams_hi;
	u8  responseOp;
	u8  reserved_0D;
	u16 reserved_0F_0E;
	u32 responseParams;
	u8  eventOp;
	u8  reserved_15;
	u16 reserved_17_16;
	u32 eventParams;
	u32 firstErrorAddress;
	u8  hwState;
	u8  warningCounter;
	u16 reserved_23_22;
	u16 interfaceVersionCommon;
	u16 reserved_27_26;
	u32 responseParams_1;
} __packed;

/**
 * union Ipa3HwFeatureInfoData_t - parameters for stats/config blob
 *
 * @offset : Location of a feature within the EventInfoData
 * @size : Size of the feature
 */
union Ipa3HwFeatureInfoData_t {
	struct IpaHwFeatureInfoParams_t {
		u32 offset:16;
		u32 size:16;
	} __packed params;
	u32 raw32b;
} __packed;

/**
 * union IpaHwErrorEventData_t - HW->CPU Common Events
 * @errorType : Entered when a system error is detected by the HW. Type of
 * error is specified by IPA_HW_ERRORS
 * @reserved : Reserved
 */
union IpaHwErrorEventData_t {
	struct IpaHwErrorEventParams_t {
		u32 errorType:8;
		u32 reserved:24;
	} __packed params;
	u32 raw32b;
} __packed;

/**
 * struct Ipa3HwEventInfoData_t - Structure holding the parameters for
 * statistics and config info
 *
 * @baseAddrOffset : Base Address Offset of the statistics or config
 * structure from IPA_WRAPPER_BASE
 * @Ipa3HwFeatureInfoData_t : Location and size of each feature within
 * the statistics or config structure
 *
 * @note    Information about each feature in the featureInfo[]
 * array is populated at predefined indices per the IPA_HW_FEATURES
 * enum definition
 */
struct Ipa3HwEventInfoData_t {
	u32 baseAddrOffset;
	union Ipa3HwFeatureInfoData_t featureInfo[IPA_HW_NUM_FEATURES];
} __packed;

/**
 * struct IpaHwEventLogInfoData_t - Structure holding the parameters for
 * IPA_HW_2_CPU_EVENT_LOG_INFO Event
 *
 * @protocolMask : Mask indicating the protocols enabled in HW.
 * Refer IPA_HW_FEATURE_MASK
 * @circBuffBaseAddrOffset : Base Address Offset of the Circular Event
 * Log Buffer structure
 * @statsInfo : Statistics related information
 * @configInfo : Configuration related information
 *
 * @note    The offset location of this structure from IPA_WRAPPER_BASE
 * will be provided as Event Params for the IPA_HW_2_CPU_EVENT_LOG_INFO
 * Event
 */
struct IpaHwEventLogInfoData_t {
	u32 protocolMask;
	u32 circBuffBaseAddrOffset;
	struct Ipa3HwEventInfoData_t statsInfo;
	struct Ipa3HwEventInfoData_t configInfo;

} __packed;

/**
 * struct ipa3_uc_ntn_ctx
 * @ntn_uc_stats_ofst: Neutrino stats offset
 * @ntn_uc_stats_mmio: Neutrino stats
 * @priv: private data of client
 * @uc_ready_cb: uc Ready cb
 */
struct ipa3_uc_ntn_ctx {
	u32 ntn_uc_stats_ofst;
	struct Ipa3HwStatsNTNInfoData_t *ntn_uc_stats_mmio;
	void *priv;
	ipa_uc_ready_cb uc_ready_cb;
	phys_addr_t ntn_reg_base_ptr_pa_rd;
	u32 smmu_mapped;
};

/**
 * enum ipa3_hw_ntn_channel_states - Values that represent NTN
 * channel state machine.
 * @IPA_HW_NTN_CHANNEL_STATE_INITED_DISABLED : Channel is
 *			initialized but disabled
 * @IPA_HW_NTN_CHANNEL_STATE_RUNNING : Channel is running.
 *     Entered after SET_UP_COMMAND is processed successfully
 * @IPA_HW_NTN_CHANNEL_STATE_ERROR : Channel is in error state
 * @IPA_HW_NTN_CHANNEL_STATE_INVALID : Invalid state. Shall not
 * be in use in operational scenario
 *
 * These states apply to both Tx and Rx paths. These do not reflect the
 * sub-state the state machine may be in.
 */
enum ipa3_hw_ntn_channel_states {
	IPA_HW_NTN_CHANNEL_STATE_INITED_DISABLED = 1,
	IPA_HW_NTN_CHANNEL_STATE_RUNNING  = 2,
	IPA_HW_NTN_CHANNEL_STATE_ERROR    = 3,
	IPA_HW_NTN_CHANNEL_STATE_INVALID  = 0xFF
};

/**
 * enum ipa3_hw_ntn_channel_errors - List of NTN Channel error
 * types. This is present in the event param
 * @IPA_HW_NTN_CH_ERR_NONE: No error persists
 * @IPA_HW_NTN_TX_FSM_ERROR: Error in the state machine
 *		transition
 * @IPA_HW_NTN_TX_COMP_RE_FETCH_FAIL: Error while calculating
 *		num RE to bring
 * @IPA_HW_NTN_RX_RING_WP_UPDATE_FAIL: Write pointer update
 *		failed in Rx ring
 * @IPA_HW_NTN_RX_FSM_ERROR: Error in the state machine
 *		transition
 * @IPA_HW_NTN_RX_CACHE_NON_EMPTY:
 * @IPA_HW_NTN_CH_ERR_RESERVED:
 *
 * These states apply to both Tx and Rx paths. These do not
 * reflect the sub-state the state machine may be in.
 */
enum ipa3_hw_ntn_channel_errors {
	IPA_HW_NTN_CH_ERR_NONE            = 0,
	IPA_HW_NTN_TX_RING_WP_UPDATE_FAIL = 1,
	IPA_HW_NTN_TX_FSM_ERROR           = 2,
	IPA_HW_NTN_TX_COMP_RE_FETCH_FAIL  = 3,
	IPA_HW_NTN_RX_RING_WP_UPDATE_FAIL = 4,
	IPA_HW_NTN_RX_FSM_ERROR           = 5,
	IPA_HW_NTN_RX_CACHE_NON_EMPTY     = 6,
	IPA_HW_NTN_CH_ERR_RESERVED        = 0xFF
};


/**
 * struct uc_channel_setup_cmd_hw_ntn  - Ntn setup command data
 * @ring_base_pa: physical address of the base of the Tx/Rx NTN
 *  ring
 * @buff_pool_base_pa: physical address of the base of the Tx/Rx
 *  buffer pool
 * @ntn_ring_size: size of the Tx/Rx NTN ring
 * @num_buffers: Rx/tx buffer pool size
 * @ntn_reg_base_ptr_pa: physical address of the Tx/Rx NTN
 *  Ring's tail pointer
 * @ipa_pipe_number: IPA pipe number that has to be used for the
 *  Tx/Rx path
 * @dir: Tx/Rx Direction
 * @data_buff_size: size of the each data buffer allocated in
 *  DDR
 */
struct uc_channel_setup_cmd_hw_ntn {
	u32 ring_base_pa;
	u32 buff_pool_base_pa;
	u16 ntn_ring_size;
	u16 num_buffers;
	u32 ntn_reg_base_ptr_pa;
	u8  ipa_pipe_number;
	u8  dir;
	u16 data_buff_size;
	u8 db_mode;
	u8 reserved1;
	u16 reserved2;

} __packed;

/**
 * struct uc_channel_teardown_cmd_hw_ntn - Structure holding the
 * parameters for Ntn Tear down command data params
 *
 *@ipa_pipe_number: IPA pipe number. This could be Tx or an Rx pipe
 */
union uc_channel_teardown_cmd_hw_ntn {
	struct IpaHwNtnCommonChCmdParams_t {
		u32  ipa_pipe_number :8;
		u32  reserved        :24;
	} __packed params;
	uint32_t raw32b;
} __packed;

/**
 * struct NTN3RxInfoData_t - NTN Structure holding the Rx pipe
 * information
 *
 *@num_pkts_processed: Number of packets processed - cumulative
 *
 *@ring_stats:
 *@gsi_stats:
 *@num_db: Number of times the doorbell was rung
 *@num_qmb_int_handled: Number of QMB interrupts handled
 *@ipa_pipe_number: The IPA Rx/Tx pipe number.
 */
struct NTN3RxInfoData_t {
	u32  num_pkts_processed;
	struct IpaHwRingStats_t ring_stats;
	struct IpaHwBamStats_t gsi_stats;
	u32 num_db;
	u32 num_qmb_int_handled;
	u32 ipa_pipe_number;
} __packed;


/**
 * struct NTN3TxInfoData_t - Structure holding the NTN Tx channel
 * Ensure that this is always word aligned
 *
 *@num_pkts_processed: Number of packets processed - cumulative
 *@tail_ptr_val: Latest value of doorbell written to copy engine
 *@num_db_fired: Number of DB from uC FW to Copy engine
 *
 *@tx_comp_ring_stats:
 *@bam_stats:
 *@num_db: Number of times the doorbell was rung
 *@num_qmb_int_handled: Number of QMB interrupts handled
 */
struct NTN3TxInfoData_t {
	u32  num_pkts_processed;
	struct IpaHwRingStats_t ring_stats;
	struct IpaHwBamStats_t gsi_stats;
	u32 num_db;
	u32 num_qmb_int_handled;
	u32 ipa_pipe_number;
} __packed;


/**
 * struct Ipa3HwStatsNTNInfoData_t - Structure holding the NTN Tx
 * channel Ensure that this is always word aligned
 *
 */
struct Ipa3HwStatsNTNInfoData_t {
	struct NTN3RxInfoData_t rx_ch_stats[IPA_UC_MAX_NTN_RX_CHANNELS];
	struct NTN3TxInfoData_t tx_ch_stats[IPA_UC_MAX_NTN_TX_CHANNELS];
} __packed;


/*
 * uC offload related data structures
 */
#define IPA_UC_OFFLOAD_CONNECTED BIT(0)
#define IPA_UC_OFFLOAD_ENABLED BIT(1)
#define IPA_UC_OFFLOAD_RESUMED BIT(2)

/**
 * enum ipa_cpu_2_hw_offload_commands -  Values that represent
 * the offload commands from CPU
 * @IPA_CPU_2_HW_CMD_OFFLOAD_CHANNEL_SET_UP : Command to set up
 * Offload protocol's Tx/Rx Path
 * @IPA_CPU_2_HW_CMD_OFFLOAD_CHANNEL_TEAR_DOWN : Command to tear
 * down Offload protocol's Tx/ Rx Path
 * @IPA_CPU_2_HW_CMD_PERIPHERAL_INIT :Command to initialize peripheral
 * @IPA_CPU_2_HW_CMD_PERIPHERAL_DEINIT : Command to deinitialize peripheral
 * @IPA_CPU_2_HW_CMD_OFFLOAD_STATS_ALLOC: Command to start the
 * uC stats calculation for a particular protocol
 * @IPA_CPU_2_HW_CMD_OFFLOAD_STATS_DEALLOC: Command to stop the
 * uC stats calculation for a particular protocol
 * @IPA_CPU_2_HW_CMD_QUOTA_MONITORING : Command to start the Quota monitoring
 * @IPA_CPU_2_HW_CMD_BW_MONITORING : Command to start the BW monitoring
 */
enum ipa_cpu_2_hw_offload_commands {
	IPA_CPU_2_HW_CMD_OFFLOAD_CHANNEL_SET_UP  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 1),
	IPA_CPU_2_HW_CMD_OFFLOAD_CHANNEL_TEAR_DOWN =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 2),
	IPA_CPU_2_HW_CMD_PERIPHERAL_INIT =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 3),
	IPA_CPU_2_HW_CMD_PERIPHERAL_DEINIT =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 4),
	IPA_CPU_2_HW_CMD_OFFLOAD_STATS_ALLOC =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 5),
	IPA_CPU_2_HW_CMD_OFFLOAD_STATS_DEALLOC =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 6),
	IPA_CPU_2_HW_CMD_QUOTA_MONITORING =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 7),
	IPA_CPU_2_HW_CMD_BW_MONITORING =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 8),
};

/**
 * struct IpaHwOffloadStatsDeAllocCmdData_t - protocol info for
 * uC stats stop
 * @protocol: Enum that indicates the protocol type
 */
struct IpaHwOffloadStatsDeAllocCmdData_t {
	uint32_t protocol;
} __packed;

/**
 * enum ipa3_hw_offload_channel_states - Values that represent
 * offload channel state machine.
 * @IPA_HW_OFFLOAD_CHANNEL_STATE_INITED_DISABLED : Channel is
 *			initialized but disabled
 * @IPA_HW_OFFLOAD_CHANNEL_STATE_RUNNING : Channel is running.
 *			Entered after SET_UP_COMMAND is processed successfully
 * @IPA_HW_OFFLOAD_CHANNEL_STATE_ERROR : Channel is in error state
 * @IPA_HW_OFFLOAD_CHANNEL_STATE_INVALID : Invalid state. Shall not
 *				be in use in operational scenario
 *
 * These states apply to both Tx and Rx paths. These do not
 * reflect the sub-state the state machine may be in
 */
enum ipa3_hw_offload_channel_states {
	IPA_HW_OFFLOAD_CHANNEL_STATE_INITED_DISABLED = 1,
	IPA_HW_OFFLOAD_CHANNEL_STATE_RUNNING  = 2,
	IPA_HW_OFFLOAD_CHANNEL_STATE_ERROR    = 3,
	IPA_HW_OFFLOAD_CHANNEL_STATE_INVALID  = 0xFF
};


/**
 * enum ipa3_hw_2_cpu_cmd_resp_status -  Values that represent
 * offload related command response status to be sent to CPU.
 */
enum ipa3_hw_2_cpu_offload_cmd_resp_status {
	IPA_HW_2_CPU_OFFLOAD_CMD_STATUS_SUCCESS  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 0),
	IPA_HW_2_CPU_OFFLOAD_MAX_TX_CHANNELS  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 1),
	IPA_HW_2_CPU_OFFLOAD_TX_RING_OVERRUN_POSSIBILITY  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 2),
	IPA_HW_2_CPU_OFFLOAD_TX_RING_SET_UP_FAILURE  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 3),
	IPA_HW_2_CPU_OFFLOAD_TX_RING_PARAMS_UNALIGNED  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 4),
	IPA_HW_2_CPU_OFFLOAD_UNKNOWN_TX_CHANNEL  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 5),
	IPA_HW_2_CPU_OFFLOAD_TX_INVALID_FSM_TRANSITION  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 6),
	IPA_HW_2_CPU_OFFLOAD_TX_FSM_TRANSITION_ERROR  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 7),
	IPA_HW_2_CPU_OFFLOAD_MAX_RX_CHANNELS  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 8),
	IPA_HW_2_CPU_OFFLOAD_RX_RING_PARAMS_UNALIGNED  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 9),
	IPA_HW_2_CPU_OFFLOAD_RX_RING_SET_UP_FAILURE  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 10),
	IPA_HW_2_CPU_OFFLOAD_UNKNOWN_RX_CHANNEL  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 11),
	IPA_HW_2_CPU_OFFLOAD_RX_INVALID_FSM_TRANSITION  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 12),
	IPA_HW_2_CPU_OFFLOAD_RX_FSM_TRANSITION_ERROR  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 13),
	IPA_HW_2_CPU_OFFLOAD_RX_RING_OVERRUN_POSSIBILITY  =
		FEATURE_ENUM_VAL(IPA_HW_FEATURE_OFFLOAD, 14),
};

/**
 * struct uc_channel_setup_cmd_hw_11ad  - 11ad setup channel
 * command data
 * @dir: Direction RX/TX
 * @wifi_ch: 11ad peripheral pipe number
 * @gsi_ch: GSI Channel number
 * @reserved: 8 bytes padding
 * @wifi_hp_addr_lsb: Head/Tail pointer absolute address
 * @wifi_hp_addr_msb: Head/Tail pointer absolute address
 */
struct uc_channel_setup_cmd_hw_11ad {
	u8 dir;
	u8 wifi_ch;
	u8 gsi_ch;
	u8 reserved;
	u32 wifi_hp_addr_lsb;
	u32 wifi_hp_addr_msb;
} __packed;


/**
 * struct uc_channel_teardown_cmd_hw_11ad - 11ad tear down
 * channel command data
 * @gsi_ch: GSI Channel number
 * @reserved_0: padding
 * @reserved_1: padding
 */
struct uc_channel_teardown_cmd_hw_11ad {
	u8 gsi_ch;
	u8 reserved_0;
	u16 reserved_1;
} __packed;

/**
 * struct IpaHw11adInitCmdData_t - 11ad peripheral init command data
 * @periph_baddr_lsb: Peripheral Base Address LSB (pa/IOVA)
 * @periph_baddr_msb: Peripheral Base Address MSB (pa/IOVA)
 */
struct IpaHw11adInitCmdData_t {
	u32 periph_baddr_lsb;
	u32 periph_baddr_msb;
} __packed;

/**
 * struct IpaHw11adDeinitCmdData_t - 11ad peripheral deinit command data
 * @reserved: Reserved for future
 */
struct IpaHw11adDeinitCmdData_t {
	u32 reserved;
} __packed;

/**
 * struct uc_channel_setup_cmd_hw_rtk  - rtk setup channel
 * command data
 * @dir: Direction RX/TX
 * @gsi_ch: GSI Channel number
 * @reserved: 16 bytes padding
 */
struct uc_channel_setup_cmd_hw_rtk {
	uint8_t dir;
	uint8_t gsi_ch;
	uint16_t reserved;
} __packed;

/**
 * struct uc_channel_teardown_cmd_hw_rtk - rtk tear down channel
 * command data
 * @gsi_ch: GSI Channel number
 * @reserved_0: padding
 * @reserved_1: padding
 */
struct uc_channel_teardown_cmd_hw_rtk {
	uint8_t gsi_ch;
	uint8_t reserved_0;
	uint16_t reserved_1;
} __packed;

/**
 * struct IpaHwAQCInitCmdData_t - AQC peripheral init command data
 * @periph_baddr_lsb: Peripheral Base Address LSB (pa/IOVA)
 * @periph_baddr_msb: Peripheral Base Address MSB (pa/IOVA)
 */
struct IpaHwAQCInitCmdData_t {
	u32 periph_baddr_lsb;
	u32 periph_baddr_msb;
} __packed;

/**
 * struct IpaHwAQCDeinitCmdData_t - AQC peripheral deinit command data
 * @reserved: Reserved for future
 */
struct IpaHwAQCDeinitCmdData_t {
	u32 reserved;
} __packed;

/**
 * struct uc_channel_setup_cmd_hw_aqc - AQC setup channel
 * command data
 * @dir: Direction RX/TX
 * @aqc_ch: aqc channel number
 * @gsi_ch: GSI Channel number
 * @reserved: 8 bytes padding
 */
struct uc_channel_setup_cmd_hw_aqc {
	u8 dir;
	u8 aqc_ch;
	u8 gsi_ch;
	u8 reserved;
} __packed;

/**
 * struct uc_channel_teardown_cmd_hw_aqc - AQC tear down channel
 * command data
 * @gsi_ch: GSI Channel number
 * @reserved_0: padding
 * @reserved_1: padding
 */
struct uc_channel_teardown_cmd_hw_aqc {
	u8 gsi_ch;
	u8 reserved_0;
	u16 reserved_1;
} __packed;

/**
 * struct uc_channel_setup_cmd_hw  - Structure holding the
 * parameters for IPA_CPU_2_HW_CMD_OFFLOAD_CHANNEL_SET_UP
 *
 *
 */
union uc_channel_setup_cmd_hw {
	struct uc_channel_setup_cmd_hw_ntn ntn_params;
	struct uc_channel_setup_cmd_hw_aqc aqc_params;
	struct uc_channel_setup_cmd_hw_11ad w11ad_params;
	struct uc_channel_setup_cmd_hw_rtk rtk_params;
} __packed;

struct IpaHwOffloadSetUpCmdData_t {
	u8 protocol;
	union uc_channel_setup_cmd_hw SetupCh_params;
} __packed;

struct IpaCommonMonitoringParams_t {
	/* max 8 */
	uint8_t  Num;
	/* Sampling interval in ms */
	uint8_t  Interval;
	uint16_t Offset[BW_QUOTA_MONITORING_MAX_ADDR_OFFSET];
} __packed; // 18 bytes

struct IpaWdiQuotaMonitoringParams_t {
	uint64_t Quota;
	struct IpaCommonMonitoringParams_t info;
} __packed;

struct IpaWdiBwMonitoringParams_t {
	uint64_t BwThreshold[BW_MONITORING_MAX_THRESHOLD];
	struct IpaCommonMonitoringParams_t info;
	uint8_t NumThresh;
	/*Variable to Start Stop Bw Monitoring*/
	uint8_t Stop;
} __packed;

union IpaQuotaMonitoringParams_t {
	struct IpaWdiQuotaMonitoringParams_t WdiQM;
} __packed;

union IpaBwMonitoringParams_t {
	struct IpaWdiBwMonitoringParams_t WdiBw;
} __packed;

struct IpaQuotaMonitoring_t {
	/* indicates below union needs to be interpreted */
	uint32_t protocol;
	union IpaQuotaMonitoringParams_t  params;
} __packed;

struct IpaBwMonitoring_t {
	/* indicates below union needs to be interpreted */
	uint32_t protocol;
	union IpaBwMonitoringParams_t   params;
} __packed;


struct IpaHwOffloadSetUpCmdData_t_v4_0 {
	u32 protocol;
	union uc_channel_setup_cmd_hw SetupCh_params;
} __packed;

/**
 * struct uc_channel_teardown_cmd_hw  - Structure holding the
 * parameters for IPA_CPU_2_HW_CMD_OFFLOAD_CHANNEL_TEAR_DOWN
 *
 *
 */
union uc_channel_teardown_cmd_hw {
	union uc_channel_teardown_cmd_hw_ntn ntn_params;
	struct uc_channel_teardown_cmd_hw_aqc aqc_params;
	struct uc_channel_teardown_cmd_hw_rtk rtk_params;
	struct uc_channel_teardown_cmd_hw_11ad w11ad_params;
} __packed;

struct IpaHwOffloadCommonChCmdData_t {
	u8 protocol;
	union uc_channel_teardown_cmd_hw CommonCh_params;
} __packed;

enum EVENT_2_CPU_OPCODE {
	BW_NOTIFY = 0x0,
	QUOTA_NOTIFY = 0x1,
	IPA_HOLB_BAD_PERIPHERAL_EVENT = 0x2,
	IPA_HOLB_PERIPHERAL_RECOVERED_EVENT = 0x3
};

struct EventStructureBwMonitoring_t {
	uint32_t ThresholdIndex;
	uint64_t throughput;
} __packed;

struct EventStructureQuotaMonitoring_t {
	/* indicate threshold has reached */
	uint32_t ThreasholdReached;
	uint64_t usage;
} __packed;


/**
 * @brief   Structure holding the parameters for
 *          IPA_HW_2_CPU_EVENT_PERIPH_BAD and
 *          IPA_HW_2_CPU_EVENT_PERIPH_RECOVERED events.
 *
 * @param   ipaProdGsiChid    bad OR recovered GSI chid
 * @param   EE                EE that the chid belongs to
 */
struct EventStructureHolbMonitoring_t {
	uint32_t ipaProdGsiChid :8;
	uint32_t EE             :8;
	uint32_t reserved       :16;
	uint32_t qTimerLSB;
	uint32_t qTimerMSB;
} __packed;

union EventParamFormat_t {
	struct EventStructureBwMonitoring_t bw_param;
	struct EventStructureQuotaMonitoring_t quota_param;
	struct EventStructureHolbMonitoring_t holb_notify_param;
} __packed;

/* EVT RING STRUCTURE
 *	|	Word|	bit	|	Field	|
 *	-----------------------------
 *	|	0	|0	-	8|	Protocol|
 *	|		|8	-	16|	Reserved0|
 *	|		|16	-	24|	Opcode	|
 *	|		|24	-	31|	Reserved1|
 *	|	1	|0	-	31|	Word1	|
 *	|	2	|0	-	31|	Word2	|
 *	|	3	|0	-	31|	Word3	|
 */
struct eventElement_t {
	uint8_t Protocol;
	uint8_t Reserved0;
	uint8_t Opcode;
	uint8_t Reserved1;
	union EventParamFormat_t Value;
} __packed;

struct IpaHwOffloadCommonChCmdData_t_v4_0 {
	u32 protocol;
	union uc_channel_teardown_cmd_hw CommonCh_params;
} __packed;


/**
 * union IpaHwPeripheralInitCmd - Structure holding the parameters
 * for IPA_CPU_2_HW_CMD_PERIPHERAL_INIT
 *
 */
union IpaHwPeripheralInitCmd {
	struct IpaHw11adInitCmdData_t W11AdInit_params;
	struct IpaHwAQCInitCmdData_t AqcInit_params;
} __packed;

struct IpaHwPeripheralInitCmdData_t {
	u32 protocol;
	union IpaHwPeripheralInitCmd Init_params;
} __packed;

/**
 * union IpaHwPeripheralDeinitCmd - Structure holding the parameters
 * for IPA_CPU_2_HW_CMD_PERIPHERAL_DEINIT
 *
 */
union IpaHwPeripheralDeinitCmd {
	struct IpaHw11adDeinitCmdData_t W11AdDeinit_params;
	struct IpaHwAQCDeinitCmdData_t AqcDeinit_params;
} __packed;

struct IpaHwPeripheralDeinitCmdData_t {
	u32 protocol;
	union IpaHwPeripheralDeinitCmd PeripheralDeinit_params;

} __packed;

#endif /* _IPA_UC_OFFLOAD_I_H_ */
