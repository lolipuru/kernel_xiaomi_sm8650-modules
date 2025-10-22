#ifndef __SYNA_XIAOMI_DRIVER_H__
#define __SYNA_XIAOMI_DRIVER_H__
#ifdef CONFIG_TRUSTED_TOUCH
#include "qts/qts_core.h"
#endif

#define SYNA_DISPLAY_RESOLUTION_SIZE 2


#define SYNA_DISPLAY_RESOLUTION_ARRAY       "synaptics,panel-display-resolution"
#define SYNAPTICS_DEBUGFS_ENABLE

#ifdef SYNAPTICS_DEBUGFS_ENABLE
#include <linux/debugfs.h>
#endif

#ifdef TOUCH_THP_SUPPORT
#define HAL_ROW_NUM             18
#define HAL_COL_NUM             40
#define HAL_NODE_NUM            ((HAL_ROW_NUM) * (HAL_COL_NUM))
#define HAL_SNODE_NUM           ((HAL_ROW_NUM) + (HAL_COL_NUM))
#pragma pack(1)
struct tp_raw {
    uint16_t signature;
    uint16_t head_count;
    int crc;
    int crc_len;
    int crc_r;
    int crc_r_len;
    char data_state;
    char err_info;
    char event_info;
    char noise_level;
    char scan_mode;
    char scan_rate;
    uint16_t scan_freq;
    uint16_t frame_index;
    uint16_t drop_frame_no;
    uint16_t noise_r0;
    uint16_t noise_r1;
    uint16_t noise_r2;
    uint16_t noise_r3;
    int reserved1;
    int reserved2;
    char col_num;
    char row_num;
    uint16_t reserved3;
    int reserved4;
    int reserved5;
    int reserved6;
    int16_t mc_raw[HAL_NODE_NUM];
    int16_t sc_raw[HAL_SNODE_NUM];
};
#pragma pack()

struct tp_frame {
	s64 time_ns;
	u64 frm_cnt;
	int fod_pressed;
	int fod_trackingId;
	char thp_frame_buf[PAGE_SIZE];
};
#endif

struct synaptics_config_info {
	u8 tp_vendor;
	const char *synaptics_cfg_name;
	const char *synaptics_fw_name;
	const char *synaptics_limit_name;
};

typedef struct {
	bool x_flip;
	bool y_flip;
	int max_x;
	int max_y;
	int fod_lx;
	int fod_ly;
	int fod_x_size;
	int fod_y_size;
	int tx_num;
	int rx_num;
	int special_rx_num;
	int special_tx_num;
	int frame_data_page_size;
	int frame_data_buf_size;
	int raw_data_page_size;
	int raw_data_buf_size;
	int super_resolution_factor;
	size_t config_array_size;
	struct synaptics_config_info *config_array;
	const char *synaptics_default_cfg_name;
	const char *synaptics_default_fw_name;
	const char *synaptics_default_limit_name;
	const char *config_file_name;
	const char *fw_image_name;
	const char *test_limit_name;
} xiaomi_bdata_t;

typedef struct {
	bool lockdown_info_ready;
	char lockdown_info[8];
#ifdef CONFIG_TRUSTED_TOUCH
	struct completion tui_finish;
	bool tui_process;
#endif
} xiaomi_driver_data_t;

#endif

#ifdef TOUCH_THP_SUPPORT
/**
 * @brief: enable touch raw on/off
 *
 * @param
 *    [ in] en: on: 1; off: 0
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_tcm_enable_touch_raw(int en);
#endif
