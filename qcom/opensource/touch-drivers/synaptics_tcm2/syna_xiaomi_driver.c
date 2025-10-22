#if defined(TOUCH_PLATFORM_XRING)
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <drm/drm_panel.h>
#include <soc/xring/display/panel_event_notifier.h>
#else
#include <linux/soc/qcom/panel_event_notifier.h>
#endif
#include "syna_tcm2.h"
#include "syna_xiaomi_driver.h"
#include "synaptics_touchcom_func_base.h"
#include "synaptics_touchcom_func_base_flash.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_reflash.h"
#include "../xiaomi/xiaomi_touch.h"

#define SYNAPTICS_DRIVER_VERSION "synaptics:2023.07.28-001-v2"
#define BIG_SMALL_CONVERT
#define TOUCH_THP_DEBUG
#define RETRY_CNT 10

bool debug_log_flag;

#ifdef BIG_SMALL_CONVERT
static u32 buf_len[] = {2, 2, 2, 2, 2, 2, 1 ,1, 1, 1, 2, 2, 2, 2, 8, 1, 2, 2, 2};
#endif

enum set_ic_mode_enum {
	ENTER_IDLE_MODE = 0,
	UPDATE_IDLE_BASELINE,
	SET_BASE_REFRESH_INTERVAL_TIME,
	SET_DYNAMIC_REPORT_RATE,
	SET_DOZE_WAKEUP_THRESHOLD,
	SET_THP_BASE_STATUS,
	SET_THP_GLOVE_STATUS,
};

static xiaomi_bdata_t xiaomi_bdata;
static xiaomi_driver_data_t xiaomi_driver_data;
static hardware_operation_t hardware_operation;
static hardware_param_t hardware_param;
static struct syna_tcm *tcm = NULL;
static void syna_set_ic_mode(enum set_ic_mode_enum mode, int *value);

#ifdef CONFIG_TRUSTED_TOUCH
struct qts_vendor_data qts_vendor_data;
#endif

#ifdef TOUCH_THP_DEBUG
static void converhex(u8 hex[4], int data)
{
    int value = data;
    hex[0] = (value & 0xFF);
    hex[1] = ((value >> 8) & 0xFF);
    hex[2] = ((value >> 16) & 0xFF);
    hex[3] = ((value >> 24) & 0xFF);
}

#ifdef BIG_SMALL_CONVERT
static inline void syna_htc_ic_data_convert(char* data, int data_len)
{
    int i = 0;
    char c = 0;
    char *buf = data;

    while (i < data_len) {
		c = buf[i];
		buf[i] = buf[i + 1];
		buf[i + 1] = c;
		i = i + 2;
	}
}

static inline int syna_htc_ic_poll_data_convert(char *tpframe)
{
	u32 buf_size = 0;
	u32 temp_len = 0;
	u8 i = 0;
	u32 j = 0;
	char temp = 0;
	char *touch_buf = tpframe;
	buf_size = sizeof(buf_len)/sizeof(u32);

	for (i = 0; i < buf_size; i++){
		if ((buf_len[i] == 2) || (buf_len[i] > 4)) {
			temp_len = buf_len[i];
			while (temp_len) {
				temp = touch_buf[j];
				touch_buf[j] = touch_buf[j + 1];
				touch_buf[j +1] = temp;
				j = j + 2;
				temp_len = temp_len - 2;
			}
		} else if (buf_len[i] == 4) {
			temp = touch_buf[j];
			touch_buf[j] = touch_buf[j + 3];
			touch_buf[j + 3] = temp;
			temp = touch_buf[j + 1];
			touch_buf[j + 1] = touch_buf[j + 2];
			touch_buf[j + 2] = temp;
			j = j + 4;
		} else if (buf_len[i] == 1) {
		   j = j + 1;
		} else {
			LOGE("buffer len error,buf_len[%d] = %d!!!",i,buf_len[i]);
			return -EIO;
		}
	}

	return 0;
}
#endif

static int syna_htc_ic_setModeValue(common_data_t *common_data)
{
	int i = 0;
	int retval = 0;
	unsigned char out_buf[5] = {0};
	int cmd_size = 0;
	unsigned char* new_out_buf = NULL;
	unsigned char *resp_buf = NULL;
	unsigned char CMD;
	struct syna_tcm *tcm_hcd = tcm;
	struct tcm_dev* tcm_dev = tcm_hcd->tcm_dev;
	u8 hex[4];
	int value_len = common_data->data_len;
	int mode = common_data->mode;

	if (common_data->mode < THP_IC_CMD_BASE) {
		LOGE("mode %d is error!!\n", common_data->mode);
		return -EIO;
	}

	if (value_len < 2) {
		LOGE("len is error!!\n");
		return -EIO;
	}

	LOGI("mode:%d, value_len:%d, value[0] = 0x%x", common_data->mode, value_len, common_data->data_buf[0]);

	if (mode == IC_MODE_44) {
		CMD = (unsigned char)common_data->data_buf[0];
		new_out_buf = (unsigned char *)&common_data->data_buf[1];
		cmd_size = value_len - 1;
		LOGI("mode:%d, CMD = 0x%x, cmd_size:%d", common_data->mode, CMD, cmd_size);
	} else {
		out_buf[0] = (u8)(common_data->mode - THP_IC_CMD_BASE);
		converhex(hex, common_data->data_buf[0]);
		for (i = 0; i < value_len; ++i) {
			out_buf[i + 1] = hex[i];
			LOGD("mode:%d, hex:%x", common_data->mode, hex[i]);
		}
		CMD = CMD_MultiFunction;
		new_out_buf = out_buf;
		cmd_size = sizeof(out_buf);
		LOGI("mode:%d, CMD = 0x%x, cmd_size:%d", common_data->mode, CMD, cmd_size);
	}

	for (i = 0; i < cmd_size; i++) {
		LOGD("new_out_buf[i:%d]:0x%x", i, new_out_buf[i]);
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD,
			new_out_buf,
			cmd_size,
			cmd_size,
			resp_buf,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Failed to write command %s\n", STR(CMD));
		goto exit;
	}
exit:
	return retval < 0 ? -EIO : 0;
}

static int syna_htc_ic_getModeValue(common_data_t *common_data)
{
	int retval = 0;
	unsigned char CMD;
	int cmd_size = 0;
	unsigned char *out_buf;
	unsigned char data_buf;
	unsigned char *resp_buf = NULL;
	struct syna_tcm *tcm_hcd = tcm;
	struct tcm_dev *tcm_dev = tcm_hcd->tcm_dev;
	int mode = common_data->mode;
	u8* value = (u8 *)common_data->data_buf;
	int value_len = common_data->data_len;
	int resp_data_len = 0;
	unsigned int copy_size = 0;
	int i = 0;

	LOGI("mode:%d, value_len:%d, data_buf:%p", mode, value_len, value);

	for (i = 0; i < value_len; ++i)
		LOGD("before copy: common_data.data_buf[%d] = %d", i, common_data->data_buf[i]);

	if (mode < THP_IC_CMD_BASE) {
		LOGE("mode is error!!\n");
		return -EIO;
	}

	if (mode == IC_MODE_44) {
		CMD = (unsigned char)common_data->data_buf[0];
		out_buf = (unsigned char *)&common_data->data_buf[1];
		cmd_size = sizeof(unsigned char);
	} else {
		CMD = CMD_MultiFunction;
		data_buf = (unsigned char )(mode - THP_IC_CMD_BASE);
		out_buf = &data_buf;
		cmd_size = sizeof(unsigned char);
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD,
			out_buf,
			cmd_size,
			cmd_size,
			resp_buf,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Failed to read command %s\n",
				STR(CMD));
		goto exit;
	}

	resp_data_len = tcm_dev->resp_buf.data_length;
	copy_size = MIN(value_len, resp_data_len);

	retval = syna_pal_mem_cpy(value,
			sizeof(common_data->data_buf),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			copy_size);
	if (retval < 0) {
		LOGE("Fail to copy ic read info\n");
		goto exit;
	}

	for (i = 0; i < copy_size; ++i)
		LOGI("after copy: value[%d] = %d, common_data.data_buf[%d] = %d", i, value[i], i, common_data->data_buf[i]);

#ifdef BIG_SMALL_CONVERT
	if (mode != IC_MODE_49) {
		syna_htc_ic_data_convert((char* )value, value_len);
	} else {
		syna_htc_ic_poll_data_convert((char* )value);
	}
	for (i = 0; i < copy_size; ++i) {
		LOGI("after convert: value[%d] = %d, common_data.data_buf[%d] = %d", i, value[i], i, common_data->data_buf[i]);
	}
#endif

exit:
	return retval < 0 ? -EIO : 0;
}
#endif

#ifdef TOUCH_FOD_SUPPORT
static void touch_fod_test(int value, int fod_center_x, int fod_center_y)
{
	struct syna_tcm *tcm_hcd = tcm;
	struct input_dev *input_dev = tcm_hcd->input_dev;

	if (value) {
		input_mt_slot(input_dev, 0);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
		input_report_key(input_dev, BTN_TOUCH, 1);
		input_report_key(input_dev, BTN_TOOL_FINGER, 1);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, 0);
		input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_report_abs(input_dev, ABS_MT_WIDTH_MINOR, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, fod_center_x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, fod_center_y);
		input_sync(input_dev);
	} else {
		input_mt_slot(input_dev, 0);
		input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(input_dev, ABS_MT_WIDTH_MINOR, 0);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
		input_sync(input_dev);
	}
	update_fod_press_status(!!value);
}

static void syna_xiaomi_touch_fod_test(int value)
{
	int fod_center_x;
	int fod_center_y;

	LOGI("fod test value = %d\n", value);

	fod_center_x = xiaomi_bdata.fod_lx + xiaomi_bdata.fod_x_size / 2;
	fod_center_y = xiaomi_bdata.fod_ly + xiaomi_bdata.fod_y_size / 2;

	if (xiaomi_bdata.super_resolution_factor > 0) {
		fod_center_x *= xiaomi_bdata.super_resolution_factor;
		fod_center_y *= xiaomi_bdata.super_resolution_factor;
	}

	touch_fod_test(value, fod_center_x, fod_center_y);
}
#endif

static int syna_ic_self_test(char *type, int *result)
{
	int retval = 0;
	char *self_test_data = NULL;
	struct syna_tcm *tcm_hcd = tcm;

	if (!tcm_hcd)
		return 0;

	if (!strncmp("short", type, 5) || !strncmp("open", type, 4)) {
		if (!tcm_hcd && !tcm_hcd->testing_xiaomi_self_test) {
			LOGE("NULL Pointer!\n");
			retval = 1;
			goto out;
		}

		self_test_data = vmalloc(PAGE_SIZE * 3);
		if (self_test_data == NULL) {
			retval = 1;
			goto out;
		}

#ifdef TOUCH_THP_SUPPORT
	syna_tcm_enable_touch_raw(0);
#endif
		retval = tcm_hcd->testing_xiaomi_self_test(self_test_data);
#ifdef TOUCH_THP_SUPPORT
	syna_tcm_enable_touch_raw(1);
#endif
		if (!retval) {
			LOGE("self test failed, retval = %d\n", retval);
			retval = 1;
			goto out;
		}
		retval = 2;
	} else if (!strncmp("i2c", type, 3)) {
		if (!tcm_hcd->testing_xiaomi_chip_id_read) {
			retval = 1;
			goto out;
		}

		retval = tcm_hcd->testing_xiaomi_chip_id_read(tcm_hcd);
		/* (retval == 2) passed  (retval == 1) failed */
	}
out:
	*result = retval;
	if (self_test_data) {
		vfree(self_test_data);
		self_test_data = NULL;
	}

	return 0;
}

static int syna_tcm_lockdown_info(void)
{
	int retval = 0;
	int i = 0;
	unsigned char *p = NULL;
	struct tcm_buffer rd_data;

	if (!tcm) {
		LOGE("tcm is NULL, please check it\n");
		return -1;
	}

	if (xiaomi_driver_data.lockdown_info_ready) {
		LOGI("lockdown info is ready, not read again from flash\n");
		goto exit;
	}

	if (ATOMIC_GET(tcm->tcm_dev->firmware_flashing)) {
		LOGI("Touch is do fw updating\n");
		retval = wait_for_completion_timeout(&tcm->tcm_dev->fw_update_completion, msecs_to_jiffies(6000));
		if (!retval) {
			LOGE("wait_for_completion_timeout!\n");
			return -1;
		}
	}

	for (i = 0; i < RETRY_CNT; ++i) {
		retval = syna_tcm_read_flash_area(tcm->tcm_dev, AREA_BOOT_CONFIG, &rd_data, DEFAULT_FLASH_READ_DELAY);
		if (retval < 0) {
			LOGE("Failed to read oem_info data\n");
			goto exit;
		}

		for (i = (BOOT_CONFIG_SLOTS - 1); i >= 0; i--) {
			if (rd_data.buf[i * BOOT_CONFIG_SIZE + 0] != 0) {
				p = &rd_data.buf[i * BOOT_CONFIG_SIZE + 0];
				break;
			}
		}

		if (!p) {
			LOG_ERROR("p pointer is NULL, get rd_data buf is error!!\n");
			continue;
		}

		for (i = 0; i < 8; i++) {
			xiaomi_driver_data.lockdown_info[i] = p[i];
			LOGD("p[%d] = 0x%02x, PAGE_SIZE = %lu\n", i , xiaomi_driver_data.lockdown_info[i], PAGE_SIZE);
		}

		xiaomi_driver_data.lockdown_info_ready = true;
		retval = 0;
		break;
	}

exit:
	return retval;
}

static int syna_tcm_lockdown_info_read(u8 lockdown_info[8])
{
	if (xiaomi_driver_data.lockdown_info_ready == true) {
		lockdown_info[0] = xiaomi_driver_data.lockdown_info[0];
		lockdown_info[1] = xiaomi_driver_data.lockdown_info[1];
		lockdown_info[2] = xiaomi_driver_data.lockdown_info[2];
		lockdown_info[3] = xiaomi_driver_data.lockdown_info[3];
		lockdown_info[4] = xiaomi_driver_data.lockdown_info[4];
		lockdown_info[5] = xiaomi_driver_data.lockdown_info[5];
		lockdown_info[6] = xiaomi_driver_data.lockdown_info[6];
		lockdown_info[7] = xiaomi_driver_data.lockdown_info[7];
		return 0;
	} else {
		lockdown_info[0] = 0x00;
		lockdown_info[1] = 0x00;
		lockdown_info[2] = 0x00;
		lockdown_info[3] = 0x00;
		lockdown_info[4] = 0x00;
		lockdown_info[5] = 0x00;
		lockdown_info[6] = 0x00;
		lockdown_info[7] = 0x00;
		return -EFAULT;
	}
}

static int syna_tcm_fw_version_read(char firmware_version[64])
{
	int cnt = 0;
	int length = 64;
	struct syna_tcm *tcm_hcd = tcm;

	if (!tcm_hcd)
		return 0;

	if (tcm_hcd->pwr_state != PWR_ON) {
		LOGE("is power off\n");
		return 0;
	}

	cnt = snprintf(firmware_version, length, "Firmware: %d Cfg: %02x %02x",
			tcm_hcd->tcm_dev->packrat_number,
			tcm_hcd->tcm_dev->app_info.customer_config_id[6] - 48,
			tcm_hcd->tcm_dev->app_info.customer_config_id[7] - 48);
	return cnt;
}

static void syna_get_config(void)
{
	int i = 0;

	for (i = 0; i < xiaomi_bdata.config_array_size; i++) {
		if (xiaomi_driver_data.lockdown_info[7] == xiaomi_bdata.config_array[i].tp_vendor)
			break;
	}
	if (i >= xiaomi_bdata.config_array_size) {
		LOGE("can't find right config, i:%d,arra_size:%zu. use default\n", i, xiaomi_bdata.config_array_size);
		xiaomi_bdata.config_file_name = xiaomi_bdata.synaptics_default_cfg_name;
		xiaomi_bdata.fw_image_name = xiaomi_bdata.synaptics_default_fw_name;
		xiaomi_bdata.test_limit_name = xiaomi_bdata.synaptics_default_limit_name;
		goto exit;
	}
	LOGI("choose config:%d\n", i);
	xiaomi_bdata.config_file_name = xiaomi_bdata.config_array[i].synaptics_cfg_name;
	xiaomi_bdata.fw_image_name = xiaomi_bdata.config_array[i].synaptics_fw_name;
	xiaomi_bdata.test_limit_name = xiaomi_bdata.config_array[i].synaptics_limit_name;

exit:
	LOGI("cfg_name:%s,fw_name:%s,limit_name:%s\n", xiaomi_bdata.config_file_name, xiaomi_bdata.fw_image_name, xiaomi_bdata.test_limit_name);
}

static u8 syna_tcm_panel_vendor_read(void)
{
	return xiaomi_driver_data.lockdown_info[0];
}

static u8 syna_tcm_panel_color_read(void)
{
	return xiaomi_driver_data.lockdown_info[2];
}

static u8 syna_tcm_panel_display_read(void)
{
	return xiaomi_driver_data.lockdown_info[1];
}

static char syna_tcm_touch_vendor_read(void)
{
	return '5';
}

#ifdef TOUCH_THP_SUPPORT
int syna_tcm_enable_touch_raw(int en)
{
	int retval;
	struct syna_tcm *tcm_hcd = tcm;

	LOGI("en:%d\n", en);
	if (!tcm_hcd) {
		LOGE("tcm hcd is null\n");
		return -EINVAL;
	}
	tcm_hcd->enable_touch_raw = en;
	if (en) {
		retval = syna_tcm_enable_report(tcm_hcd->tcm_dev, REPORT_TOUCH, !en);
		if (retval < 0)
			LOGE("failed to disable LBP, en = %d, retval = %d\n", en, retval);
		retval = syna_tcm_enable_report(tcm_hcd->tcm_dev, REPORT_THP, en);
		if (retval < 0)
			LOGE("failed to enable touch raw, en = %d, retval = %d\n", en, retval);
	} else {
		retval = syna_tcm_enable_report(tcm_hcd->tcm_dev, REPORT_THP, en);
		if (retval < 0)
			LOGE("failed to disable touch raw, en = %d, retval = %d\n", en, retval);
		retval = syna_tcm_enable_report(tcm_hcd->tcm_dev, REPORT_TOUCH, !en);
		if (retval < 0)
			LOGE("failed to enable LBP, en = %d, retval = %d\n", en, retval);
	}

	return retval;
}
#endif

int xiaomi_parse_dt(struct device *dev)
{
	int retval;
	u32 value;
	struct property *prop;
	struct device_node *np = dev->of_node;
	struct synaptics_config_info *config_info;
	struct device_node *temp;

	LOGI("enter\n");
	prop = of_find_property(np, "synaptics,x-flip", NULL);
	xiaomi_bdata.x_flip = prop > 0 ? true : false;

	prop = of_find_property(np, "synaptics,y-flip", NULL);
	xiaomi_bdata.y_flip = prop > 0 ? true : false;

	retval = of_property_read_u32(np, "synaptics,max-x", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,max-x property\n");
		return retval;
	} else {
		xiaomi_bdata.max_x = value;
	}

	retval = of_property_read_u32(np, "synaptics,max-y", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,max-y property\n");
		return retval;
	} else {
		xiaomi_bdata.max_y = value;
	}

	retval = of_property_read_u32(np, "synaptics,fod-lx", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,fod-lx property\n");
		return retval;
	} else {
		xiaomi_bdata.fod_lx = value;
	}

	retval = of_property_read_u32(np, "synaptics,fod-ly", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,fod-ly property\n");
		return retval;
	} else {
		xiaomi_bdata.fod_ly = value;
	}

	retval = of_property_read_u32(np, "synaptics,fod-x-size", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,fod-x-size property\n");
		return retval;
	} else {
		xiaomi_bdata.fod_x_size = value;
	}

	retval = of_property_read_u32(np, "synaptics,fod-y-size", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,fod-y-size property\n");
		return retval;
	} else {
		xiaomi_bdata.fod_y_size = value;
	}

	retval = of_property_read_u32(np, "synaptics,rx-num", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,rx-num property\n");
		return retval;
	} else {
		xiaomi_bdata.rx_num = value;
	}

	retval = of_property_read_u32(np, "synaptics,tx-num", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,tx-num property\n");
		return retval;
	} else {
		xiaomi_bdata.tx_num = value;
	}

	retval = of_property_read_u32(np, "synaptics,special-rx-num", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,special-rx-num property, use normal\n");
		xiaomi_bdata.special_rx_num = xiaomi_bdata.rx_num;
	} else {
		xiaomi_bdata.special_rx_num = value;
	}

	retval = of_property_read_u32(np, "synaptics,special-tx-num", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,special-tx-num property, use normal\n");
		xiaomi_bdata.special_tx_num = xiaomi_bdata.tx_num;
	} else {
		xiaomi_bdata.special_tx_num = value;
	}

	retval = of_property_read_u32(np, "synaptics,frame-data-page-size", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,frame-data-page-size property\n");
		return retval;
	} else {
		xiaomi_bdata.frame_data_page_size = value;
	}

	retval = of_property_read_u32(np, "synaptics,frame-data-buf-size", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,frame-data-buf-size property\n");
		return retval;
	} else {
		xiaomi_bdata.frame_data_buf_size = value;
	}

	retval = of_property_read_u32(np, "synaptics,raw-data-page-size", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,raw-data-page-size property\n");
		return retval;
	} else {
		xiaomi_bdata.raw_data_page_size = value;
	}

	retval = of_property_read_u32(np, "synaptics,raw-data-buf-size", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,raw-data-buf-size property\n");
		return retval;
	} else {
		xiaomi_bdata.raw_data_buf_size = value;
	}

	retval = of_property_read_u32(np, "synaptics,super-resolution-factor", &value);
	if (retval < 0) {
		LOGE("Failed to read synaptics,super-resolution-factor property\n");
		xiaomi_bdata.super_resolution_factor = 1;
	} else {
		xiaomi_bdata.super_resolution_factor = value;
	}

	retval = of_property_read_string(np, "synaptics,default-thp-config-name",
					 &xiaomi_bdata.synaptics_default_cfg_name);
	if (retval && (retval != -EINVAL)) {
		xiaomi_bdata.synaptics_default_cfg_name = NULL;
		LOGE("Unable to read default cfg name\n");
	} else {
		LOGI("default cfg_name:%s\n", xiaomi_bdata.synaptics_default_cfg_name);
	}

	retval = of_property_read_string(np, "synaptics,default-fw-image-name",
					 &xiaomi_bdata.synaptics_default_fw_name);
	if (retval && (retval != -EINVAL)) {
		xiaomi_bdata.synaptics_default_fw_name = NULL;
		LOGE("Unable to read default fw name\n");
	} else {
		LOGI("default fw_name:%s\n",  xiaomi_bdata.synaptics_default_fw_name);
	}

	retval = of_property_read_string(np, "synaptics,default-test-limit-name",
					 &xiaomi_bdata.synaptics_default_limit_name);
	if (retval && (retval != -EINVAL)) {
		xiaomi_bdata.synaptics_default_limit_name = NULL;
		LOGE("Unable to read default limit name\n");
	} else {
		LOGI("default limit_name:%s\n", xiaomi_bdata.synaptics_default_limit_name);
	}

	retval = of_property_read_u32(np, "synaptics,config-array-size", (u32 *)&xiaomi_bdata.config_array_size);
	if (retval) {
		LOGE("Unable to get array size\n");
		return retval;
	}

	xiaomi_bdata.config_array = devm_kzalloc(dev, xiaomi_bdata.config_array_size *
					sizeof(struct synaptics_config_info), GFP_KERNEL);

	if (!xiaomi_bdata.config_array) {
		LOGE("Unable to allocate memory\n");
		return -ENOMEM;
	}

	config_info = xiaomi_bdata.config_array;
	for_each_child_of_node(np, temp) {
		retval = of_property_read_u32(temp, "synaptics,tp-vendor", &value);
		if (retval) {
			LOGE("Unable to read tp vendor\n");
		} else {
			config_info->tp_vendor = (u8)value;
			LOGI("tp vendor: %u\n", config_info->tp_vendor);
		}

		retval = of_property_read_string(temp, "synaptics,thp-config-name",
						 &config_info->synaptics_cfg_name);
		if (retval && (retval != -EINVAL)) {
			config_info->synaptics_cfg_name = NULL;
			LOGE("Unable to read cfg name\n");
		} else {
			LOGI("cfg_name:%s\n", config_info->synaptics_cfg_name);
		}

		retval = of_property_read_string(temp, "synaptics,fw-image-name",
						 &config_info->synaptics_fw_name);
		if (retval && (retval != -EINVAL)) {
			config_info->synaptics_fw_name = NULL;
			LOGE("Unable to read fw name\n");
		} else {
			LOGI("fw_name:%s\n",  config_info->synaptics_fw_name);
		}

		retval = of_property_read_string(temp, "synaptics,test-limit-name",
						 &config_info->synaptics_limit_name);
		if (retval && (retval != -EINVAL)) {
			config_info->synaptics_limit_name = NULL;
			LOGE("Unable to read limit name\n");
		} else {
			LOGI("limit_name:%s\n", config_info->synaptics_limit_name);
		}
		config_info++;
	}

	LOGI("complete\n");

	return 0;
}

const char *xiaomi_get_firmware_image_name(void)
{
	LOGI("fw image name %s\n", xiaomi_bdata.fw_image_name);
	return xiaomi_bdata.fw_image_name;
}

const char *xiaomi_get_test_limit_name(void)
{
	LOGI("limit image name %s\n", xiaomi_bdata.test_limit_name);
	return xiaomi_bdata.test_limit_name;
}

int xiaomi_get_super_resolution_factor(void)
{
	return xiaomi_bdata.super_resolution_factor;
}

int xiaomi_get_x_resolution(void)
{
	return xiaomi_bdata.max_x;
}

int xiaomi_get_y_resolution(void)
{
	return xiaomi_bdata.max_y;
}

static int syna_tcm_resume_suspend(bool is_resume, u8 gesture_type)
{
	int result = 0;
#ifdef  CONFIG_TRUSTED_TOUCH
	struct qts_data *qts_data = NULL;
	qts_data = get_qts_data_helper(&qts_vendor_data);
#endif

	if (tcm == NULL || tcm->tp_probe_success == 0)
		return -1;
	if (is_resume) {
#ifdef  CONFIG_TRUSTED_TOUCH
		qts_ts_resume(qts_data);
#endif
		result = tcm->dev_resume(&tcm->pdev->dev);
		syna_set_ic_mode(SET_BASE_REFRESH_INTERVAL_TIME, NULL);
		return result;
	}

#ifdef  CONFIG_TRUSTED_TOUCH
		qts_ts_suspend(qts_data);
#endif
	return tcm->dev_suspend(&tcm->pdev->dev);
}
#ifdef CONFIG_TRUSTED_TOUCH
static int syna_tcm_enable_touch_irq(void *data, bool enable)
{
	struct syna_tcm  *tcm =  (struct syna_tcm *)data;
	if (IS_ERR_OR_NULL(tcm))
		return -EINVAL;
	return 	tcm->hw_if->ops_enable_irq(tcm->hw_if,enable);
}
static int syna_tcm_get_irq_num(void *data)
{
	struct syna_tcm *tcm =  (struct syna_tcm *)data;
	if (IS_ERR_OR_NULL(tcm))
		return -EINVAL;
	return tcm->hw_if->bdata_attn.irq_id;
}

static int syna_tcm_pre_la_tui_enable(void *data)
{
	struct syna_tcm *tcm =  (struct syna_tcm *)data;
	if (IS_ERR_OR_NULL(tcm))
		return -EINVAL;
	syna_tcm_enable_touch_raw(0);
	xiaomi_driver_data.tui_process = true;
	reinit_completion(&xiaomi_driver_data.tui_finish);
	 return 0;
}
static int syna_tcm_post_la_tui_disable(void *data)
{
	struct syna_tcm *tcm =  (struct syna_tcm *)data;
	if (IS_ERR_OR_NULL(tcm))
		return -EINVAL;
	syna_tcm_enable_touch_raw(1);
	complete_all(&xiaomi_driver_data.tui_finish);
	xiaomi_driver_data.tui_process = false;
	 return 0;
}

static int syna_tcm_pre_la_tui_disable(void *data)
{
	struct syna_tcm *tcm =  (struct syna_tcm *)data;
	if (IS_ERR_OR_NULL(tcm))
		return -EINVAL;
	 return 0;
}
static int syna_tcm_post_la_tui_enable(void *data)
{
	struct syna_tcm *tcm =  (struct syna_tcm *)data;
	if (IS_ERR_OR_NULL(tcm))
		return -EINVAL;
	return 0;
}
static void syna_tcm_fill_qts_vendor_data(struct qts_vendor_data *qts_vendor_data,
                 struct syna_tcm *tcm)
{
	struct spi_device *spi = tcm->hw_if->pdev;
	struct device_node *node = spi->dev.of_node;
	const char *touch_type;
	int rc = 0;

	rc = of_property_read_string(node, "syna,touch-type", &touch_type);
	if (rc) {
		LOGE("No touch type\n");
		return;
	}

	if (!strcmp(touch_type, "primary"))
		qts_vendor_data->client_type = QTS_CLIENT_PRIMARY_TOUCH;
	else
		qts_vendor_data->client_type = QTS_CLIENT_SECONDARY_TOUCH;

	qts_vendor_data->client = NULL;
	qts_vendor_data->spi = spi;
	qts_vendor_data->bus_type = QTS_BUS_TYPE_SPI;

	qts_vendor_data->vendor_data = tcm;
	qts_vendor_data->qts_vendor_ops.enable_touch_irq = syna_tcm_enable_touch_irq;
	qts_vendor_data->qts_vendor_ops.get_irq_num = syna_tcm_get_irq_num;
	qts_vendor_data->qts_vendor_ops.pre_la_tui_enable = syna_tcm_pre_la_tui_enable;
	qts_vendor_data->qts_vendor_ops.post_la_tui_enable = syna_tcm_post_la_tui_enable;
	qts_vendor_data->qts_vendor_ops.pre_la_tui_disable = syna_tcm_pre_la_tui_disable;
	qts_vendor_data->qts_vendor_ops.post_la_tui_disable = syna_tcm_post_la_tui_disable;
}
#endif
static int syna_tcm_set_touch_multi_function(struct tcm_dev *tcm_dev,
		unsigned char *config, unsigned int config_size)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned char *data;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if ((!config) || (config_size == 0)) {
		LOGE("Invalid given config data\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Not in application fw mode, mode: %d\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	data = syna_pal_mem_alloc(config_size, sizeof(unsigned char));
	if (!data) {
		LOGE("Fail to allocate memory for touch config setting\n");
		return -ERR_NOMEM;
	}

	retval = syna_pal_mem_cpy(data,
			config_size,
			config,
			config_size,
			config_size);
	if (retval < 0) {
		LOGE("Fail to copy custom touch config\n");
		goto exit;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_MultiFunction,
			data,
			config_size,
			config_size,
			&resp_code,
			RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Fail to write command CMD_MultiFunction\n");
		goto exit;
	}

exit:
	if (data)
		syna_pal_mem_free((void *)data);

	return retval;
}

static int syna_tcm_set_cmd_noresponse(struct tcm_dev *tcm_dev,
		unsigned char *config, unsigned int config_size)
{
	int retval = 0;
	unsigned char *data;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if ((!config) || (config_size == 0)) {
		LOGE("Invalid given config data\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Not in application fw mode, mode: %d\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	data = syna_pal_mem_alloc(config_size, sizeof(unsigned char));
	if (!data) {
		LOGE("Fail to allocate memory for touch config setting\n");
		return -ERR_NOMEM;
	}
	syna_pal_mutex_lock(&tcm_dev->msg_data.cmd_mutex);
	syna_pal_mutex_lock(&tcm_dev->msg_data.rw_mutex);
	retval = syna_pal_mem_cpy(data,
			config_size,
			config,
			config_size,
			config_size);
	if (retval < 0) {
		LOGE("Fail to copy custom touch config\n");
		goto exit;
	}
	retval = syna_tcm_write(tcm_dev, data, config_size);
	if (retval < 0) {
		LOGE("Fail to write data\n");
		goto exit;
	}
exit:
	if (data)
		syna_pal_mem_free((void *)data);
	syna_pal_mutex_unlock(&tcm_dev->msg_data.rw_mutex);
	syna_pal_mutex_unlock(&tcm_dev->msg_data.cmd_mutex);
	return retval;
}

#ifdef TOUCH_THP_SUPPORT
static void syna_set_fod_downup(struct syna_tcm *tcm_hcd, int enable)
{
	LOGI("enable:%d\n", enable);
	if (enable) {
		update_fod_press_status(1);
	} else {
		update_fod_press_status(0);
		tcm_hcd->fod_finger = false;
	}
}

static void syna_set_ic_mode(enum set_ic_mode_enum mode, int *value)
{
	unsigned char out_buf[11] = {0};
	int out_buf_size = 0;
	int retval;
	struct syna_tcm *tcm_hcd = tcm;

	if (!tcm_hcd) {
		LOGE("%s tcm_hcd is null\n", __func__);
		return;
	}
	if (tcm_hcd->pwr_state != PWR_ON) {
		LOGI("%s pwr_state is not PWR_ON\n", __func__);
		return;
	}
	if (mode == ENTER_IDLE_MODE && value) {
		out_buf[0] = 0x43;
		out_buf[1] = 0x00;
		out_buf[2] = 0x00;
		if (!value[1]) {
			out_buf[1] = (value[0] == 0x00 ? 0x01 : value[0]);
			out_buf[3] = (25 & 0xFF);
			out_buf[4] = ((25 >> 8) & 0xFF);
			out_buf[7] = (4167 & 0xFF);
			out_buf[8] = ((4167 >> 8) & 0xFF);
		} else {
			out_buf[2] = (value[0] == 0x00 ? 0x01 : value[0]);
			out_buf[5] = (value[2] & 0xFF);
			out_buf[6] = ((value[2] >> 8) & 0xFF);
			out_buf[9] = (value[1] & 0xFF);
			out_buf[10] = ((value[1] >> 8) & 0xFF);
		}
		out_buf_size = 11;
	} else if (mode == UPDATE_IDLE_BASELINE) {
		out_buf[0] = 0x30;
		out_buf[1] = 0x01;
		out_buf[2] = 0x00;
		out_buf_size = 3;
	} else if (mode == SET_BASE_REFRESH_INTERVAL_TIME) {
		out_buf[0] = 0x31;
		out_buf[1] = 10;
		out_buf[2] = 0x00;
		out_buf_size = 3;
	} else if (mode == SET_DYNAMIC_REPORT_RATE) {
		out_buf[0] = 0x07;
		out_buf[1] = ((value[0] == 1) ? 60 : 120);
		out_buf[2] = 0x00;
		out_buf_size = 3;
	} else if (mode == SET_DOZE_WAKEUP_THRESHOLD) {
		out_buf[0] = 0x39;
		out_buf[1] = value[0];
		out_buf[2] = 0x00;
		out_buf_size = 3;
		LOGD("%s out_buf:%x,%x,%x\n", __func__, out_buf[0], out_buf[1], out_buf[2]);
	} else if (mode == SET_THP_BASE_STATUS) {
		out_buf[0] = 0xC8;
		out_buf[1] = 0x03;
		out_buf[2] = 0x00;
		out_buf[3] = 0x3A;
		out_buf[4] = value[0] ? 0x04 : 0x03;
		out_buf[5] = 0x00;
		LOGD("%s out_buf:%x,%x,%x,%x,%x,%x\n", __func__, out_buf[0], out_buf[1], out_buf[2], out_buf[3], out_buf[4], out_buf[5]);
		syna_tcm_set_cmd_noresponse(tcm_hcd->tcm_dev, out_buf, 6);
		return;
	} else if (mode == SET_THP_GLOVE_STATUS) {
		out_buf[0] = 0xC8;
		out_buf[1] = 0x07;
		out_buf[2] = 0x00;
		out_buf[3] = 0x5c;
		out_buf[4] = (value[0] & 0xFF);
		out_buf[5] = ((value[0] >> 8) & 0xFF);
		out_buf[6] = (value[1] & 0xFF);
		out_buf[7] = ((value[1] >> 8) & 0xFF);
		out_buf[8] = (value[2] & 0xFF);
		out_buf[9] = ((value[2] >> 8) & 0xFF);
		LOGD("%s out_buf:%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", __func__, out_buf[0], out_buf[1], out_buf[2], out_buf[3], out_buf[4], out_buf[5], out_buf[6], out_buf[7], out_buf[8], out_buf[9]);
		syna_tcm_set_cmd_noresponse(tcm_hcd->tcm_dev, out_buf, 10);
		return;
	} else {
		LOGE("unkow mode %d, return\n", mode);
		return;
	}

	retval = syna_tcm_set_touch_multi_function(tcm_hcd->tcm_dev, out_buf, out_buf_size);
	if (retval < 0) {
		LOGE("%s Failed to write command %s, mode = %d\n", __func__, STR(CMD_MultiFunction), mode);
	} else {
		LOGD("%s ok\n", __func__);
	}
}

static void syna_set_report_rate(int value)
{
	unsigned char out_buf[3] = {0x07, 0x78, 0x00};
	int retval;
	struct syna_tcm *tcm_hcd = tcm;

	if (!tcm_hcd) {
		LOGE("%s tcm_hcd is null\n", __func__);
		return;
	}
	if (tcm_hcd->pwr_state != PWR_ON) {
		LOGE("Touch is in not in PWR_ON mode, don't need to set report rate\n");
		return;
	}
	switch (value) {
	case 60:
		out_buf[1] = 0x3C;
		break;
	case 68:
		out_buf[1] = 0x44;
		break;
	case 120:
		out_buf[1] = 0x78;
		break;
	case 135:
		out_buf[1] = 0x87;
		break;
	case 240:
		out_buf[1] = 0xF0;
		break;
	case 300:
		out_buf[1] = 0x2c;
		out_buf[2] = 0x01;
		break;
	default:
		break;
	}
	retval = syna_tcm_set_touch_multi_function(tcm_hcd->tcm_dev, out_buf, sizeof(out_buf));
	if (retval < 0) {
		LOGE("%s Failed to write command %s\n", __func__, STR(CMD_MultiFunction));
	} else {
		LOGI("%s ok\n", __func__);
	}
}

#ifdef CONFIG_TOUCH_FACTORY_BUILD
static void thp_send_cmd_to_hal(s8 touch_id , int cmd, int value)
{
    add_common_data_to_buf(touch_id, SET_CUR_VALUE, cmd, 1, &value);
}
#endif

static void syna_tmd_signal_work(struct work_struct *work)
{
	int retval = 0;

#ifdef CONFIG_TOUCH_FACTORY_BUILD
	LOGI("notify hal to enable fod!\n");
	thp_send_cmd_to_hal(0, DATA_MODE_10, 1);
#endif
	if (ATOMIC_GET(tcm->tcm_dev->firmware_flashing)) {
		LOGI("Touch is do fw updating\n");
		retval = wait_for_completion_timeout(&tcm->tcm_dev->fw_update_completion, msecs_to_jiffies(6000));
		if (!retval) {
			LOGE("wait_for_completion_timeout!\n");
			return;
		}
	}
	syna_tcm_enable_touch_raw(1);
	return;
}

int syna_tcm_report_thp_frame(struct syna_tcm *tcm_hcd, s64 irq_start_time)
{
	struct tp_frame *thp_frame = NULL;
	unsigned int frame_length = 0;
	static u64 thp_cnt = 0;
	struct timespec64 ts;
	struct tcm_buffer *pevent_data = NULL;
#ifdef SYNA_CRC_CHECK
	uint32_t ap_ic_crc, ic_crc;
	int32_t ap_thp_crc;
	int crc_check_size = (HAL_ROW_NUM * HAL_COL_NUM + HAL_ROW_NUM + HAL_COL_NUM) * 2 + 64;
    struct tp_raw* tp_raw = NULL;
#endif

	pevent_data = &tcm_hcd->event_data;
	if ((pevent_data == NULL) || (pevent_data->buf == NULL)) {
		LOGE("Returned, invalid event data pointer\n");
		return -EIO;
	}
	if (pevent_data->data_length == 0) {
		LOGE("Returned, invalid event data length = 0\n");
		return -EIO;
	}

	thp_frame = (struct tp_frame *)get_raw_data_base(TOUCH_ID);
	if (thp_frame == NULL) {
		LOGE("Returned, no memory\n");
		return -ENOMEM;
	}

	frame_length = pevent_data->data_length;
	memcpy(thp_frame->thp_frame_buf, pevent_data->buf, frame_length);
	ktime_get_real_ts64(&ts);
	thp_frame->time_ns = timespec64_to_ns(&ts);
	thp_frame->frm_cnt = thp_cnt++;
	thp_frame->fod_pressed = tcm_hcd->fod_finger;
	thp_frame->fod_trackingId = 0;
	add_input_event_timeline_before_event_time(0, thp_frame->frm_cnt, irq_start_time, ktime_get());
	notify_raw_data_update(TOUCH_ID);
#ifdef SYNA_CRC_CHECK
	tp_raw = (struct tp_raw*)((thp_frame->thp_frame_buf));
	ic_crc = tp_raw->crc;
	ap_ic_crc = tp_ic_bcc_check(&((int *)(thp_frame->thp_frame_buf))[5], crc_check_size / 4 - 5);
	ap_thp_crc = tp_thp_crc32_check(&((int *)(thp_frame->thp_frame_buf))[5], crc_check_size / 4 - 5);
	LOGI("ap_ic_crc:%u, ic_crc:%u, ap_thp_crc:%u\n", ap_ic_crc, ic_crc, ap_thp_crc);
	if (ap_ic_crc == ic_crc) {
		tp_raw->crc = ap_thp_crc;
		tp_raw->crc_r = -(ap_thp_crc + 1);
	} else {
		tp_raw->crc = ap_thp_crc + 10;
		tp_raw->crc_r = -(ap_thp_crc + 10 + 1);
	}
#endif
	return 0;
}

#endif

int syna_tcm_set_gesture_type(struct syna_tcm *tcm, u8 val)
{
	int retval = 0;

	/*set gesture type*/
	if (tcm->hw_if->ops_enable_irq)
		tcm->hw_if->ops_enable_irq(tcm->hw_if, true);

	if ((val & GESTURE_SINGLETAP_EVENT))
		tcm->gesture_type |= (0x0001 << 13);
	else
		tcm->gesture_type &= ~(0x0001 << 13);
	if ((val & GESTURE_DOUBLETAP_EVENT))
		tcm->gesture_type |= 0x0001;
	else
		tcm->gesture_type &= ~0x0001;

	retval = syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_GESTURE_TYPE_ENABLE, tcm->gesture_type, RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Failed to enable gesture type\n");
		return retval;
	} else {
		LOGI("set gesture type:%x\n", tcm->gesture_type);
	}
	if ((val & GESTURE_LONGPRESS_EVENT)) {
		LOGI("set touch and hold enable control\n");
		/*set 0:disable fod
		 * set 1: enable fod in lpwg
		 * set 3:enable fod in lpwg and in active*/
		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_ENABLE_TOUCH_AND_HOLD, 1, RESP_IN_ATTN);
		if (retval < 0) {
			LOGE("FOD_PRESS_EVENT touch and hold enable control failed\n");
			return retval;
		}
	} else {
		LOGI("set touch and hold disable control\n");
		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_ENABLE_TOUCH_AND_HOLD, 0, RESP_IN_ATTN);
		if (retval < 0) {
			LOGE("FOD_PRESS_EVENT touch and hold disable control failed\n");
			return retval;
		}
	}
	return retval;
}

static void syna_tcm_switch_mode(u8 gesture_type)
{
	int retval, i = 0, retry = 3;

	LOGI("enter\n");
#ifdef CONFIG_TOUCH_FACTORY_BUILD
	LOGI("factory version,skip set gesture mode\n");
	return;
#endif
#ifdef CONFIG_TRUSTED_TOUCH
	if (xiaomi_driver_data.tui_process) {
		if (wait_for_completion_interruptible(&xiaomi_driver_data.tui_finish)) {
			LOGE("cautious, ERESTARTSYS may cause cmd loss recomand try again\n");
			return ;
		}
		LOGN("wait finished, its time to go ahead\n");
	}
#endif
	if (tcm == NULL) {
		LOGE("tcm is null\n");
		return;
	}
	if (gesture_type < 0) {
		LOGE("get geture type error\n");
		return;
	}
	if (tcm->fod_finger && tcm->pwr_state != PWR_ON && !(gesture_type & GESTURE_LONGPRESS_EVENT)) {
		LOGI("fod pressed, skip set gesture mode\n");
		return;
	}
	for (i = 0; i < retry; i++) {
		retval = syna_tcm_set_gesture_type(tcm, gesture_type);
		if (retval < 0) {
			LOGE("set gesture type error, retry\n");
			msleep(10);
		} else {
			LOGI("set gesture type ok\n");
			break;
		}
	}
	if (retval < 0) {
		LOGE("set gesture type error\n");
		return;
	}
	if (tcm->pwr_state == PWR_ON) {
		LOGI("in resume mode,don't need to set gesture mode\n");
		return;
	}

	if (gesture_type == 0) {
		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_ENABLE_WAKEUP_GESTURE_MODE, 0, RESP_IN_ATTN);
		if (retval < 0) {
			LOGE("Failed to disable wakeup gesture mode\n");
			return;
		} else {
			LOGI("clear DC_ENABLE_WAKEUP_GESTURE_MODE\n");
		}
		retval = syna_tcm_sleep(tcm->tcm_dev, true);
		if (retval < 0) {
			LOGE("Fail to enter deep sleep\n");
			return;
		} else {
			LOGI("Enter sleep mode\n");
			tcm->pwr_state = PWR_OFF;
		}
	} else {
		retval = syna_tcm_sleep(tcm->tcm_dev, false);
		if (retval < 0) {
			LOGE("Fail to exit deep sleep\n");
			return;
		} else {
			LOGI("Exit sleep mode\n");
		}
		if (!tcm->irq_wake) {
			enable_irq_wake(tcm->hw_if->bdata_attn.irq_id);
			tcm->irq_wake = true;
		}
		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_ENABLE_WAKEUP_GESTURE_MODE, 1, RESP_IN_ATTN);
		if (retval < 0) {
			LOGE("Failed to enable wakeup gesture mode\n");
			return;
		} else {
			LOGI("enter DC_ENABLE_WAKEUP_GESTURE_MODE\n");
			tcm->pwr_state = LOW_PWR;
		}
	}
}
static void syna_tcm_enable_irq(bool enable)
{
	if (tcm == NULL) {
		LOGE("tcm is null\n");
		return;
	}
	if (tcm->hw_if->ops_enable_irq) {
		LOGD("enable:%d\n", enable);
		tcm->hw_if->ops_enable_irq(tcm->hw_if, enable);
	} else {
		return;
	}
	if (enable) {
		if (!tcm->irq_wake) {
			enable_irq_wake(tcm->hw_if->bdata_attn.irq_id);
			tcm->irq_wake = true;
		}
	} else {
		if (tcm->irq_wake) {
			disable_irq_wake(tcm->hw_if->bdata_attn.irq_id);
			tcm->irq_wake = false;
		}
	}
}

static void syna_tcm_set_charge_state(int state)
{
	int retval = 0;
	unsigned short val = 0;

	if (tcm == NULL) {
		LOGE("tcm is null\n");
		return;
	}
	if (tcm->pwr_state != PWR_ON) {
		LOGI("in suspend mode,don't need to set charge state\n");
		return;
	}
#ifdef CONFIG_TRUSTED_TOUCH
	if (xiaomi_driver_data.tui_process) {
		if (wait_for_completion_interruptible(&xiaomi_driver_data.tui_finish)) {
			LOGE("cautious, ERESTARTSYS may cause cmd loss recomand try again\n");
			return ;
		}
		LOGN("wait finished, its time to go ahead\n");
	}
#endif
	if (ATOMIC_GET(tcm->tcm_dev->firmware_flashing)) {
			LOGI("Touch is do fw updating\n");
			retval = wait_for_completion_timeout(&tcm->tcm_dev->fw_update_completion, msecs_to_jiffies(6000));
			if (!retval) {
				LOGE("wait_for_completion_timeout!\n");
				return;
			}
	}
	val = state & 0x01; /* Default Value: 0, disconnected: 1, connected */
	retval = syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_ENABLE_CHARGER_CONNECTED, val, RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Failed to set charge mode\n");
	} else {
		LOGI("set charger_connected, value = 0x%02x\n", val);
	}
}

static void syna_tcm_set_cur_value(int mode, int *value)
{
	struct syna_tcm *tcm_hcd = tcm;
	struct input_dev *input_dev = tcm_hcd->input_dev;

	if (value[0] < 0) {
		LOGE("error value [%d]\n", value[0]);
		return;
	}
#ifdef CONFIG_TRUSTED_TOUCH
	if (xiaomi_driver_data.tui_process) {
		if (wait_for_completion_interruptible(&xiaomi_driver_data.tui_finish)) {
			LOGE("cautious, ERESTARTSYS may cause cmd loss recomand try again");
			return ;
		}
		LOGN("wait finished, its time to go ahead");
	}
#endif
	if (mode != DATA_MODE_134)
		LOGI("mode:%d,value:%d\n", mode, value[0]);
	else
		LOGD("mode:%d,value:%d\n", mode, value[0]);
#ifdef TOUCH_THP_SUPPORT
	if (mode == DATA_MODE_44) {
		syna_set_ic_mode(ENTER_IDLE_MODE, value);
		return;
	}
	if (mode == DATA_MODE_45) {
		if (tcm_hcd->enable_touch_raw)
			syna_set_fod_downup(tcm_hcd, value[0]);
		return;
	}
	if (mode == DATA_MODE_127) {
		if (tcm_hcd->enable_touch_raw)
			syna_set_ic_mode(SET_DOZE_WAKEUP_THRESHOLD, value);
		return;
	}
	if (mode == DATA_MODE_46) {
		//TODO
		return;
	}
	if (mode == DATA_MODE_47) {
		//TODO
		return;
	}
	if (mode == DATA_MODE_48) {
		schedule_delayed_work(&tcm_hcd->signal_work, 1 * HZ);
		return;
	}

	if (mode == DATA_MODE_55) {
		syna_set_report_rate(value[0]);
		if (tcm_hcd->pwr_state == PWR_ON && tcm_hcd->hw_if->ops_enable_irq)
			tcm_hcd->hw_if->ops_enable_irq(tcm_hcd->hw_if, true);
		return;
	}
	if (mode == DATA_MODE_60) {
		//TODO
		return;
	}
	if (mode == DATA_MODE_115) {
		syna_set_ic_mode(UPDATE_IDLE_BASELINE, value);
		return;
	}
	if (mode == DATA_MODE_129) {
		syna_set_ic_mode(SET_DYNAMIC_REPORT_RATE, value);
		return;
	}
	if (mode == DATA_MODE_134) {
		syna_set_ic_mode(SET_THP_BASE_STATUS, value);
		return;
	}
	if (mode == DATA_MODE_135) {
		syna_set_ic_mode(SET_THP_GLOVE_STATUS, value);
		return;
	}
	if (mode == DATA_MODE_32) {
		if ((xiaomi_get_gesture_type(TOUCH_ID) & GESTURE_DOUBLETAP_EVENT) && value[0] == GESTURE_ID_DOUBLE_TAP) {
			LOGI("Double Tap\n");
			input_report_key(input_dev, KEY_WAKEUP, 1);
			input_sync(input_dev);
			input_report_key(input_dev, KEY_WAKEUP, 0);
			input_sync(input_dev);
		}
		if ((xiaomi_get_gesture_type(TOUCH_ID) & GESTURE_SINGLETAP_EVENT) && value[0] == GESTURE_ID_SINGLE_TAP) {
			LOGI("Single Tap\n");
			input_report_key(input_dev, KEY_GOTO, 1);
			input_sync(input_dev);
			input_report_key(input_dev, KEY_GOTO, 0);
			input_sync(input_dev);
		}
		return;
	}
#endif
}

static void syna_tcm_set_fod_value(s32 *value, int length)
{
	int gesture_id, fod_x, fod_y, fod_overlap, fod_area;

	if (tcm == NULL || length != FOD_VALUE_LEN) {
		LOGE("abnomal fod event\n");
		return;
	}
	gesture_id = value[0];
	fod_x = value[1];
	fod_y = value[2];
	fod_x *= xiaomi_get_super_resolution_factor();
	fod_y *= xiaomi_get_super_resolution_factor();
	fod_area = value[3];
	fod_overlap = value[4];
	LOGD("fod event,gesture_id:%d,x:%d,y:%d,area:%d,overlap:%d\n", gesture_id, fod_x, fod_y, fod_area, fod_overlap);
	if (gesture_id == GESTURE_TOUCH_AND_HOLD_DOWN_EVENT || gesture_id == GESTURE_TOUCH_AND_HOLD_MOVE_EVENT) {
		if ((xiaomi_get_gesture_type(TOUCH_ID) & GESTURE_LONGPRESS_EVENT) != 0) {
			syna_touch_fod_down_event(tcm, fod_x, fod_y, fod_overlap, fod_area);
		} else if (gesture_id == GESTURE_TOUCH_AND_HOLD_DOWN_EVENT) {
			LOGI("skip fod event because fod is not enabled\n");
		}
	} else if (gesture_id == GESTURE_TOUCH_AND_HOLD_UP_EVENT) {
		/* Todo: add customer FOD action. */
		if (tcm->fod_finger) {
			syna_touch_fod_up_event(tcm);
		}
	}
}

static int syna_tcm_touch_doze_analysis(int input)
{
	int result = 0;
	struct syna_tcm *tcm_hcd = tcm;
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;
#ifdef TOUCH_SENSORHUB_SUPPORT
	struct spi_device *spi = (struct spi_device *)tcm->hw_if->pdev;
#endif

	switch(input) {
		case TOUCH_0:
			tcm_hcd->doze_test = true;
			schedule_resume_suspend_work(TOUCH_ID, false);
			schedule_resume_suspend_work(TOUCH_ID, true);
			tcm_hcd->doze_test = false;
		break;
#ifdef STARTUP_REFLASH
		case TOUCH_1:
			syna_dev_reflash_startup(tcm, true);
#endif
		break;
		case TOUCH_2:
			enable_irq(attn->irq_id);
			attn->irq_enabled = true;
		break;
		case TOUCH_3:
			disable_irq_nosync(attn->irq_id);
			attn->irq_enabled = false;
		break;
		case TOUCH_4:
			/* release irq */
			if (tcm_hcd->hw_if->bdata_attn.irq_id)
				tcm->dev_release_irq(tcm_hcd);
			/* register the interrupt handler */
			result = tcm->dev_request_irq(tcm_hcd);
			if (result < 0) {
				LOGE("Fail to request the interrupt line\n");
			} else {
				result = tcm_hcd->hw_if->ops_enable_irq(tcm_hcd->hw_if, true);
				if (result < 0)
					LOGE("Failed to enable interrupt\n");
			}
		break;
		case TOUCH_5:
			result = gpio_get_value(attn->irq_gpio) == 0 ? 0 : 1;
		break;
	#ifdef TOUCH_SENSORHUB_SUPPORT
		case TOUCH_10:
			xiaomi_set_sensorhub_nonui_enable(true);
		break;
		case TOUCH_11:
			xiaomi_set_sensorhub_nonui_enable(false);
		break;
		case TOUCH_12:
		result = pm_runtime_get_sync(spi->master->dev.parent);
		if (result < 0) {
			LOGE("Fail to failed to get sync, retval = %d\n", result);
			return result;
		}
		break;
		case TOUCH_13:
		result = pm_runtime_put_sync(spi->master->dev.parent);
		if (result < 0) {
			LOGE("Fail to failed to put sync, retval = %d\n", result);
			return result;
		}
		break;
	#endif
		default:
			LOGE("Don't support touch doze analysis\n");
		break;
	}
	return result;
}

static int syna_tcm_touch_log_level_control(bool flag)
{
	debug_log_flag = flag;
	return 0;
}

#ifdef SYNAPTICS_DEBUGFS_ENABLE
static void syna_tcm_dbg_suspend(struct syna_tcm *tcm_hcd, bool enable)
{
	if (enable) {
		schedule_resume_suspend_work(TOUCH_ID, false);
	} else {
		schedule_resume_suspend_work(TOUCH_ID, true);
	}
}

static int syna_tcm_dbg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t syna_tcm_dbg_read(struct file *file, char __user *buf, size_t size,
				loff_t *ppos)
{
	const char *str = "cmd support as below:\n \
	\necho \"irq-disable\" or \"irq-enable\" to ctrl irq\n \
	\necho \"tp-suspend-en\" or \"tp-suspend-off\" to ctrl panel in or off suspend status\n \
	\necho \"tp-sd-en\" or \"tp-sd-off\" to ctrl panel in or off sleep status\n";

	loff_t pos = *ppos;
	int len = strlen(str);

	if (pos < 0)
		return -EINVAL;
	if (pos >= len)
		return 0;

	if (copy_to_user(buf, str, len))
		return -EFAULT;

	*ppos = pos + len;

	return len;
}

static ssize_t syna_tcm_dbg_write(struct file *file, const char __user *buf,
					size_t size, loff_t *ppos)
{
	char *cmd = kzalloc(size + 1, GFP_KERNEL);
	int ret = size;
	struct syna_tcm *tcm_hcd = tcm;

	if (!cmd)
		return -ENOMEM;

	if (copy_from_user(cmd, buf, size)) {
		ret = -EFAULT;
		goto out;
	}

	cmd[size] = '\0';

	if (!strncmp(cmd, "irq-disable", 11)) {
		LOGI("touch irq is disabled!\n");
		tcm_hcd->hw_if->ops_enable_irq(tcm_hcd->hw_if,false);
	} else if (!strncmp(cmd, "irq-enable", 10)) {
		LOGI("touch irq is enabled!\n");
		tcm_hcd->hw_if->ops_enable_irq(tcm_hcd->hw_if,true);
	} else if (!strncmp(cmd, "tp-sd-en", 8)) {
		tcm_hcd->doze_test = true;
		schedule_resume_suspend_work(TOUCH_ID, false);
		tcm_hcd->doze_test = false;
	} else if (!strncmp(cmd, "tp-sd-off", 9)) {
		tcm_hcd->doze_test = true;
		schedule_resume_suspend_work(TOUCH_ID, true);
		tcm_hcd->doze_test = false;
	} else if (!strncmp(cmd, "tp-suspend-en", 13))
		syna_tcm_dbg_suspend(tcm_hcd, true);
	else if (!strncmp(cmd, "tp-suspend-off", 14))
		syna_tcm_dbg_suspend(tcm_hcd, false);
out:
	kfree(cmd);

	return ret;
}

static int syna_tcm_dbg_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static const struct file_operations tpdbg_operations = {
	.owner = THIS_MODULE,
	.open = syna_tcm_dbg_open,
	.read = syna_tcm_dbg_read,
	.write = syna_tcm_dbg_write,
	.release = syna_tcm_dbg_release,
};
#endif

void syna_xiaomi_touch_probe(struct syna_tcm *syna_tcm)
{
	int retry_get_lockdown_info = 5;
	struct spi_device *spi = NULL;
#ifdef CONFIG_TRUSTED_TOUCH
	struct device_node *node = NULL;
#endif

	memset(&xiaomi_driver_data, 0, sizeof(xiaomi_driver_data_t));
	if (syna_tcm == NULL)
		return;
	tcm = syna_tcm;
	spi = tcm->hw_if->pdev;
#ifdef CONFIG_TRUSTED_TOUCH
	node = spi->dev.of_node;
#endif
	/* get lockdown info */
	while (retry_get_lockdown_info > 0) {
		if (syna_tcm_lockdown_info() >= 0)
			break;
		LOGE("read lockdown info has error, retry %d\n", retry_get_lockdown_info);
		msleep(1000);
		retry_get_lockdown_info--;
	}
	syna_get_config();

	INIT_DELAYED_WORK(&tcm->signal_work, syna_tmd_signal_work);

	hardware_param.x_resolution = xiaomi_bdata.max_x;
	hardware_param.y_resolution = xiaomi_bdata.max_y;
	hardware_param.rx_num = xiaomi_driver_data.lockdown_info[7] == 0 ?
		xiaomi_bdata.rx_num : xiaomi_bdata.special_rx_num;
	hardware_param.tx_num = xiaomi_driver_data.lockdown_info[7] == 0 ?
		xiaomi_bdata.tx_num : xiaomi_bdata.special_tx_num;
	hardware_param.super_resolution_factor = xiaomi_bdata.super_resolution_factor;
	hardware_param.frame_data_page_size = xiaomi_bdata.frame_data_page_size;
	hardware_param.frame_data_buf_size = xiaomi_bdata.frame_data_buf_size;
	hardware_param.raw_data_page_size = xiaomi_bdata.raw_data_page_size;
	hardware_param.raw_data_buf_size = xiaomi_bdata.raw_data_buf_size;

	memset(hardware_param.config_file_name, 0, 64);
	memset(hardware_param.driver_version, 0, 64);
	memset(hardware_param.fw_version, 0, 64);
	memcpy(hardware_param.config_file_name,
		xiaomi_bdata.config_file_name,
		strlen(xiaomi_bdata.config_file_name));
	memcpy(hardware_param.driver_version, SYNAPTICS_DRIVER_VERSION, strlen(SYNAPTICS_DRIVER_VERSION));
	syna_tcm_fw_version_read(hardware_param.fw_version);

	memset(&hardware_operation, 0, sizeof(hardware_operation_t));
	hardware_operation.ic_resume_suspend = syna_tcm_resume_suspend;
	hardware_operation.ic_self_test = syna_ic_self_test;
	hardware_operation.ic_data_collect = NULL;
	hardware_operation.ic_get_lockdown_info = syna_tcm_lockdown_info_read;
	hardware_operation.ic_get_fw_version = syna_tcm_fw_version_read;
	hardware_operation.set_mode_value = syna_tcm_set_cur_value;
	hardware_operation.ic_set_fod_value = syna_tcm_set_fod_value;
	hardware_operation.cmd_update_func = NULL;
	hardware_operation.set_mode_long_value = NULL;

	hardware_operation.palm_sensor_write = NULL;

	hardware_operation.panel_vendor_read = syna_tcm_panel_vendor_read;
	hardware_operation.panel_color_read = syna_tcm_panel_color_read;
	hardware_operation.panel_display_read = syna_tcm_panel_display_read;
	hardware_operation.touch_vendor_read = syna_tcm_touch_vendor_read;
	hardware_operation.get_touch_ic_buffer = NULL;
	hardware_operation.touch_doze_analysis = syna_tcm_touch_doze_analysis;
	hardware_operation.touch_log_level_control = syna_tcm_touch_log_level_control;
#ifdef TOUCH_THP_SUPPORT
	hardware_operation.enable_touch_raw = syna_tcm_enable_touch_raw;
#ifdef TOUCH_THP_DEBUG
	hardware_operation.htc_ic_setModeValue = syna_htc_ic_setModeValue;
	hardware_operation.htc_ic_getModeValue = syna_htc_ic_getModeValue;
#endif
#ifdef TOUCH_FOD_SUPPORT
	hardware_operation.xiaomi_touch_fod_test = syna_xiaomi_touch_fod_test;
#endif
	hardware_operation.ic_switch_mode = syna_tcm_switch_mode;
	hardware_operation.ic_enable_irq = syna_tcm_enable_irq;
	hardware_operation.ic_set_charge_state = syna_tcm_set_charge_state;
#endif
	register_touch_panel(&spi->dev, TOUCH_ID, &hardware_param, &hardware_operation);
#if defined(TOUCH_PLATFORM_XRING)
	xiaomi_register_panel_notifier(&spi->dev, TOUCH_ID,
		XRING_PANEL_EVENT_TAG_PRIMARY, XRING_PANEL_EVENT_CLIENT_PRIMARY_TOUCH);
#else
	xiaomi_register_panel_notifier(&spi->dev, TOUCH_ID,
		PANEL_EVENT_NOTIFICATION_PRIMARY, PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH);
#endif
	syna_tcm_enable_touch_raw(0);
#ifdef CONFIG_TRUSTED_TOUCH
	if (of_property_read_bool(node, "syna,qts_en")) {
		syna_tcm_fill_qts_vendor_data(&qts_vendor_data, tcm);
		if (qts_client_register(qts_vendor_data))
			LOGE("qts client register failed\n");
	}
	init_completion(&xiaomi_driver_data.tui_finish);
	xiaomi_driver_data.tui_process = false;
#endif

#ifdef SYNAPTICS_DEBUGFS_ENABLE
	tcm->debugfs = debugfs_create_dir("tp_debug", NULL);
	if (tcm->debugfs) {
		debugfs_create_file("switch_state", 0660, tcm->debugfs, tcm,
					&tpdbg_operations);
	}
#endif
}

void syna_xiaomi_touch_remove(struct syna_tcm *syna_tcm)
{
	struct spi_device *spi_dev = syna_tcm->hw_if->pdev;

	cancel_delayed_work_sync(&tcm->signal_work);
	xiaomi_unregister_panel_notifier(&spi_dev->dev, TOUCH_ID);
	unregister_touch_panel(TOUCH_ID);
}
