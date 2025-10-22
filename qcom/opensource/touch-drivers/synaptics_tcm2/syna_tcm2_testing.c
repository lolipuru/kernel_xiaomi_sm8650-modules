// SPDX-License-Identifier: GPL-2.0
/*
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/**
 * @file syna_tcm2_testing.c
 *
 * This file implements the sample code to perform chip testing.
 */
#include "syna_tcm2_testing.h"
#include "syna_tcm2_testing_limits.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_base.h"

/* g_testing_dir represents the root folder of testing sysfs
 */
static struct kobject *g_testing_dir;
static struct syna_tcm *g_tcm_ptr;
static struct testing_hcd *testing_hcd;
static struct proc_dir_entry *proc_selftest = NULL;
char *g_save_buf = NULL;
static short rawMin[720] = {0};
static short rawMax[720] = {0};
static short rawDiffPT85[720] = {0};
static short rawDiffPT86[720] = {0};
static bool result_pt85 = true;
static bool result_pt86 = true;

#define DATA_SIZE_MAX    (30)
#define PT1_PT3_LIMITS_BYTES_SIZE  (8)
#define PT10_LIMITS_BYTES_SIZE     (1)
#define PT44_LIMITS_BYTES_SIZE     (1)
#define SYNA_TCM_TESTING_LIMITS_FILE_NAME  "def_test_limits_S3910P.csv"
#define CSV_PT1_TESTING_LIMITS             "PT1_TRx_TRx_short_test"
#define CSV_PT3_TESTING_LIMITS             "PT3_TRX_GND_short_test"
#define CSV_PT5_TESTING_LIMITS_MIN         "PT5_Full_raw_cap_test_min"
#define CSV_PT5_TESTING_LIMITS_MAX         "PT5_Full_raw_cap_test_max"
#define CSV_PT10_TESTING_LIMITS            "PT10_(Mutal)_noise_test"
#define CSV_PT18_TESTING_LIMITS_MIN        "PT18_Abs_raw_cap_test_min"
#define CSV_PT18_TESTING_LIMITS_MAX        "PT18_Abs_raw_cap_test_max"
#define CSV_PT22_TESTING_LIMITS_MIN        "PT22_Trans_raw_cap_test_min"
#define CSV_PT22_TESTING_LIMITS_MAX        "PT22_Trans_raw_cap_test_max"
#define CSV_GAP_DIFF_TESTING_LIMITS_MAX    "Gap_Diff_test_max"
#define CSV_PT44_TESTING_LIMITS            "PT44_(Mutal)_noise_test"
#define CSV_PT81_TESTING_RX_STDEV_LIMITS   "PT81_Pixel_uniformity_test_Rx_stdev"
#define CSV_PT81_TESTING_SUM_STDEV_LIMITS  "PT81_Pixel_uniformity_test_sum_stdev"
#define CSV_PT85_TESTING_RAW_SHIFT_LIMITS  "PT85_Switch_screen_Raw_shift_max"
#define CSV_PT86_TESTING_RAW_SHIFT_LIMITS  "PT86_Switch_frequency_Raw_shift_max"

#define CHECK_BIT(var, pos) ((var) & (1<<(pos)))
#define ABS(x)           (((x) < 0) ? (-(x)) : (x))
#define SAVE_BUF_SIZE    (4096*5)
#define RESULT_INFO_LEN  (300)

static inline void uint_to_le1(unsigned char *dest, unsigned int val)
{
	dest[0] = val%0x100;
}

static inline void uint_to_le2(unsigned char *dest, unsigned int val)
{
	dest[0] = val%0x100;
	dest[1] = val/0x100;
}

static inline void uint_to_le4(unsigned char *dest, unsigned int val)
{
	unsigned int vl, vh;
	vl = val%0x10000;
	vh = val/0x10000;
	uint_to_le2(dest, vl);
	uint_to_le2(dest + 2, vh);
}

static inline unsigned int le2_to_uint(const unsigned char *src)
{
	return (unsigned int)src[0] +
			(unsigned int)src[1] * 0x100;
}

static inline unsigned int le4_to_uint(const unsigned char *src)
{
	return (unsigned int)src[0] +
			(unsigned int)src[1] * 0x100 +
			(unsigned int)src[2] * 0x10000 +
			(unsigned int)src[3] * 0x1000000;
}


static int testing_get_limits_bytes(unsigned int *row, unsigned int *col, enum tcm_test_code code)
{
	unsigned int rows;
	unsigned int cols;
	unsigned int size = 0;
	struct tcm_application_info *app_info;

	app_info = &g_tcm_ptr->tcm_dev->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	switch(code) {
		case TEST_PID01_TRX_TRX_SHORTS:
		case TEST_PID03_TRX_GROUND_SHORTS:
			*row = 1;
			*col = PT1_PT3_LIMITS_BYTES_SIZE;
			size = PT1_PT3_LIMITS_BYTES_SIZE;
			break;
		case TEST_PID05_FULL_RAW_CAP:
			*row = rows;
			*col = cols;
			size = rows*cols*2;
			break;
		case TEST_PID22_TRANS_CAP_RAW:
			*row = rows;
			*col = cols;
			size = rows*cols*2;
			break;
		case TEST_PID44_MULTI_Gear_NOISE_TEST:
			*row = 1;
			*col = PT44_LIMITS_BYTES_SIZE;
			size = PT44_LIMITS_BYTES_SIZE*2;
			break;
		case TEST_PID18_HYBRID_ABS_RAW:
			*row = 1;
			*col = rows + cols;
			size = (rows + cols)*4;
			break;
		case TEST_PID81_PIXEL_UNIFORMITY:
			*row = 1;
			*col = cols;
			size = cols * 2;
			break;
		case TEST_PID85_RAWSHIFTTEST:
			*row = rows;
			*col = cols;
			size = rows*cols*2;
			break;
		case TEST_PID86_RAWSHIFTTEST:
			*row = rows;
			*col = cols;
			size = rows*cols*2;
			break;
		default:
			*row = rows;
			*col = cols;
			size = 0;
			break;
	}

	return size;
}

static void goto_next_line(char **p)
{
	while (**p != '\n') {
		if (**p == '\0') {
			return;
		}
		*p = *p + 1;
	}
	*p = *p + 1;
	return;
}

static int testing_copy_valid_data(struct tcm_buffer *dest_buffer, char *src_buf, enum tcm_test_code code)
{
	int retval = 0;
	int i, j, n;
	int offset = 0;
	long data;
	unsigned int byte_cnt = 0;
	unsigned int limit_cnt = 0;
	char *pdest = dest_buffer->buf;
	char *psrc = src_buf;
	char data_buf[DATA_SIZE_MAX+2] = {0};
	unsigned int limit_rows;
	unsigned int limit_cols;
	unsigned int buf_size_bytes = 0;

	if (dest_buffer == NULL || src_buf == NULL) {
		LOGE("Invalid src or dest pointer\n");
		retval = -EINVAL;
		goto exit;
	}

	buf_size_bytes = testing_get_limits_bytes(&limit_rows, &limit_cols, code);
	if (buf_size_bytes == 0) {
		LOGE("Failed to get buf_size_bytes for PT%d\n",code);
		retval = -EINVAL;
		goto exit;
	}

	if (buf_size_bytes > dest_buffer->buf_size) {
		LOGE("Incorrect buf_size[%d] for PT%d, expected buf bytes size[%d]\n",
				dest_buffer->buf_size, code, buf_size_bytes);
		retval = -EINVAL;
		goto exit;
	}

	byte_cnt = 0;
	limit_cnt = 0;
	for (i = 0; i < limit_rows; i++) {
		for (j = 0; j < limit_cols; j++) {
			/* get one data */
			n = 0;
			data_buf[0] = '\0';
			while (n < DATA_SIZE_MAX) {
				if (*psrc == '\n' || *psrc == '\0' || *psrc == '\r') {
					break;
				} else if (*psrc == ',') {
					psrc++;
					break;
				} else if (*psrc == ' ') {
					psrc++;
					continue;
				} else {
					data_buf[n] = *psrc;
					n++;
					psrc++;
				}
			}
			data_buf[n] = '\0';

			if (strlen(data_buf) == 0)
				continue;

			if (kstrtol(data_buf, 0, &data)) {
				LOGE("Err to convert string(%s) to a long value for PT%d\n",
					data_buf, code);
				retval = -EINVAL;
				goto exit;
			}

			if (code == TEST_PID18_HYBRID_ABS_RAW) {
				uint_to_le4(pdest, data);
				offset = 4;
			} else if (code == TEST_PID01_TRX_TRX_SHORTS ||
						code == TEST_PID03_TRX_GROUND_SHORTS) {
				uint_to_le1(pdest, data);
				offset = 1;
			} else {
				uint_to_le2(pdest, data);
				offset = 2;
			}

			pdest += offset;
			byte_cnt += offset;
			limit_cnt += 1;
		}
		goto_next_line(&psrc);
	}

	if (byte_cnt != buf_size_bytes) {
		LOGE("Incorrect valid limit bytes size[%d], expected bytes size[%d]\n",
			byte_cnt, buf_size_bytes);
		goto exit;
	}

	if (limit_cnt != (limit_rows*limit_cols)) {
		LOGE("Incorrect valid limit data size[%d], expected size[%d]\n",
			limit_cnt, limit_rows*limit_cols);
		goto exit;
	}

	dest_buffer->data_length = byte_cnt;

exit:
	return retval;
}


static int testing_parse_csv_data(struct tcm_buffer *dest_buffer, char *src_buf,
						char *name, enum tcm_test_code code)
{
	int retval;
	char *psrc = src_buf;

	if (!dest_buffer || !src_buf || !name) {
		retval = -EINVAL;
		LOGE("Invalid buf pointer for %s\n", name);
		goto exit;
	}

	psrc = strstr(psrc, name);
	if (!psrc) {
		retval = -EINTR;
		LOGE( "search %s failed\n", name);
		goto exit;
	}

	goto_next_line(&psrc);
	if (!psrc || (strlen(psrc) == 0)) {
		retval = -EIO;
		LOGE("there is no valid data for %s\n", name);
		goto exit;
	}

	retval = testing_copy_valid_data(dest_buffer, psrc, code);
	if(retval < 0) {
		LOGE("load PT%d %s data: Failed[%d]\n", code, name, retval);
		goto exit;
	} else {
		LOGI("load PT%d %s data: Success\n", code, name);
	}

exit:
	return retval;
}


static int testing_load_testlimits(enum tcm_test_code testcode, unsigned int gapdiff)
{
	int retval = 0;
	char *csv_buf = NULL;
	char limit_file_name[100] = {0};
	char test_name0[100] = {0};
	char test_name1[100] = {0};
	unsigned int rows;
	unsigned int cols;
	unsigned int buf_size_bytes = 0;
	const struct firmware *firmware = NULL;
	struct tcm_buffer *dest_buffer0 = NULL;
	struct tcm_buffer *dest_buffer1 = NULL;

	switch(testcode) {
		case TEST_PID01_TRX_TRX_SHORTS:
			sprintf(test_name0, "%s", CSV_PT1_TESTING_LIMITS);
			dest_buffer0 = &testing_hcd->pt_hi_limits;
			dest_buffer1 = NULL;
			break;
		case TEST_PID03_TRX_GROUND_SHORTS:
			sprintf(test_name0, "%s", CSV_PT3_TESTING_LIMITS);
			dest_buffer0 = &testing_hcd->pt_hi_limits;
			dest_buffer1 = NULL;
			break;
		case TEST_PID05_FULL_RAW_CAP:
			sprintf(test_name0, "%s", CSV_PT5_TESTING_LIMITS_MIN);
			dest_buffer0 = &testing_hcd->pt_lo_limits;

			sprintf(test_name1, "%s", CSV_PT5_TESTING_LIMITS_MAX);
			dest_buffer1 = &testing_hcd->pt_hi_limits;
			break;
		case TEST_PID22_TRANS_CAP_RAW:
			if (gapdiff) {
				sprintf(test_name0, "%s", CSV_GAP_DIFF_TESTING_LIMITS_MAX);
				dest_buffer0 = &testing_hcd->pt_hi_limits;
			} else {
				sprintf(test_name0, "%s", CSV_PT22_TESTING_LIMITS_MIN);
				dest_buffer0 = &testing_hcd->pt_lo_limits;

				sprintf(test_name1, "%s", CSV_PT22_TESTING_LIMITS_MAX);
				dest_buffer1 = &testing_hcd->pt_hi_limits;
			}
			break;
		case TEST_PID44_MULTI_Gear_NOISE_TEST:
			sprintf(test_name0, "%s", CSV_PT44_TESTING_LIMITS);
			dest_buffer0 = &testing_hcd->pt_hi_limits;
			dest_buffer1 = NULL;
			break;
		case TEST_PID18_HYBRID_ABS_RAW:
			sprintf(test_name0, "%s", CSV_PT18_TESTING_LIMITS_MIN);
			dest_buffer0 = &testing_hcd->pt_lo_limits;

			sprintf(test_name1, "%s", CSV_PT18_TESTING_LIMITS_MAX);
			dest_buffer1 = &testing_hcd->pt_hi_limits;
			break;
		case TEST_PID81_PIXEL_UNIFORMITY:
			sprintf(test_name0, "%s", CSV_PT81_TESTING_RX_STDEV_LIMITS);
			dest_buffer0 = &testing_hcd->pt_lo_limits;

			sprintf(test_name1, "%s", CSV_PT81_TESTING_SUM_STDEV_LIMITS);
			dest_buffer1 = &testing_hcd->pt_hi_limits;
			break;
		case TEST_PID85_RAWSHIFTTEST:
			sprintf(test_name0, "%s", CSV_PT85_TESTING_RAW_SHIFT_LIMITS);
			dest_buffer0 = &testing_hcd->pt_hi_limits;
			dest_buffer1 = NULL;
			break;
		case TEST_PID86_RAWSHIFTTEST:
			sprintf(test_name0, "%s", CSV_PT86_TESTING_RAW_SHIFT_LIMITS);
				dest_buffer0 = &testing_hcd->pt_hi_limits;
			dest_buffer1 = NULL;
			break;
		default:
			dest_buffer0 = NULL;
			dest_buffer1 = NULL;
			break;
	}

	if (!dest_buffer0) {
		LOGE("Limit data is not in csv file for [PT%d]\n", testcode);
		goto exit;
	}

	buf_size_bytes = testing_get_limits_bytes(&rows, &cols, testcode);
	if (buf_size_bytes == 0) {
		LOGE("Failed to get buf size for PT%d\n", testcode);
		retval = -EINVAL;
		goto exit;
	}

	/* read limit csv file*/
	if (xiaomi_get_test_limit_name() != NULL)
		sprintf(limit_file_name, "%s", xiaomi_get_test_limit_name());
	else
		sprintf(limit_file_name, "%s", SYNA_TCM_TESTING_LIMITS_FILE_NAME);
	LOGI("limit_file_name:%s.\n", limit_file_name);

	retval = request_firmware(&firmware,
				limit_file_name, &g_tcm_ptr->pdev->dev);
	if (retval < 0) {
		LOGE("Failed to request limit file:%s\n",limit_file_name);
		goto exit;
	}

	if (firmware->size == 0) {
		LOGE("request_firmware, limits file length error\n");
		retval = -EINVAL;
		goto exit;
	}

	csv_buf = kzalloc(firmware->size + 1, GFP_KERNEL);
	if (!csv_buf) {
		LOGE("Failed to allocate memory for csv_buf\n");
		retval = -ENOMEM;
		goto exit;
	}
	memcpy(csv_buf, firmware->data, firmware->size);

	if (dest_buffer0) {
		/* allocate the mem for limits buffer */
		retval = syna_tcm_buf_alloc( dest_buffer0, buf_size_bytes);
		if (retval < 0) {
			LOGE("Failed to allocate memory for PT%d buf\n", testcode);
			goto exit;
		}

		/* load the limits value */
		retval = testing_parse_csv_data(dest_buffer0, csv_buf, test_name0, testcode);
		if (retval < 0) {
			LOGE("Failed to read limts data for %s [PT%d] from csv file\n",test_name0, testcode);
			goto exit;
		}
	}

	if (dest_buffer1) {
		/* allocate the mem for limits buffer */
		retval = syna_tcm_buf_alloc(dest_buffer1, buf_size_bytes);;
		if (retval < 0) {
			LOGE("Failed to allocate memory for PT%d buf\n", testcode);
			goto exit;
		}

		/* load the limits */
		retval = testing_parse_csv_data(dest_buffer1, csv_buf, test_name1, testcode);
		if (retval < 0) {
			LOGE("Failed to read limts data for %s [PT%d] from csv file\n",test_name1, testcode);
			goto exit;
		}
	}

exit:
	if (csv_buf)
		kfree(csv_buf);

	if (firmware)
		release_firmware(firmware);

	return retval;
}

static void testing_get_frame_size_words(unsigned int *size, bool image_only)
{
	unsigned int rows;
	unsigned int cols;
	unsigned int hybrid;
	unsigned int buttons;
	struct tcm_application_info *app_info;

	app_info = &g_tcm_ptr->tcm_dev->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);
	hybrid = le2_to_uint(app_info->has_hybrid_data);
	buttons = le2_to_uint(app_info->num_of_buttons);

	*size = rows * cols;

	if (!image_only) {
		if (hybrid)
			*size += rows + cols;
		*size += buttons;
	}

	return;
}

static unsigned int testing_save_output(char *out_buf, unsigned int offset, char *pstr)
{
	unsigned int data_len;
	unsigned int cnt, sum;
	int data;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int idx;
	unsigned char *buf = NULL;
	struct tcm_application_info *app_info;

	sum = offset;
	if (out_buf == NULL) {
		LOGI("Do not support save ouput data\n");
		goto exit;
	}

	app_info = &g_tcm_ptr->tcm_dev->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	LOCK_BUFFER(testing_hcd->output);
	data_len = testing_hcd->output.data_length;
	if (data_len == 0)
		goto unlockbuffer;

	cnt = 0;
	if (pstr == NULL) {
		cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "PT%d Test Result = %s\n",
						testing_hcd->test_item, (testing_hcd->result)?"pass":"fail");
	} else {
		cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "%s Test Result = %s\n",
						pstr, (testing_hcd->result)?"pass":"fail");
	}
	sum += cnt;
	if (data_len == (rows*cols*2)) {
		cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "Rows=%d,Cols=%d\n", rows, cols);
		sum += cnt;
	}

	/* print data */
	buf = testing_hcd->output.buf;
	if (data_len == (rows*cols*2)) {
		idx = 0;
		for (row = 0; row < rows; row++) {
			cnt = 0;
			for (col = 0; col < cols; col++) {
				data = (short)((unsigned short)(buf[idx] & 0xff) |
						(unsigned short)(buf[idx+1] << 8));
				cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "%d,", data);
				sum += cnt;

				idx += 2;
			}

			cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "\n");
			sum += cnt;
		}
	} else if ((data_len == ((rows + cols)*4)) && (testing_hcd->test_item == TEST_PID18_HYBRID_ABS_RAW)){
		idx = 0;
		cnt = 0;

		for (col = 0; col < (cols + rows); col++) {
			data = (int)(buf[idx] & 0xff) |
					(int)(buf[idx+1] << 8) |
					(int)(buf[idx+2] << 16) |
					(int)(buf[idx+3] << 24);

			cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "%d,", data);
			sum += cnt;

			idx+=4;
		}

		cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "\n");
		sum += cnt;
	} else if (data_len <= ((rows + cols)*2)) {
		idx = 0;
		cnt = 0;
		if (testing_hcd->test_item == TEST_PID81_PIXEL_UNIFORMITY) {
			for (col = 0; col < cols; col++) {
        			data = (unsigned short)(buf[idx] & 0xff) |
					(unsigned short)(buf[idx+1] << 8);
				cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "%d,", data);
				sum += cnt;
				idx+=2;
			}
			cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "\n");
			sum += cnt;
			idx = 0;
			cnt = 0;
			for (col = 0; col < cols - 2; col++) {
        			data = (unsigned short)(buf[idx] & 0xff) |
					(unsigned short)(buf[idx+1] << 8);
				data += (unsigned short)(buf[idx+ 2] & 0xff) |
					(unsigned short)(buf[idx+3] << 8);
				data += (unsigned short)(buf[idx+ 4] & 0xff) |
					(unsigned short)(buf[idx+5] << 8);
				cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "%d,", data);
				sum += cnt;

				idx+=2;
			}

			cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "\n");
			sum += cnt;
		} else {
			for (col = 0; col < data_len; col++) {
				cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "0x%02X,", buf[idx]);
				sum += cnt;

				idx += 1;
			}

			cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "\n");
			sum += cnt;
		}
	} else if (data_len == ((rows + 1) * cols * 2)) {
		/* pt05+rt163 */
		idx = 0;
		for (row = 0; row < rows + 1; row++) {
			cnt = 0;
			for (col = 0; col < cols; col++) {
				data = (short)((unsigned short)(buf[idx] & 0xff) |
						(unsigned short)(buf[idx+1] << 8));
				cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "%d,", data);
				sum += cnt;

				idx += 2;
			}

			cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "\n");
			sum += cnt;

			if(row == rows - 1)
			{
				cnt = snprintf(out_buf + sum, SAVE_BUF_SIZE - sum, "PT84:\n");
				sum += cnt;
			}
		}
	} else{
		LOGE("Invalid data\n");
	}

unlockbuffer:
	UNLOCK_BUFFER(testing_hcd->output);
exit:
	return sum;
}


static void testing_copy_resp_to_output(void)
{
	int retval;
	unsigned int output_size;

	output_size = testing_hcd->resp.data_length;

	LOCK_BUFFER(testing_hcd->output);

	retval = syna_tcm_buf_alloc(&testing_hcd->output,output_size);
	if (retval < 0) {
		LOGE("Failed to allocate memory for testing_hcd->output.buf\n");
		UNLOCK_BUFFER(testing_hcd->output);
		return;
	}
	retval = syna_tcm_buf_copy(&testing_hcd->output,&testing_hcd->resp);
	if (retval < 0) {
		LOGE("Failed to copy test resp data\n");
		UNLOCK_BUFFER(testing_hcd->output);
		return;
	}

	testing_hcd->output.data_length = output_size;

	UNLOCK_BUFFER(testing_hcd->output);

	return;
}


/**
 * syna_testing_compare_byte_vector()
 *
 * Sample code to compare the test result with limits
 * by byte vector
 *
 * @param
 *    [ in] data: target test data
 *    [ in] data_size: size of test data
 *    [ in] limit: test limit value to be compared with
 *    [ in] limit_size: size of test limit
 *
 * @return
 *    on success, true; otherwise, return false
 */
/*
 *static bool syna_testing_compare_byte_vector(unsigned char *data,
 *		unsigned int data_size, const unsigned char *limit,
 *		unsigned int limit_size)
 *{
 *	bool result = false;
 *	unsigned char tmp;
 *	unsigned char p, l;
 *	int i, j;
 *
 *	if (!data || (data_size == 0)) {
 *		LOGE("Invalid test data\n");
 *		return false;
 *	}
 *	if (!limit || (limit_size == 0)) {
 *		LOGE("Invalid limits\n");
 *		return false;
 *	}
 *
 *	if (limit_size < data_size) {
 *		LOGE("Limit size mismatched, data size: %d, limits: %d\n",
 *			data_size, limit_size);
 *		return false;
 *	}
 *
 *	result = true;
 *	for (i = 0; i < data_size; i++) {
 *		tmp = data[i];
 *
 *		for (j = 0; j < 8; j++) {
 *			p = GET_BIT(tmp, j);
 *			l = GET_BIT(limit[i], j);
 *			if (p != l) {
 *				LOGE("Fail on TRX-%03d (data:%X, limit:%X)\n",
 *					(i*8 + j), p, l);
 *				result = false;
 *			}
 *		}
 *	}
 *
 *	return result;
 *}
 */

/**
 * syna_testing_compare_frame()
 *
 * Sample code to compare the test result with limits
 * being formatted as a frame
 *
 * @param
 *    [ in] data: target test data
 *    [ in] data_size: size of test data
 *    [ in] rows: the number of rows
 *    [ in] cols: the number of column
 *    [ in] limits_hi: upper-bound test limit
 *    [ in] limits_lo: lower-bound test limit
 *
 * @return
 *    on success, true; otherwise, return false
 */
/*
 *static bool syna_testing_compare_frame(unsigned char *data,
 *		unsigned int data_size, int rows, int cols,
 *		const short *limits_hi, const short *limits_lo)
 *{
 *	bool result = false;
 *	short *data_ptr = NULL;
 *	short limit;
 *	int i, j;
 *
 *	if (!data || (data_size == 0)) {
 *		LOGE("Invalid test data\n");
 *		return false;
 *	}
 *
 *	if (data_size < (2 * rows * cols)) {
 *		LOGE("Size mismatched, data:%d (exppected:%d)\n",
 *			data_size, (2 * rows * cols));
 *		result = false;
 *		return false;
 *	}
 *
 *	if (rows > LIMIT_BOUNDARY) {
 *		LOGE("Rows mismatched, rows:%d (exppected:%d)\n",
 *			rows, LIMIT_BOUNDARY);
 *		result = false;
 *		return false;
 *	}
 *
 *	if (cols > LIMIT_BOUNDARY) {
 *		LOGE("Columns mismatched, cols: %d (exppected:%d)\n",
 *			cols, LIMIT_BOUNDARY);
 *		result = false;
 *		return false;
 *	}
 *
 *	result = true;
 *
 *	if (!limits_hi)
 *		goto end_of_upper_bound_limit;
 *
 *	data_ptr = (short *)&data[0];
 *	for (i = 0; i < rows; i++) {
 *		for (j = 0; j < cols; j++) {
 *			limit = limits_hi[i * LIMIT_BOUNDARY + j];
 *			if (*data_ptr > limit) {
 *				LOGE("Fail on (%2d,%2d)=%5d, limits_hi:%4d\n",
 *					i, j, *data_ptr, limit);
 *				result = false;
 *			}
 *			data_ptr++;
 *		}
 *	}
 *
 *end_of_upper_bound_limit:
 *
 *	if (!limits_lo)
 *		goto end_of_lower_bound_limit;
 *
 *	data_ptr = (short *)&data[0];
 *	for (i = 0; i < rows; i++) {
 *		for (j = 0; j < cols; j++) {
 *			limit = limits_lo[i * LIMIT_BOUNDARY + j];
 *			if (*data_ptr < limit) {
 *				LOGE("Fail on (%2d,%2d)=%5d, limits_lo:%4d\n",
 *					i, j, *data_ptr, limit);
 *				result = false;
 *			}
 *			data_ptr++;
 *		}
 *	}
 *
 *end_of_lower_bound_limit:
 *	return result;
 *}
 */
/**
 * syna_testing_device_id()
 *
 * Sample code to ensure the device id is expected
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_device_id(struct syna_tcm *tcm)
{
	int retval;
	bool result;
	struct tcm_identification_info info;
	char *strptr = NULL;

	LOGI("Start testing\n");

	retval = syna_tcm_identify(tcm->tcm_dev, &info);
	if (retval < 0) {
		LOGE("Fail to get identification\n");
		result = false;
		goto exit;
	}

	strptr = strnstr(info.part_number,
					device_id_limit,
					sizeof(info.part_number));
	if (strptr != NULL)
		result = true;
	else {
		LOGE("Device ID mismatched, FW: %s (limit: %s)\n",
			info.part_number, device_id_limit);
		result = false;
	}

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	/*2-pass, 1-fail*/
	return ((result) ? 2 : 1);
}

/**
 * syna_testing_config_id()
 *
 * Sample code to ensure the config id is expected
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_config_id(struct syna_tcm *tcm)
{
	int retval;
	bool result;
	struct tcm_application_info info;
	int idx;

	LOGI("Start testing\n");

	retval = syna_tcm_get_app_info(tcm->tcm_dev, &info);
	if (retval < 0) {
		LOGE("Fail to get app info\n");
		result = false;
		goto exit;
	}

	result = true;
	for (idx = 0; idx < sizeof(config_id_limit); idx++) {
		if (config_id_limit[idx] != info.customer_config_id[idx]) {
			LOGE("Fail on byte.%d (data: %02X, limit: %02X)\n",
				idx, info.customer_config_id[idx],
				config_id_limit[idx]);
			result = false;
		}
	}

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_check_id_show()
 *
 * Attribute to show the result of ID comparsion to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_check_id_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		retval = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	count = 0;

	retval = syna_testing_device_id(tcm);

	retval = snprintf(buf, PAGE_SIZE - count,
			"Device ID check: %s\n",
			(retval < 0) ? "fail" : "pass");

	buf += retval;
	count += retval;

	retval = syna_testing_config_id(tcm);

	retval = snprintf(buf, PAGE_SIZE - count,
			"Config ID check: %s\n",
			(retval < 0) ? "fail" : "pass");

	buf += retval;
	count += retval;

	retval = count;
exit:
	return retval;
}

static struct kobj_attribute kobj_attr_check_id =
	__ATTR(check_id, 0444, syna_testing_check_id_show, NULL);

/**
 * syna_testing_pt01()
 *
 * Sample code to perform PT01 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt01(struct syna_tcm *tcm)
{
	int retval;
	int i, j;
	int phy_pin;
	bool do_pin_test = false;
	unsigned int limits_size;
	unsigned int limits_row;
	unsigned int limits_col;
	unsigned char *buf;
	unsigned char data;
	unsigned char pt1_limits;

	testing_hcd->result = false;


	LOGI("Start testing pt01\n");

	LOCK_BUFFER(testing_hcd->pt_hi_limits);
	limits_size = testing_get_limits_bytes(&limits_row, &limits_col, TEST_PID01_TRX_TRX_SHORTS);
	retval = testing_load_testlimits(TEST_PID01_TRX_TRX_SHORTS, 0);
	if (retval < 0) {
		LOGE("Failed to load PT%d limits from csv file\n", TEST_PID01_TRX_TRX_SHORTS);
		retval = -EINVAL;
		goto exit;
	}

	if (limits_size != testing_hcd->pt_hi_limits.data_length) {
		LOGE( "Mismatching limits size\n");
	}

	LOCK_BUFFER(testing_hcd->resp);

	testing_hcd->test_item = TEST_PID01_TRX_TRX_SHORTS;
	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID01_TRX_TRX_SHORTS,
			&testing_hcd->resp);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID01_TRX_TRX_SHORTS);
		UNLOCK_BUFFER(testing_hcd->resp);
		goto exit;
	}

	
	if (limits_size < testing_hcd->resp.data_length) {
		LOGE("Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	for (i = 0; i < testing_hcd->resp.data_length; i++) {
		data = buf[i];
		pt1_limits = testing_hcd->pt_hi_limits.buf[i];
		LOGI("[%d]: 0x%02x, limit[0x%02x]\n",i, data, pt1_limits);
		for (j = 0; j < 8; j++) {
			phy_pin = (i*8 + j);

			do_pin_test = true;

			if (do_pin_test) {
				if (CHECK_BIT(data, j) != CHECK_BIT(pt1_limits, j)){
					LOGE("pin-%2d : fail\n", phy_pin);
					testing_hcd->result = false;
				}
				else
					LOGD("pin-%2d : pass\n", phy_pin);
			}
		}
	}

	testing_copy_resp_to_output();

	UNLOCK_BUFFER(testing_hcd->resp);

exit:
	LOGI("Result = %s\n", (testing_hcd->result)?"pass":"fail");
	UNLOCK_BUFFER(testing_hcd->pt_hi_limits);

	return retval;
}

/**
 * syna_testing_pt01_show()
 *
 * Attribute to show the result of PT01 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt01_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt01(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT01: %s\n",
			testing_hcd->result? "Passed" : "Failed");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt01 =
	__ATTR(pt01, 0444, syna_testing_pt01_show, NULL);

	static int syna_testing_pt03(struct syna_tcm *tcm)
{
	int retval;
	int i, j;
	int phy_pin;
	unsigned int limits_size;
	unsigned int limits_row;
	unsigned int limits_col;
	unsigned char data;
	unsigned char pt3_limits;

	testing_hcd->result = false;

	LOGI("Start testing pt03\n");

	LOCK_BUFFER(testing_hcd->pt_hi_limits);
	limits_size = testing_get_limits_bytes(&limits_row, &limits_col, TEST_PID03_TRX_GROUND_SHORTS);
	retval = testing_load_testlimits(TEST_PID03_TRX_GROUND_SHORTS, 0);
	if (retval < 0) {
		LOGE("Failed to load PT%d limits from csv file\n", TEST_PID03_TRX_GROUND_SHORTS);
		retval = -EINVAL;
		goto exit;
	}

	if (limits_size != testing_hcd->pt_hi_limits.data_length) {
		LOGE( "Mismatching limits size\n");
	}

	LOCK_BUFFER(testing_hcd->resp);

	testing_hcd->test_item = TEST_PID03_TRX_GROUND_SHORTS;
	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID03_TRX_GROUND_SHORTS,
			&testing_hcd->resp);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID03_TRX_GROUND_SHORTS);
		UNLOCK_BUFFER(testing_hcd->resp);
		goto exit;
	}


	if (limits_size < testing_hcd->resp.data_length) {
		LOGE("Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	testing_hcd->result = true;

	for (i = 0; i < testing_hcd->resp.data_length; i++) {

		data = testing_hcd->resp.buf[i];
		pt3_limits = testing_hcd->pt_hi_limits.buf[i];
		LOGI("[%d]: 0x%02x, limit[0x%02x]\n",i , data, pt3_limits);

		for (j = 0; j < 8; j++) {

			phy_pin = (i*8 + j);

			if (CHECK_BIT(data, j) != CHECK_BIT(pt3_limits, j)) {
				LOGE("pin-%2d : fail\n", phy_pin);
				testing_hcd->result = false;
			}
			else
				LOGD("pin-%2d : pass\n", phy_pin);
		}
	}

	testing_copy_resp_to_output();

	UNLOCK_BUFFER(testing_hcd->resp);

exit:
	LOGI("Result = %s\n", (testing_hcd->result)?"pass":"fail");
	UNLOCK_BUFFER(testing_hcd->pt_hi_limits);

	return retval;
}


static ssize_t syna_testing_pt03_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt03(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$03: %s\n",
			testing_hcd->result? "Passed" : "Failed");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt03 =
	__ATTR(pt03, 0444, syna_testing_pt03_show, NULL);


static void testing_copy_resp_to_output_PT05(struct tcm_buffer *resp_pt05,struct tcm_buffer *report_rt163)
{
	int retval;
	unsigned int output_size;

	output_size = resp_pt05->data_length + report_rt163->data_length;

	LOGI("output_size:%d,resp_pt05.data_length:%d,report_rt163.data_length:%d\n",output_size, resp_pt05->data_length, report_rt163->data_length);

	LOCK_BUFFER(testing_hcd->output);

	retval = syna_tcm_buf_alloc(&testing_hcd->output,output_size);
	if (retval < 0) {
		LOGE("Failed to allocate memory for testing_hcd->output.buf\n");
		UNLOCK_BUFFER(testing_hcd->output);
		return;
	}

	retval = syna_pal_mem_cpy(testing_hcd->output.buf,
			testing_hcd->output.buf_size,
			resp_pt05->buf,
			resp_pt05->buf_size,
			resp_pt05->data_length);
	if (retval < 0) {
		LOGE("Fail to copy PT05 data to output, size: %d\n",
			resp_pt05->data_length);
		UNLOCK_BUFFER(testing_hcd->output);
		return;
	}

	retval = syna_pal_mem_cpy(testing_hcd->output.buf + resp_pt05->data_length,
			testing_hcd->output.buf_size - resp_pt05->data_length,
			report_rt163->buf,
			report_rt163->buf_size,
			report_rt163->data_length);
	if (retval < 0) {
		LOGE("Fail to copy RT163 data to output, size: %d\n",
			resp_pt05->data_length);
		UNLOCK_BUFFER(testing_hcd->output);
		return;
	}

	testing_hcd->output.data_length = output_size;

	UNLOCK_BUFFER(testing_hcd->output);

	return;
}

/**
 * syna_testing_pt05()
 *
 * Sample code to perform PT05 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt05(struct syna_tcm *tcm)
{
	int retval;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_size;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size;
	unsigned short data;
	unsigned short pt5_hi, pt5_lo;
	char *pt5_hi_limits;
	char *pt5_lo_limits;
	struct tcm_application_info *app_info;
	struct tcm_buffer report_rt163;

	testing_hcd->result = false;

	LOGI("Start testing pt05\n");

	LOCK_BUFFER(testing_hcd->pt_hi_limits);
	LOCK_BUFFER(testing_hcd->pt_lo_limits);
	limits_size = testing_get_limits_bytes(&limits_rows, &limits_cols, TEST_PID05_FULL_RAW_CAP);
	retval = testing_load_testlimits(TEST_PID05_FULL_RAW_CAP, 0);
	if (retval < 0) {
		LOGE("Failed to load PT%d limits from csv file\n", TEST_PID05_FULL_RAW_CAP);
		retval = -EINVAL;
		goto exit;
	}

	if ((limits_size != testing_hcd->pt_hi_limits.data_length) ||
		(limits_size != testing_hcd->pt_lo_limits.data_length)){
		LOGE("Mismatching limits size\n");
	}


	app_info = &g_tcm_ptr->tcm_dev->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);


	frame_size = rows * cols * 2;

	LOCK_BUFFER(testing_hcd->resp);

	testing_hcd->test_item = TEST_PID05_FULL_RAW_CAP;
	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID05_FULL_RAW_CAP,
			&testing_hcd->resp);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID05_FULL_RAW_CAP);
		UNLOCK_BUFFER(testing_hcd->resp);
		goto exit;
	}

	if ((limits_size != frame_size) &&
		(frame_size != testing_hcd->resp.data_length)) {
		LOGE("Frame size mismatch\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	if (rows > limits_rows || cols > limits_cols) {
		LOGE("Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	pt5_hi_limits = testing_hcd->pt_hi_limits.buf;
	pt5_lo_limits = testing_hcd->pt_lo_limits.buf;
	testing_hcd->result = true;

	idx = 0;
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {

			data = (unsigned short)(buf[idx] & 0xff) |
					(unsigned short)(buf[idx+1] << 8);
			pt5_hi = (unsigned short)(pt5_hi_limits[idx] & 0xff) |
					(unsigned short)(pt5_hi_limits[idx+1] << 8);
			pt5_lo = (unsigned short)(pt5_lo_limits[idx] & 0xff) |
					(unsigned short)(pt5_lo_limits[idx+1] << 8);

			if (data > pt5_hi || data < pt5_lo) {
				LOGE("fail at (%2d, %2d) data = %5d, limit = [%4d, %4d]\n",
					row, col, data, pt5_lo, pt5_hi);

				testing_hcd->result = false;
			}

			idx += 2;
		}
	}

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID84_FREQRAWTEST,
			&report_rt163);
	if (retval < 0) {
		LOGE("Fail to run test %d %d\n",TEST_PID84_FREQRAWTEST,report_rt163.buf_size);
		testing_copy_resp_to_output();
	} else {
		testing_copy_resp_to_output_PT05(&testing_hcd->resp,&report_rt163);
	}

	UNLOCK_BUFFER(testing_hcd->resp);

	retval = 0;
exit:
	LOGI("Result = %s\n", (testing_hcd->result)?"pass":"fail");

	UNLOCK_BUFFER(testing_hcd->pt_lo_limits);
	UNLOCK_BUFFER(testing_hcd->pt_hi_limits);

	return retval;
}

/**
 * syna_testing_pt05_show()
 *
 * Attribute to show the result of PT05 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt05_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt05(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$05: %s\n", testing_hcd->result? "Passed" : "Failed");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt05 =
	__ATTR(pt05, 0444, syna_testing_pt05_show, NULL);

/**
 * syna_testing_pt44()
 *
 * Sample code to perform PT44 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *    [ casenum] casenum: test case
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt44(struct syna_tcm *tcm, unsigned int casenum)
{
	int retval;
	short data;
	short pt44_limits;
	unsigned char *pt44_buf;
	unsigned char *buf;
	unsigned int idx;
	unsigned int limits_size;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int frame_size_words;
	struct tcm_application_info *app_info;

	testing_hcd->result = false;

	retval = syna_tcm_set_dynamic_config(g_tcm_ptr->tcm_dev,
				DC_REQUEST_FREQUENCY_INDEX,
				casenum,
				RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Failed to set dynamic_config with gear 0, retval=%d\n", retval);
		return retval;
	}
	
	LOGI("Start testing pt44\n");

	app_info = &g_tcm_ptr->tcm_dev->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	testing_get_frame_size_words(&frame_size_words, true);

	LOCK_BUFFER(testing_hcd->pt_hi_limits);

	limits_size = testing_get_limits_bytes(&limits_rows, &limits_cols, TEST_PID44_MULTI_Gear_NOISE_TEST);
	retval = testing_load_testlimits(TEST_PID44_MULTI_Gear_NOISE_TEST, 0);
	if (retval < 0) {
		LOGE("Failed to load PT%d limits from csv file\n", TEST_PID44_MULTI_Gear_NOISE_TEST);
		retval = -EINVAL;
		goto exit;
	}

	if (limits_size != testing_hcd->pt_hi_limits.data_length) {
		LOGE("Mismatching limits size\n");
	}

	LOCK_BUFFER(testing_hcd->resp);
	
	testing_hcd->test_item = TEST_PID44_MULTI_Gear_NOISE_TEST;
	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID44_MULTI_Gear_NOISE_TEST,
			&testing_hcd->resp);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID44_MULTI_Gear_NOISE_TEST);
		UNLOCK_BUFFER(testing_hcd->resp);
		goto exit;
	}

	if (frame_size_words != testing_hcd->resp.data_length / 2  && frame_size_words != (testing_hcd->resp.data_length / 2 - 1)) {
			LOGE("Frame size mismatch resp:%d frame_size_words:%d\n",testing_hcd->resp.data_length / 2,frame_size_words);
			UNLOCK_BUFFER(testing_hcd->resp);
			retval = -EINVAL;
		goto exit;
	}


	idx = 0;
	buf = testing_hcd->resp.buf;
	pt44_buf = testing_hcd->pt_hi_limits.buf;
	pt44_limits = (unsigned short)(pt44_buf[0] & 0xff) |
					(unsigned short)(pt44_buf[1] << 8);
	testing_hcd->result = true;

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (short)le2_to_uint(&buf[idx * 2]);
			if (data > pt44_limits) {

				LOGE("fail at (%2d, %2d) data = %5d, limit = %4d\n",
					row, col, data, pt44_limits);

				testing_hcd->result = false;
			}
			idx++;
		}
	}

	if(frame_size_words == (testing_hcd->resp.data_length / 2 - 1)) {
		LOGE("The last word need to be removed\n");
		testing_hcd->resp.data_length = testing_hcd->resp.data_length - 2;
	}
	
	testing_copy_resp_to_output();

	UNLOCK_BUFFER(testing_hcd->resp);

		
	retval = syna_tcm_set_dynamic_config(g_tcm_ptr->tcm_dev,
				DC_REQUEST_FREQUENCY_INDEX,
				0,
				RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Failed to set dynamic_config with gear 0, retval=%d\n", retval);
		goto exit;
	}
	
	retval = 0;
exit:
	LOGI("Result = %s\n", (testing_hcd->result)?"pass":"fail");

	UNLOCK_BUFFER(testing_hcd->pt_hi_limits);
	
	return retval;
}

/**
 * syna_testing_pt44_store()
 *
 * Attribute to show the result of PT44 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt44_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int casenum = 0;
	struct syna_tcm *tcm = g_tcm_ptr;
	char buffer[50] = {0};

	if (kstrtouint(buf, 10, &casenum))
               return -EINVAL;

	if (!tcm->is_connected) {
		count = snprintf(buffer, 50,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt44(tcm, casenum);

	if (casenum == 3) {
		retval = snprintf(buffer, 50,
				"TEST PT$44: %s\n", testing_hcd->result? "Passed" : "Failed");
	}

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt44 =
	__ATTR(pt44, 0220, NULL, syna_testing_pt44_store);

static int syna_testing_pt18(struct syna_tcm *tcm)
{
	int retval;
	unsigned char *buf;
	unsigned int idx;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_size;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size;
	struct tcm_application_info *app_info;
	int data;
	int pt18_hi, pt18_lo;
	char *pt18_hi_limits;
	char *pt18_lo_limits;

	testing_hcd->result = false;

	LOGI("Start testing pt18\n");

	app_info = &g_tcm_ptr->tcm_dev->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);


	frame_size = (rows + cols) * 4;

	LOCK_BUFFER(testing_hcd->pt_hi_limits);
	LOCK_BUFFER(testing_hcd->pt_lo_limits);


	limits_size = testing_get_limits_bytes(&limits_rows, &limits_cols, TEST_PID18_HYBRID_ABS_RAW);
	retval = testing_load_testlimits(TEST_PID18_HYBRID_ABS_RAW, 0);
	if (retval < 0) {
		LOGE("Failed to load PT%d limits from csv file\n", TEST_PID18_HYBRID_ABS_RAW);
		retval = -EINVAL;
		goto exit;
	}
	if ((limits_size != frame_size) ||
		(limits_size != testing_hcd->pt_hi_limits.data_length) ||
		(limits_size != testing_hcd->pt_lo_limits.data_length)){
		LOGE("Mismatching limits size\n");
	}

	LOCK_BUFFER(testing_hcd->resp);

	testing_hcd->test_item = TEST_PID18_HYBRID_ABS_RAW;
	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID18_HYBRID_ABS_RAW,
			&testing_hcd->resp);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID18_HYBRID_ABS_RAW);
		UNLOCK_BUFFER(testing_hcd->resp);
		goto exit;
	}

	if (frame_size != testing_hcd->resp.data_length) {
		LOGE("Frame size is mismatching\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	if ((rows +cols) != limits_cols || limits_rows != 1) {
		LOGE("Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	idx = 0;
	buf = testing_hcd->resp.buf;
	pt18_hi_limits = testing_hcd->pt_hi_limits.buf;
	pt18_lo_limits = testing_hcd->pt_lo_limits.buf;
	testing_hcd->result = true;


	idx = 0;
	for (col = 0; col < limits_cols; col++) {
		data = (int)(buf[idx] & 0xff) |
				(int)(buf[idx+1] << 8) |
				(int)(buf[idx+2] << 16) |
				(int)(buf[idx+3] << 24);

		pt18_hi = (int)(pt18_hi_limits[idx] & 0xff) |
				(int)(pt18_hi_limits[idx+1] << 8) |
				(int)(pt18_hi_limits[idx+2] << 16) |
				(int)(pt18_hi_limits[idx+3] << 24);

		pt18_lo = (int)(pt18_lo_limits[idx] & 0xff) |
				(int)(pt18_lo_limits[idx+1] << 8) |
				(int)(pt18_lo_limits[idx+2] << 16) |
				(int)(pt18_lo_limits[idx+3] << 24);

		if ((data > pt18_hi) || (data < pt18_lo)) {
			testing_hcd->result = false;
			LOGE("fail at index = %-2d. data = %d, limit = [%d, %d]\n",
					col, data, pt18_lo, pt18_hi);
		}
		idx+=4;
	}

	testing_copy_resp_to_output();

	UNLOCK_BUFFER(testing_hcd->resp);

	retval = 0;

exit:
	UNLOCK_BUFFER(testing_hcd->pt_lo_limits);
	UNLOCK_BUFFER(testing_hcd->pt_hi_limits);
	LOGN("Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}

static ssize_t syna_testing_pt18_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt18(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$18: %s\n",testing_hcd->result? "Passed" : "Failed");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt18 =
	__ATTR(pt18, 0444, syna_testing_pt18_show, NULL);

static int syna_testing_pt22(struct syna_tcm *tcm)
{
	int retval;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row, col;
	unsigned int rows, cols;
	unsigned int limits_size;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size;
	unsigned short data, pt22_hi, pt22_lo;
	char *pt22_hi_limits;
	char *pt22_lo_limits;
	struct tcm_application_info *app_info;

	testing_hcd->result = false;

	LOGI("Start testing pt22\n");

	LOCK_BUFFER(testing_hcd->pt_hi_limits);
	LOCK_BUFFER(testing_hcd->pt_lo_limits);

	retval = testing_load_testlimits(TEST_PID22_TRANS_CAP_RAW, 0);
	if (retval < 0) {
		LOGE("Failed to load PT%d limits from csv file\n", TEST_PID22_TRANS_CAP_RAW);
		retval = -EINVAL;
		goto exit;
	}

	app_info = &g_tcm_ptr->tcm_dev->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	frame_size = rows * cols * 2;

	limits_size = testing_get_limits_bytes(&limits_rows, &limits_cols, TEST_PID22_TRANS_CAP_RAW);

	LOCK_BUFFER(testing_hcd->resp);

	testing_hcd->test_item = TEST_PID22_TRANS_CAP_RAW;
	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID22_TRANS_CAP_RAW,
			&testing_hcd->resp);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID22_TRANS_CAP_RAW);
		UNLOCK_BUFFER(testing_hcd->resp);
		goto exit;
	}


	if ((frame_size != limits_size) ||
		(limits_size != testing_hcd->pt_hi_limits.data_length) ||
		(limits_size != testing_hcd->pt_lo_limits.data_length)){
		LOGE( "Mismatching limits size\n");
	}


	if (frame_size != testing_hcd->resp.data_length) {
		LOGE("Frame size mismatch\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}
	if (rows > limits_rows || cols > limits_cols) {
		LOGE("Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	pt22_hi_limits = testing_hcd->pt_hi_limits.buf;
	pt22_lo_limits = testing_hcd->pt_lo_limits.buf;
	testing_hcd->result = true;


	/* check PT22 min/max limits */
	idx = 0;
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {

			data = (unsigned short)(buf[idx] & 0xff) |
					(unsigned short)(buf[idx+1] << 8);
			pt22_hi = (unsigned short)(pt22_hi_limits[idx] & 0xff) |
					(unsigned short)(pt22_hi_limits[idx+1] << 8);
			pt22_lo = (unsigned short)(pt22_lo_limits[idx] & 0xff) |
					(unsigned short)(pt22_lo_limits[idx+1] << 8);

			if (data > pt22_hi || data < pt22_lo) {
				LOGE("fail at (%2d, %2d) data = %5d, limit = [%4d, %4d]\n",
					row, col, data, pt22_lo, pt22_hi);

				testing_hcd->result = false;
			}

			idx += 2;
		}
	}

	testing_copy_resp_to_output();

	UNLOCK_BUFFER(testing_hcd->resp);

	retval = 0;
exit:
	LOGI("Result = %s\n", (testing_hcd->result)?"pass":"fail");

	UNLOCK_BUFFER(testing_hcd->pt_lo_limits);
	UNLOCK_BUFFER(testing_hcd->pt_hi_limits);

	return retval;
}

static ssize_t syna_testing_pt22_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt22(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$22: %s\n", testing_hcd->result? "Passed" : "Failed");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt22 =
	__ATTR(pt22, 0444, syna_testing_pt22_show, NULL);

static int syna_testing_pt81(struct syna_tcm *tcm)
{
	int retval;
	unsigned char *buf;
	unsigned int idx;
	unsigned int col;
	unsigned int rows, cols;
	unsigned int limits_size;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size;
	unsigned short data, pt81_sum, pt81_rx_stdev;
	char *pt81_sum_limits;
	char *pt81_rx_stdev_limits;
	unsigned short rx_stdev[40] = {0};
	struct tcm_application_info *app_info;

	testing_hcd->result = false;

	LOGI("Start testing pt81\n");

	LOCK_BUFFER(testing_hcd->pt_hi_limits);
	LOCK_BUFFER(testing_hcd->pt_lo_limits);

	retval = testing_load_testlimits(TEST_PID81_PIXEL_UNIFORMITY, 0);
	if (retval < 0) {
		LOGE("Failed to load PT%d limits from csv file\n", TEST_PID81_PIXEL_UNIFORMITY);
		retval = -EINVAL;
		goto exit;
	}

	app_info = &g_tcm_ptr->tcm_dev->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	frame_size =  cols * 2;

	limits_size = testing_get_limits_bytes(&limits_rows, &limits_cols, TEST_PID81_PIXEL_UNIFORMITY);

	LOCK_BUFFER(testing_hcd->resp);

	testing_hcd->test_item = TEST_PID81_PIXEL_UNIFORMITY;
	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID81_PIXEL_UNIFORMITY,
			&testing_hcd->resp);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID81_PIXEL_UNIFORMITY);
		UNLOCK_BUFFER(testing_hcd->resp);
		goto exit;
	}


	if ((frame_size != limits_size) ||
		(limits_size != testing_hcd->pt_hi_limits.data_length) ||
		(limits_size != testing_hcd->pt_lo_limits.data_length)){
		LOGE( "Mismatching limits size frame_size:%d limits_size:%d Rx_stdev limits size:%d Sum limits size:%d \n",frame_size
			,limits_size,testing_hcd->pt_lo_limits.data_length,testing_hcd->pt_hi_limits.data_length);
	}


	if (frame_size != testing_hcd->resp.data_length) {
		LOGE("Frame size mismatch frame_size %d resp.data_length: %d\n",frame_size,testing_hcd->resp.data_length);
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}
	if (cols > limits_cols) {
		LOGE("Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	/* CSV_PT81_TESTING_SUM_STDEV_LIMITS */
	pt81_sum_limits = testing_hcd->pt_hi_limits.buf;
	/* CSV_PT81_TESTING_RX_STDEV_LIMITS */
	pt81_rx_stdev_limits = testing_hcd->pt_lo_limits.buf;
	testing_hcd->result = true;

	/* check PT81 limits */
	idx = 0;
	for (col = 0; col < cols; col++) {

		data = (unsigned short)(buf[idx] & 0xff) |
				(unsigned short)(buf[idx+1] << 8);
		pt81_rx_stdev = (unsigned short)(pt81_rx_stdev_limits[idx] & 0xff) |
				(unsigned short)(pt81_rx_stdev_limits[idx+1] << 8);

		LOGD("col: (%2d) data = %5d, rx stdev limit = [%4d]\n",
			col, data, pt81_rx_stdev);

		if(data > pt81_rx_stdev)
		{
			LOGE("fail at (%2d) data = %5d, rx stdev limit = [%4d]\n",
			col, data, pt81_rx_stdev);
			testing_hcd->result = false;
		}
		rx_stdev[col] = data;
		idx += 2;
	}

	idx = 0;
	for (col = 0; col < cols-2; col++)
	{
		data = rx_stdev[col] + rx_stdev[col + 1] + rx_stdev[col + 2];
		pt81_sum = (unsigned short)(pt81_sum_limits[idx] & 0xff) |
				(unsigned short)(pt81_sum_limits[idx+1] << 8);

		LOGD("col: (%2d) data = %5d, sum limit = [%4d]\n",
			col, data, pt81_sum);

		if(data > pt81_sum)
		{
			LOGE("fail at (%2d) data = %5d, sum limit = [%4d]\n",
			col, data, pt81_sum);
			testing_hcd->result = false;
		}
		idx += 2;
	}

	testing_copy_resp_to_output();

	UNLOCK_BUFFER(testing_hcd->resp);

	retval = 0;
exit:
	LOGI("Result = %s\n", (testing_hcd->result)?"pass":"fail");

	UNLOCK_BUFFER(testing_hcd->pt_lo_limits);
	UNLOCK_BUFFER(testing_hcd->pt_hi_limits);

	return retval;
}

static ssize_t syna_testing_pt81_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt81(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$81: %s\n", testing_hcd->result? "Passed" : "Failed");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt81 =
	__ATTR(pt81, 0444, syna_testing_pt81_show, NULL);

static int testing_copy_rawshift_diff_to_output(enum tcm_test_code testcode)
{
	int retval;
	unsigned int output_size;
	unsigned int rows;
	unsigned int cols;
	short *rawdiff;
	struct tcm_application_info *app_info;
 
	app_info = &g_tcm_ptr->tcm_dev->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);
 
	output_size = rows*cols*2;

	LOCK_BUFFER(testing_hcd->output);
	testing_hcd->test_item = testcode;
	retval = syna_tcm_buf_alloc(&testing_hcd->output,output_size);
	if (retval < 0) {
		LOGE("Failed to allocate memory for testing_hcd->output.buf\n");
		UNLOCK_BUFFER(testing_hcd->output);
		return retval;
	}

	if (testing_hcd->test_item == TEST_PID85_RAWSHIFTTEST) {
		rawdiff = rawDiffPT85;
		testing_hcd->result = result_pt85;
	} else if(testing_hcd->test_item == TEST_PID86_RAWSHIFTTEST){
		rawdiff = rawDiffPT86;
		testing_hcd->result = result_pt86;
	}

	/* copy data content to the destination */
	retval = syna_pal_mem_cpy(testing_hcd->output.buf,
			testing_hcd->output.buf_size,
			rawdiff,
			output_size,
			output_size);
	if (retval < 0) {
		LOGE("Fail to copy data to output, size: %d\n",
			output_size);
		UNLOCK_BUFFER(testing_hcd->output);
		return retval;
	}

	testing_hcd->output.data_length = output_size;
	UNLOCK_BUFFER(testing_hcd->output);

	return retval;
}

//switch gear
static int syna_testing_pt86(struct syna_tcm *tcm, unsigned int casenum)
{
	int retval;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row, col;
	unsigned int rows, cols;
	unsigned int limits_size;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size;
	unsigned short data,pt86_hi;
	char *pt86_hi_limits;
	struct tcm_application_info *app_info;
	short rawTest[720] = {0};

	testing_hcd->result = false;

	result_pt86= false;

	retval = syna_tcm_set_dynamic_config(g_tcm_ptr->tcm_dev,
				DC_REQUEST_FREQUENCY_INDEX,
				casenum,
				RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Failed to set dynamic_config with gear 0, retval=%d\n", retval);
		return retval;
	}

	LOGI("Start testing pt86, casenum = %d\n", casenum);

	app_info = &g_tcm_ptr->tcm_dev->app_info;
	
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	LOCK_BUFFER(testing_hcd->resp);

	testing_hcd->test_item = TEST_PID86_RAWSHIFTTEST;
	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID85_RAWSHIFTTEST,  //same as PT85，get the avg of 100 frames from ic
			&testing_hcd->resp);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID85_RAWSHIFTTEST);
		UNLOCK_BUFFER(testing_hcd->resp);
		goto exit;
	}

	buf = testing_hcd->resp.buf;

	/* get PT86 max/min */
	idx = 0;
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {

			data = (unsigned short)(buf[idx] & 0xff) |
					(unsigned short)(buf[idx+1] << 8);
			rawTest[row*cols+col] = data;
			if (casenum == 0) {
				rawMin[row*cols+col] = data;
				rawMax[row*cols+col] = data;
			} else {
				if(data > rawMax[row*cols+col])
					rawMax[row*cols+col] = data;
				if(data < rawMin[row*cols+col])
					rawMin[row*cols+col] = data;
			}
			LOGD("row:%d col:%d id:%d data:%d min:%d max:%d\n",row,col,row*cols+col,data,rawMin[row*cols+col],rawMax[row*cols+col]);
			idx += 2;
		}
	}
	LOGD("casenum:%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",casenum,
	rawTest[0],rawTest[1],rawTest[2],rawTest[3],rawTest[4],rawTest[5],rawTest[6],rawTest[7],rawTest[8],rawTest[9],
	rawTest[10],rawTest[11],rawTest[12],rawTest[13],rawTest[14],rawTest[15],rawTest[16],rawTest[17],rawTest[18],rawTest[19],
	rawTest[20],rawTest[21],rawTest[22],rawTest[23],rawTest[24],rawTest[25],rawTest[26],rawTest[27],rawTest[28],rawTest[29],
	rawTest[30],rawTest[31],rawTest[32],rawTest[33],rawTest[34],rawTest[35],rawTest[36],rawTest[37],rawTest[38],rawTest[39]);
	
	LOGD("rawMin casenum:%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",casenum,
	rawMin[0],rawMin[1],rawMin[2],rawMin[3],rawMin[4],rawMin[5],rawMin[6],rawMin[7],rawMin[8],rawMin[9],
	rawMin[10],rawMin[11],rawMin[12],rawMin[13],rawMin[14],rawMin[15],rawMin[16],rawMin[17],rawMin[18],rawMin[19],
	rawMin[20],rawMin[21],rawMin[22],rawMin[23],rawMin[24],rawMin[25],rawMin[26],rawMin[27],rawMin[28],rawMin[29],
	rawMin[30],rawMin[31],rawMin[32],rawMin[33],rawMin[34],rawMin[35],rawMin[36],rawMin[37],rawMin[38],rawMin[39]);

	LOGD("rawMax casenum:%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",casenum,
	rawMax[0],rawMax[1],rawMax[2],rawMax[3],rawMax[4],rawMax[5],rawMax[6],rawMax[7],rawMax[8],rawMax[9],
	rawMax[10],rawMax[11],rawMax[12],rawMax[13],rawMax[14],rawMax[15],rawMax[16],rawMax[17],rawMax[18],rawMax[19],
	rawMax[20],rawMax[21],rawMax[22],rawMax[23],rawMax[24],rawMax[25],rawMax[26],rawMax[27],rawMax[28],rawMax[29],
	rawMax[30],rawMax[31],rawMax[32],rawMax[33],rawMax[34],rawMax[35],rawMax[36],rawMax[37],rawMax[38],rawMax[39]);

	if (casenum == 2) {
		LOCK_BUFFER(testing_hcd->pt_hi_limits);

		retval = testing_load_testlimits(TEST_PID86_RAWSHIFTTEST, 0);
		if (retval < 0) {
			LOGE("Failed to load PT%d limits from csv file\n", TEST_PID86_RAWSHIFTTEST);
			UNLOCK_BUFFER(testing_hcd->resp);
			UNLOCK_BUFFER(testing_hcd->pt_hi_limits);
			retval = -EINVAL;
			goto exit;
		}

		frame_size = rows * cols * 2;

		limits_size = testing_get_limits_bytes(&limits_rows, &limits_cols, TEST_PID86_RAWSHIFTTEST);
		pt86_hi_limits = testing_hcd->pt_hi_limits.buf;

		if ((frame_size != limits_size) ||
				(limits_size != testing_hcd->pt_hi_limits.data_length)){
			LOGE( "Mismatching limits size\n");
		}

		if (frame_size != testing_hcd->resp.data_length) {
			LOGE("Frame size mismatch\n");
			UNLOCK_BUFFER(testing_hcd->resp);
			UNLOCK_BUFFER(testing_hcd->pt_hi_limits);
			retval = -EINVAL;
			goto exit;
		}
		if (rows > limits_rows || cols > limits_cols) {
			LOGE("Mismatching limits data\n");
			UNLOCK_BUFFER(testing_hcd->resp);
			UNLOCK_BUFFER(testing_hcd->pt_hi_limits);
			retval = -EINVAL;
			goto exit;
		}

		result_pt86 = true;


		/* check PT86 max limits */
		idx = 0;
		for (row = 0; row < rows; row++) {
			for (col = 0; col < cols; col++) {

				rawDiffPT86[row*cols+col] = rawMax[row*cols+col] - rawMin[row*cols+col];
				pt86_hi = (unsigned short)(pt86_hi_limits[idx] & 0xff) |
						(unsigned short)(pt86_hi_limits[idx+1] << 8);
				LOGD("row:%d col:%d PT86 limit:%d rawMax %d rawMin:%d diff:%d\n",row,col,pt86_hi,rawMax[row*cols+col],rawMin[row*cols+col],rawDiffPT86[row*cols+col]);
				if (rawDiffPT86[row*cols+col] > pt86_hi) {
					LOGE("fail at (%2d, %2d) diff data = %5d, limit = [%4d]\n",
						row, col, rawDiffPT86[row*cols+col], pt86_hi);

					result_pt86 = false;
				}

				idx += 2;
			}
		}
		UNLOCK_BUFFER(testing_hcd->pt_hi_limits);
	}
	
	UNLOCK_BUFFER(testing_hcd->resp);
	
	retval = syna_tcm_set_dynamic_config(g_tcm_ptr->tcm_dev,
				DC_REQUEST_FREQUENCY_INDEX,
				0,
				RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Failed to set dynamic_config with gear 0, retval=%d\n", retval);
		goto exit;
	}

	retval = 0;
exit:
	if (retval == 0) {
		if(casenum == 2)
			LOGI("Result = %s\n", (result_pt86)?"pass":"fail");
		else
			LOGI("Already read %d frames\n", casenum + 1);
	} else {
		LOGE("PT86 testing : read %d frame failed!", casenum + 1);
	}
	return retval;

}

static ssize_t syna_testing_pt86_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	int i;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}
	/* do case 0, 1, 2*/
	for (i = 0; i < 3; i++) {
		retval = syna_testing_pt86(tcm, i);
		if (retval < 0)
			LOGE("test pt86 failed, casenum = %d\n", i);
	}

	count = snprintf(buf, PAGE_SIZE,
				"TEST PT$86: %s\n", result_pt86? "Passed" : "Failed");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt86 =
	__ATTR(pt86, 0444, syna_testing_pt86_show, NULL);

static int syna_testing_pt85(struct syna_tcm *tcm, unsigned int casenum)
{
	int retval;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row, col;
	unsigned int rows, cols;
	unsigned int limits_size;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size;
	unsigned short data,pt85_hi;
	char *pt85_hi_limits;
	struct tcm_application_info *app_info;
	short rawTest[720] = {0};

	testing_hcd->result = false;

	result_pt85 = false;

	LOGI("Start testing pt85, casenum = %d\n", casenum);

	app_info = &g_tcm_ptr->tcm_dev->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	LOCK_BUFFER(testing_hcd->resp);

	testing_hcd->test_item = TEST_PID85_RAWSHIFTTEST;
	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID85_RAWSHIFTTEST,
			&testing_hcd->resp);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID85_RAWSHIFTTEST);
		UNLOCK_BUFFER(testing_hcd->resp);
		goto exit;
	}

	buf = testing_hcd->resp.buf;

	/* get PT85 max/min */
	idx = 0;
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {

			data = (unsigned short)(buf[idx] & 0xff) |
					(unsigned short)(buf[idx+1] << 8);
			rawTest[row*cols+col] = data;
			if(casenum == 0) {
				rawMin[row*cols+col] = data;
				rawMax[row*cols+col] = data;
			} else {
				if(data > rawMax[row*cols+col])
					rawMax[row*cols+col] = data;
				if(data < rawMin[row*cols+col])
					rawMin[row*cols+col] = data;
			}
			LOGD("row:%d col:%d id:%d data:%d min:%d max:%d\n",row,col,row*cols+col,data,rawMin[row*cols+col],rawMax[row*cols+col]);
			idx += 2;
		}
	}
	LOGD("rawTest casenum:%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",casenum,
	rawTest[0],rawTest[1],rawTest[2],rawTest[3],rawTest[4],rawTest[5],rawTest[6],rawTest[7],rawTest[8],rawTest[9],
	rawTest[10],rawTest[11],rawTest[12],rawTest[13],rawTest[14],rawTest[15],rawTest[16],rawTest[17],rawTest[18],rawTest[19],
	rawTest[20],rawTest[21],rawTest[22],rawTest[23],rawTest[24],rawTest[25],rawTest[26],rawTest[27],rawTest[28],rawTest[29],
	rawTest[30],rawTest[31],rawTest[32],rawTest[33],rawTest[34],rawTest[35],rawTest[36],rawTest[37],rawTest[38],rawTest[39]);

	LOGD("rawMin casenum:%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",casenum,
	rawMin[0],rawMin[1],rawMin[2],rawMin[3],rawMin[4],rawMin[5],rawMin[6],rawMin[7],rawMin[8],rawMin[9],
	rawMin[10],rawMin[11],rawMin[12],rawMin[13],rawMin[14],rawMin[15],rawMin[16],rawMin[17],rawMin[18],rawMin[19],
	rawMin[20],rawMin[21],rawMin[22],rawMin[23],rawMin[24],rawMin[25],rawMin[26],rawMin[27],rawMin[28],rawMin[29],
	rawMin[30],rawMin[31],rawMin[32],rawMin[33],rawMin[34],rawMin[35],rawMin[36],rawMin[37],rawMin[38],rawMin[39]);

	LOGD("rawMax casenum:%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",casenum,
	rawMax[0],rawMax[1],rawMax[2],rawMax[3],rawMax[4],rawMax[5],rawMax[6],rawMax[7],rawMax[8],rawMax[9],
	rawMax[10],rawMax[11],rawMax[12],rawMax[13],rawMax[14],rawMax[15],rawMax[16],rawMax[17],rawMax[18],rawMax[19],
	rawMax[20],rawMax[21],rawMax[22],rawMax[23],rawMax[24],rawMax[25],rawMax[26],rawMax[27],rawMax[28],rawMax[29],
	rawMax[30],rawMax[31],rawMax[32],rawMax[33],rawMax[34],rawMax[35],rawMax[36],rawMax[37],rawMax[38],rawMax[39]);

	if(casenum == 1) {
		LOCK_BUFFER(testing_hcd->pt_hi_limits);
	
		retval = testing_load_testlimits(TEST_PID85_RAWSHIFTTEST, 0);
		if (retval < 0) {
			LOGE("Failed to load PT%d limits from csv file\n", TEST_PID85_RAWSHIFTTEST);
			UNLOCK_BUFFER(testing_hcd->resp);
			UNLOCK_BUFFER(testing_hcd->pt_hi_limits);
			retval = -EINVAL;
			goto exit;
		}

		frame_size = rows * cols * 2;

		limits_size = testing_get_limits_bytes(&limits_rows, &limits_cols, TEST_PID85_RAWSHIFTTEST);
		pt85_hi_limits = testing_hcd->pt_hi_limits.buf;
		if ((frame_size != limits_size) ||
			(limits_size != testing_hcd->pt_hi_limits.data_length)) {
			LOGE( "Mismatching limits size\n");
		}

		if (frame_size != testing_hcd->resp.data_length) {
			LOGE("Frame size mismatch\n");
			UNLOCK_BUFFER(testing_hcd->resp);
			UNLOCK_BUFFER(testing_hcd->pt_hi_limits);
			retval = -EINVAL;
			goto exit;
		}
		if (rows > limits_rows || cols > limits_cols) {
			LOGE("Mismatching limits data\n");
			UNLOCK_BUFFER(testing_hcd->resp);
			UNLOCK_BUFFER(testing_hcd->pt_hi_limits);
			retval = -EINVAL;
			goto exit;
		}
		result_pt85 = true;

		/* check PT85 max limits */
		idx = 0;
		for (row = 0; row < rows; row++) {
			for (col = 0; col < cols; col++) {

				rawDiffPT85[row*cols+col] = rawMax[row*cols+col] - rawMin[row*cols+col];
				pt85_hi = (unsigned short)(pt85_hi_limits[idx] & 0xff) |
						(unsigned short)(pt85_hi_limits[idx+1] << 8);
				LOGD("row:%d col:%d PT85 limit:%d rawMax:%d rawMin:%d diff:%d\n",
						row,col,pt85_hi,rawMax[row*cols+col],rawMin[row*cols+col],rawDiffPT85[row*cols+col]);
				if (rawDiffPT85[row*cols+col] > pt85_hi) {
					LOGE("row:%d col:%d PT85 limit:%d rawMax:%d rawMin:%d diff:%d\n",
							row,col,pt85_hi,rawMax[row*cols+col],rawMin[row*cols+col],rawDiffPT85[row*cols+col]);

					result_pt85 = false;
				}

				idx += 2;
			}
		}
		UNLOCK_BUFFER(testing_hcd->pt_hi_limits);
	}
	
	UNLOCK_BUFFER(testing_hcd->resp);

	retval = 0;
exit:
	if (retval == 0) {
		if(casenum == 1)
			LOGI("Result = %s\n", (result_pt85)?"pass":"fail");
		else
			LOGI("Already read %d frames\n",casenum + 1);
	} else {
		LOGI("PT85 testing : read %d frame failed!",casenum + 1);
	}
	return retval;

}

static ssize_t syna_testing_pt85_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int casenum;
	struct syna_tcm *tcm = g_tcm_ptr;
	char buffer[50] = {0};

	if (!tcm->is_connected) {
		count = snprintf(buffer, 50,
				"Device is NOT connected\n");
		goto exit;
	}

	if (kstrtouint(buf, 10, &casenum))
		return -EINVAL;

	retval = syna_testing_pt85(tcm, casenum);
	
	if (casenum == 1) {
		retval = snprintf(buffer, 50,
				"TEST PT$85: %s\n", result_pt85? "Passed" : "Failed");
	}
exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt85 =
	__ATTR(pt85, 0220, NULL, syna_testing_pt85_store);

static int syna_testing_gap_diff(struct syna_tcm *tcm)
{
	int retval;
	unsigned char *buf;
	unsigned int row, col;
	unsigned int rows, cols;
	unsigned int limits_size;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size;
	short data, d0, d1, d2, d3, dmax, diff;
	short gap_diff_limits;
	char *gap_diff_limits_buf;
	char *gap_diff_data_buf = NULL;

	struct tcm_application_info *app_info;

	testing_hcd->result = false;

	LOGI("Start testing gap diff based on PT%d output data\n",testing_hcd->test_item);

	LOCK_BUFFER(testing_hcd->pt_hi_limits);

	retval = testing_load_testlimits(testing_hcd->test_item, 1);
	if (retval < 0) {
		LOGE("Failed to load PT%d limits from csv file\n", testing_hcd->test_item);
		goto exit;
	}

	app_info = &g_tcm_ptr->tcm_dev->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	frame_size = rows * cols * 2;

	limits_size = testing_get_limits_bytes(&limits_rows, &limits_cols, testing_hcd->test_item);
	/* data in output buffer, no need to read from FW */
	if ((limits_size != frame_size) ||
		(limits_size != testing_hcd->pt_hi_limits.data_length)){
		LOGE("Mismatching limits size\n");
	}


	/* calc the gap diff based on the data in output */
	LOCK_BUFFER(testing_hcd->output);

	if (frame_size != testing_hcd->output.data_length) {
		LOGE("Frame size mismatch\n");
		retval = -EINVAL;
		goto unlock_bufffer;
	}

	if (rows > limits_rows || cols > limits_cols) {
		LOGE("Mismatching limits data\n");
		retval = -EINVAL;
		goto unlock_bufffer;
	}

	/* alloc the buf to store the gap diff data */
	gap_diff_data_buf = kzalloc(frame_size, GFP_KERNEL);
	if (!gap_diff_data_buf) {
		LOGE("Failed to allocate memory for gap_diff_data_buf\n");
		retval = -ENOMEM;
		goto unlock_bufffer;
	}


	buf = testing_hcd->output.buf;
	gap_diff_limits_buf = testing_hcd->pt_hi_limits.buf;
	testing_hcd->result = true;


	/* calc the gap diff */
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			gap_diff_limits = le2_to_uint(&gap_diff_limits_buf[(row*cols + col)*2]);
			data = le2_to_uint(&buf[(row*cols + col)*2]);

			d0 = (row == 0) ? data : (le2_to_uint(&buf[((row-1)*cols + col)*2]));
			d1 = (row == (rows-1)) ? data : (le2_to_uint(&buf[((row+1)*cols + col)*2]));
			d2 = (col == 0) ? data : (le2_to_uint(&buf[(row*cols + (col-1))*2]));
			d3 = (col == (cols-1)) ? data : (le2_to_uint(&buf[(row*cols + (col+1))*2]));

			dmax = MAX(ABS(data-d0), ABS(data-d1));
			dmax = MAX(dmax, ABS(data-d2));
			dmax = MAX(dmax, ABS(data-d3));

			if (data == 0) {
				diff = 100;
			} else {
				diff = (unsigned short)((unsigned int)dmax*100/data);
			}
			if (diff > gap_diff_limits) {
				LOGE("fail at (%2d, %2d), diff_max = %4d, data = %4d, gap_diff = %4d, limit = %4d\n",
					row, col, dmax, data, diff, gap_diff_limits);

				testing_hcd->result = false;
			}

			/* backup the diff data */
			uint_to_le2(&gap_diff_data_buf[(row*cols + col)*2], diff);
		}
	}

	retval = syna_pal_mem_cpy(testing_hcd->output.buf,
			testing_hcd->output.buf_size,
			gap_diff_data_buf,
			frame_size,
			frame_size);
	if (retval < 0) {
		LOGE("Failed to copy test resp data\n");
		goto unlock_bufffer;
	}

	testing_hcd->output.data_length = frame_size;

unlock_bufffer:
	UNLOCK_BUFFER(testing_hcd->output);

exit:
	UNLOCK_BUFFER(testing_hcd->pt_hi_limits);

	if (gap_diff_data_buf)
		kfree(gap_diff_data_buf);

	LOGI("Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}

static ssize_t syna_testing_gap_diff_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_gap_diff(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST gap diff: %s\n",testing_hcd->result? "Passed" : "Failed");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_gap_diff =
	__ATTR(gap_diff, 0444, syna_testing_gap_diff_show, NULL);

static int syna_testing_HwReset(struct syna_tcm *tcm)
{
	int retval;
	bool result = false;
	unsigned char status;
	int retrytimes = 5;
	int i = 0;

	LOGI("Start testing HwReset\n");

	if (!tcm->hw_if->ops_hw_reset){
		LOGE("No hardware reset support\n");
		goto exit;
	}

	retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
	if (retval < 0) {
		LOGE("Failed to disable interrupt\n");
		goto exit;
	}

	tcm->hw_if->ops_hw_reset(tcm->hw_if, 0);

	for ( ; i < retrytimes; i++){
		retval = syna_tcm_get_event_data(tcm->tcm_dev,
				&status, NULL);
		if ((retval < 0) || (status != REPORT_IDENTIFY)) {
			LOGE("Fail to get identify report, retry :%d\n",i);
			if( i == retrytimes - 1){
				LOGE("Fail to complete hw reset\n");
				goto exit;
			}
		}else{
			LOGI("get identify report\n");
			break;
		}
	}

	retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
	if (retval < 0) {
		LOGE("Failed to enable interrupt\n");
		goto exit;
	}

	if (IS_APP_FW_MODE(tcm->tcm_dev->dev_mode)){
		LOGI("Prepare to set up application firmware\n");
		retval = tcm->dev_set_up_app_fw(tcm);
		if (retval < 0) {
		LOGE("Fail to set up app fw\n");
			goto exit;
		}
	}

	result = true;

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 1 : 0);
}

static ssize_t syna_testing_HwReset_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_HwReset(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"HwReset Test: %s\n", (retval) ? "pass":"fail");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_ptHwReset =
	__ATTR(ptHwReset, 0444, syna_testing_HwReset_show, NULL);

static int syna_testing_self_test(char *buf, char *save_buf, unsigned int *length)
{

	int retval;
	int result_hwreset = 0;
	char result_info[RESULT_INFO_LEN] = {0};
	unsigned int offset = 0;
	unsigned int cnt;
	u64 result_flag = 0;
	u64 mask_base = 1;
	u64 mask;
	struct syna_tcm *tcm = g_tcm_ptr;
#ifdef CONFIG_TOUCH_FACTORY_BUILD
	int i;
#endif

	/*
	description for mask:
		TEST_PID44_MULTI_Gear_NOISE_TEST: gear0-0x2c; gear1-0x13; gear2-0x14; gearFF-0x15
		TEST_PID81_PIXEL_UNIFORMITY: 0x18
		TEST_PID85_RAWSHIFTTEST: 0x19
		TEST_PID86_RAWSHIFTTEST: 0x1a
	*/

	mask = (mask_base<<TEST_PID01_TRX_TRX_SHORTS) |
			(mask_base<<TEST_PID03_TRX_GROUND_SHORTS) |
			(mask_base<<TEST_PID05_FULL_RAW_CAP) |
			(mask_base<<TEST_PID22_TRANS_CAP_RAW) |
			(mask_base<<TEST_GAP_DIFF) |
			(mask_base<<TEST_PID44_MULTI_Gear_NOISE_TEST) |
			(mask_base<<TEST_PID18_HYBRID_ABS_RAW) |
			(mask_base<<0x13) |
			(mask_base<<0x14) |
			(mask_base<<0x15) |
			(mask_base<<0x18) |
			(mask_base<<0x19) |
			(mask_base<<0x1a) ;

	testing_hcd->result = false;

	retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
	if (retval < 0) {
		LOGE("Failed to enable interrupt\n");
		goto exit;
	}

	LOGI("\n");
	LOGI("Start Panel testing\n");

	/* PT1 Test */
	retval = syna_testing_pt01(tcm);
	if (retval < 0) {
		goto exit;
	}

	result_flag |= (testing_hcd->result) ? (mask_base<<TEST_PID01_TRX_TRX_SHORTS) : 0;
	offset = testing_save_output(save_buf, offset, NULL);


	/* PT03 Test */
	retval = syna_testing_pt03(tcm);
	if (retval < 0) {
		goto exit;
	}
	result_flag |= (testing_hcd->result) ? (mask_base<<TEST_PID03_TRX_GROUND_SHORTS) : 0;
	offset = testing_save_output(save_buf, offset, NULL);

	/* PT5 Test (TEST_PT5_FULL_RAW_CAP) */
	retval = syna_testing_pt05(tcm);
	if (retval < 0) {
		goto exit;
	}
	result_flag |= (testing_hcd->result) ? (mask_base<<TEST_PID05_FULL_RAW_CAP) : 0;
	offset = testing_save_output(save_buf, offset, NULL);

	/* PT22 Test */
	retval = syna_testing_pt22(tcm);
	if (retval < 0) {
		goto exit;
	}
	result_flag |= (testing_hcd->result) ? (mask_base<<TEST_PID22_TRANS_CAP_RAW) : 0;
	offset = testing_save_output(save_buf, offset, NULL);

	/* gap_diff Test */
	retval = syna_testing_gap_diff(tcm);
	if (retval < 0) {
		goto exit;
	}
	result_flag |= (testing_hcd->result) ? (mask_base<<TEST_GAP_DIFF) : 0;
	offset = testing_save_output(save_buf, offset, "GAP_DIFF");

	/* PT44 Test gear0*/
	retval = syna_testing_pt44(tcm, 0);
	if (retval < 0) {
		goto exit;
	}
	result_flag |= (testing_hcd->result) ? (mask_base<<TEST_PID44_MULTI_Gear_NOISE_TEST) : 0;
	offset = testing_save_output(save_buf, offset, "PT44 gear0");

	/* PT44 Test gear1*/
	retval = syna_testing_pt44(tcm, 1);
	if (retval < 0) {
		goto exit;
	}
	/* TEST_PID44_MULTI_Gear_NOISE_TEST gear1, use 0x13 for mask */
	result_flag |= (testing_hcd->result) ? (mask_base<<0x13) : 0;
	offset = testing_save_output(save_buf, offset, "PT44 gear1");

	/* PT44 Test gear2*/
	retval = syna_testing_pt44(tcm, 2);
	if (retval < 0) {
		goto exit;
	}
	/* TEST_PID44_MULTI_Gear_NOISE_TEST gear2, use 0x14 for mask */
	result_flag |= (testing_hcd->result) ? (mask_base<<0x14) : 0;
	offset = testing_save_output(save_buf, offset, "PT44 gear2");

	/* PT44 Test all gear test*/
	retval = syna_testing_pt44(tcm, 0xFF);
	if (retval < 0) {
		goto exit;
	}
	/* TEST_PID44_MULTI_Gear_NOISE_TEST gearFF, use 0x15 for mask */
	result_flag |= (testing_hcd->result) ? (mask_base<<0x15) : 0;
	offset = testing_save_output(save_buf, offset, "PT44 gearFF");

	/* PT18 Test */
	retval = syna_testing_pt18(tcm);
	if (retval < 0) {
		goto exit;
	}
	result_flag |= (testing_hcd->result) ? (mask_base<<TEST_PID18_HYBRID_ABS_RAW) : 0;
	offset = testing_save_output(save_buf, offset, NULL);

	/* PT81 Test*/
	retval = syna_testing_pt81(tcm);
	if (retval < 0) {
		goto exit;
	}
	/* TEST_PID81_PIXEL_UNIFORMITY 0x51 out of range, use 0x18 for mask */
	result_flag |= (testing_hcd->result) ? (mask_base<<0x18 ) : 0;
	offset = testing_save_output(save_buf, offset, NULL);

	/* PT85 only save result */
	testing_copy_rawshift_diff_to_output(TEST_PID85_RAWSHIFTTEST);
	/* TEST_PID85_RAWSHIFTTEST 0x55 out of range, use 0x19 for mask */
	result_flag |= (testing_hcd->result) ? (1<< 0x19 ) : 0;
	offset = testing_save_output(save_buf, offset, NULL);

#ifdef CONFIG_TOUCH_FACTORY_BUILD
	/* PT86 Test will run only in factory build */
	for (i = 0; i < 3; ++i) {
		retval = syna_testing_pt86(tcm, i);
		if (retval < 0) {
			LOGE("test pt86 failed, casenum = %d\n", i);
			goto exit;
		}
	}
#endif
	/* PT86 save result */
	testing_copy_rawshift_diff_to_output(TEST_PID86_RAWSHIFTTEST);
	/* TEST_PID86_RAWSHIFTTEST 0x56 out of range, use 0x1a for mask */
 	result_flag |= (testing_hcd->result) ? (1<< 0x1a ) : 0;
 	offset = testing_save_output(save_buf, offset, NULL);

	/* HW reset test */
	result_hwreset = syna_testing_HwReset(tcm);
	if (result_hwreset < 1) {
		LOGE("HW reset test failed.\n");
	}
	/* Save hw reset test result */
	cnt = snprintf(save_buf + offset, SAVE_BUF_SIZE - offset, "HwReset Test Result = %s\n", result_hwreset ? "pass":"fail");
	offset += cnt;

	/* print the panel result */
	testing_hcd->result = ((result_flag & mask) == mask) ? true : false;

	cnt = snprintf(result_info, RESULT_INFO_LEN,
				"Firmware: %d Cfg: %02x %02x\nPanel Test Result = %s: PT1 = %s  PT3 = %s  PT5 = %s PT22 = %s  GAP_DIFF = %s PT18 = %s \
				PT44 gear0 = %s PT44 gear1 =  %s, PT44 gear2 =  %s, PT44 gearFF =  %s, PT81 = %s, PT85 = %s, PT86 = %s, HW reset = %s\n",
				tcm->tcm_dev->packrat_number,
				tcm->tcm_dev->app_info.customer_config_id[6] - 48,
				tcm->tcm_dev->app_info.customer_config_id[7] - 48,
				(testing_hcd->result) ? "pass" : "fail",
				(result_flag & (mask_base<<TEST_PID01_TRX_TRX_SHORTS)) ? "pass":"fail",
				(result_flag & (mask_base<<TEST_PID03_TRX_GROUND_SHORTS)) ? "pass":"fail",
				(result_flag & (mask_base<<TEST_PID05_FULL_RAW_CAP)) ? "pass":"fail",
				(result_flag & (mask_base<<TEST_PID22_TRANS_CAP_RAW)) ? "pass":"fail",
				(result_flag & (mask_base<<TEST_GAP_DIFF)) ? "pass":"fail",
				(result_flag & (mask_base<<TEST_PID18_HYBRID_ABS_RAW)) ? "pass":"fail",
				(result_flag & (mask_base<<TEST_PID44_MULTI_Gear_NOISE_TEST)) ? "pass":"fail",
				(result_flag & (mask_base<<0x13)) ? "pass":"fail",
				(result_flag & (mask_base<<0x14)) ? "pass":"fail",
				(result_flag & (mask_base<<0x15)) ? "pass":"fail",
				(result_flag & (mask_base<<0x18)) ? "pass":"fail",
				(result_flag & (mask_base<<0x19)) ? "pass":"fail",
				(result_flag & (mask_base<<0x1a)) ? "pass":"fail",
				result_hwreset ? "pass":"fail");

	LOGI("%s\n", result_info);

	cnt = snprintf(save_buf + offset, SAVE_BUF_SIZE - offset, "%s\n", result_info);
	offset += cnt;
	*length = offset;
	if (buf != NULL)
		snprintf(buf, RESULT_INFO_LEN, "%s", result_info);

exit:
	LOGN("Panel Test Result = %s\n", (testing_hcd->result)?"pass":"fail");

	return retval;
}


static ssize_t syna_testing_self_test_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	char *save_buf = g_save_buf;
	unsigned int length = 0;
	struct syna_tcm *tcm = g_tcm_ptr;
	char result_info[RESULT_INFO_LEN + 1] = {0};

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

        if (!save_buf) {
	        LOGE("Failed to allocate memory for printBuf\n");
	        retval = -ENOMEM;
	        goto exit;
        }

	memset(save_buf, 0, SAVE_BUF_SIZE);
	retval = syna_testing_self_test(result_info, save_buf, &length);
	if (retval < 0) {
		LOGE("Failed to do self_test\n");
		goto exit;
	}

	count = snprintf(buf, PAGE_SIZE, "%s\n", testing_hcd->result? "Passed" : "Failed");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_self_test =
	__ATTR(self_test, 0444, syna_testing_self_test_show, NULL);


/*
 * declaration of sysfs attributes
 */
static struct attribute *attrs[] = {
	&kobj_attr_check_id.attr,
	&kobj_attr_pt01.attr,
	&kobj_attr_pt03.attr,
	&kobj_attr_pt05.attr,
	&kobj_attr_pt44.attr,
	&kobj_attr_pt18.attr,
	&kobj_attr_pt22.attr,
	&kobj_attr_pt81.attr,
	&kobj_attr_pt85.attr,
	&kobj_attr_pt86.attr,
	&kobj_attr_gap_diff.attr,
	&kobj_attr_ptHwReset.attr,
	&kobj_attr_self_test.attr,
	NULL,
};

static struct attribute_group attr_testing_group = {
	.attrs = attrs,
};

static int syna_selftest_proc_show(struct seq_file *m, void *v)
{
	struct syna_tcm *tcm = g_tcm_ptr;
	char *save_buf = g_save_buf;

	if (!tcm->is_connected) {
		seq_puts(m,"Device is NOT connected\n");
		goto exit;
	}

	if (!save_buf) {
		seq_puts(m,"save_buf is NULL\n");
		goto exit;
	}

	seq_puts(m, save_buf);
exit:
	return 0;
}
static int syna_selftest_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, syna_selftest_proc_show,NULL);
}
static const struct proc_ops syna_selftest_proc_fops = {
	.proc_open           = syna_selftest_proc_open,
	.proc_read           = seq_read,
	.proc_lseek         = seq_lseek,
	.proc_release        = single_release,
};

static int testing_xiaomi_self_test(char *buf)
{
	int retval = 0;
	char result_info[RESULT_INFO_LEN + 1] = {0};
	struct syna_tcm *tcm_hcd = g_tcm_ptr;
	unsigned int length = 0;
	char *save_buf = g_save_buf;

	if (tcm_hcd->hw_if->ops_enable_irq) {
		retval = tcm_hcd->hw_if->ops_enable_irq(tcm_hcd->hw_if, true);
		if (retval < 0) {
			LOGE("Failed to enable interrupt\n");
			goto exit;
		}
	}

	retval = syna_testing_self_test(result_info, save_buf, &length);
	if (retval < 0) {
		LOGE("Failed to do self_test, testing_hcd->result = %d\n",testing_hcd->result);
		goto exit;
	}

exit:
	if (testing_hcd->result)
		retval = true;
	else
		retval = false;

	return retval;
}

static int testing_hcd_init(void)
{
	g_tcm_ptr->testing_hcd = kzalloc(sizeof(struct testing_hcd), GFP_KERNEL);
	testing_hcd = g_tcm_ptr->testing_hcd;
	if (!testing_hcd) {
		LOGE("Failed to allocate memory for testing_hcd\n");
		return -ENOMEM;
	}

	/* testing_hcd->collect_reports = testing_collect_reports; */

	INIT_BUFFER(testing_hcd->out);
	INIT_BUFFER(testing_hcd->resp);
	INIT_BUFFER(testing_hcd->report);
	INIT_BUFFER(testing_hcd->process);
	INIT_BUFFER(testing_hcd->output);
	INIT_BUFFER(testing_hcd->pt_hi_limits);
	INIT_BUFFER(testing_hcd->pt_lo_limits);
	testing_hcd->buffer_flag = true;

	/* tcm_hcd->testing_xiaomi_report_data = testing_xiaomi_report_data; */
	g_tcm_ptr->testing_xiaomi_self_test = testing_xiaomi_self_test;
	g_tcm_ptr->testing_xiaomi_chip_id_read = syna_testing_device_id;

	return 0;
}

/**
 * syna_testing_create_dir()
 *
 * Create a directory and register it with sysfs and proc
 * Then, create all defined sysfs files.
 *
 *
 * @param
 *    [ in] tcm:  the driver handle
 *    [ in] sysfs_dir: root directory of sysfs nodes
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_testing_create_dir(struct syna_tcm *tcm,
		struct kobject *sysfs_dir)
{
	int retval = 0;

	g_testing_dir = kobject_create_and_add("testing",
			sysfs_dir);
	if (!g_testing_dir) {
		LOGE("Fail to create testing directory\n");
		return -EINVAL;
	}

	retval = sysfs_create_group(g_testing_dir, &attr_testing_group);
	if (retval < 0) {
		LOGE("Fail to create sysfs group\n");
		kobject_put(g_testing_dir);
		return retval;
	}

	proc_selftest = proc_create("syna_selftest", 0, NULL, &syna_selftest_proc_fops);
	if (!proc_selftest) {
		LOGE("Failed to create proc/syna_selftest\n");
	        retval = -EINVAL;
	}

        g_save_buf = kzalloc(SAVE_BUF_SIZE, GFP_KERNEL);
        if (!g_save_buf) {
	        LOGE("Failed to allocate memory for printBuf\n");
	        retval = -ENOMEM;
        }

	g_tcm_ptr = tcm;

	testing_hcd_init();

	return 0;
}
/**
 *syna_testing_remove_dir()
 *
 * Remove the allocate sysfs directory
 *
 * @param
 *    none
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
void syna_testing_remove_dir(void)
{	
	if (testing_hcd != NULL && testing_hcd->buffer_flag) {
		RELEASE_BUFFER(testing_hcd->pt_lo_limits);
		RELEASE_BUFFER(testing_hcd->pt_hi_limits);
		RELEASE_BUFFER(testing_hcd->output);
		RELEASE_BUFFER(testing_hcd->process);
		RELEASE_BUFFER(testing_hcd->report);
		RELEASE_BUFFER(testing_hcd->resp);
		RELEASE_BUFFER(testing_hcd->out);
		testing_hcd->buffer_flag = false;
	}

	if (g_testing_dir) {
		sysfs_remove_group(g_testing_dir, &attr_testing_group);

		kobject_put(g_testing_dir);
	}

	if(proc_selftest){
	        proc_remove(proc_selftest);
	        proc_selftest = NULL;
        }

        if (g_save_buf){
	        kfree(g_save_buf);
	        g_save_buf = NULL;
        }
}
