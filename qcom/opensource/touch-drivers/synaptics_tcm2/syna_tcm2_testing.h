/* SPDX-License-Identifier: GPL-2.0
 *
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2018 Synaptics Incorporated. All rights reserved.
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

#include "syna_tcm2.h"

#ifndef _SYNAPTICS_TCM2_TESTING_H_
#define _SYNAPTICS_TCM2_TESTING_H_

#define INIT_BUFFER(buffer) \
	syna_tcm_buf_init(&buffer)

#define LOCK_BUFFER(buffer) \
	syna_tcm_buf_lock(&buffer)

#define UNLOCK_BUFFER(buffer) \
	syna_tcm_buf_unlock(&buffer)

#define RELEASE_BUFFER(buffer) \
	syna_tcm_buf_release(&buffer)

enum report_type {
	REPORT_IDENTIFY_TEST = 0x10,
	REPORT_TOUCH_TEST = 0x11,
	REPORT_DELTA_TEST = 0x12,
	REPORT_RAW_TEST = 0x13,
	REPORT_STATUS_TEST = 0x1b,
	REPORT_PRINTF_TEST = 0x82,
	REPORT_RID161_TEST = 0xA1,
	REPORT_THP_TEST = 0xC0,
	REPORT_HDL_ROMBOOT_TEST = 0xfd,
	REPORT_HDL_F35_TEST = 0xfe,
};

struct testing_hcd {
	bool result;
	unsigned char report_type;
	enum tcm_test_code test_item;
	unsigned int report_index;
	unsigned int num_of_reports;
	struct kobject *sysfs_dir;
	struct tcm_buffer out;
	struct tcm_buffer resp;
	struct tcm_buffer report;
	struct tcm_buffer process;
	struct tcm_buffer output;
	struct tcm_buffer pt_hi_limits;
	struct tcm_buffer pt_lo_limits;
	bool buffer_flag;
	int (*collect_reports)(enum report_type report_type,
			unsigned int num_of_reports);
};

/**
 * syna_testing_create_dir()
 *
 * Create a directory and register it with sysfs.
 * Then, create all defined sysfs files.
 *
 * @param
 *    [ in] tcm:  the driver handle
 *    [ in] sysfs_dir: root directory of sysfs nodes
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_testing_create_dir(struct syna_tcm *tcm,
		struct kobject *sysfs_dir);
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
void syna_testing_remove_dir(void);


#endif /* end of _SYNAPTICS_TCM2_TESTING_H_ */
