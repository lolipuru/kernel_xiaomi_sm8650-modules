
// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/of_device.h>
#include <linux/sysfs.h>
#include <linux/sort.h>
#include <linux/atomic.h>
#include <linux/pinctrl/qcom-pinctrl.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/component.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/debugfs.h>
#include <linux/wait.h>
#include <linux/time.h>
#include "qts_core.h"
static struct qts_data_entries *qts_data_entries;
static void qts_trusted_touch_abort_handler(struct qts_data *qts_data, int error);
static struct gh_acl_desc *qts_vm_get_acl(enum gh_vm_names vm_name)
{
	struct gh_acl_desc *acl_desc;
	gh_vmid_t vmid;
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
	ghd_rm_get_vmid(vm_name, &vmid);
#else
	gh_rm_get_vmid(vm_name, &vmid);
#endif
	acl_desc = kzalloc(offsetof(struct gh_acl_desc, acl_entries[1]),
			GFP_KERNEL);
	if (!acl_desc)
		return ERR_PTR(ENOMEM);
	acl_desc->n_acl_entries = 1;
	acl_desc->acl_entries[0].vmid = vmid;
	acl_desc->acl_entries[0].perms = GH_RM_ACL_R | GH_RM_ACL_W;
	return acl_desc;
}
static struct gh_sgl_desc *qts_vm_get_sgl(struct trusted_touch_vm_info *vm_info)
{
	struct gh_sgl_desc *sgl_desc;
	int i;
	sgl_desc = kzalloc(offsetof(struct gh_sgl_desc,
			sgl_entries[vm_info->iomem_list_size]), GFP_KERNEL);
	if (!sgl_desc)
		return ERR_PTR(ENOMEM);
	sgl_desc->n_sgl_entries = vm_info->iomem_list_size;
	for (i = 0; i < vm_info->iomem_list_size; i++) {
		sgl_desc->sgl_entries[i].ipa_base = vm_info->iomem_bases[i];
		sgl_desc->sgl_entries[i].size = vm_info->iomem_sizes[i];
	}
	return sgl_desc;
}
static int qts_populate_vm_info_iomem(struct qts_data *qts_data)
{
	int i, gpio, rc = 0;
	int num_regs, num_sizes, num_gpios, list_size;
	struct resource res;
	struct device_node *np = qts_data->dev->of_node;
	struct trusted_touch_vm_info *vm_info = qts_data->vm_info;
	num_regs = of_property_count_u32_elems(np, "qts,trusted-touch-io-bases");
	if (num_regs < 0) {
		pr_err("Invalid number of IO regions specified\n");
		return -EINVAL;
	}
	num_sizes = of_property_count_u32_elems(np, "qts,trusted-touch-io-sizes");
	if (num_sizes < 0) {
		pr_err("Invalid number of IO regions specified\n");
		return -EINVAL;
	}
	if (num_regs != num_sizes) {
		pr_err("IO bases and sizes array lengths mismatch\n");
		return -EINVAL;
	}
	/* num_gpios = of_gpio_named_count(np, "qts,trusted-touch-vm-gpio-list"); */
	num_gpios = of_count_phandle_with_args(np, "qts,trusted-touch-vm-gpio-list", "#gpio-cells");
	/*new base use interface*/

	if (num_gpios < 0) {
		pr_warn("Ignoring invalid trusted gpio list: %d\n", num_gpios);
		num_gpios = 0;
	}
	list_size = num_regs + num_gpios;
	vm_info->iomem_list_size = list_size;
	vm_info->iomem_bases = devm_kcalloc(qts_data->dev, list_size, sizeof(*vm_info->iomem_bases),
			GFP_KERNEL);
	if (!vm_info->iomem_bases)
		return -ENOMEM;
	vm_info->iomem_sizes = devm_kcalloc(qts_data->dev, list_size, sizeof(*vm_info->iomem_sizes),
			GFP_KERNEL);
	if (!vm_info->iomem_sizes)
		return -ENOMEM;
	for (i = 0; i < num_gpios; ++i) {
		gpio = of_get_named_gpio(np, "qts,trusted-touch-vm-gpio-list", i);
		if (gpio < 0 || !gpio_is_valid(gpio)) {
			pr_err("Invalid gpio %d at position %d\n", gpio, i);
			return gpio;
		}
		if (!msm_gpio_get_pin_address(gpio, &res)) {
			pr_err("Failed to retrieve gpio-%d resource\n", gpio);
			return -ENODATA;
		}
		vm_info->iomem_bases[i] = res.start;
		vm_info->iomem_sizes[i] = resource_size(&res);
	}
	rc = of_property_read_u32_array(np, "qts,trusted-touch-io-bases",
			&vm_info->iomem_bases[i], list_size - i);
	if (rc) {
		pr_err("Failed to read trusted touch io bases:%d\n", rc);
		return rc;
	}
	rc = of_property_read_u32_array(np, "qts,trusted-touch-io-sizes",
			&vm_info->iomem_sizes[i], list_size - i);
	if (rc) {
		pr_err("Failed to read trusted touch io sizes:%d\n", rc);
		return rc;
	}
	return 0;
}
static int qts_populate_vm_info(struct qts_data *qts_data)
{
	int rc;
	struct trusted_touch_vm_info *vm_info;
	struct device_node *np = qts_data->dev->of_node;
	vm_info = devm_kzalloc(qts_data->dev, sizeof(struct trusted_touch_vm_info), GFP_KERNEL);
	if (!vm_info)
		return -ENOMEM;
	qts_data->vm_info = vm_info;
	vm_info->vm_name = GH_TRUSTED_VM;
	rc = of_property_read_u32(np, "qts,trusted-touch-spi-irq", &vm_info->hw_irq);
	if (rc) {
		pr_err("Failed to read trusted touch SPI irq:%d\n", rc);
		return rc;
	}
	rc = qts_populate_vm_info_iomem(qts_data);
	if (rc) {
		pr_err("Failed to read trusted touch mmio ranges:%d\n", rc);
		return rc;
	}
	rc = of_property_read_string(np, "qts,trusted-touch-type",
						&vm_info->trusted_touch_type);
	if (rc) {
		pr_warn("%s: No trusted touch type selection made\n", __func__);
		vm_info->mem_tag = GH_MEM_NOTIFIER_TAG_TOUCH_PRIMARY;
		vm_info->irq_label = GH_IRQ_LABEL_TRUSTED_TOUCH_PRIMARY;
		rc = 0;
	} else if (!strcmp(vm_info->trusted_touch_type, "primary")) {
		vm_info->mem_tag = GH_MEM_NOTIFIER_TAG_TOUCH_PRIMARY;
		vm_info->irq_label = GH_IRQ_LABEL_TRUSTED_TOUCH_PRIMARY;
	} else if (!strcmp(vm_info->trusted_touch_type, "secondary")) {
		vm_info->mem_tag = GH_MEM_NOTIFIER_TAG_TOUCH_SECONDARY;
		vm_info->irq_label = GH_IRQ_LABEL_TRUSTED_TOUCH_SECONDARY;
	}
	return 0;
}
static void qts_destroy_vm_info(struct qts_data *qts_data)
{
	kfree(qts_data->vm_info->iomem_sizes);
	kfree(qts_data->vm_info->iomem_bases);
	kfree(qts_data->vm_info);
}
static void qts_vm_deinit(struct qts_data *qts_data)
{
	if (qts_data->vm_info->mem_cookie)
		gh_mem_notifier_unregister(qts_data->vm_info->mem_cookie);
	qts_destroy_vm_info(qts_data);
}
static int qts_trusted_touch_get_vm_state(struct qts_data *qts_data)
{
	return atomic_read(&qts_data->vm_info->vm_state);
}
static void qts_trusted_touch_set_vm_state(struct qts_data *qts_data,
							int state)
{
	pr_err("state %d\n", state);
	atomic_set(&qts_data->vm_info->vm_state, state);
}
static void qts_bus_put(struct qts_data *qts_data);
static void qts_trusted_touch_abort_pvm(struct qts_data *qts_data)
{
	int rc = 0;
	int vm_state = qts_trusted_touch_get_vm_state(qts_data);
	if (vm_state >= TRUSTED_TOUCH_PVM_STATE_MAX) {
		pr_err("Invalid driver state: %d\n", vm_state);
		return;
	}
	switch (vm_state) {
	case PVM_IRQ_RELEASE_NOTIFIED:
	case PVM_ALL_RESOURCES_RELEASE_NOTIFIED:
	case PVM_IRQ_LENT:
	case PVM_IRQ_LENT_NOTIFIED:
		rc = gh_irq_reclaim(qts_data->vm_info->irq_label);
		if (rc)
			pr_err("failed to reclaim irq on pvm rc:%d\n", rc);
		fallthrough;
	case PVM_IRQ_RECLAIMED:
	case PVM_IOMEM_LENT:
	case PVM_IOMEM_LENT_NOTIFIED:
	case PVM_IOMEM_RELEASE_NOTIFIED:
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
		rc = ghd_rm_mem_reclaim(qts_data->vm_info->vm_mem_handle, 0);
#else
		rc = gh_rm_mem_reclaim(qts_data->vm_info->vm_mem_handle, 0);
#endif
		if (rc)
			pr_err("failed to reclaim iomem on pvm rc:%d\n", rc);
		qts_data->vm_info->vm_mem_handle = 0;
		fallthrough;
	case PVM_IOMEM_RECLAIMED:
	case PVM_INTERRUPT_DISABLED:
		if (qts_data->vendor_ops.enable_touch_irq)
			qts_data->vendor_ops.enable_touch_irq(qts_data->vendor_data, true);
		fallthrough;
	case PVM_I2C_RESOURCE_ACQUIRED:
	case PVM_INTERRUPT_ENABLED:
		qts_bus_put(qts_data);
		fallthrough;
	case TRUSTED_TOUCH_PVM_INIT:
	case PVM_I2C_RESOURCE_RELEASED:
		atomic_set(&qts_data->trusted_touch_enabled, 0);
		atomic_set(&qts_data->trusted_touch_transition, 0);
	}
	atomic_set(&qts_data->trusted_touch_abort_status, 0);
	qts_trusted_touch_set_vm_state(qts_data, TRUSTED_TOUCH_PVM_INIT);
}
static int qts_clk_prepare_enable(struct qts_data *qts_data)
{
	int ret;
	ret = clk_prepare_enable(qts_data->iface_clk);
	if (ret) {
		pr_err("error on clk_prepare_enable(iface_clk):%d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(qts_data->core_clk);
	if (ret) {
		clk_disable_unprepare(qts_data->iface_clk);
		pr_err("error clk_prepare_enable(core_clk):%d\n", ret);
	}
	return ret;
}
static void qts_clk_disable_unprepare(struct qts_data *qts_data)
{
	clk_disable_unprepare(qts_data->core_clk);
	clk_disable_unprepare(qts_data->iface_clk);
}
static int qts_bus_get(struct qts_data *qts_data)
{
	int rc = 0;
	struct device *dev = NULL;
	reinit_completion(&qts_data->trusted_touch_powerdown);
	if (qts_data->bus_type == QTS_BUS_TYPE_I2C)
		dev = qts_data->client->adapter->dev.parent;
	else
		dev = qts_data->spi->master->dev.parent;
	mutex_lock(&qts_data->qts_clk_io_ctrl_mutex);
	rc = pm_runtime_get_sync(dev);
	if (rc >= 0 &&  qts_data->core_clk != NULL &&
				qts_data->iface_clk != NULL) {
		rc = qts_clk_prepare_enable(qts_data);
		if (rc)
			pm_runtime_put_sync(dev);
	}
	mutex_unlock(&qts_data->qts_clk_io_ctrl_mutex);
	return rc;
}
static void qts_bus_put(struct qts_data *qts_data)
{
	struct device *dev = NULL;
	if (qts_data->bus_type == QTS_BUS_TYPE_I2C)
		dev = qts_data->client->adapter->dev.parent;
	else
		dev = qts_data->spi->master->dev.parent;
	mutex_lock(&qts_data->qts_clk_io_ctrl_mutex);
	if (qts_data->core_clk != NULL && qts_data->iface_clk != NULL)
		qts_clk_disable_unprepare(qts_data);
	pm_runtime_put_sync(dev);
	mutex_unlock(&qts_data->qts_clk_io_ctrl_mutex);
	complete(&qts_data->trusted_touch_powerdown);
}
static struct gh_notify_vmid_desc *qts_vm_get_vmid(gh_vmid_t vmid)
{
	struct gh_notify_vmid_desc *vmid_desc;
	vmid_desc = kzalloc(offsetof(struct gh_notify_vmid_desc,
				vmid_entries[1]), GFP_KERNEL);
	if (!vmid_desc)
		return ERR_PTR(ENOMEM);
	vmid_desc->n_vmid_entries = 1;
	vmid_desc->vmid_entries[0].vmid = vmid;
	return vmid_desc;
}
static void qts_trusted_touch_pvm_vm_mode_disable(struct qts_data *qts_data)
{
	int rc = 0;
	atomic_set(&qts_data->trusted_touch_transition, 1);
	if (atomic_read(&qts_data->trusted_touch_abort_status)) {
		qts_trusted_touch_abort_pvm(qts_data);
		return;
	}
	if (qts_trusted_touch_get_vm_state(qts_data) != PVM_ALL_RESOURCES_RELEASE_NOTIFIED)
		pr_err("all release notifications are not received yet\n");
	if (qts_data->vendor_ops.pre_la_tui_disable)
		qts_data->vendor_ops.pre_la_tui_disable(qts_data->vendor_data);
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
	rc = ghd_rm_mem_reclaim(qts_data->vm_info->vm_mem_handle, 0);
#else
	rc = gh_rm_mem_reclaim(qts_data->vm_info->vm_mem_handle, 0);
#endif
	if (rc) {
		pr_err("Trusted touch VM mem reclaim failed rc:%d\n", rc);
		goto error;
	}
	qts_trusted_touch_set_vm_state(qts_data, PVM_IOMEM_RECLAIMED);
	qts_data->vm_info->vm_mem_handle = 0;
	pr_err("vm mem reclaim success!\n");
	rc = gh_irq_reclaim(qts_data->vm_info->irq_label);
	if (rc) {
		pr_err("failed to reclaim irq on pvm rc:%d\n", rc);
		goto error;
	}
	qts_trusted_touch_set_vm_state(qts_data, PVM_IRQ_RECLAIMED);
	pr_err("vm irq reclaim success!\n");
	if (qts_data->vendor_ops.enable_touch_irq)
		qts_data->vendor_ops.enable_touch_irq(qts_data->vendor_data, true);
	qts_trusted_touch_set_vm_state(qts_data, PVM_INTERRUPT_ENABLED);
	qts_bus_put(qts_data);
	atomic_set(&qts_data->trusted_touch_transition, 0);
	qts_trusted_touch_set_vm_state(qts_data, PVM_I2C_RESOURCE_RELEASED);
	qts_trusted_touch_set_vm_state(qts_data, TRUSTED_TOUCH_PVM_INIT);
	atomic_set(&qts_data->trusted_touch_enabled, 0);
	if (qts_data->vendor_ops.post_la_tui_disable)
		qts_data->vendor_ops.post_la_tui_disable(qts_data->vendor_data);
	pr_err("trusted touch disabled\n");
	return;
error:
	qts_trusted_touch_abort_handler(qts_data,
			TRUSTED_TOUCH_EVENT_RECLAIM_FAILURE);
}
static void qts_vm_irq_on_release_callback(void *data,
					unsigned long notif_type,
					enum gh_irq_label label)
{
	struct qts_data *qts_data = data;
	if (notif_type != GH_RM_NOTIF_VM_IRQ_RELEASED) {
		pr_err("invalid notification type\n");
		return;
	}
	if (qts_trusted_touch_get_vm_state(qts_data) == PVM_IOMEM_RELEASE_NOTIFIED)
		qts_trusted_touch_set_vm_state(qts_data, PVM_ALL_RESOURCES_RELEASE_NOTIFIED);
	else
		qts_trusted_touch_set_vm_state(qts_data, PVM_IRQ_RELEASE_NOTIFIED);
}
static void qts_vm_mem_on_release_handler(enum gh_mem_notifier_tag tag,
		unsigned long notif_type, void *entry_data, void *notif_msg)
{
	struct gh_rm_notif_mem_released_payload *release_payload;
	struct trusted_touch_vm_info *vm_info;
	struct qts_data *qts_data;
	qts_data = (struct qts_data *)entry_data;
	if (!qts_data)
		return;
	vm_info = qts_data->vm_info;
	if (!vm_info) {
		pr_err("Invalid vm_info\n");
		return;
	}
	if (notif_type != GH_RM_NOTIF_MEM_RELEASED) {
		pr_err("Invalid notification type\n");
		return;
	}
	if (tag != vm_info->mem_tag) {
		pr_err("Invalid tag\n");
		return;
	}
	if (!entry_data || !notif_msg) {
		pr_err("Invalid data or notification message\n");
		return;
	}
	release_payload = (struct gh_rm_notif_mem_released_payload  *)notif_msg;
	if (release_payload->mem_handle != vm_info->vm_mem_handle) {
		pr_err("Invalid mem handle detected\n");
		return;
	}
	if (qts_trusted_touch_get_vm_state(qts_data) == PVM_IRQ_RELEASE_NOTIFIED)
		qts_trusted_touch_set_vm_state(qts_data, PVM_ALL_RESOURCES_RELEASE_NOTIFIED);
	else
		qts_trusted_touch_set_vm_state(qts_data, PVM_IOMEM_RELEASE_NOTIFIED);
}
static int qts_vm_mem_lend(struct qts_data *qts_data)
{
	struct gh_acl_desc *acl_desc;
	struct gh_sgl_desc *sgl_desc;
	struct gh_notify_vmid_desc *vmid_desc;
	gh_memparcel_handle_t mem_handle = 0;
	gh_vmid_t trusted_vmid = 0;
	int rc = 0;
	acl_desc = qts_vm_get_acl(GH_TRUSTED_VM);
	if (IS_ERR(acl_desc)) {
		pr_err("Failed to get acl of IO memories for Trusted touch\n");
		rc = PTR_ERR(acl_desc);
		return rc;
	}
	sgl_desc = qts_vm_get_sgl(qts_data->vm_info);
	if (IS_ERR(sgl_desc)) {
		pr_err("Failed to get sgl of IO memories for Trusted touch\n");
		rc = PTR_ERR(sgl_desc);
		goto sgl_error;
	}
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
	 rc = ghd_rm_mem_lend(GH_RM_MEM_TYPE_IO, 0, TRUSTED_TOUCH_MEM_LABEL,
	 		acl_desc, sgl_desc, NULL, &mem_handle);
#else
	 rc = gh_rm_mem_reclaim(qts_data->vm_info->vm_mem_handle, 0);
#endif
	 if (rc) {
	 	pr_err("Failed to lend IO memories for Trusted touch rc:%d\n", rc);
	 	goto error;
	 }
	pr_err("vm mem lend success\n");
	qts_trusted_touch_set_vm_state(qts_data, PVM_IOMEM_LENT);
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
	ghd_rm_get_vmid(GH_TRUSTED_VM, &trusted_vmid);
#else 
	gh_rm_get_vmid(GH_TRUSTED_VM, &trusted_vmid);
#endif
	vmid_desc = qts_vm_get_vmid(trusted_vmid);
	rc = gh_rm_mem_notify(mem_handle, GH_RM_MEM_NOTIFY_RECIPIENT_SHARED,
			qts_data->vm_info->mem_tag, vmid_desc);
	if (rc) {
		pr_err("Failed to notify mem lend to hypervisor rc:%d\n", rc);
		goto vmid_error;
	}
	qts_trusted_touch_set_vm_state(qts_data, PVM_IOMEM_LENT_NOTIFIED);
	qts_data->vm_info->vm_mem_handle = mem_handle;
vmid_error:
	kfree(vmid_desc);
error:
	kfree(sgl_desc);
sgl_error:
	kfree(acl_desc);
	return rc;
}
static int qts_trusted_touch_pvm_vm_mode_enable(struct qts_data *qts_data)
{
	int rc = 0;
	struct trusted_touch_vm_info *vm_info = qts_data->vm_info;
	atomic_set(&qts_data->trusted_touch_transition, 1);
	mutex_lock(&qts_data->transition_lock);
	if (qts_data->suspended) {
		pr_err("Invalid power state for operation\n");
		atomic_set(&qts_data->trusted_touch_transition, 0);
		rc =  -EPERM;
		goto error;
	}
	if (qts_data->vendor_ops.pre_la_tui_enable)
		qts_data->vendor_ops.pre_la_tui_enable(qts_data->vendor_data);
	/* i2c session start and resource acquire */
	if (qts_bus_get(qts_data) < 0) {
		pr_err("qts_bus_get failed\n");
		rc = -EIO;
		goto error;
	}
	qts_trusted_touch_set_vm_state(qts_data, PVM_I2C_RESOURCE_ACQUIRED);
	if (qts_data->vendor_ops.enable_touch_irq)
		qts_data->vendor_ops.enable_touch_irq(qts_data->vendor_data, false);
	qts_trusted_touch_set_vm_state(qts_data, PVM_INTERRUPT_DISABLED);
	msleep(5);
	rc = qts_vm_mem_lend(qts_data);
	if (rc) {
		pr_err("Failed to lend memory\n");
		goto abort_handler;
	}
	pr_err("vm mem lend success\n");
	if (atomic_read(&qts_data->delayed_pvm_probe_pending)) {
		if (qts_data->vendor_ops.get_irq_num)
			qts_data->irq = qts_data->vendor_ops.get_irq_num(qts_data->vendor_data);
		atomic_set(&qts_data->delayed_tvm_probe_pending, 0);
	}
	rc = gh_irq_lend_v2(vm_info->irq_label, vm_info->vm_name,
		qts_data->irq, &qts_vm_irq_on_release_callback, qts_data);
	if (rc) {
		pr_err("Failed to lend irq\n");
		goto abort_handler;
	}
	pr_err("vm irq lend success for irq:%d\n", qts_data->irq);
	qts_trusted_touch_set_vm_state(qts_data, PVM_IRQ_LENT);
	rc = gh_irq_lend_notify(vm_info->irq_label);
	if (rc) {
		pr_err("Failed to notify irq\n");
		goto abort_handler;
	}
	qts_trusted_touch_set_vm_state(qts_data, PVM_IRQ_LENT_NOTIFIED);
	if (qts_data->vendor_ops.post_la_tui_enable)
		qts_data->vendor_ops.post_la_tui_enable(qts_data->vendor_data);
	mutex_unlock(&qts_data->transition_lock);
	atomic_set(&qts_data->trusted_touch_transition, 0);
	atomic_set(&qts_data->trusted_touch_enabled, 1);
	pr_err("trusted touch enabled\n");
	return rc;
abort_handler:
	qts_trusted_touch_abort_handler(qts_data, TRUSTED_TOUCH_EVENT_LEND_FAILURE);
error:
	mutex_unlock(&qts_data->transition_lock);
	return rc;
}
static int qts_handle_trusted_touch_pvm(struct qts_data *qts_data, int value)
{
	int err = 0;
	switch (value) {
	case 0:
		if (atomic_read(&qts_data->trusted_touch_enabled) == 0 &&
			(atomic_read(&qts_data->trusted_touch_abort_status) == 0)) {
			pr_err("Trusted touch is already disabled\n");
			break;
		}
		if (atomic_read(&qts_data->trusted_touch_mode) ==
				TRUSTED_TOUCH_VM_MODE) {
			qts_trusted_touch_pvm_vm_mode_disable(qts_data);
		} else {
			pr_err("Unsupported trusted touch mode\n");
		}
		break;
	case 1:
		if (atomic_read(&qts_data->trusted_touch_enabled)) {
			pr_err("Trusted touch usecase underway\n");
			err = -EBUSY;
			break;
		}
		if (atomic_read(&qts_data->trusted_touch_mode) ==
				TRUSTED_TOUCH_VM_MODE) {
			err = qts_trusted_touch_pvm_vm_mode_enable(qts_data);
		} else {
			pr_err("Unsupported trusted touch mode\n");
		}
		break;
	default:
		pr_err("unsupported value: %d\n", value);
		err = -EINVAL;
		break;
	}
	return err;
}
static void qts_trusted_touch_event_notify(struct qts_data *qts_data, int event)
{
	atomic_set(&qts_data->trusted_touch_event, event);
	sysfs_notify(&qts_data->dev->kobj, NULL, "trusted_touch_event");
}
static void qts_trusted_touch_abort_handler(struct qts_data *qts_data, int error)
{
	atomic_set(&qts_data->trusted_touch_abort_status, error);
	pr_info("TUI session aborted with failure:%d\n", error);
	qts_trusted_touch_event_notify(qts_data, error);
}
static int qts_vm_init(struct qts_data *qts_data)
{
	int rc = 0;
	struct trusted_touch_vm_info *vm_info;
	void *mem_cookie;
	rc = qts_populate_vm_info(qts_data);
	if (rc) {
		pr_err("Cannot setup vm pipeline\n");
		rc = -EINVAL;
		goto fail;
	}
	vm_info = qts_data->vm_info;
	mem_cookie = gh_mem_notifier_register(vm_info->mem_tag,
			qts_vm_mem_on_release_handler, qts_data);
	if (!mem_cookie) {
		pr_err("Failed to register on release mem notifier\n");
		rc = -EINVAL;
		goto init_fail;
	}
	vm_info->mem_cookie = mem_cookie;
	qts_trusted_touch_set_vm_state(qts_data, TRUSTED_TOUCH_PVM_INIT);
	return rc;
init_fail:
	qts_vm_deinit(qts_data);
fail:
	return rc;
}
static void qts_dt_parse_trusted_touch_info(struct qts_data *qts_data)
{
	struct device_node *np = qts_data->dev->of_node;
	int rc = 0;
	const char *selection;
	const char *environment;
	rc = of_property_read_string(np, "qts,trusted-touch-mode", &selection);
	if (rc) {
		pr_err("No trusted touch mode selection made\n");
		atomic_set(&qts_data->trusted_touch_mode,
						TRUSTED_TOUCH_MODE_NONE);
		return;
	}
	if (!strcmp(selection, "vm_mode")) {
		atomic_set(&qts_data->trusted_touch_mode,
						TRUSTED_TOUCH_VM_MODE);
		pr_err("Selected trusted touch mode to VM mode\n");
	} else {
		atomic_set(&qts_data->trusted_touch_mode,
						TRUSTED_TOUCH_MODE_NONE);
		pr_err("Invalid trusted_touch mode\n");
	}
	rc = of_property_read_string(np, "qts,touch-environment",
						&environment);
	if (rc)
		pr_err("No trusted touch mode environment\n");
	qts_data->touch_environment = environment;
	qts_data->tui_supported = true;
	pr_err("Trusted touch environment:%s\n", qts_data->touch_environment);
}
static void qts_trusted_touch_init(struct qts_data *qts_data)
{
	atomic_set(&qts_data->trusted_touch_initialized, 0);
	qts_dt_parse_trusted_touch_info(qts_data);
	if (atomic_read(&qts_data->trusted_touch_mode) == TRUSTED_TOUCH_MODE_NONE)
		return;
	init_completion(&qts_data->trusted_touch_powerdown);
	/* Get clocks */
	qts_data->core_clk = devm_clk_get(qts_data->dev->parent, "m-ahb");
	if (IS_ERR(qts_data->core_clk)) {
		qts_data->core_clk = NULL;
		pr_err("core_clk is not defined\n");
	}
	qts_data->iface_clk = devm_clk_get(qts_data->dev->parent, "se-clk");
	if (IS_ERR(qts_data->iface_clk)) {
		qts_data->iface_clk = NULL;
		pr_err("iface_clk is not defined\n");
	}
	if (atomic_read(&qts_data->trusted_touch_mode) ==
						TRUSTED_TOUCH_VM_MODE) {
		int rc = 0;
		rc = qts_vm_init(qts_data);
		if (rc)
			pr_err("Failed to init VM\n");
	}
	atomic_set(&qts_data->trusted_touch_initialized, 1);
}
static bool qts_ts_is_primary(struct kobject *kobj)
{
	char *path = NULL;
	if (!kobj)
		return true;
	path = kobject_get_path(kobj, GFP_KERNEL);
	if (strstr(path, "primary"))
		return true;
	else
		return false;
}
static ssize_t trusted_touch_enable_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	struct qts_data *qts_data;
	u32 idx = qts_ts_is_primary(kobj) ? 0 : 1;
	qts_data = &qts_data_entries->info[idx];
	return scnprintf(buf, PAGE_SIZE, "%d",
			atomic_read(&qts_data->trusted_touch_enabled));
}
static ssize_t trusted_touch_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	struct qts_data *qts_data;
	unsigned long value;
	int err = 0;
	u32 idx = qts_ts_is_primary(kobj) ? 0 : 1;
	if (count > 2)
		return -EINVAL;
	err = kstrtoul(buf, 10, &value);
	if (err != 0)
		return err;
	qts_data = &qts_data_entries->info[idx];
	if (!atomic_read(&qts_data->trusted_touch_initialized))
		return -EIO;
	err = qts_handle_trusted_touch_pvm(qts_data, value);
	if (err) {
		pr_err("Failed to handle trusted touch in pvm\n");
		return -EINVAL;
	}
	return count;
}
static ssize_t trusted_touch_event_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	struct qts_data *qts_data;
	u32 idx = qts_ts_is_primary(kobj) ? 0 : 1;
	qts_data = &qts_data_entries->info[idx];
	return scnprintf(buf, PAGE_SIZE, "%d",
			atomic_read(&qts_data->trusted_touch_event));
}
static ssize_t trusted_touch_event_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	struct qts_data *qts_data;
	unsigned long value;
	int err = 0;
	u32 idx = qts_ts_is_primary(kobj) ? 0 : 1;
	if (count > 2)
		return -EINVAL;
	err = kstrtoul(buf, 10, &value);
	if (err != 0)
		return err;
	qts_data = &qts_data_entries->info[idx];
	if (!atomic_read(&qts_data->trusted_touch_initialized))
		return -EIO;
	if (value)
		return -EIO;
	atomic_set(&qts_data->trusted_touch_event, value);
	return count;
}
static ssize_t trusted_touch_type_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct qts_data *qts_data;
	u32 idx = qts_ts_is_primary(kobj) ? 0 : 1;
	qts_data = &qts_data_entries->info[idx];
	return scnprintf(buf, PAGE_SIZE, "%s", qts_data->vm_info->trusted_touch_type);
}
static ssize_t trusted_touch_device_path_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	struct qts_data *qts_data;
	char *path = NULL;
	u32 idx = qts_ts_is_primary(kobj) ? 0 : 1;
	qts_data = &qts_data_entries->info[idx];
	if (qts_data && qts_data->dev)
		path = kobject_get_path(&qts_data->dev->kobj, GFP_KERNEL);
	return scnprintf(buf, PAGE_SIZE, "%s", path ? path : "");
}
static struct kobj_attribute trusted_touch_event_attr =
	__ATTR(trusted_touch_event, 0664, trusted_touch_event_show, trusted_touch_event_store);
static struct kobj_attribute trusted_touch_type_attr =
	__ATTR(trusted_touch_type, 0444, trusted_touch_type_show, NULL);
static struct kobj_attribute trusted_touch_device_path_attr =
	__ATTR(trusted_touch_device_path, 0444, trusted_touch_device_path_show, NULL);
static struct kobj_attribute trusted_touch_enable_attr =
	__ATTR(trusted_touch_enable, 0664, trusted_touch_enable_show, trusted_touch_enable_store);
static struct attribute *qts_attributes[] = {
	&trusted_touch_event_attr.attr,
	&trusted_touch_type_attr.attr,
	&trusted_touch_device_path_attr.attr,
	&trusted_touch_enable_attr.attr,
	NULL,
};
static struct attribute_group qts_attribute_group = {
	.attrs = qts_attributes,
};
static int qts_create_sysfs(struct qts_data *qts_data)
{
	int ret = 0;
	struct kobject *qts_kobj;
	struct kobject *client_kobj;
	qts_kobj = &qts_data_entries->qts_kset->kobj;
	if (qts_data->client_type == QTS_CLIENT_PRIMARY_TOUCH) {
		client_kobj = kobject_create_and_add("primary", qts_kobj);
		if (!client_kobj) {
			pr_err("primary kobject_create_and_add failed\n");
			return -ENOMEM;
		}
		ret = sysfs_create_group(client_kobj, &qts_attribute_group);
		if (ret) {
			pr_err("[EX]: sysfs_create_group() failed!!\n");
			sysfs_remove_group(client_kobj, &qts_attribute_group);
			return -ENOMEM;
		}
	} else if (qts_data->client_type == QTS_CLIENT_SECONDARY_TOUCH) {
		client_kobj = kobject_create_and_add("secondary", qts_kobj);
		if (!client_kobj) {
			pr_err("secondary kobject_create_and_add failed\n");
			return -ENOMEM;
		}
		ret = sysfs_create_group(client_kobj, &qts_attribute_group);
		if (ret) {
			pr_err("[EX]: sysfs_create_group() failed!!\n");
			sysfs_remove_group(client_kobj, &qts_attribute_group);
			return -ENOMEM;
		}
	}
	pr_err("sysfs_create_group() succeeded\n");
	return ret;
}
void qts_ts_suspend(struct qts_data *qts_data)
{
	if (qts_data->suspended) {
		pr_warn("already in suspend state\n");
		return;
	}
	if (qts_data->tui_supported) {
		if (atomic_read(&qts_data->trusted_touch_transition)
			|| atomic_read(&qts_data->trusted_touch_enabled))
			wait_for_completion_interruptible(&qts_data->trusted_touch_powerdown);
	}
	mutex_lock(&qts_data->transition_lock);
	qts_data->suspended = true;
	mutex_unlock(&qts_data->transition_lock);
}
EXPORT_SYMBOL(qts_ts_suspend);
void qts_ts_resume(struct qts_data *qts_data)
{
	if (!qts_data->suspended) {
		pr_warn("Already in awake state\n");
		return;
	}
	if (qts_data->tui_supported)
		if (atomic_read(&qts_data->trusted_touch_transition))
			wait_for_completion_interruptible(&qts_data->trusted_touch_powerdown);
	mutex_lock(&qts_data->transition_lock);
	qts_data->suspended = false;
	mutex_unlock(&qts_data->transition_lock);
}
EXPORT_SYMBOL(qts_ts_resume);
int qts_client_register(struct qts_vendor_data qts_vendor_data)
{
	struct qts_data *qts_data;
	struct device_node *dp = qts_vendor_data.spi->dev.of_node;
	int rc = 0;
	if (!qts_data_entries) {
		pr_err("QTS client register\n");
		qts_data_entries = kzalloc(sizeof(*qts_data_entries), GFP_KERNEL);
		if (!qts_data_entries) {
			pr_err("mem allocation failed\n");
			return -EPROBE_DEFER;
		}
		mutex_init(&qts_data_entries->qts_data_entries_lock);
		qts_data_entries->qts_kset = kset_create_and_add("qts", NULL, kernel_kobj);
		if (!qts_data_entries->qts_kset) {
			pr_err("qts kset create failed\n");
			return -ENOMEM;
		}
	}
	mutex_lock(&qts_data_entries->qts_data_entries_lock);
	if (qts_vendor_data.bus_type == QTS_BUS_TYPE_I2C)
		dp = qts_vendor_data.client->dev.of_node;
	else
		dp = qts_vendor_data.spi->dev.of_node;
	pr_err("QTS client register starts\n");
	qts_data = &qts_data_entries->info[qts_vendor_data.client_type];
	qts_data->client = qts_vendor_data.client;
	qts_data->spi = qts_vendor_data.spi;
	if (qts_vendor_data.bus_type == QTS_BUS_TYPE_I2C)
		qts_data->dev = &qts_data->client->dev;
	else
		qts_data->dev = &qts_data->spi->dev;
	qts_data->bus_type = qts_vendor_data.bus_type;
	qts_data->client_type = qts_vendor_data.client_type;
	qts_data->dp = dp;
	qts_data->vendor_data = qts_vendor_data.vendor_data;
	qts_data->vendor_ops = qts_vendor_data.qts_vendor_ops;
	qts_trusted_touch_init(qts_data);
	mutex_init(&(qts_data->qts_clk_io_ctrl_mutex));
	if (qts_data->tui_supported)
		qts_create_sysfs(qts_data);
	mutex_init(&qts_data->transition_lock);
	atomic_set(&qts_data->delayed_pvm_probe_pending, 1);
	pr_err("client register end\n");
	mutex_unlock(&qts_data_entries->qts_data_entries_lock);
	rc = 0;
	return rc;
}
EXPORT_SYMBOL(qts_client_register);

struct  qts_data *get_qts_data_helper(struct  qts_vendor_data *qts_vendor_data) {
	const int id = qts_vendor_data->client_type;
	return &qts_data_entries->info[id];
}
EXPORT_SYMBOL(get_qts_data_helper);

