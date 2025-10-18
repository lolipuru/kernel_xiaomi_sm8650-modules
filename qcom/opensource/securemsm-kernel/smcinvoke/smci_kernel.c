// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fdtable.h>
#include <linux/anon_inodes.h>
#include <linux/delay.h>
#include <linux/kref.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/elf.h>
#if IS_ENABLED(CONFIG_QCOM_SMCI_PROXY)
#include <linux/smci_object.h>
#include <linux/smci_clientenv.h>
#include "smcinvoke.h"

#define SMCINVOKE_INTERFACE_MAX_RETRY       5
#define SMCINVOKE_INTERFACE_BUSY_WAIT_MS    5

struct tzobject_context {
	int fd;
	struct kref refs;
};

int smcinvoke_release_from_kernel_client(int fd);

int get_root_fd(int *root_fd);

int process_invoke_request_from_kernel_client(
		int fd, struct smcinvoke_cmd_req *req);

static int32_t smci_invoke_over_smcinvoke(void *cxt,
			uint32_t op,
			union smci_object_arg *args,
			uint32_t counts);

static struct smci_object tzobject_new(int fd)
{
	struct tzobject_context *me =
			kzalloc(sizeof(struct tzobject_context), GFP_KERNEL);
	if (!me)
		return SMCI_OBJECT_NULL;

	kref_init(&me->refs);
	me->fd = fd;
	pr_debug("%s: me->fd = %d, me->refs = %u\n", __func__,
			me->fd, kref_read(&me->refs));
	return (struct smci_object) { smci_invoke_over_smcinvoke, me };
}

static void tzobject_delete(struct kref *refs)
{
	struct tzobject_context *me = container_of(refs,
				struct tzobject_context, refs);

	pr_info("%s: me->fd = %d, me->refs = %d, files = %p\n",
		__func__, me->fd, kref_read(&me->refs), current->files);
	/*
	 * after _close_fd(), ref_cnt will be 0,
	 * but smcinvoke_release() was still not called,
	 * so we first call smcinvoke_release_from_kernel_client() to
	 * free filp and ask TZ to release object, then call _close_fd()
	 */
	smcinvoke_release_from_kernel_client(me->fd);
	close_fd(me->fd);
	kfree(me);
}

static int get_object_from_handle(int handle, struct smci_object *obj)
{
	int ret = 0;

	if (handle == SMCINVOKE_USERSPACE_OBJ_NULL) {
		/* NULL object*/
		 SMCI_OBJECT_ASSIGN_NULL(*obj);
	} else if (handle > SMCINVOKE_USERSPACE_OBJ_NULL) {
		*obj = tzobject_new(handle);
		if (SMCI_OBJECT_IS_NULL(*obj))
			ret = SMCI_OBJECT_ERROR_BADOBJ;
	} else {
		pr_err("CBobj not supported for handle %d\n", handle);
		ret = SMCI_OBJECT_ERROR_BADOBJ;
	}

	return ret;
}

static int get_handle_from_object(struct smci_object obj, int *handle)
{
	int ret = 0;

	if (SMCI_OBJECT_IS_NULL(obj)) {
	/* set NULL Object's fd to be -1 */
		*handle = SMCINVOKE_USERSPACE_OBJ_NULL;
		return ret;
	}

	if (obj.invoke == smci_invoke_over_smcinvoke) {
		struct tzobject_context *ctx = (struct tzobject_context *)(obj.context);

		if (ctx != NULL) {
			*handle = ctx->fd;
		} else {
			pr_err("Failed to get tzobject_context obj handle, ret = %d\n", ret);
			ret = SMCI_OBJECT_ERROR_BADOBJ;
		}
	} else {
		pr_err("CBobj not supported\n");
		ret = SMCI_OBJECT_ERROR_BADOBJ;
	}

	return ret;
}

static int marshal_in(struct smcinvoke_cmd_req *req,
			union smcinvoke_arg *argptr,
			uint32_t op, union smci_object_arg *args,
			uint32_t counts)
{
	size_t i = 0;

	req->op = op;
	req->counts = counts;
	req->argsize = sizeof(union smcinvoke_arg);
	req->args = (uintptr_t)argptr;

	SMCI_FOR_ARGS(i, counts, BUFFERS) {
		argptr[i].b.addr = (uintptr_t) args[i].b.ptr;
		argptr[i].b.size = args[i].b.size;
	}

	SMCI_FOR_ARGS(i, counts, OI) {
		int handle = -1, ret;

		ret = get_handle_from_object(args[i].o, &handle);
		if (ret) {
			pr_err("invalid OI[%zu]\n", i);
			return SMCI_OBJECT_ERROR_BADOBJ;
		}
		argptr[i].o.fd = handle;
	}

	SMCI_FOR_ARGS(i, counts, OO) {
		argptr[i].o.fd = SMCINVOKE_USERSPACE_OBJ_NULL;
	}
	return SMCI_OBJECT_OK;
}

static int marshal_out(struct smcinvoke_cmd_req *req,
			union smcinvoke_arg *argptr,
			union smci_object_arg *args, uint32_t counts,
			struct tzobject_context *me)
{
	int ret = req->result;
	bool failed = false;
	size_t i = 0;

	argptr = (union smcinvoke_arg *)(uintptr_t)(req->args);

	SMCI_FOR_ARGS(i, counts, BO) {
		args[i].b.size = argptr[i].b.size;
	}

	SMCI_FOR_ARGS(i, counts, OO) {
		ret = get_object_from_handle(argptr[i].o.fd, &(args[i].o));
		if (ret) {
			pr_err("Failed to get OO[%zu] from handle = %d\n",
				i, (int)argptr[i].o.fd);
			failed = true;
			break;
		}
		pr_debug("Succeed to create OO for args[%zu].o, fd = %d\n",
			i, (int)argptr[i].o.fd);
	}
	if (failed) {
		SMCI_FOR_ARGS(i, counts, OO) {
			SMCI_OBJECT_ASSIGN_NULL(args[i].o);
		}
		/* Only overwrite ret value if invoke result is 0 */
		if (ret == 0)
			ret = SMCI_OBJECT_ERROR_BADOBJ;
	}
	return ret;
}

static int smci_invoke_over_smcinvoke(void *cxt,
			uint32_t op,
			union smci_object_arg *args,
			uint32_t counts)
{
	int ret = SMCI_OBJECT_OK;
	struct smcinvoke_cmd_req req = {0, 0, 0, 0, 0};
	size_t i = 0;
	struct tzobject_context *me = NULL;
	uint32_t method;
	union smcinvoke_arg *argptr = NULL;

	SMCI_FOR_ARGS(i, counts, OO) {
		args[i].o = SMCI_OBJECT_NULL;
	}

	me = (struct tzobject_context *)cxt;
	method = SMCI_OBJECT_OP_METHODID(op);
	pr_debug("%s: cxt = %p, fd = %d, op = %u, cnt = %x, refs = %u\n",
			__func__, me, me->fd, op, counts, kref_read(&me->refs));

	if (SMCI_OBJECT_OP_IS_LOCAL(op)) {
		switch (method) {
		case SMCI_OBJECT_OP_RETAIN:
			kref_get(&me->refs);
			return SMCI_OBJECT_OK;
		case SMCI_OBJECT_OP_RELEASE:
			kref_put(&me->refs, tzobject_delete);
			return SMCI_OBJECT_OK;
		}
		return SMCI_OBJECT_ERROR_REMOTE;
	}

	argptr = kcalloc(SMCI_OBJECT_COUNTS_TOTAL(counts),
			sizeof(union smcinvoke_arg), GFP_KERNEL);
	if (argptr == NULL)
		return SMCI_OBJECT_ERROR_KMEM;

	ret = marshal_in(&req, argptr, op, args, counts);
	if (ret)
		goto exit;

	ret = process_invoke_request_from_kernel_client(me->fd, &req);
	if (ret) {
		pr_err("INVOKE failed with ret = %d, result = %d\n"
			"obj.context = %p, fd = %d, op = %d, counts = 0x%x\n",
			ret, req.result, me, me->fd, op, counts);
		SMCI_FOR_ARGS(i, counts, OO) {
			struct smcinvoke_obj obj = argptr[i].o;

			if (obj.fd >= 0) {
				pr_err("Close OO[%zu].fd = %lld\n", i, obj.fd);
				close_fd(obj.fd);
			}
		}
		if (ret == -EBUSY) {
			ret = SMCI_OBJECT_ERROR_BUSY;
		}
		else if (ret == -ENOMEM){
			ret = SMCI_OBJECT_ERROR_KMEM;
		} else {
			ret = SMCI_OBJECT_ERROR_UNAVAIL;
		}
		goto exit;
	}

	if (!req.result)
		ret = marshal_out(&req, argptr, args, counts, me);
exit:
	kfree(argptr);
	return ret | req.result;
}

static int smci_get_root_obj(struct smci_object *rootObj)
{
	int ret = 0;
	int root_fd = -1;

	ret = get_root_fd(&root_fd);
	if (ret) {
		pr_err("Failed to get root fd, ret = %d\n", ret);
		return ret;
	}
	*rootObj = tzobject_new(root_fd);
	if (SMCI_OBJECT_IS_NULL(*rootObj)) {
		close_fd(root_fd);
		ret = -ENOMEM;
	}
	return ret;
}

/*
 * Get a client environment using a NULL credentials smci object
 */
static int32_t __smci_get_client_env_object(struct smci_object *client_env_obj)
{
	int32_t  ret = SMCI_OBJECT_ERROR;
	int retry_count = 0;
	struct smci_object rootObj = SMCI_OBJECT_NULL;

	/* get rootObj */
	ret = smci_get_root_obj(&rootObj);
	if (ret) {
		pr_err("Failed to create rootobj\n");
		return ret;
	}

	/* get client env */
	do {
		ret = smci_clientenv_registerwithcredentials(rootObj,
			SMCI_OBJECT_NULL, client_env_obj);
		if (ret == SMCI_OBJECT_ERROR_BUSY) {
			pr_err("Secure side is busy,will retry after 5 ms, retry_count = %d",retry_count);
			msleep(SMCINVOKE_INTERFACE_BUSY_WAIT_MS);
		}
	} while ((ret == SMCI_OBJECT_ERROR_BUSY) && (retry_count++ < SMCINVOKE_INTERFACE_MAX_RETRY));

	if (ret)
		pr_err("Failed to smci get client env object, ret = %d\n", ret);
	SMCI_OBJECT_RELEASE_IF(rootObj);
	return ret;
}

const static struct smci_drv_ops smci_driver_ops = {
	.smci_get_client_env_object = __smci_get_client_env_object,
};

int get_smci_kernel_fun_ops(void)
{
	return provide_smci_kernel_fun_ops(&smci_driver_ops);
}
#endif