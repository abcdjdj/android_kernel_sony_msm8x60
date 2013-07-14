/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/ioctl.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include "msm.h"
#include "msm_cam_server.h"
#include "msm_sensor.h"
#include "msm_actuator.h"
#include "msm_camera_eeprom.h"

#define MSM_MAX_CAMERA_SENSORS 5

#ifdef CONFIG_MSM_CAMERA_DEBUG
#define D(fmt, args...) pr_debug("msm: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

static unsigned msm_camera_v4l2_nr = -1;
static int vnode_count;

module_param(msm_camera_v4l2_nr, uint, 0644);
MODULE_PARM_DESC(msm_camera_v4l2_nr, "videoX start number, -1 is autodetect");

/* callback function from all subdevices of a msm_cam_v4l2_device */
static void msm_cam_v4l2_subdev_notify(struct v4l2_subdev *sd,
				unsigned int notification, void *arg)
{
	struct msm_cam_v4l2_device *pcam;
	struct msm_cam_media_controller *pmctl;

	if (sd == NULL)
		return;

	pcam = to_pcam(sd->v4l2_dev);

	if (pcam == NULL)
		return;

	pmctl = msm_cam_server_get_mctl(pcam->mctl_handle);
	if (pmctl == NULL)
		return;
}

/*
 *
 * implementation of v4l2_ioctl_ops
 *
 */
static int msm_camera_v4l2_querycap(struct file *f, void *pctx,
				struct v4l2_capability *pcaps)
{

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	/* some other day, some other time */
	/*cap->version = LINUX_VERSION_CODE; */
	pcaps->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}

static int msm_camera_v4l2_queryctrl(struct file *f, void *pctx,
				struct v4l2_queryctrl *pqctrl)
{
	int rc = 0;
#if defined(CONFIG_SONY_CAM_MAIN_V4L2) || defined(CONFIG_SONY_CAM_SUB_V4L2)
	int cnt = 0;
#endif
	void *value;
	struct msm_queue_cmd *rcmd;
	struct msm_queue_cmd *event_qcmd;
	struct msm_ctrl_cmd *ctrlcmd;
	struct msm_device_queue *queue =
		&server_dev->server_queue[out->queue_idx].ctrl_q;
	struct msm_cam_v4l2_device *pcam = server_dev->pcam_active;

	struct v4l2_event v4l2_evt;
	struct msm_isp_event_ctrl *isp_event;
	void *ctrlcmd_data;

	event_qcmd = kzalloc(sizeof(struct msm_queue_cmd), GFP_KERNEL);
	if (!event_qcmd) {
		pr_err("%s Insufficient memory. return", __func__);
		rc = -ENOMEM;
		goto event_qcmd_alloc_fail;
	}

	isp_event = kzalloc(sizeof(struct msm_isp_event_ctrl), GFP_KERNEL);
	if (!isp_event) {
		pr_err("%s Insufficient memory. return", __func__);
		rc = -ENOMEM;
		goto isp_event_alloc_fail;
	}

	D("%s\n", __func__);
	mutex_lock(&server_dev->server_queue_lock);
	if (++server_dev->server_evt_id == 0)
		server_dev->server_evt_id++;
	D("%s qid %d evtid %d\n", __func__, out->queue_idx,
		server_dev->server_evt_id);

	server_dev->server_queue[out->queue_idx].evt_id =
		server_dev->server_evt_id;
	v4l2_evt.type = V4L2_EVENT_PRIVATE_START + MSM_CAM_RESP_V4L2;
	v4l2_evt.id = 0;
	v4l2_evt.u.data[0] = out->queue_idx;
	/* setup event object to transfer the command; */
	isp_event->resptype = MSM_CAM_RESP_V4L2;
	isp_event->isp_data.ctrl = *out;
	isp_event->isp_data.ctrl.evt_id = server_dev->server_evt_id;

	if (out->value != NULL && out->length != 0) {
		ctrlcmd_data = kzalloc(out->length, GFP_KERNEL);
		if (!ctrlcmd_data) {
			rc = -ENOMEM;
			goto ctrlcmd_alloc_fail;
		}
		memcpy(ctrlcmd_data, out->value, out->length);
		isp_event->isp_data.ctrl.value = ctrlcmd_data;
	}

	atomic_set(&event_qcmd->on_heap, 1);
	event_qcmd->command = isp_event;

	msm_enqueue(&server_dev->server_queue[out->queue_idx].eventData_q,
				&event_qcmd->list_eventdata);

	/* now send command to config thread in userspace,
	 * and wait for results */
	v4l2_event_queue(server_dev->server_command_queue.pvdev,
					  &v4l2_evt);
	D("%s v4l2_event_queue: type = 0x%x\n", __func__, v4l2_evt.type);
	mutex_unlock(&server_dev->server_queue_lock);

	/* wait for config return status */
	D("Waiting for config status\n");
#if defined(CONFIG_SONY_CAM_MAIN_V4L2) || defined(CONFIG_SONY_CAM_SUB_V4L2)
retry:
#endif
	rc = wait_event_interruptible_timeout(queue->wait,
		!list_empty_careful(&queue->list),
		msecs_to_jiffies(out->timeout_ms));
#if defined(CONFIG_SONY_CAM_MAIN_V4L2) || defined(CONFIG_SONY_CAM_SUB_V4L2)
	if (rc == -ERESTARTSYS && cnt < 10) {
		pr_err("ERESTARTSYS happen cnt=%d\n", cnt);
		msleep(20);
		cnt++;
		goto retry;
	}
#endif
	D("Waiting is over for config status\n");
	if (list_empty_careful(&queue->list)) {
		if (!rc)
			rc = -ETIMEDOUT;
		if (rc < 0) {
			if (++server_dev->server_evt_id == 0)
				server_dev->server_evt_id++;
			pr_err("%s: wait_event error %d\n", __func__, rc);
			msm_cam_stop_hardware(pcam);
			return rc;
		}
	}

	rcmd = msm_dequeue(queue, list_control);
	BUG_ON(!rcmd);
	D("%s Finished servicing ioctl\n", __func__);

	ctrlcmd = (struct msm_ctrl_cmd *)(rcmd->command);
	value = out->value;
	if (ctrlcmd->length > 0 && value != NULL &&
	    ctrlcmd->length <= out->length)
		memcpy(value, ctrlcmd->value, ctrlcmd->length);

	memcpy(out, ctrlcmd, sizeof(struct msm_ctrl_cmd));
	out->value = value;

	kfree(ctrlcmd);
	free_qcmd(rcmd);
	D("%s: rc %d\n", __func__, rc);
	/* rc is the time elapsed. */
	if (rc >= 0) {
		/* TODO: Refactor msm_ctrl_cmd::status field */
		if (out->status == 0)
			rc = -1;
		else if (out->status == 1 || out->status == 4)
			rc = 0;
		else
			rc = -EINVAL;
	}
	return rc;

	mutex_lock(&pcam->vid_lock);
	rc = msm_server_q_ctrl(pcam, pqctrl);
	mutex_unlock(&pcam->vid_lock);
	return rc;
}

static int msm_camera_v4l2_private_general(struct file *f, void *pctx,
	struct msm_camera_v4l2_ioctl_t *ioctl_ptr)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	WARN_ON(pctx != f->private_data);

	rc = msm_server_private_general(pcam, ioctl_ptr);
	if (rc < 0)
		pr_err("%s: Private command failed rc %d\n", __func__, rc);
	return rc;
}

static int msm_camera_v4l2_private_g_ctrl(struct file *f, void *pctx,
	struct msm_camera_v4l2_ioctl_t *ioctl_ptr)
{
	int rc = -EINVAL;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_lock);
	switch (ioctl_ptr->id) {
	case MSM_V4L2_PID_INST_HANDLE:
		COPY_TO_USER(rc, (void __user *)ioctl_ptr->ioctl_ptr,
			(void *)&pcam_inst->inst_handle, sizeof(uint32_t));
		if (rc)
			ERR_COPY_TO_USER();
		break;
	default:
		pr_err("%s Unsupported ioctl %d ", __func__, ioctl_ptr->id);
		break;
	}
	mutex_unlock(&pcam->vid_lock);
	return rc;
}

static int msm_camera_v4l2_g_ctrl(struct file *f, void *pctx,
					struct v4l2_control *c)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_lock);
	rc = msm_server_g_ctrl(pcam, c);
	mutex_unlock(&pcam->vid_lock);

	return rc;
}

static int msm_camera_v4l2_private_s_ctrl(struct file *f, void *pctx,
			struct msm_camera_v4l2_ioctl_t *ioctl_ptr)
{
	int rc = -EINVAL;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);
	WARN_ON(pctx != f->private_data);
	mutex_lock(&pcam->vid_lock);
	switch (ioctl_ptr->id) {
	case MSM_V4L2_PID_CTRL_CMD:
		rc = msm_server_proc_ctrl_cmd(pcam, ioctl_ptr, 1);
		break;
	}
	mutex_unlock(&pcam->vid_lock);
	return rc;
}

static int msm_camera_v4l2_s_ctrl(struct file *f, void *pctx,
					struct v4l2_control *ctrl)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);

	WARN_ON(pctx != f->private_data);
	mutex_lock(&pcam->vid_lock);
	switch (ctrl->id) {
	case MSM_V4L2_PID_MMAP_INST:
		D("%s: mmap_inst=(0x%p, %d)\n",
			 __func__, pcam_inst, pcam_inst->my_index);
		pcam_inst->is_mem_map_inst = 1;
		break;
	default:
		if (ctrl->id == MSM_V4L2_PID_CAM_MODE)
			pcam->op_mode = ctrl->value;
		rc = msm_server_s_ctrl(pcam, ctrl);
		break;
	}
	mutex_unlock(&pcam->vid_lock);

	return rc;
}

static int msm_camera_v4l2_reqbufs(struct file *f, void *pctx,
				struct v4l2_requestbuffers *pb)
{
	int rc = 0, i, j;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	struct msm_cam_media_controller *pmctl;
	struct msm_cam_v4l2_device *pcam = video_drvdata(f);
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);
	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam_inst->inst_lock);
	if (!pcam_inst->vbqueue_initialized && pb->count) {
		pmctl = msm_cam_server_get_mctl(pcam->mctl_handle);
		if (pmctl == NULL) {
			pr_err("%s Invalid mctl ptr", __func__);
			mutex_unlock(&pcam_inst->inst_lock);
			return -EINVAL;
		}
		pmctl->mctl_vbqueue_init(pcam_inst, &pcam_inst->vid_bufq,
			pb->type);
		pcam_inst->vbqueue_initialized = 1;
	}

	rc = vb2_reqbufs(&pcam_inst->vid_bufq, pb);
	if (rc < 0) {
		pr_err("%s reqbufs failed %d ", __func__, rc);
		mutex_unlock(&pcam_inst->inst_lock);
		return rc;
	}
	if (!pb->count) {
		/* Deallocation. free buf_offset array */
		D("%s Inst %p freeing buffer offsets array",
			__func__, pcam_inst);
		for (j = 0 ; j < pcam_inst->buf_count ; j++) {
			kfree(pcam_inst->buf_offset[j]);
			pcam_inst->buf_offset[j] = NULL;
		}
		kfree(pcam_inst->buf_offset);
		pcam_inst->buf_offset = NULL;
		/* If the userspace has deallocated all the
		 * buffers, then release the vb2 queue */
		if (pcam_inst->vbqueue_initialized) {
			vb2_queue_release(&pcam_inst->vid_bufq);
			pcam_inst->vbqueue_initialized = 0;
		}
	} else {
		D("%s Inst %p Allocating buf_offset array",
			__func__, pcam_inst);
		/* Allocation. allocate buf_offset array */
		pcam_inst->buf_offset = (struct msm_cam_buf_offset **)
			kzalloc(pb->count * sizeof(struct msm_cam_buf_offset *),
							GFP_KERNEL);
		if (!pcam_inst->buf_offset) {
			pr_err("%s out of memory ", __func__);
			mutex_unlock(&pcam_inst->inst_lock);
			return -ENOMEM;
		}
		for (i = 0; i < pb->count; i++) {
			pcam_inst->buf_offset[i] =
				kzalloc(sizeof(struct msm_cam_buf_offset) *
				pcam_inst->plane_info.num_planes, GFP_KERNEL);
			if (!pcam_inst->buf_offset[i]) {
				pr_err("%s out of memory ", __func__);
				for (j = i-1 ; j >= 0; j--) {
					kfree(pcam_inst->buf_offset[j]);
					pcam_inst->buf_offset[j] = NULL;
				}
				kfree(pcam_inst->buf_offset);
				pcam_inst->buf_offset = NULL;
				mutex_unlock(&pcam_inst->inst_lock);
				return -ENOMEM;
			}
		}
	}
	pcam_inst->buf_count = pb->count;
	mutex_unlock(&pcam_inst->inst_lock);
	return rc;
}

static int msm_camera_v4l2_querybuf(struct file *f, void *pctx,
					struct v4l2_buffer *pb)
{
	/* get the video device */
	int rc = 0;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);
	mutex_lock(&pcam_inst->inst_lock);
	rc = vb2_querybuf(&pcam_inst->vid_bufq, pb);
	mutex_unlock(&pcam_inst->inst_lock);
	return rc;
}

static int msm_camera_v4l2_qbuf(struct file *f, void *pctx,
					struct v4l2_buffer *pb)
{
	int rc = 0, i = 0;
	/* get the camera device */
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s Inst=%p, mode=%d, idx=%d\n", __func__, pcam_inst,
		pcam_inst->image_mode, pb->index);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam_inst->inst_lock);
	if (!pcam_inst->buf_offset) {
		pr_err("%s Buffer is already released. Returning.\n", __func__);
		mutex_unlock(&pcam_inst->inst_lock);
		return -EINVAL;
	}

	if (pb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* Reject the buffer if planes array was not allocated */
		if (pb->m.planes == NULL) {
			pr_err("%s Planes array is null\n", __func__);
			mutex_unlock(&pcam_inst->inst_lock);
			return -EINVAL;
		}
		for (i = 0; i < pcam_inst->plane_info.num_planes; i++) {
			D("%s stored offsets for plane %d as"
				"addr offset %d, data offset %d\n",
				__func__, i, pb->m.planes[i].reserved[0],
				pb->m.planes[i].data_offset);
			pcam_inst->buf_offset[pb->index][i].data_offset =
				pb->m.planes[i].data_offset;
			pcam_inst->buf_offset[pb->index][i].addr_offset =
				pb->m.planes[i].reserved[0];
		}
	} else {
		D("%s stored reserved info %d\n", __func__, pb->reserved);
		pcam_inst->buf_offset[pb->index][0].addr_offset = pb->reserved;
	}

	rc = vb2_qbuf(&pcam_inst->vid_bufq, pb);
	D("%s, videobuf_qbuf mode %d and idx %d returns %d\n", __func__,
		pcam_inst->image_mode, pb->index, rc);
	mutex_unlock(&pcam_inst->inst_lock);
	return rc;
}

static int msm_camera_v4l2_dqbuf(struct file *f, void *pctx,
					struct v4l2_buffer *pb)
{
	int rc = 0, i = 0;
	/* get the camera device */
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam_inst->inst_lock);
	if (0 == pcam_inst->streamon) {
		mutex_unlock(&pcam_inst->inst_lock);
		return -EACCES;
	}
	rc = vb2_dqbuf(&pcam_inst->vid_bufq, pb,  f->f_flags & O_NONBLOCK);
	if (rc < 0) {
		pr_err("%s, videobuf_dqbuf returns %d\n", __func__, rc);
		mutex_unlock(&pcam_inst->inst_lock);
		return rc;
	}

	if (pb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* Reject the buffer if planes array was not allocated */
		if (pb->m.planes == NULL) {
			pr_err("%s Planes array is null\n", __func__);
			mutex_unlock(&pcam_inst->inst_lock);
			return -EINVAL;
		}
		for (i = 0; i < pcam_inst->plane_info.num_planes; i++) {
			pb->m.planes[i].data_offset =
				pcam_inst->buf_offset[pb->index][i].data_offset;
			pb->m.planes[i].reserved[0] =
				pcam_inst->buf_offset[pb->index][i].addr_offset;
			D("%s stored offsets for plane %d as "
				"addr offset %d, data offset %d\n",
				__func__, i, pb->m.planes[i].reserved[0],
				pb->m.planes[i].data_offset);
		}
	} else {
		D("%s stored reserved info %d\n", __func__, pb->reserved);
		pb->reserved = pcam_inst->buf_offset[pb->index][0].addr_offset;
	}

	mutex_unlock(&pcam_inst->inst_lock);
	return rc;
}

static int msm_camera_v4l2_streamon(struct file *f, void *pctx,
					enum v4l2_buf_type buf_type)
{
	int rc = 0;
	/* get the camera device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s Inst %p\n", __func__, pcam_inst);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_lock);
	mutex_lock(&pcam_inst->inst_lock);
	if ((buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) &&
		(buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
		pr_err("%s Invalid buffer type ", __func__);
		mutex_unlock(&pcam_inst->inst_lock);
		mutex_unlock(&pcam->vid_lock);
		return -EINVAL;
	}

	D("%s Calling videobuf_streamon", __func__);
	/* if HW streaming on is successful, start buffer streaming */
	rc = vb2_streamon(&pcam_inst->vid_bufq, buf_type);
	D("%s, videobuf_streamon returns %d\n", __func__, rc);

	/* turn HW (VFE/sensor) streaming */
	pcam_inst->streamon = 1;
	rc = msm_server_streamon(pcam, pcam_inst->my_index);
	mutex_unlock(&pcam_inst->inst_lock);
	mutex_unlock(&pcam->vid_lock);
	D("%s rc = %d\n", __func__, rc);
	return rc;
}

static int msm_camera_v4l2_streamoff(struct file *f, void *pctx,
					enum v4l2_buf_type buf_type)
{
	int rc = 0;
	/* get the camera device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s Inst %p\n", __func__, pcam_inst);
	WARN_ON(pctx != f->private_data);

	if ((buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) &&
		(buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
		pr_err("%s Invalid buffer type ", __func__);
		return -EINVAL;
	}

	/* first turn of HW (VFE/sensor) streaming so that buffers are
		not in use when we free the buffers */
	mutex_lock(&pcam->vid_lock);
	mutex_lock(&pcam_inst->inst_lock);
	pcam_inst->streamon = 0;
	if (msm_server_get_usecount() > 0)
		rc = msm_server_streamoff(pcam, pcam_inst->my_index);

	if (rc < 0)
		pr_err("%s: hw failed to stop streaming\n", __func__);

	/* stop buffer streaming */
	vb2_streamoff(&pcam_inst->vid_bufq, buf_type);
	D("%s, videobuf_streamoff returns %d\n", __func__, rc);

	mutex_unlock(&pcam_inst->inst_lock);
	mutex_unlock(&pcam->vid_lock);
	return rc;
}

static int msm_camera_v4l2_enum_fmt_cap(struct file *f, void *pctx,
					struct v4l2_fmtdesc *pfmtdesc)
{
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	const struct msm_isp_color_fmt *isp_fmt;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);
	if ((pfmtdesc->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) &&
		(pfmtdesc->type != V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return -EINVAL;

	if (pfmtdesc->index >= pcam->num_fmts)
		return -EINVAL;

	isp_fmt = &pcam->usr_fmts[pfmtdesc->index];

	if (isp_fmt->name)
		strlcpy(pfmtdesc->description, isp_fmt->name,
						sizeof(pfmtdesc->description));

	pfmtdesc->pixelformat = isp_fmt->fourcc;

	D("%s: [%d] 0x%x, %s\n", __func__, pfmtdesc->index,
		isp_fmt->fourcc, isp_fmt->name);
	return 0;
}

static int msm_camera_v4l2_g_fmt_cap(struct file *f,
		void *pctx, struct v4l2_format *pfmt)
{
	int rc = 0;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	if (pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	rc = msm_server_get_fmt(pcam, pcam_inst->my_index, pfmt);
	D("%s: current_fmt->fourcc: 0x%08x, rc = %d\n", __func__,
				pfmt->fmt.pix.pixelformat, rc);
	return rc;
}

static int msm_camera_v4l2_g_fmt_cap_mplane(struct file *f,
		void *pctx, struct v4l2_format *pfmt)
{
	int rc = 0;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	if (pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	rc = msm_server_get_fmt_mplane(pcam, pcam_inst->my_index, pfmt);
	D("%s: current_fmt->fourcc: 0x%08x, rc = %d\n", __func__,
					pfmt->fmt.pix_mp.pixelformat, rc);
	return rc;
}

/* This function will readjust the format parameters based in HW
  capabilities. Called by s_fmt_cap
*/
static int msm_camera_v4l2_try_fmt_cap(struct file *f, void *pctx,
					struct v4l2_format *pfmt)
{
	int rc = 0;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_lock);
	rc = msm_server_try_fmt(pcam, pfmt);
	if (rc)
		pr_err("Format %x not found, rc = %d\n",
				pfmt->fmt.pix.pixelformat, rc);

	mutex_unlock(&pcam->vid_lock);
	return rc;
}

static int msm_camera_v4l2_try_fmt_cap_mplane(struct file *f, void *pctx,
					struct v4l2_format *pfmt)
{
	int rc = 0;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_lock);
	rc = msm_server_try_fmt_mplane(pcam, pfmt);
	if (rc)
		pr_err("Format %x not found, rc = %d\n",
				pfmt->fmt.pix_mp.pixelformat, rc);

	mutex_unlock(&pcam->vid_lock);
	return rc;
}

/* This function will reconfig the v4l2 driver and HW device, it should be
   called after the streaming is stopped.
*/
static int msm_camera_v4l2_s_fmt_cap(struct file *f, void *pctx,
					struct v4l2_format *pfmt)
{
	int rc;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	D("%s, inst=0x%x,idx=%d,priv = 0x%p\n",
		__func__, (u32)pcam_inst, pcam_inst->my_index,
		(void *)pfmt->fmt.pix.priv);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_lock);

	rc = msm_server_set_fmt(pcam, pcam_inst->my_index, pfmt);
	if (rc < 0) {
		pr_err("%s: msm_server_set_fmt Error: %d\n",
				__func__, rc);
	}
	mutex_unlock(&pcam->vid_lock);

	return rc;
}

static int msm_camera_v4l2_s_fmt_cap_mplane(struct file *f, void *pctx,
				struct v4l2_format *pfmt)
{
	int rc;
	struct msm_cam_v4l2_device *pcam = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
			struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s Inst %p\n", __func__, pcam_inst);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_lock);
	rc = msm_server_set_fmt_mplane(pcam, pcam_inst->my_index, pfmt);
	mutex_unlock(&pcam->vid_lock);

	return rc;
}
static int msm_camera_v4l2_g_jpegcomp(struct file *f, void *pctx,
				struct v4l2_jpegcompression *pcomp)
{
	int rc = -EINVAL;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}

static int msm_camera_v4l2_s_jpegcomp(struct file *f, void *pctx,
				struct v4l2_jpegcompression *pcomp)
{
	int rc = -EINVAL;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}


static int msm_camera_v4l2_g_crop(struct file *f, void *pctx,
					struct v4l2_crop *crop)
{
	int rc = -EINVAL;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;

	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_lock);
	rc = msm_server_get_crop(pcam, pcam_inst->my_index, crop);
	mutex_unlock(&pcam->vid_lock);
	return rc;
}

static int msm_camera_v4l2_s_crop(struct file *f, void *pctx,
					struct v4l2_crop *a)
{
	int rc = -EINVAL;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}

/* Stream type-dependent parameter ioctls */
static int msm_camera_v4l2_g_parm(struct file *f, void *pctx,
				struct v4l2_streamparm *a)
{
	int rc = -EINVAL;
	return rc;
}
static int msm_vidbuf_get_path(u32 extendedmode)
{
	switch (extendedmode) {
	case MSM_V4L2_EXT_CAPTURE_MODE_THUMBNAIL:
		return OUTPUT_TYPE_T;
	case MSM_V4L2_EXT_CAPTURE_MODE_MAIN:
		return OUTPUT_TYPE_S;
	case MSM_V4L2_EXT_CAPTURE_MODE_VIDEO:
		return OUTPUT_TYPE_V;
	case MSM_V4L2_EXT_CAPTURE_MODE_RDI:
		return OUTPUT_TYPE_R;
	case MSM_V4L2_EXT_CAPTURE_MODE_RDI1:
		return OUTPUT_TYPE_R1;
	case MSM_V4L2_EXT_CAPTURE_MODE_RDI2:
		return OUTPUT_TYPE_R2;
	case MSM_V4L2_EXT_CAPTURE_MODE_AEC:
		return OUTPUT_TYPE_SAEC;
	case MSM_V4L2_EXT_CAPTURE_MODE_AF:
		return OUTPUT_TYPE_SAFC;
	case MSM_V4L2_EXT_CAPTURE_MODE_AWB:
		return OUTPUT_TYPE_SAWB;
	case MSM_V4L2_EXT_CAPTURE_MODE_IHIST:
		return OUTPUT_TYPE_IHST;
	case MSM_V4L2_EXT_CAPTURE_MODE_CSTA:
		return OUTPUT_TYPE_CSTA;
	case MSM_V4L2_EXT_CAPTURE_MODE_DEFAULT:
	case MSM_V4L2_EXT_CAPTURE_MODE_PREVIEW:
	default:
		return OUTPUT_TYPE_P;
	}
}

static int msm_camera_v4l2_s_parm(struct file *f, void *pctx,
				struct v4l2_streamparm *a)
{
	int rc = 0;
	int is_bayer_sensor = 0;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);
	pcam_inst->image_mode = (a->parm.capture.extendedmode & 0x7F);
	SET_DEVID_MODE(pcam_inst->inst_handle, pcam_inst->pcam->vnode_id);
	SET_IMG_MODE(pcam_inst->inst_handle, pcam_inst->image_mode);
	SET_VIDEO_INST_IDX(pcam_inst->inst_handle, pcam_inst->my_index);
	pcam_inst->pcam->dev_inst_map[pcam_inst->image_mode] = pcam_inst;
	pcam_inst->path = msm_vidbuf_get_path(pcam_inst->image_mode);
	if (pcam_inst->pcam->sdata->sensor_type == BAYER_SENSOR)
		is_bayer_sensor = 1;
	rc = msm_cam_server_config_interface_map(pcam_inst->image_mode,
			pcam_inst->pcam->mctl_handle, pcam_inst->pcam->vnode_id,
			is_bayer_sensor);
	D("%s path=%d, rc=%d\n", __func__,
		pcam_inst->path, rc);
	return rc;
}

static int msm_camera_v4l2_subscribe_event(struct v4l2_fh *fh,
			struct v4l2_event_subscription *sub)
{
	int rc = 0;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst =
		(struct msm_cam_v4l2_dev_inst *)container_of(fh,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s:fh = 0x%x, type = 0x%x\n", __func__, (u32)fh, sub->type);
	if (pcam_inst->my_index != 0)
		return -EINVAL;
	if (sub->type == V4L2_EVENT_ALL)
		sub->type = V4L2_EVENT_PRIVATE_START+MSM_CAM_APP_NOTIFY_EVENT;
	rc = v4l2_event_subscribe(fh, sub, 100);
	if (rc < 0)
		D("%s: failed for evtType = 0x%x, rc = %d\n",
						__func__, sub->type, rc);
	return rc;
}

static int msm_camera_v4l2_unsubscribe_event(struct v4l2_fh *fh,
			struct v4l2_event_subscription *sub)
{
	int rc = 0;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst =
		(struct msm_cam_v4l2_dev_inst *)container_of(fh,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s: fh = 0x%x\n", __func__, (u32)fh);
	if (pcam_inst->my_index != 0)
		return -EINVAL;

	rc = v4l2_event_unsubscribe(fh, sub);
	D("%s: rc = %d\n", __func__, rc);
	return rc;
}

static long msm_camera_v4l2_private_ioctl(struct file *file, void *fh,
					  bool valid_prio, int cmd,
					  void *arg)
{
	int rc = -EINVAL;
	struct msm_camera_v4l2_ioctl_t *ioctl_ptr = arg;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(file);
	D("%s: cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case MSM_CAM_V4L2_IOCTL_PRIVATE_S_CTRL:
		rc = msm_camera_v4l2_private_s_ctrl(file, fh, ioctl_ptr);
		break;
	case MSM_CAM_V4L2_IOCTL_PRIVATE_G_CTRL:
		rc = msm_camera_v4l2_private_g_ctrl(file, fh, ioctl_ptr);
		break;
	case MSM_CAM_V4L2_IOCTL_PRIVATE_GENERAL:
		rc = msm_camera_v4l2_private_general(file, fh, ioctl_ptr);
		break;
	case MSM_CAM_V4L2_IOCTL_GET_EVENT_PAYLOAD: {
		struct msm_queue_cmd *event_cmd;
		void *payload;
		mutex_lock(&pcam->event_lock);
		event_cmd = msm_dequeue(&pcam->eventData_q, list_eventdata);
		if (!event_cmd) {
			pr_err("%s: No event payload\n", __func__);
			rc = -EINVAL;
			mutex_unlock(&pcam->event_lock);
			return rc;
		}
		payload = event_cmd->command;
		if (event_cmd->trans_code != ioctl_ptr->trans_code) {
			pr_err("%s: Events don't match\n", __func__);
			kfree(payload);
			kfree(event_cmd);
			rc = -EINVAL;
			mutex_unlock(&pcam->event_lock);
			break;
		}
		if (ioctl_ptr->len > 0) {
			if (copy_to_user(ioctl_ptr->ioctl_ptr, payload,
				 ioctl_ptr->len)) {
				pr_err("%s Copy to user failed for cmd %d",
					__func__, cmd);
				kfree(payload);
				kfree(event_cmd);
				rc = -EINVAL;
				mutex_unlock(&pcam->event_lock);
				break;
			}
		}
		kfree(payload);
		kfree(event_cmd);
		mutex_unlock(&pcam->event_lock);
		rc = 0;
		break;
	}
	default:
		pr_err("%s Unsupported ioctl cmd %d ", __func__, cmd);
		break;
	}
	return rc;
}

/* v4l2_ioctl_ops */
static const struct v4l2_ioctl_ops g_msm_ioctl_ops = {
	.vidioc_querycap = msm_camera_v4l2_querycap,

	.vidioc_s_crop = msm_camera_v4l2_s_crop,
	.vidioc_g_crop = msm_camera_v4l2_g_crop,

	.vidioc_queryctrl = msm_camera_v4l2_queryctrl,
	.vidioc_g_ctrl = msm_camera_v4l2_g_ctrl,
	.vidioc_s_ctrl = msm_camera_v4l2_s_ctrl,

	.vidioc_reqbufs = msm_camera_v4l2_reqbufs,
	.vidioc_querybuf = msm_camera_v4l2_querybuf,
	.vidioc_qbuf = msm_camera_v4l2_qbuf,
	.vidioc_dqbuf = msm_camera_v4l2_dqbuf,

	.vidioc_streamon = msm_camera_v4l2_streamon,
	.vidioc_streamoff = msm_camera_v4l2_streamoff,

	/* format ioctls */
	.vidioc_enum_fmt_vid_cap = msm_camera_v4l2_enum_fmt_cap,
	.vidioc_enum_fmt_vid_cap_mplane = msm_camera_v4l2_enum_fmt_cap,
	.vidioc_try_fmt_vid_cap = msm_camera_v4l2_try_fmt_cap,
	.vidioc_try_fmt_vid_cap_mplane = msm_camera_v4l2_try_fmt_cap_mplane,
	.vidioc_g_fmt_vid_cap = msm_camera_v4l2_g_fmt_cap,
	.vidioc_g_fmt_vid_cap_mplane = msm_camera_v4l2_g_fmt_cap_mplane,
	.vidioc_s_fmt_vid_cap = msm_camera_v4l2_s_fmt_cap,
	.vidioc_s_fmt_vid_cap_mplane = msm_camera_v4l2_s_fmt_cap_mplane,

	.vidioc_g_jpegcomp = msm_camera_v4l2_g_jpegcomp,
	.vidioc_s_jpegcomp = msm_camera_v4l2_s_jpegcomp,

	/* Stream type-dependent parameter ioctls */
	.vidioc_g_parm =  msm_camera_v4l2_g_parm,
	.vidioc_s_parm =  msm_camera_v4l2_s_parm,

	/* event subscribe/unsubscribe */
	.vidioc_subscribe_event = msm_camera_v4l2_subscribe_event,
	.vidioc_unsubscribe_event = msm_camera_v4l2_unsubscribe_event,
	.vidioc_default = msm_camera_v4l2_private_ioctl,
};

/* v4l2_file_operations */
static int msm_open(struct file *f)
{
	int i, rc = -EINVAL;
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	int ion_client_created = 0;
#endif
	int server_q_idx = 0;
	/* get the video device */
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	struct msm_cam_media_controller *pmctl = NULL;

	D("%s\n", __func__);

	if (!pcam) {
		pr_err("%s NULL pointer passed in!\n", __func__);
		return rc;
	}
	if (!msm_server_get_usecount()) {
		pr_err("%s: error, daemon not yet started.", __func__);
		return -EINVAL;
	}
	mutex_lock(&pcam->vid_lock);
	for (i = 0; i < MSM_DEV_INST_MAX; i++) {
		if (pcam->dev_inst[i] == NULL)
			break;
	}

	/* if no instance is available, return error */
	if (i == MSM_DEV_INST_MAX) {
		mutex_unlock(&pcam->vid_lock);
		return rc;
	}
	pcam_inst = kzalloc(sizeof(struct msm_cam_v4l2_dev_inst), GFP_KERNEL);
	if (!pcam_inst) {
		mutex_unlock(&pcam->vid_lock);
		return rc;
	}
	mutex_init(&pcam_inst->inst_lock);
	pcam_inst->sensor_pxlcode = pcam->usr_fmts[0].pxlcode;
	pcam_inst->my_index = i;
	pcam_inst->pcam = pcam;
	pcam->dev_inst[i] = pcam_inst;

	D("%s index %d nodeid %d count %d\n", __func__,
			pcam_inst->my_index,
			pcam->vnode_id, pcam->use_count);
	pcam->use_count++;
	D("%s Inst %p use_count %d\n", __func__, pcam_inst, pcam->use_count);
	if (pcam->use_count == 1) {
		server_q_idx = msm_find_free_queue();
		if (server_q_idx < 0) {
			pr_err("%s No free queue available ", __func__);
			goto msm_cam_server_begin_session_failed;
		}
		rc = msm_server_begin_session(pcam, server_q_idx);
		if (rc < 0) {
			pr_err("%s error starting server session ", __func__);
			goto msm_cam_server_begin_session_failed;
		}
		pmctl = msm_cam_server_get_mctl(pcam->mctl_handle);
		if (!pmctl) {
			pr_err("%s mctl ptr is null ", __func__);
			goto msm_cam_server_get_mctl_failed;
		}
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		if (!pmctl->client) {
			pmctl->client = msm_ion_client_create(-1, "camera");
			kref_init(&pmctl->refcount);
		}
		ion_client_created = 1;
#endif

		/* Should be set to sensor ops if any but right now its OK!! */
		if (!pmctl->mctl_open) {
			D("%s: media contoller is not inited\n", __func__);
			rc = -ENODEV;
			goto mctl_open_failed;
		}

		/* Now we really have to activate the camera */
		D("%s: call mctl_open\n", __func__);
		rc = pmctl->mctl_open(pmctl, MSM_APPS_ID_V4L2);
		if (rc < 0) {
			pr_err("%s: HW open failed rc = 0x%x\n",  __func__, rc);
			goto mctl_open_failed;
		}
		pmctl->pcam_ptr = pcam;

		msm_setup_v4l2_event_queue(&pcam_inst->eventHandle,
			pcam->pvdev);
		mutex_init(&pcam->event_lock);
		msm_queue_init(&pcam->eventData_q, "eventData");
	}
	pcam_inst->vbqueue_initialized = 0;
	pcam_inst->sequence = 0;
	pcam_inst->avtimerOn = 0;
	rc = 0;

	f->private_data = &pcam_inst->eventHandle;

	D("f->private_data = 0x%x, pcam = 0x%x\n",
		(u32)f->private_data, (u32)pcam_inst);

	if (pcam->use_count == 1) {
		rc = msm_send_open_server(pcam);
		if (rc < 0 && rc != -ERESTARTSYS) {
			pr_err("%s: msm_send_open_server failed %d\n",
				__func__, rc);
			goto msm_send_open_server_failed;
		}
	}
	mutex_unlock(&pcam->vid_lock);
	D("%s: end\n", __func__);
	return rc;

msm_send_open_server_failed:
	msm_drain_eventq(&pcam->eventData_q);
	msm_destroy_v4l2_event_queue(&pcam_inst->eventHandle);

	if (pmctl->mctl_release) {
		pmctl->mctl_release(pmctl);
		pmctl->mctl_release = NULL;
	}
mctl_open_failed:
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	if (ion_client_created) {
		D("%s: destroy ion client", __func__);
		kref_put(&pmctl->refcount, msm_release_ion_client);
	}
#endif
msm_cam_server_get_mctl_failed:
	if (msm_server_end_session(pcam) < 0)
		pr_err("%s: msm_server_end_session failed\n",
			__func__);
msm_cam_server_begin_session_failed:
	if (pcam->use_count == 1) {
		pcam->dev_inst[i] = NULL;
		pcam->use_count = 0;
	}
	pcam->dev_inst[i] = NULL;
	mutex_unlock(&pcam->vid_lock);
	mutex_destroy(&pcam_inst->inst_lock);
	kfree(pcam_inst);
	pr_err("%s: error end", __func__);
	return rc;
}

static int msm_addr_remap(struct msm_cam_v4l2_dev_inst *pcam_inst,
				struct vm_area_struct *vma)
{
	int phyaddr;
	int retval;
	unsigned long size;
	int rc = 0;
	struct msm_cam_media_controller *mctl;

	mctl = msm_cam_server_get_mctl(pcam_inst->pcam->mctl_handle);
	if (!mctl) {
		pr_err("%s: invalid mctl pointer", __func__);
		return -EFAULT;
	}

	rc = msm_pmem_region_get_phy_addr(&mctl->stats_info.pmem_stats_list,
			&pcam_inst->mem_map,
			&phyaddr);
	if (rc) {
		pr_err("%s: cannot map vaddr", __func__);
		return -EFAULT;
	}
	size = vma->vm_end - vma->vm_start;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	retval = remap_pfn_range(vma, vma->vm_start,
			phyaddr >> PAGE_SHIFT,
			size, vma->vm_page_prot);
	if (retval) {
		pr_err("%s:mmap: remap failed with error %d. ",
			   __func__, retval);
		memset(&pcam_inst->mem_map, 0, sizeof(pcam_inst->mem_map));
		return -ENOMEM;
	}
	D("%s:mmap: phy_addr=0x%x: %08lx-%08lx, pgoff %08lx\n",
		   __func__, (uint32_t)phyaddr,
		   vma->vm_start, vma->vm_end, vma->vm_pgoff);
	memset(&pcam_inst->mem_map, 0, sizeof(pcam_inst->mem_map));
	return 0;
}

static int msm_mmap(struct file *f, struct vm_area_struct *vma)
{
	int rc = 0;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("mmap called, vma=0x%08lx\n", (unsigned long)vma);

	if (pcam_inst->is_mem_map_inst &&
		pcam_inst->mem_map.cookie) {
		rc = msm_addr_remap(pcam_inst, vma);
		D("%s: msm_addr_remap ret=%d\n", __func__, rc);
		return rc;
	} else
		rc = vb2_mmap(&pcam_inst->vid_bufq, vma);
	D("vma start=0x%08lx, size=%ld, ret=%d\n",
		(unsigned long)vma->vm_start,
		(unsigned long)vma->vm_end - (unsigned long)vma->vm_start,
		rc);

	return rc;
}

void msm_release_ion_client(struct kref *ref)
{
	struct msm_cam_media_controller *mctl = container_of(ref,
		struct msm_cam_media_controller, refcount);
	pr_err("%s Calling ion_client_destroy\n", __func__);
	ion_client_destroy(mctl->client);
}

static int msm_close(struct file *f)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	struct msm_cam_server_queue *queue;
	struct msm_cam_media_controller *pmctl;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);
	pcam = pcam_inst->pcam;
	if (!pcam) {
		pr_err("%s NULL pointer of camera device!\n", __func__);
		return -EINVAL;
	}

	pmctl = msm_camera_get_mctl(pcam->mctl_handle);
	if (!pmctl) {
		pr_err("%s NULL mctl pointer\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&pcam->vid_lock);
	mutex_lock(&pcam_inst->inst_lock);

	if (pcam_inst->streamon) {
		/*something went wrong since instance
		is closing without streamoff*/
		msm_cam_stop_hardware(pcam);
	}

	pcam_inst->streamon = 0;
	pcam->use_count--;
	pcam->dev_inst_map[pcam_inst->image_mode] = NULL;
	if (pcam_inst->vbqueue_initialized)
		vb2_queue_release(&pcam_inst->vid_bufq);
	D("%s Closing down instance %p ", __func__, pcam_inst);
	D("%s index %d nodeid %d count %d\n", __func__, pcam_inst->my_index,
		pcam->vnode_id, pcam->use_count);
	pcam->dev_inst[pcam_inst->my_index] = NULL;
	if (pcam_inst->my_index == 0) {
		v4l2_fh_del(&pcam_inst->eventHandle);
		v4l2_fh_exit(&pcam_inst->eventHandle);
	}
	mutex_unlock(&pcam_inst->inst_lock);
	mutex_destroy(&pcam_inst->inst_lock);
	kfree(pcam_inst);
	f->private_data = NULL;

	if (pcam->use_count == 0) {
		int ges_evt = MSM_V4L2_GES_CAM_CLOSE;
		if (g_server_dev.use_count > 0) {
			rc = msm_send_close_server(pcam);
			if (rc < 0)
				pr_err("msm_send_close_server failed %d\n", rc);
		}

		if (pmctl->mctl_release) {
			rc = pmctl->mctl_release(pmctl);
			if (rc < 0)
				pr_err("mctl_release fails %d\n", rc);
		}

#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		kref_put(&pmctl->refcount, msm_release_ion_client);
#endif
		mutex_lock(&g_server_dev.server_queue_lock);
		queue = &g_server_dev.server_queue[pcam->server_queue_idx];
		queue->queue_active = 0;
		kfree(queue->ctrl_data);
		queue->ctrl_data = NULL;
		msm_queue_drain(&queue->ctrl_q, list_control);
		msm_drain_eventq(&queue->eventData_q);
		mutex_unlock(&g_server_dev.server_queue_lock);
		rc = msm_cam_server_close_session(&g_server_dev, pcam);
		if (rc < 0)
			pr_err("msm_cam_server_close_session fails %d\n", rc);

		msm_cam_server_subdev_notify(g_server_dev.gesture_device,
			NOTIFY_GESTURE_CAM_EVT, &ges_evt);
	}
	mutex_unlock(&pcam->vid_lock);
	return rc;
}

static unsigned int msm_poll(struct file *f, struct poll_table_struct *wait)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);
	pcam = pcam_inst->pcam;
	D("%s\n", __func__);
	if (!pcam) {
		pr_err("%s NULL pointer of camera device!\n", __func__);
		return -EINVAL;
	}
	if (pcam_inst->my_index == 0) {
		poll_wait(f, &(pcam_inst->eventHandle.wait), wait);
		if (v4l2_event_pending(&pcam_inst->eventHandle))
			rc |= POLLPRI;
	} else {
		if (!pcam_inst->vid_bufq.streaming) {
			D("%s vid_bufq.streaming is off, inst=0x%x\n",
			__func__, (u32)pcam_inst);
			return -EINVAL;
		}
		rc |= vb2_poll(&pcam_inst->vid_bufq, f, wait);
	}
	D("%s returns, rc  = 0x%x\n", __func__, rc);
	return rc;
}

static unsigned int msm_poll_server(struct file *fp,
					struct poll_table_struct *wait)
{
	int rc = 0;

	D("%s\n", __func__);
	poll_wait(fp,
		 &g_server_dev.server_command_queue.eventHandle.wait,
		 wait);
	if (v4l2_event_pending(&g_server_dev.server_command_queue.eventHandle))
		rc |= POLLPRI;

	return rc;
}
static long msm_ioctl_server(struct file *file, void *fh,
		bool valid_prio, int cmd, void *arg)
{
	int rc = -EINVAL;
	struct msm_camera_v4l2_ioctl_t *ioctl_ptr = arg;
	struct msm_camera_info temp_cam_info;
	struct msm_cam_config_dev_info temp_config_info;
	struct msm_mctl_node_info temp_mctl_info;
	int i;

	D("%s: cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case MSM_CAM_V4L2_IOCTL_GET_CAMERA_INFO:
		if (copy_from_user(&temp_cam_info,
			(void __user *)ioctl_ptr->ioctl_ptr,
			sizeof(struct msm_camera_info))) {
			rc = -EINVAL;
			return rc;
		}
		for (i = 0; i < g_server_dev.camera_info.num_cameras; i++) {
			if (copy_to_user((void __user *)
				temp_cam_info.video_dev_name[i],
				g_server_dev.camera_info.video_dev_name[i],
				strnlen(
				g_server_dev.camera_info.video_dev_name[i],
				MAX_DEV_NAME_LEN))) {
				rc = -EINVAL;
				return rc;
			}
			temp_cam_info.has_3d_support[i] =
				g_server_dev.camera_info.has_3d_support[i];
			temp_cam_info.is_internal_cam[i] =
				g_server_dev.camera_info.is_internal_cam[i];
			temp_cam_info.s_mount_angle[i] =
				g_server_dev.camera_info.s_mount_angle[i];
			temp_cam_info.sensor_type[i] =
				g_server_dev.camera_info.sensor_type[i];

		}
		temp_cam_info.num_cameras =
			g_server_dev.camera_info.num_cameras;
		if (copy_to_user((void __user *)ioctl_ptr->ioctl_ptr,
				&temp_cam_info,
				sizeof(struct msm_camera_info))) {
					rc = -EINVAL;
					return rc;
		}
		rc = 0;
		break;

	case MSM_CAM_V4L2_IOCTL_GET_CONFIG_INFO:
		if (copy_from_user(&temp_config_info,
				(void __user *)ioctl_ptr->ioctl_ptr,
				sizeof(struct msm_cam_config_dev_info))) {

			rc = -EINVAL;
			return rc;
		}
		for (i = 0;
		 i < g_server_dev.config_info.num_config_nodes; i++) {
			if (copy_to_user(
			(void __user *)temp_config_info.config_dev_name[i],
			g_server_dev.config_info.config_dev_name[i],
			strnlen(g_server_dev.config_info.config_dev_name[i],
			MAX_DEV_NAME_LEN))) {
				rc = -EINVAL;
				return rc;
			}
		}
		temp_config_info.num_config_nodes =
			g_server_dev.config_info.num_config_nodes;
		if (copy_to_user((void __user *)ioctl_ptr->ioctl_ptr,
							  &temp_config_info,
				sizeof(struct msm_cam_config_dev_info))) {
			rc = -EINVAL;
			return rc;
		}
		rc = 0;
		break;
	case MSM_CAM_V4L2_IOCTL_GET_MCTL_INFO:
		if (copy_from_user(&temp_mctl_info,
				(void __user *)ioctl_ptr->ioctl_ptr,
				sizeof(struct msm_mctl_node_info))) {
			rc = -EINVAL;
			return rc;
		}
		for (i = 0; i < g_server_dev.mctl_node_info.num_mctl_nodes;
				i++) {
			if (copy_to_user((void __user *)
			temp_mctl_info.mctl_node_name[i],
			g_server_dev.mctl_node_info.mctl_node_name[i], strnlen(
			g_server_dev.mctl_node_info.mctl_node_name[i],
			MAX_DEV_NAME_LEN))) {
				rc = -EINVAL;
				return rc;
			}
		}
		temp_mctl_info.num_mctl_nodes =
			g_server_dev.mctl_node_info.num_mctl_nodes;
		if (copy_to_user((void __user *)ioctl_ptr->ioctl_ptr,
							  &temp_mctl_info,
				sizeof(struct msm_mctl_node_info))) {
			rc = -EINVAL;
			return rc;
		}
		rc = 0;
	break;

	case MSM_CAM_V4L2_IOCTL_CTRL_CMD_DONE:
		D("%s: MSM_CAM_IOCTL_CTRL_CMD_DONE\n", __func__);
		rc = msm_ctrl_cmd_done(arg);
		break;

	case MSM_CAM_V4L2_IOCTL_GET_EVENT_PAYLOAD: {
		struct msm_queue_cmd *event_cmd;
		struct msm_isp_event_ctrl u_isp_event;
		struct msm_isp_event_ctrl *k_isp_event;
		struct msm_device_queue *queue;
		void __user *u_ctrl_value = NULL;
		if (copy_from_user(&u_isp_event,
			(void __user *)ioctl_ptr->ioctl_ptr,
			sizeof(struct msm_isp_event_ctrl))) {
			rc = -EINVAL;
			return rc;
		}
		mutex_lock(&g_server_dev.server_queue_lock);
		if (!g_server_dev.server_queue
			[u_isp_event.isp_data.ctrl.queue_idx].queue_active) {
			pr_err("%s: Invalid queue\n", __func__);
			mutex_unlock(&g_server_dev.server_queue_lock);
			rc = -EINVAL;
			return rc;
		}
		queue = &g_server_dev.server_queue
			[u_isp_event.isp_data.ctrl.queue_idx].eventData_q;
		event_cmd = msm_dequeue(queue, list_eventdata);
		if (!event_cmd) {
			pr_err("%s: No event payload\n", __func__);
			rc = -EINVAL;
			mutex_unlock(&g_server_dev.server_queue_lock);
			return rc;
		}
		k_isp_event = (struct msm_isp_event_ctrl *)
				event_cmd->command;
		free_qcmd(event_cmd);

		/* Save the pointer of the user allocated command buffer*/
		u_ctrl_value = u_isp_event.isp_data.ctrl.value;

		/* Copy the event structure into user struct*/
		u_isp_event = *k_isp_event;

		/* Restore the saved pointer of the user
		 * allocated command buffer. */
		u_isp_event.isp_data.ctrl.value = u_ctrl_value;

		/* Copy the ctrl cmd, if present*/
		if (k_isp_event->isp_data.ctrl.length > 0 &&
			k_isp_event->isp_data.ctrl.value != NULL) {
			void *k_ctrl_value =
				k_isp_event->isp_data.ctrl.value;
			if (copy_to_user(u_ctrl_value, k_ctrl_value,
				 k_isp_event->isp_data.ctrl.length)) {
				kfree(k_isp_event->isp_data.ctrl.value);
				kfree(k_isp_event);
				rc = -EINVAL;
				mutex_unlock(&g_server_dev.server_queue_lock);
				break;
			}
			kfree(k_isp_event->isp_data.ctrl.value);
		}
		if (copy_to_user((void __user *)ioctl_ptr->ioctl_ptr,
							  &u_isp_event,
				sizeof(struct msm_isp_event_ctrl))) {
			kfree(k_isp_event);
			rc = -EINVAL;
			mutex_unlock(&g_server_dev.server_queue_lock);
			return rc;
		}
		kfree(k_isp_event);
		mutex_unlock(&g_server_dev.server_queue_lock);
		rc = 0;
		break;
	}

	case MSM_CAM_IOCTL_SEND_EVENT:
		rc = msm_server_send_v4l2_evt(arg);
		break;

	default:
		pr_err("%s: Invalid IOCTL = %d", __func__, cmd);
		break;
	}
	return rc;
}

static int msm_open_server(struct file *fp)
{
	int rc = 0;
	D("%s: open %s\n", __func__, fp->f_path.dentry->d_name.name);
	mutex_lock(&g_server_dev.server_lock);
	g_server_dev.use_count++;
	if (g_server_dev.use_count == 1)
		fp->private_data =
			&g_server_dev.server_command_queue.eventHandle;
	mutex_unlock(&g_server_dev.server_lock);
	return rc;
}

static unsigned int msm_poll_config(struct file *fp,
					struct poll_table_struct *wait)
{
	int rc = 0;
	struct msm_cam_config_dev *config = fp->private_data;
	if (config == NULL)
		return -EINVAL;

	D("%s\n", __func__);

	poll_wait(fp,
	&config->config_stat_event_queue.eventHandle.wait, wait);
	if (v4l2_event_pending(&config->config_stat_event_queue.eventHandle))
		rc |= POLLPRI;
	return rc;
}

static int msm_close_server(struct file *fp)
{
	struct v4l2_event_subscription sub;
	D("%s\n", __func__);
	mutex_lock(&g_server_dev.server_lock);
	if (g_server_dev.use_count > 0)
		g_server_dev.use_count--;
	mutex_unlock(&g_server_dev.server_lock);
	if (g_server_dev.use_count == 0) {
		mutex_lock(&g_server_dev.server_lock);
		if (g_server_dev.pcam_active) {
			struct v4l2_event v4l2_ev;
			msm_cam_stop_hardware(g_server_dev.pcam_active);
			v4l2_ev.type = V4L2_EVENT_PRIVATE_START
				+ MSM_CAM_APP_NOTIFY_ERROR_EVENT;
			v4l2_ev.id = 0;
			ktime_get_ts(&v4l2_ev.timestamp);
			v4l2_event_queue(
				g_server_dev.pcam_active->pvdev, &v4l2_ev);
		}
		sub.type = V4L2_EVENT_ALL;
		msm_server_v4l2_unsubscribe_event(
			&g_server_dev.server_command_queue.eventHandle, &sub);
		mutex_unlock(&g_server_dev.server_lock);
	}
	return 0;
}

static long msm_server_send_v4l2_evt(void *evt)
{
	struct v4l2_event *v4l2_ev = (struct v4l2_event *)evt;
	int rc = 0;

	if (NULL == evt) {
		pr_err("%s: evt is NULL\n", __func__);
		return -EINVAL;
	}

	D("%s: evt type 0x%x\n", __func__, v4l2_ev->type);
	if ((v4l2_ev->type >= MSM_GES_APP_EVT_MIN) &&
		(v4l2_ev->type < MSM_GES_APP_EVT_MAX)) {
		msm_cam_server_subdev_notify(g_server_dev.gesture_device,
			NOTIFY_GESTURE_EVT, v4l2_ev);
	} else {
		pr_err("%s: Invalid evt %d\n", __func__, v4l2_ev->type);
		rc = -EINVAL;
	}
	D("%s: end\n", __func__);

	return rc;
}

static long msm_v4l2_evt_notify(struct msm_cam_media_controller *mctl,
		unsigned int cmd, unsigned long evt)
{
	struct v4l2_event v4l2_ev;
	struct msm_cam_v4l2_device *pcam = NULL;

	if (!mctl) {
		pr_err("%s: mctl is NULL\n", __func__);
		return -EINVAL;
	}

	if (copy_from_user(&v4l2_ev, (void __user *)evt,
		sizeof(struct v4l2_event))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	v4l2_ev.id = 0;
	pcam = mctl->pcam_ptr;
	ktime_get_ts(&v4l2_ev.timestamp);
	v4l2_event_queue(pcam->pvdev, &v4l2_ev);
	return 0;
}

static long msm_ioctl_config(struct file *fp, unsigned int cmd,
	unsigned long arg)
{

	int rc = 0;
	struct v4l2_event ev;
	struct msm_cam_config_dev *config_cam = fp->private_data;
	struct v4l2_event_subscription temp_sub;
	ev.id = 0;
	D("%s: cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	/* memory management shall be handeld here*/
	case MSM_CAM_IOCTL_REGISTER_PMEM:
		return msm_register_pmem(
			&config_cam->p_mctl->stats_info.pmem_stats_list,
			(void __user *)arg, config_cam->p_mctl->client);
		break;

	case MSM_CAM_IOCTL_UNREGISTER_PMEM:
		return msm_pmem_table_del(
			&config_cam->p_mctl->stats_info.pmem_stats_list,
			(void __user *)arg, config_cam->p_mctl->client);
		break;

	case VIDIOC_SUBSCRIBE_EVENT:
		if (copy_from_user(&temp_sub,
			(void __user *)arg,
			sizeof(struct v4l2_event_subscription))) {
			rc = -EINVAL;
			return rc;
		}
		rc = msm_server_v4l2_subscribe_event
			(&config_cam->config_stat_event_queue.eventHandle,
			&temp_sub);
		if (rc < 0) {
			pr_err("%s: cam_v4l2_subscribe_event failed rc=%d\n",
				__func__, rc);
			return rc;
		}
		break;

	case VIDIOC_UNSUBSCRIBE_EVENT:
		if (copy_from_user(&temp_sub, (void __user *)arg,
			sizeof(struct v4l2_event_subscription))) {
			rc = -EINVAL;
			return rc;
		}
		rc = msm_server_v4l2_unsubscribe_event
			(&config_cam->config_stat_event_queue.eventHandle,
			&temp_sub);
		if (rc < 0) {
			pr_err("%s: server_unsubscribe_event failed rc=%d\n",
				__func__, rc);
			return rc;
		}
		break;

	case VIDIOC_DQEVENT: {
		void __user *u_msg_value = NULL, *user_ptr = NULL;
		struct msm_isp_event_ctrl u_isp_event;
		struct msm_isp_event_ctrl *k_isp_event;

		/* First, copy the v4l2 event structure from userspace */
		D("%s: VIDIOC_DQEVENT\n", __func__);
		if (copy_from_user(&ev, (void __user *)arg,
				sizeof(struct v4l2_event)))
			break;
		/* Next, get the pointer to event_ctrl structure
		 * embedded inside the v4l2_event.u.data array. */
		user_ptr = (void __user *)(*((uint32_t *)ev.u.data));

		/* Next, copy the userspace event ctrl structure */
		if (copy_from_user((void *)&u_isp_event, user_ptr,
				   sizeof(struct msm_isp_event_ctrl))) {
			rc = -EFAULT;
			break;
		}
		/* Save the pointer of the user allocated command buffer*/
		u_msg_value = u_isp_event.isp_data.isp_msg.data;

		/* Dequeue the event queued into the v4l2 queue*/
		rc = v4l2_event_dequeue(
			&config_cam->config_stat_event_queue.eventHandle,
			&ev, fp->f_flags & O_NONBLOCK);
		if (rc < 0) {
			pr_err("no pending events?");
			rc = -EFAULT;
			break;
		}
		/* Use k_isp_event to point to the event_ctrl structure
		 * embedded inside v4l2_event.u.data */
		k_isp_event = (struct msm_isp_event_ctrl *)
				(*((uint32_t *)ev.u.data));
		/* Copy the event structure into user struct. */
		u_isp_event = *k_isp_event;
		if (ev.type != (V4L2_EVENT_PRIVATE_START +
				MSM_CAM_RESP_DIV_FRAME_EVT_MSG) &&
				ev.type != (V4L2_EVENT_PRIVATE_START +
				MSM_CAM_RESP_MCTL_PP_EVENT)) {

			/* Restore the saved pointer of the
			 * user allocated command buffer. */
			u_isp_event.isp_data.isp_msg.data = u_msg_value;

			if (ev.type == (V4L2_EVENT_PRIVATE_START +
					MSM_CAM_RESP_STAT_EVT_MSG)) {
				if (k_isp_event->isp_data.isp_msg.len > 0) {
					void *k_msg_value =
					k_isp_event->isp_data.isp_msg.data;
					if (copy_to_user(u_msg_value,
							k_msg_value,
					 k_isp_event->isp_data.isp_msg.len)) {
						rc = -EINVAL;
						break;
					}
					kfree(k_msg_value);
				}
			}
		}
		/* Copy the event ctrl structure back
		 * into user's structure. */
		if (copy_to_user(user_ptr,
				(void *)&u_isp_event, sizeof(
				struct msm_isp_event_ctrl))) {
			rc = -EINVAL;
			break;
		}
		kfree(k_isp_event);

		/* Copy the v4l2_event structure back to the user*/
		if (copy_to_user((void __user *)arg, &ev,
				sizeof(struct v4l2_event))) {
			rc = -EINVAL;
			break;
		}
		}

		break;

	case MSM_CAM_IOCTL_V4L2_EVT_NOTIFY:
		rc = msm_v4l2_evt_notify(config_cam->p_mctl, cmd, arg);
		break;

	default:{
		/* For the rest of config command, forward to media controller*/
		struct msm_cam_media_controller *p_mctl = config_cam->p_mctl;
		if (p_mctl && p_mctl->mctl_cmd) {
			rc = config_cam->p_mctl->mctl_cmd(p_mctl, cmd, arg);
		} else {
			rc = -EINVAL;
			pr_err("%s: media controller is null\n", __func__);
		}

		break;
	} /* end of default*/
	} /* end of switch*/
	return rc;
}

static int msm_open_config(struct inode *inode, struct file *fp)
{
	int rc;
	struct msm_cam_config_dev *config_cam = container_of(inode->i_cdev,
		struct msm_cam_config_dev, config_cdev);
	D("%s: open %s\n", __func__, fp->f_path.dentry->d_name.name);

	rc = nonseekable_open(inode, fp);
	if (rc < 0) {
		pr_err("%s: nonseekable_open error %d\n", __func__, rc);
		return rc;
	}
	config_cam->use_count++;

	/*config_cam->isp_subdev = g_server_dev.pcam_active->mctl.isp_sdev;*/
	/* assume there is only one active camera possible*/
	config_cam->p_mctl =
		msm_camera_get_mctl(g_server_dev.pcam_active->mctl_handle);

	INIT_HLIST_HEAD(&config_cam->p_mctl->stats_info.pmem_stats_list);
	spin_lock_init(&config_cam->p_mctl->stats_info.pmem_stats_spinlock);

	return rc;
}

void msm_release_ion_client(struct kref *ref)
{
	struct msm_cam_media_controller *mctl = container_of(ref,
		struct msm_cam_media_controller, refcount);
	pr_err("%s Calling ion_client_destroy\n", __func__);
	ion_client_destroy(mctl->client);
	mctl->client = NULL;
}

static struct v4l2_file_operations g_msm_fops = {
	.owner   = THIS_MODULE,
	.open	= msm_open,
	.poll	= msm_poll,
	.mmap	= msm_mmap,
	.release = msm_close,
	.ioctl   = video_ioctl2,
};

/* Init a config node for ISP control,
 * which will create a config device (/dev/config0/ and plug in
 * ISP's operation "v4l2_ioctl_ops*"
 */
static const struct v4l2_file_operations msm_fops_server = {
	.owner = THIS_MODULE,
	.open  = msm_open_server,
	.poll  = msm_poll_server,
	.unlocked_ioctl = video_ioctl2,
	.release = msm_close_server,
};

static const struct v4l2_ioctl_ops msm_ioctl_ops_server = {
	.vidioc_subscribe_event = msm_server_v4l2_subscribe_event,
	.vidioc_default = msm_ioctl_server,
};

static const struct file_operations msm_fops_config = {
	.owner = THIS_MODULE,
	.open  = msm_open_config,
	.poll  = msm_poll_config,
	.unlocked_ioctl = msm_ioctl_config,
	.release = msm_close_config,
};

int msm_setup_v4l2_event_queue(struct v4l2_fh *eventHandle,
	struct video_device *pvdev)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	struct msm_cam_media_controller *pmctl;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);
	pcam = pcam_inst->pcam;
	if (!pcam) {
		pr_err("%s NULL pointer of camera device!\n", __func__);
		return -EINVAL;
	}

	pmctl = msm_cam_server_get_mctl(pcam->mctl_handle);
	if (!pmctl) {
		pr_err("%s NULL mctl pointer\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&pcam->vid_lock);
	mutex_lock(&pcam_inst->inst_lock);

	if (pcam_inst->streamon) {
		/*something went wrong since instance
		is closing without streamoff*/
		if (pmctl->mctl_release)
			pmctl->mctl_release(pmctl);
		pmctl->mctl_release = NULL;/*so that it isn't closed again*/
	}

	pcam_inst->streamon = 0;
	pcam_inst->avtimerOn = 0;
	pcam->use_count--;
	pcam->dev_inst_map[pcam_inst->image_mode] = NULL;
	if (pcam_inst->vbqueue_initialized)
		vb2_queue_release(&pcam_inst->vid_bufq);
	D("%s Closing down instance %p ", __func__, pcam_inst);
	D("%s index %d nodeid %d count %d\n", __func__, pcam_inst->my_index,
		pcam->vnode_id, pcam->use_count);
	pcam->dev_inst[pcam_inst->my_index] = NULL;
	if (pcam_inst->my_index == 0) {
		mutex_lock(&pcam->event_lock);
		msm_drain_eventq(&pcam->eventData_q);
		mutex_unlock(&pcam->event_lock);
		mutex_destroy(&pcam->event_lock);
		msm_destroy_v4l2_event_queue(&pcam_inst->eventHandle);
	}

	CLR_VIDEO_INST_IDX(pcam_inst->inst_handle);
	CLR_IMG_MODE(pcam_inst->inst_handle);
	CLR_DEVID_MODE(pcam_inst->inst_handle);
	mutex_unlock(&pcam_inst->inst_lock);
	mutex_destroy(&pcam_inst->inst_lock);
	kfree(pcam_inst);
	f->private_data = NULL;

	if (pcam->use_count == 0) {
		if (msm_server_get_usecount() > 0) {
			rc = msm_send_close_server(pcam);
			if (rc < 0)
				pr_err("msm_send_close_server failed %d\n", rc);
		}

		if (pmctl->mctl_release) {
			pmctl->mctl_release(pmctl);
			pmctl->mctl_release = NULL;
		}

#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		kref_put(&pmctl->refcount, msm_release_ion_client);
#endif
		rc = msm_server_end_session(pcam);
		if (rc < 0)
			pr_err("msm_server_end_session fails %d\n", rc);
	}
	mutex_unlock(&pcam->vid_lock);
	return rc;
}

static unsigned int msm_poll(struct file *f, struct poll_table_struct *wait)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);
	pcam = pcam_inst->pcam;
	D("%s\n", __func__);
	if (!pcam) {
		pr_err("%s NULL pointer of camera device!\n", __func__);
		return -EINVAL;
	}
	if (pcam_inst->my_index == 0) {
		poll_wait(f, &(pcam_inst->eventHandle.wait), wait);
		if (v4l2_event_pending(&pcam_inst->eventHandle))
			rc |= POLLPRI;
	} else {
		if (!pcam_inst->vid_bufq.streaming) {
			D("%s vid_bufq.streaming is off, inst=0x%x\n",
			__func__, (u32)pcam_inst);
			return -EINVAL;
		}
		rc |= vb2_poll(&pcam_inst->vid_bufq, f, wait);
	}
	D("%s returns, rc  = 0x%x\n", __func__, rc);
	return rc;
}

long msm_v4l2_evt_notify(struct msm_cam_media_controller *mctl,
	unsigned int cmd, unsigned long evt)
{
	struct v4l2_event v4l2_ev;
	struct v4l2_event_and_payload evt_payload;
	struct msm_cam_v4l2_device *pcam = NULL;
	int rc = 0;
	struct msm_queue_cmd *event_qcmd;
	void *payload;
	if (!mctl) {
		pr_err("%s: mctl is NULL\n", __func__);
		return -EINVAL;
	}

	if (copy_from_user(&evt_payload, (void __user *)evt,
		sizeof(struct v4l2_event_and_payload))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	v4l2_ev = evt_payload.evt;
	v4l2_ev.id = 0;
	pcam = mctl->pcam_ptr;
	if(!pcam) {
		pr_err("%s: pcam is NULL\n", __func__);
		return -EINVAL;
	}
	ktime_get_ts(&v4l2_ev.timestamp);
	if (evt_payload.payload_length > 0 && evt_payload.payload != NULL) {
		mutex_lock(&pcam->event_lock);
		event_qcmd = kzalloc(sizeof(struct msm_queue_cmd), GFP_KERNEL);
		if (!event_qcmd) {
			pr_err("%s Insufficient memory. return", __func__);
			rc = -ENOMEM;
			goto event_qcmd_alloc_fail;
		}
		payload = kzalloc(evt_payload.payload_length, GFP_KERNEL);
		if (!payload) {
			pr_err("%s Insufficient memory. return", __func__);
			rc = -ENOMEM;
			goto payload_alloc_fail;
		}
		if (copy_from_user(payload,
				(void __user *)evt_payload.payload,
				evt_payload.payload_length)) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
			goto copy_from_user_failed;
		}
		event_qcmd->command = payload;
		event_qcmd->trans_code = evt_payload.transaction_id;
		msm_enqueue(&pcam->eventData_q, &event_qcmd->list_eventdata);
		mutex_unlock(&pcam->event_lock);
	}
	v4l2_event_queue(pcam->pvdev, &v4l2_ev);
	return rc;
copy_from_user_failed:
	kfree(payload);
payload_alloc_fail:
	kfree(event_qcmd);
event_qcmd_alloc_fail:
	mutex_unlock(&pcam->event_lock);
	return rc;
}


static struct v4l2_file_operations g_msm_fops = {
	.owner   = THIS_MODULE,
	.open	= msm_open,
	.poll	= msm_poll,
	.mmap	= msm_mmap,
	.release = msm_close,
	.ioctl   = video_ioctl2,
};

static int msm_cam_dev_init(struct msm_cam_v4l2_device *pcam)
{
	int rc = -ENOMEM;
	struct video_device *pvdev = NULL;
	struct i2c_client *client = NULL;
	struct platform_device *pdev = NULL;
	D("%s\n", __func__);

	/* first register the v4l2 device */
	if (pcam->sensor_sdev->flags & V4L2_SUBDEV_FL_IS_I2C) {
		client = v4l2_get_subdevdata(pcam->sensor_sdev);
		pcam->v4l2_dev.dev = &client->dev;
		pcam->media_dev.dev = &client->dev;
	} else {
		pdev = v4l2_get_subdevdata(pcam->sensor_sdev);
		pcam->v4l2_dev.dev = &pdev->dev;
		pcam->media_dev.dev = &pdev->dev;
	}

	rc = v4l2_device_register(pcam->v4l2_dev.dev, &pcam->v4l2_dev);
	if (rc < 0)
		return -EINVAL;
	else
		pcam->v4l2_dev.notify = msm_cam_v4l2_subdev_notify;


	/* now setup video device */
	pvdev = video_device_alloc();
	if (pvdev == NULL) {
		pr_err("%s: video_device_alloc failed\n", __func__);
		return rc;
	}

	strlcpy(pcam->media_dev.model, QCAMERA_NAME,
			sizeof(pcam->media_dev.model));
	rc = media_device_register(&pcam->media_dev);
	pvdev->v4l2_dev = &pcam->v4l2_dev;
	pcam->v4l2_dev.mdev = &pcam->media_dev;

	/* init video device's driver interface */
	D("sensor name = %s, sizeof(pvdev->name)=%d\n",
		pcam->sensor_sdev->name, sizeof(pvdev->name));

	/* device info - strlcpy is safer than strncpy but
	   only if architecture supports*/
	strlcpy(pvdev->name, pcam->sensor_sdev->name, sizeof(pvdev->name));

	pvdev->release   = video_device_release;
	pvdev->fops	     = &g_msm_fops;
	pvdev->ioctl_ops = &g_msm_ioctl_ops;
	pvdev->minor	 = -1;
	pvdev->vfl_type  = VFL_TYPE_GRABBER;

	media_entity_init(&pvdev->entity, 0, NULL, 0);
	pvdev->entity.type = MEDIA_ENT_T_DEVNODE_V4L;
	pvdev->entity.group_id = QCAMERA_VNODE_GROUP_ID;

	/* register v4l2 video device to kernel as /dev/videoXX */
	D("video_register_device\n");
	rc = video_register_device(pvdev,
					VFL_TYPE_GRABBER,
					msm_camera_v4l2_nr);
	if (rc) {
		pr_err("%s: video_register_device failed\n", __func__);
		goto reg_fail;
	}
	pvdev->entity.name = video_device_node_name(pvdev);
	D("%s: video device registered as /dev/video%d\n",
		__func__, pvdev->num);

	/* connect pcam and video dev to each other */
	pcam->pvdev	= pvdev;
	video_set_drvdata(pcam->pvdev, pcam);

	return rc;

reg_fail:
	video_device_release(pvdev);
	v4l2_device_unregister(&pcam->v4l2_dev);
	pcam->v4l2_dev.dev = NULL;
	return rc;
}

static struct v4l2_subdev *msm_actuator_probe(
	struct msm_actuator_info *actuator_info)
{
	struct v4l2_subdev *act_sdev;
	struct i2c_adapter *adapter = NULL;
	struct msm_actuator_ctrl_t *actrl;
	void *act_client = NULL;

	D("%s called\n", __func__);

	if (!actuator_info || !actuator_info->board_info)
		goto probe_fail;

	adapter = i2c_get_adapter(actuator_info->bus_id);
	if (!adapter)
		goto probe_fail;

	act_client = i2c_new_device(adapter, actuator_info->board_info);
	if (!act_client)
		goto device_fail;

	act_sdev = (struct v4l2_subdev *)i2c_get_clientdata(act_client);
	if (act_sdev == NULL)
		goto client_fail;

	if (actuator_info->vcm_enable) {
		actrl = get_actrl(act_sdev);
		if (actrl) {
			actrl->vcm_enable = actuator_info->vcm_enable;
			actrl->vcm_pwd = actuator_info->vcm_pwd;
		}
	}

	return act_sdev;

client_fail:
	i2c_unregister_device(act_client);
device_fail:
	i2c_put_adapter(adapter);
	adapter = NULL;
probe_fail:
	return NULL;
}

static struct v4l2_subdev *msm_eeprom_probe(
	struct msm_eeprom_info *eeprom_info)
{
	struct v4l2_subdev *eeprom_sdev;
	struct i2c_adapter *adapter = NULL;
	void *eeprom_client = NULL;

	D("%s called\n", __func__);

	if (!eeprom_info || !eeprom_info->board_info)
		goto probe_fail;

	adapter = i2c_get_adapter(eeprom_info->bus_id);
	if (!adapter)
		goto probe_fail;

	eeprom_client = i2c_new_device(adapter, eeprom_info->board_info);
	if (!eeprom_client)
		goto device_fail;

	eeprom_sdev = (struct v4l2_subdev *)i2c_get_clientdata(eeprom_client);
	if (eeprom_sdev == NULL)
		goto client_fail;

	return eeprom_sdev;
client_fail:
	pr_err("%s client_fail\n", __func__);
	i2c_unregister_device(eeprom_client);
device_fail:
	pr_err("%s device_fail\n", __func__);
	i2c_put_adapter(adapter);
	adapter = NULL;
probe_fail:
	pr_err("%s probe_fail\n", __func__);
	return NULL;
}

/* register a msm sensor into the msm device, which will probe the
 * sensor HW. if the HW exist then create a video device (/dev/videoX/)
 * to represent this sensor */
int msm_sensor_register(struct v4l2_subdev *sensor_sd)
{
	int rc = -EINVAL;
	struct msm_camera_sensor_info *sdata;
	struct msm_cam_v4l2_device *pcam;
	struct msm_sensor_ctrl_t *s_ctrl;
	struct msm_cam_subdev_info sd_info;

	D("%s for %s\n", __func__, sensor_sd->name);

	/* allocate the memory for the camera device first */
	pcam = kzalloc(sizeof(*pcam), GFP_KERNEL);
	if (!pcam) {
		pr_err("%s: could not allocate mem for msm_cam_v4l2_device\n",
			__func__);
		return -ENOMEM;
	}

	pcam->sensor_sdev = sensor_sd;
	s_ctrl = get_sctrl(sensor_sd);
	sdata = (struct msm_camera_sensor_info *) s_ctrl->sensordata;

	pcam->act_sdev = msm_actuator_probe(sdata->actuator_info);
	pcam->eeprom_sdev = msm_eeprom_probe(sdata->eeprom_info);

	D("%s: pcam =0x%p\n", __func__, pcam);

	pcam->sdata = sdata;

	/* init the user count and lock*/
	pcam->use_count = 0;
	mutex_init(&pcam->vid_lock);
	mutex_init(&pcam->mctl_node.dev_lock);

	/* Initialize the formats supported */
	rc  = msm_mctl_init_user_formats(pcam);
	if (rc < 0)
		goto failure;

	rc  = msm_cam_dev_init(pcam);
	if (rc < 0)
		goto failure;

	rc = msm_setup_mctl_node(pcam);
	if (rc < 0) {
		pr_err("%s:failed to create mctl device: %d\n",
			 __func__, rc);
		goto failure;
	}
	msm_server_update_sensor_info(pcam, sdata);

	sd_info.sdev_type = SENSOR_DEV;
	sd_info.sd_index = vnode_count;
	sd_info.irq_num = 0;
	/* register the subdevice, must be done for callbacks */
	rc = msm_cam_register_subdev_node(sensor_sd, &sd_info);
	if (rc < 0) {
		D("%s sensor sub device register failed\n",
			__func__);
		goto failure;
	}

	if (pcam->act_sdev) {
		rc = v4l2_device_register_subdev(&pcam->v4l2_dev,
				pcam->act_sdev);
		if (rc < 0) {
			D("%s actuator sub device register failed\n",
			  __func__);
			goto failure;
		}
	}

	if (pcam->eeprom_sdev) {
		rc = v4l2_device_register_subdev(&pcam->v4l2_dev,
			pcam->eeprom_sdev);
		if (rc < 0) {
			D("%s eeprom sub device register failed\n", __func__);
			goto failure;
		}
	}

	pcam->vnode_id = vnode_count++;
	return rc;

failure:
	kzfree(pcam);
	return rc;
}
EXPORT_SYMBOL(msm_sensor_register);

