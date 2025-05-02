// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Sample HMD device, use as a starting point to make your own device driver.
 *
 *
 * Based largely on simulated_hmd.c
 *
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup drv_cheapovr
 */


#include "cheapovr_interface.h"
#include "math/m_mathinclude.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "os/os_hid.h"

#include "math/m_api.h"

#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_logging.h"

#include "util/u_var.h"



/*
 *
 * Structs and defines.
 *
 */

/*!
 * A cheapovr HMD device.
 *
 * @implements xrt_device
 */


static void
cheapovr_device_destroy(struct xrt_device *xdev)
{
	struct cheapovr_hmd *hmd = cheapovr_hmd(xdev);

	os_thread_helper_destroy(&hmd->imu_thread);

	os_mutex_destroy(&hmd->lock);

	if (hmd->dev != NULL) {
		os_hid_destroy(hmd->dev);
		hmd->dev = NULL;
	}

	free(hmd);
}



static int
cheapovr_hmd_update(struct cheapovr_hmd *hd)
{
	uint8_t buffer[6];

	int bytesRead = os_hid_read(hd->dev, buffer, sizeof(buffer), 100);
	if (bytesRead == -1) {
		if (!hd->disconnect_notified) {
			U_LOG_E("%s: HDK appeared to disconnect. Please "
			          "quit, reconnect, and try again.",
			          __func__);
			hd->disconnect_notified = true;
		}
		hd->quat_valid = false;
		return 0;
	} else if (bytesRead == 0) {
		U_LOG_W("Read 0 bytes from device");
		return 1;
	}
	while (bytesRead > 0) {
		if (bytesRead != 6) {
			U_LOG_E( "Only got %d bytes", bytesRead);
			return 1;
		}
		bytesRead = os_hid_read(hd->dev, buffer, sizeof(buffer), 0);
	}

	uint8_t *usbData = &(buffer[0]);


	struct xrt_pose tmp = XRT_POSE_IDENTITY;
	struct xrt_vec3 tmpvec;


	tmpvec.y = -((int16_t)((int16_t)usbData[1] << 8) | usbData[0]) * 0.0174533; // OK
	tmpvec.x = ((int16_t)((int16_t)usbData[3] << 8) | usbData[2]) * 0.0174533;
	tmpvec.z = -((int16_t)((int16_t)usbData[5] << 8) | usbData[4]) * 0.0174533;
	math_quat_from_euler_angles(&tmpvec, &tmp.orientation);
	U_LOG_I("%d", (int)tmp.orientation.x);
	U_LOG_I("%d", (int)tmp.orientation.y);
	U_LOG_I("%d", (int)tmp.orientation.z);
	os_mutex_lock(&hd->lock);

	math_quat_normalize(&tmp.orientation);

	// Transform with center to set it.
	math_pose_transform(&hd->center, &tmp, &hd->pose);

	os_mutex_unlock(&hd->lock);

	return 1;
}

static void *
cheapovr_hmd_run_thread(void *ptr)
{
	struct cheapovr_hmd *hd = cheapovr_hmd((struct xrt_device *)ptr);

	os_thread_helper_lock(&hd->imu_thread);
	while (os_thread_helper_is_running_locked(&hd->imu_thread)) {
		os_thread_helper_unlock(&hd->imu_thread);

		cheapovr_hmd_update(hd);

		os_thread_helper_lock(&hd->imu_thread);
	}
	return NULL;
}


static xrt_result_t
cheapovr_hmd_get_tracked_pose(struct xrt_device *xdev,
                              enum xrt_input_name name,
                              int64_t at_timestamp_ns,
                              struct xrt_space_relation *out_relation)
{
	struct cheapovr_hmd *hmd = cheapovr_hmd(xdev);


	os_mutex_lock(&hmd->lock);

	out_relation->pose = hmd->pose;

	os_mutex_unlock(&hmd->lock);
	out_relation->relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);

	return XRT_SUCCESS;
}



struct cheapovr_hmd *
cheapovr_hmd_create(struct os_hid_device *dev)
{
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);
	struct cheapovr_hmd *hmd = U_DEVICE_ALLOCATE(struct cheapovr_hmd, flags, 1, 0);
	hmd->base.update_inputs = u_device_noop_update_inputs;
	hmd->base.get_tracked_pose = cheapovr_hmd_get_tracked_pose;
	hmd->base.get_view_poses = u_device_get_view_poses;
	hmd->base.get_visibility_mask = u_device_get_visibility_mask;
	hmd->base.destroy = cheapovr_device_destroy;
	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.orientation_tracking_supported = true;
	hmd->base.position_tracking_supported = false;
	hmd->dev = dev;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->pose.orientation.w = 1.0f; // All other values set to zero.
	hmd->diameter_m = 0.05f;
	const struct xrt_pose center = XRT_POSE_IDENTITY;
	hmd->center = center;
	hmd->base.hmd->view_count = 2;

	// Setup input.
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;

	// Setup info.
	bool ret = true;
	struct u_device_simple_info info;
	info.display.w_pixels = 1280;
	info.display.h_pixels = 720;
	info.display.w_meters = 0.13f;
	info.display.h_meters = 0.07f;
	info.lens_horizontal_separation_meters = 0.13f / 2.0f;
	info.lens_vertical_position_meters = 0.07f / 2.0f;

	if (hmd->base.hmd->view_count == 1) {
		info.fov[0] = 120.0f * (M_PI / 180.0f);
		ret = u_device_setup_one_eye(&hmd->base, &info);
	} else if (hmd->base.hmd->view_count == 2) {
		info.fov[0] = 85.0f * (M_PI / 180.0f);
		info.fov[1] = 85.0f * (M_PI / 180.0f);
		ret = u_device_setup_split_side_by_side(&hmd->base, &info);
	} else {
		U_LOG_E("Invalid view count");
		ret = false;
	}
	if (!ret) {
		cheapovr_device_destroy(&hmd->base);
		return NULL;
	}
	int retr = os_thread_helper_init(&hmd->imu_thread);
	if (retr != 0) {
		U_LOG_E("Failed to start imu thread!");
		cheapovr_device_destroy(&hmd->base);
		return 0;
	}

	if (hmd->dev) {
		// Mutex before thread.
		retr = os_mutex_init(&hmd->lock);
		if (retr != 0) {
			U_LOG_E("Failed to init mutex!");
			cheapovr_device_destroy(&hmd->base);
			return NULL;
		}

		retr = os_thread_helper_start(&hmd->imu_thread, cheapovr_hmd_run_thread, hmd);
		if (retr != 0) {
			U_LOG_E("Failed to start mainboard thread!");
			cheapovr_device_destroy(&hmd->base);
			return 0;
		}
	}
	// Setup variable tracker.
	u_var_add_root(hmd, "CheapOVR HMD", true);
	u_var_add_pose(hmd, &hmd->pose, "pose");
	u_var_add_pose(hmd, &hmd->center, "center");
	u_var_add_f32(hmd, &hmd->diameter_m, "diameter_m");

	// Distortion information, fills in xdev->compute_distortion().
	u_distortion_mesh_set_none(&hmd->base);
	return hmd;
}
