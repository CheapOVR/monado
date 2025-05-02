// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to Sample HMD driver.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup drv_cheapovr
 */

#pragma once

#include <stdlib.h>
#include "os/os_threading.h"
#include "util/u_logging.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"
#ifdef __cplusplus
extern "C" {
#endif

#define CHPVR_VID 0xcafe
#define CHPVR_PID 0x4004

/*!
 * @defgroup drv_cheapovr Sample HMD driver
 * @ingroup drv
 *
 * @brief Driver for a Sample HMD.
 *
 * Does no actual work.
 * Assumed to not be detectable by USB VID/PID,
 * and thus exposes an "auto-prober" to explicitly discover the device.
 *
 * See @ref writing-driver for additional information.
 *
 * This device has an implementation of @ref xrt_auto_prober to perform hardware
 * detection, as well as an implementation of @ref xrt_device for the actual device.
 *
 * If your device is or has USB HID that **can** be detected based on USB VID/PID,
 * you can skip the @ref xrt_auto_prober implementation, and instead implement a
 * "found" function that matches the signature expected by xrt_prober_entry::found.
 * See for example @ref hdk_found.
 * Alternately, you might create a builder or an instance implementation directly.
 */

/*!
 * Create a auto prober for a Sample HMD.
 *
 * @ingroup drv_cheapovr
 */

struct cheapovr_hmd
{
	struct xrt_device base;
	struct os_hid_device *dev;

	struct os_thread_helper imu_thread;
	struct os_mutex lock;
	bool quat_valid;

	bool disconnect_notified;

	struct xrt_pose pose;
	struct xrt_pose center;

	float diameter_m;
};

int
chpvr_found(struct xrt_prober *xp,
            struct xrt_prober_device **devices,
            size_t device_count,
            size_t index,
            cJSON *attached_data,
            struct xrt_device **out_xdev);


struct cheapovr_hmd *
cheapovr_hmd_create(struct os_hid_device *dev);

static inline struct cheapovr_hmd *
cheapovr_hmd(struct xrt_device *xdev)
{
	return (struct cheapovr_hmd *)xdev;
}

/*!
 * @dir drivers/cheapovr
 *
 * @brief @ref drv_cheapovr files.
 */
#define HMD_TRACE(hmd, ...) U_LOG_XDEV_IFL_T(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_DEBUG(hmd, ...) U_LOG_XDEV_IFL_D(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_INFO(hmd, ...) U_LOG_XDEV_IFL_I(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_ERROR(hmd, ...) U_LOG_XDEV_IFL_E(&hmd->base, hmd->log_level, __VA_ARGS__)


#ifdef __cplusplus
}
#endif
