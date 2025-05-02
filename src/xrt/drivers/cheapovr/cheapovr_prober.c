// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  "auto-prober" for Sample HMD that can be autodetected but not through USB VID/PID.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_cheapovr
 */

#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"


#include "cheapovr_interface.h"
#include <stdio.h>




struct cheapovr_prober
{
	struct xrt_auto_prober base;
};

int
chpvr_found(struct xrt_prober *xp,
          struct xrt_prober_device **devices,
          size_t device_count,
          size_t index,
          cJSON *attached_data,
          struct xrt_device **out_xdev)
{	
	
	struct xrt_prober_device *dev = devices[index];

	unsigned char buf[256] = {0};
	int result = xrt_prober_get_string_descriptor(xp, dev, XRT_PROBER_STRING_PRODUCT, buf, sizeof(buf));

	struct os_hid_device *hid = NULL;
	result = xrt_prober_open_hid_interface(xp, dev, 0, &hid);
	if (result != 0) {
		return -1;
	}

	struct cheapovr_hmd *hd = cheapovr_hmd_create(hid);
	if (hd == NULL) {
		return -1;
	}
	*out_xdev = &hd->base;

	return 1;
}
