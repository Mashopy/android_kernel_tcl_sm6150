/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __MSM_DRM_H__
#define __MSM_DRM_H__

#include "drm.h"
#include "sde_drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* Please note that modifications to all structs defined here are
 * subject to backwards-compatibility constraints:
 *  1) Do not use pointers, use __u64 instead for 32 bit / 64 bit
 *     user/kernel compatibility
 *  2) Keep fields aligned to their size
 *  3) Because of how drm_ioctl() works, we can add new fields at
 *     the end of an ioctl if some care is taken: drm_ioctl() will
 *     zero out the new fields at the tail of the ioctl, so a zero
 *     value should have a backwards compatible meaning.  And for
 *     output params, userspace won't see the newly added output
 *     fields.. so that has to be somehow ok.
 */

#define MSM_PIPE_NONE        0x00
#define MSM_PIPE_2D0         0x01
#define MSM_PIPE_2D1         0x02
#define MSM_PIPE_3D0         0x10

/* The pipe-id just uses the lower bits, so can be OR'd with flags in
 * the upper 16 bits (which could be extended further, if needed, maybe
 * we extend/overload the pipe-id some day to deal with multiple rings,
 * but even then I don't think we need the full lower 16 bits).
 */
#define MSM_PIPE_ID_MASK     0xffff
#define MSM_PIPE_ID(x)       ((x) & MSM_PIPE_ID_MASK)
#define MSM_PIPE_FLAGS(x)    ((x) & ~MSM_PIPE_ID_MASK)

/* timeouts are specified in clock-monotonic absolute times (to simplify
 * restarting interrupted ioctls).  The following struct is logically the
 * same as 'struct timespec' but 32/64b ABI safe.
 */
struct drm_msm_timespec {
	__s64 tv_sec;          /* seconds */
	__s64 tv_nsec;         /* nanoseconds */
};

/*
 * HDR Metadata
 * These are defined as per EDID spec and shall be used by the sink
 * to set the HDR metadata for playback from userspace.
 */

#define HDR_PRIMARIES_COUNT   3

/* HDR EOTF */
#define HDR_EOTF_SDR_LUM_RANGE	0x0
#define HDR_EOTF_HDR_LUM_RANGE	0x1
#define HDR_EOTF_SMTPE_ST2084	0x2
#define HDR_EOTF_HLG		0x3

#define DRM_MSM_EXT_HDR_METADATA
struct drm_msm_ext_hdr_metadata {
	__u32 hdr_state;        /* HDR state */
	__u32 eotf;             /* electro optical transfer function */
	__u32 hdr_supported;    /* HDR supported */
	__u32 display_primaries_x[HDR_PRIMARIES_COUNT]; /* Primaries x */
	__u32 display_primaries_y[HDR_PRIMARIES_COUNT]; /* Primaries y */
	__u32 white_point_x;    /* white_point_x */
	__u32 white_point_y;    /* white_point_y */
	__u32 max_luminance;    /* Max luminance */
	__u32 min_luminance;    /* Min Luminance */
	__u32 max_content_light_level; /* max content light level */
	__u32 max_average_light_level; /* max average light level */
};

/**
 * HDR sink properties
 * These are defined as per EDID spec and shall be used by the userspace
 * to determine the HDR properties to be set to the sink.
 */
#define DRM_MSM_EXT_HDR_PROPERTIES
struct drm_msm_ext_hdr_properties {
	__u8 hdr_metadata_type_one;   /* static metadata type one */
	__u32 hdr_supported;          /* HDR supported */
	__u32 hdr_eotf;               /* electro optical transfer function */
	__u32 hdr_max_luminance;      /* Max luminance */
	__u32 hdr_avg_luminance;      /* Avg luminance */
	__u32 hdr_min_luminance;      /* Min Luminance */
};

#define MSM_PARAM_GPU_ID     0x01
#define MSM_PARAM_GMEM_SIZE  0x02
#define MSM_PARAM_CHIP_ID    0x03
#define MSM_PARAM_MAX_FREQ   0x04
#define MSM_PARAM_TIMESTAMP  0x05
#define MSM_PARAM_GMEM_BASE  0x06

struct drm_msm_param {
	__u32 pipe;           /* in, MSM_PIPE_x */
	__u32 param;          /* in, MSM_PARAM_x */
	__u64 value;          /* out (get_param) or in (set_param) */
};

/*
 * GEM buffers:
 */

#define MSM_BO_SCANOUT       0x00000001     /* scanout capable */
#define MSM_BO_GPU_READONLY  0x00000002
#define MSM_BO_CACHE_MASK    0x000f0000
/* cache modes */
#define MSM_BO_CACHED        0x00010000
#define MSM_BO_WC            0x00020000
#define MSM_BO_UNCACHED      0x00040000

#define MSM_BO_FLAGS         (MSM_BO_SCANOUT | \
                              MSM_BO_GPU_READONLY | \
                              MSM_BO_CACHED | \
                              MSM_BO_WC | \
                              MSM_BO_UNCACHED)

struct drm_msm_gem_new {
	__u64 size;           /* in */
	__u32 flags;          /* in, mask of MSM_BO_x */
	__u32 handle;         /* out */
};

#define MSM_INFO_IOVA	0x01

#define MSM_INFO_FLAGS (MSM_INFO_IOVA)

struct drm_msm_gem_info {
	__u32 handle;         /* in */
	__u32 flags;	      /* in - combination of MSM_INFO_* flags */
	__u64 offset;         /* out, mmap() offset or iova */
};

#define MSM_PREP_READ        0x01
#define MSM_PREP_WRITE       0x02
#define MSM_PREP_NOSYNC      0x04

#define MSM_PREP_FLAGS       (MSM_PREP_READ | MSM_PREP_WRITE | MSM_PREP_NOSYNC)

struct drm_msm_gem_cpu_prep {
	__u32 handle;         /* in */
	__u32 op;             /* in, mask of MSM_PREP_x */
	struct drm_msm_timespec timeout;   /* in */
};

struct drm_msm_gem_cpu_fini {
	__u32 handle;         /* in */
};

/*
 * Cmdstream Submission:
 */

/* The value written into the cmdstream is logically:
 *
 *   ((relocbuf->gpuaddr + reloc_offset) << shift) | or
 *
 * When we have GPU's w/ >32bit ptrs, it should be possible to deal
 * with this by emit'ing two reloc entries with appropriate shift
 * values.  Or a new MSM_SUBMIT_CMD_x type would also be an option.
 *
 * NOTE that reloc's must be sorted by order of increasing submit_offset,
 * otherwise EINVAL.
 */
struct drm_msm_gem_submit_reloc {
	__u32 submit_offset;  /* in, offset from submit_bo */
#ifdef __cplusplus
	__u32 or_val;
#else
	__u32 or; /* in, value OR'd with result */
#endif
	__s32  shift;          /* in, amount of left shift (can be negative) */
	__u32 reloc_idx;      /* in, index of reloc_bo buffer */
	__u64 reloc_offset;   /* in, offset from start of reloc_bo */
};

/* submit-types:
 *   BUF - this cmd buffer is executed normally.
 *   IB_TARGET_BUF - this cmd buffer is an IB target.  Reloc's are
 *      processed normally, but the kernel does not setup an IB to
 *      this buffer in the first-level ringbuffer
 *   CTX_RESTORE_BUF - only executed if there has been a GPU context
 *      switch since the last SUBMIT ioctl
 */
#define MSM_SUBMIT_CMD_BUF             0x0001
#define MSM_SUBMIT_CMD_IB_TARGET_BUF   0x0002
#define MSM_SUBMIT_CMD_CTX_RESTORE_BUF 0x0003
struct drm_msm_gem_submit_cmd {
	__u32 type;           /* in, one of MSM_SUBMIT_CMD_x */
	__u32 submit_idx;     /* in, index of submit_bo cmdstream buffer */
	__u32 submit_offset;  /* in, offset into submit_bo */
	__u32 size;           /* in, cmdstream size */
	__u32 pad;
	__u32 nr_relocs;      /* in, number of submit_reloc's */
	__u64 relocs;         /* in, ptr to array of submit_reloc's */
};

/* Each buffer referenced elsewhere in the cmdstream submit (ie. the
 * cmdstream buffer(s) themselves or reloc entries) has one (and only
 * one) entry in the submit->bos[] table.
 *
 * As a optimization, the current buffer (gpu virtual address) can be
 * passed back through the 'presumed' field.  If on a subsequent reloc,
 * userspace passes back a 'presumed' address that is still valid,
 * then patching the cmdstream for this entry is skipped.  This can
 * avoid kernel needing to map/access the cmdstream bo in the common
 * case.
 */
#define MSM_SUBMIT_BO_READ             0x0001
#define MSM_SUBMIT_BO_WRITE            0x0002

#define MSM_SUBMIT_BO_FLAGS            (MSM_SUBMIT_BO_READ | MSM_SUBMIT_BO_WRITE)

struct drm_msm_gem_submit_bo {
	__u32 flags;          /* in, mask of MSM_SUBMIT_BO_x */
	__u32 handle;         /* in, GEM handle */
	__u64 presumed;       /* in/out, presumed buffer address */
};

/* Valid submit ioctl flags: */
#define MSM_SUBMIT_NO_IMPLICIT   0x80000000 /* disable implicit sync */
#define MSM_SUBMIT_FENCE_FD_IN   0x40000000 /* enable input fence_fd */
#define MSM_SUBMIT_FENCE_FD_OUT  0x20000000 /* enable output fence_fd */
#define MSM_SUBMIT_FLAGS                ( \
		MSM_SUBMIT_NO_IMPLICIT   | \
		MSM_SUBMIT_FENCE_FD_IN   | \
		MSM_SUBMIT_FENCE_FD_OUT  | \
		0)

/* Each cmdstream submit consists of a table of buffers involved, and
 * one or more cmdstream buffers.  This allows for conditional execution
 * (context-restore), and IB buffers needed for per tile/bin draw cmds.
 */
struct drm_msm_gem_submit {
	__u32 flags;          /* MSM_PIPE_x | MSM_SUBMIT_x */
	__u32 fence;          /* out */
	__u32 nr_bos;         /* in, number of submit_bo's */
	__u32 nr_cmds;        /* in, number of submit_cmd's */
	__u64 bos;            /* in, ptr to array of submit_bo's */
	__u64 cmds;           /* in, ptr to array of submit_cmd's */
	__s32 fence_fd;       /* in/out fence fd (see MSM_SUBMIT_FENCE_FD_IN/OUT) */
};

/* The normal way to synchronize with the GPU is just to CPU_PREP on
 * a buffer if you need to access it from the CPU (other cmdstream
 * submission from same or other contexts, PAGE_FLIP ioctl, etc, all
 * handle the required synchronization under the hood).  This ioctl
 * mainly just exists as a way to implement the gallium pipe_fence
 * APIs without requiring a dummy bo to synchronize on.
 */
struct drm_msm_wait_fence {
	__u32 fence;          /* in */
	__u32 pad;
	struct drm_msm_timespec timeout;   /* in */
};

/* madvise provides a way to tell the kernel in case a buffers contents
 * can be discarded under memory pressure, which is useful for userspace
 * bo cache where we want to optimistically hold on to buffer allocate
 * and potential mmap, but allow the pages to be discarded under memory
 * pressure.
 *
 * Typical usage would involve madvise(DONTNEED) when buffer enters BO
 * cache, and madvise(WILLNEED) if trying to recycle buffer from BO cache.
 * In the WILLNEED case, 'retained' indicates to userspace whether the
 * backing pages still exist.
 */
#define MSM_MADV_WILLNEED 0       /* backing pages are needed, status returned in 'retained' */
#define MSM_MADV_DONTNEED 1       /* backing pages not needed */
#define __MSM_MADV_PURGED 2       /* internal state */

struct drm_msm_gem_madvise {
	__u32 handle;         /* in, GEM handle */
	__u32 madv;           /* in, MSM_MADV_x */
	__u32 retained;       /* out, whether backing store still exists */
};

/* HDR WRGB x and y index */
#define DISPLAY_PRIMARIES_WX 0
#define DISPLAY_PRIMARIES_WY 1
#define DISPLAY_PRIMARIES_RX 2
#define DISPLAY_PRIMARIES_RY 3
#define DISPLAY_PRIMARIES_GX 4
#define DISPLAY_PRIMARIES_GY 5
#define DISPLAY_PRIMARIES_BX 6
#define DISPLAY_PRIMARIES_BY 7
#define DISPLAY_PRIMARIES_MAX 8

struct drm_panel_hdr_properties {
	__u32 hdr_enabled;

	/* WRGB X and y values arrayed in format */
	/* [WX, WY, RX, RY, GX, GY, BX, BY] */
	__u32 display_primaries[DISPLAY_PRIMARIES_MAX];

	/* peak brightness supported by panel */
	__u32 peak_brightness;
	/* Blackness level supported by panel */
	__u32 blackness_level;
};

/**
 * struct drm_msm_event_req - Payload to event enable/disable ioctls.
 * @object_id: DRM object id. e.g.: for crtc pass crtc id.
 * @object_type: DRM object type. e.g.: for crtc set it to DRM_MODE_OBJECT_CRTC.
 * @event: Event for which notification is being enabled/disabled.
 *         e.g.: for Histogram set - DRM_EVENT_HISTOGRAM.
 * @client_context: Opaque pointer that will be returned during event response
 *                  notification.
 * @index: Object index(e.g.: crtc index), optional for user-space to set.
 *         Driver will override value based on object_id and object_type.
 */
struct drm_msm_event_req {
	__u32 object_id;
	__u32 object_type;
	__u32 event;
	__u64 client_context;
	__u32 index;
};

/**
 * struct drm_msm_event_resp - payload returned when read is called for
 *                            custom notifications.
 * @base: Event type and length of complete notification payload.
 * @info: Contains information about DRM that which raised this event.
 * @data: Custom payload that driver returns for event type.
 *        size of data = base.length - (sizeof(base) + sizeof(info))
 */
struct drm_msm_event_resp {
	struct drm_event base;
	struct drm_msm_event_req info;
	__u8 data[];
};

/**
 * struct drm_msm_power_ctrl: Payload to enable/disable the power vote
 * @enable: enable/disable the power vote
 * @flags:  operation control flags, for future use
 */
struct drm_msm_power_ctrl {
	__u32 enable;
	__u32 flags;
};

#define DRM_MSM_GET_PARAM              0x00
/* placeholder:
#define DRM_MSM_SET_PARAM              0x01
 */
#define DRM_MSM_GEM_NEW                0x02
#define DRM_MSM_GEM_INFO               0x03
#define DRM_MSM_GEM_CPU_PREP           0x04
#define DRM_MSM_GEM_CPU_FINI           0x05
#define DRM_MSM_GEM_SUBMIT             0x06
#define DRM_MSM_WAIT_FENCE             0x07
#define DRM_MSM_GEM_MADVISE            0x08

#define DRM_SDE_WB_CONFIG              0x40
#define DRM_MSM_REGISTER_EVENT         0x41
#define DRM_MSM_DEREGISTER_EVENT       0x42
#define DRM_MSM_RMFB2                  0x43
#define DRM_MSM_POWER_CTRL             0x44

/* sde custom events */
#define DRM_EVENT_HISTOGRAM 0x80000000
#define DRM_EVENT_AD_BACKLIGHT 0x80000001
#define DRM_EVENT_CRTC_POWER 0x80000002
#define DRM_EVENT_SYS_BACKLIGHT 0x80000003
#define DRM_EVENT_SDE_POWER 0x80000004
#define DRM_EVENT_IDLE_NOTIFY 0x80000005
#define DRM_EVENT_PANEL_DEAD 0x80000006 /* ESD event */
#define DRM_EVENT_SDE_HW_RECOVERY 0X80000007

#define DRM_IOCTL_MSM_GET_PARAM        DRM_IOWR(DRM_COMMAND_BASE + DRM_MSM_GET_PARAM, struct drm_msm_param)
#define DRM_IOCTL_MSM_GEM_NEW          DRM_IOWR(DRM_COMMAND_BASE + DRM_MSM_GEM_NEW, struct drm_msm_gem_new)
#define DRM_IOCTL_MSM_GEM_INFO         DRM_IOWR(DRM_COMMAND_BASE + DRM_MSM_GEM_INFO, struct drm_msm_gem_info)
#define DRM_IOCTL_MSM_GEM_CPU_PREP     DRM_IOW (DRM_COMMAND_BASE + DRM_MSM_GEM_CPU_PREP, struct drm_msm_gem_cpu_prep)
#define DRM_IOCTL_MSM_GEM_CPU_FINI     DRM_IOW (DRM_COMMAND_BASE + DRM_MSM_GEM_CPU_FINI, struct drm_msm_gem_cpu_fini)
#define DRM_IOCTL_MSM_GEM_SUBMIT       DRM_IOWR(DRM_COMMAND_BASE + DRM_MSM_GEM_SUBMIT, struct drm_msm_gem_submit)
#define DRM_IOCTL_MSM_WAIT_FENCE       DRM_IOW (DRM_COMMAND_BASE + DRM_MSM_WAIT_FENCE, struct drm_msm_wait_fence)
#define DRM_IOCTL_MSM_GEM_MADVISE      DRM_IOWR(DRM_COMMAND_BASE + DRM_MSM_GEM_MADVISE, struct drm_msm_gem_madvise)
#define DRM_IOCTL_SDE_WB_CONFIG \
	DRM_IOW((DRM_COMMAND_BASE + DRM_SDE_WB_CONFIG), struct sde_drm_wb_cfg)
#define DRM_IOCTL_MSM_REGISTER_EVENT   DRM_IOW((DRM_COMMAND_BASE + \
			DRM_MSM_REGISTER_EVENT), struct drm_msm_event_req)
#define DRM_IOCTL_MSM_DEREGISTER_EVENT DRM_IOW((DRM_COMMAND_BASE + \
			DRM_MSM_DEREGISTER_EVENT), struct drm_msm_event_req)
#define DRM_IOCTL_MSM_RMFB2 DRM_IOW((DRM_COMMAND_BASE + \
			DRM_MSM_RMFB2), unsigned int)
#define DRM_IOCTL_MSM_POWER_CTRL DRM_IOW((DRM_COMMAND_BASE + \
			DRM_MSM_POWER_CTRL), struct drm_msm_power_ctrl)

/* MODIFIED-BEGIN by Haojun Chen, 2019-05-11,BUG-7765094*/
#if defined(CONFIG_PXLW_IRIS3) || defined(PXLW_IRIS3)
#define DRM_MSM_IRIS_OPERATE_CONF      0x50
#define DRM_MSM_IRIS_OPERATE_TOOL      0x51
#define PANEL_BL_MAX_RATIO 10000

enum iris_oprt_type {
	IRIS_OPRT_TOOL_DSI,
	IRIS_OPRT_CONFIGURE,
	IRIS_OPRT_CONFIGURE_NEW,
	IRIS_OPRT_CONFIGURE_NEW_GET,
	IRIS_OPRT_MAX_TYPE,
};

struct msmfb_mipi_dsi_cmd {
	__u8 dtype;
	__u8 vc;
#define MSMFB_MIPI_DSI_COMMAND_LAST 1
#define MSMFB_MIPI_DSI_COMMAND_ACK  2
#define MSMFB_MIPI_DSI_COMMAND_HS   4
#define MSMFB_MIPI_DSI_COMMAND_BLLP 8
#define MSMFB_MIPI_DSI_COMMAND_DEBUG 16
#define MSMFB_MIPI_DSI_COMMAND_TO_PANEL 32
#define MSMFB_MIPI_DSI_COMMAND_T 64

	__u32 iris_ocp_type;
	__u32 iris_ocp_addr;
	__u32 iris_ocp_value;
	__u32 iris_ocp_size;

	__u16 flags;
	__u16 length;
	__u8 *payload;
	__u8 response[16];
};

struct msm_iris_operate_value {
	unsigned int type;
	unsigned int count;
	void *values;
};

enum iris_config_type {
	IRIS_PEAKING = 0,
	IRIS_CHIP_VERSION = 33,      // 0x0 : IRIS2, 0x1 : IRIS2-plus, 0x2 : IRIS3-lite
	IRIS_LUX_VALUE = 34,
	IRIS_CCT_VALUE = 35,
	IRIS_READING_MODE = 36,

	IRIS_CM_6AXES = 37,
	IRIS_CM_FTC_ENABLE = 38,
	IRIS_CM_COLOR_TEMP_MODE = 39,
	IRIS_CM_COLOR_GAMUT = 40,
	IRIS_LCE_MODE = 41,
	IRIS_LCE_LEVEL = 42,
	IRIS_GRAPHIC_DET_ENABLE = 43,
	IRIS_AL_ENABLE = 44,			//AL means ambient light
	IRIS_DBC_LEVEL = 45,
	IRIS_DEMO_MODE = 46,
	IRIS_SDR2HDR = 47,
	IRIS_COLOR_TEMP_VALUE = 48,
	IRIS_HDR_MAXCLL = 49,
	IRIS_CM_COLOR_GAMUT_PRE = 51,
	IRIS_DBC_LCE_POWER= 52,
	IRIS_DBC_LCE_DATA_PATH = 53,
	IRIS_DYNAMIC_POWER_CTRL = 54,
	IRIS_DMA_LOAD = 55,
	IRIS_ANALOG_BYPASS_MODE = 56,
	IRIS_PANEL_TYPE = 57,
	IRIS_HDR_PANEL_NITES_SET = 60,
	IRIS_PEAKING_IDLE_CLK_ENABLE = 61,
	IRIS_CM_MAGENTA_GAIN = 62,
	IRIS_CM_RED_GAIN = 63,
	IRIS_CM_YELLOW_GAIN = 64,
	IRIS_CM_GREEN_GAIN = 65,
	IRIS_CM_BLUE_GAIN = 66,
	IRIS_CM_CYAN_GAIN = 67,
	IRIS_BLC_PWM_ENABLE = 68,
	IRIS_DBC_LED_GAIN = 69,
	IRIS_SCALER_FILTER_LEVEL = 70,
	IRIS_CCF1_UPDATE = 71,
	IRIS_CCF2_UPDATE = 72,
	IRIS_FW_UPDATE = 73,
	IRIS_HUE_SAT_ADJ = 74,
	IRIS_SCALER_PP_FILTER_LEVEL = 76,
	IRIS_CSC_MATRIX = 75,
	IRIS_HDR_PREPARE = 90,
	IRIS_HDR_COMPLETE = 91,
	IRIS_MCF_DATA = 92,
	IRIS_Y5P = 93,
	IRIS_PANEL_NITS = 99,

	IRIS_DBG_TARGET_REGADDR_VALUE_GET = 103,
	IRIS_DBG_TARGET_REGADDR_VALUE_SET = 105,
	IRIS_DBG_KERNEL_LOG_LEVEL = 106,
	IRIS_DBG_SEND_PACKAGE = 107,
	IRIS_DBG_TARGET_REGADDR_VALUE_SET2 = 112,
	IRIS_DEBUG_CAP = 113,
	IRIS_POWER_MODE_CHECK=114, // MODIFIED by ji.chen, 2019-08-16,BUG-8256828

	IRIS_MODE_SET = 120,
	IRIS_VIDEO_FRAME_RATE_SET = 121,
	IRIS_OUT_FRAME_RATE_SET = 122,	// debug only
	IRIS_OSD_ENABLE = 123,
	IRIS_WORK_MODE = 126,	// [23-16]: pwil mode, [15-8]: tx mode, [7-0]: rx mode
	IRIS_FRC_LOW_LATENCY = 127,
	IRIS_PANEL_TE = 128,
	IRIS_AP_TE = 129,

	IRIS_CONFIG_TYPE_MAX
};

enum SDR2HDR_CASE{
	SDR2HDR_Bypass = 0,
	HDR10In_ICtCp,
	HDR10In_YCbCr,
	ICtCpIn_YCbCr,
	SDR709_2_709,
	SDR709_2_p3,
	SDR709_2_2020,
};

enum SDR2HDR_LUT_GAMMA_INDEX{
	SDR2HDR_LUT_GAMMA_120 = 0,
	SDR2HDR_LUT_GAMMA_106 = 1,
	SDR2HDR_LUT_GAMMA_102 = 2,
	SDR2HDR_LUT_GAMMA_100 = 3,
	SDR2HDR_LUT_GAMMA_MAX
};

struct msmfb_iris_ambient_info {
	uint32_t ambient_lux;
	uint32_t ambient_bl_ratio;
	void *lut_lut2_payload;
};

struct msmfb_iris_maxcll_info {
    uint32_t mMAXCLL;
    void *lut_luty_payload;
    void *lut_lutuv_payload;
};

enum iris_chip_version {
	IRIS2_VER = 0,
	IRIS2PLUS_VER,
	IRIS3LITE_VER,
	IRIS5_VER,
	UNKOWN_VER
};

#define DRM_IOCTL_MSM_IRIS_OPERATE_CONF     DRM_IOW (DRM_COMMAND_BASE + DRM_MSM_IRIS_OPERATE_CONF, struct msm_iris_operate_value)
#define DRM_IOCTL_MSM_IRIS_OPERATE_TOOL     DRM_IOW (DRM_COMMAND_BASE + DRM_MSM_IRIS_OPERATE_TOOL, struct msm_iris_operate_value)

#endif // CONFIG_PXLW_IRIS3 || PXLW_IRIS3
/* MODIFIED-END by Haojun Chen,BUG-7765094*/

#if defined(__cplusplus)
}
#endif

#endif /* __MSM_DRM_H__ */
