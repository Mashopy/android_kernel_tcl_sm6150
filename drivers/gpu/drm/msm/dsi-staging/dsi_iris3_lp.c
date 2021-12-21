/* Copyright (C) 2019 Tcl Corporation Limited */
#include <drm/drm_bridge.h>
#include <drm/drm_encoder.h>
#include "dsi_drm.h"
#include "dsi_iris3_api.h"
#include "dsi_iris3_lightup.h"
#include "dsi_iris3_lp.h"
#include "dsi_iris3_pq.h"
#include "sde_encoder.h"
#include "sde_connector.h"
#include "dsi_iris3_log.h"

static int gpio_pulse_delay = 16 * 16 * 4 * 10 / POR_CLOCK;
static int gpio_cmd_delay = 10;

static int debug_lp_opt = 0;
/*
 * abyp light up option (need panel off/on to take effect)
 * bit[0]: 0 -- light up with PT, 1 -- light up with ABYP
 * bit[1]: 0 -- efuse mode is PT, 1 -- efuse mode is ABYP
 * bit[2]: 0 -- use mipi command to switch, 1 -- use GPIO to switch
 */
static int debug_on_opt;

static bool iris_lce_power = false;
extern uint8_t iris_pq_update_path;

#define DEBUG  false
// #undef IRIS_LOGI
// #define IRIS_LOGI IRIS_LOGE

/* set iris low power */
void iris_lp_set(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	pcfg->abypss_ctrl.abypass_mode = PASS_THROUGH_MODE;
	if (debug_on_opt & 0x1)
		pcfg->abypss_ctrl.abypass_mode = ANALOG_BYPASS_MODE;

	iris_one_wired_cmd_init(pcfg->panel);
}

/* dynamic power gating set */
void iris_dynamic_power_set(bool enable)
{
	struct iris_update_ipopt popt[IP_OPT_MAX];
	struct iris_cfg *pcfg;
	int len;

	pcfg = iris_get_cfg();

	iris_init_ipopt_ip(popt, IP_OPT_MAX);

	len = iris_update_ip_opt(popt, IP_OPT_MAX, IRIS_IP_SYS, enable ? 0xf0 : 0xf1, 0x1);

	/* 0xf0: read; 0xf1: non-read */
	len = iris_update_ip_opt(popt, IP_OPT_MAX, IRIS_IP_DMA, enable ? 0xf0 : 0xf1, 0x0);

	iris_update_pq_opt(popt, len, iris_pq_update_path);

	pcfg->dynamic_power = enable;
	IRIS_LOGD("%s: %d\n", __func__, enable);
}

/* trigger DMA to load */
void iris_dma_trigger_load(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	/* only effective when dynamic power gating off */
	if (!pcfg->dynamic_power) {
		if (iris_lce_power_status_get())
			iris_send_ipopt_cmds(IRIS_IP_DMA, 0xe1);
		else
			iris_send_ipopt_cmds(IRIS_IP_DMA, 0xe5);
	}
}

/* dynamic power gating get */
bool iris_dynamic_power_get(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	return pcfg->dynamic_power;
}

/* TE delay or EVS delay select.
   0: TE delay; 1: EVS delay */
void iris_te_select(int sel)
{
	struct iris_update_ipopt popt[IP_OPT_MAX];
	int len;

	iris_init_ipopt_ip(popt, IP_OPT_MAX);

	len = iris_update_ip_opt(popt, IP_OPT_MAX, IRIS_IP_SYS, sel ? 0xe1 : 0xe0, 0x1);
	len = iris_update_ip_opt(popt, IP_OPT_MAX, IRIS_IP_TX, sel ? 0xe1 : 0xe0, 0x1);
	len = iris_update_ip_opt(popt, IP_OPT_MAX, IRIS_IP_DTG, sel ? 0xe1 : 0xe0, 0x0);
	iris_update_pq_opt(popt, len, iris_pq_update_path);

	IRIS_LOGD("%s: %s\n", __func__, (sel ? "EVS delay" : "TE delay"));
}

/* power on/off core domain via PMU */
void iris_pmu_core_set(bool enable)
{
	struct iris_update_regval regval;
	struct iris_update_ipopt popt;

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = 0xfc;
	regval.mask = 0x00000004;
	regval.value = (enable ? 0x4 : 0x0);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt(&popt, IRIS_IP_SYS, 0xfc, 0xfc, 0);
	iris_update_pq_opt(&popt, 1, iris_pq_update_path);

	IRIS_LOGD("%s: %d\n", __func__, enable);
}

/* power on/off lce domain via PMU */
void iris_pmu_lce_set(bool enable)
{
	struct iris_update_regval regval;
	struct iris_update_ipopt popt;

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = 0x00fc;
	regval.mask = 0x00000020;
	regval.value = (enable ? 0x20 : 0x0);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt(&popt, IRIS_IP_SYS, 0xfc, 0xfc, 0);
	iris_update_pq_opt(&popt, 1, iris_pq_update_path);

	iris_lce_power_status_set(enable);

	IRIS_LOGD("%s: %d\n", __func__, enable);
}

/* lce dynamic pmu mask enable */
void iris_lce_dynamic_pmu_mask_set(bool enable)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->dynamic_power) {
		struct iris_update_regval regval;
		struct iris_update_ipopt popt;

		regval.ip = IRIS_IP_SYS;
		regval.opt_id = 0xf0;
		regval.mask = 0x00000080;
		regval.value = (enable ? 0x80 : 0x0);
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt(&popt, IRIS_IP_SYS, regval.opt_id, regval.opt_id, 0);
		iris_update_pq_opt(&popt, 1, iris_pq_update_path);
		IRIS_LOGD("%s: %d\n", __func__, enable);
	} else
		IRIS_LOGE("%s: %d. Dynmaic power is off!\n", __func__, enable);
}

void iris_abyp_flag_set(bool abyp)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (abyp) {
		pcfg->abypss_ctrl.abypass_mode = ANALOG_BYPASS_MODE;
		debug_on_opt = 0x5;
	} else {
		pcfg->abypss_ctrl.abypass_mode = PASS_THROUGH_MODE;
		debug_on_opt = 0;
	}
}

int iris_one_wired_cmd_init(struct dsi_panel *panel)
{
	int one_wired_gpio = 0;
	struct iris_cfg *pcfg = NULL;

	if (!panel) {
		return -EINVAL;
	}

	pcfg = iris_get_cfg();
	one_wired_gpio = panel->iris_reset_config.abyp_gpio;

	IRIS_LOGD("%s: %d\n", __func__, __LINE__);

	if (!gpio_is_valid(one_wired_gpio)) {
		pcfg->abypss_ctrl.analog_bypass_disable = true;

		IRIS_LOGE("%s:%d, one wired GPIO not configured\n",
			   __func__, __LINE__);
		return 0;
	}

	gpio_direction_output(one_wired_gpio, 0);

	return 0;
}

/* send one wired commands via GPIO */
void iris_one_wired_cmd_send(struct dsi_panel *panel, int cmd)
{
	int cnt = 0;
	u32 start_end_delay = 0, pulse_delay = 0;
	unsigned long flags;
	struct iris_cfg *pcfg;
	int one_wired_gpio = panel->iris_reset_config.abyp_gpio;

	pcfg = iris_get_cfg();

	if (!gpio_is_valid(one_wired_gpio)) {
		IRIS_LOGE("%s:%d, one wired GPIO not configured\n",
			   __func__, __LINE__);
		return;
	}

	start_end_delay = 16 * 16 * 16 * 10 / POR_CLOCK;  /*us*/
	pulse_delay = gpio_pulse_delay;  /*us*/

	IRIS_LOGD("cmd:%d, pulse:%d, delay:%d\n",
			cmd, pulse_delay, gpio_cmd_delay);
	if (1 == cmd)
		IRIS_LOGD("POWER_UP_SYS\n");
	else if (2 == cmd)
		IRIS_LOGD("ENTER_ANALOG_BYPASS\n");
	else if (3 == cmd)
		IRIS_LOGD("EXIT_ANALOG_BYPASS\n");
	else if (4 == cmd)
		IRIS_LOGD("POWER_DOWN_SYS\n");

	spin_lock_irqsave(&pcfg->iris_lock, flags);
	for (cnt = 0; cnt < cmd; cnt++) {
		gpio_set_value(one_wired_gpio, 1);
		udelay(pulse_delay);
		gpio_set_value(one_wired_gpio, 0);
		udelay(pulse_delay);
	}
	udelay(gpio_cmd_delay);
	spin_unlock_irqrestore(&pcfg->iris_lock, flags);
	/*end*/
	udelay(start_end_delay);
}

static ktime_t iris_ktime0;
static ktime_t iris_ktime1;
bool rec_time_en;
void iris_rec_time(int type)
{
	if (type == 0) {
		iris_ktime0 = iris_ktime1 = ktime_get();
		IRIS_LOGD("init iris time.\n");
		rec_time_en = true;
	} else if (type == 2) {
		rec_time_en = false;
	} else if (rec_time_en) {
		ktime_t ktime2;
		uint32_t timeus0, timeus1;

		ktime2 = ktime_get();
		timeus0 = (u32) ktime_to_us(ktime2)
				- (u32)ktime_to_us(iris_ktime0);
		timeus1 = (u32) ktime_to_us(ktime2)
				- (u32)ktime_to_us(iris_ktime1);
		IRIS_LOGD("iris time used = %d, total: %d\n", timeus1, timeus0);
		iris_ktime1 = ktime2;
	}
}

void iris_video_abyp_switch(bool abyp)
{
	struct sde_connector *conn;
	struct drm_event event;
	bool panel_dead = false;
	struct iris_cfg *pcfg;

	iris_rec_time(0);
	pcfg = iris_get_cfg();

	conn = to_sde_connector(pcfg->display->drm_conn);

	if (abyp)
		debug_on_opt = 0x5;
	else
		debug_on_opt = 0x0;

	IRIS_LOGI("%s: %d, %s\n", __func__, debug_on_opt, abyp ? "abyp":"pt");

	panel_dead = true;
	event.type = DRM_EVENT_PANEL_DEAD;
	event.length = sizeof(bool);
	msm_mode_object_event_notify(&conn->base.base,
		conn->base.dev, &event, (u8 *)&panel_dead);
}

static ssize_t iris_abyp_dbg_write(struct file *file,
	const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg *pcfg;
	static int cnt;

	pcfg = iris_get_cfg();

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	if (0 == val) {
		iris_abypass_to_pt_switch(pcfg->display, true, pcfg->chip_id);
		IRIS_LOGI("analog bypass->pt, %d\n", cnt);
	} else if (1 == val) {
		iris_pt_to_abypass_switch();
		IRIS_LOGI("pt->analog bypass, %d\n", cnt);
	} else if (11 <= val && val <= 18) {
		IRIS_LOGI("%s one wired %d\n", __func__, (int)(val - 10));
		iris_one_wired_cmd_send(pcfg->panel, (int)(val - 10));
	} else if (20 == val) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 5);
		IRIS_LOGI("miniPMU analog bypass->pt\n");
	} else if (21 == val) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 4);
		IRIS_LOGI("miniPMU pt->analog bypass\n");
	} else if (22 == val) {
		iris_send_ipopt_cmds(IRIS_IP_TX, 4);
		IRIS_LOGI("Enable Tx\n");
	} else if (23 == val) {
		// mutex_lock(&g_debug_mfd->switch_lock);
		iris3_lightup(pcfg->panel, &(pcfg->panel->cur_mode->priv_info->cmd_sets[DSI_CMD_SET_ABYP]));
		// mutex_unlock(&g_debug_mfd->switch_lock);
		IRIS_LOGI("lightup Iris abyp_panel_cmds\n");
	} else if (24 == val) {
		iris_video_abyp_switch(true);
	} else if (25 == val) {
		iris_video_abyp_switch(false);
	} else if (26 == val) {
		iris_rec_time(0);
		iris_pt_to_abypass_switch();
		iris_rec_time(1);
	}

	return count;
}

static ssize_t iris_lp_dbg_write(struct file *file,
	const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	if (0 == val) {
		iris_dynamic_power_set(false);
		udelay(100);
		iris_dma_trigger_load();
	} else if (1 == val)
		iris_dynamic_power_set(true);
	else if (2 == val)
		IRIS_LOGE("%s: dynamic power -- %d\n", __func__, iris_dynamic_power_get());
	else if (3 == val)
		iris_te_select(0);
	else if (4 == val)
		iris_te_select(1);
	 else if (5 == val) {
		/* for debug */
		iris_pmu_core_set(true);
	} else if (6 == val) {
		/* for debug */
		iris_pmu_core_set(false);
	} else if (7 == val) {
		/* for debug */
		iris_pmu_lce_set(true);
	} else if (8 == val) {
		/* for debug */
		iris_pmu_lce_set(false);
	}

	return count;
}

static const struct file_operations iris_abyp_dbg_fops = {
	.open = simple_open,
	.write = iris_abyp_dbg_write,
};

static const struct file_operations iris_lp_dbg_fops = {
	.open = simple_open,
	.write = iris_lp_dbg_write,
};

int iris_lp_debugfs_init(struct dsi_display *display)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld\n",
				PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u32("pulse_delay", 0644, pcfg->dbg_root,
		(u32 *)&gpio_pulse_delay);

	debugfs_create_u32("cmd_delay", 0644, pcfg->dbg_root,
		(u32 *)&gpio_cmd_delay);

	debugfs_create_u32("lp_opt", 0644, pcfg->dbg_root,
		(u32 *)&debug_lp_opt);

	debugfs_create_u32("abyp_opt", 0644, pcfg->dbg_root,
		(u32 *)&debug_on_opt);

	if (debugfs_create_file("abyp", 0644, pcfg->dbg_root, display,
				&iris_abyp_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("lp", 0644, pcfg->dbg_root, display,
				&iris_lp_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

static u32 iris_get_panel_frame_rate(void)
{
	u32 frame = 0;
	struct iris_cfg *pcfg = NULL;
	struct dsi_panel *panel = NULL;
	struct dsi_display_mode *mode = NULL;

	pcfg = iris_get_cfg();
	panel = pcfg->panel;
	mode = panel->cur_mode;

	frame = mode->timing.refresh_rate;
	if (!(frame >= 24 && frame <= 240))
		frame = 24;

	frame = ((1000/frame) + 1);
	return frame;
}

static struct drm_encoder * iris_get_drm_encoder_handle(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->display->bridge == NULL || pcfg->display->bridge->base.encoder == NULL)
		IRIS_LOGE("Can not get drm encoder\n");

	return pcfg->display->bridge->base.encoder;
}

static int iris_wait_for_lane_idle(void)
{
	int i = 0;
	int rc = 0;
	u32 lanes = 0;
	struct iris_cfg *pcfg;
	struct dsi_display * display = NULL;
	struct dsi_ctrl *dsi_ctrl = NULL;

	pcfg = iris_get_cfg();
	display = pcfg->display;

	for (i = 0; i < display->ctrl_count; i++) {
		dsi_ctrl = display->ctrl[i].ctrl;

		if (dsi_ctrl->host_config.panel_mode == DSI_OP_CMD_MODE)
			lanes = dsi_ctrl->host_config.common_config.data_lanes;

		rc = dsi_ctrl->hw.ops.wait_for_lane_idle(&dsi_ctrl->hw, lanes);
		if (rc) {
			IRIS_LOGE("lanes not entering idle, skip ULPS\n");
			return rc;
		}
	}
	return rc;
}

static void iris_dsi_ctrl_lock(void)
{
	int i = 0;
	struct iris_cfg *pcfg;
	struct dsi_display * display = NULL;
	struct dsi_ctrl *dsi_ctrl = NULL;

	pcfg = iris_get_cfg();
	display = pcfg->display;

	for (i = 0; i < display->ctrl_count; i++) {
		dsi_ctrl = display->ctrl[i].ctrl;
		mutex_lock(&dsi_ctrl->ctrl_lock);
	}
}

static void iris_dsi_ctrl_unlock(void)
{
	int i = 0;
	struct iris_cfg *pcfg;
	struct dsi_display * display = NULL;
	struct dsi_ctrl *dsi_ctrl = NULL;

	pcfg = iris_get_cfg();
	display = pcfg->display;

	for (i = 0; i < display->ctrl_count; i++) {
		dsi_ctrl = display->ctrl[i].ctrl;
		mutex_unlock(&dsi_ctrl->ctrl_lock);
	}
}

void iris_sde_encoder_rc_lock(void)
{
	struct drm_encoder * drm_enc = NULL;

	drm_enc = iris_get_drm_encoder_handle();

	sde_encoder_rc_lock(drm_enc);
}

void iris_sde_encoder_rc_unlock(void)
{
	struct drm_encoder * drm_enc = NULL;

	drm_enc = iris_get_drm_encoder_handle();

	sde_encoder_rc_unlock(drm_enc);
}

static void iris_switch_abypass_pt_pre(bool is_abyp)
{
	/*pt to abypass need to lock ctrl_lock*/
	if (is_abyp)
		iris_dsi_ctrl_lock();
	iris_wait_for_lane_idle();
}

static void iris_switch_abypass_pt_post(bool is_abyp)
{
	/*pt to abypass need to unlock ctrl_lock*/
	if (is_abyp)
		iris_dsi_ctrl_unlock();
}

bool iris_pt_to_abypass_switch(void)
{
	u32 frame = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	frame = iris_get_panel_frame_rate();

	iris_switch_abypass_pt_pre(true);

	/* enter analog bypass */
	iris_one_wired_cmd_send(pcfg->panel, ENTER_ANALOG_BYPASS);
	IRIS_LOGI("enter analog bypass switch\n");

	mdelay(frame); /* wait for 1 frames */

	/* power down sys */
	iris_one_wired_cmd_send(pcfg->panel, POWER_DOWN_SYS);

	iris_switch_abypass_pt_post(true);

	IRIS_LOGI("sys power down\n");
	pcfg->abypss_ctrl.abypass_mode = ANALOG_BYPASS_MODE;

	IRIS_LOGE("Enter analog bypass mode\n");

	return false;
}

bool iris_abypass_to_pt_switch(struct dsi_display *display, bool one_wired, int chip_id)
{
	u32 frame = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	frame = iris_get_panel_frame_rate();

	IRIS_LOGI("%s:%d\n", __func__, __LINE__);

	iris_switch_abypass_pt_pre(false);

	/* power up sys */
	if (one_wired) {
		iris_one_wired_cmd_send(pcfg->panel, POWER_UP_SYS);
		IRIS_LOGI("power up sys\n");

		udelay(10000); /* wait for 10ms */
	}

	iris_send_ipopt_cmds(IRIS_IP_RX, 0xF1);
	iris_send_ipopt_cmds(IRIS_IP_TX, 4); /* enable mipi Tx (LP) */
	IRIS_LOGI("enable mipi rx/tx\n");

	/* exit analog bypass */
	if (one_wired) {
		iris_one_wired_cmd_send(pcfg->panel, EXIT_ANALOG_BYPASS);
	} else {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 5);
	}
	IRIS_LOGE("exit analog bypass\n");

	mdelay(frame); /* wait for 1 frame */
	//mdelay(frame); /* wait for 1 frame */

	iris_read_power_mode(pcfg->panel);
	iris_read_power_mode(pcfg->panel);
	chip_id = iris_read_chip_id();
	if (chip_id == 1) {
		/* light up iris */
		if ((debug_lp_opt & 0x2) == 0) {
			iris3_lightup(pcfg->panel, &(pcfg->panel->cur_mode->priv_info->cmd_sets[DSI_CMD_SET_ABYP]));
			IRIS_LOGD("light up iris\n");
		}

		pcfg->abypss_ctrl.abypass_mode = PASS_THROUGH_MODE;
	} else {
		IRIS_LOGD("still in analog bypass\n");
		pcfg->abypss_ctrl.abypass_mode = ANALOG_BYPASS_MODE;
	}

	iris_switch_abypass_pt_post(false);

	return false;
}

void iris_abypass_switch_proc(struct dsi_display *display, int mode, bool pending)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->abypss_ctrl.analog_bypass_disable) {
		IRIS_LOGE("gpio is not setting for abypass\n");
		return;
	}

	if (pcfg->rx_mode == DSI_OP_VIDEO_MODE) {
		if (mode == ANALOG_BYPASS_MODE)
			iris_video_abyp_switch(true);
		else if (mode == PASS_THROUGH_MODE)
			iris_video_abyp_switch(false);
		else
			IRIS_LOGE("%s: switch mode: %d not supported!\n",
				__func__, mode);
		return;
	}

	if (pending) {
		mutex_lock(&pcfg->abypss_ctrl.abypass_mutex);
		pcfg->abypss_ctrl.pending_mode = mode;
		mutex_unlock(&pcfg->abypss_ctrl.abypass_mutex);
		return;
	}

	if (mode == pcfg->abypss_ctrl.abypass_mode)
		return;

	if (mode == ANALOG_BYPASS_MODE)
		iris_pt_to_abypass_switch();
	else if (mode == PASS_THROUGH_MODE)
		iris_abypass_to_pt_switch(display, true, pcfg->chip_id);
	else
		IRIS_LOGE("%s: switch mode: %d not supported!\n", __func__, mode);
}

/* Use Iris3 Analog bypass mode to light up panel */
int iris3_abyp_lightup(struct dsi_panel *panel, bool one_wired)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	if (DEBUG)
		IRIS_LOGE("iris abyp on start %s\n", one_wired ? "gpio" : "mipi");
	IRIS_LOGD("[Display] iris3_abyp_lightup ++");

#ifdef IRIS3_MIPI_TEST
	//send rx cmds first with low power
	iris_send_ipopt_cmds(IRIS_IP_RX, 0xF1);
	iris_read_chip_id();
#endif

	/* enable mipi Tx */
	iris_send_ipopt_cmds(IRIS_IP_TX, 4);

	/* enter analog bypass */
	if (one_wired) {

		iris_one_wired_cmd_send(pcfg->panel, ENTER_ANALOG_BYPASS);
		usleep_range(10000, 10100);
		/* power down sys */
		iris_one_wired_cmd_send(pcfg->panel, POWER_DOWN_SYS);
		IRIS_LOGI("sys power down\n");
		pcfg->abypss_ctrl.abypass_mode = ANALOG_BYPASS_MODE;
	} else {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 4);
		/* delay for Iris to enter bypass mode */
		udelay(10000);
	}

#ifdef IRIS3_MIPI_TEST
	iris_read_power_mode(panel);
#endif

	IRIS_LOGD("[Display] iris3_abyp_lightup --");
	if (DEBUG)
		IRIS_LOGE("iris abyp on end\n");
	return 0;
}

/* Exit Iris3 Analog bypass mode*/
void iris_abyp_lightup_exit(struct dsi_panel *panel, bool one_wire)
{
	/* exit analog bypass */
	if (one_wire)
		iris_one_wired_cmd_send(panel, EXIT_ANALOG_BYPASS);
	else
		iris_send_ipopt_cmds(IRIS_IP_SYS, 5);

	/* delay for Iris to exit bypass mode */
	udelay(10000);
	udelay(10000);

	IRIS_LOGD("%s\n", __func__);
}

void iris_lce_power_status_set(bool enable)
{
	iris_lce_power = enable;

	IRIS_LOGD("%s: %d\n", __func__, enable);
}

bool iris_lce_power_status_get(void)
{
	return iris_lce_power;
}

int iris_lightup_opt_get(void)
{
	return debug_on_opt;
}

void iris_lp_setting_off(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->abypss_ctrl.pending_mode = MAX_MODE;
}

int iris3_prepare_for_kickoff(void)
{
	struct iris_cfg *pcfg;
	int mode;

	pcfg = iris_get_cfg();

	if (pcfg->abypss_ctrl.pending_mode != MAX_MODE) {
		mutex_lock(&pcfg->abypss_ctrl.abypass_mutex);
		mode = pcfg->abypss_ctrl.pending_mode;
		pcfg->abypss_ctrl.pending_mode = MAX_MODE;
		mutex_unlock(&pcfg->abypss_ctrl.abypass_mutex);
		iris_abypass_switch_proc(pcfg->display, mode, false);
	}

	return 0;
}
