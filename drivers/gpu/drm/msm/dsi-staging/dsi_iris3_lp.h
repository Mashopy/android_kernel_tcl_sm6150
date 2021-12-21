/* Copyright (C) 2019 Tcl Corporation Limited */
#ifndef _DSI_IRIS3_LP_H_
#define _DSI_IRIS3_LP_H_

#define POR_CLOCK 180	/* 0.1 Mhz*/

enum iris_onewired_cmd {
	POWER_UP_SYS = 1,
	ENTER_ANALOG_BYPASS = 2,
	EXIT_ANALOG_BYPASS = 3,
	POWER_DOWN_SYS = 4,
	RESET_SYS = 5,
	FORCE_ENTER_ANALOG_BYPASS = 6,
	FORCE_EXIT_ANALOG_BYPASS = 7,
	POWER_UP_MIPI = 8,
};

/* set iris low power */
void iris_lp_set(void);

/* dynamic power gating set */
void iris_dynamic_power_set(bool enable);

/* dynamic power gating get */
bool iris_dynamic_power_get(void);

/* lce dynamic pmu mask enable */
void iris_lce_dynamic_pmu_mask_set(bool enable);

/* send one wired commands via GPIO */
void iris_one_wired_cmd_send(struct dsi_panel *panel, int cmd);

int iris_one_wired_cmd_init(struct dsi_panel *panel);

void iris_abypass_switch_proc(struct dsi_display *display, int mode, bool pending);

void iris_lce_power_status_set(bool enable);

bool iris_lce_power_status_get(void);

/* trigger DMA to load */
void iris_dma_trigger_load(void);

int iris_lp_debugfs_init(struct dsi_display *display);

void iris_sde_encoder_rc_lock(void);

void iris_sde_encoder_rc_unlock(void);

bool iris_pt_to_abypass_switch(void);

bool iris_abypass_to_pt_switch(struct dsi_display *display, bool one_wired, int chip_id);

void iris_lp_setting_off(void);

/* Get Iris lightup opt */
int iris_lightup_opt_get(void);

void iris_abyp_lightup_exit(struct dsi_panel *panel, bool one_wire);

void iris_video_abyp_switch(bool abyp);
void iris_abyp_flag_set(bool abyp);
void iris_rec_time(int type);

#endif // _DSI_IRIS3_LP_H_
