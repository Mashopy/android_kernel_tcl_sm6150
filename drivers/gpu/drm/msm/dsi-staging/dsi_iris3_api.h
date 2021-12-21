/* Copyright (C) 2019 Tcl Corporation Limited */
#ifndef _DSI_IRIS3_API_H_
#define _DSI_IRIS3_API_H_

// Use Iris3 Analog bypass mode to light up panel
// Note: input timing should be same with output timing
//#define IRIS3_ABYP_LIGHTUP
//#define IRIS3_MIPI_TEST

#include "dsi_display.h"
#include "dsi_iris3_def.h"

int iris3_parse_params(struct device_node *of_node, struct dsi_panel *panel);
void iris3_init(struct dsi_display *display, struct dsi_panel *panel);
int iris3_parse_lut_cmds(struct device *device, const char *name);
int iris3_lightup(struct dsi_panel *panel, struct dsi_panel_cmd_set *on_cmds);
int iris3_abyp_lightup(struct dsi_panel *panel, bool one_wired);
void iris3_lightoff_pre(void);
int iris3_lightoff(struct dsi_panel *panel, int cmd_type);
int iris3_panel_cmd_passthrough(struct dsi_panel *panel, struct dsi_panel_cmd_set *cmdset);
int iris_panel_enable(struct dsi_panel *panel, struct dsi_panel_cmd_set *on_cmds);

/*
* @Description: send continuous splash commands
* @param type IRIS_CONT_SPLASH_LK/IRIS_CONT_SPLASH_KERNEL
*/
void iris3_send_cont_splash_pkt(uint32_t type);

int iris3_operate_conf(struct msm_iris_operate_value *argp);
int iris3_operate_tool(struct msm_iris_operate_value *argp);

int iris3_hdr_enable_get(void);
int iris3_abypass_mode_get(void);

void iris3_display_prepare(struct dsi_display *display);

int iris3_update_backlight(u32 bl_lvl);
int iris3_set_panel_fod(struct mipi_dsi_device *dsi, u32 enable);
int iris3_prepare_for_kickoff(void);
int iris3_kickoff(void);
void iris3_display_enable(struct dsi_display *display);
void iris3_set_mode_from_cmdline(struct dsi_display *display, char *boot_str);

void iris_dsi_error_flag_set(void);
#endif // _DSI_IRIS3_API_H_
