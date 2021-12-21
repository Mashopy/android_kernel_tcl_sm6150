/* Copyright (C) 2019 Tcl Corporation Limited */
#ifndef _DSI_IRIS3_LIGHTUP_H_
#define _DSI_IRIS3_LIGHTUP_H_

#include <linux/completion.h>

#include "dsi_iris3.h"

#define IRIS_IP_OPT_CNT   15
#define IRIS_IP_CNT      19

#define IRIS_CHIP_CNT   2

#define MDSS_MAX_PANEL_LEN      256

/*use to parse dtsi cmd list*/
struct iris_parsed_hdr {
	uint32_t dtype;  /* dsi command type 0x23 0x29*/
	//uint32_t lwtype; /* 8bit burst single */
	uint32_t last; /*last in chain*/
	uint32_t wait; /*wait time*/
	uint32_t ip; /*ip type*/
	uint32_t opt; /*ip option and lp or hs*/
	uint32_t dlen; /*payload len*/
};


/* iris ip option, it will create according to opt_id.
*  link_state will be create according to the last cmds
*/
struct iris_ip_opt {
	uint8_t opt_id; /*option identifier*/
	uint8_t len; /*option length*/
	uint8_t link_state; /*high speed or low power*/
	struct dsi_cmd_desc *cmd; /*the first cmd of desc*/
};

/*ip search index*/
struct iris_ip_index {
	//char ip; /*IP index*/
	//char enable; /*1-open 0-close*/
	int32_t opt_cnt; /*ip option number*/
	//char opt_cur; /*current use option*/
	struct iris_ip_opt *opt; /*option array*/
};

struct iris_pq_ipopt_val {
	int32_t opt_cnt;
	uint8_t ip;
	uint8_t *popt;
};

struct iris_pq_init_val {
	int32_t ip_cnt;
	struct iris_pq_ipopt_val  *val;
};

struct iris_cmd_statics {
	int cnt;
	int len;
};

/*used to control iris_ctrl opt sequence*/
struct iris_ctrl_opt {
	uint8_t ip;
	uint8_t opt_id;
	uint8_t skip_last;
	uint8_t reserved;
	//struct iris_ip_opt *opt;
};

struct iris_ctrl_seq {
	int32_t cnt;
	struct iris_ctrl_opt *ctrl_opt;
};

struct iris_update_ipopt
{
	uint8_t ip;
	uint8_t opt_old;
	uint8_t opt_new;
	uint8_t skip_last;
};

struct iris_update_regval {
	uint8_t ip;
	uint8_t opt_id;
	uint16_t reserved;
	uint32_t mask;
	//uint32_t addr;
	uint32_t value;
};

struct iris_abypass_ctrl {
	bool analog_bypass_disable;
	uint8_t abypass_mode;
	uint16_t pending_mode;		// pending_mode is accessed by SDEEncoder and HWBinder.
	struct mutex abypass_mutex;
};

//will pack all the commands here
struct iris_out_cmds{
	/* will be used before cmds sent out */
	struct dsi_cmd_desc *iris_cmds_buf;
	u32 cmds_index;
};


typedef int (*iris3_i2c_read_cb)(u32 reg_addr,
			      u32 *reg_val);

typedef int (*iris3_i2c_write_cb)(u32 reg_addr,
			      u32 reg_val);

typedef int (*iris3_i2c_burst_write_cb)(u32 start_addr,
			      u32 *lut_buffer,
			      u16 reg_num);
enum {
	IRIS_STATUS_INVALID,
	IRIS_STATUS_INIT,
	IRIS_STATUS_MINI_LIGHTUP,
	IRIS_STATUS_FULL_LIGHTUP,

};
/*iris lightup configure commands*/
struct iris_cfg {
	bool dynamic_power;
	uint8_t panel_type;
	uint16_t panel_nits;
	uint8_t chip_id;
	uint8_t power_mode;
	uint8_t lut_mode;
	uint32_t add_last_flag;
	uint32_t add_on_last_flag;	/* panel on packet size */
	uint32_t add_cont_last_flag;	/* cont-splash packet size */
	uint32_t add_pt_last_flag;		/* panel on finished or cont-splash finished packet size */
	uint32_t split_pkt_size;
	uint32_t lut_cmds_cnt;
	uint32_t none_lut_cmds_cnt;
	uint32_t panel_dimming_brightness;
	int32_t valid;
	spinlock_t iris_lock;
	struct mutex mutex;
	struct dsi_display *display;
	struct dsi_panel *panel;
	struct iris_ip_index  ip_index_arr[IRIS_IP_CNT];
	struct dsi_panel_cmd_set  cmds;
	struct iris_ctrl_seq   ctrl_seq[IRIS_CHIP_CNT];
	struct iris_ctrl_seq   ctrl_seq_cs[IRIS_CHIP_CNT];
	struct iris_pq_init_val  pq_init_val;
	struct dentry *dbg_root;
	struct iris_abypass_ctrl abypss_ctrl;
	struct iris_out_cmds iris_cmds;
	uint32_t min_color_temp;
	uint32_t max_color_temp;
	u8 rx_mode;
	u8 tx_mode;
	int pwil_mode;
	struct work_struct cont_splash_work;
	struct work_struct lut_update_work;
	struct completion frame_ready_completion;
	iris3_i2c_read_cb iris3_i2c_read;
	iris3_i2c_write_cb iris3_i2c_write;
	iris3_i2c_burst_write_cb iris3_i2c_burst_write;
	struct mutex gs_mutex;
	bool bta_error;
};

struct iris_cmd_comp {
	int32_t link_state;
	int32_t cnt;
	struct dsi_cmd_desc *cmd;
};

struct iris_cmd_priv{
	int32_t cmd_num;
	struct iris_cmd_comp cmd_comp[IRIS_IP_OPT_CNT];
};

enum PATH_TYPE {
	PATH_I2C = 0,
	PATH_DSI,
};

struct iris_cfg * iris_get_cfg(void);
void iris_out_cmds_buf_reset(void);
void iris_send_ipopt_cmds(int32_t ip, int32_t opt_id);
void iris_update_pq_opt(struct iris_update_ipopt * popt, int len, uint8_t path);
void iris_update_bitmask_regval_nonread(
				struct iris_update_regval *pregval, bool is_commit);

void iris_alloc_seq_space(void);

void iris_init_update_ipopt(struct iris_update_ipopt *popt,
		uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t skip_last);
struct iris_pq_ipopt_val  *  iris_get_cur_ipopt_val(uint8_t ip);

int iris_init_update_ipopt_t(struct iris_update_ipopt *popt,  int len,
						uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t skip_last);
struct iris_ip_opt * iris_find_ip_opt(uint8_t ip, uint8_t opt_id);
/*
* @description  get assigned position data of ip opt
* @param ip       ip sign
* @param opt_id   option id of ip
* @param pos      the position of option payload
* @return   fail NULL/success payload data of position
*/
uint32_t  * iris_get_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos);

int iris_lut_send(u8 lut_type, u8 lut_table_index, u32 lut_pkt_index, bool bSendFlag);
void iris_set_lut_cnt(uint32_t cnt);
int32_t iris_update_lut_payload(int32_t ip, int32_t opt_id, struct dsi_panel_cmd_set *pcmds);

/*
*@Description: get current continue splash stage
				first light up panel only
				second pq effect
*/
uint8_t iris_get_cont_splash_type(void);

void iris_dump_packet(u8 *data, int size);
/*
*@Description: print continuous splash commands for bootloader
*@param: pcmd: cmds array  cnt: cmds cound
*/
void  iris_print_cmds(struct dsi_cmd_desc *pcmd, int cnt, int state);
void iris_csc_lut_check(const u8 *fw_data);

int iris_read_chip_id(void);
void iris_read_power_mode(struct dsi_panel *panel);
void iris_read_mcf_lgd(struct dsi_panel *panel, int index, u32 size, u32 *pvalues);
void iris_read_mcf_tianma(struct dsi_panel *panel, int address, u32 size, u32 *pvalues);

#endif // _DSI_IRIS3_LIGHTUP_H_
