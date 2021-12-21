/* Copyright (C) 2019 Tcl Corporation Limited */
#include <drm/drm_mipi_dsi.h>
#include <video/mipi_display.h>
#include "dsi_ctrl_reg.h"
#include "dsi_hw.h"
#include "dsi_iris3_api.h"
#include "dsi_iris3_lightup.h"
#include "dsi_iris3_lightup_ocp.h"
#include "dsi_iris3_lp.h"
#include "dsi_iris3_pq.h"
#include "dsi_iris3_ioctl.h"
#include "dsi_iris3_log.h"
#include "dsi_iris3_i2c.h"

#define DEBUG false
// #undef IRIS_LOGI
// #define IRIS_LOGI IRIS_LOGE
// #undef IRIS_LOGD
// #define IRIS_LOGD IRIS_LOGE
#define IRIS_CHIP_VER_0   0
#define IRIS_CHIP_VER_1   1
#define IRIS_OCP_DIRECT_BUS  (0x0001000C)
#define IRIS_OCP_HEADER_ADDR_LEN  8

#define DIRECT_DBC_TYPE			0x0020000c
#define DIRECT_CM_TYPE			0x0008000c
#define DIRECT_SDR2HDR_TYPE		0x0004000c
#define DIRECT_SCALER1D_TYPE	0x0010000c

static uint32_t sacler1d_addr_map[6][2] = {
	{ 0x00000000, 0xF1A31400 },
	{ 0x00000100, 0xF1A31800 },
	{ 0x00000200, 0xF1A31000 },
	{ 0x00000300, 0xF1A31200 },
	{ 0x00000400, 0xF1A31600 },
	{ 0x00000500, 0xF1A31A00 }
};

static uint32_t dbc_addr_map[6][2] = {
	{ 0x00000800, 0xF0550800 },
	{ 0x00000C00, 0xF0550800 },
	{ 0x00001000, 0xF0550C00 },
	{ 0x00001400, 0xF0550C00 },
	{ 0x00001800, 0xF0550000 },
	{ 0x00001C00, 0xF0550400 }
};

static struct iris_cfg  gcfg;
struct iris_setting_info iris_setting;
u8 iris_dbc_lut_index = 0;
u8 iris_sdr2hdr_mode = 0;
uint8_t iris_pq_update_path = PATH_DSI;
uint8_t iris_lightup_path = PATH_DSI;
static uint8_t g_cont_splash_type = IRIS_CONT_SPLASH_NONE;
enum {
	DSI_CMD_ONE_LAST_FOR_MULT_IPOPT = 0,
	DSI_CMD_ONE_LAST_FOR_PKT,
	DSI_CMD_ONE_LAST_FOR_MAX_PKT = 1000,
};

u8 panel_mcf_data[128] = {0}; // MODIFIED by hongwei.tian, 2019-06-06,BUG-7808648

static int iris_cont_splash_debugfs_init(struct dsi_display *display);

void iris3_init(struct dsi_display *display, struct dsi_panel *panel)
{
	struct iris_cfg * pcfg;

	IRIS_LOGD("%s:%d\n", __func__, __LINE__);

	pcfg = iris_get_cfg();
	pcfg->display = display;
	pcfg->panel = panel;
	pcfg->iris3_i2c_read = NULL;
	pcfg->iris3_i2c_write = NULL;
	pcfg->chip_id = IRIS_CHIP_VER_1;
#if defined(IRIS3_ABYP_LIGHTUP)
	iris_abyp_flag_set(ANALOG_BYPASS_MODE);
#else
	iris_abyp_flag_set(display->cmdline_iris_mode);
#endif
	pcfg->abypss_ctrl.pending_mode = MAX_MODE;
	mutex_init(&pcfg->abypss_ctrl.abypass_mutex);
	iris_lp_debugfs_init(display);
	iris_pq_debugfs_init(display);
	iris_cont_splash_debugfs_init(display);
	iris_adb_type_debugfs_init(display);
	mutex_init(&pcfg->gs_mutex);
}

struct iris_cfg * iris_get_cfg(void)
{
	return &gcfg;
}

int iris3_abypass_mode_get(void)
{
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();
	return pcfg->abypss_ctrl.abypass_mode;
}

uint8_t iris_get_cont_splash_type(void)
{
	return g_cont_splash_type;
}

struct iris_ctrl_seq * iris_get_ctrl_seq_addr(struct iris_ctrl_seq *base, uint8_t chip_id)
{
	struct iris_ctrl_seq *pseq = NULL;
	struct iris_cfg * pcfg = iris_get_cfg();

	switch(chip_id) {
		case IRIS_CHIP_VER_0:
			pseq = base;
			break;
		case IRIS_CHIP_VER_1:
			pseq = base + 1;
			break;
		default:
			while(iris_read_chip_id() != 1) {
				iris_pt_to_abypass_switch();
				iris_abypass_to_pt_switch(pcfg->display, true, 1);
			}
			pseq = base + 1;
			break;
	}
	return pseq;
}

struct iris_ctrl_seq * iris_get_ctrl_seq_common(struct iris_cfg *pcfg, int32_t type)
{
	struct iris_ctrl_seq *pseq = NULL;

	if (type == IRIS_CONT_SPLASH_NONE) {
		pseq = iris_get_ctrl_seq_addr(pcfg->ctrl_seq, pcfg->chip_id);
	} else if (type == IRIS_CONT_SPLASH_LK) {
		pseq = iris_get_ctrl_seq_addr(pcfg->ctrl_seq_cs, pcfg->chip_id);
	}
	return pseq;
}

struct iris_ctrl_seq * iris_get_ctrl_seq(struct iris_cfg *pcfg)
{
	return iris_get_ctrl_seq_common(pcfg, IRIS_CONT_SPLASH_NONE);
}

struct iris_ctrl_seq * iris_get_ctrl_seq_cs(struct iris_cfg * pcfg)
{
	return iris_get_ctrl_seq_common(pcfg, IRIS_CONT_SPLASH_LK);
}


static uint32_t iris_get_ocp_type(const uint8_t * payload)
{
	uint32_t * pval = NULL;

	pval  = (uint32_t *)payload;
	return cpu_to_le32(pval[0]);
}

static uint32_t iris_get_ocp_base_addr(const uint8_t *payload)
{
	uint32_t * pval = NULL;

	pval  = (uint32_t *)payload;
	return cpu_to_le32(pval[1]);
}

static void iris_set_ocp_type(const uint8_t * payload, uint32_t val)
{
	uint32_t * pval = NULL;

	pval  = (uint32_t *)payload;
	pval[0] = val;
}

static void iris_set_ocp_base_addr(const uint8_t *payload, uint32_t val)
{
	uint32_t * pval = NULL;

	pval  = (uint32_t *)payload;
	pval[1] = val;
}

static bool iris_is_direct_bus(const uint8_t * payload)
{
#if 0
	if (iris_get_ocp_type(payload) == IRIS_OCP_DIRECT_BUS) {
		return true;
	}

	return false;
#else
	uint32_t val = 0;

	val = iris_get_ocp_type(payload);

	if ((val & 0xf) == 0x0c)
		return true;

	return false;
#endif
}

static int iris_split_mult_pkt(const uint8_t *payload, int len)
{
	uint32_t pkt_size = 0;
	int mult = 1;
	struct iris_cfg * pcfg = iris_get_cfg();

	if (! iris_is_direct_bus(payload))
		return mult;

	pkt_size = pcfg->split_pkt_size;

	if (len > pkt_size + IRIS_OCP_HEADER_ADDR_LEN)
		mult =  (len - IRIS_OCP_HEADER_ADDR_LEN + pkt_size - 1) / pkt_size;

	return mult;
}


void iris_set_lut_cnt(uint32_t cnt)
{
	struct iris_cfg * pcfg = NULL;

	pcfg = iris_get_cfg();

	pcfg->lut_cmds_cnt = cnt;
}


static void iris_set_cont_splash_type(uint8_t type)
{
	g_cont_splash_type = type;
}

static void iris_init_ip_index(struct iris_ip_index  *pip_index)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t cnt = 0;
	for(i = 0; i < IRIS_IP_CNT; i++) {
		cnt = pip_index[i].opt_cnt;
		for (j = 0; j < cnt; j++) {
			pip_index[i].opt[j].cmd = NULL;
			pip_index[i].opt[j].link_state = 0xff;
		}
	}
}


static int32_t iris_alloc_buf(struct iris_cfg *pcfg,
                                 const struct iris_cmd_statics *pcmd_statics)
{
	int i = 0;
	int j = 0;
	int opt_cnt = 0;
	struct dsi_panel_cmd_set *pcmds = NULL;
	struct iris_ip_index  *pip_index = NULL;

	/*create dsi cmds*/
	if (pcmd_statics->cnt == 0)
		return -EINVAL;

	pcmds = &(pcfg->cmds);
	pip_index = pcfg->ip_index_arr;

	pcmds->cmds = kzalloc(pcmd_statics->cnt * sizeof(struct dsi_cmd_desc),
					GFP_KERNEL);
	if (!pcmds->cmds) {
		IRIS_LOGE("can not kzalloc space for dsi\n");
		return -ENOMEM;
	}

	for (i = 0; i < IRIS_IP_CNT; i++) {
		opt_cnt = pip_index[i].opt_cnt;
		if ( opt_cnt != 0) {
			pip_index[i].opt = kzalloc(opt_cnt * sizeof(struct iris_ip_opt),
									GFP_KERNEL);
			if (! pip_index[i].opt) {
				//free already malloc space
				for(j = 0; j < i; j++) {
					kfree(pip_index[j].opt);
					pip_index[j].opt = NULL;
				}
				kfree(pcmds->cmds);
				pcmds->cmds = NULL;

				return -ENOMEM;;
			}
		}
	}
	return 0;
}

static uint32_t iris_change_cmd_hdr(struct iris_parsed_hdr * phdr, int len)
{
	struct iris_cfg *pcfg = NULL;
	pcfg = iris_get_cfg();

	if (pcfg->add_last_flag) {
		phdr->last = 1;
	} else {
		phdr->last = 0;
	}

	phdr->dlen = (len + IRIS_OCP_HEADER_ADDR_LEN);
	return phdr->dlen;
}


static int32_t iris_create_ip_index(struct iris_ip_index  *pip_index,
								struct dsi_cmd_desc *cmd, struct iris_parsed_hdr *hdr)
{
	uint8_t i = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	uint8_t cnt = 0;

	if (!hdr || !cmd || !pip_index)
		return -EINVAL;

	ip = hdr->ip & 0xff;
	opt_id = hdr->opt & 0xff;

	cnt = pip_index[ip].opt_cnt;

	for (i = 0; i < cnt; i++) {
		if (pip_index[ip].opt[i].cmd == NULL) {
			pip_index[ip].opt[i].cmd = cmd;
			pip_index[ip].opt[i].len = 1;
			pip_index[ip].opt[i].opt_id = opt_id;
			break;
		} else  if (pip_index[ip].opt[i].opt_id == opt_id){  /*find the right opt_id*/
			pip_index[ip].opt[i].len++;
			break;
		}
	}
	//to set link state
	if (pip_index[ip].opt[i].link_state == 0xff
				&& pip_index[ip].opt[i].opt_id == opt_id) {
		uint8_t link_state = 0;
		link_state = (hdr->opt >> 8) & 0xff;
		pip_index[ip].opt[i].link_state =
			link_state ? DSI_CMD_SET_STATE_LP : DSI_CMD_SET_STATE_HS;
	}

	if (i == cnt) {
		IRIS_LOGE("ip= %d opt = %d can not find right position\n", ip , opt_id);
		return -EINVAL;
	}
	return 0;
}

static int32_t iris_create_cmd_hdr(struct dsi_cmd_desc *cmd, struct iris_parsed_hdr * hdr)
{
	memset(cmd, 0x00, sizeof(struct dsi_cmd_desc));
	cmd->msg.type = (hdr->dtype & 0xff);
	cmd->post_wait_ms = (hdr->wait & 0xff);
	cmd->last_command = ((hdr->last & 0xff) != 0);
	cmd->msg.tx_len = (hdr->dlen & 0xffff);

	if (DEBUG) {
		IRIS_LOGE("cmd list: dtype = %0x wait = %0x last = %0x dlen = %zx\n",
		cmd->msg.type, cmd->post_wait_ms, cmd->last_command, cmd->msg.tx_len);
	}

	return cmd->msg.tx_len;
}

static void iris_modify_parsed_hdr(struct iris_parsed_hdr *dest, struct iris_parsed_hdr * src,
												int i, const int mult)
{
	int pkt_size = 0;
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();
	pkt_size = pcfg->split_pkt_size;

	memcpy(dest, src, sizeof(*src));
	if (i == mult - 1) {
		dest->dlen = src->dlen - (mult -1) * pkt_size;
	} else {
		iris_change_cmd_hdr(dest, pkt_size);
	}
}


static int iris_write_ip_opt( const uint8_t *payload, struct dsi_cmd_desc *cmd,
						struct iris_parsed_hdr *phdr, int mult)
{
	int i = 0;
	struct iris_cfg *pcfg = NULL;
	struct iris_ip_index  *pip_index = NULL;

	pcfg = iris_get_cfg();
	pip_index = pcfg->ip_index_arr;

	for (i = 0; i < mult; i++) {
		iris_create_ip_index(pip_index, cmd + i, phdr);
	}
	return 0;
}


static int iris_write_cmd_hdr(const uint8_t * payload,
					struct dsi_cmd_desc *cmd, struct iris_parsed_hdr *phdr, int mult)
{
	int i = 0;
	struct iris_parsed_hdr  tmp_hdr;

	mult = iris_split_mult_pkt(payload, phdr->dlen);

	for (i = 0; i < mult; i++) {
		iris_modify_parsed_hdr(&tmp_hdr, phdr, i, mult);

		//add cmds hdr information
		iris_create_cmd_hdr(cmd + i, &tmp_hdr);
	}
	return 0;
}


static void iris_create_cmd_payload(uint8_t *payload, const uint8_t *data , int32_t len)
{
	int32_t i = 0;
	uint32_t * pval = NULL;
	uint32_t cnt = 0;

	pval  = (uint32_t *)data;
	cnt = (len) >> 2;
	for (i = 0; i < cnt; i ++) {
		*(uint32_t *) (payload + (i << 2)) = cpu_to_le32(pval[i]);
	}
}


static int iris_write_cmd_payload(const char * buf,
					 struct dsi_cmd_desc * pdesc, int mult)
{
	int i = 0;
	uint32_t dlen = 0;
	//uint32_t len = 0;
	uint32_t ocp_type = 0;
	uint32_t base_addr = 0;
	uint32_t pkt_size = 0;
	uint8_t * ptr = NULL;
	struct iris_cfg * pcfg = NULL;

	pcfg = iris_get_cfg();
	pkt_size = pcfg->split_pkt_size;

	ocp_type = iris_get_ocp_type(buf);
	base_addr = iris_get_ocp_base_addr(buf);

	if (mult == 1) {
		dlen = pdesc->msg.tx_len;
		ptr = (uint8_t *) kzalloc(dlen, GFP_KERNEL);
		if (!ptr) {
			IRIS_LOGE("can not malloc space \n");
			return -ENOMEM;
		}
		iris_create_cmd_payload(ptr, buf, dlen);

		pdesc->msg.tx_buf = ptr;
	} else {
		//remove header and base address
		buf += IRIS_OCP_HEADER_ADDR_LEN;

		for (i = 0; i < mult; i++) {
			dlen = pdesc[i].msg.tx_len; //(i != mult -1) ? pkt_size + IRIS_OCP_HEADER_ADDR_LEN : dlen - (mult-1 )* pkt_size;

			ptr =(uint8_t *) kzalloc(dlen , GFP_KERNEL);
			if (!ptr) {
				IRIS_LOGE("can not malloc space \n");
				return -ENOMEM;
			}

			iris_set_ocp_base_addr(ptr, base_addr + i * pkt_size);
			iris_set_ocp_type(ptr, ocp_type);

			iris_create_cmd_payload(ptr + IRIS_OCP_HEADER_ADDR_LEN, buf, dlen - IRIS_OCP_HEADER_ADDR_LEN);
			// add payload
			buf += (dlen - IRIS_OCP_HEADER_ADDR_LEN);

			pdesc[i].msg.tx_buf = ptr;
		}
	}

	if (DEBUG) {
		int len = 0;
		for (i = 0; i < mult; i++) {
			len = (pdesc[i].msg.tx_len > 16) ? 16 : pdesc[i].msg.tx_len;
			print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 4,
							  pdesc[i].msg.tx_buf, len, false);
		}
	}

	return 0;
}

static void iris_init_cmd_info(const uint8_t *buf, int32_t cnt)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t mult = 0;

	const uint8_t *payload = NULL;
	struct dsi_cmd_desc *pdesc = NULL;
	struct dsi_panel_cmd_set *pcmds = NULL;
	struct iris_parsed_hdr * hdr = NULL;
	struct iris_cfg *pcfg = NULL;
	struct iris_ip_index  *pip_index = NULL;

	pcfg = iris_get_cfg();
	pcmds = &(pcfg->cmds);
	pip_index = pcfg->ip_index_arr;

	iris_init_ip_index(pip_index);

	while (i < cnt) {
		hdr = (struct iris_parsed_hdr *)buf;
		pdesc = &(pcmds->cmds[i]);
		payload = buf + sizeof(*hdr);

		mult = iris_split_mult_pkt(payload, hdr->dlen);
		if (DEBUG) {
			if (mult > 1) {
				IRIS_LOGE("mult is = %d\n", mult);
			}
		}

		//need to first write desc header and then write payload
		iris_write_cmd_hdr(payload, pdesc, hdr , mult);
		iris_write_cmd_payload(payload, pdesc, mult);

		//write cmd link information
		iris_write_ip_opt(payload, pdesc, hdr, mult);

		buf  += sizeof(*hdr) + hdr->dlen;
		i += mult;
	}

	if (DEBUG) {
		for (i = 0; i < IRIS_IP_CNT; i++) {
			for (j = 0; j < pip_index[i].opt_cnt; j++) {
				IRIS_LOGE("ip = %02x opt_id = %02x cmd = %p  len = %0x  link_state = %0x \n",
					i, pip_index[i].opt[j].opt_id, pip_index[i].opt[j].cmd,
					pip_index[i].opt[j].len, pip_index[i].opt[j].link_state);
			}
		}
	}
}

static void iris_create_cmd_list(const uint8_t *buf, struct iris_cmd_statics *pcmd_statics)
{
	int32_t cnt = 0;
	int32_t rc = 0;
	struct dsi_panel_cmd_set *pcmds = NULL;
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();
	pcmds = &(pcfg->cmds);

	/*create dsi cmd list*/
	rc = iris_alloc_buf(pcfg, pcmd_statics);
	if (rc) {
		IRIS_LOGE("create dsi memory failure\n");
		return;
	}

	cnt = pcmds->count = pcmd_statics->cnt;

	iris_init_cmd_info(buf, cnt);
}

static int32_t iris_cmd_statics_cal(struct iris_parsed_hdr *hdr, const uint8_t * payload,
				struct iris_cmd_statics *pcmd_statics)
{
	int mult = 1;
	uint16_t pkt_size = 0;
	struct iris_cfg  * pcfg = NULL;
	if (!hdr || !pcmd_statics || !payload)
		return -EINVAL;

	pcfg = iris_get_cfg();
	pkt_size = pcfg->split_pkt_size;

	//change to uint8_t length
	hdr->dlen = (hdr->dlen << 2);

	mult = iris_split_mult_pkt(payload, hdr->dlen);

	/*it will split to mult dsi cmds
	add (mult-1) ocp_header(4 bytes) and ocp_type(4 bytes) */
	pcmd_statics->cnt += mult;
	//TODO: total len is not used
	pcmd_statics->len += sizeof(uint8_t)* ((mult-1)* IRIS_OCP_HEADER_ADDR_LEN + hdr->dlen);

	if (DEBUG) {
		IRIS_LOGE("dsi  cnt = %d len = %d\n", pcmd_statics->cnt, pcmd_statics->len);
	}
	return 0;
}

static int32_t iris_ip_opt_statics_cal(struct iris_parsed_hdr *hdr, struct iris_ip_index  *pip_index)
{
	uint8_t last = hdr->last & 0xff;
	uint8_t  ip = hdr->ip & 0xff;

	if (!hdr || !pip_index) {
		return -EINVAL;
	}
	if (last == 1) {
		pip_index[ip].opt_cnt ++;
	}

	return 0;
}

static int32_t iris_statics_cal(struct iris_parsed_hdr *hdr, const char * payload,
				struct iris_cmd_statics *pcmd_statics, struct iris_ip_index  *pip_index){
	int32_t rc = 0;

	rc = iris_cmd_statics_cal(hdr, payload, pcmd_statics);
	if (rc)
		goto EXIT_VAL;

	rc = iris_ip_opt_statics_cal(hdr, pip_index);
	if (rc)
		goto EXIT_VAL;

	return 0;

EXIT_VAL:

	IRIS_LOGE("cmd static is error\n");
	return rc;
}

static int32_t iris_verify_dtsi(struct iris_parsed_hdr *hdr) {
	uint32_t * pval = NULL;
	uint8_t  tmp = 0;
	if (hdr->ip >= IRIS_IP_CNT) {
		IRIS_LOGE("hdr->ip is  0x%0x which is out of max ip value\n", hdr->ip);
		return -EINVAL;
	} else if (((hdr->opt >> 8) & 0xff)  > 1) {
		IRIS_LOGE("hdr->opt link state is not right 0x%0x\n", hdr->opt);
		return -EINVAL;
	}

	switch (hdr->dtype) {
		case MIPI_DSI_DCS_SHORT_WRITE:
		case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
		case MIPI_DSI_DCS_READ:
		case MIPI_DSI_DCS_LONG_WRITE:
		case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
		case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
		case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
		case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
		case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
		case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
			break;
		case MIPI_DSI_GENERIC_LONG_WRITE:
			/*judge payload0 for iris header*/
			pval = (uint32_t *)hdr + (sizeof(*hdr) >> 2);
			tmp = *pval & 0x0f;
			if (tmp == 0x00 || tmp == 0x05 || tmp == 0x0c || tmp == 0x08) {
				break;
			} else if (tmp == 0x04) {
				if ((hdr->dlen -1) % 2 != 0) {
					IRIS_LOGE("dlen is not right = %d\n", hdr->dlen);
					return -EINVAL;
				}
			} else {
				IRIS_LOGE("payload hdr is not right = %0x\n", *pval);
				return -EINVAL;
			}
			break;
		default:
			IRIS_LOGE("dtype is not right %0x\n", hdr->dtype);
			return -EINVAL;
	}
	return 0;
}


static int32_t iris_parse_panel_type(struct device_node *np, struct iris_cfg * pcfg)
{
	const char *data;
	uint32_t value = 0;
	int32_t rc = 0;

	data = of_get_property(np, "pxlw,panel-type", NULL);
	if (data) {
		if (!strcmp(data, "PANEL_LCD_SRGB")) {
			pcfg->panel_type = PANEL_LCD_SRGB;
		} else if (!strcmp(data, "PANEL_LCD_P3")) {
			pcfg->panel_type = PANEL_LCD_P3;
		} else if (!strcmp(data, "PANEL_OLED")) {
			pcfg->panel_type = PANEL_OLED;
		} else {
			pcfg->panel_type = PANEL_LCD_SRGB;   //default value is 0
		}
	} else {
		pcfg->panel_type = PANEL_LCD_SRGB;   //default value is 0
		IRIS_LOGD("parse panel type failed\n");
	}

	rc = of_property_read_u32(np, "pxlw,panel-dimming-brightness", &value);
	if (rc == 0) {
		pcfg->panel_dimming_brightness = value;
	} else {
		/* for V30 panel, 255 may cause panel during exit HDR, and lost TE.*/
		pcfg->panel_dimming_brightness = 250;
		IRIS_LOGD("parse panel dimming brightness failed\n");
		rc = 0;
	}

	return rc;
}

static int32_t iris_parse_lut_mode(
		struct device_node *np, struct iris_cfg *pcfg)
{
	const char *data;
	int32_t rc = 0;

	data = of_get_property(np, "pxlw,lut-mode", NULL);
	if (data) {
		if (!strcmp(data, "single"))
			pcfg->lut_mode = SINGLE_MODE;
		else if (!strcmp(data, "interpolation"))
			pcfg->lut_mode = INTERPOLATION_MODE;
		else/*default value is 0*/
			pcfg->lut_mode = INTERPOLATION_MODE;
	} else { /*default value is 0*/
		pcfg->lut_mode = INTERPOLATION_MODE;
		IRIS_LOGE("parse lut mode failed");
		rc =  -EINVAL;
	}
	IRIS_LOGD("pxlw,lut-mode: %d\n", pcfg->lut_mode);
	return rc;
}

static int32_t iris_parse_lp_control(struct device_node *np, struct iris_cfg * pcfg)
{
	int32_t rc = 0;

	pcfg->dynamic_power = of_property_read_bool(np, "pxlw,dynamic-power");
	IRIS_LOGD("pxlw,dynamic-power: %i\n", pcfg->dynamic_power);

	return rc;
}

static int32_t iris_parse_split_pkt_info(struct device_node *np, struct iris_cfg * pcfg)
{
	int32_t rc = 0;

	rc = of_property_read_u32(np, "pxlw,pkt-payload-size", &(pcfg->split_pkt_size));
	if (rc) {
		IRIS_LOGE("can not get property: pxlw,pkt-payload-size\n");
		return rc;
	}
	IRIS_LOGD("pxlw,split-pkt-payload-size: %d\n", pcfg->split_pkt_size);

	rc = of_property_read_u32(np, "pxlw,last-for-per-pkt", &(pcfg->add_last_flag));
	if (rc) {
		IRIS_LOGE("can not get property:pxlw,last-for-per-pkt \n");
		return rc;
	}
	IRIS_LOGD("pxlw,add-last-for-splitted-pkt: %d\n", pcfg->add_last_flag);
	pcfg->add_on_last_flag = pcfg->add_last_flag;

	rc = of_property_read_u32(np, "pxlw,cont-last-for-per-pkt",
			&(pcfg->add_cont_last_flag));
	if (rc) {
		pcfg->add_cont_last_flag = 2;
		rc = 0;
	}
	IRIS_LOGD("pxlw,add-last-for-splitted-pkt: %d, %d\n", pcfg->add_last_flag, pcfg->add_cont_last_flag);

	rc = of_property_read_u32(np, "pxlw,pt-last-for-per-pkt",
			&(pcfg->add_pt_last_flag));
	if (rc) {
		pcfg->add_pt_last_flag = 2;
		rc = 0;
	}
	IRIS_LOGD("pxlw,add-last-for-splitted-pkt: %d, %d, %d\n", pcfg->add_on_last_flag, pcfg->add_cont_last_flag, pcfg->add_pt_last_flag);


	return rc;
}

static int32_t iris_parse_color_temp_info(struct device_node *np, struct iris_cfg * pcfg)
{
	int32_t rc = 0;

	rc = of_property_read_u32(np, "pxlw,min-color-temp", &(pcfg->min_color_temp));
	if (rc) {
		IRIS_LOGE("can not get property: pxlw,min-color-temp\n");
		return rc;
	}
	IRIS_LOGD("pxlw,min-color-temp: %d\n", pcfg->min_color_temp);

	rc = of_property_read_u32(np, "pxlw,max-color-temp", &(pcfg->max_color_temp));
	if (rc) {
		IRIS_LOGE("can not get property:pxlw,max-color-temp \n");
		return rc;
	}
	IRIS_LOGD("pxlw,max-color-temp: %d\n", pcfg->max_color_temp);

	return rc;
}

static int32_t iris_parse_cmd_list(struct device_node *np, struct iris_cfg * pcfg) {
	int32_t len = 0;
	int32_t blen = 0;
	int32_t rc = 0;
	const char *data;
	uint8_t *ptr = NULL;
	char * bp = NULL;
	uint32_t *buf = NULL;
	struct iris_cmd_statics cmd_statics;
	struct iris_parsed_hdr *hdr;
	char * key = "pxlw,iris-cmd-list";
	struct dsi_panel_cmd_set *pcmds = NULL;
	struct iris_ip_index  *pip_index;
	int i = 0;

	pcmds = &(pcfg->cmds);
	pip_index = pcfg->ip_index_arr;

	memset(&cmd_statics, 0x00, sizeof(cmd_statics));
	for (i = 0; i < IRIS_IP_CNT; i++)
		memset(&pip_index[i], 0x00, sizeof(pip_index[0]) );

	data = of_get_property(np, key,&blen);
	if (!data) {
		IRIS_LOGE("%s: failed, pxlw,iris-cmd-list\n", __func__);
		return -EINVAL;
	}

	if (blen %4 != 0) {
		IRIS_LOGE("the len of the array lenght = %d is not multpile of 4\n", blen);
		return -EINVAL;
	}

	buf = (uint32_t *)kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf) {
		IRIS_LOGE("can not kzalloc memory\n");
		rc = -ENOMEM;
		goto exit_free;
	}

	rc = of_property_read_u32_array(np, key, buf, blen >> 2);
	if (rc != 0) {
		IRIS_LOGE("read array is not right\n");
		rc = -EINVAL;
		goto exit_free;
	}

	/* scan dcs commands */
	bp = (uint8_t *)buf;
	len = blen;
	while (len >= sizeof(*hdr)) {
		hdr = (struct iris_parsed_hdr *)bp;
		if (hdr->dlen > len) {
			IRIS_LOGE("%s: dtsi cmd=%d error, len=%d",
				__func__, hdr->dtype, hdr->dlen);
			rc = -EINVAL;
			goto exit_free;
		}
		bp += sizeof(*hdr);
		len -= sizeof(*hdr);

		if (DEBUG) {
			rc = iris_verify_dtsi(hdr);
			if (rc) {
				IRIS_LOGE("dtsi is error \n");
				IRIS_LOGE("hdr infor is %d %d %d %d %d %d\n",
					hdr->dtype,hdr->last,
					hdr->wait, hdr->ip, hdr->opt, hdr->dlen);
				rc = -EINVAL;
				goto exit_free;
			}
		}
		ptr = bp;
		bp += hdr->dlen * sizeof(uint32_t);
		len -= hdr->dlen * sizeof(uint32_t);

		if (DEBUG)
			IRIS_LOGE("hdr infor is %02x %02x %02x %02x %04x %02x\n",
				hdr->dtype,hdr->last,
				hdr->wait, hdr->ip, hdr->opt, hdr->dlen );

		iris_statics_cal(hdr, ptr, &cmd_statics, pip_index);

	}

	if (len != 0) {
		IRIS_LOGE("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		rc = -EINVAL;
		goto exit_free;
	}

	iris_create_cmd_list((uint8_t *)buf, &cmd_statics);

exit_free:
	kfree(buf);
	buf = NULL;
	return rc;
}

static void iris_add_cmd_seq_common(struct iris_ctrl_opt *ctrl_opt,
													int len, const uint8_t *pdata)
{
	int32_t i = 0;
	int32_t span = 3;
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	uint8_t skip_last = 0;

	for (i = 0; i < len; i++) {
		ip = pdata[span *i];
		opt_id = pdata[span * i + 1];
		skip_last = pdata[span * i + 2];

		ctrl_opt[i].ip = ip & 0xff;
		ctrl_opt[i].opt_id = opt_id & 0xff;
		ctrl_opt[i].skip_last = skip_last & 0xff;

		if (DEBUG) {
			IRIS_LOGE("ip = %d opt = %d  skip=%d\n", ip, opt_id, skip_last);
		}
	}
}

static int32_t iris_alloc_cmd_seq_common(struct iris_ctrl_seq  * pctrl_seq, int32_t seq_cnt)
{
	pctrl_seq->ctrl_opt = kmalloc(seq_cnt * sizeof(struct iris_ctrl_seq),
						GFP_KERNEL);
	if (pctrl_seq->ctrl_opt == NULL) {
		IRIS_LOGE("can not malloc space for pctrl opt\n");
		return -ENOMEM;
	}
	pctrl_seq->cnt = seq_cnt;

	return 0;
}

static int32_t iris_parse_cmd_seq_data(struct device_node * np,
			const uint8_t * key, const uint8_t **pval)
{
	const uint8_t *pdata = NULL;
	int32_t blen = 0;
	int32_t seq_cnt = 0;
	int32_t span = 3;

	pdata = of_get_property(np, key, &blen);
	if (!pdata) {
		IRIS_LOGE("%s %s is error\n", __func__, key);
		return -EINVAL;
	}

	seq_cnt =  (blen / span);
	if (blen == 0 || blen != span * seq_cnt) {
		IRIS_LOGE("parse %s len is not right = %d\n", key, blen);
		return -EINVAL;
	}

	*pval = pdata;

	return seq_cnt;
}


static int32_t iris_parse_cmd_seq_common(struct device_node * np,
			const uint8_t * pre_key, const uint8_t *key, struct iris_ctrl_seq  * pctrl_seq)
{
	int32_t pre_len = 0;
	int32_t len = 0;
	int32_t sum = 0;
	int32_t rc = 0;
	const uint8_t *pdata = NULL;
	const uint8_t *pre_pdata = NULL;

	pre_len = iris_parse_cmd_seq_data(np, pre_key, &pre_pdata);
	if (pre_len <= 0)
		return -EINVAL;

	len = iris_parse_cmd_seq_data(np, key, &pdata);
	if (len <= 0)
		return -EINVAL;

	sum = pre_len + len;

	rc = iris_alloc_cmd_seq_common(pctrl_seq, sum);
	if (rc != 0)
		return rc;

	iris_add_cmd_seq_common(pctrl_seq->ctrl_opt, pre_len, pre_pdata);
	iris_add_cmd_seq_common(&pctrl_seq->ctrl_opt[pre_len], len, pdata);

	return rc;
}


static int32_t iris_parse_cmd_seq(struct device_node * np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	uint8_t * pre0_key = "pxlw,iris-lightup-sequence-pre0";
	uint8_t * pre1_key = "pxlw,iris-lightup-sequence-pre1";
	uint8_t * key = "pxlw,iris-lightup-sequence";

	rc = iris_parse_cmd_seq_common(np, pre0_key, key, pcfg->ctrl_seq);
	if (rc != 0)
		return rc;

	return iris_parse_cmd_seq_common(np, pre1_key, key, pcfg->ctrl_seq + 1);
}

/*use for debug cont-splash lk part*/
static int32_t iris_parse_cmd_seq_cont_splash(struct device_node * np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	uint8_t * pre0_key = "pxlw,iris-lightup-sequence-pre0";
	uint8_t * pre1_key = "pxlw,iris-lightup-sequence-pre1";
	uint8_t * key = "pxlw,iris-lightup-sequence-cont-splash";

	rc = iris_parse_cmd_seq_common(np, pre0_key, key, pcfg->ctrl_seq_cs);
	if (rc != 0)
		return rc;

	return iris_parse_cmd_seq_common(np, pre1_key, key, pcfg->ctrl_seq_cs + 1);
}

static int32_t iris_parse_tx_mode(
		struct device_node *np,
		struct dsi_panel *panel,
		struct iris_cfg *pcfg)
{
	pcfg->rx_mode = panel->panel_mode;
	pcfg->tx_mode = panel->panel_mode;

	return 0;
}

static int32_t iris_ip_statics_cal(const uint8_t *data, int32_t len, int32_t *pval)
{
	int tmp = 0;

	int i = 0;
	int j = 0;

	if (data == NULL || len == 0 || pval == NULL) {
		IRIS_LOGE("data is null or len = %d\n", len);
		return -EINVAL;
	}

	tmp = data[0];
	len = len >> 1;

	for (i = 0; i < len; i++) {
		if (tmp == data[2 * i]) {
			pval[j]++;
		} else {
			tmp = data[2 * i];
			j++;
			pval[j]++;
		}
	}

	/*j begin from 0*/
	return (j +1);

}


static int32_t iris_alloc_pq_init_space(struct iris_cfg *pcfg, const uint8_t *pdata, int32_t blen)
{
	int32_t i = 0;
	int32_t ip_cnt = 0;
	int32_t rc = 0;
	int32_t *ptr = NULL;
	struct iris_pq_init_val * pinit_val = NULL;

	pinit_val = &(pcfg->pq_init_val);

	if (pdata == NULL || blen == 0) {
		IRIS_LOGE("pdata is %p, blen = %0x\n", pdata, blen);
		return -EINVAL;
	}

	ptr = (int32_t *)kmalloc(sizeof(*ptr) * (blen >>1), GFP_KERNEL);
	if (ptr == NULL) {
		IRIS_LOGE("can not malloc space for ptr\n");
		return -EINVAL;
	}
	memset(ptr, 0x00, sizeof(*ptr) *(blen >>1));

	ip_cnt = iris_ip_statics_cal(pdata, blen, ptr);
	if (ip_cnt <= 0) {
		IRIS_LOGE("can not static ip option\n");
		rc = -EINVAL;
		goto EXIT_FREE;
	}

	pinit_val->ip_cnt = ip_cnt;
	pinit_val->val = (struct iris_pq_ipopt_val *) kmalloc(sizeof(struct iris_pq_ipopt_val) * ip_cnt, GFP_KERNEL);
	if (pinit_val->val == NULL) {
		IRIS_LOGE("can not malloc pinit_val->val\n");
		rc = -EINVAL;
		goto EXIT_FREE;
	}

	for (i = 0; i < ip_cnt; i++) {
		pinit_val->val[i].opt_cnt = ptr[i];
		pinit_val->val[i].popt = kmalloc(sizeof(uint8_t) * ptr[i], GFP_KERNEL);
	}

EXIT_FREE:
	kfree(ptr);
	ptr = NULL;

	return rc;

}

static int32_t iris_parse_pq_default_params(struct device_node * np, struct iris_cfg *pcfg)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	int32_t blen = 0;
	int32_t rc = 0;

	uint8_t * key = "pxlw,iris-pq-default-val";
	const uint8_t *pdata = NULL;
	struct iris_pq_init_val * pinit_val = NULL;

	pinit_val = &(pcfg->pq_init_val);

	pdata = of_get_property(np, key, &blen);
	if (!pdata) {
		IRIS_LOGE("%s pxlw,iris-pq-default-val fail\n", __func__);
		return -EINVAL;
	}

	rc = iris_alloc_pq_init_space(pcfg, pdata, blen);
	if (rc) {
		IRIS_LOGE("malloc error \n");
		return rc;
	}

	for (i = 0; i < pinit_val->ip_cnt; i++) {
		struct iris_pq_ipopt_val  * pval = &(pinit_val->val[i]);
		pval->ip = pdata[k++];

		for (j = 0; j < pval->opt_cnt; j++) {
			pval->popt[j] = pdata[k];
				k += 2;
		}
		/*need to skip one*/
		k -= 1;
	}

	if (DEBUG) {
		IRIS_LOGE("ip_cnt = %0x\n", pinit_val->ip_cnt);
		for (i = 0; i < pinit_val->ip_cnt; i++) {
			char ptr[256];
			struct iris_pq_ipopt_val  * pval = &(pinit_val->val[i]);
			sprintf(ptr, "ip is %0x opt is ", pval->ip);
			for (j =0; j < pval->opt_cnt; j++) {
				sprintf(ptr + strlen(ptr), "%0x ", pval->popt[j]);
			}
			IRIS_LOGE("%s \n", ptr);
		}
	}
	return rc;
}
#if 0
static void __cont_splash_work_handler(struct work_struct *work)
{
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();

	do {
		iris_pt_to_abypass_switch();
		iris_abypass_to_pt_switch(pcfg->display, true, 1);
	} while (pcfg->chip_id != 1);
}
#endif

int iris3_parse_params(struct device_node *np, struct dsi_panel *panel)
{
	int32_t rc = 0;
	struct device_node *lightup_node = NULL;
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();
	memset(pcfg, 0x00, sizeof(*pcfg));

	if (!np) {
		IRIS_LOGE("the param is null\n");
		return -EINVAL;
	}

	lightup_node = of_find_node_by_name(np, "pxlw,mdss_iris_cfg");
	if (!lightup_node) {
		IRIS_LOGE("can not find the right node");
		return -EINVAL;
	}

	spin_lock_init(&pcfg->iris_lock);
	mutex_init(&pcfg->mutex);

	rc = iris_parse_split_pkt_info(lightup_node, pcfg);
	if (rc) {
		//use 64 split packet and do not add last for every packet.
		pcfg->split_pkt_size = 64;
		pcfg->add_last_flag = 0;
	}

	rc = iris_parse_color_temp_info(lightup_node, pcfg);
	if (rc) {
		/*use 2500K~7500K if do not define in dtsi*/
		pcfg->min_color_temp= 2500;
		pcfg->max_color_temp= 7500;
	}

	rc = iris_parse_cmd_list(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse cmd list error");
		return -EINVAL;
	}

	rc = iris_parse_cmd_seq(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse cmd seq error\n");
		return -EINVAL;
	}

	rc = iris_parse_cmd_seq_cont_splash(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse cont splash cmd seq error\n");
		return -EINVAL;
	}

	rc = iris_parse_pq_default_params(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse pq init error\n");
		return -EINVAL;
	}

	rc = iris_parse_panel_type(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse panel type error\n");
		return -EINVAL;
	}

	rc = iris_parse_lut_mode(lightup_node, pcfg);

	rc = iris_parse_lp_control(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse low power control error\n");
		return -EINVAL;
	}

	iris_parse_tx_mode(lightup_node, panel, pcfg);

	//INIT_WORK(&pcfg->cont_splash_work, __cont_splash_work_handler);
	init_completion(&pcfg->frame_ready_completion);

	pcfg->valid = IRIS_STATUS_INIT;
	return 0;
}


struct iris_pq_ipopt_val  *  iris_get_cur_ipopt_val(uint8_t ip)
{
	int i = 0;
	struct iris_cfg *pcfg = NULL;
	struct iris_pq_init_val * pinit_val = NULL;

	pcfg = iris_get_cfg();
	pinit_val = &(pcfg->pq_init_val);

	for (i = 0; i < pinit_val->ip_cnt; i++) {
		struct iris_pq_ipopt_val  *pq_ipopt_val = pinit_val->val + i;
		if (ip ==  pq_ipopt_val->ip) {
			return pq_ipopt_val;
		}
	}
	return NULL;
}


void iris_out_cmds_buf_reset(void)
{
	//reset buf
	int sum = 0;
	struct iris_cfg * pcfg;
	pcfg = iris_get_cfg();

	sum = pcfg->none_lut_cmds_cnt  + pcfg->lut_cmds_cnt;
	memset(pcfg->iris_cmds.iris_cmds_buf, 0x00,
			sum * sizeof(struct dsi_cmd_desc));
	pcfg->iris_cmds.cmds_index = 0;
}


int32_t iris_dsi_get_cmd_comp(struct iris_cfg * pcfg, int32_t ip,
					int32_t opt_index, struct iris_cmd_priv *pcmd_priv)
{
	uint8_t opt_id = 0;
	int32_t cnt = 0;
	int32_t i = 0;
	int32_t j = 0;
	struct iris_ip_opt * opt= NULL;
	struct dsi_panel_cmd_set *pcmds = NULL;
	struct iris_ip_index  *pip_index = NULL;
	struct iris_cmd_comp  *pcmd_comp = NULL;

	pcmds = &(pcfg->cmds);
	pip_index = pcfg->ip_index_arr;

	pcmd_comp = pcmd_priv->cmd_comp;
	if (ip >= IRIS_IP_CNT) {
		IRIS_LOGE("the index is not right opt ip = %d\n", ip);
		return -EINVAL;
	}

	cnt = pip_index[ip].opt_cnt;
	for (i = 0; i < cnt; i++) {
		opt = pip_index[ip].opt + i;
		opt_id = opt->opt_id;

		if (opt_id == opt_index) {
			pcmd_comp[j].cmd = opt->cmd;
			pcmd_comp[j].cnt = opt->len;
			pcmd_comp[j].link_state = opt->link_state;
			j++;
		}
	}
	if (DEBUG) {
		if (j == 0) {
			IRIS_LOGE("could not find the cmd opt");
			return -EINVAL;
		} else if (j > IRIS_IP_OPT_CNT) {
			IRIS_LOGE("cmd len %dis long then IRIS_IP_OPT_CNT\n", j);
			return -EINVAL;
		}
	}
	pcmd_priv->cmd_num = j;
	return 0;
}

void iris_dump_packet(u8 *data, int size)
{
	if (DEBUG)
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 4,
							  data, size, false);
}

void  iris_print_cmds(struct dsi_cmd_desc *p, int num, int state)
{
	int i = 0;
	int j = 0;
	int len = 0;
	int dlen = 0;
	uint8_t *arr = NULL;
	uint8_t *ptr = NULL;
	uint8_t *ptr_tx = NULL;
	struct dsi_cmd_desc *pcmd = NULL;

	IRIS_LOGD("%s cmd len= %d, state= %s\n",__func__, num, (state == DSI_CMD_SET_STATE_HS) ? "high speed" : "low power");

	for (i = 0; i < num; i++) {
		pcmd = p + i ;

		dlen = pcmd->msg.tx_len;

		len = 3 * dlen + 23; //3* 7(dchdr) + 1(\n) + 1 (0)
		arr = (uint8_t *)kmalloc(len * sizeof(uint8_t), GFP_KERNEL);
		if (! arr) {
			IRIS_LOGE("the malloc is error\n");
			return;
		}
		memset(arr, 0x00, sizeof(uint8_t)* len);

		ptr = arr;
		ptr_tx = (uint8_t *) pcmd->msg.tx_buf;
		len = sprintf(ptr, " %02X", pcmd->msg.type);
		ptr += len;
		for (j = 0; j < dlen; j++) {
			len = sprintf(ptr, " %02X", ptr_tx[j]);
			ptr += len;
		}
		//sprintf(ptr, "\\n\"");
		IRIS_LOGE("%s\n", arr);

		if (pcmd->post_wait_ms > 0) {
			IRIS_LOGE("\" FF %02X\\n\"\n", pcmd->post_wait_ms);
		}

		kfree(arr);
		arr = NULL;
	}
}

static int32_t iris_i2c_send_ocp_cmds(
						struct dsi_panel *panel,
						struct iris_cmd_comp *pcmd_comp)
{
	int i = 0;
	int j = 0;
	int ret = 0;
	bool is_burst;
	bool is_allburst;
	int len = 0;
	uint32_t * payload = NULL;
	uint32_t header = 0;
	uint32_t index = 0;
	uint32_t offset = 0;
	uint32_t lut_addr = 0;
	uint32_t pi_addr = 0;
	uint32_t dbc_data_buf[2] = {0};
	struct iris_i2c_msg * msg = NULL;
	uint32_t iris_i2c_msg_num = 0;

	is_allburst = true;
	for (i = 0; i < pcmd_comp->cnt; i++) {
		is_burst = iris_is_direct_bus(pcmd_comp->cmd[i].msg.tx_buf);
		if (is_burst == false) {
			is_allburst = false;
			break;
		} else {
			header = *(uint32_t *)(pcmd_comp->cmd[i].msg.tx_buf);
			if (header == DIRECT_DBC_TYPE) {
				is_allburst = false;
				break;
			}
		}
	}

	if (is_allburst == false) {
		for (i = 0; i < pcmd_comp->cnt; i++) {
			header = *(uint32_t *)(pcmd_comp->cmd[i].msg.tx_buf);
			payload = (uint32_t *)(pcmd_comp->cmd[i].msg.tx_buf) + 1;
			len = (pcmd_comp->cmd[i].msg.tx_len >> 2) - 1;
			is_burst = iris_is_direct_bus(pcmd_comp->cmd[i].msg.tx_buf);

			if (is_burst) {
				if (header == DIRECT_DBC_TYPE) {
					lut_addr = *payload;
					index = (lut_addr - 0x800) / 0x400;
					offset = lut_addr - (index * 0x400 + 0x800);
					if (( index == 4 ) || ( index == 5 )) {
						pi_addr = dbc_addr_map[index][1] + offset;
						for (j = 0; j < (len - 1); j++) {
							dbc_data_buf[0] = pi_addr + 4*j;
							dbc_data_buf[1] = payload[j + 1];
							iris_i2c_single_conver_ocp_write(dbc_data_buf, 1);
						}
					} else {
						pi_addr = dbc_addr_map[index][1] + offset*2 + (index % 2)*4;
						for (j = 0; j < (len - 1); j++) {
							dbc_data_buf[0] = pi_addr + 4*j*2;
							dbc_data_buf[1] = payload[j+1];
							iris_i2c_single_conver_ocp_write(dbc_data_buf, 1);
						}
					}
				} else if (header == DIRECT_CM_TYPE) {
					lut_addr = *payload;
					pi_addr = lut_addr + 0xF0560000;
					iris_i2c_burst_conver_ocp_write(pi_addr, payload, len-1);
				} else if (header == DIRECT_SDR2HDR_TYPE) {
					lut_addr = *payload;
					pi_addr = lut_addr + 0xF15D0000;
					iris_i2c_burst_conver_ocp_write(pi_addr, payload, len-1);
				} else if (header == DIRECT_SCALER1D_TYPE) {
					lut_addr = *payload;
					index = (lut_addr >> 8) & 0xF;
					offset = lut_addr & 0xFF;
					pi_addr = sacler1d_addr_map[index][1] + offset;
					iris_i2c_burst_conver_ocp_write(pi_addr, payload, len-1);
				} else {
					pi_addr = *payload;
					iris_i2c_burst_conver_ocp_write(pi_addr, payload, len-1);
				}
			} else {
				iris_i2c_single_conver_ocp_write(payload, len/2);
			}
			if (DEBUG) {
				IRIS_LOGE("%s,%d: i = %d, header = 0x%08x, len = %d, is_burst = %d \n", __func__, __LINE__, i, header, len, is_burst);
				for (j = 0; j < len; j++)
					IRIS_LOGE("0x%08x\n", *(payload + j));
			}

		}
		ret = 0;
	} else {
		iris_i2c_msg_num = pcmd_comp->cnt;
		msg = kmalloc(sizeof(struct iris_i2c_msg) * iris_i2c_msg_num + 1, GFP_KERNEL);
		if (NULL == msg) {
			IRIS_LOGE("[iris3] %s: allocate memory fails\n", __func__);
			return -EINVAL;
		}
		for (i = 0; i < iris_i2c_msg_num; i++) {
			msg[i].payload = (uint32_t *)(pcmd_comp->cmd[i].msg.tx_buf) + 1;
			msg[i].len = (pcmd_comp->cmd[i].msg.tx_len >> 2) - 1;
			header = *(uint32_t *)(pcmd_comp->cmd[i].msg.tx_buf);
			if (header == DIRECT_CM_TYPE) {
				lut_addr = *msg[i].payload;
				msg[i].base_addr = lut_addr + 0xF0560000;
			} else if (header == DIRECT_SDR2HDR_TYPE) {
				lut_addr = *msg[i].payload;
				msg[i].base_addr = lut_addr + 0xF15D0000;
			} else if (header == DIRECT_SCALER1D_TYPE) {
				lut_addr = *msg[i].payload;
				index = (lut_addr >> 8) & 0xF;
				offset = lut_addr & 0xFF;
				msg[i].base_addr = sacler1d_addr_map[index][1] + offset;
			} else {
				msg[i].base_addr = *msg[i].payload;
			}

			if (DEBUG) {
				IRIS_LOGE("%s,%d: i = %d, header = 0x%08x, len = %d, base_addr = 0x%08x \n", __func__, __LINE__, i, header, msg[i].len, msg[i].base_addr);
				for (j = 0; j < msg[i].len; j++)
					IRIS_LOGE("0x%08x\n", *(msg[i].payload + j));
			}
		}
		ret = iris_i2c_group_write(msg, iris_i2c_msg_num);
		kfree(msg);
	}
	return ret;
}


static int32_t iris_dsi_send_ocp_cmds(struct dsi_panel *panel,
									struct iris_cmd_comp *pcmd_comp)
{
	int ret;
	uint32_t wait = 0;
	struct dsi_cmd_desc *cmd = NULL;

	if (!pcmd_comp){
		IRIS_LOGE("cmd list is null\n");
		return -EINVAL;
	}

	/*use us than ms*/
	cmd = pcmd_comp->cmd + pcmd_comp->cnt -1;
	wait = cmd->post_wait_ms;
	if (wait) {
		cmd->post_wait_ms = 0;
	}

	if (DEBUG) {
		int i = 0;
		int len = 0;
		IRIS_LOGE("cmds cnt = %d\n", pcmd_comp->cnt);
		for (i = 0; i < pcmd_comp->cnt; i++) {
			len = pcmd_comp->cmd[i].msg.tx_len > 16 ?  16 : pcmd_comp->cmd[i].msg.tx_len;
			IRIS_LOGE("cmd list: dtype = %0x wait = %0x last = %0x dlen = %zx\n",
					pcmd_comp->cmd[i].msg.type, pcmd_comp->cmd[i].post_wait_ms,
					pcmd_comp->cmd[i].last_command, pcmd_comp->cmd[i].msg.tx_len);
			print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 4,
							  pcmd_comp->cmd[i].msg.tx_buf, len, false);
		}
	}

	ret = iris3_dsi_cmds_send(panel, pcmd_comp->cmd, pcmd_comp->cnt, pcmd_comp->link_state);
	if (wait != 0) {
		if (DEBUG)
			IRIS_LOGE("func wait = %x \n", wait);
		udelay(wait);
	}

	if (IRIS_CONT_SPLASH_LK == iris_get_cont_splash_type()) {
		if (wait != 0) {
			cmd->post_wait_ms = 1;
		}
		iris_print_cmds(pcmd_comp->cmd, pcmd_comp->cnt, pcmd_comp->link_state);
	}

	return ret;
}

static int32_t iris_send_ocp_cmds(
				struct dsi_panel *panel,
				struct iris_cmd_comp *pcmd_comp, uint8_t path)
{
	int ret = 0;

	if (DEBUG)
		IRIS_LOGE("%s,%d: path = %d\n",__func__,__LINE__,path);

	if (!pcmd_comp) {
		IRIS_LOGE("cmd list is null\n");
		return -EINVAL;
	}

	if (path == PATH_DSI)
		ret = iris_dsi_send_ocp_cmds(panel, pcmd_comp);
	else if (path == PATH_I2C)
		ret = iris_i2c_send_ocp_cmds(panel, pcmd_comp);
	else
		ret = -EINVAL;

	return ret;

}


int32_t iris_dump_cmd_payload(struct iris_cmd_priv  *pcmd_priv)
{
	int32_t cnt = 0;
	int32_t i = 0;
	int32_t j = 0;
	struct iris_cmd_comp *pcmd_comp = NULL;

	if (!pcmd_priv) {
		IRIS_LOGE("pcmd component is null \n");
		return -EINVAL;
	}

	cnt = pcmd_priv->cmd_num;
	for (i = 0; i < cnt; i++) {
		pcmd_comp = &(pcmd_priv->cmd_comp[i]);
		for (j = 0; j < pcmd_comp[i].cnt; j++) {
			iris_dump_packet((u8 *) pcmd_comp[i].cmd[j].msg.tx_buf,
							pcmd_comp[i].cmd[j].msg.tx_len);
		}
	}

	return 0;

}


int32_t  iris_dsi_send_cmds(struct dsi_panel *panel,
			struct iris_cmd_priv * pcmd_priv)
{
	uint32_t i = 0;

	for (i = 0; i < pcmd_priv->cmd_num; i++) {
		iris_send_ocp_cmds(panel, & pcmd_priv->cmd_comp[i], PATH_DSI);
	}

	return 0;
}

static void iris_send_cmd_to_panel(struct dsi_panel *panel,
										struct dsi_panel_cmd_set * cmds)
{
	if (!cmds || !cmds->count) {
		IRIS_LOGI("cmds = %p or cmd_cnt = 0\n", cmds);
		return;
	}

	iris3_panel_cmd_passthrough(panel, cmds);
}


void iris_send_ipopt_cmds(int32_t ip, int32_t opt_id)
{
	int32_t rc = 0;
	struct iris_cfg * pcfg;
	struct iris_cmd_priv  cmd_priv;

	pcfg = iris_get_cfg();

	rc = iris_dsi_get_cmd_comp(pcfg, ip, opt_id, &cmd_priv);
	if (rc) {
		IRIS_LOGE("%s can not find  in seq ip = %0x  opt_id = %0x \n",__func__, ip, opt_id);
		return;
	}
	mutex_lock(&pcfg->mutex);

	iris_dsi_send_cmds(pcfg->panel, &cmd_priv);

	mutex_unlock(&pcfg->mutex);
}



/**********************************************
* the API will only be called when suspend/resume and boot up.
*
***********************************************/
static void iris_send_specific_lut(uint8_t opt_id, int32_t skip_last)
{
	uint8_t lut_table = (opt_id >> 5) & 0x07;
	uint8_t lut_idx = opt_id & 0x1f;

	if(lut_table ==DBC_LUT) {
		if( lut_idx == DBC_INIT ) {
			//iris_dbc_income_set();
			iris_lut_send(lut_table, lut_idx, 1, !skip_last);
			iris_lut_send(lut_table, lut_idx, 0, !skip_last);
		} else {
			//iris_dbc_compenk_set(lut_idx);
			iris_lut_send(lut_table, lut_idx, 1, !skip_last);
			iris_lut_send(lut_table, lut_idx, 0, !skip_last);
		}
	} else if((lut_table != AMBINET_HDR_GAIN) && (lut_table != AMBINET_SDR2HDR_LUT))
		iris_lut_send(lut_table, lut_idx, 0, !skip_last);
}

static void iris_send_new_lut(uint8_t opt_id)
{
	uint8_t lut_table = (opt_id >> 5) & 0x07;
	uint8_t lut_idx = opt_id & 0x1f;

	// don't change the following three lines source code
	if(lut_table == DBC_LUT) {
		iris_lut_send(lut_table, lut_idx, iris_dbc_lut_index, 0);
	} else {
		iris_lut_send(lut_table, lut_idx, 0, 0);
	}
}

static void iris_init_cmds_buf(struct iris_cmd_comp *pcmd_comp, int32_t link_state)
{
	struct iris_cfg * pcfg;

	pcfg = iris_get_cfg();

	iris_out_cmds_buf_reset();

	memset(pcmd_comp, 0x00, sizeof(*pcmd_comp));
	pcmd_comp->cmd = pcfg->iris_cmds.iris_cmds_buf;
	pcmd_comp->link_state = link_state;
	pcmd_comp->cnt = pcfg->iris_cmds.cmds_index;
}

static void iris_remove_last_every_pkt(struct dsi_cmd_desc *pcmd, int cnt)
{
	int i = 0;
	for (i = 0; i < cnt; i++) {
		if (pcmd[i].last_command) {
			pcmd[i].last_command = false;
		}
	}
}


static void iris_add_last_every_pkt(struct dsi_cmd_desc *pcmd, int cnt)
{
	int i = 0;
	for (i = 0; i < cnt; i++) {
		if (!pcmd[i].last_command) {
			pcmd[i].last_command = true;
		}
	}
}

static void iris_add_last_pkt(struct dsi_cmd_desc * cmd, int cnt)
{
	iris_remove_last_every_pkt(cmd,  cnt);
	iris_add_last_every_pkt(cmd + cnt - 1, 1);
}

static void iris_add_last_multi_pkt(struct dsi_cmd_desc * cmd, int cnt)
{
	int i = 0;
	int num = 0;
	int surplus = 0;
	int span = 0;
	struct iris_cfg * pcfg = NULL;

	pcfg = iris_get_cfg();
	span = pcfg->add_last_flag;

	num = cnt / span;
	surplus = cnt  - num * span;

	for (i = 0; i < num; i++)
		iris_add_last_pkt(cmd + i*span, span);

	iris_add_last_pkt(cmd + i*span, surplus);
}

static int iris_set_pkt_last(struct dsi_cmd_desc * cmd, int cnt)
{
	int32_t ret = 0;
	int32_t add_last_flag = 0;
	struct iris_cfg * pcfg = NULL;


	pcfg = iris_get_cfg();
	add_last_flag = pcfg->add_last_flag;

	switch(add_last_flag) {
		case DSI_CMD_ONE_LAST_FOR_MULT_IPOPT:
			//only last packet has one last
			iris_add_last_pkt(cmd, cnt);
			break;
		case DSI_CMD_ONE_LAST_FOR_PKT:
			//every pkt has one last
			iris_add_last_every_pkt(cmd, cnt);
			break;
		default:
			//multiple pkts(add_last_flag) have one last
			iris_add_last_multi_pkt(cmd, cnt);
			break;
	}

	return ret;
}

static int iris_send_lut_table_pkt(struct iris_ctrl_opt *popt,
				struct iris_cmd_comp *pcomp, bool is_update, uint8_t path)
{
	uint8_t opt_id = 0;
	int32_t skip_last = 0;
	int32_t prev = 0;
	int32_t cur = 0;
	struct iris_cfg * pcfg = NULL;

	pcfg = iris_get_cfg();
	opt_id = popt->opt_id;
	skip_last = popt->skip_last;

	prev = pcomp->cnt;

	pcfg->iris_cmds.cmds_index = prev;
	if (is_update) {
		iris_send_new_lut(opt_id);
	} else {
		iris_send_specific_lut(opt_id, skip_last);
	}
	cur = pcfg->iris_cmds.cmds_index;

	if (cur == prev) {
		IRIS_LOGD("lut table is empty ip = %02x opt_id = %02x\n", popt->ip, opt_id);
		return 0;
	}

	pcomp->cnt = cur;

	if (!skip_last) {
		iris_set_pkt_last(pcomp->cmd, pcomp->cnt);
		iris_send_ocp_cmds(pcfg->panel, pcomp, path);
		iris_init_cmds_buf(pcomp, pcomp->link_state);
	}

	return 0;
}


static int iris_send_none_lut_table_pkt(struct iris_ctrl_opt *pip_opt,
				struct iris_cmd_comp *pcomp, uint8_t path)
{
	int i = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	int32_t flag = 0;
	int32_t prev = 0;
	int32_t cur = 0;
	int32_t skip_last = 0;
	int32_t add_last_flag = 0;
	int32_t rc = 0;
	struct iris_cfg * pcfg  = NULL;
	struct iris_cmd_priv  cmd_priv;
	struct iris_cmd_comp *pcomp_priv = NULL;

	pcfg = iris_get_cfg();

	ip = pip_opt->ip;
	opt_id = pip_opt->opt_id;
	skip_last = pip_opt->skip_last;
	add_last_flag = pcfg->add_last_flag;

	/*get single/multiple selection(s) according to option of ip*/
	rc = iris_dsi_get_cmd_comp(pcfg, ip, opt_id, &cmd_priv);
	if (rc) {
		IRIS_LOGE("lightup seq command can not find right ip = %0x  opt_id = %d\n", ip , opt_id);
		return -EINVAL;
	}

	pcomp_priv = cmd_priv.cmd_comp;

	if (pcomp->cnt == 0) {
		pcomp->link_state = pcomp_priv->link_state;
	} else if (pcomp_priv->link_state != pcomp->link_state){
		//send link state different packet.
		flag = 1;
	}

	if (flag == 0) {
		prev = pcomp->cnt;
		/*move single/multiples selection to one command*/
		for (i = 0; i < cmd_priv.cmd_num; i++) {
			memcpy(pcomp->cmd + pcomp->cnt, pcomp_priv[i].cmd,
					pcomp_priv[i].cnt * sizeof( * pcomp_priv[i].cmd));
			pcomp->cnt += pcomp_priv[i].cnt;
		}
		cur = pcomp->cnt;
	}

	/*if need to send or the last packet of sequence, it should send out to the MIPI*/
	if (!skip_last || flag == 1) {
		iris_set_pkt_last(pcomp->cmd, pcomp->cnt);
		iris_send_ocp_cmds(pcfg->panel, pcomp, path);
		iris_init_cmds_buf(pcomp, pcomp->link_state);
	}
	return 0;
}

static void iris_send_assembled_pkt(struct iris_ctrl_opt * arr, int len)
{
	int i = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	int32_t rc = -1;
	struct iris_cmd_comp cmd_comp;
	struct iris_cfg *pcfg;

	iris_init_cmds_buf(&cmd_comp, DSI_CMD_SET_STATE_HS);
	pcfg = iris_get_cfg();

	mutex_lock(&pcfg->mutex);
	for (i = 0; i < len; i++) {
		ip = arr[i].ip;
		opt_id = arr[i].opt_id;

		IRIS_LOGD("%s ip=%0x opt_id=%0x \n", __func__, ip, opt_id);

		/*lut table*/
		if (ip == IRIS_IP_EXT) {
			rc = iris_send_lut_table_pkt(arr + i, &cmd_comp, false, iris_lightup_path);
		} else {
			rc = iris_send_none_lut_table_pkt(arr + i, &cmd_comp, iris_lightup_path);
		}

		if (rc) {
			panic("%s error\n", __func__);
		}
	}
	mutex_unlock(&pcfg->mutex);
}

static void iris_send_lightup_pkt(void)
{
	struct iris_cfg * pcfg;
	struct iris_ctrl_seq *pseq = NULL;

	pcfg = iris_get_cfg();
	pseq = iris_get_ctrl_seq(pcfg);

	iris_send_assembled_pkt(pseq->ctrl_opt, pseq->cnt );
}


void iris_init_update_ipopt(struct iris_update_ipopt *popt,
						uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t skip_last)
{
	popt->ip = ip;
	popt->opt_old = opt_old;
	popt->opt_new = opt_new;
	popt->skip_last = skip_last;
}


int iris_init_update_ipopt_t(struct iris_update_ipopt *popt,  int len,
						uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t skip_last)
{
	int i  = 0;
	int cnt = 0;

	for (i = 0; i < len; i++ ){
		if (popt[i].ip == 0xff)
			break;
	}

	if (i >=  len) {
		IRIS_LOGE("there no empty space for install ip opt\n");
		return -1;
	}


	iris_init_update_ipopt(&popt[i],  ip, opt_old, opt_new, skip_last );
	cnt = i + 1;

	return cnt;
}


static void iris_cal_none_lut_cmds_cnt(void)
{
	int i = 0;
	int j = 0;
	int cnt = 0;
	int sum = 0;
	int rc = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;

	struct iris_cfg * pcfg  = NULL;
	struct iris_ctrl_seq *pseq = NULL;
	struct iris_cmd_priv  cmd_priv;
	struct iris_cmd_comp *pcomp_priv = NULL;
	struct iris_ctrl_opt *pctrl_opt = NULL;

	pcfg = iris_get_cfg();
	pseq = iris_get_ctrl_seq(pcfg);
	cnt = pseq->cnt;
	pctrl_opt = pseq->ctrl_opt;

	for (i = 0; i < cnt; i++) {
		ip = pctrl_opt[i].ip;
		opt_id = pctrl_opt[i].opt_id;

		if (IRIS_IP_EXT == ip) {
			continue;
		}

		/*get single/multiple selection(s) according to option of ip*/
		rc = iris_dsi_get_cmd_comp(pcfg, ip, opt_id, &cmd_priv);
		if (rc) {
			IRIS_LOGE("not find right ip = %0x  opt_id = %d\n", ip , opt_id);
			return;
		}

		pcomp_priv = cmd_priv.cmd_comp;

		for (j = 0; j < cmd_priv.cmd_num; j++) {
			sum += pcomp_priv[j].cnt;
		}
	}
	pcfg->none_lut_cmds_cnt = sum;

}

int iris_read_chip_id(void)
{
	struct iris_cfg * pcfg  = NULL;
	uint32_t sys_pll_ro_status = 0xf0000010;

	pcfg = iris_get_cfg();
	pcfg->chip_id = (iris_ocp_read(sys_pll_ro_status, DSI_CMD_SET_STATE_HS)) & 0xff;

	return pcfg->chip_id;
}


void iris_alloc_seq_space(void)
{
	int sum = 0;
	struct iris_cfg * pcfg  = NULL;
	struct dsi_cmd_desc * pdesc = NULL;

	pcfg = iris_get_cfg();

	iris_cal_none_lut_cmds_cnt();

	sum = pcfg->none_lut_cmds_cnt + pcfg->lut_cmds_cnt;

	if (DEBUG) {
		IRIS_LOGE("non = %d  lut = %d\n", pcfg->none_lut_cmds_cnt, pcfg->lut_cmds_cnt);
	}

	pdesc =
		(struct dsi_cmd_desc *)kmalloc(sum  * sizeof(struct dsi_cmd_desc), GFP_KERNEL);
	if (!pdesc) {
		IRIS_LOGE("can not alloc buffer \n");
		return;
	}
	pcfg->iris_cmds.iris_cmds_buf = pdesc;
	iris_out_cmds_buf_reset();

	// Need to init PQ parameters here for video panel.
	iris_pq_parameter_init();
}

static void iris_pre_lightup(struct dsi_panel *panel)
{
	udelay(500); //WA: delay 0.5ms

	//send rx cmds first with low power
	iris_send_ipopt_cmds(IRIS_IP_RX, 0xF1);

#ifdef IRIS3_MIPI_TEST
	iris_read_chip_id();
#endif

	//read chip_id
	IRIS_LOGD("%s: chip_id = %d", iris_read_chip_id());

	iris_pq_parameter_init();
}

void iris_read_power_mode(struct dsi_panel *panel)
{
	char get_power_mode[1] = {0x0a};
	char read_cmd_rbuf[16] = {0};
	struct dsi_cmd_desc cmds = {
			{0, MIPI_DSI_DCS_READ, MIPI_DSI_MSG_REQ_ACK, 0, 0, sizeof(get_power_mode), get_power_mode, 1, read_cmd_rbuf}, 1, 0};
	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &cmds,
	};
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();

	if (iris3_abypass_mode_get() == ANALOG_BYPASS_MODE)
		iris3_dsi_cmds_send(panel, cmdset.cmds, cmdset.count, cmdset.state);
	else {
		iris3_dsi_cmds_send(panel, cmdset.cmds, cmdset.count, cmdset.state);
		/* MODIFIED-BEGIN by ji.chen, 2019-08-16,BUG-8256828*/
		//IRIS_LOGE("[a]power mode: 0x%02x\n", read_cmd_rbuf[0]);
		//read_cmd_rbuf[0] = 0;
		//iris_send_cmd_to_panel(panel, &cmdset);
		/* MODIFIED-END by ji.chen,BUG-8256828*/
	}
	pcfg->power_mode = read_cmd_rbuf[0];

	IRIS_LOGE("[b]power mode: 0x%02x\n", pcfg->power_mode);
}

void iris_read_mcf_tianma(struct dsi_panel *panel, int address, u32 size, u32 *pvalues)
{
	char write_bank0[2] = {0xB0, 0x00};
	char read_register[1];
	char read_cmd_rbuf[256] = {0};
	struct dsi_cmd_desc bank0_cmds = {
			{0, MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, 0, 0, sizeof(write_bank0), write_bank0, 0, NULL}, 1, 0};
	struct dsi_cmd_desc register_cmds = {
			{0, MIPI_DSI_DCS_READ, MIPI_DSI_MSG_REQ_ACK, 0, 0, sizeof(read_register), read_register, size, read_cmd_rbuf}, 1, 0};
	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
	};
	struct iris_cfg * pcfg = iris_get_cfg();
	int i;

	// Use bank 0.
	cmdset.cmds = &bank0_cmds;
	if (pcfg->abypss_ctrl.abypass_mode == PASS_THROUGH_MODE)
		iris3_panel_cmd_passthrough(panel, &cmdset);
	else
		iris3_dsi_cmds_send(panel, cmdset.cmds, cmdset.count, cmdset.state);

	// Read 21 raw points.
	read_register[0] = (char) address;
	cmdset.cmds = &register_cmds;
	IRIS_LOGD("read_register: 0x%02x, rx_len: %zu\n", read_register[0], register_cmds.msg.rx_len);
	if (pcfg->abypss_ctrl.abypass_mode == PASS_THROUGH_MODE)
		iris3_panel_cmd_passthrough(panel, &cmdset);
	else
		iris3_dsi_cmds_send(panel, cmdset.cmds, cmdset.count, cmdset.state);
	for (i = 0; i < register_cmds.msg.rx_len; i += 4) {
		IRIS_LOGD("register: %02x %02x %02x %02x\n", read_cmd_rbuf[i], read_cmd_rbuf[i+1], read_cmd_rbuf[i+2], read_cmd_rbuf[i+3]);
	}

	for (i = 0; i < size && i < register_cmds.msg.rx_len; i++) {
		pvalues[i] = read_cmd_rbuf[i];
	}
}

void iris_read_mcf_lgd(struct dsi_panel *panel, int index, u32 size, u32 *pvalues)
{
	char get_ptnctl[1];
	char read_cmd_rbuf[64] = {0};
	struct dsi_cmd_desc cmds = {
			{0, MIPI_DSI_DCS_READ, MIPI_DSI_MSG_REQ_ACK, 0, 0, sizeof(get_ptnctl), get_ptnctl, 32, read_cmd_rbuf}, 1, 0};
	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &cmds,
	};
	int i;

	get_ptnctl[0] = 0x86 + index;

	IRIS_LOGD("get_ptnctl: 0x%02x, rx_len: %zu\n", get_ptnctl[0], cmds.msg.rx_len);

	if (iris3_abypass_mode_get() == ANALOG_BYPASS_MODE)
		iris3_dsi_cmds_send(panel, cmdset.cmds, cmdset.count, cmdset.state);
	else
		iris_send_cmd_to_panel(panel, &cmdset);

	for (i = 0; i < cmds.msg.rx_len; i += 4) {
		IRIS_LOGD("ptnctl: %02x %02x %02x %02x\n", read_cmd_rbuf[i], read_cmd_rbuf[i+1], read_cmd_rbuf[i+2], read_cmd_rbuf[i+3]);
	}

	for (i = 0; i < size && i < cmds.msg.rx_len; i++) {
		pvalues[i] = read_cmd_rbuf[i];
	}
}

/* MODIFIED-BEGIN by hongwei.tian, 2019-06-06,BUG-7808648*/
enum {
        READ_MCF_INIT = 0,
        READ_MCF_READY,
        READ_MCF_DONE,
        READ_MCF_FAIL,
};

extern ssize_t dsi_lcd_otp_read(struct dsi_display *display,u32 size,char *buf);
extern ssize_t dsi_lcd_t1pro_otp_read(struct dsi_display *display,u32 size,char *buf);
extern u8 panel_calibrate_status;
extern bool IriscrcCheckForMcf(const char *data_in, int len_in);

static int iris_read_mcf_from_panel(struct dsi_panel *panel, u32 size, u8 *pvalues)
{

	struct iris_cfg * pcfg;
	u32 len=0;

	pcfg = iris_get_cfg();

	IRIS_LOGD("%s\n", __func__);
#ifdef CONFIG_TCT_SM6150_T1_PRO
	len = dsi_lcd_t1pro_otp_read(pcfg->display, size, pvalues);
#else
	len = dsi_lcd_otp_read(pcfg->display, size, pvalues);
#endif
	if(IriscrcCheckForMcf(pvalues,106))
		return 0;
	else
		return -1;
}

/* MODIFIED-BEGIN by hongwei.tian, 2019-07-01,BUG-7974585*/
int iris_read_mcf_from_ram( u32 size, u8 *pvalues)
{

	u32 len=0;

	IRIS_LOGD("%s\n", __func__);
	len = (size <= 128) ? size : 128;
	memcpy(pvalues, panel_mcf_data, len);

	return len;
}
EXPORT_SYMBOL(iris_read_mcf_from_ram);

int panel_calibrate_state_get(void)
{

	IRIS_LOGD("%s  status = %d , \n",__func__,panel_calibrate_status);

	return panel_calibrate_status;
}
EXPORT_SYMBOL(panel_calibrate_state_get);

int panel_calibrate_state_set(int state)
{
	panel_calibrate_status = state;
	IRIS_LOGD("%s status = %d , \n",__func__,panel_calibrate_status);

	return 0;
}
EXPORT_SYMBOL(panel_calibrate_state_set);
/* MODIFIED-END by hongwei.tian,BUG-7974585*/

/* MODIFIED-END by hongwei.tian,BUG-7808648*/

int iris3_lightup(struct dsi_panel *panel, struct dsi_panel_cmd_set *on_cmds)
{
	ktime_t ktime0;
	ktime_t ktime1;
	uint32_t timeus0 = 0;
	uint32_t timeus1 = 0;
	uint8_t type = 0;
	/* MODIFIED-BEGIN by hongwei.tian, 2019-06-06,BUG-7808648*/
	int calibrate_state = READ_MCF_INIT;
	const int mcf_count = 106;
	/* MODIFIED-END by hongwei.tian,BUG-7808648*/
	struct iris_cfg * pcfg = iris_get_cfg();

	IRIS_LOGD("[Display] iris3_lightup ++\n");

	ktime0 = ktime_get();

	if (pcfg->valid == IRIS_STATUS_INVALID)
		IRIS_LOGE("%s valid is not right\n", __func__);

	pcfg->add_last_flag = pcfg->add_on_last_flag;
	iris_pre_lightup(panel);

	if (DEBUG)
		IRIS_LOGE("iris on start\n");

	type = iris_get_cont_splash_type();

	/*use to debug cont splash*/
	if (type == IRIS_CONT_SPLASH_LK){
		IRIS_LOGD("enter cont splash\n");
		iris3_send_cont_splash_pkt(IRIS_CONT_SPLASH_LK);
		pcfg->valid = IRIS_STATUS_MINI_LIGHTUP;
	} else {
		iris_send_lightup_pkt();
		pcfg->valid = IRIS_STATUS_FULL_LIGHTUP;
	}

	if (panel->bl_config.type == DSI_BACKLIGHT_PWM)
		iris_pwm_freq_set(panel->bl_config.pwm_period_usecs);

	ktime1 = ktime_get();
	if (on_cmds) {
		iris_send_cmd_to_panel(panel, on_cmds);
	}

	if (type == IRIS_CONT_SPLASH_LK){
		IRIS_LOGD("exit cont splash\n");
	} else {
		//continuous splahs should not use dma setting low power
		//iris_lp_set(); /* set iris low power */
	}

	timeus0 = (u32) ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
	timeus1 = (u32) ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime1);
	IRIS_LOGI("%s: spend time us 0 = %d  time us 1 = %d\n", __func__, timeus0, timeus1);

#ifdef IRIS3_MIPI_TEST
	iris_read_power_mode(panel);
#endif
	IRIS_LOGD("[Display] iris3_lightup --\n");
	/* MODIFIED-BEGIN by hongwei.tian, 2019-06-06,BUG-7808648*/
	calibrate_state = panel_calibrate_state_get();
	IRIS_LOGD("IRIS get calibrate state %d\n", calibrate_state);
	if (calibrate_state == READ_MCF_READY) {
		int rc = 0;
		ktime1 = ktime_get();
		rc = iris_read_mcf_from_panel(panel, mcf_count, panel_mcf_data);
		IRIS_LOGI("IRIS read mcf return: %d, value: %d, %d, CRC: %d, %d\n",
		        rc, panel_mcf_data[0], panel_mcf_data[1],
		        panel_mcf_data[mcf_count-2], panel_mcf_data[mcf_count-1]);
		calibrate_state = READ_MCF_DONE;
		if (rc) {
			calibrate_state = READ_MCF_READY;
		}
		panel_calibrate_state_set(calibrate_state);

		IRIS_LOGI("IRIS read MCF return: %d, set calibrate state: %d, spend time: %d us\n",
			rc, calibrate_state,
			(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime1));
	}
	/* MODIFIED-END by hongwei.tian,BUG-7808648*/

	pcfg->add_last_flag = pcfg->add_pt_last_flag;
	if (DEBUG)
		IRIS_LOGE("iris on end\n");
	return 0;
}

int iris_panel_enable(struct dsi_panel *panel, struct dsi_panel_cmd_set *on_cmds)
{
	int rc = 0;
	int lightup_opt = 0;
	struct iris_cfg *pcfg = NULL;

	IRIS_LOGD("%s\n", __func__);
	iris_rec_time(1);

	iris_lp_set();

	lightup_opt = iris_lightup_opt_get();
	pcfg = iris_get_cfg();

	if (iris3_abypass_mode_get() == PASS_THROUGH_MODE) {
		/* light up with PT */
		if (((lightup_opt >> 1) & 0x1) == 1) { /* efuse mode is ABYP */
			if (lightup_opt & 0x4) /* use one-wire to switch */
				iris_abyp_lightup_exit(panel, true);
			else
				iris_abyp_lightup_exit(panel, false);
		}
		rc = iris3_lightup(panel, NULL);
		IRIS_LOGI("%s iris3\n", __func__);
		iris_rec_time(1);
		rc = iris3_panel_cmd_passthrough(panel,
			&(panel->cur_mode->priv_info->cmd_sets[DSI_CMD_SET_ON]));
		IRIS_LOGI("%s panel\n", __func__);
		iris_rec_time(1);
	} else {
		/* light up with ABYP */
		if (((lightup_opt >> 1) & 0x1) == 0) { /* efuse mode is PT */
			if (lightup_opt & 0x4) /* use one-wire to switch */
				iris3_abyp_lightup(panel, true);
			else
				iris3_abyp_lightup(panel, false);
		}
		IRIS_LOGI("%s abyp\n", __func__);
		iris_rec_time(1);
		rc = iris3_dsi_cmds_send(panel, on_cmds->cmds,
			on_cmds->count, on_cmds->state);
		IRIS_LOGI("%s panel\n", __func__);
		iris_rec_time(1);
	}
	IRIS_LOGD("%s on_opt:0x%x\n", __func__, lightup_opt);
	iris_rec_time(2);
	return rc;
}

/*check whether it is in initial cont-splash packet*/
static bool iris_check_cont_splash_ipopt(uint8_t ip, uint8_t opt_id)
{
	int i = 0;
	uint8_t cs_ip = 0;
	uint8_t cs_opt_id = 0;
	struct iris_cfg * pcfg = NULL;
	struct iris_ctrl_seq *pseq_cs = NULL;

	pcfg = iris_get_cfg();
	pseq_cs = iris_get_ctrl_seq_cs(pcfg);;

	for (i = 0; i < pseq_cs->cnt; i++) {
		cs_ip = pseq_cs->ctrl_opt[i].ip;
		cs_opt_id = pseq_cs->ctrl_opt[i].opt_id;

		if (ip == cs_ip && opt_id == cs_opt_id)
			return true;
	}

	return false;
}

/*
select ip/opt to the opt_arr according to lightup stage type
*/
static int iris_select_cont_splash_ipopt(int type, struct iris_ctrl_opt * opt_arr)
{
	uint32_t i = 0;
	uint32_t j = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;

	struct iris_cfg * pcfg = NULL;
	struct iris_ctrl_seq *pseq = NULL;
	struct iris_ctrl_opt *pctrl_opt = NULL;

	pcfg = iris_get_cfg();
	pseq = iris_get_ctrl_seq(pcfg);

	for (i = 0; i < pseq->cnt; i++) {
	    pctrl_opt = pseq->ctrl_opt + i;
		ip = pctrl_opt->ip;
		opt_id = pctrl_opt->opt_id;

		if (iris_check_cont_splash_ipopt(ip, opt_id))
			continue;

		memcpy(opt_arr + j, pctrl_opt, sizeof(*pctrl_opt));
		j++;
	}

	IRIS_LOGD("real len = %d\n", j);
	return j;
}


void iris3_send_cont_splash_pkt(uint32_t type)
{
	int len = 0;
	struct iris_ctrl_opt opt_arr[IRIS_IP_CNT * IRIS_IP_OPT_CNT];
	struct iris_cfg * pcfg = NULL;
	struct iris_ctrl_seq   *pseq_cs;
	pcfg = iris_get_cfg();
	memset(opt_arr, 0xff, sizeof(opt_arr));

	if (type == IRIS_CONT_SPLASH_LK) {
		pseq_cs = iris_get_ctrl_seq_cs(pcfg);
		iris_send_assembled_pkt(pseq_cs->ctrl_opt, pseq_cs->cnt);
	} else if (type == IRIS_CONT_SPLASH_KERNEL) {
		len = iris_select_cont_splash_ipopt(type, opt_arr);
		/*stop video -->set pq --> start video*/
		iris_sde_encoder_rc_lock();
		mdelay(20);
		pcfg->add_last_flag = pcfg->add_cont_last_flag;
		iris_send_assembled_pkt(opt_arr, len);
		iris_sde_encoder_rc_unlock();
		iris_lp_set(); /* set iris low power */
		//iris_read_chip_id();
		pcfg->add_last_flag = pcfg->add_pt_last_flag;
	} else if (type == IRIS_CONT_SPLASH_BYPASS) {
		iris_abypass_to_pt_switch(pcfg->display, false, 1);
		// iris_lp_set(); /* set iris low power */
		// schedule_work(&pcfg->cont_splash_work);
	}
}

void iris3_lightoff_pre(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->valid == IRIS_STATUS_INVALID)
		IRIS_LOGE("iris status is invalid");

	pcfg->valid = IRIS_STATUS_INIT;

	if (pcfg->abypss_ctrl.abypass_mode == PASS_THROUGH_MODE){
		/*close dport*/
		iris_send_ipopt_cmds(IRIS_IP_DPORT, 0x8B);
		mdelay(16);
	}
}

int iris3_lightoff(struct dsi_panel *panel, int cmd_type)
{
	ktime_t ktime0;
	ktime_t ktime1;
	uint32_t timeus0 = 0;
	uint32_t timeus1 = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_panel_cmd_set *off_cmds;

	ktime0 = ktime_get();
	off_cmds = &(panel->cur_mode->priv_info->cmd_sets[cmd_type]);
	if (DEBUG)
		IRIS_LOGE("iris off start: %d\n", pcfg->abypss_ctrl.abypass_mode);
	if (pcfg->abypss_ctrl.abypass_mode == PASS_THROUGH_MODE)
		iris3_panel_cmd_passthrough(panel, off_cmds);
	else {
		iris3_dsi_cmds_send(panel, off_cmds->cmds, off_cmds->count, off_cmds->state);
		return 0;
	}
	ktime1 = ktime_get();

	iris_quality_setting_off();
	iris_lp_setting_off();

	timeus0 = (u32) ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
	timeus1 = (u32) ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime1);
	IRIS_LOGD("%s spend time us 0 = %d  time us 1 = %d\n", __func__, timeus0, timeus1);
	if (DEBUG)
		IRIS_LOGE("iris off end\n");

	return 0;
}

static void iris_send_update_new_opt(struct iris_update_ipopt * popt, struct iris_cmd_comp *pasm_comp, uint8_t path)
{
	int32_t ip = 0;
	int32_t rc = 0;
	struct iris_ctrl_opt ctrl_opt;

	ip = popt->ip;
	ctrl_opt.ip = popt->ip;
	ctrl_opt.opt_id = popt->opt_new;
	ctrl_opt.skip_last = popt->skip_last;

	/*speical deal with lut table*/
	if (ip == IRIS_IP_EXT) {
		rc = iris_send_lut_table_pkt(&ctrl_opt, pasm_comp, true, path);
	} else {
		rc = iris_send_none_lut_table_pkt(&ctrl_opt, pasm_comp, path);
	}

	if (rc) {
		panic("%s\n", __func__);
	}
}

void iris_dsi_error_flag_set(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	pcfg->bta_error = true;
	IRIS_LOGD("%s\n", __func__);
}

int iris_dsi_bta_status_check(void)
{
	int ret = 0;
	int i;
	struct iris_cfg *pcfg;
	struct dsi_display * display = NULL;
	struct dsi_ctrl *dsi_ctrl = NULL;
	struct dsi_ctrl_hw *ctrl;

	pcfg = iris_get_cfg();
	display = pcfg->display;

	for (i = 0; i < display->ctrl_count; i++) {
		dsi_ctrl = display->ctrl[i].ctrl;
		ctrl = &(dsi_ctrl->hw);
		DSI_W32(ctrl, DSI_CMD_MODE_BTA_SW_TRIGGER, 0x1);
	}
	IRIS_LOGD("%s\n", __func__);

	return ret;
}

static void iris_send_pq_cmds(struct iris_update_ipopt * popt, int len, uint8_t path)
{
	int32_t i = 0;
	int retries = 3;
	struct iris_cmd_comp cmd_comp;
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();
	if (! popt  || !len) {
		IRIS_LOGE("popt is null\n");
		return ;
	}

	IRIS_LOGD("iris_send_pq_cmds\n");
	memset(&cmd_comp, 0x00, sizeof(cmd_comp));
	cmd_comp.cmd =  pcfg->iris_cmds.iris_cmds_buf;
	cmd_comp.link_state = DSI_CMD_SET_STATE_HS;
	cmd_comp.cnt = pcfg->iris_cmds.cmds_index;

	mutex_lock(&pcfg->mutex);
retry:
	pcfg->bta_error = false;
	for (i = 0; i < len; i++) {
		iris_send_update_new_opt(&popt[i], &cmd_comp, path);
	}
	IRIS_LOGD("request BTA\n");
	iris_dsi_bta_status_check();
	if (pcfg->bta_error && --retries > 0) {
		IRIS_LOGE("%s retry\n", __func__);
		pcfg->add_last_flag = pcfg->add_on_last_flag; /* Change to single pkg send */
		goto retry;
	}

	mutex_unlock(&pcfg->mutex);
	IRIS_LOGD("done\n");
}


static int iris_update_pq_seq(struct iris_update_ipopt * popt, int len)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t ip = 0;
	int32_t opt_id = 0;
	struct iris_cfg * pcfg;
	struct iris_ctrl_seq *pseq = NULL;

	pcfg = iris_get_cfg();
	pseq = iris_get_ctrl_seq(pcfg);

	for (i = 0; i < len; i++) {
		/*need to update sequence*/
		if (popt[i].opt_new != popt[i].opt_old) {
			for (j = 0; j < pseq->cnt; j++) {
				ip = pseq->ctrl_opt[j].ip;
				opt_id = pseq->ctrl_opt[j].opt_id;

				/*TODO need to modify for pxlw iris*/
				if (ip == popt[i].ip && opt_id == popt[i].opt_old) {
					break;
				}
			}

			if (j == pseq->cnt) {
				IRIS_LOGE("%s can not find the ip = %02x opt_id = %02x\n update opt_id =%02x",
						__func__, popt[i].ip, popt[i].opt_old, popt[i].opt_new);
				return -EINVAL;
			}

			pseq->ctrl_opt[j].opt_id = popt[i].opt_new;
		}
	}
	return 0;
}


void iris_update_pq_opt(struct iris_update_ipopt * popt, int len, uint8_t path)
{
	int32_t rc = 0;
	if (! popt  || !len) {
		IRIS_LOGE("popt is null\n");
		return ;
	}

	rc = iris_update_pq_seq(popt, len);
	if (!rc)
		iris_send_pq_cmds(popt, len, path);
}


struct iris_ip_opt * iris_find_ip_opt(uint8_t ip, uint8_t opt_id)
{
	int32_t i = 0;
	struct iris_cfg * pcfg = NULL;
	struct iris_ip_opt *popt = NULL;
	struct iris_ip_index *pip_index = NULL;

	if (ip >= IRIS_IP_CNT){
		IRIS_LOGE("ip %d is out of maxvalue %d \n", ip , IRIS_IP_CNT);
		return NULL;
	}

	pcfg = iris_get_cfg();
	pip_index = &pcfg->ip_index_arr[ip];
	for (i = 0; i < pip_index->opt_cnt; i++){
		popt = pip_index->opt + i;
		if (popt->opt_id == opt_id) {
			return popt;
		}
	}

	return NULL;
}

static struct dsi_cmd_desc * iris_get_ipopt_desc(uint8_t ip, uint8_t opt_id)
{
	struct iris_ip_opt  *popt = NULL;

	popt = iris_find_ip_opt(ip, opt_id);
	if (popt == NULL) {
		IRIS_LOGE("can not find right popt ip = %02x opt_id = %02x\n", ip , opt_id);
		return NULL;
	} else if (popt->len > 1) {
		IRIS_LOGE("this opt %d is more than 1 \n", popt->len);
		return NULL;
	}
	return popt->cmd;
}


uint32_t  * iris_get_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos)
{
	struct dsi_cmd_desc * pdesc = NULL;

	pdesc = iris_get_ipopt_desc(ip, opt_id);
	if (!pdesc) {
		IRIS_LOGE("can not find right desc\n");
		return NULL;
	} else if (pos > pdesc->msg.tx_len) {
		IRIS_LOGE("pos %d is out of paload length %zu\n", pos , pdesc->msg.tx_len);
		return NULL;
	}

	return (uint32_t *)(pdesc->msg.tx_buf) + pos;
}

void iris_update_bitmask_regval_nonread(
				struct iris_update_regval *pregval, bool is_commit)
{
	int32_t ip = 0;
	int32_t opt_id = 0;
	uint32_t orig_val = 0;
	uint32_t *data = NULL;
	uint32_t val = 0;
	struct iris_ip_opt *popt;

	if (! pregval) {
		IRIS_LOGE("pregval is null\n");
		return ;
	}

	ip = pregval->ip;
	opt_id = pregval->opt_id;

	popt = iris_find_ip_opt(ip, opt_id);
	if (popt == NULL) {
		IRIS_LOGE("can not find ip = %02x opt_id = %02x\n", ip, opt_id);
		return;
	} else if (popt->len != 1) {
			IRIS_LOGE("error for bitmask popt->len = %d \n", popt->len);
			return;
	}

	data =(uint32_t *)popt->cmd[0].msg.tx_buf;

	orig_val = cpu_to_le32(data[2]);
	val = orig_val & (~ pregval->mask);
	val |= (pregval->value  & pregval->mask);
	data[2] = val;

	if (is_commit)
		iris_send_ipopt_cmds(ip, opt_id);

}


void iris_update_bitmask_regval(struct iris_update_regval *pregval, bool is_commit)
{
	int32_t ip = 0;
	int32_t opt_id = 0;
	uint32_t *data = NULL;
	struct iris_ip_opt *popt = NULL;

	if (! pregval) {
		IRIS_LOGE("pregval is null\n");
		return ;
	}

	ip = pregval->ip;
	opt_id = pregval->opt_id;

	popt = iris_find_ip_opt(ip, opt_id);
	if (popt == NULL) {
		IRIS_LOGE("can not find ip = %02x opt_id = %02x\n", ip, opt_id);
		return;
	} else if (popt->len != 2) {
			IRIS_LOGE("error for bitmask popt->len = %d \n", popt->len);
			return;
	}

	data =(uint32_t *)popt->cmd[1].msg.tx_buf;
	data[2] = cpu_to_le32(pregval->mask);
	data[3] = cpu_to_le32(pregval->value);

	if (is_commit)
		iris_send_ipopt_cmds(ip, opt_id);
}


int32_t iris_update_lut_payload(int32_t ip, int32_t opt_id, struct dsi_panel_cmd_set *pcmds)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t opt_len = 0;
	int16_t len = 0;
	int16_t dlen = 0;
	uint8_t *payload = NULL;
	uint8_t * data = NULL;
	struct iris_cfg * pcfg;
	struct iris_ip_index *pip_index;

	pcfg = iris_get_cfg();
	pip_index = &pcfg->ip_index_arr[ip];

	for (i = 0; i < pip_index->opt_cnt; i++){
		if (pip_index->opt[i].opt_id == opt_id) {
			opt_len = pip_index->opt[i].len;

			if (pcmds->count != opt_len) {
				IRIS_LOGE("ip %d\n", ip);
				IRIS_LOGE("the dtsi seq is not competence cmds cnt = %d  real cnt = %d \n",
					opt_len, pcmds->count);
				return -EINVAL;
			}

			for (j = 0; j < opt_len; j++) {
				/*remove the base address and header*/
				payload = (uint8_t *)pcmds->cmds[j].msg.tx_buf + 8;
				len = pcmds->cmds[j].msg.tx_len;

				data = (uint8_t *)pip_index->opt[i].cmd[j].msg.tx_buf + 8;
				dlen = pip_index->opt[i].cmd[j].msg.tx_len;

				if (dlen != len) {
					IRIS_LOGE("the current cmd payload len %d is not equal  %d\n", dlen, len);
					return -EINVAL;
				}

				/*remove base address and header*/
				memcpy(data, payload, len - 8);
				//iris_dump_packet(payload, len);
			}

			break;
		}
	}
	return 0;
}


static ssize_t iris_cont_splash_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	iris_set_cont_splash_type(val);

	if (val == IRIS_CONT_SPLASH_KERNEL){
		iris3_send_cont_splash_pkt(val);
	} else if (val != IRIS_CONT_SPLASH_LK && val != IRIS_CONT_SPLASH_NONE){
		IRIS_LOGE("the value is %zu, need to be 1 or 2 3", val);
	}
	return count;
}


static ssize_t iris_cont_splash_read(struct file *file, char __user *buff,
                size_t count, loff_t *ppos)
{
	uint8_t type;
	int len, tot = 0;
	char bp[512];
	if (*ppos)
		return 0;

	type = iris_get_cont_splash_type();
	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", type);

	if (copy_to_user(buff, bp, tot))
	        return -EFAULT;

	*ppos += tot;

	return tot;
}


static const struct file_operations iris_cont_splash_fops = {
	.open = simple_open,
	.write = iris_cont_splash_write,
	.read = iris_cont_splash_read,
};


static ssize_t iris_split_pkt_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg *pcfg = NULL;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	pcfg = iris_get_cfg();
	pcfg->add_last_flag = val;

	return count;
}


static ssize_t iris_split_pkt_read(struct file *file, char __user *buff,
                size_t count, loff_t *ppos)
{
	uint8_t type;
	int len, tot = 0;
	struct iris_cfg *pcfg = NULL;
	char bp[512];
	if (*ppos)
		return 0;

	pcfg = iris_get_cfg();
	type = pcfg->add_last_flag;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", type);

	if (copy_to_user(buff, bp, tot))
	        return -EFAULT;

	*ppos += tot;

	return tot;
}


static const struct file_operations iris_split_pkt_fops = {
	.open = simple_open,
	.write = iris_split_pkt_write,
	.read = iris_split_pkt_read,
};

static ssize_t iris_chip_id_read(struct file *file, char __user *buff,
                size_t count, loff_t *ppos)
{
	int tot = 0;
	struct iris_cfg *pcfg = NULL;
	char bp[512];
	if (*ppos)
		return 0;

	pcfg = iris_get_cfg();

	tot = scnprintf(bp, sizeof(bp), "%u\n", pcfg->chip_id);
	if (copy_to_user(buff, bp, tot))
	    return -EFAULT;
	*ppos += tot;

	return tot;
}

static const struct file_operations iris_chip_id_fops = {
	.open = simple_open,
	.read = iris_chip_id_read,
};

static ssize_t iris_power_mode_read(struct file *file, char __user *buff,
                size_t count, loff_t *ppos)
{
	int tot = 0;
	struct iris_cfg *pcfg = NULL;
	char bp[512];
	if (*ppos)
		return 0;

	pcfg = iris_get_cfg();

	tot = scnprintf(bp, sizeof(bp), "0x%02x\n", pcfg->power_mode);
	if (copy_to_user(buff, bp, tot))
	    return -EFAULT;
	*ppos += tot;

	return tot;
}

static const struct file_operations iris_power_mode_fops = {
	.open = simple_open,
	.read = iris_power_mode_read,
};

static ssize_t iris_dbg_i2c_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;
	int i;
	uint32_t arr[100] = {0};


	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	IRIS_LOGD("%s,%d \n", __func__,__LINE__);

	arr[0] = 0xf1200000;
	for (i = 1; i < 10; i++) {
		arr[i] = 0x66778800 + i*2;
	}
	iris_i2c_conver_ocp_write(arr[0], &arr[1], 4, 1);

	return count;
}

static ssize_t iris_dbg_i2c_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int i, cnt = 0;
	bool is_burst;
	uint32_t arr[100] = {0};


	arr[0] = 0xf1200000;

	is_burst = 1;
	cnt = 4;
	iris_i2c_conver_ocp_read(arr, cnt, is_burst);
	for (i = 0; i < cnt; i++)
		IRIS_LOGE("%s,%d: arr[%d] = %x\n", __func__, __LINE__, i, arr[i]);

	return 0;
}

static const struct file_operations iris_i2c_srw_fops = {
	.open = simple_open,
	.write = iris_dbg_i2c_write,
	.read = iris_dbg_i2c_read,
};


static int iris_cont_splash_debugfs_init(struct dsi_display *display)
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
	if (debugfs_create_file("iris_cont_splash", 0644, pcfg->dbg_root, display,
				&iris_cont_splash_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("iris_split_pkt", 0644, pcfg->dbg_root, display,
				&iris_split_pkt_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("chip_id", 0644, pcfg->dbg_root, display,
				&iris_chip_id_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("power_mode", 0644, pcfg->dbg_root, display,
				&iris_power_mode_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("iris_i2c_srw", 0644, pcfg->dbg_root, display,
				&iris_i2c_srw_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	debugfs_create_u8("iris_pq_update_path", 0644, pcfg->dbg_root,
		(uint8_t *)&iris_pq_update_path);

	return 0;
}


void iris3_display_prepare(struct dsi_display *display)
{
	static bool iris_boot = false;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->valid == IRIS_STATUS_INVALID)
		return;

	if (iris_boot == false) {
		iris3_parse_lut_cmds(&display->pdev->dev, IRIS3_FIRMWARE_NAME);
		iris_alloc_seq_space();
		iris_boot = true;
	}
}

extern bool hbm_mode;
extern u16 g_brightness;
/* MODIFIED-BEGIN by hongwei.tian, 2019-12-17,BUG-8681047*/
#if defined(CONFIG_TCT_SM6150_T1_PRO)
extern int panel_batch_id;
#endif
/* MODIFIED-END by hongwei.tian,BUG-8681047*/
int iris3_update_backlight(u32 bl_lvl)
{
	int rc = 0;
	struct iris_cfg *pcfg = NULL;
	struct mipi_dsi_device *dsi;
	struct dsi_panel *panel;
	char led_pwm1[3] = {MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x0, 0x0};
	struct dsi_cmd_desc backlight_cmd = {
		{0, MIPI_DSI_DCS_LONG_WRITE, 0, 0, 0, sizeof(led_pwm1), led_pwm1, 0, NULL}, 1, 1};

	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &backlight_cmd,
	};
	struct dsi_backlight_config *bl = NULL;

/* MODIFIED-BEGIN by hongwei.tian, 2019-12-17,BUG-8681047*/
#if defined(CONFIG_TCT_SM6150_T1_PRO)
	char page_cmd[2] = {0xfe, 0x72};
	char bl_cmd[2] = {0x69, 0x80};
	char page_cmd2[2] = {0xfe, 0x00};
	struct dsi_cmd_desc bl_cmds[3] = {
		{{0, 0x39, 0, 0, 0, sizeof(page_cmd), page_cmd, 0, NULL}, 0, 1},
		{{0, 0x39, 0, 0, 0, sizeof(bl_cmd), bl_cmd, 0, NULL}, 0, 1},
		{{0, 0x39, 0, 0, 0, sizeof(page_cmd2), page_cmd2, 0, NULL}, 1, 1}};

	struct dsi_panel_cmd_set page_cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 3,
		.cmds = bl_cmds,
	};
	u8 write_reg,aod_bl_level;
	ssize_t err;
	u8 reg_buf[4] = {0x30,0x80,0x20,0xff};
#endif
/* MODIFIED-END by hongwei.tian,BUG-8681047*/

	pcfg = iris_get_cfg();
	panel = pcfg->panel;
	dsi = &panel->mipi_device;
	bl = &panel->bl_config;

	iris_setting.quality_cur.system_brightness = bl_lvl;
	g_brightness = bl_lvl;

	/* Don't set panel's brightness during HDR/SDR2HDR */
	/* Set panel's brightness when sdr2hdr mode is 3 */
	/*if (iris_setting.quality_cur.pq_setting.sdr2hdr != SDR2HDR_Bypass  && iris_sdr2hdr_mode != 3)
		return rc;
	*/
	/*only save bl_lvl value*/
	if (bl->type == DSI_BACKLIGHT_WLED) {
		IRIS_LOGD("%s bl_lvl =%d ************************\n", __func__, bl_lvl);
		rc = backlight_device_set_brightness(bl->raw_bd, bl_lvl);
		return rc;
	}

/* MODIFIED-BEGIN by hongwei.tian, 2019-12-17,BUG-8681047*/
#if defined(CONFIG_TCT_SM6150_T1_PRO)
	if (panel->power_mode == SDE_MODE_DPMS_LP1 ||
			panel->power_mode == SDE_MODE_DPMS_LP2)
	{
		if (hbm_mode == true)
			goto set_bl_noaod;

		pr_info("%s: set backlight in AOD ,bl_lvl = %d \n", __func__,bl_lvl);
		if(panel_batch_id >= 2)
		{
			pr_err("%s: set backlight in AOD ,invalid batch_id \n", __func__);
			return 0;
		}
		if(bl_lvl == 1)
			aod_bl_level = 0;
		else if(bl_lvl == 2)
			aod_bl_level = 1;
		else
		{
			pr_err("%s: set backlight in AOD ,invalid level =%d \n", __func__,bl_lvl);
			/* MODIFIED-BEGIN by hongwei.tian, 2019-12-31,BUG-8728752*/
			//return 0;
			goto set_bl_noaod;
			/* MODIFIED-END by hongwei.tian,BUG-8728752*/
		}

		if(iris3_abypass_mode_get() == PASS_THROUGH_MODE)
		{
			bl_cmd[1] = reg_buf[panel_batch_id*2+aod_bl_level];
			pr_info("%s: set backlight in AOD ,bl_cmd = 0x%x \n", __func__,bl_cmd[1]);
			iris3_panel_cmd_passthrough(panel, &page_cmdset);

		}
		else
		{
			write_reg = 0x72;
			err = mipi_dsi_dcs_write(dsi, 0xFE,&write_reg, 1);
			write_reg = reg_buf[panel_batch_id*2+aod_bl_level];
			pr_info("%s: set backlight in AOD ,write_reg = 0x%x \n", __func__,write_reg);
			err = mipi_dsi_dcs_write(dsi, 0x69,&write_reg, 1);
			write_reg = 0x00;
			err = mipi_dsi_dcs_write(dsi, 0xFE,&write_reg, 1);
			/* code */
		}
		return 0;
	}
set_bl_noaod: // MODIFIED by hongwei.tian, 2019-12-31,BUG-8728752
#endif
/* MODIFIED-END by hongwei.tian,BUG-8681047*/
	if (panel->bl_config.bl_max_level > 255) {
		led_pwm1[1] = (unsigned char)(bl_lvl >> 8);
		led_pwm1[2] = (unsigned char)(bl_lvl & 0xff);
	} else {
		led_pwm1[1] = (unsigned char)(bl_lvl & 0xff);
		backlight_cmd.msg.tx_len = 2;
	}

	if (hbm_mode == true) {
		pr_info("%s: in hbm\n", __func__);
		led_pwm1[1] = 0x0f;
		led_pwm1[2] = 0xff;
	}

	IRIS_LOGD("%s bl_lvl =%d 0x%x 0x%x\n", __func__, bl_lvl, led_pwm1[1], led_pwm1[2]);

	if (pcfg->abypss_ctrl.abypass_mode == PASS_THROUGH_MODE)
		rc = iris3_panel_cmd_passthrough(panel, &cmdset);
	else
		rc = mipi_dsi_dcs_set_display_brightness(dsi, bl_lvl);

	return rc;
}

int iris3_set_panel_fod(struct mipi_dsi_device *dsi, u32 enable)
{
	int rc = 0;
	struct iris_cfg *pcfg = NULL;
	struct dsi_panel *panel;
	char fod_cmd[2] = {0x83, 0x0};
	struct dsi_cmd_desc cmd = {
		{0, MIPI_DSI_DCS_LONG_WRITE, 0, 0, 0, sizeof(fod_cmd), fod_cmd, 0, NULL}, 1, 1};

	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &cmd,
	};

	pcfg = iris_get_cfg();
	panel = pcfg->panel;

	if (enable)
		fod_cmd[1] = 0x03;
	else
		fod_cmd[1] = 0x00;

	if (pcfg->abypss_ctrl.abypass_mode == PASS_THROUGH_MODE)
		rc = iris3_panel_cmd_passthrough(panel, &cmdset);
	else
		rc = mipi_dsi_dcs_set_fod(dsi, enable);

	return rc;
}


void iris3_display_enable(struct dsi_display *display)
{
	/*TODO: i2c read abypass mode*/

	struct iris_cfg *pcfg = iris_get_cfg();

	if (iris3_abypass_mode_get() == PASS_THROUGH_MODE) {
		pcfg->valid = IRIS_STATUS_MINI_LIGHTUP;

		iris3_send_cont_splash_pkt(IRIS_CONT_SPLASH_KERNEL);

		pcfg->valid = IRIS_STATUS_FULL_LIGHTUP;
	}

}

void iris3_set_mode_from_cmdline(struct dsi_display *display, char *boot_str)
{
	char *str = NULL;
	char buf[10] = {0};
	//default iris in abypass mode
	unsigned long cmdline_iris_mode = 1;

	display->cmdline_iris_mode = cmdline_iris_mode;
	str = strnstr(boot_str, ":iris_mode", strlen(boot_str));
	if (!str) {
		IRIS_LOGE("%s do not set iris_mode\n", __func__);
		return;
	}
	strncpy(buf, str + strlen(":iris_mode"), 1);
	if (kstrtol(buf, 10, (unsigned long *)&cmdline_iris_mode)) {
		IRIS_LOGE("invalid timing index override: %s. resetting iris mode  timing and config\n", boot_str);
		return;
	}

	IRIS_LOGI("successfully iris_mode =%ld\n", cmdline_iris_mode);
	display->cmdline_iris_mode = cmdline_iris_mode;
}
