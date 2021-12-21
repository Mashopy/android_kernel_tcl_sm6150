/* Copyright (C) 2019 Tcl Corporation Limited */
#include "dsi_iris3_api.h"
#include "dsi_iris3.h"
#include "dsi_iris3_lightup.h"
#include "dsi_iris3_lightup_ocp.h"
#include "dsi_iris3_lp.h"
#include "dsi_iris3_lut.h"
#include "dsi_iris3_pq.h"
#include "dsi_iris3_ioctl.h"
#include "dsi_iris3_log.h"

extern struct iris_setting_info iris_setting;
extern u8 iris_sdr2hdr_mode;
extern bool iris_debug_cap;

extern struct msmfb_iris_ambient_info iris_ambient_lut;
extern struct msmfb_iris_maxcll_info iris_maxcll_lut;

// for game station settings via i2c
uint32_t CM_CNTL[14] = {
	0xf0560000, 0x8020e000,
	0xf0560000, 0x820e000,
	0xf0560008, 0x00000000,
	0xf056000c, 0x6e,
	0xf056000c, 0x5f,
	0xf0560110, 0x00000000,
	0xf0560140, 0x00000100
};

// 0: mipi, 1: i2c
static int adb_type = 0;
extern u8 panel_raw_data[254];
extern u8 panel_mcf_data[128]; // MODIFIED by hongwei.tian, 2019-06-06,BUG-7808648

static int mdss_mipi_dsi_command(void __user *values)
{
	struct msmfb_mipi_dsi_cmd cmd;

	char read_response_buf[16] = {0};
	struct dsi_cmd_desc desc = {
		.msg.rx_buf = &read_response_buf,
		.msg.rx_len = 16
	};
	struct dsi_cmd_desc *pdesc_muti = NULL;
	struct dsi_cmd_desc *pdesc;
	struct dsi_panel_cmd_set cmdset = {
		.count = 1,
		.cmds = &desc
	};
	int ret, indx, cmd_len, cmd_cnt;
	char *pcmd_indx;
	struct iris_cfg *pcfg = iris_get_cfg();

	struct iris_ocp_dsi_tool_input iris_ocp_input={0,0,0,0,0};

	ret = copy_from_user(&cmd, values, sizeof(cmd));
	if (ret) {
		IRIS_LOGE("can not copy from user\n");
		return -EPERM;
	}

	IRIS_LOGD("#### %s:%d vc=%u d=%02x f=%u l=%u\n", __func__, __LINE__,
		   cmd.vc, cmd.dtype, cmd.flags, cmd.length);

	IRIS_LOGD("#### %s:%d %x, %x, %x\n", __func__, __LINE__,
		   cmd.iris_ocp_type, cmd.iris_ocp_addr, cmd.iris_ocp_size);

	if (cmd.length) {
		desc.msg.tx_buf = kmalloc(cmd.length, GFP_KERNEL);
		if (!desc.msg.tx_buf)
			return -ENOMEM;
		ret = copy_from_user((char *)desc.msg.tx_buf, cmd.payload, cmd.length);
		if (ret) {
			ret = -EPERM;
			goto err;
		}
	}

	desc.msg.type = cmd.dtype;
	desc.msg.channel = cmd.vc;
	desc.last_command = (cmd.flags & MSMFB_MIPI_DSI_COMMAND_LAST) > 0;
	desc.msg.flags |= ((cmd.flags & MSMFB_MIPI_DSI_COMMAND_ACK) > 0 ? MIPI_DSI_MSG_REQ_ACK : 0);
	desc.msg.tx_len = cmd.length;
	desc.post_wait_ms = 0;
	desc.msg.ctrl = 0;
	if (cmd.dtype == 0x0f) {
		cmd_cnt = *((char *)desc.msg.tx_buf);
		pdesc_muti = kmalloc(sizeof(struct dsi_cmd_desc) * cmd_cnt, GFP_KERNEL);
		pcmd_indx = (char *)desc.msg.tx_buf + cmd_cnt + 1;
		for (indx = 0; indx < cmd_cnt; indx++) {
			pdesc = pdesc_muti + indx;
			cmd_len = *((char *)desc.msg.tx_buf + 1 + indx);
			pdesc->msg.type = *pcmd_indx;
			pdesc->msg.channel = 0;
			pdesc->last_command = false;
			pdesc->msg.flags |= 0;
			pdesc->msg.tx_len = cmd_len - 1;
			pdesc->post_wait_ms = 0;
			pdesc->msg.tx_buf = pcmd_indx + 1;

			pcmd_indx += cmd_len;
			if (indx == (cmd_cnt - 1))
				pdesc->last_command = true;
			printk("dtype:%x, dlen: %zu, last: %d\n", pdesc->msg.type, pdesc->msg.tx_len, pdesc->last_command);
		}
		cmdset.cmds = pdesc_muti;
		cmdset.count = cmd_cnt;
	}

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_ACK) {
		desc.msg.flags = desc.msg.flags | DSI_CTRL_CMD_READ;
	}

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_HS)
		cmdset.state = DSI_CMD_SET_STATE_HS;

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_TO_PANEL)
		iris3_panel_cmd_passthrough(pcfg->panel, &cmdset);
	else if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_T){
		u32 pktCnt = (cmd.iris_ocp_type >> 8) & 0xFF;

		//only test LUT send command
		if((cmd.iris_ocp_type & 0xF) == PXLW_DIRECTBUS_WRITE){
			u8 lut_type = (cmd.iris_ocp_type >> 8) & 0xFF;
			u8 lut_index = (cmd.iris_ocp_type >> 16) & 0xFF;
			u8 lut_parse = (cmd.iris_ocp_type >> 24) & 0xFF;
			u32 lut_pkt_index = cmd.iris_ocp_addr;
			if (lut_parse) // only parse firmware when value is not zero;
				iris3_parse_lut_cmds(&pcfg->display->pdev->dev, IRIS3_FIRMWARE_NAME);
			iris_lut_send(lut_type, lut_index, lut_pkt_index, true);
		}
		else { // test ocp wirte
			if(pktCnt > DSI_CMD_CNT)
				pktCnt = DSI_CMD_CNT;

			if(cmd.iris_ocp_size < OCP_MIN_LEN)
				cmd.iris_ocp_size = OCP_MIN_LEN;

			iris_ocp_input.iris_ocp_type = cmd.iris_ocp_type & 0xF;
			iris_ocp_input.iris_ocp_cnt = pktCnt;
			iris_ocp_input.iris_ocp_addr = cmd.iris_ocp_addr;
			iris_ocp_input.iris_ocp_value = cmd.iris_ocp_value;
			iris_ocp_input.iris_ocp_size = cmd.iris_ocp_size;

			if(pktCnt)
				iris_write_test_muti_pkt(pcfg->panel, &iris_ocp_input);
			else
				iris_write_test(pcfg->panel, cmd.iris_ocp_addr, cmd.iris_ocp_type & 0xF, cmd.iris_ocp_size);
				//iris_ocp_bitmask_write(ctrl,cmd.iris_ocp_addr,cmd.iris_ocp_size,cmd.iris_ocp_value);
		}
	} else
		iris3_dsi_cmds_send(pcfg->panel, cmdset.cmds, cmdset.count, cmdset.state);

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_ACK) {
		// Both length of cmd.response and read_response_buf are 16.
		memcpy(cmd.response, read_response_buf, sizeof(cmd.response));
	}
	ret = copy_to_user(values, &cmd, sizeof(cmd));
	if (ret)
		ret = -EPERM;
err:
	kfree(desc.msg.tx_buf);
	if (cmd.dtype == 0x0f && pdesc_muti != NULL)
		kfree(pdesc_muti);
	return ret;
}


int iris3_operate_tool(struct msm_iris_operate_value *argp)
{
	int ret = -1;
	uint32_t parent_type = 0;

	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->valid == IRIS_STATUS_INVALID)
		return -EINVAL;

	// FIXME: copy_from_user() is failed.
	// ret = copy_from_user(&configure, argp, sizeof(configure));
	// if (ret) {
	// 	IRIS_LOGE("1st %s type = %d, value = %d\n",
	// 		__func__, configure.type, configure.count);
	// 	return -EPERM;
	// }
	IRIS_LOGE("%s type = %d, value = %d\n", __func__, argp->type, argp->count);

	parent_type = argp->type & 0xff;
	switch (parent_type) {
	case IRIS_OPRT_TOOL_DSI:
		ret = mdss_mipi_dsi_command(argp->values);
		break;
	default:
		IRIS_LOGE("could not find right opertat type = %d\n", argp->type);
		ret = -EINVAL;
		break;
	}
	return ret;
}

/* Iris log level definition, for 'iris_log.h' */
static int iris_log_level = 2;

void iris_set_loglevel(int level)
{
	iris_log_level = level;
}

inline int iris_get_loglevel(void)
{
	return iris_log_level;
}

int iris_configure(u32 type, u32 value)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct quality_setting *pqlt_cur_setting = & iris_setting.quality_cur;

	IRIS_LOGD("%s type=0x%04x, value=%d", __func__, type, value);

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return -1;

	if (type != IRIS_ANALOG_BYPASS_MODE && pcfg->abypss_ctrl.abypass_mode == ANALOG_BYPASS_MODE)
		return -1;

	if (pcfg->valid < IRIS_STATUS_FULL_LIGHTUP) {
		IRIS_LOGE("%s iris is not ready\n", __func__);
		return -EINVAL;
	}

	switch (type) {
	case IRIS_PEAKING:
		pqlt_cur_setting->pq_setting.peaking = value & 0xf;
		if (pqlt_cur_setting->pq_setting.peaking > 4)
			goto error;

		iris_peaking_level_set(pqlt_cur_setting->pq_setting.peaking);
		break;
	case IRIS_CM_6AXES:
		pqlt_cur_setting->pq_setting.cm6axis = value & 0x3;
		iris_cm_6axis_level_set(pqlt_cur_setting->pq_setting.cm6axis);
		break;
	case IRIS_CM_FTC_ENABLE:
		pqlt_cur_setting->pq_setting.cmftc = value & 0x1;
		iris_cm_ftc_enable_set(pqlt_cur_setting->pq_setting.cmftc);
		break;
	case IRIS_CM_COLOR_TEMP_MODE:
		pqlt_cur_setting->pq_setting.cmcolortempmode = value & 0x3;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode > 2)
			goto error;

		iris_cm_colortemp_mode_set(pqlt_cur_setting->pq_setting.cmcolortempmode);
		break;
	case IRIS_CM_COLOR_GAMUT_PRE:
		iris_cm_color_gamut_pre_set(value & 0x03);
		break;
	case IRIS_CM_COLOR_GAMUT:
		pqlt_cur_setting->pq_setting.cmcolorgamut = value;
		if (pqlt_cur_setting->pq_setting.cmcolorgamut > 6)
			goto error;

		iris_cm_color_gamut_set(pqlt_cur_setting->pq_setting.cmcolorgamut);
		break;
	case IRIS_DBC_LCE_POWER:
		if (0 == value)
			iris_dbclce_power_set(false);
		else if (1 == value)
			iris_dbclce_power_set(true);
		else if (2 == value)
			iris_lce_dynamic_pmu_mask_set(false);
		else if (3 == value)
			iris_lce_dynamic_pmu_mask_set(true);

		break;
	case IRIS_DBC_LCE_DATA_PATH:
		iris_dbclce_datapath_set(value & 0x01);
		break;
	case IRIS_LCE_MODE:
		if (pqlt_cur_setting->pq_setting.lcemode != (value & 0x1)) {
			pqlt_cur_setting->pq_setting.lcemode = value & 0x1;
			iris_lce_mode_set(pqlt_cur_setting->pq_setting.lcemode);
		}
		break;
	case IRIS_LCE_LEVEL:
		if (pqlt_cur_setting->pq_setting.lcelevel != (value & 0x7)) {
			pqlt_cur_setting->pq_setting.lcelevel = value & 0x7;
			if (pqlt_cur_setting->pq_setting.lcelevel > 5)
				goto error;

			iris_lce_level_set(pqlt_cur_setting->pq_setting.lcelevel);
		}
		break;
	case IRIS_GRAPHIC_DET_ENABLE:
		pqlt_cur_setting->pq_setting.graphicdet = value & 0x1;
		iris_lce_graphic_det_set(pqlt_cur_setting->pq_setting.graphicdet);
		break;
	case IRIS_AL_ENABLE:

		if (pqlt_cur_setting->pq_setting.alenable != (value & 0x1)) {
			pqlt_cur_setting->pq_setting.alenable = value & 0x1;

			/*check the case here*/
			if (pqlt_cur_setting->pq_setting.sdr2hdr == SDR2HDR_Bypass)
				iris_lce_al_set(pqlt_cur_setting->pq_setting.alenable);
			else
				iris_ambient_light_lut_set();
		}
		break;
	case IRIS_DBC_LEVEL:
		pqlt_cur_setting->pq_setting.dbc = value & 0x3;
		iris_dbc_level_set(pqlt_cur_setting->pq_setting.dbc);
		break;
	case IRIS_BLC_PWM_ENABLE:
		iris_pwm_enable_set(value & 0x1);
		break;
	case IRIS_DEMO_MODE:
		pqlt_cur_setting->pq_setting.demomode = value & 0x3;
		break;
	case IRIS_DYNAMIC_POWER_CTRL:
		iris_dynamic_power_set(value & 0x01);
		break;
	case IRIS_DMA_LOAD:
		iris_dma_trigger_load();
		break;
	case IRIS_SDR2HDR:
		iris_sdr2hdr_mode = (value & 0xf00) >> 8;
		value = value & 0xff;
		if (value/10 == 4) {/*magic code to enable YUV input.*/
			iris_set_yuv_input(true);
			value -= 40;
		} else if (value/10 == 6) {
			iris_set_HDR10_YCoCg(true);
			value -= 60;
		} else if (value == 55) {
			iris_set_yuv_input(true);
			return 0;
		} else if (value == 56) {
			iris_set_yuv_input(false);
			return 0;
		} else {
			iris_set_yuv_input(false);
			iris_set_HDR10_YCoCg(false);
		}

		pqlt_cur_setting->pq_setting.sdr2hdr = value;
		if(pqlt_cur_setting->pq_setting.sdr2hdr > SDR709_2_2020)
			goto error;
		iris_sdr2hdr_level_set(pqlt_cur_setting->pq_setting.sdr2hdr);
		break;
	case IRIS_READING_MODE:
		pqlt_cur_setting->pq_setting.readingmode = value & 0x1;
		iris_reading_mode_set(pqlt_cur_setting->pq_setting.readingmode);
		break;
	case IRIS_COLOR_TEMP_VALUE:
		iris_configure_ex(type, 1, &value);
		break;
	case IRIS_CCT_VALUE:
		pqlt_cur_setting->cctvalue = value;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_AUTO)
			iris_cm_color_temp_set();
		break;
	case IRIS_LUX_VALUE:
		/* move to iris_configure_ex*/
		pqlt_cur_setting->luxvalue = value;
		if (pqlt_cur_setting->pq_setting.alenable == 1) {

			if (pqlt_cur_setting->pq_setting.sdr2hdr == SDR2HDR_Bypass)
				iris_lce_lux_set();
			else
				iris_ambient_light_lut_set();
		}
		break;
	case IRIS_HDR_MAXCLL:
		pqlt_cur_setting->maxcll = value;
		break;
	case IRIS_ANALOG_BYPASS_MODE:
		if (value == ANALOG_BYPASS_MODE && pqlt_cur_setting->pq_setting.sdr2hdr != SDR2HDR_Bypass) {
			iris_panel_nits_set(0, true, value);
			iris_quality_setting_off();
		}
		iris_abypass_switch_proc(pcfg->display, value, true);
		break;
	case IRIS_HDR_PANEL_NITES_SET:
		if (pqlt_cur_setting->al_bl_ratio != value) {
			pqlt_cur_setting->al_bl_ratio = value;
			iris_panel_nits_set(value, false, pqlt_cur_setting->pq_setting.sdr2hdr);
		}
		break;
	case IRIS_PEAKING_IDLE_CLK_ENABLE:
		iris_peaking_idle_clk_enable(value & 0x01);
		break;
	case IRIS_CM_MAGENTA_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_MAGENTA_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_CM_RED_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_RED_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_CM_YELLOW_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_YELLOW_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_CM_GREEN_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_GREEN_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_CM_BLUE_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_BLUE_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_CM_CYAN_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_CYAN_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_DBC_LED_GAIN:
		iris_dbc_led0d_gain_set(value & 0x3f);
		break;
	case IRIS_SCALER_FILTER_LEVEL:
		iris_scaler_filter_update(value & 0xf);
		break;
	case IRIS_HDR_PREPARE:
		if ((value == 0) || ((value == 1) && !iris_debug_cap))
			iris_hdr_csc_prepare();
		else if (value == 3)
			iris_set_skip_dma(true);
		break;
	case IRIS_HDR_COMPLETE:
		if ((value == 3) || (value == 5))
			iris_set_skip_dma(false);
		if ((value == 0) || ((value == 1) && !iris_debug_cap))
			iris_hdr_csc_complete(value);
		else if (value >= 2)
			iris_hdr_csc_complete(value);

		/*if (value != 4) {
			if (pqlt_cur_setting->pq_setting.sdr2hdr == SDR2HDR_Bypass)
				iris_panel_nits_set(0, true, value);
			else
				iris_panel_nits_set(PANEL_BL_MAX_RATIO, false, value);
		}*/
		break;
	case IRIS_DEBUG_CAP:
		iris_debug_cap = value & 0x01;
		break;
	case IRIS_FW_UPDATE:
		// Need do multi-thread protection.
		if (value == 0) {
			iris3_parse_lut_cmds(&pcfg->display->pdev->dev, IRIS3_FIRMWARE_NAME);
			iris_cm_color_gamut_set(pqlt_cur_setting->pq_setting.cmcolorgamut);
		} else if (value == 1) {
			iris3_parse_lut_cmds(&pcfg->display->pdev->dev, IRIS3_FIRMWARE_GS_NAME);
		} else if (value == 3) {
			iris3_parse_lut_cmds(&pcfg->display->pdev->dev, IRIS3_FIRMWARE_OEM_NAME);
		}
		break;
	case IRIS_DBG_KERNEL_LOG_LEVEL:
		iris_set_loglevel(value);
		break;
	default:
		goto error;
		break;
	}

	return 0;

error:
	return -EINVAL;

}

int iris_configure_t(u32 type, void __user *argp)
{
	int ret = -1;
	uint32_t value = 0;

	ret = copy_from_user(&value, argp, sizeof(uint32_t));
	if (ret) {
		IRIS_LOGE("can not copy from user");
		return -EPERM;
	}

	ret = iris_configure(type, value);
	return ret;
}

int iris_configure_ex(u32 type, u32 count, u32 *values)
{
	int ret = -1;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	struct msmfb_iris_ambient_info iris_ambient;
	struct msmfb_iris_maxcll_info iris_maxcll;
	uint8_t i=0;
	u32 TempValue=0;
	bool is_phone;

	IRIS_LOGD("%s type=0x%04x, count=%d, value=%d", __func__, type, count, values[0]);

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return -1;
	/* FIXME
	if (mfd->panel_power_state == MDSS_PANEL_POWER_OFF)
		return -1;
	*/
	if (pcfg->abypss_ctrl.abypass_mode == ANALOG_BYPASS_MODE)
		return -1;

	switch (type) {
		case IRIS_LUX_VALUE:

			iris_ambient = *(struct msmfb_iris_ambient_info *)(values);
			iris_ambient_lut.ambient_lux = iris_ambient.ambient_lux;
			pqlt_cur_setting->luxvalue= iris_ambient_lut.ambient_lux;

			if (iris_ambient.lut_lut2_payload != NULL) {
				ret = copy_from_user(iris_ambient_lut.lut_lut2_payload, iris_ambient.lut_lut2_payload, sizeof(uint32_t)*LUT_LEN);
				if (ret) {
					IRIS_LOGE("can not copy from user sdr2hdr");
					goto error1;
				}
				iris_ambient_lut_update(AMBINET_SDR2HDR_LUT);
			}

			if (pqlt_cur_setting->pq_setting.alenable == 1) {

				if (pqlt_cur_setting->pq_setting.sdr2hdr == SDR2HDR_Bypass)
					iris_lce_lux_set();
				else
					iris_ambient_light_lut_set();
			}
			break;
		case IRIS_HDR_MAXCLL:
			iris_maxcll = *(struct msmfb_iris_maxcll_info *)(values);
			iris_maxcll_lut.mMAXCLL= iris_maxcll.mMAXCLL;

			if (iris_maxcll.lut_luty_payload != NULL) {
				ret = copy_from_user(iris_maxcll_lut.lut_luty_payload, iris_maxcll.lut_luty_payload, sizeof(uint32_t)*LUT_LEN);
				if (ret) {
					IRIS_LOGE("can not copy lut y from user sdr2hdr");
					goto error1;
				}
			}
			if (iris_maxcll.lut_lutuv_payload != NULL) {
				ret = copy_from_user(iris_maxcll_lut.lut_lutuv_payload, iris_maxcll.lut_lutuv_payload, sizeof(uint32_t)*LUT_LEN);
				if (ret) {
					IRIS_LOGE("can not copy lut uv from user sdr2hdr");
					goto error1;
				}
			}
			iris_maxcll_lut_update(AMBINET_HDR_GAIN);
			iris_maxcll_lut_set();
			break;
		case IRIS_CCF1_UPDATE:
			iris_cm_lut_read(values[0], (u8*)&values[2]);
			if(values[1] == 1)   //last flag is 1
				iris_cm_color_gamut_set(pqlt_cur_setting->pq_setting.cmcolorgamut);
			break;
		case IRIS_CCF2_UPDATE:
			iris_gamma_lut_update(values[0], &values[2]);
			if(values[1] == 1)   //last flag is 1
				iris_dpp_gamma_set();
			break;
		case IRIS_HUE_SAT_ADJ:
			IRIS_LOGD("cm csc value: csc0 = 0x%x, csc1 = 0x%x, csc2 = 0x%x, csc3 = 0x%x, csc4 = 0x%x", values[0], values[1], values[2], values[3], values[4]);
			IRIS_LOGD("game mode %d", values[5]);
			if (values[5] == 1) {
				for(i=0;i<=4;i++) {
					if (pcfg->iris3_i2c_write) {
						if (pcfg->iris3_i2c_write(CM_CNTL[10] +i*4, values[i]) < 0) {
							IRIS_LOGE("i2c set reg fails, reg=0x%x, val=0x%x",  CM_CNTL[10] +i*4, values[i]);
						}
					} else {
						IRIS_LOGE("Game Station is not connected");
					}
				}
			} else {
				iris_cm_csc_level_set(IRIS_IP_CM, &values[0], NULL);
			}
			break;
		case IRIS_COLOR_TEMP_VALUE:
			is_phone = (count > 1) ? (values[1] == 0) : true;
			pqlt_cur_setting->colortempvalue = values[0];

			if (is_phone) {
				if (count > 3) {
					pqlt_cur_setting->min_colortempvalue = values[2];
					pqlt_cur_setting->max_colortempvalue = values[3];
				} else {
					pqlt_cur_setting->min_colortempvalue = 0;
					pqlt_cur_setting->max_colortempvalue = 0;
				}
				if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_MANUL)
					iris_cm_color_temp_set();
			} else {
				TempValue = iris_cm_ratio_set_for_iic();
				IRIS_LOGD("set reg=0x%x, val=0x%x", CM_CNTL[4], TempValue);
				if (pcfg->iris3_i2c_write) {
					if (pcfg->iris3_i2c_write(CM_CNTL[4], TempValue) < 0) {
						IRIS_LOGE("i2c set reg fails, reg=0x%x, val=0x%x",  CM_CNTL[4], TempValue);
					}
				} else {
					IRIS_LOGE("Game Station is not connected");
				}
			}
			break;
		case IRIS_DBG_TARGET_REGADDR_VALUE_SET:
			if (0 == adb_type) {
				iris_ocp_write(values[0], values[1]);
			} else if (1 == adb_type) {
				if (pcfg->iris3_i2c_write) {
					if (pcfg->iris3_i2c_write(values[0], values[1]) < 0) {
						IRIS_LOGE("i2c set reg fails, reg=0x%x, val=0x%x", values[0], values[1]);
					}
				} else {
					IRIS_LOGE("Game Station is not connected");
				}
			}
			break;
		case IRIS_DBG_TARGET_REGADDR_VALUE_SET2:
			iris_ocp_write2(values[0], values[1], count-2, values+2);
			break;
		case IRIS_CM_6AXES:
			// phone
			pqlt_cur_setting->pq_setting.cm6axis = values[0] & 0x3;
			iris_cm_6axis_level_set( pqlt_cur_setting->pq_setting.cm6axis );

			// game station
			if (pcfg->iris3_i2c_write) {
				if (pcfg->iris3_i2c_write(CM_CNTL[0], values[1] ? CM_CNTL[3] : CM_CNTL[1]) < 0) {
					IRIS_LOGE("i2c set reg fails, reg=0x%x", CM_CNTL[0]);
				} else if (pcfg->iris3_i2c_write(CM_CNTL[12], CM_CNTL[13]) < 0) {
					IRIS_LOGE("i2c set reg fails, reg=0x%x", CM_CNTL[12]);
				}
			} else {
				IRIS_LOGE("Game Station is not connected");
			}
			break;
		case IRIS_CM_COLOR_TEMP_MODE:
			// phone
			pqlt_cur_setting->pq_setting.cmcolortempmode= values[0] & 0x3;
			if ( pqlt_cur_setting->pq_setting.cmcolortempmode > 2 ) {
				goto error;
			}
			iris_cm_colortemp_mode_set( pqlt_cur_setting->pq_setting.cmcolortempmode );

			// game station
			if (pcfg->iris3_i2c_write) {
				if (pcfg->iris3_i2c_write(CM_CNTL[6], values[1] ? CM_CNTL[9] : CM_CNTL[7]) < 0) {
					IRIS_LOGE("i2c set reg fails, reg=0x%x", CM_CNTL[6]);
				} else if (pcfg->iris3_i2c_write(CM_CNTL[12], CM_CNTL[13]) < 0) {
					IRIS_LOGE("i2c set reg fails, reg=0x%x", CM_CNTL[12]);
				}
			} else {
				IRIS_LOGE("Game Station is not connected");
			}
			break;
		case IRIS_CSC_MATRIX:
			if (count > 9) {
				if (values[0] == 1)
					iris_cm_csc_level_set(IRIS_IP_CM, &values[2], &values[7]);
				else if (values[0] == 2)
					iris_cm_csc_level_set(IRIS_IP_DPP, &values[2], &values[7]);
				else
					return -EPERM;
			}
			else
				return -EPERM;
			break;
		default:
			goto error;
			break;

	}

	return 0;

	error:
		return -EINVAL;
	error1:
		return -EPERM;
}

static int iris_configure_ex_t(uint32_t type,
								uint32_t count, void __user *values)
{
	int ret = -1;
	uint32_t *val = NULL;

	val = kmalloc(sizeof(uint32_t) * count, GFP_KERNEL);
	if (!val) {
		IRIS_LOGE("can not kmalloc space");
		return -ENOSPC;
	}
	ret = copy_from_user(val, values, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("can not copy from user");
		kfree(val);
		return -EPERM;
	}
	ret = iris_configure_ex(type, count, val);
	kfree(val);
	return ret;
}

int iris_configure_get(u32 type, u32 count, u32 *values)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct quality_setting *pqlt_cur_setting = & iris_setting.quality_cur;

	int i;

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return -EINVAL;

	switch (type) {
	case IRIS_PEAKING:
		*values = pqlt_cur_setting->pq_setting.peaking;
		break;
	case IRIS_CM_6AXES:
		*values = pqlt_cur_setting->pq_setting.cm6axis;
		break;
	case IRIS_CM_FTC_ENABLE:
		*values = pqlt_cur_setting->pq_setting.cmftc;
		break;
	case IRIS_CM_COLOR_TEMP_MODE:
		*values = pqlt_cur_setting->pq_setting.cmcolortempmode;
		break;
	case IRIS_CM_COLOR_GAMUT:
		*values = pqlt_cur_setting->pq_setting.cmcolorgamut;
		break;
	case IRIS_LCE_MODE:
		*values = pqlt_cur_setting->pq_setting.lcemode;
		break;
	case IRIS_LCE_LEVEL:
		*values = pqlt_cur_setting->pq_setting.lcelevel;
		break;
	case IRIS_GRAPHIC_DET_ENABLE:
		*values = pqlt_cur_setting->pq_setting.graphicdet;
		break;
	case IRIS_AL_ENABLE:
		*values = pqlt_cur_setting->pq_setting.alenable;
		break;
	case IRIS_DBC_LEVEL:
		*values = pqlt_cur_setting->pq_setting.dbc;
		break;
	case IRIS_DEMO_MODE:
		*values = pqlt_cur_setting->pq_setting.demomode;
		break;
	case IRIS_SDR2HDR:
		*values = pqlt_cur_setting->pq_setting.sdr2hdr;
		break;
	case IRIS_LUX_VALUE:
		*values = pqlt_cur_setting->luxvalue;
		break;
	case IRIS_READING_MODE:
		*values = pqlt_cur_setting->pq_setting.readingmode;
		break;
	case IRIS_DYNAMIC_POWER_CTRL:
		*values = iris_dynamic_power_get();
		break;
	case IRIS_HDR_MAXCLL:
		*values = pqlt_cur_setting->maxcll;
		break;
	case IRIS_ANALOG_BYPASS_MODE:
		*values = pcfg->abypss_ctrl.abypass_mode;
		break;
	case IRIS_CM_COLOR_GAMUT_PRE:
		*values = pqlt_cur_setting->source_switch;
		break;
	case IRIS_CCT_VALUE:
		*values = pqlt_cur_setting->cctvalue;
		break;
	case IRIS_COLOR_TEMP_VALUE:
		*values = pqlt_cur_setting->colortempvalue;
		break;
	case IRIS_CHIP_VERSION:
		*values = 2;
		break;
	case IRIS_PANEL_TYPE:
		*values = pcfg->panel_type;
		break;
	case IRIS_PANEL_NITS:
		*values = pcfg->panel_nits;
		break;
	case IRIS_MCF_DATA:
		if (0 == values[1]) {
			// read via mipi
			/* MODIFIED-BEGIN by hongwei.tian, 2019-06-06,BUG-7808648*/
			for (i=0; i<count; i++) {
				values[i] = panel_mcf_data[i];
			}
			/* MODIFIED-END by hongwei.tian,BUG-7808648*/
		} else if (1 == values[1]) {
			if (0xC7 == values[0]) {
				for (i = 0; i < count; i++) {
					values[i] = panel_raw_data[i];
				}
			} else if (0xC8 == values[0]) {
				for (i = 0; i < count; i++) {
					values[i] = panel_raw_data[i+128];
				}
			}
		}
		break;
	case IRIS_DBG_TARGET_REGADDR_VALUE_GET:
		if (iris3_abypass_mode_get() == PASS_THROUGH_MODE) {
			if (0 == adb_type) {
				*values = iris_ocp_read(*values, DSI_CMD_SET_STATE_HS);
			} else if (1 == adb_type) {
				u32 reg_addr, reg_val;
				reg_addr = *values;
				if (pcfg->iris3_i2c_read) {
					if (pcfg->iris3_i2c_read(reg_addr, &reg_val) < 0) {
						IRIS_LOGE("i2c read reg fails, reg=0x%x", reg_addr);
					} else {
						*values = reg_val;
					}
				} else {
					IRIS_LOGE("Game Station is not connected");
				}
			}
		}
		break;
	case IRIS_DBG_KERNEL_LOG_LEVEL:
		*values = iris_get_loglevel();
		break;
	case IRIS_WORK_MODE:
		*values = ((int)pcfg->pwil_mode<<16) | ((int)pcfg->tx_mode<<8) | ((int)pcfg->rx_mode);
		break;
	/* MODIFIED-BEGIN by ji.chen, 2019-08-16,BUG-8256828*/
	case IRIS_POWER_MODE_CHECK:
		iris_read_power_mode(pcfg->panel);
		*values = pcfg->power_mode;
		break;
		/* MODIFIED-END by ji.chen,BUG-8256828*/
	default:
		return -EFAULT;
	}

	IRIS_LOGD("%s type=0x%04x, value=%d", __func__, type, *values);
	return 0;
}

int iris_configure_get_t(
				uint32_t type, uint32_t count, void __user *values)
{
	int ret = -1;
	uint32_t *val = NULL;

	val = kmalloc(count * sizeof(uint32_t), GFP_KERNEL);
	if (val == NULL) {
		IRIS_LOGE("could not kmalloc space for func = %s", __func__);
		return -ENOSPC;
	}
	ret = copy_from_user(val, values, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("can not copy from user");
		kfree(val);
		return -EPERM;
	}
	ret = iris_configure_get(type, count, val);
	if (ret) {
		IRIS_LOGE("get error");
		kfree(val);
		return ret;
	}
	ret = copy_to_user(values, val, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("copy to user error");
		kfree(val);
		return -EPERM;
	}
	kfree(val);
	return ret;
}

int iris3_operate_conf(struct msm_iris_operate_value *argp)
{
	int ret = -1;
	uint32_t parent_type = 0;
	uint32_t child_type = 0;
	ktime_t ktime0;
	uint32_t timeus0 = 0;
	struct iris_cfg * pcfg;

	pcfg = iris_get_cfg();

	parent_type = argp->type & 0xff;
	child_type = (argp->type >> 8) & 0xff;
	/*close panel*/
	if (pcfg->valid < IRIS_STATUS_MINI_LIGHTUP) {
		if (child_type == IRIS_DBG_KERNEL_LOG_LEVEL
			|| child_type == IRIS_WORK_MODE || child_type == IRIS_CHIP_VERSION)
			;
		else
			return -EINVAL;
	}

	ktime0 = ktime_get();
	switch (parent_type) {
	case IRIS_OPRT_CONFIGURE:
		ret = iris_configure_t(child_type, argp->values);
		break;
	case IRIS_OPRT_CONFIGURE_NEW:
		ret = iris_configure_ex_t(child_type, argp->count, argp->values);
		break;
	case IRIS_OPRT_CONFIGURE_NEW_GET:
		ret = iris_configure_get_t(child_type, argp->count, argp->values);
		break;
	default:
		IRIS_LOGE("could not find right operate type = %d", argp->type);
		break;
	}
	timeus0 = (u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime0);
	IRIS_LOGD("%s: configure parent_type=%d  child_type=%d spend time=%d\n",
			__func__, parent_type, child_type, timeus0);
	return ret;
}

static ssize_t iris_adb_type_read(struct file *file, char __user *buff,
                size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	tot = scnprintf(bp, sizeof(bp), "%d\n", adb_type);
	if (copy_to_user(buff, bp, tot))
	    return -EFAULT;
	*ppos += tot;

	return tot;
}

static ssize_t iris_adb_type_write(struct file *file,
	const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	adb_type = val;

	return count;
}

static const struct file_operations iris_adb_type_write_fops = {
	.open = simple_open,
	.write = iris_adb_type_write,
	.read = iris_adb_type_read,
};

int iris_adb_type_debugfs_init(struct dsi_display *display)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (debugfs_create_file("adb_type", 0644, pcfg->dbg_root, display,
				&iris_adb_type_write_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}
