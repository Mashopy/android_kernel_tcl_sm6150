/* Copyright (C) 2019 Tcl Corporation Limited */
#include <video/mipi_display.h>
#include "dsi_iris3_api.h"
#include "dsi_iris3_lightup.h"
#include "dsi_iris3_lightup_ocp.h"
#include "dsi_iris3_log.h"

#define DEBUG  false
// #undef IRIS_LOGI
// #define IRIS_LOGI IRIS_LOGE
// #undef IRIS_LOGD
// #define IRIS_LOGD IRIS_LOGE

#define IRIS_TX_HV_PAYLOAD_LEN   120
#define IRIS_TX_PAYLOAD_LEN 124
#define IRIS_PT_RD_CMD_NUM 3
#define IRIS_RD_PACKET_DATA  0xF0C1C018

static char iris_read_cmd_rbuf[16];
static struct iris_ocp_cmd ocp_cmd;
static struct iris_ocp_cmd ocp_test_cmd[DSI_CMD_CNT];
static struct dsi_cmd_desc iris_test_cmd[DSI_CMD_CNT];
u8 panel_raw_data[254] = {0};

static void iris_cmd_reg_add(struct iris_ocp_cmd *pcmd, u32 addr, u32 val)
{
	*(u32 *)(pcmd->cmd + pcmd->cmd_len) = cpu_to_le32(addr);
	*(u32 *)(pcmd->cmd + pcmd->cmd_len + 4) = cpu_to_le32(val);
	pcmd->cmd_len += 8;
}

static void iris_cmd_add(struct iris_ocp_cmd *pcmd, u32 payload)
{
	*(u32 *)(pcmd->cmd + pcmd->cmd_len) = cpu_to_le32(payload);
	pcmd->cmd_len += 4;
}

void iris_ocp_write(u32 address, u32 value)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0, CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL}, 1, 0}};

	memset(&ocp_cmd, 0, sizeof(ocp_cmd));

	iris_cmd_add(&ocp_cmd, 0xFFFFFFF0 | OCP_SINGLE_WRITE_BYTEMASK);
	iris_cmd_reg_add(&ocp_cmd, address, value);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;
	// TODO: use IRIS_LOGD after confirm features.
	IRIS_LOGD("[ocp][write]addr=0x%08x, value=0x%08x\n", address, value);

	iris3_dsi_cmds_send(pcfg->panel, iris_ocp_cmd, 1, DSI_CMD_SET_STATE_HS);
}

void iris_ocp_write2(u32 header, u32 address, u32 size, u32 *pvalues)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0, CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL}, 1, 0}};
	u32 max_size = CMD_PKT_SIZE / 4 - 2;
	u32 i;

	while (size > 0) {
		memset(&ocp_cmd, 0, sizeof(ocp_cmd));

		iris_cmd_add(&ocp_cmd, header);
		iris_cmd_add(&ocp_cmd, address);
		if (size < max_size) {
			for (i = 0; i < size; i++) {
				iris_cmd_add(&ocp_cmd, pvalues[i]);
			}
			size = 0;
		} else {
			for (i = 0; i < max_size; i++) {
				iris_cmd_add(&ocp_cmd, pvalues[i]);
			}
			address += max_size * 4;
			pvalues += max_size;
			size -= max_size;
		}
		iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;

		// TODO: use IRIS_LOGD after confirm features.
		IRIS_LOGE("[ocp][write2]header=0x%08x, addr=0x%08x, dlen=%zu\n", header, address, iris_ocp_cmd[0].msg.tx_len);

		iris3_dsi_cmds_send(pcfg->panel, iris_ocp_cmd, 1, DSI_CMD_SET_STATE_HS);
	}
}

void iris_ocp_write_address(u32 address, u32 mode)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0, CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL}, 1, 0}};

	// Send OCP command.
	memset(&ocp_cmd, 0, sizeof(ocp_cmd));
	iris_cmd_add(&ocp_cmd, OCP_SINGLE_READ);
	iris_cmd_add(&ocp_cmd, address);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;

	iris3_dsi_cmds_send(pcfg->panel, iris_ocp_cmd, 1, mode);
}

u32 iris_ocp_read_value(u32 mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	char pi_read[1] = {0x00};
	struct dsi_cmd_desc pi_read_cmd[] = {
		{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_REQ_ACK, 0, 0, sizeof(pi_read), pi_read, 0, NULL}, 1, 0}};
	u32 response_value;

	// Read response.
	memset(iris_read_cmd_rbuf, 0, sizeof(iris_read_cmd_rbuf));
	pi_read_cmd[0].msg.rx_len = 4;
	pi_read_cmd[0].msg.rx_buf = iris_read_cmd_rbuf;
	iris3_dsi_cmds_send(pcfg->panel, pi_read_cmd, 1, mode);
	IRIS_LOGD("read register %02x %02x %02x  %02x \n",
			iris_read_cmd_rbuf[0], iris_read_cmd_rbuf[1], iris_read_cmd_rbuf[2], iris_read_cmd_rbuf[3]);
	response_value = iris_read_cmd_rbuf[0] | (iris_read_cmd_rbuf[1] << 8) |
			(iris_read_cmd_rbuf[2] << 16) | (iris_read_cmd_rbuf[3] << 24);

	return response_value;
}

u32 iris_ocp_read(u32 address, u32 mode)
{
	u32 value = 0;

	iris_ocp_write_address(address, mode);

	value = iris_ocp_read_value(mode);

	IRIS_LOGD("[ocp][read]addr=0x%x, value=0x%x\n", address, value);
	return value;
}

void iris_write_test(struct dsi_panel *panel, u32 iris_addr, int ocp_type, u32 pkt_size)
{
	union iris_ocp_cmd_header ocp_header;
	struct dsi_cmd_desc iris_cmd = {
		{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0, CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL}, 1, 0};

	u32 test_value = 0xFFFF0000;
	memset(&ocp_header, 0, sizeof(ocp_header));
	ocp_header.header32 = 0xFFFFFFF0 | ocp_type;

	memset(&ocp_cmd, 0, sizeof(ocp_cmd));
	memcpy(ocp_cmd.cmd, &ocp_header.header32, OCP_HEADER);
	ocp_cmd.cmd_len = OCP_HEADER ;

	switch(ocp_type) {
		case OCP_SINGLE_WRITE_BYTEMASK:
		case OCP_SINGLE_WRITE_BITMASK:
			for (;ocp_cmd.cmd_len <= (pkt_size - 8);) {
				iris_cmd_reg_add(&ocp_cmd, iris_addr, test_value);
				test_value ++;
				}
			break;

		case OCP_BURST_WRITE:
			test_value = 0xFFFF0000;
			iris_cmd_reg_add(&ocp_cmd, iris_addr, test_value);
			if(pkt_size <= ocp_cmd.cmd_len)
				break;
			test_value ++;
			for (;ocp_cmd.cmd_len <= pkt_size - 4;) {
				iris_cmd_add(&ocp_cmd, test_value);
				test_value++;
			}
			break;
		default:
			break;

	}

	IRIS_LOGD("len=0x%x iris_addr=0x%x  test_value=0x%x \n", ocp_cmd.cmd_len, iris_addr, test_value);

	iris_cmd.msg.tx_len = ocp_cmd.cmd_len;

	iris3_dsi_cmds_send(panel, &iris_cmd, 1, DSI_CMD_SET_STATE_HS);

	iris_dump_packet(ocp_cmd.cmd, ocp_cmd.cmd_len);

}


void iris_write_test_muti_pkt(struct dsi_panel *panel, struct iris_ocp_dsi_tool_input * ocp_input)
{
	union iris_ocp_cmd_header ocp_header;
	u32 test_value = 0xFF000000;
	int iCnt = 0;

	u32 iris_addr, ocp_type, pkt_size, totalCnt;

	ocp_type = ocp_input->iris_ocp_type;
	test_value = ocp_input->iris_ocp_value;
	iris_addr = ocp_input->iris_ocp_addr;
	totalCnt = ocp_input->iris_ocp_cnt;
	pkt_size = ocp_input->iris_ocp_size;

	memset(iris_test_cmd, 0, sizeof(iris_test_cmd));
	memset(ocp_test_cmd, 0, sizeof(ocp_test_cmd));

	memset(&ocp_header, 0, sizeof(ocp_header));
	ocp_header.header32 = 0xFFFFFFF0 | ocp_type;

	switch(ocp_type) {
		case OCP_SINGLE_WRITE_BYTEMASK:
		case OCP_SINGLE_WRITE_BITMASK:

			for(iCnt = 0; iCnt < totalCnt; iCnt++) {

				memcpy(ocp_test_cmd[iCnt].cmd, &ocp_header.header32, OCP_HEADER);
				ocp_test_cmd[iCnt].cmd_len = OCP_HEADER;

				test_value = 0xFF000000;
				test_value = 0xFF000000 | (iCnt << 16);
				while (ocp_test_cmd[iCnt].cmd_len <= (pkt_size - 8)) {
					iris_cmd_reg_add(&ocp_test_cmd[iCnt], (iris_addr + iCnt*4), test_value);
					test_value ++;
				}

				iris_test_cmd[iCnt].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
				iris_test_cmd[iCnt].msg.tx_len= ocp_test_cmd[iCnt].cmd_len;
				iris_test_cmd[iCnt].msg.tx_buf = ocp_test_cmd[iCnt].cmd;
			}
			iris_test_cmd[totalCnt - 1].last_command = true;
			break;

		case OCP_BURST_WRITE:
			for(iCnt = 0; iCnt < totalCnt; iCnt++) {
				memcpy(ocp_test_cmd[iCnt].cmd, &ocp_header.header32, OCP_HEADER);
				ocp_test_cmd[iCnt].cmd_len = OCP_HEADER;
				test_value = 0xFF000000;
				test_value = 0xFF000000 | (iCnt << 16);

				iris_cmd_reg_add(&ocp_test_cmd[iCnt], (iris_addr + iCnt*4), test_value);
				//if(pkt_size <= ocp_test_cmd[iCnt].cmd_len)
				//	break;
				test_value ++;
				while(ocp_test_cmd[iCnt].cmd_len <= pkt_size - 4) {
					iris_cmd_add(&ocp_test_cmd[iCnt], test_value);
					test_value++;
				}

				iris_test_cmd[iCnt].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
				iris_test_cmd[iCnt].msg.tx_len = ocp_test_cmd[iCnt].cmd_len;
				iris_test_cmd[iCnt].msg.tx_buf = ocp_test_cmd[iCnt].cmd;

			}
			iris_test_cmd[totalCnt - 1].last_command = true;
			break;
		default:
			break;

	}

	IRIS_LOGD("%s totalCnt=0x%x iris_addr=0x%x  test_value=0x%x \n", __func__,totalCnt, iris_addr, test_value);

	iris3_dsi_cmds_send(panel, iris_test_cmd, totalCnt, DSI_CMD_SET_STATE_HS);

	for(iCnt = 0; iCnt < totalCnt; iCnt++)
		iris_dump_packet(ocp_test_cmd[iCnt].cmd, ocp_test_cmd[iCnt].cmd_len);

}


int iris3_dsi_cmds_send(struct dsi_panel *panel,
				struct dsi_cmd_desc *cmds,
				u32 count,
				enum dsi_cmd_set_state state)
{
	int rc = 0, i = 0;
	ssize_t len;
	const struct mipi_dsi_host_ops *ops = panel->host->ops;

	if (!panel || !panel->cur_mode)
		return -EINVAL;

	if (count == 0) {
		IRIS_LOGE("[%s] No commands to be sent for state\n",
			 panel->name);
		goto error;
	}

	for (i = 0; i < count; i++) {
		if (state == DSI_CMD_SET_STATE_LP)
			cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;

		if (cmds->last_command)
			cmds->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;

		len = ops->transfer(panel->host, &cmds->msg);

		iris_dump_packet((u8 *) cmds->msg.tx_buf, cmds->msg.tx_len);

		if (len < 0) {
			rc = len;
			IRIS_LOGE("failed to set cmds(%d), rc=%d\n", cmds->msg.type, rc);
			goto error;
		}
		if (cmds->post_wait_ms)
			usleep_range(cmds->post_wait_ms*1000,
					((cmds->post_wait_ms*1000)+10));
		cmds++;
	}
error:
	return rc;
}

static u32 iris_pt_split_pkt_cnt(int dlen)
{
	u32 sum = 1;

	if (dlen > IRIS_TX_HV_PAYLOAD_LEN)
		sum = (dlen - IRIS_TX_HV_PAYLOAD_LEN + IRIS_TX_PAYLOAD_LEN -1) / IRIS_TX_PAYLOAD_LEN + 1;
	return sum;
}


/*
* @Description: use to do statitics for cmds which should not less than 252
*               if the payload is out of 252, it will change to more than one cmds
the first payload need to be
	4 (ocp_header) + 8 (tx_addr_header + tx_val_header) + 2* payload_len (TX_payloadaddr + payload_len)<= 252
the sequence payloader need to be
	4 (ocp_header) + 2* payload_len (TX_payloadaddr + payload_len)<= 252
	so the first payload should be no more than 120 the second and sequence need to be no more than 124

* @Param: cmdset  cmds request
* @return: the cmds number need to split
*/
static u32 iris_pt_calc_cmds_num(struct dsi_panel_cmd_set *cmdset)
{
	u32 i = 0;
	u32 sum = 0;
	u32 dlen = 0;

	for (i = 0; i < cmdset->count; i++) {
		dlen = cmdset->cmds[i].msg.tx_len;

		sum += iris_pt_split_pkt_cnt(dlen);
	}
	return sum;
}

static u32 iris_pt_alloc_cmds_space(struct dsi_panel_cmd_set *cmdset,
		struct dsi_cmd_desc **ptx_cmds, struct iris_ocp_cmd **pocp_cmds)
{
	u32 cmds_cnt = 0;

	cmds_cnt = iris_pt_calc_cmds_num(cmdset);

	*ptx_cmds = (struct dsi_cmd_desc *) kmalloc(cmds_cnt *sizeof(**ptx_cmds), GFP_KERNEL);
	if (!(*ptx_cmds)) {
		IRIS_LOGE("can not kmalloc len = %lu\n", cmds_cnt * sizeof(**ptx_cmds));
		return -ENOMEM;
	}

	*pocp_cmds = (struct iris_ocp_cmd *)kmalloc(cmds_cnt * sizeof(**pocp_cmds), GFP_KERNEL);
	if (!(*pocp_cmds)) {
		IRIS_LOGE("can not kmalloc pocp cmds\n");
		kfree(*ptx_cmds);
		*ptx_cmds = NULL;
		return -ENOMEM;
	}
	return cmds_cnt;
}

static void iris_pt_init_tx_cmd_header(struct dsi_panel_cmd_set *cmdset,
		struct dsi_cmd_desc *dsi_cmd, union iris_mipi_tx_cmd_header *header)
{
	u8 dtype = 0;
	dtype = dsi_cmd->msg.type;

	memset(header, 0x00, sizeof(*header));
	header->stHdr.dtype = dtype;
	header->stHdr.linkState = (cmdset->state == DSI_CMD_SET_STATE_LP) ? 1 : 0;
}


static void iris_pt_set_cmd_header(union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd, bool is_write)
{
	u32 dlen = 0;
	u8 * ptr = NULL;

	if (dsi_cmd == NULL) {
		return;
	}

	dlen = dsi_cmd->msg.tx_len;

	if (is_write) {
		pheader->stHdr.writeFlag = 0x01;
	} else {
		pheader->stHdr.writeFlag = 0x00;
	}

	if (pheader->stHdr.longCmdFlag == 0) {
		ptr = (u8 *)dsi_cmd->msg.tx_buf;
		if (dlen == 1) {
			pheader->stHdr.len[0] = ptr[0];
		} else if (dlen == 2) {
			pheader->stHdr.len[0] = ptr[0];
			pheader->stHdr.len[1] = ptr[1];
		}
	} else {
		pheader->stHdr.len[0] = dlen & 0xff;
		pheader->stHdr.len[1] = (dlen >> 8) & 0xff;
	}
}

static void iris_pt_set_wrcmd_header(union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd)
{
	iris_pt_set_cmd_header(pheader, dsi_cmd, true);
}


static void iris_pt_set_rdcmd_header(union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd)
{
	iris_pt_set_cmd_header(pheader, dsi_cmd, false);
}

static void iris_pt_init_ocp_cmd(struct iris_ocp_cmd *pocp_cmd)
{
	union iris_ocp_cmd_header ocp_header;

	if (!pocp_cmd) {
		IRIS_LOGE("pocp_cmd is null\n");
		return;
	}

	memset(pocp_cmd, 0x00, sizeof(*pocp_cmd));
	ocp_header.header32 = 0xfffffff0 | OCP_SINGLE_WRITE_BYTEMASK;
	memcpy(pocp_cmd->cmd, &ocp_header.header32, OCP_HEADER);
	pocp_cmd->cmd_len = OCP_HEADER;
}

static void iris_add_tx_cmds(struct dsi_cmd_desc *ptx_cmd, struct iris_ocp_cmd *pocp_cmd, u8 wait)
{
	struct dsi_cmd_desc desc_init_val = {
		{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0, CMD_PKT_SIZE, NULL, 0, NULL}, 1, 0};

	memcpy(ptx_cmd, &desc_init_val, sizeof(struct dsi_cmd_desc));
	ptx_cmd->msg.tx_buf = pocp_cmd->cmd;
	ptx_cmd->msg.tx_len = pocp_cmd->cmd_len;
	ptx_cmd->post_wait_ms = wait;
}

static u32 iris_pt_short_write(struct iris_ocp_cmd *pocp_cmd,
				union iris_mipi_tx_cmd_header *pheader, struct dsi_cmd_desc *dsi_cmd)
{
	u32 sum = 1;
	pheader->stHdr.longCmdFlag= 0x00;

	iris_pt_set_wrcmd_header(pheader, dsi_cmd);

	IRIS_LOGD("%s, line%d, header=0x%4x\n", __func__, __LINE__, pheader->hdr32);
	iris_cmd_reg_add(pocp_cmd, IRIS_MIPI_TX_HEADER_ADDR, pheader->hdr32);

	return sum;
}

static u32 iris_pt_short_read(struct iris_ocp_cmd *pocp_cmd,
				union iris_mipi_tx_cmd_header *pheader, struct dsi_cmd_desc *dsi_cmd)
{
	u32 sum = 1;
	pheader->stHdr.longCmdFlag= 0x00;
	iris_pt_set_rdcmd_header(pheader, dsi_cmd);

	IRIS_LOGD("%s, line%d, header=0x%4x\n", __func__, __LINE__, pheader->hdr32);
	iris_cmd_reg_add(pocp_cmd, IRIS_MIPI_TX_HEADER_ADDR, pheader->hdr32);

	return sum;
}


static u32 iris_pt_split_pkt_len(u16 dlen, int sum, int k)
{
	u16 split_len = 0;

	if (k == 0) {
		split_len = dlen <  IRIS_TX_HV_PAYLOAD_LEN ? dlen : IRIS_TX_HV_PAYLOAD_LEN;
	} else if (k == sum -1) {
		split_len = dlen - IRIS_TX_HV_PAYLOAD_LEN - (k -1) * IRIS_TX_PAYLOAD_LEN;
	} else {
		split_len = IRIS_TX_PAYLOAD_LEN;
	}
	return split_len;
}

static void iris_pt_add_split_pkt_payload(struct iris_ocp_cmd *pocp_cmd,
				u8 * ptr, u16 split_len)
{
	u32 i = 0;
	union iris_mipi_tx_cmd_payload payload;

	memset(&payload, 0x00, sizeof(payload));
	for (i = 0; i < split_len; i += 4, ptr += 4) {
		if (i + 4 > split_len) {
			payload.pld32 = 0;
			memcpy(payload.p, ptr, split_len - i);
		} else {
			payload.pld32 = *(u32 *)ptr;
		}
		IRIS_LOGD("payload=0x%x\n", payload.pld32);
		iris_cmd_reg_add(pocp_cmd, IRIS_MIPI_TX_PAYLOAD_ADDR, payload.pld32);
	}
}

static u32 iris_pt_long_write(struct iris_ocp_cmd *pocp_cmd,
			union iris_mipi_tx_cmd_header *pheader, struct dsi_cmd_desc *dsi_cmd)
{
	u8 * ptr = NULL;
	u32 i = 0;
	u32 sum = 0;
	u16 dlen = 0;
	u32 split_len = 0;

	dlen = dsi_cmd->msg.tx_len;

	pheader->stHdr.longCmdFlag = 0x1;
	iris_pt_set_wrcmd_header(pheader, dsi_cmd);

	IRIS_LOGD("%s, line%d, header=0x%x\n", __func__, __LINE__, pheader->hdr32);
	iris_cmd_reg_add(pocp_cmd, IRIS_MIPI_TX_HEADER_ADDR, pheader->hdr32);

	ptr = (u8 *)dsi_cmd->msg.tx_buf;
	sum = iris_pt_split_pkt_cnt(dlen);

	while (i < sum){
		ptr += split_len;

		split_len = iris_pt_split_pkt_len(dlen, sum, i);

		iris_pt_add_split_pkt_payload(pocp_cmd + i, ptr, split_len);

		i++;
		if (i < sum) {
			iris_pt_init_ocp_cmd(pocp_cmd + i);
		}
	}
	return sum;
}

static u32 iris_pt_add_cmd(struct dsi_cmd_desc *ptx_cmd,
		struct iris_ocp_cmd *pocp_cmd, struct dsi_cmd_desc *dsi_cmd,
		struct dsi_panel_cmd_set *cmdset)
{
	u32 i = 0;
	u16 dtype = 0;
	u32 sum = 0;
	u8 wait = 0;
	union iris_mipi_tx_cmd_header header;

	iris_pt_init_tx_cmd_header(cmdset, dsi_cmd, &header);

	dtype = dsi_cmd->msg.type;
	switch (dtype) {
		case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
		case MIPI_DSI_DCS_READ:
		case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
		case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
			sum = iris_pt_short_read(pocp_cmd, &header, dsi_cmd);
			break;
		case MIPI_DSI_DCS_SHORT_WRITE:
		case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
		case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
		case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
		case MIPI_DSI_COMPRESSION_MODE:
		case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
			sum = iris_pt_short_write(pocp_cmd, &header, dsi_cmd);
			break;
		case MIPI_DSI_GENERIC_LONG_WRITE:
		case MIPI_DSI_DCS_LONG_WRITE:
		case MIPI_DSI_PPS:
			sum = iris_pt_long_write(pocp_cmd, &header,dsi_cmd);
			break;
		default:
			IRIS_LOGE("could not identify the type = %0x\n", dsi_cmd->msg.type);
			break;
	}

	for (i = 0; i < sum; i++) {
		wait = dsi_cmd->post_wait_ms;
		iris_add_tx_cmds(ptx_cmd + i, pocp_cmd + i, wait);
	}
	return sum;
}

static void iris_pt_send_cmds(struct dsi_panel *panel,
		struct dsi_cmd_desc *ptx_cmds, u32 cmds_cnt)
{
	struct dsi_panel_cmd_set panel_cmds;
	memset(&panel_cmds, 0x00, sizeof(panel_cmds));

	panel_cmds.cmds = ptx_cmds;
	panel_cmds.count = cmds_cnt;
	panel_cmds.state = DSI_CMD_SET_STATE_HS;
	iris3_dsi_cmds_send(panel, panel_cmds.cmds, panel_cmds.count, panel_cmds.state);

	if (IRIS_CONT_SPLASH_LK == iris_get_cont_splash_type()) {
		iris_print_cmds(panel_cmds.cmds, panel_cmds.count, panel_cmds.state);
	}
}


void iris_panel_cmd_passthrough_wr(struct dsi_panel *panel, struct dsi_panel_cmd_set *cmdset)
{
	u32 i = 0;
	u32 j = 0;
	u32 cmds_cnt = 0;
	u32 offset = 0;
	struct iris_ocp_cmd *pocp_cmds = NULL;
	struct dsi_cmd_desc *ptx_cmds = NULL;
	struct dsi_cmd_desc *dsi_cmds = NULL;

	if (!panel || !cmdset) {
		IRIS_LOGE("cmdset is null\n");
		return;
	}

	cmds_cnt =  iris_pt_alloc_cmds_space(cmdset, &ptx_cmds, &pocp_cmds);

	for(i = 0; i < cmdset->count; i++){
		/*initial val*/
		dsi_cmds = cmdset->cmds + i;

		iris_pt_init_ocp_cmd(pocp_cmds + j);

		offset = iris_pt_add_cmd(ptx_cmds + j, pocp_cmds + j, dsi_cmds, cmdset);
		j += offset;
	}

	if (j != cmds_cnt) {
		IRIS_LOGE("cmds cnt is not right real cmds_cnt = %d j = %d\n", cmds_cnt, j);
	} else {
		iris_pt_send_cmds(panel, ptx_cmds, cmds_cnt);
	}

	kfree(pocp_cmds);
	kfree(ptx_cmds);
	pocp_cmds = NULL;
	ptx_cmds = NULL;
}

static void iris_pt_switch_cmd(struct dsi_panel *panel,
			struct dsi_panel_cmd_set *cmdset, struct dsi_cmd_desc *dsi_cmd)
{
	if (!cmdset || !panel || !dsi_cmd) {
		IRIS_LOGE("%s there have null pointer \n", __func__);
		return;
	}

	cmdset->cmds = dsi_cmd;
	cmdset->count = 1;
	// FIXME
	// cmdreq->flags |= CMD_REQ_COMMIT;

	// FIXME: support DMA_TPG
	// if((dsi_cmd->dchdr.dlen < DMA_TPG_FIFO_LEN)
	// 	&& (ctrl->shared_data->hw_rev >= MDSS_DSI_HW_REV_103) )
	// 	cmdreq->flags |= CMD_REQ_DMA_TPG;
}

static int iris_pt_write_max_ptksize(struct dsi_panel *panel,
			struct dsi_panel_cmd_set *cmdset)
{
	u32 rlen = 0;
	struct dsi_panel_cmd_set local_cmdset;
	static char max_pktsize[2] = {0x00, 0x00}; /* LSB tx first, 10 bytes */
	static struct dsi_cmd_desc pkt_size_cmd = {
		{0, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, MIPI_DSI_MSG_REQ_ACK, 0, 0, sizeof(max_pktsize), max_pktsize, 0, NULL}, 1, 0};

	rlen = cmdset->cmds[0].msg.rx_len;
	if (rlen > 128) {
		IRIS_LOGE("dlen = %d  > 128\n", rlen);
		return -EINVAL;
	}

	max_pktsize[0] = (rlen & 0xFF);

	memset(&local_cmdset, 0x00, sizeof(local_cmdset));

	iris_pt_switch_cmd(panel, &local_cmdset, &pkt_size_cmd);

	iris_panel_cmd_passthrough_wr(panel, &local_cmdset);

	return 0;
}


static void iris_pt_write_rdcmd_to_panel(struct dsi_panel *panel,
			struct dsi_panel_cmd_set *cmdset)
{
	struct dsi_panel_cmd_set local_cmdset;
	struct dsi_cmd_desc *dsi_cmd = NULL;

	dsi_cmd = cmdset->cmds;

	memset(&local_cmdset, 0x00, sizeof(local_cmdset));

	iris_pt_switch_cmd(panel, &local_cmdset, dsi_cmd);

	//passthrough write to panel
	iris_panel_cmd_passthrough_wr(panel, &local_cmdset);
}

static int iris_pt_remove_resp_header(char *ptr, int *offset)
{
	int rc = 0;
	char cmd;

	if (! ptr)
		return -EINVAL;

	cmd = ptr[0];
	switch (cmd) {
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		IRIS_LOGD("%s: rx ACK_ERR_REPORT\n", __func__);
		rc = -EINVAL;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		*offset = 1;
		rc = 1;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		*offset = 1;
		rc = 2;
		break;
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		*offset = 4;
		rc = ptr[1];
		break;
	default:
		rc = 0;
	}

	return rc;
}


static void iris_pt_read_value(struct dsi_panel_cmd_set *cmdset)
{
	u32 i = 0;
	u32 rlen = 0;
	u32 offset = 0;
	union iris_mipi_tx_cmd_payload val;
	u8 * rbuf = NULL;

	rbuf = (u8 *)cmdset->cmds[0].msg.rx_buf;
	rlen = cmdset->cmds[0].msg.rx_len;

	if (!rbuf || rlen <= 0) {
		IRIS_LOGE("rbuf %p  rlen =%d\n", rbuf, rlen);
		return;
	}

	//read iris for data
	val.pld32 = iris_ocp_read(IRIS_RD_PACKET_DATA, cmdset->state);

	rlen = iris_pt_remove_resp_header(val.p, &offset);
	//IRIS_LOGD("rlen = %d \n", rlen);

	if (rlen <= 0){
		IRIS_LOGE("do not return value\n");
		return;
	}

	if (rlen <= 2) {
		for (i = 0; i < rlen; i++) {
			rbuf[i] = val.p[offset + i];
		}
	} else {
		int j = 0;
		int len = 0;
		int num = (rlen + 3) / 4;

		for (i = 0; i < num; i++) {
			len = (i == num -1) ? rlen - 4 * i : 4;

			val.pld32= iris_ocp_read(IRIS_RD_PACKET_DATA, DSI_CMD_SET_STATE_HS);
			for (j = 0; j < len; j++) {
				rbuf[i * 4 + j] = val.p[j];
			}
		}
	}
}

void iris_panel_cmd_passthrough_rd(struct dsi_panel *panel, struct dsi_panel_cmd_set *cmdset)
{
	struct iris_cfg *pcfg = NULL;
	struct dsi_display * display = NULL;

	IRIS_LOGD("enter rd commands");

	if (!panel || !cmdset || cmdset->count != 1) {
		IRIS_LOGE("cmdset is error cmdset = %p \n", cmdset);
		return;
	}

	pcfg = iris_get_cfg();
	display = pcfg->display;

	dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_ON);

	//step1  write max packket size
	iris_pt_write_max_ptksize(panel, cmdset);

	//step2 write read cmd to panel
	iris_pt_write_rdcmd_to_panel(panel, cmdset);

	//step3 read panel data
	iris_pt_read_value(cmdset);

	dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_OFF);
}


int iris3_panel_cmd_passthrough(struct dsi_panel *panel, struct dsi_panel_cmd_set *cmdset)
{
	struct iris_cfg *pcfg = NULL;
	if (!cmdset || !panel) {
		IRIS_LOGE("cmdset = %p  panel = %p\n", cmdset, panel);
		return -1;
	}

	pcfg = iris_get_cfg();
	mutex_lock(&pcfg->mutex);
	if (cmdset->count == 1 && (cmdset->cmds[0].msg.type & MIPI_DSI_DCS_READ)) { // MODIFIED by hongwei.tian, 2019-06-06,BUG-7808648
		iris_panel_cmd_passthrough_rd(panel, cmdset);
	} else {
		iris_panel_cmd_passthrough_wr(panel, cmdset);
	}
	mutex_unlock(&pcfg->mutex);

	return 0;
}

static int iris_i2c_read_value(u8 *rbuf, u32 *rlen)
{
	u32 i = 0;
	u32 offset = 0;
	struct iris_cfg *pcfg = NULL;
	union iris_mipi_tx_cmd_payload val;

	if (!rbuf || *rlen <= 0) {
		IRIS_LOGE("rbuf %p  rlen=%d\n", rbuf, *rlen);
		return 1;
	}

	pcfg = iris_get_cfg();

	//read iris for data via i2c
	if (pcfg->iris3_i2c_read(IRIS_RD_PACKET_DATA, &val.pld32) < 0) {
		IRIS_LOGE("[iris3] i2c read reg fails, reg=0x%x\n", IRIS_RD_PACKET_DATA);
		return 1;
	}

	*rlen = iris_pt_remove_resp_header(val.p, &offset);
	IRIS_LOGD("rlen = %d \n", *rlen);

	if (*rlen <= 0){
		IRIS_LOGE("i2c do not return value\n");
		return 0;
	}

	if (*rlen <= 2) {
		for (i = 0; i < *rlen; i++) {
			rbuf[i] = val.p[offset + i];
		}
	} else {
		int j = 0;
		int len = 0;
		int num = (*rlen + 3) / 4;

		for (i = 0; i < num; i++) {
			len = (i == num -1) ? *rlen - 4 * i : 4;

			if (pcfg->iris3_i2c_read(IRIS_RD_PACKET_DATA, &val.pld32) < 0) {
				IRIS_LOGE("[iris3] i2c read reg fails, reg=0x%x\n", IRIS_RD_PACKET_DATA);
				return 1;
			}

			for (j = 0; j < len; j++) {
				rbuf[i * 4 + j] = val.p[j];
			}
		}
	}

	return 0;
}

int iris3_i2c_read_panel_data( u32 reg_addr, u32 size)
{
	u32 rsize;
	char read_rbuf[256] = {0};
	struct iris_cfg *pcfg = NULL;
	union iris_mipi_tx_cmd_payload val;

	pcfg = iris_get_cfg();

	if ((NULL == pcfg->iris3_i2c_write) || (NULL == pcfg->iris3_i2c_read)) {
		IRIS_LOGE("Game Station is not connected\n");
		return -1;
	}

	// Use bank0
	if (pcfg->iris3_i2c_write(IRIS_MIPI_TX_HEADER_ADDR , 0x0100b015) < 0) {
		IRIS_LOGE("[iris3] i2c set reg fails, reg=0x%x, val=0x%x\n", IRIS_MIPI_TX_HEADER_ADDR, 0x0100b015);
		return -1;
	}

	// Set max packet size
	if(size > 128) {
		IRIS_LOGE("size = %d  > 128\n", size);
		return -EINVAL;
	}
	val.pld32 = 0x03000037;
	val.p[1] = size & 0xff;
	IRIS_LOGD("set max packet size: val.pld32=0x%x\n", val.pld32);
	if (pcfg->iris3_i2c_write(IRIS_MIPI_TX_HEADER_ADDR , val.pld32) < 0) {
		IRIS_LOGE("[iris3] i2c set reg fails, reg=0x%x, val=0x%x\n", IRIS_MIPI_TX_HEADER_ADDR, val.pld32);
		return -1;
	}

	// Set panel read address
	val.pld32 = 0x02000006;
	val.p[1] = reg_addr;
	IRIS_LOGD("set panel read address:val.pld32=0x%x\n", val.pld32);
	if (pcfg->iris3_i2c_write(IRIS_MIPI_TX_HEADER_ADDR ,  val.pld32) < 0) {
		IRIS_LOGE("[iris3] i2c set reg fails, reg=0x%x, val=0x%x\n", IRIS_MIPI_TX_HEADER_ADDR, val.pld32 );
		return -1;
	}

	// wait before mipi read back data become ready
	usleep_range(20000, 20000);

	// Start to read panel data
	rsize = size;
	if (iris_i2c_read_value(read_rbuf, &rsize)) {
		IRIS_LOGE("[iris3] i2c read reg fails\n");
		return -1;
	}

	if (rsize != size) {
		IRIS_LOGE("[iris3] read return size is not what we want:rsize=%d,size=%d\n", rsize, size);
		return -1;
	} else {
		if (0xC7 == reg_addr) {
			memcpy(panel_raw_data, read_rbuf, rsize);
		} else if (0xC8 == reg_addr) {
			memcpy(panel_raw_data+128, read_rbuf, rsize);
		}
	}

	return 0;
}
EXPORT_SYMBOL(iris3_i2c_read_panel_data);

void iris3_i2c_cb_register(iris3_i2c_read_cb read_cb, iris3_i2c_write_cb write_cb, iris3_i2c_burst_write_cb burst_write_cb)
{
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();

	mutex_lock(&pcfg->gs_mutex);
	pcfg->iris3_i2c_read = read_cb;
	pcfg->iris3_i2c_write = write_cb;
	pcfg->iris3_i2c_burst_write = burst_write_cb;

	if (NULL == read_cb) {
		cancel_work_sync(&pcfg->lut_update_work);
	}
	mutex_unlock(&pcfg->gs_mutex);
}
EXPORT_SYMBOL(iris3_i2c_cb_register);
