/* Copyright (C) 2019 Tcl Corporation Limited */
#ifndef _DSI_IRIS3_DEF_H_
#define _DSI_IRIS3_DEF_H_

#define IRIS3_FIRMWARE_NAME     "iris3.fw"
#define IRIS3_FIRMWARE_GS_NAME  "iris3_gs.fw"
#define IRIS3_FIRMWARE_OEM_NAME "iris3_oem.fw"
#define IRIS3_FIRMWARE_DEFAULT_NAME  "iris3_default.fw" // MODIFIED by ji.chen, 2019-08-01,BUG-8195210

enum {
	IRIS_IP_SYS = 0x00,
	IRIS_IP_RX = 0x01,
	IRIS_IP_TX = 0x02,
	IRIS_IP_PWIL = 0x03,
	IRIS_IP_DPORT = 0x04,
	IRIS_IP_DTG = 0x05,
	IRIS_IP_PWM = 0x06,
	IRIS_IP_DSC_DEN = 0x07,
	IRIS_IP_DSC_ENC = 0x08,
	IRIS_IP_SDR2HDR = 0x09,
	IRIS_IP_CM = 0x0a,
	IRIS_IP_SCALER1D = 0x0b,
	IRIS_IP_PEAKING = 0x0c,
	IRIS_IP_LCE = 0x0d,
	IRIS_IP_DPP = 0x0e,
	IRIS_IP_DBC = 0x0f,
	IRIS_IP_EXT =0x10,
	IRIS_IP_DMA =0x11,
};



struct iris_pq_setting {
	u32 peaking:4;
	u32 cm6axis:2;
	u32 cmftc:1;
	u32 cmcolortempmode:2;
	u32 cmcolorgamut:4;
	u32 lcemode:2;
	u32 lcelevel:3;
	u32 graphicdet:1;
	u32 alenable:1;
	u32 dbc:2;
	u32 demomode:3;
	u32 sdr2hdr:4;
	u32 readingmode:3;
};


struct quality_setting {
	struct iris_pq_setting pq_setting;
	u32 cctvalue;
	u32 colortempvalue;
	u32 luxvalue;
	u32 maxcll;
	u32 source_switch;
	u32 al_bl_ratio;
	u32 system_brightness;
	u32 min_colortempvalue;
	u32 max_colortempvalue;
};

struct iris_setting_info {
	struct quality_setting quality_cur;
	struct quality_setting quality_def;
};

enum {
	IRIS_CONT_SPLASH_LK = 1,
	IRIS_CONT_SPLASH_KERNEL,
	IRIS_CONT_SPLASH_NONE,
	IRIS_CONT_SPLASH_BYPASS,
};

enum iris_abypass_status {
	PASS_THROUGH_MODE = 0,
	ANALOG_BYPASS_MODE,
	MAX_MODE = 255,
};

#endif // _DSI_IRIS3_DEF_H_
