/* Copyright (C) 2019 Tcl Corporation Limited */
#ifndef _DSI_IRIS3_LUT_H_
#define _DSI_IRIS3_LUT_H_

int iris_cm_lut_read(u32 lut_table_index, const u8 *fw_data);

void iris_ambient_lut_update(enum LUT_TYPE lutType);
void iris_maxcll_lut_update(enum LUT_TYPE lutType);

int iris_gamma_lut_update(u32 lut_table_index, const u32 *fw_data);

#endif // _DSI_IRIS3_LUT_H_
