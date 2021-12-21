/* Copyright (C) 2019 Tcl Corporation Limited */
#ifndef _DSI_IRIS3_IOCTL_H_
#define _DSI_IRIS3_IOCTL_H_

int iris_configure(u32 type, u32 value);
int iris_configure_ex(u32 type, u32 count, u32 *values);
int iris_configure_get(u32 type, u32 count, u32 *values);
int iris_adb_type_debugfs_init(struct dsi_display *display);

#endif // _DSI_IRIS3_IOCTL_H_
