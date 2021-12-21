/* Copyright (C) 2019 Tcl Corporation Limited */
#ifndef __SOUND_TFA_MMI_H
#define __SOUND_TFA_MMI_H

unsigned int tfa_get_mi2s_interface(void);
void msm_set_enable_clk_cb(int (*tfa_ext_sclk)(int enable));

#endif
