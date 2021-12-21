/* Copyright (C) 2019 Tcl Corporation Limited */
#ifndef _DT_BINDINGS_AB_SYSTEM_AB_SYSTEM_H
#define _DT_BINDINGS_AB_SYSTEM_AB_SYSTEM_H

#ifdef AB_SYSTEM_ENABLE
#define DEFINED_PARTS "vbmeta,boot,system,vendor,dtbo"
#define DEFINED_FSMGR_FLAGS "wait,slotselect,avb"
#else
#define DEFINED_PARTS "vbmeta,boot,system,vendor,dtbo,recovery"
#define DEFINED_FSMGR_FLAGS "wait,avb"
#endif

#endif
