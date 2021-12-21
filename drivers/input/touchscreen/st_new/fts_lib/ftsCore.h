/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				 **
  **************************************************************************
  *                                                                        *
  *			FTS Core definitions				 **
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsCore.h
  * \brief Contains all the definitions and structs of Core functionalities
  */

#ifndef FTS_CORE_H
#define FTS_CORE_H

#include "ftsHardware.h"
#include "ftsSoftware.h"
#include "../fts.h"

/* HW DATA */
#define GPIO_NOT_DEFINED	-1	/* /< value assumed by reset_gpio when
					 * the reset pin of the IC is not
					 * connected */


#define ADDR_SIZE_HW_REG	BITS_32	/* /< value of AddrSize for Hw register
					 * in FTI @see AddrSize */


#define DATA_HEADER		4	/* /< size in byte of the header loaded
					 * with the data in the frambuffer */

/**
  * Type of CRC errors
  */
typedef enum {
	CRC_CODE	= 1,	/* /< CRC in the code section */
	CRC_CONFIG	= 2,	/* /< CRC in the config section */
	CRC_CX		= 3,	/* /< CRC in the cx section */
	CRC_PANEL	= 4	/* /< CRC in the panel section */
} CRC_Error;

/* CHIP INFO */
/** @defgroup system_info	System Info
  * System Info Data collect the most important informations about hw and fw
  * @{
  */
/* Size in bytes of System Info data */
#define SYS_INFO_SIZE			216	/* Num bytes of die info */
#define DIE_INFO_SIZE			16	/* Num bytes of external release
						 * in config */
#define EXTERNAL_RELEASE_INFO_SIZE	8	/* Num bytes of release info in
						  * sys info
						  *  (first bytes are external
						  *release) */
#define RELEASE_INFO_SIZE		(EXTERNAL_RELEASE_INFO_SIZE)
/** @}*/

/* RETRY MECHANISM */
#define RETRY_MAX_REQU_DATA		2	/* /< Max number of attemps
						 * performed
						  * when requesting data */
#define RETRY_SYSTEM_RESET		3	/* /< Max number of attemps
						 * performed
						  * to reset the IC */

/** @addtogroup system_info
  * @{
  */

/**
  * Struct which contains fundamental informations about the chip and its
  *configuration
  */
typedef struct {
	uint16_t u16_apiVer_rev;	/* /< API revision version */
	uint8_t u8_apiVer_minor;	/* /< API minor version */
	uint8_t u8_apiVer_major;	/* /< API major version */
	uint16_t u16_chip0Ver;	/* /< Dev0 version */
	uint16_t u16_chip0Id;	/* /< Dev0 ID */
	uint16_t u16_chip1Ver;	/* /< Dev1 version */
	uint16_t u16_chip1Id;	/* /< Dev1 ID */
	uint16_t u16_fwVer;	/* /< Fw version */
	uint16_t u16_svnRev;	/* /< SVN Revision */
	uint16_t u16_cfgVer;	/* /< Config Version */
	uint16_t u16_cfgProjectId;	/* /< Config Project ID */
	uint16_t u16_cxVer;	/* /< Cx Version */
	uint16_t u16_cxProjectId;	/* /< Cx Project ID */
	uint8_t u8_cfgAfeVer;	/* /< AFE version in Config */
	uint8_t u8_cxAfeVer;	/* /< AFE version in CX */
	uint8_t u8_panelCfgAfeVer;	/* /< AFE version in PanelMem */
	uint8_t u8_protocol;	/* /< Touch Report Protocol */
	uint8_t u8_dieInfo[DIE_INFO_SIZE];	/* /< Die information */
	uint8_t u8_releaseInfo[RELEASE_INFO_SIZE];	/* /< Release information */
	uint32_t u32_fwCrc;	/* /< Crc of FW */
	uint32_t u32_cfgCrc;	/* /< Crc of config */
	uint8_t u8_mpFlag; /* /< MP Flag */
	uint8_t u8_ssDetScanSet; /* /< Type of Detect Scan Selected */

	uint16_t u16_scrResX;	/* /< X resolution on main screen */
	uint16_t u16_scrResY;	/* /< Y resolution on main screen */
	uint8_t u8_scrTxLen;	/* /< Tx length */
	uint8_t u8_scrRxLen;	/* /< Rx length */
	uint8_t u8_keyLen;	/* /< Key Len */
	uint8_t u8_forceLen;	/* /< Force Len */

	uint16_t u16_dbgInfoAddr;	/* /< Offset of debug Info structure */

	uint16_t u16_msTchRawAddr;	/* /< Offset of MS touch raw frame */
	uint16_t u16_msTchFilterAddr;	/* /< Offset of MS touch filter frame */
	uint16_t u16_msTchStrenAddr;	/* /< Offset of MS touch strength frame */
	uint16_t u16_msTchBaselineAddr;	/* /< Offset of MS touch baseline frame
					 * */

	uint16_t u16_ssTchTxRawAddr;	/* /< Offset of SS touch force raw frame */
	uint16_t u16_ssTchTxFilterAddr;	/* /< Offset of SS touch force filter
					 * frame */
	uint16_t u16_ssTchTxStrenAddr;	/* /< Offset of SS touch force strength
					 * frame */
	uint16_t u16_ssTchTxBaselineAddr;	/* /< Offset of SS touch force baseline
					 * frame */

	uint16_t u16_ssTchRxRawAddr;	/* /< Offset of SS touch sense raw frame */
	uint16_t u16_ssTchRxFilterAddr;	/* /< Offset of SS touch sense filter
					 * frame */
	uint16_t u16_ssTchRxStrenAddr;	/* /< Offset of SS touch sense strength
					 * frame */
	uint16_t u16_ssTchRxBaselineAddr;	/* /< Offset of SS touch sense baseline
					 * frame */

	uint16_t u16_keyRawAddr;	/* /< Offset of key raw frame */
	uint16_t u16_keyFilterAddr;	/* /< Offset of key filter frame */
	uint16_t u16_keyStrenAddr;	/* /< Offset of key strength frame */
	uint16_t u16_keyBaselineAddr;	/* /< Offset of key baseline frame */

	uint16_t u16_frcRawAddr;	/* /< Offset of force touch raw frame */
	uint16_t u16_frcFilterAddr;	/* /< Offset of force touch filter frame */
	uint16_t u16_frcStrenAddr;	/* /< Offset of force touch strength frame */
	uint16_t u16_frcBaselineAddr;	/* /< Offset of force touch baseline
					 * frame */

	uint16_t u16_ssHvrTxRawAddr;	/* /< Offset of SS hover Force raw frame */
	uint16_t u16_ssHvrTxFilterAddr;	/* /< Offset of SS hover Force filter
					 * frame */
	uint16_t u16_ssHvrTxStrenAddr;	/* /< Offset of SS hover Force strength
					 * frame */
	uint16_t u16_ssHvrTxBaselineAddr;	/* /< Offset of SS hover Force baseline
					 * frame */

	uint16_t u16_ssHvrRxRawAddr;	/* /< Offset of SS hover Sense raw frame */
	uint16_t u16_ssHvrRxFilterAddr;	/* /< Offset of SS hover Sense filter
					 * frame */
	uint16_t u16_ssHvrRxStrenAddr;	/* /< Offset of SS hover Sense strength
					 * frame */
	uint16_t u16_ssHvrRxBaselineAddr;	/* /< Offset of SS hover Sense baseline
					 * frame */

	uint16_t u16_ssPrxTxRawAddr;	/* /< Offset of SS proximity force raw frame */
	uint16_t u16_ssPrxTxFilterAddr;	/* /< Offset of SS proximity force
					 * filter frame */
	uint16_t u16_ssPrxTxStrenAddr;	/* /< Offset of SS proximity force
					 * strength frame */
	uint16_t u16_ssPrxTxBaselineAddr;	/* /< Offset of SS proximity force
					 * baseline frame */

	uint16_t u16_ssPrxRxRawAddr;	/* /< Offset of SS proximity sense raw frame */
	uint16_t u16_ssPrxRxFilterAddr;	/* /< Offset of SS proximity sense
					 * filter frame */
	uint16_t u16_ssPrxRxStrenAddr;	/* /< Offset of SS proximity sense
					 * strength frame */
	uint16_t u16_ssPrxRxBaselineAddr;	/* /< Offset of SS proximity sense
					 * baseline frame */

	uint16_t u16_ssDetRawAddr;		/* /< Offset of SS detect raw frame */
	uint16_t u16_ssDetFilterAddr;	/* /< Offset of SS detect filter
					 * frame */
	uint16_t u16_ssDetStrenAddr;		/* /< Offset of SS detect strength
					 * frame */
	uint16_t u16_ssDetBaselineAddr;	/* /< Offset of SS detect baseline
					 * frame */
} SysInfo;

/** @}*/

int initCore(struct fts_ts_info *info);
void setResetGpio(int gpio);
int fts_system_reset(void);
int isSystemResettedUp(void);
int isSystemResettedDown(void);
void setSystemResetedUp(int val);
void setSystemResetedDown(int val);
int pollForEvent(int *event_to_search, int event_bytes, uint8_t *readData, int
		 time_to_wait);
int checkEcho(uint8_t *cmd, int size);
int setScanMode(uint8_t mode, uint8_t settings);
int setFeatures(uint8_t feat, uint8_t *settings, int size);
int defaultSysInfo(int i2cError);
int writeSysCmd(uint8_t sys_cmd, uint8_t *sett, int size);
int readSysInfo(int request);
int readConfig(uint16_t offset, uint8_t *outBuf, int len);
int writeConfig(uint16_t offset, uint8_t *data, int len);
int fts_disableInterrupt(void);
int fts_disableInterruptNoSync(void);
int fts_resetDisableIrqCount(void);
int fts_enableInterrupt(void);
int fts_crc_check(void);
int requestSyncFrame(uint8_t type);
int saveMpFlag(uint8_t mpflag);

#endif	/* FTS_CORE_H */
