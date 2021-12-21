/* Copyright (C) 2016 Tcl Corporation Limited */
#ifdef CONFIG_SND_SOC_TFA98XX_MMI_TEST

#define DEBUG


#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>

#include "tfa98xx_tfafieldnames.h"
#include "tfa_internal.h"
#include "tfa.h"
#include "tfa_service.h"
#include "tfa_container.h"
#include "tfa98xx_parameters.h"
#include "tfa98xx_genregs_N1C.h"
#include "tfa98xx.h"
#include <sound/tfa_mmi_test.h>

struct speaker_model_s {
	char name[64];
	unsigned int model_id;
	char  path[256];
	int cal_lohm;	/*float to inter (* coefficient) */
	int cal_rohm;	/*float to inter (* coefficient) */
	int   lf0;
	int   rf0;
	int min_lohm;	/*float to inter (* coefficient) */
	int max_lohm;	/*float to inter (* coefficient) */
	int  min_rohm;	/*float to inter (* coefficient) */
	int max_rohm;	/*float to inter (* coefficient) */
	int   min_lf0;
	int   max_lf0;
	int   min_rf0;
	int   max_rf0;
};

typedef struct SPKRBST_SpkrModel {
        double pFIR[128];       /* Pointer to Excurcussion  Impulse response or
                                   Admittance Impulse response (reversed order!!) */
        int Shift_FIR;          /* Exponent of HX data */
        float leakageFactor;    /* Excursion model integration leakage */
        float ReCorrection;     /* Correction factor for Re */
        float xInitMargin;      /*(1)Margin on excursion model during startup */
        float xDamageMargin;    /* Margin on excursion modelwhen damage has been detected */
        float xMargin;          /* Margin on excursion model activated when LookaHead is 0 */
        float Bl;               /* Loudspeaker force factor */
        int fRes;               /*(1)Estimated Speaker Resonance Compensation Filter cutoff frequency */
        int fResInit;           /* Initial Speaker Resonance Compensation Filter cutoff frequency */
        float Qt;               /* Speaker Resonance Compensation Filter Q-factor */
        float xMax;             /* Maximum excursion of the speaker membrane */
        float tMax;             /* Maximum Temperature of the speaker coil */
        float tCoefA;           /*(1)Temperature coefficient */
} SPKRBST_SpkrModel_t;          /* (1) this value may change dynamically */

enum {
	SPK_BOX_ID_DEFAULT = 0x20160422,
};

typedef enum Tfa98xx_Error Tfa98xx_Error_t;
typedef unsigned char Tfa98xx_SpeakerParameters_t[TFA2_SPEAKERPARAMETER_LENGTH];

#define TFA98XX_NOMINAL_IMPEDANCE_MIN     600
#define TFA98XX_NOMINAL_IMPEDANCE_MAX     900

#define SPKBOX_TEST_MIN     600
#define SPKBOX_TEST_MAX     800

#define PARAM_GET_LSMODEL           0x86        // Gets current LoudSpeaker impedance Model.
#define PARAM_GET_LSMODELW            0xC1        // Gets current LoudSpeaker xcursion Model.
#define LOCATION_FILES "/system/etc/firmware/"

#define TFA98XX_KEY2_PROTECTED_SPKR_CAL_MTP  0x80
/* MTP bits */
/* one time calibration */
#define TFA_MTPOTC_POS          TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC_POS /**/
/* calibration done executed */
#define TFA_MTPEX_POS           TFA98XX_KEY2_PROTECTED_MTP0_MTPEX_POS /**/
#define TFA98XX_MTP                TFA98XX_KEY2_PROTECTED_SPKR_CAL_MTP

#define FLOAT_TO_INTER_COEF 100
#define COEFF_PRECISION 100000
#define TFA98XX_RESONANCE_CHECK_TIMES 5

extern int tfa98xx_ext_reset(struct tfa98xx *tfa98xx);
//extern struct tfa98xx *tfa98xx_devices[];
extern struct tfa98xx *ext_tfa98xx;
int (*tfa_ext_sclk_enable)(int) = NULL;


static struct mutex tfa_test_mutex;
static char res_status[10] = "not start";
static char cal_status[10] = "not start";

static struct speaker_model_s speaker_model = {
	"TFA9891", SPK_BOX_ID_DEFAULT, LOCATION_FILES, \
	0, 0, \
	-1, -1, \
	TFA98XX_NOMINAL_IMPEDANCE_MIN, TFA98XX_NOMINAL_IMPEDANCE_MAX,\
	TFA98XX_NOMINAL_IMPEDANCE_MIN, TFA98XX_NOMINAL_IMPEDANCE_MAX,\
	SPKBOX_TEST_MIN, SPKBOX_TEST_MAX,\
 	SPKBOX_TEST_MIN, SPKBOX_TEST_MAX
};

void msm_set_enable_clk_cb(int (*tfa_ext_sclk)(int enable))
{
	tfa_ext_sclk_enable = tfa_ext_sclk;
}
EXPORT_SYMBOL_GPL(msm_set_enable_clk_cb);

static int nxpTfa98xxGetF0(struct tfa_device *tfa, int model, int *f0, int *fResInit)
{
	Tfa98xx_Error_t error = Tfa98xx_Error_Ok;
	unsigned char bytes[3 * 141];
	int data[141];
	int i = 0;
	SPKRBST_SpkrModel_t srecord;
	SPKRBST_SpkrModel_t *record = &srecord;

	error = tfa_dsp_cmd_id_write_read(tfa,MODULE_SPEAKERBOOST, model ? PARAM_GET_LSMODELW : PARAM_GET_LSMODEL, 423, bytes);
	if (error != Tfa98xx_Error_Ok) {
		pr_err(" %s %d Tfa98xx_DspGetParam failed\n", __func__, __LINE__);
	}

	tfa98xx_convert_bytes2data(sizeof(bytes), bytes, data);

	for (i = 0; i < 128; i++) {
		/*record->pFIR[i] = (double)data[i] / ((1 << 23) * 2);*/
		record->pFIR[i] = (double)data[i] / (1 << 22);
	}

	record->Shift_FIR = data[i++];   ///< Exponent of HX data
	record->leakageFactor = (float)data[i++] / (1 << (23));  ///< Excursion model integration leakage
	record->ReCorrection = (float)data[i++] / (1 << (23));   ///< Correction factor for Re
	record->xInitMargin = (float)data[i++] / (1 << (23 - 2));        ///< (can change) Margin on excursion model during startup
	record->xDamageMargin = (float)data[i++] / (1 << (23 - 2));      ///< Margin on excursion modelwhen damage has been detected
	record->xMargin = (float)data[i++] / (1 << (23 - 2));    ///< Margin on excursion model activated when LookaHead is 0
	record->Bl = (float)data[i++] / (1 << (23 - 2)); ///< Loudspeaker force factor
	record->fRes = data[i++];        ///< (can change) Estimated Speaker Resonance Compensation Filter cutoff frequency
	record->fResInit = data[i++];    ///< Initial Speaker Resonance Compensation Filter cutoff frequency
	record->Qt = (float)data[i++] / (1 << (23 - 6)); ///< Speaker Resonance Compensation Filter Q-factor
	record->xMax = (float)data[i++] / (1 << (23 - 7));       ///< Maximum excursion of the speaker membrane
	record->tMax = (float)data[i++] / (1 << (23 - 9));       ///< Maximum Temperature of the speaker coil
	record->tCoefA = (float)data[i++] / (1 << 23);   ///< (can change) Temperature coefficient

	*f0 = record->fRes;
	*fResInit = record->fResInit;

	return error;
}
#define SWAP(a, b, t)	((t) = (a), (a) = (b), (b) = (t))

static void bsort(int a[], int n)
{
	int i, j;
	int t = 0;

	for (i = 0; i < n; i++) {
		for (j = 1; j < n - i; j++) {
			if (a[j - 1] > a[j]) {
				SWAP(a[j - 1], a[j], t);
			}
		}
	}
}

/*
 * because mediaplayer play pink noise in here,
 * no need enable sclk, sleep 8s
 * */
int tfa98xx_speaker_resonance(int model)
{
	int i;
	int rc;
	enum tfa_error err;
	int topavgF0, btmavgF0;
	int topFres[TFA98XX_RESONANCE_CHECK_TIMES];
	int btmFres[TFA98XX_RESONANCE_CHECK_TIMES];
	int dev = 0;
	int fResInitTop = 0;
	int fResInitBtm = 0;
	struct tfa98xx *tfa98xx = ext_tfa98xx;


	msleep(8000);

	err = tfa_dev_start(tfa98xx->tfa, tfa98xx->profile, tfa98xx->vstep);
	if (err) {
			pr_err("%s %d open dev %d error\n", __func__, __LINE__, dev);
			return err;
	}

	/*
	* get F0 from top and btm
	* */
	for (i = 0; i < TFA98XX_RESONANCE_CHECK_TIMES; i++) {
		rc = nxpTfa98xxGetF0(tfa98xx->tfa, model, &topFres[i], &fResInitTop);

		pr_debug("%s  fResInitTop = %d, fResInitBtm = %d\n", __func__, fResInitTop, fResInitBtm);
		msleep(1000);
	}

	//FIXME BRUCE
	for (i = 0; i < TFA98XX_RESONANCE_CHECK_TIMES; i++)
		pr_debug("%s RAW top F0[%d] = %d\n", __func__, i, topFres[i]);
	for (i = 0; i < TFA98XX_RESONANCE_CHECK_TIMES; i++)
		pr_debug("%s RAW btm F0[%d] = %d\n", __func__, i, btmFres[i]);

	//low to high
	bsort(topFres, TFA98XX_RESONANCE_CHECK_TIMES);
	bsort(btmFres, TFA98XX_RESONANCE_CHECK_TIMES);

	//FIXME BRUCE
	for (i = 0; i < TFA98XX_RESONANCE_CHECK_TIMES; i++)
		pr_debug("%s SORT top F0[%d] = %d\n", __func__, i, topFres[i]);
	for (i = 0; i < TFA98XX_RESONANCE_CHECK_TIMES; i++)
		pr_debug("%s SORT btm F0[%d] = %d\n", __func__, i, btmFres[i]);
	/*
	* report
	* */
	//discard min and max, avg three
	topavgF0 = (topFres[1] + topFres[2] + topFres[3])/3;
	btmavgF0 = (btmFres[1] + btmFres[2] + btmFres[3])/3;

	pr_debug("%s top avgF0 = %d   btm avgF0 = %d\n", __func__, topavgF0, btmavgF0);

	//save value
	speaker_model.lf0 = topavgF0;
	speaker_model.rf0 = btmavgF0;


	tfa_dev_stop(tfa98xx->tfa);

	return 0;
}


/*------------------------------------------------calibration---------------------------------------------*/
/*
 * set OTC
 */
static Tfa98xx_Error_t tfa98xxCalSetCalibrateOnce(struct tfa_device *tfa)
{
	Tfa98xx_Error_t err;

	err = tfa98xx_set_mtp(tfa, 1<<TFA_MTPOTC_POS, 1<<TFA_MTPOTC_POS); /* val mask */
	if (err)
		pr_err("%s  :MTP write failed\n", __func__);

	return err;
}
/*
 * clear OTC
 */
static Tfa98xx_Error_t tfa98xxCalSetCalibrationAlways(struct tfa_device *tfa){
	Tfa98xx_Error_t err;

	err = tfa98xx_set_mtp(tfa, 0, (1<<TFA_MTPEX_POS) | (1<<TFA_MTPOTC_POS));/* val mask */
	if (err)
		pr_err("%s  :MTP write failed\n", __func__);

	return err;
}
/*
 *	MTPEX is 1, calibration is done
 *
 */
static int tfa98xxCalCheckMTPEX(struct tfa_device *tfa)
{
	/* MTPEX is 1, calibration is done */
	return TFA_GET_BF(tfa, MTPEX) == 1 ? 1 : 0;
}

static int tfa98xxCalDspSupporttCoef(struct tfa_device *tfa)
{
	return supportYes;
}
static int tfa98xxCaltCoefFromSpeaker(Tfa98xx_SpeakerParameters_t speakerBytes)
{
	int iCoef;

	/* tCoef(A) is the last parameter of the speaker */
	iCoef = (speakerBytes[TFA1_SPEAKERPARAMETER_LENGTH-3]<<16) +
		(speakerBytes[TFA1_SPEAKERPARAMETER_LENGTH-2]<<8) +
		speakerBytes[TFA1_SPEAKERPARAMETER_LENGTH-1];

	return iCoef/((1<<23) /COEFF_PRECISION);
}

static enum Tfa98xx_Error
tfa98xx_dsp_get_calibration_impedance(struct tfa_device *tfa, int *p_re25)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	int spkr_count, i;

	/* Get number of speakers */
	error = tfa_supported_speakers(tfa, &spkr_count);
	if (error == Tfa98xx_Error_Ok) {
		/* Get calibration values (from MTP or Speakerboost) */
		error = tfa_dsp_get_calibration_impedance(tfa);
		for(i=0; i<spkr_count; i++) {
			p_re25[i] = tfa_get_calibration_info(tfa, i) / (1000 / FLOAT_TO_INTER_COEF);
		}
	}

	return error;
}


static void tfa98xxCaltCoefToSpeaker(Tfa98xx_SpeakerParameters_t speakerBytes, int tCoef)
{
	int iCoef;

	iCoef =(int)((tCoef/COEFF_PRECISION)*(1<<23));
	speakerBytes[TFA1_SPEAKERPARAMETER_LENGTH-3] = (iCoef>>16)&0xFF;
	speakerBytes[TFA1_SPEAKERPARAMETER_LENGTH-2] = (iCoef>>8)&0xFF;
	speakerBytes[TFA1_SPEAKERPARAMETER_LENGTH-1] = (iCoef)&0xFF;
}

/*
 *  calculate a new tCoefA and put the result into the loaded Speaker params
 */
static Tfa98xx_Error_t tfa98xxCalComputeSpeakertCoefA(struct tfa_device *tfa,
		Tfa98xx_SpeakerParameters_t loadedSpeaker,
		int tCoef)
{
	Tfa98xx_Error_t err;
	long tCoefA;
	int re25;
	int Tcal; /* temperature at which the calibration happened */
	int T0;
	int calibrateDone = 0;
	int nxpTfaCurrentProfile = tfa_dev_get_swprof(tfa);

	/* make sure there is no valid calibration still present */
	err = tfa98xx_dsp_get_calibration_impedance(tfa, &re25);
	PRINT_ASSERT(err);
	_ASSERT(fabs(re25) < 10);
	pr_debug(" re25 = %d.%02d\n", (re25/FLOAT_TO_INTER_COEF), (re25%FLOAT_TO_INTER_COEF));

	/* use dummy tCoefA, also eases the calculations, because tCoefB=re25 */
	tfa98xxCaltCoefToSpeaker(loadedSpeaker, 0.0f);

	// write all the files from the device list (typically spk and config)
	err = tfaContWriteFiles(tfa);
	if (err) return err;

	/* write all the files from the profile list (typically preset) */
	err = tfaContWriteFilesProf(tfa, nxpTfaCurrentProfile, 0); /* use volumestep 0 */
	if (err) return err;

	/* start calibration and wait for result */
	err = -TFA_SET_BF_VOLATILE(tfa, SBSL, 1);
	if (err != Tfa98xx_Error_Ok) {
		return err;
	}

	tfaRunWaitCalibration(tfa, &calibrateDone);
	if (calibrateDone) {
		err = tfa98xx_dsp_get_calibration_impedance(tfa, &re25);
	} else {
		re25 = 0;
	}

	err = tfa98xx_dsp_read_mem(tfa, 232, 1, &Tcal);
	if (err != Tfa98xx_Error_Ok) {
		return err;
	}
	pr_debug("Calibration value is %d.%02d ohm @ %d degrees\n", (re25/FLOAT_TO_INTER_COEF), (re25%FLOAT_TO_INTER_COEF), Tcal);

	/* calculate the tCoefA */
	T0 = 25; /* definition of temperature for Re0 */
	tCoefA = ((long)tCoef* re25 / ((tCoef* (Tcal - T0))/COEFF_PRECISION+1))/FLOAT_TO_INTER_COEF; /* TODO: need Rapp influence */
	pr_debug(" Final tCoefA %d.%05d\n", ((int)tCoefA/COEFF_PRECISION), ((int)tCoefA%COEFF_PRECISION));

	/* update the speaker model */
	tfa98xxCaltCoefToSpeaker(loadedSpeaker, tCoefA);

	/* !!! the host needs to save this loadedSpeaker as it is needed after the next cold boot !!! */
	return err;
}


static enum Tfa98xx_Error tfa98xx_verify_speaker_range(struct tfa_device *tfa, int imp[2], int spkr_count)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	nxpTfaSpeakerFile_t *spkr;
	unsigned int Rtypical_left;
	unsigned int Rtypical_right;

	spkr = (nxpTfaSpeakerFile_t *)tfacont_getfiledata(tfa, 0, speakerHdr);
	if (spkr==0)
		return Tfa98xx_Error_Other;

	Rtypical_left = spkr->ohm_primary;
	Rtypical_right = spkr->ohm_secondary;

	/* We always have atleast one speaker */
	if (Rtypical_left==0) {
		pr_warn("Warning: Speaker impedance (primary) not defined in spkr file, assuming 8 Ohm!\n");
		Rtypical_left = 8;
	}

	/* If we have multiple speakers also check the secondary */
	if (spkr_count == 2) {
		if (Rtypical_right==0) {
			pr_warn("Warning: Speaker impedance (secondary) not defined in spkr file, assuming 8 Ohm!\n");
			Rtypical_right = 8;
		}
	}

        /* 15% variation possible , whole argu * FLOAT_TO_INTER_COEF precision*/
	if ( imp[0] < (Rtypical_left*85) || imp[0] > (Rtypical_left*115))
		pr_err("Warning: Primary Speaker calibration value is not within expected range! (%d.%02d Ohm) \n", (imp[0]/FLOAT_TO_INTER_COEF), (imp[0]%FLOAT_TO_INTER_COEF));

	if(spkr_count > 1) {
		if ( imp[1] < (Rtypical_right*85) || imp[1] > (Rtypical_right*115))
			pr_err("Warning: Secondary Speaker calibration value is not within expected range! (%d.%02d Ohm) \n", (imp[1]/FLOAT_TO_INTER_COEF), (imp[1]%FLOAT_TO_INTER_COEF));
	}

        return err;
}

static Tfa98xx_Error_t tfa98xxCalibration(struct tfa_device *tfa, int once, int profile)
{
	uint8_t *speakerbuffer = NULL;
	Tfa98xx_Error_t err = Tfa98xx_Error_Ok;
	int tCoef = 0;
	int imp[2] = { 0 };
	int spkr_count = 0, cal_profile = -1;

	pr_err("calibrate %s\n", once ? "once" : "always" );

	/* Search if there is a calibration profile
	* Only if the user did not give a specific profile and coldstart
	*/
	if (profile == 0) {
		cal_profile = tfaContGetCalProfile(tfa);
		if (cal_profile >= 0)
			profile = cal_profile;
		else
			profile = 0;
	}

	err = tfa_supported_speakers(tfa, &spkr_count);
	if (err) {
		pr_err("Getting the number of supported speakers failed");
		return err;
	}

	/* Do a full startup */
	if (tfaRunStartup(tfa, profile))
		return err;

	/* force cold boot to set ACS to 1 */
	if (tfaRunColdboot(tfa, 1))
		return err;

	if (once) {
		/* Set MTPOTC */
		tfa98xxCalSetCalibrateOnce(tfa);
	} else {
		/* Clear MTPOTC */
		tfa98xxCalSetCalibrationAlways(tfa);
	}

	/* Only for tfa1 (tfa9887 specific) */
	if ((tfa98xxCalCheckMTPEX(tfa) == 0) && (!tfa98xxCalDspSupporttCoef(tfa))) {
		nxpTfaSpeakerFile_t *spkFile;

		/* ensure no audio during special calibration */
		err = tfa98xx_set_mute(tfa, Tfa98xx_Mute_Digital);
		_ASSERT(err == Tfa98xx_Error_Ok);

		pr_err(" 2 step calibration\n");

		spkFile = (nxpTfaSpeakerFile_t *)tfacont_getfiledata(tfa, 0, speakerHdr);

		if (spkFile == NULL || spkFile->data == NULL) {
			pr_err("No speaker data found\n");
			return Tfa98xx_Error_Bad_Parameter;
		}
		speakerbuffer = spkFile->data;

		tCoef = tfa98xxCaltCoefFromSpeaker(speakerbuffer);	/*float to inter(x  100000) precision*/
		pr_err(" tCoef = %d.%05d\n", (tCoef/COEFF_PRECISION), (tCoef%COEFF_PRECISION));

		err = tfa98xxCalComputeSpeakertCoefA(tfa, speakerbuffer, tCoef);
		_ASSERT(err == Tfa98xx_Error_Ok);

		/* if we were in one-time calibration (OTC) mode, clear the calibration results
		from MTP so next time 2nd calibartion step can start. */
		tfa98xx_set_mtp(tfa, 0, 1<<TFA_MTPEX_POS);

		/* force recalibration now with correct tCoefA */
		tfa98xx_set_mute(tfa, Tfa98xx_Mute_Off);
		tfaRunColdStartup(tfa, profile);
	}

	/* if CF is in bypass then return here */
	if (TFA_GET_BF(tfa, CFE) == 0)
		return err;

	/* Load patch and delay tables */
	if (tfaRunStartDSP(tfa))
		return err;

	/* DSP is running now */
	/* write all the files from the device list (speaker, vstep and all messages) */
	if (tfaContWriteFiles(tfa))
		return err;

	/* write all the files from the profile list (typically preset) */
	if (tfaContWriteFilesProf(tfa, profile, 0)) /* use volumestep 0 */
		return err;

	/* Check if CF is not in bypass */
	if (TFA_GET_BF(tfa, CFE) == 0) {
		pr_err("It is not possible to calibrate with CF in bypass! \n");
		return Tfa98xx_Error_DSP_not_running;
	}

	if (tfa98xxCalCheckMTPEX(tfa) == 1) {
		pr_err("DSP already calibrated.\n Calibration skipped, previous results loaded.\n");
		err = tfa_dsp_get_calibration_impedance(tfa);
#ifdef __KERNEL__ /* Necessary otherwise we are thrown out of operating mode in kernel (because of internal clock) */
		if((strstr(tfaContProfileName(tfa->cnt, tfa->dev_idx, profile), ".cal") == NULL) && (tfa->tfa_family == 2)) {
			TFA_SET_BF_VOLATILE(tfa, SBSL, 1);
		} else if (tfa->tfa_family != 2)
#endif
            TFA_SET_BF_VOLATILE(tfa, SBSL, 1);
	} else {
		/* Save the current profile */
		tfa_dev_set_swprof(tfa, (unsigned short)profile);
		/* calibrate */
		err = tfaRunSpeakerCalibration(tfa);
		if (err) return err;
	}

	imp[0] = tfa_get_calibration_info(tfa, 0);
	imp[1] = tfa_get_calibration_info(tfa, 1);
	imp[0] = imp[0]/(1000 /FLOAT_TO_INTER_COEF);
	imp[1] = imp[1]/(1000 /FLOAT_TO_INTER_COEF);

	if (spkr_count == 1)
		pr_err("Calibration value is: %d.%02d ohm\n", (imp[0]/FLOAT_TO_INTER_COEF), (imp[0]%FLOAT_TO_INTER_COEF));
	else
		pr_err("Calibration value is: %d.%02d  %d.%02d ohm\n", (imp[0]/FLOAT_TO_INTER_COEF), (imp[0]%FLOAT_TO_INTER_COEF), (imp[1]/FLOAT_TO_INTER_COEF), (imp[1]%FLOAT_TO_INTER_COEF));

	/* Check speaker range */
	err = tfa98xx_verify_speaker_range(tfa, imp, spkr_count);

	/* Unmute after calibration */
	tfa98xx_set_mute(tfa, Tfa98xx_Mute_Off);

	if(tCoef != 0) {
		if (!tfa98xxCalDspSupporttCoef(tfa))
			tfa98xxCaltCoefToSpeaker(speakerbuffer, tCoef);
	}

	/* Save the current profile */
	tfa_dev_set_swprof(tfa, (unsigned short)profile);

	/* After loading calibration profile we need to load acoustic shock (safe) profile */
	if (cal_profile >= 0) {
		profile = 0;
		pr_err("Loading %s profile! \n", tfaContProfileName( tfa->cnt, tfa->dev_idx, profile));
		err = tfaContWriteProfile(tfa, profile, 0);
	}

	/* Always search and apply filters after a startup */
	err = tfa_set_filters(tfa, profile);

	/* Save the current profile */
	tfa_dev_set_swprof(tfa, (unsigned short)profile);

	return err;
}

/*
 * internal function, no lock hold
 * */
static int smartpa_calibration(void)
{
	Tfa98xx_Error_t err = Tfa98xx_Error_Ok;
	struct tfa98xx *tfa98xx = ext_tfa98xx;
	int re25 = 0;

	pr_debug("%s %d\n", __func__, __LINE__);
	if(tfa_ext_sclk_enable)
		tfa_ext_sclk_enable(1);//open i2s // MODIFIED by hongwei.tian, 2017-12-01,BUG-5709367
	/*
	* speaker mode, using tinymix interface better
	* */
//	if (tfa98xx->pinctrl) {/*recv mode switch disable*/
//		if (pinctrl_select_state(tfa98xx_devices[0]->pinctrl, tfa98xx_devices[0]->pinctrl_state_suspend))
//			pr_err("%s: can not set rcv_switch_suspend pinstate\n", __func__);
//	}

	/*
	* for audio cal, force all device reset
	* */
	tfa98xx_ext_reset(tfa98xx);//reset all devices

	msleep(100);

	//TODO FIXME if the second open faild, need close the first one
	err = (Tfa98xx_Error_t) tfa_dev_start(tfa98xx->tfa, tfa98xx->profile, tfa98xx->vstep);
	if (err) {
		pr_err("%s %d open cont error %d\n", __func__, __LINE__, err);
		return err;
	}

	/*
	* do cal, if failed, cal next,
	* help OP to find the bad one
	* */
//	pr_debug("%s %d  handlesIn[%d]=%d\n", __func__, __LINE__, dev, handlesIn[dev]);
	err = tfa98xxCalibration(tfa98xx->tfa, 1, 0);
	if (err) {
		pr_err("error tfa98xxCalibration dev@%d: %d\n", tfa98xx->i2c->addr, err);
		goto tfa_stop;
	}

	err = tfa98xx_dsp_get_calibration_impedance(tfa98xx->tfa, &re25);
	if (err) {
		pr_err("Tfa98xx_DspGetCalibrationImpedance dev@%d error: %d\n", tfa98xx->i2c->addr, err);
		goto tfa_stop;
	}

	/*
	* record calibration value anyway
	* let's MINI to show
	* */

	speaker_model.cal_lohm = re25;

	if ((re25 < speaker_model.min_lohm) ||(re25 > speaker_model.max_lohm)) {
		pr_err("Calibration Value error, reset MtpEx and, do not open device %d", tfa98xx->i2c->addr);
		err = tfa98xx_set_mtp(tfa98xx->tfa, 0, 1<<TFA_MTPEX_POS);
		if (err) {
			pr_err("reset calibration state failed error: %d\n", err);
			goto tfa_stop;
		}
	} else
		pr_err("Calibration ok dev %d", tfa98xx->i2c->addr);
	/*
	* close everything
	* after calibration is done turned down all of the devices
	* */

	/* tfaRunSpeakerBoost (called by start) implies unmute */
	/* mute + SWS wait */
	err = tfaRunMute( tfa98xx->tfa );
	if (err != Tfa98xx_Error_Ok) {
		pr_err("ERROR %s %d dev=%d err=%d\n", __func__, __LINE__, tfa98xx->i2c->addr, err);
		goto tfa_stop;
	}

	/* powerdown CF */
	err = tfa98xx_powerdown(tfa98xx->tfa, 1 );
	if (err != Tfa98xx_Error_Ok) {
		pr_err("ERROR %s %d dev=%d err=%d\n", __func__, __LINE__, tfa98xx->i2c->addr, err);
	}

tfa_stop:
	tfa_dev_stop(tfa98xx->tfa);

	if(tfa_ext_sclk_enable)
		tfa_ext_sclk_enable(0);//close i2s // MODIFIED by hongwei.tian, 2017-12-01,BUG-5709367
	pr_debug("[NXP] %s END Calibration is done!",__func__);

	return err;
}


static int smartpa_ftm_force_clear_MTPEX(void)
{
	int err = Tfa98xx_Error_Ok;
	unsigned short mtp = 0;
	struct tfa98xx *tfa98xx = ext_tfa98xx;

	pr_debug("%s %d\n", __func__, __LINE__);

	if(tfa_ext_sclk_enable)
		tfa_ext_sclk_enable(1);//open i2s // MODIFIED by hongwei.tian, 2017-12-01,BUG-5709367
	/*
	* for audio cal, force tfa9890 reset
	* */
	tfa98xx_ext_reset(tfa98xx);//reset all need

	//TODO FIXME if the second open faild, need close the first one
	err = tfa_dev_start(tfa98xx->tfa, tfa98xx->profile, tfa98xx->vstep);
	if (err) {
		pr_err("%s %d open cont error %d\n", __func__, __LINE__, err);
		return err;
	}

	/*
	* god bless
	* */
	err = tfaRunColdStartup(tfa98xx->tfa, 0);
	if (err) {
		pr_err("%s cold startup failed\n", __func__);
		goto tfa_stop;
	}

	mtp = 0;
	err = tfa98xx_read_register16(tfa98xx->tfa, TFA98XX_MTP, &mtp);
	pr_debug("b4 clearmtp mpt dev=%d reg value 0x%x\n", tfa98xx->i2c->addr, mtp);
	/*
	* if in ftm audio calibration mode,
	* force clear tfa9897 calibration bit
	* */
	pr_debug("ftm_aduio_cal_mode reset MTPEX for dev=%d\n", tfa98xx->i2c->addr);
	err = tfa98xx_set_mtp(tfa98xx->tfa, 0, 1<<TFA_MTPEX_POS);
	if (err) {
		pr_err("%s reset calibration state failed error: %d\n", __func__, err);
		goto tfa_stop;
	}

	mtp = 0;
	err = tfa98xx_read_register16(tfa98xx->tfa, TFA98XX_MTP, &mtp);
	pr_debug("after clearmtp mpt dev=%d reg value 0x%x\n", tfa98xx->i2c->addr, mtp);

	/*
	* call tfa98xx_close in here after cal or check cal
	* cal call spk boost which setup dev regs
	* */
	/* after calibration is done turned down all of the devices */
	/* tfaRunSpeakerBoost (called by start) implies unmute */
	/* mute + SWS wait */
	err = tfaRunMute(tfa98xx->tfa);
	if (err != Tfa98xx_Error_Ok)
			goto tfa_stop;

		/* powerdown CF */
	err = tfa98xx_powerdown(tfa98xx->tfa, 1 );
	if (err != Tfa98xx_Error_Ok) {
		pr_err("ERROR %s %d dev=%d err=%d\n", __func__, __LINE__, tfa98xx->i2c->addr, err);
	}

tfa_stop:
	tfa_dev_stop(tfa98xx->tfa);

	if(tfa_ext_sclk_enable)
		tfa_ext_sclk_enable(0);//close i2s // MODIFIED by hongwei.tian, 2017-12-01,BUG-5709367
	return err;
}

/*
 * befor cal, clear old mtpex for clean
 * self enable disable clock and etc
 * */
static int tfa98xx_ftm_calibration(void)
{
	int ret = 0;
    	mutex_lock(&tfa_test_mutex);
	ret = smartpa_ftm_force_clear_MTPEX();
	ret = smartpa_calibration();
  	mutex_unlock(&tfa_test_mutex);

	return ret;
}

/*------------------------------------------------------ sys fs control node----------------------------------------------*/
static ssize_t ftm_spr_res_status_store(struct class *class, struct class_attribute *attr,
			const char *buf, size_t count)
{
	int ret = 0;

	if(!strncmp(buf, "start", 5)) {	//start test
		strlcpy(res_status, "start....", 10);
		ret = tfa98xx_speaker_resonance(0);
		if (ret)
			pr_err("ftm speaker resonance test failed!\n");
		strlcpy(res_status, "completed", 10);
	}else
		pr_err("ftm speaker resonance test set command error!\n");

	return count;
}

static ssize_t ftm_spr_res_status_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", res_status);
}

static ssize_t ftm_spr_top_res_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", speaker_model.lf0);
}
static ssize_t ftm_spr_btm_res_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", speaker_model.rf0);
}
static ssize_t ftm_spr_top_res_max_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", speaker_model.max_lf0);
}
static ssize_t ftm_spr_top_res_min_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", speaker_model.min_lf0);
}
static ssize_t ftm_spr_btm_res_max_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", speaker_model.max_rf0);
}
static ssize_t ftm_spr_btm_res_min_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", speaker_model.min_rf0);
}



static ssize_t ftm_cal_status_store(struct class *class, struct class_attribute *attr,
			const char *buf, size_t count)
{
	int ret = 0;

	if(!strncmp(buf, "start", 5)) {
		strlcpy(cal_status, "start....", 10);
		ret = tfa98xx_ftm_calibration();
		if (ret)
			pr_err("ftm calibration failed !\n");
		strlcpy(cal_status, "completed", 10);
	}else
		pr_err("ftm_calibration set command error!\n");

	return count;
}

static ssize_t ftm_cal_status_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", cal_status);
}

static ssize_t ftm_cal_top_imp_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d.%02d\n", (speaker_model.cal_lohm/FLOAT_TO_INTER_COEF), (speaker_model.cal_lohm%FLOAT_TO_INTER_COEF));
}
static ssize_t ftm_cal_btm_imp_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d.%02d\n", (speaker_model.cal_rohm/FLOAT_TO_INTER_COEF), (speaker_model.cal_rohm%FLOAT_TO_INTER_COEF));
}
static ssize_t ftm_cal_top_imp_max_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d.%02d\n", (speaker_model.max_lohm/FLOAT_TO_INTER_COEF), (speaker_model.max_lohm%FLOAT_TO_INTER_COEF));
}
static ssize_t ftm_cal_top_imp_min_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d.%02d\n", (speaker_model.min_lohm/FLOAT_TO_INTER_COEF), (speaker_model.min_lohm%FLOAT_TO_INTER_COEF));
}
static ssize_t ftm_cal_btm_imp_max_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d.%02d\n", (speaker_model.max_rohm/FLOAT_TO_INTER_COEF), (speaker_model.max_rohm%FLOAT_TO_INTER_COEF));
}
static ssize_t ftm_cal_btm_imp_min_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d.%02d\n", (speaker_model.min_rohm/FLOAT_TO_INTER_COEF), (speaker_model.min_rohm%FLOAT_TO_INTER_COEF));
}

/* MODIFIED-BEGIN by hongwei.tian, 2019-05-17,BUG-7782517*/
static ssize_t ftm_cal_bitclk_store(struct class *class, struct class_attribute *attr,
			const char *buf, size_t count)
{
	if(!tfa_ext_sclk_enable)
		return count;

	if(!strncmp(buf, "enable", 6)) {
		tfa_ext_sclk_enable(1);
	}else
		tfa_ext_sclk_enable(0);

	return count;
}
/* MODIFIED-END by hongwei.tian,BUG-7782517*/


//speaker_resonance attr
static struct class_attribute ftm_spr_res_status =
	__ATTR(status, 0664, ftm_spr_res_status_show, ftm_spr_res_status_store);
static struct class_attribute ftm_spr_top_res =
	__ATTR(LRES, 0664, ftm_spr_top_res_show, NULL);
static struct class_attribute ftm_spr_btm_res =
	__ATTR(RRES, 0664, ftm_spr_btm_res_show, NULL);
static struct class_attribute ftm_spr_top_res_max =
	__ATTR(top_res_max, 0664, ftm_spr_top_res_max_show, NULL);
static struct class_attribute ftm_spr_top_res_min =
	__ATTR(top_res_min, 0664, ftm_spr_top_res_min_show, NULL);
static struct class_attribute ftm_spr_btm_res_max =
	__ATTR(btm_res_max, 0664, ftm_spr_btm_res_max_show, NULL);
static struct class_attribute ftm_spr_btm_res_min =
	__ATTR(btm_res_min, 0664, ftm_spr_btm_res_min_show, NULL);

//calib attr
static struct class_attribute ftm_cal_status =
	__ATTR(status, 0664, ftm_cal_status_show, ftm_cal_status_store);
static struct class_attribute ftm_cal_top_imp =
	__ATTR(LOHM, 0664, ftm_cal_top_imp_show, NULL);
static struct class_attribute ftm_cal_btm_imp =
	__ATTR(ROHM, 0664, ftm_cal_btm_imp_show, NULL);
static struct class_attribute ftm_cal_top_imp_max =
	__ATTR(top_imp_max, 0664, ftm_cal_top_imp_max_show, NULL);
static struct class_attribute ftm_cal_top_imp_min =
	__ATTR(top_imp_min, 0664, ftm_cal_top_imp_min_show, NULL);
static struct class_attribute ftm_cal_btm_imp_max =
	__ATTR(btm_imp_max, 0664, ftm_cal_btm_imp_max_show, NULL);
static struct class_attribute ftm_cal_btm_imp_min =
	__ATTR(btm_imp_min, 0664, ftm_cal_btm_imp_min_show, NULL);
/* MODIFIED-BEGIN by hongwei.tian, 2019-05-17,BUG-7782517*/
static struct class_attribute ftm_cal_bitclk =
	__ATTR(enable_bitclk, 0664, NULL, ftm_cal_bitclk_store);
	/* MODIFIED-END by hongwei.tian,BUG-7782517*/


void tfa98xx_sysfs_init(void)
{
	struct class *ftm_cal_class;
	struct class *ftm_spr_res_class;

	/* ftm_cal_class create (/<sysfs>/class/ftm_cal) */
	ftm_cal_class = class_create(THIS_MODULE, "ftm_cal");
	if (IS_ERR(ftm_cal_class)) {
		pr_err("%s: ftm_cal_class: couldn't create class ftm_cal_class\n", __func__);
	}
	if (class_create_file(ftm_cal_class, &ftm_cal_status) 			|| \
		class_create_file(ftm_cal_class, &ftm_cal_top_imp) 		|| \
		class_create_file(ftm_cal_class, &ftm_cal_btm_imp) 		|| \
		class_create_file(ftm_cal_class, &ftm_cal_top_imp_max) 	|| \
		class_create_file(ftm_cal_class, &ftm_cal_top_imp_min) 	|| \
		class_create_file(ftm_cal_class, &ftm_cal_btm_imp_max) 	|| \
		class_create_file(ftm_cal_class, &ftm_cal_bitclk) 	|| \
		class_create_file(ftm_cal_class, &ftm_cal_btm_imp_min)) {

		pr_err("%s: ftm_cal_class: couldn't create sub file node\n", __func__);
	}

	/* ftm_spr_res_class create (/<sysfs>/class/ftm_spr_res_class) */
	ftm_spr_res_class = class_create(THIS_MODULE, "ftm_spr_res");
	if (IS_ERR(ftm_spr_res_class)) {
		pr_err("%s: ftm_spr_res_class: couldn't create class ftm_spr_res_class\n", __func__);
	}
	if (class_create_file(ftm_spr_res_class, &ftm_spr_res_status) 				|| \
		class_create_file(ftm_spr_res_class, &ftm_spr_top_res) 			|| \
		class_create_file(ftm_spr_res_class, &ftm_spr_btm_res) 			|| \
		class_create_file(ftm_spr_res_class, &ftm_spr_top_res_max) 		|| \
		class_create_file(ftm_spr_res_class, &ftm_spr_top_res_min) 		|| \
		class_create_file(ftm_spr_res_class, &ftm_spr_btm_res_max) 	|| \
		class_create_file(ftm_spr_res_class, &ftm_spr_btm_res_min)) {

		pr_err("%s: ftm_spr_res_class: couldn't create sub file node\n", __func__);
	}

	mutex_init(&tfa_test_mutex);
}
#endif

