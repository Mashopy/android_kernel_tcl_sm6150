/* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/crc32.h>
#include <media/cam_sensor.h>

#include "cam_eeprom_core.h"
#include "cam_eeprom_soc.h"
#include "cam_debug_util.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"


struct msm_eeprom_power_setting_array {
	struct cam_sensor_power_setting  power_setting_a[MAX_POWER_CONFIG];
	struct cam_sensor_power_setting *power_setting;
	unsigned short size;
	struct cam_sensor_power_setting  power_down_setting_a[MAX_POWER_CONFIG];
	struct cam_sensor_power_setting *power_down_setting;
	unsigned short size_down;
};

int cali_value = 0;
char cali_flag = 0;
char ae_cali_flag = 0;
unsigned char *bin_buffer = NULL;

#define CCI_BLOCK_SIZE          32
#define SLAVE_ADDRESS           0xA0        /*eeprom slave address*/
#define CCI_WRITE_MASTER        0
#define CALI_DATA_SIZE          0x800       //2k
#define CALI_DATA_OFFSET_FLAG   0x15FF
#define CALI_DATA_OFFSET        CALI_DATA_OFFSET_FLAG + 1 // cci write continuesly  start address should be integ multi of 32
#define CALI_DATA_CHECKSUM      CALI_DATA_OFFSET_FLAG + CALI_DATA_SIZE + 1
#define PATH_CALI               "/sdcard/MMI/dualcam_cali.bin"
#define PATH_CALI_FLAG          "/sdcard/MMI/dualcam_flag.bin"


#define AE_BLOCK_SIZE           10
#define AE_DATA_OFFSET_FLAG     CALI_DATA_CHECKSUM + 1
#define AE_DATA_OFFSET          AE_DATA_OFFSET_FLAG +1
#define AE_DATA_CHECKSUM        AE_DATA_OFFSET + AE_BLOCK_SIZE
#define PATH_CALI_AE            "/sdcard/MMI/dualcam_ae.bin"
#define PATH_AE_FLAG            "/sdcard/MMI/dualcam_ae_flag.bin"

struct msm_eeprom_power_setting_array  *power_setting_array_dc = NULL;

static void msm_eeprom_copy_power_settings_compat(
  struct msm_eeprom_power_setting_array *ps,
  struct cam_sensor_power_ctrl_t *power_info)
{
  int i = 0;

  ps->size = power_info->power_setting_size;
  for (i = 0; i < power_info->power_setting_size; i++) {
    ps->power_setting_a[i].config_val =
      power_info->power_setting[i].config_val;
    ps->power_setting_a[i].delay =
      power_info->power_setting[i].delay;
    ps->power_setting_a[i].seq_type =
      power_info->power_setting[i].seq_type;
    ps->power_setting_a[i].seq_val =
    	power_info->power_setting[i].seq_val;
  }

  ps->size_down = power_info->power_down_setting_size;
  for (i = 0; i < power_info->power_down_setting_size; i++) {
    ps->power_down_setting_a[i].config_val =
      power_info->power_down_setting[i].config_val;
    ps->power_down_setting_a[i].delay =
      power_info->power_down_setting[i].delay;
    ps->power_down_setting_a[i].seq_type =
      power_info->power_down_setting[i].seq_type;
    ps->power_down_setting_a[i].seq_val =
      power_info->power_down_setting[i].seq_val;
  }
}

static uint32_t read_dualcam_cali_data(int write_en)
{
  struct file *fp;
  mm_segment_t fs;
  loff_t pos = 0;
  uint32_t data_size = 0;
  //int i = 0;
  if(write_en == 1){
    fp = filp_open(PATH_CALI, O_RDWR, 0666);
  }else if(write_en == 2){
    fp = filp_open(PATH_CALI_AE, O_RDWR, 0666);
  }else{
    CAM_ERR(CAM_EEPROM, "write_en %d not support",write_en);
    return 0;
  }

  if (IS_ERR(fp)) {
    CAM_ERR(CAM_EEPROM, "faile to open file cali data");
    return 0;
  }

  data_size = vfs_llseek(fp,0,SEEK_END);
  CAM_ERR(CAM_EEPROM, "Binary data size is %d bytes",data_size);

  if(data_size > 0) {
    bin_buffer = kzalloc(data_size, GFP_KERNEL);
    if (bin_buffer == NULL){
      CAM_ERR(CAM_EEPROM, "[Error]malloc memery failed");
      goto close;
  }

    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_read(fp, bin_buffer, data_size, &pos);
    CAM_ERR(CAM_EEPROM, "Read new calibration data done!");

    filp_close(fp, NULL);
    set_fs(fs);
    return data_size;
  }else{
    CAM_ERR(CAM_EEPROM, "[Error] Get calibration data failed");
  }

  close:
    filp_close(fp, NULL);
    set_fs(fs);
    CAM_ERR(CAM_EEPROM, "read dualcam cali data exit");
    return -1;
}


static int calibration_check(struct cam_eeprom_ctrl_t *e_ctrl)
{
	int rc = 0;
	uint32_t calibration_flag = 0;

	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "%s e_ctrl is NULL", __func__);
		return -EINVAL;
	}

	rc = camera_io_dev_read(
		&e_ctrl->io_master_info, CALI_DATA_OFFSET,
		&calibration_flag, CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM, "Calibration flag read failed");
	}else{
    CAM_ERR(CAM_EEPROM, "calibration_flag0=0x%x",calibration_flag);
  }

	rc = camera_io_dev_read(
		&e_ctrl->io_master_info, CALI_DATA_OFFSET_FLAG,
		&calibration_flag, CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM, "Calibration flag read failed");
	}else{
    CAM_ERR(CAM_EEPROM, "calibration_flag=0x%x",calibration_flag);
  }

	rc = camera_io_dev_read(
		&e_ctrl->io_master_info, CALI_DATA_CHECKSUM,
		&calibration_flag, CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM, "Calibration flag read failed");
	}else{
    CAM_ERR(CAM_EEPROM, "calibration_flag_checksum=0x%x",calibration_flag);
  }

	rc = camera_io_dev_read(
		&e_ctrl->io_master_info, CALI_DATA_OFFSET + 368,
		&calibration_flag, CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM, "Calibration flag read failed");
	}else{
    CAM_ERR(CAM_EEPROM, "calibration_flag368=0x%x",calibration_flag);
  }
	rc = camera_io_dev_read(
		&e_ctrl->io_master_info, CALI_DATA_OFFSET + 2045,
		&calibration_flag, CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM, "Calibration flag read failed");
	}else{
    CAM_ERR(CAM_EEPROM, "calibration_flag2045=0x%x",calibration_flag);
  }

	rc = camera_io_dev_read(
		&e_ctrl->io_master_info, CALI_DATA_OFFSET + CCI_BLOCK_SIZE,
		&calibration_flag, CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM, "Calibration flag read failed");
	}else{
    CAM_ERR(CAM_EEPROM, "calibration_flag_block=0x%x",calibration_flag);
  }

	rc = camera_io_dev_read(
		&e_ctrl->io_master_info, CALI_DATA_OFFSET + CALI_DATA_SIZE - 1,
		&calibration_flag, CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM, "Calibration flag read failed");
	}else{
    CAM_ERR(CAM_EEPROM, "calibration_flag_last=0x%x",calibration_flag);
  }

	return rc;
}

static int read_calibration_flag(struct cam_eeprom_ctrl_t *e_ctrl)
{
  int rc = 0;
  uint32_t calibration_flag = 0;

  if (!e_ctrl) {
    CAM_ERR(CAM_EEPROM, "%s e_ctrl is NULL", __func__);
    return -EINVAL;
  }

  rc = camera_io_dev_read(
  &e_ctrl->io_master_info, CALI_DATA_OFFSET_FLAG,
  &calibration_flag, CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE);  // read offset 0x0D2A to check if it's calibrated
  if (rc < 0) {
    CAM_ERR(CAM_EEPROM, "%s: Calibration flag read failed\n", __func__);
  }

  return calibration_flag;
}

static int read_ae_calibration_flag(struct cam_eeprom_ctrl_t *e_ctrl)
{
  int rc = 0;
  uint32_t calibration_flag = 0;

  if (!e_ctrl) {
    CAM_ERR(CAM_EEPROM, "%s e_ctrl is NULL", __func__);
    return -EINVAL;
  }

  rc = camera_io_dev_read(
  &e_ctrl->io_master_info, AE_DATA_OFFSET_FLAG,
  &calibration_flag, CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE);  // read offset 0x0D2A to check if it's calibrated
  if (rc < 0) {
    CAM_ERR(CAM_EEPROM, "%s: Calibration flag read failed\n", __func__);
  }

  return calibration_flag;
}
#if 0 // avoid kernel to write userspace fs
static uint32_t write_dualcam_cali_flag(const char* file_path, char cflag)
{
  struct file *fp;
  mm_segment_t fs;
  loff_t pos = 0;
  const char flag = cflag;

  fp = filp_open(file_path, O_RDWR|O_CREAT, 0666);
  if (IS_ERR(fp)) {
    CAM_ERR(CAM_EEPROM, "faile to open file cali data error");
    return -1;
  }
  fs = get_fs();
  set_fs(KERNEL_DS);
  vfs_write(fp, &flag, 1, &pos);
  filp_close(fp, NULL);
  set_fs(fs);

  CAM_ERR(CAM_EEPROM, "Write %s = 0x%x done!", file_path,flag);
  return 0;
}
#endif
static int write_eeprom_memory(struct cam_eeprom_ctrl_t *e_ctrl, struct cam_eeprom_memory_block_t *block, uint32_t size)
{
  struct cam_sensor_i2c_reg_setting  i2c_reg_settings = {0};
  struct cam_sensor_i2c_reg_array    i2c_reg_array = {0};
  struct cam_sensor_i2c_reg_array*    i2c_reg_array_block = NULL;
  struct cam_sensor_cci_client *cci_client = NULL;
  struct cam_eeprom_soc_private *eb_info = NULL;
  int rc = -1;
  uint32_t i = 0,j = 0;
  uint32_t checksum = 0;
  cali_flag = 0x0;

  if (!e_ctrl)
  {
    CAM_ERR(CAM_EEPROM, "%s e_ctrl is NULL");
    return -EINVAL;
  }

  if (!bin_buffer)
  {
    CAM_ERR(CAM_EEPROM, "%s bin_buffer is NULL");
    return -EINVAL;
  }

  if(size != CALI_DATA_SIZE){
    CAM_ERR(CAM_EEPROM, "dc size is invalid");
    return -EINVAL;
  }

  eb_info = (struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;

  if (e_ctrl->io_master_info.cci_client) {
    e_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
    cci_client = e_ctrl->io_master_info.cci_client;
    cci_client->sid = eb_info->i2c_info.slave_addr >> 1;
    cci_client->cci_i2c_master = CCI_WRITE_MASTER;
  }else{
    CAM_ERR(CAM_EEPROM, "cci_client is NULL");
    return -EINVAL;
  }

  i2c_reg_array_block = kzalloc(sizeof(struct cam_sensor_i2c_reg_array)*CCI_BLOCK_SIZE, GFP_KERNEL);

  for (i = 0; i < size; i++)
  {
    checksum += bin_buffer[i];
  }

  //unlock eeprom writing access;
  CAM_ERR(CAM_EEPROM, "unlock 0x80 (cci error is ok)");
  cci_client->sid = 0x80 >> 1;
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = 0x0000;
  i2c_reg_array.reg_data = 0x00;
  i2c_reg_array.delay = 1;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,
  &i2c_reg_settings);

  CAM_ERR(CAM_EEPROM, "unlock 0x%x 0x8000 0x00",SLAVE_ADDRESS);
  cci_client->sid = SLAVE_ADDRESS >> 1;
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = 0x8000;
  i2c_reg_array.reg_data = 0x00;
  i2c_reg_array.delay = 1;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,
  &i2c_reg_settings);
  if (rc < 0)
  {
    CAM_ERR(CAM_EEPROM, "unlock eeprom writing failed rc = %d", rc);
  }
  msleep(10);

  CAM_ERR(CAM_EEPROM, "write eeprom blocks ...");
  for (j = 0; j < size; j = j + CCI_BLOCK_SIZE){
    if(j + CCI_BLOCK_SIZE <= size){
      for (i = 0; i < CCI_BLOCK_SIZE; i = i + 1){
        i2c_reg_array_block[i].reg_addr = CALI_DATA_OFFSET + i + j;
        i2c_reg_array_block[i].reg_data = bin_buffer[ i + j ];
        i2c_reg_array_block[i].delay = 1;
      }
      i2c_reg_settings.reg_setting = i2c_reg_array_block;
      i2c_reg_settings.size = CCI_BLOCK_SIZE;
      rc = camera_io_dev_write_continuous(&e_ctrl->io_master_info,
      &i2c_reg_settings, 0);

      if (rc < 0)
      {
        CAM_ERR(CAM_EEPROM, "write eeprom data failed1 rc = %d j = %d size = %d", rc,j,size);
        goto FREE;
      }
      msleep(10);
    } else {
      CAM_ERR(CAM_EEPROM, "write eeprom data j = %d", j);
      for (i = 0; i < size - j; i = i + 1){
        i2c_reg_array.reg_addr = CALI_DATA_OFFSET + i + j;
        i2c_reg_array.reg_data = bin_buffer[ i + j ];
        i2c_reg_array.delay = 1;

        i2c_reg_settings.reg_setting = &i2c_reg_array;
        i2c_reg_settings.size = 1;
        rc = camera_io_dev_write(&e_ctrl->io_master_info,
          &i2c_reg_settings);
        if (rc < 0)
        {
          CAM_ERR(CAM_EEPROM, "write eeprom data failed2 rc = %d", rc);
          goto FREE;
        }
      }
      msleep(5);
    }
  }


  CAM_ERR(CAM_EEPROM, "write eeprom Flag...");
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = CALI_DATA_OFFSET_FLAG;
  i2c_reg_array.reg_data = 0x01;
  i2c_reg_array.delay = 0;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,
  &i2c_reg_settings);
  if (rc < 0)
  {
    CAM_ERR(CAM_EEPROM, "write eeprom Flag Fail");
    goto FREE;
  }
  msleep(5);

  checksum = checksum % 0xFF + 1;
  CAM_ERR(CAM_EEPROM, "write eeprom Checksum... checksum = %d",checksum);
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = CALI_DATA_CHECKSUM;
  i2c_reg_array.reg_data = checksum;
  i2c_reg_array.delay = 0;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,
  &i2c_reg_settings);
  if (rc < 0)
  {
    CAM_ERR(CAM_EEPROM, "write eeprom data Checksum Fail");
    goto FREE;
  }
  msleep(5);

  CAM_ERR(CAM_EEPROM, "write eeprom Finish");

  //lock eeprom writing access;
  CAM_ERR(CAM_EEPROM, "lock 0x%x 0x8000 0x01",SLAVE_ADDRESS);
  cci_client->sid = SLAVE_ADDRESS >> 1;
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = 0x8000;
  i2c_reg_array.reg_data = 0x01;
  i2c_reg_array.delay = 1;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,&i2c_reg_settings);
  if (rc < 0)
  {
    CAM_ERR(CAM_EEPROM, "lock eeprom writing failed rc = %d", rc);
    goto FREE;
  }

  CAM_ERR(CAM_EEPROM, "lock 0xF0 (cci error is ok)");
  cci_client->sid = 0xF0 >> 1;
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = 0x0000;
  i2c_reg_array.reg_data = 0x00;
  i2c_reg_array.delay = 1;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,&i2c_reg_settings);


  cali_flag = 0x01;
  kfree(i2c_reg_array_block);
  return 0;
  FREE:
    kfree(i2c_reg_array_block);
    return -1;
}


static int write_eeprom_dc_ae(struct cam_eeprom_ctrl_t *e_ctrl, struct cam_eeprom_memory_block_t *block, uint32_t size)
{
  struct cam_sensor_i2c_reg_setting  i2c_reg_settings = {0};
  struct cam_sensor_i2c_reg_array    i2c_reg_array = {0};
  struct cam_sensor_cci_client *cci_client = NULL;
  struct cam_eeprom_soc_private *eb_info = NULL;
  int rc = -1;
  uint32_t i = 0,j = 0;
  uint32_t checksum = 0;
  ae_cali_flag = 0x0;

  if (!e_ctrl)
  {
    CAM_ERR(CAM_EEPROM, "%s e_ctrl is NULL");
    return -EINVAL;
  }

  if (!bin_buffer)
  {
    CAM_ERR(CAM_EEPROM, "%s bin_buffer is NULL");
    return -EINVAL;
  }

  if(size != AE_BLOCK_SIZE){
    CAM_ERR(CAM_EEPROM, "ae bin size is invalid");
    return -EINVAL;
  }

  eb_info = (struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;

  if (e_ctrl->io_master_info.cci_client) {
    e_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
    cci_client = e_ctrl->io_master_info.cci_client;
    cci_client->sid = eb_info->i2c_info.slave_addr >> 1;
    cci_client->cci_i2c_master = 0;
  }else{
    CAM_ERR(CAM_EEPROM, "cci_client is NULL");
    return -EINVAL;
  }

  for (i = 0; i < AE_BLOCK_SIZE; i++)
  {
    checksum += bin_buffer[i];
  }

  //unlock eeprom writing access;
  CAM_ERR(CAM_EEPROM, "unlock 0x80 (cci error is ok)");
  cci_client->sid = 0x80 >> 1;
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = 0x0000;
  i2c_reg_array.reg_data = 0x00;
  i2c_reg_array.delay = 1;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,
  &i2c_reg_settings);

  CAM_ERR(CAM_EEPROM, "unlock 0x%x 0x8000 0x00",SLAVE_ADDRESS);
  cci_client->sid = SLAVE_ADDRESS >> 1;
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = 0x8000;
  i2c_reg_array.reg_data = 0x00;
  i2c_reg_array.delay = 1;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,
  &i2c_reg_settings);
  if (rc < 0)
  {
    CAM_ERR(CAM_EEPROM, "unlock eeprom writing failed rc = %d", rc);
  }
  msleep(10);

  for (i = 0; i < AE_BLOCK_SIZE; i = i + 1){
    i2c_reg_array.reg_addr = AE_DATA_OFFSET + i;
    i2c_reg_array.reg_data = bin_buffer[i];
    i2c_reg_array.delay = 1;

    i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
    i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    i2c_reg_settings.reg_setting = &i2c_reg_array;
    i2c_reg_settings.size = 1;
    rc = camera_io_dev_write(&e_ctrl->io_master_info,
      &i2c_reg_settings);

    if (rc < 0)
    {
      CAM_ERR(CAM_EEPROM, "write eeprom data failed1 rc = %d j = %d size = %d", rc,j,size);
      goto FREE;
    }
    msleep(5);
  }



  CAM_ERR(CAM_EEPROM, "write eeprom Flag...");
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = AE_DATA_OFFSET_FLAG;
  i2c_reg_array.reg_data = 0x01;
  i2c_reg_array.delay = 0;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,
  &i2c_reg_settings);
  if (rc < 0)
  {
    CAM_ERR(CAM_EEPROM, "write eeprom Flag Fail");
    goto FREE;
  }
  msleep(5);

  checksum = checksum % 0xFF + 1;
  CAM_ERR(CAM_EEPROM, "write eeprom Checksum... checksum = %d",checksum);
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = AE_DATA_CHECKSUM;
  i2c_reg_array.reg_data = checksum;
  i2c_reg_array.delay = 0;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,
  &i2c_reg_settings);
  if (rc < 0)
  {
    CAM_ERR(CAM_EEPROM, "write eeprom data Checksum Fail");
    goto FREE;
  }
  msleep(5);

  CAM_ERR(CAM_EEPROM, "write eeprom Finish");

  //lock eeprom writing access;
  CAM_ERR(CAM_EEPROM, "lock 0x%x 0x8000 0x01",SLAVE_ADDRESS);
  cci_client->sid = SLAVE_ADDRESS >> 1;
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = 0x8000;
  i2c_reg_array.reg_data = 0x01;
  i2c_reg_array.delay = 1;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,&i2c_reg_settings);
  if (rc < 0)
  {
    CAM_ERR(CAM_EEPROM, "lock eeprom writing failed rc = %d", rc);
    goto FREE;
  }

  CAM_ERR(CAM_EEPROM, "lock 0xF0 (cci error is ok)");
  cci_client->sid = 0xF0 >> 1;
  i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_reg_settings.size = 1;
  i2c_reg_array.reg_addr = 0x0000;
  i2c_reg_array.reg_data = 0x00;
  i2c_reg_array.delay = 1;
  i2c_reg_settings.reg_setting = &i2c_reg_array;
  rc = camera_io_dev_write(&e_ctrl->io_master_info,&i2c_reg_settings);



  ae_cali_flag = 0x01;
  return 0;
  FREE:
    return -1;
}

/**
 * cam_eeprom_read_memory() - read map data into buffer
 * @e_ctrl:     eeprom control struct
 * @block:      block to be read
 *
 * This function iterates through blocks stored in block->map, reads each
 * region and concatenate them into the pre-allocated block->mapdata
 */
static int cam_eeprom_read_memory(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_eeprom_memory_block_t *block)
{
	int                                rc = 0;
	int                                j;
	struct cam_sensor_i2c_reg_setting  i2c_reg_settings = {0};
	struct cam_sensor_i2c_reg_array    i2c_reg_array = {0};
	struct cam_eeprom_memory_map_t    *emap = block->map;
	struct cam_eeprom_soc_private     *eb_info = NULL;
	uint8_t                           *memptr = block->mapdata;

	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "e_ctrl is NULL");
		return -EINVAL;
	}

	eb_info = (struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;

	for (j = 0; j < block->num_map; j++) {
		CAM_DBG(CAM_EEPROM, "slave-addr = 0x%X", emap[j].saddr);
		if (emap[j].saddr) {
			eb_info->i2c_info.slave_addr = emap[j].saddr;
			rc = cam_eeprom_update_i2c_info(e_ctrl,
				&eb_info->i2c_info);
			if (rc) {
				CAM_ERR(CAM_EEPROM,
					"failed: to update i2c info rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].page.valid_size) {
			i2c_reg_settings.addr_type = emap[j].page.addr_type;
			i2c_reg_settings.data_type = emap[j].page.data_type;
			i2c_reg_settings.size = 1;
			i2c_reg_array.reg_addr = emap[j].page.addr;
			i2c_reg_array.reg_data = emap[j].page.data;
			i2c_reg_array.delay = emap[j].page.delay;
			i2c_reg_settings.reg_setting = &i2c_reg_array;
			rc = camera_io_dev_write(&e_ctrl->io_master_info,
				&i2c_reg_settings);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "page write failed rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].pageen.valid_size) {
			i2c_reg_settings.addr_type = emap[j].pageen.addr_type;
			i2c_reg_settings.data_type = emap[j].pageen.data_type;
			i2c_reg_settings.size = 1;
			i2c_reg_array.reg_addr = emap[j].pageen.addr;
			i2c_reg_array.reg_data = emap[j].pageen.data;
			i2c_reg_array.delay = emap[j].pageen.delay;
			i2c_reg_settings.reg_setting = &i2c_reg_array;
			rc = camera_io_dev_write(&e_ctrl->io_master_info,
				&i2c_reg_settings);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "page enable failed rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].poll.valid_size) {
			rc = camera_io_dev_poll(&e_ctrl->io_master_info,
				emap[j].poll.addr, emap[j].poll.data,
				0, emap[j].poll.addr_type,
				emap[j].poll.data_type,
				emap[j].poll.delay);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "poll failed rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].mem.valid_size) {
			rc = camera_io_dev_read_seq(&e_ctrl->io_master_info,
				emap[j].mem.addr, memptr,
				emap[j].mem.addr_type,
				emap[j].mem.data_type,
				emap[j].mem.valid_size);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "read failed rc %d",
					rc);
				return rc;
			}
			memptr += emap[j].mem.valid_size;
		}

		if (emap[j].pageen.valid_size) {
			i2c_reg_settings.addr_type = emap[j].pageen.addr_type;
			i2c_reg_settings.data_type = emap[j].pageen.data_type;
			i2c_reg_settings.size = 1;
			i2c_reg_array.reg_addr = emap[j].pageen.addr;
			i2c_reg_array.reg_data = 0;
			i2c_reg_array.delay = emap[j].pageen.delay;
			i2c_reg_settings.reg_setting = &i2c_reg_array;
			rc = camera_io_dev_write(&e_ctrl->io_master_info,
				&i2c_reg_settings);
			if (rc) {
				CAM_ERR(CAM_EEPROM,
					"page disable failed rc %d",
					rc);
				return rc;
			}
		}
	}
	return rc;
}

/**
 * cam_eeprom_power_up - Power up eeprom hardware
 * @e_ctrl:     ctrl structure
 * @power_info: power up/down info for eeprom
 *
 * Returns success or failure
 */
static int cam_eeprom_power_up(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_sensor_power_ctrl_t *power_info)
{
	int32_t                 rc = 0;
	struct cam_hw_soc_info *soc_info =
		&e_ctrl->soc_info;

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		&e_ctrl->soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_EEPROM,
			"failed to fill power up vreg params rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		&e_ctrl->soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_EEPROM,
			"failed to fill power down vreg params  rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed in eeprom power up rc %d", rc);
		return rc;
	}

	if (e_ctrl->io_master_info.master_type == CCI_MASTER) {
		rc = camera_io_init(&(e_ctrl->io_master_info));
		if (rc) {
			CAM_ERR(CAM_EEPROM, "cci_init failed");
			return -EINVAL;
		}
	}
	return rc;
}

/**
 * cam_eeprom_power_down - Power down eeprom hardware
 * @e_ctrl:    ctrl structure
 *
 * Returns success or failure
 */
static int cam_eeprom_power_down(struct cam_eeprom_ctrl_t *e_ctrl)
{
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info         *soc_info;
	struct cam_eeprom_soc_private  *soc_private;
	int                             rc = 0;

	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "failed: e_ctrl %pK", e_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &e_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_EEPROM, "failed: power_info %pK", power_info);
		return -EINVAL;
	}
	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "power down the core is failed:%d", rc);
		return rc;
	}

	if (e_ctrl->io_master_info.master_type == CCI_MASTER)
		camera_io_release(&(e_ctrl->io_master_info));

	return rc;
}

/**
 * cam_eeprom_match_id - match eeprom id
 * @e_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_eeprom_match_id(struct cam_eeprom_ctrl_t *e_ctrl)
{
	int                      rc;
	struct camera_io_master *client = &e_ctrl->io_master_info;
	uint8_t                  id[2];

	rc = cam_spi_query_id(client, 0, CAMERA_SENSOR_I2C_TYPE_WORD,
		&id[0], 2);
	if (rc)
		return rc;
	CAM_DBG(CAM_EEPROM, "read 0x%x 0x%x, check 0x%x 0x%x",
		id[0], id[1], client->spi_client->mfr_id0,
		client->spi_client->device_id0);
	if (id[0] != client->spi_client->mfr_id0
		|| id[1] != client->spi_client->device_id0)
		return -ENODEV;
	return 0;
}

/**
 * cam_eeprom_parse_read_memory_map - Parse memory map
 * @of_node:    device node
 * @e_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
int32_t cam_eeprom_parse_read_memory_map(struct device_node *of_node,
	struct cam_eeprom_ctrl_t *e_ctrl)
{
	int32_t                         rc = 0;
	struct cam_eeprom_soc_private  *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "failed: e_ctrl is NULL");
		return -EINVAL;
	}

	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	rc = cam_eeprom_parse_dt_memory_map(of_node, &e_ctrl->cal_data);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed: eeprom dt parse rc %d", rc);
		return rc;
	}
	rc = cam_eeprom_power_up(e_ctrl, power_info);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed: eeprom power up rc %d", rc);
		goto data_mem_free;
	}

	e_ctrl->cam_eeprom_state = CAM_EEPROM_CONFIG;
	if (e_ctrl->eeprom_device_type == MSM_CAMERA_SPI_DEVICE) {
		rc = cam_eeprom_match_id(e_ctrl);
		if (rc) {
			CAM_DBG(CAM_EEPROM, "eeprom not matching %d", rc);
			goto power_down;
		}
	}
	rc = cam_eeprom_read_memory(e_ctrl, &e_ctrl->cal_data);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "read_eeprom_memory failed");
		goto power_down;
	}

	rc = cam_eeprom_power_down(e_ctrl);
	if (rc)
		CAM_ERR(CAM_EEPROM, "failed: eeprom power down rc %d", rc);

	e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
	return rc;
power_down:
	cam_eeprom_power_down(e_ctrl);
data_mem_free:
	vfree(e_ctrl->cal_data.mapdata);
	vfree(e_ctrl->cal_data.map);
	e_ctrl->cal_data.num_data = 0;
	e_ctrl->cal_data.num_map = 0;
	e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
	return rc;
}

/**
 * cam_eeprom_get_dev_handle - get device handle
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_get_dev_handle(struct cam_eeprom_ctrl_t *e_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    eeprom_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (e_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_EEPROM, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&eeprom_acq_dev,
		u64_to_user_ptr(cmd->handle),
		sizeof(eeprom_acq_dev))) {
		CAM_ERR(CAM_EEPROM,
			"EEPROM:ACQUIRE_DEV: copy from user failed");
		return -EFAULT;
	}

	bridge_params.session_hdl = eeprom_acq_dev.session_handle;
	bridge_params.ops = &e_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = e_ctrl;
	bridge_params.dev_id = CAM_EEPROM;
	eeprom_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	e_ctrl->bridge_intf.device_hdl = eeprom_acq_dev.device_handle;
	e_ctrl->bridge_intf.session_hdl = eeprom_acq_dev.session_handle;

	CAM_DBG(CAM_EEPROM, "Device Handle: %d", eeprom_acq_dev.device_handle);
	if (copy_to_user(u64_to_user_ptr(cmd->handle),
		&eeprom_acq_dev, sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_EEPROM, "EEPROM:ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

/**
 * cam_eeprom_update_slaveInfo - Update slave info
 * @e_ctrl:     ctrl structure
 * @cmd_buf:    command buffer
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_update_slaveInfo(struct cam_eeprom_ctrl_t *e_ctrl,
	void *cmd_buf)
{
	int32_t                         rc = 0;
	struct cam_eeprom_soc_private  *soc_private;
	struct cam_cmd_i2c_info        *cmd_i2c_info = NULL;

	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	cmd_i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;
	soc_private->i2c_info.slave_addr = cmd_i2c_info->slave_addr;
	soc_private->i2c_info.i2c_freq_mode = cmd_i2c_info->i2c_freq_mode;

	rc = cam_eeprom_update_i2c_info(e_ctrl,
		&soc_private->i2c_info);
	CAM_DBG(CAM_EEPROM, "Slave addr: 0x%x Freq Mode: %d",
		soc_private->i2c_info.slave_addr,
		soc_private->i2c_info.i2c_freq_mode);

	return rc;
}

/**
 * cam_eeprom_parse_memory_map - Parse memory map info
 * @data:             memory block data
 * @cmd_buf:          command buffer
 * @cmd_length:       command buffer length
 * @num_map:          memory map size
 * @cmd_length_bytes: command length processed in this function
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_parse_memory_map(
	struct cam_eeprom_memory_block_t *data,
	void *cmd_buf, int cmd_length, uint16_t *cmd_length_bytes,
	int *num_map, size_t remain_buf_len)
{
	int32_t                            rc = 0;
	int32_t                            cnt = 0;
	int32_t                            processed_size = 0;
	uint8_t                            generic_op_code;
	struct cam_eeprom_memory_map_t    *map = data->map;
	struct common_header              *cmm_hdr =
		(struct common_header *)cmd_buf;
	uint16_t                           cmd_length_in_bytes = 0;
	struct cam_cmd_i2c_random_wr      *i2c_random_wr = NULL;
	struct cam_cmd_i2c_continuous_rd  *i2c_cont_rd = NULL;
	struct cam_cmd_conditional_wait   *i2c_poll = NULL;
	struct cam_cmd_unconditional_wait *i2c_uncond_wait = NULL;
	size_t                             validate_size = 0;

	generic_op_code = cmm_hdr->third_byte;

	if (cmm_hdr->cmd_type == CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR)
		validate_size = sizeof(struct cam_cmd_i2c_random_wr);
	else if (cmm_hdr->cmd_type == CAMERA_SENSOR_CMD_TYPE_I2C_CONT_RD)
		validate_size = sizeof(struct cam_cmd_i2c_continuous_rd);
	else if (cmm_hdr->cmd_type == CAMERA_SENSOR_CMD_TYPE_WAIT)
		validate_size = sizeof(struct cam_cmd_unconditional_wait);

	if (remain_buf_len < validate_size ||
	    *num_map >= (MSM_EEPROM_MAX_MEM_MAP_CNT *
		MSM_EEPROM_MEMORY_MAP_MAX_SIZE)) {
		CAM_ERR(CAM_EEPROM, "not enough buffer");
		return -EINVAL;
	}
	switch (cmm_hdr->cmd_type) {
	case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR:
		i2c_random_wr = (struct cam_cmd_i2c_random_wr *)cmd_buf;

		if (i2c_random_wr->header.count == 0 ||
		    i2c_random_wr->header.count >= MSM_EEPROM_MAX_MEM_MAP_CNT ||
		    (size_t)*num_map >= ((MSM_EEPROM_MAX_MEM_MAP_CNT *
				MSM_EEPROM_MEMORY_MAP_MAX_SIZE) -
				i2c_random_wr->header.count)) {
			CAM_ERR(CAM_EEPROM, "OOB Error");
			return -EINVAL;
		}
		cmd_length_in_bytes   = sizeof(struct cam_cmd_i2c_random_wr) +
			((i2c_random_wr->header.count - 1) *
			sizeof(struct i2c_random_wr_payload));

		if (cmd_length_in_bytes > remain_buf_len) {
			CAM_ERR(CAM_EEPROM, "Not enough buffer remaining");
			return -EINVAL;
		}
		for (cnt = 0; cnt < (i2c_random_wr->header.count);
			cnt++) {
			map[*num_map + cnt].page.addr =
				i2c_random_wr->random_wr_payload[cnt].reg_addr;
			map[*num_map + cnt].page.addr_type =
				i2c_random_wr->header.addr_type;
			map[*num_map + cnt].page.data =
				i2c_random_wr->random_wr_payload[cnt].reg_data;
			map[*num_map + cnt].page.data_type =
				i2c_random_wr->header.data_type;
			map[*num_map + cnt].page.valid_size = 1;
		}

		*num_map += (i2c_random_wr->header.count - 1);
		cmd_buf += cmd_length_in_bytes / sizeof(int32_t);
		processed_size +=
			cmd_length_in_bytes;
		break;
	case CAMERA_SENSOR_CMD_TYPE_I2C_CONT_RD:
		i2c_cont_rd = (struct cam_cmd_i2c_continuous_rd *)cmd_buf;
		cmd_length_in_bytes = sizeof(struct cam_cmd_i2c_continuous_rd);

		if (i2c_cont_rd->header.count >= U32_MAX - data->num_data) {
			CAM_ERR(CAM_EEPROM,
				"int overflow on eeprom memory block");
			return -EINVAL;
		}
		map[*num_map].mem.addr = i2c_cont_rd->reg_addr;
		map[*num_map].mem.addr_type = i2c_cont_rd->header.addr_type;
		map[*num_map].mem.data_type = i2c_cont_rd->header.data_type;
		map[*num_map].mem.valid_size =
			i2c_cont_rd->header.count;
		cmd_buf += cmd_length_in_bytes / sizeof(int32_t);
		processed_size +=
			cmd_length_in_bytes;
		data->num_data += map[*num_map].mem.valid_size;
		break;
	case CAMERA_SENSOR_CMD_TYPE_WAIT:
		if (generic_op_code ==
			CAMERA_SENSOR_WAIT_OP_HW_UCND ||
			generic_op_code ==
			CAMERA_SENSOR_WAIT_OP_SW_UCND) {
			i2c_uncond_wait =
				(struct cam_cmd_unconditional_wait *)cmd_buf;
			cmd_length_in_bytes =
				sizeof(struct cam_cmd_unconditional_wait);

			if (*num_map < 1) {
				CAM_ERR(CAM_EEPROM,
					"invalid map number, num_map=%d",
					*num_map);
				return -EINVAL;
			}

			/*
			 * Though delay is added all of them, but delay will
			 * be applicable to only one of them as only one of
			 * them will have valid_size set to >= 1.
			 */
			map[*num_map - 1].mem.delay = i2c_uncond_wait->delay;
			map[*num_map - 1].page.delay = i2c_uncond_wait->delay;
			map[*num_map - 1].pageen.delay = i2c_uncond_wait->delay;
		} else if (generic_op_code ==
			CAMERA_SENSOR_WAIT_OP_COND) {
			i2c_poll = (struct cam_cmd_conditional_wait *)cmd_buf;
			cmd_length_in_bytes =
				sizeof(struct cam_cmd_conditional_wait);

			map[*num_map].poll.addr = i2c_poll->reg_addr;
			map[*num_map].poll.addr_type = i2c_poll->addr_type;
			map[*num_map].poll.data = i2c_poll->reg_data;
			map[*num_map].poll.data_type = i2c_poll->data_type;
			map[*num_map].poll.delay = i2c_poll->timeout;
			map[*num_map].poll.valid_size = 1;
		}
		cmd_buf += cmd_length_in_bytes / sizeof(int32_t);
		processed_size +=
			cmd_length_in_bytes;
		break;
	default:
		break;
	}

	*cmd_length_bytes = processed_size;
	return rc;
}

/**
 * cam_eeprom_init_pkt_parser - Parse eeprom packet
 * @e_ctrl:       ctrl structure
 * @csl_packet:	  csl packet received
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_init_pkt_parser(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_packet *csl_packet)
{
	int32_t                         rc = 0;
	int                             i = 0;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uint32_t                       *offset = NULL;
	uint32_t                       *cmd_buf = NULL;
	uintptr_t                        generic_pkt_addr;
	size_t                          pkt_len = 0;
	size_t                          remain_len = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	uint32_t                        processed_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uint16_t                        cmd_length_in_bytes = 0;
	struct cam_cmd_i2c_info        *i2c_info = NULL;
	int                             num_map = -1;
	struct cam_eeprom_memory_map_t *map = NULL;
	struct cam_eeprom_soc_private  *soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;
        struct cam_sensor_cci_client *cci_client = NULL;

	if (e_ctrl->io_master_info.cci_client) {
		cci_client = e_ctrl->io_master_info.cci_client;
	}

	e_ctrl->cal_data.map = vzalloc((MSM_EEPROM_MEMORY_MAP_MAX_SIZE *
		MSM_EEPROM_MAX_MEM_MAP_CNT) *
		(sizeof(struct cam_eeprom_memory_map_t)));
	if (!e_ctrl->cal_data.map) {
		rc = -ENOMEM;
		CAM_ERR(CAM_EEPROM, "failed");
		return rc;
	}
	map = e_ctrl->cal_data.map;

	offset = (uint32_t *)&csl_packet->payload;
	offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);

	/* Loop through multiple command buffers */
	for (i = 0; i < csl_packet->num_cmd_buf; i++) {
		total_cmd_buf_in_bytes = cmd_desc[i].length;
		processed_cmd_buf_in_bytes = 0;
		if (!total_cmd_buf_in_bytes)
			continue;
		rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
			&generic_pkt_addr, &pkt_len);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "Failed to get cpu buf");
			return rc;
		}
		cmd_buf = (uint32_t *)generic_pkt_addr;
		if (!cmd_buf) {
			CAM_ERR(CAM_EEPROM, "invalid cmd buf");
			rc = -EINVAL;
			goto rel_cmd_buf;
		}

		if ((pkt_len < sizeof(struct common_header)) ||
			(cmd_desc[i].offset > (pkt_len -
			sizeof(struct common_header)))) {
			CAM_ERR(CAM_EEPROM, "Not enough buffer");
			rc = -EINVAL;
			goto rel_cmd_buf;
		}
		remain_len = pkt_len - cmd_desc[i].offset;
		cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);

		if (total_cmd_buf_in_bytes > remain_len) {
			CAM_ERR(CAM_EEPROM, "Not enough buffer for command");
			rc = -EINVAL;
			goto rel_cmd_buf;
		}
		/* Loop through multiple cmd formats in one cmd buffer */
		while (processed_cmd_buf_in_bytes < total_cmd_buf_in_bytes) {
			if ((remain_len - processed_cmd_buf_in_bytes) <
				sizeof(struct common_header)) {
				CAM_ERR(CAM_EEPROM, "Not enough buf");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}
			cmm_hdr = (struct common_header *)cmd_buf;
			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;
				if ((remain_len - processed_cmd_buf_in_bytes) <
					sizeof(struct cam_cmd_i2c_info)) {
					CAM_ERR(CAM_EEPROM, "Not enough buf");
					rc = -EINVAL;
					goto rel_cmd_buf;
				}
				/* Configure the following map slave address */
				map[num_map + 1].saddr = i2c_info->slave_addr;
				rc = cam_eeprom_update_slaveInfo(e_ctrl,
					cmd_buf);
				cmd_length_in_bytes =
					sizeof(struct cam_cmd_i2c_info);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
					sizeof(uint32_t);
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				cmd_length_in_bytes = total_cmd_buf_in_bytes;
				rc = cam_sensor_update_power_settings(cmd_buf,
					cmd_length_in_bytes, power_info,
					(remain_len -
					processed_cmd_buf_in_bytes));
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
					sizeof(uint32_t);
				if (rc) {
					CAM_ERR(CAM_EEPROM, "Failed");
					goto rel_cmd_buf;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR:
			case CAMERA_SENSOR_CMD_TYPE_I2C_CONT_RD:
			case CAMERA_SENSOR_CMD_TYPE_WAIT:
				num_map++;
				rc = cam_eeprom_parse_memory_map(
					&e_ctrl->cal_data, cmd_buf,
					total_cmd_buf_in_bytes,
					&cmd_length_in_bytes, &num_map,
					(remain_len -
					processed_cmd_buf_in_bytes));
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/sizeof(uint32_t);
				break;
			default:
				break;
			}
		}
		e_ctrl->cal_data.num_map = num_map + 1;

		if(e_ctrl->io_master_info.cci_client){
			CAM_ERR(CAM_EEPROM,"saddr=0x%x,num_map=%d,cci_i2c_master=%d",map[0].saddr,num_map,cci_client->cci_i2c_master);
			if(map[0].saddr == SLAVE_ADDRESS && cci_client->cci_i2c_master == CCI_WRITE_MASTER) {
				power_setting_array_dc =
					kzalloc(sizeof(struct msm_eeprom_power_setting_array),
						GFP_KERNEL);
				if (power_setting_array_dc ==  NULL) {
					CAM_ERR(CAM_EEPROM,"power_setting_array_dc Mem Alloc Fail");
					rc = -EINVAL;
					goto rel_cmd_buf;
				}
				msm_eeprom_copy_power_settings_compat(
					power_setting_array_dc,
					power_info);
			}
		}

		if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
			CAM_WARN(CAM_EEPROM, "Failed to put cpu buf: 0x%x",
				cmd_desc[i].mem_handle);
	}

	return rc;

rel_cmd_buf:
	if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
		CAM_WARN(CAM_EEPROM, "Failed to put cpu buf: 0x%x",
			cmd_desc[i].mem_handle);

	return rc;
}

/**
 * cam_eeprom_get_cal_data - parse the userspace IO config and
 *                                        copy read data to share with userspace
 * @e_ctrl:     ctrl structure
 * @csl_packet: csl packet received
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_get_cal_data(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_packet *csl_packet)
{
	struct cam_buf_io_cfg *io_cfg;
	uint32_t              i = 0;
	int                   rc = 0;
	uintptr_t              buf_addr;
	size_t                buf_size;
	uint8_t               *read_buffer;
	size_t                remain_len = 0;

	io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
		&csl_packet->payload +
		csl_packet->io_configs_offset);

	CAM_DBG(CAM_EEPROM, "number of IO configs: %d:",
		csl_packet->num_io_configs);

	for (i = 0; i < csl_packet->num_io_configs; i++) {
		CAM_DBG(CAM_EEPROM, "Direction: %d:", io_cfg->direction);
		if (io_cfg->direction == CAM_BUF_OUTPUT) {
			rc = cam_mem_get_cpu_buf(io_cfg->mem_handle[0],
				&buf_addr, &buf_size);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "Fail in get buffer: %d",
					rc);
				return rc;
			}
			if (buf_size <= io_cfg->offsets[0]) {
				CAM_ERR(CAM_EEPROM, "Not enough buffer");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}

			remain_len = buf_size - io_cfg->offsets[0];
			CAM_DBG(CAM_EEPROM, "buf_addr : %pK, buf_size : %zu\n",
				(void *)buf_addr, buf_size);

			read_buffer = (uint8_t *)buf_addr;
			if (!read_buffer) {
				CAM_ERR(CAM_EEPROM,
					"invalid buffer to copy data");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}
			read_buffer += io_cfg->offsets[0];

			if (remain_len < e_ctrl->cal_data.num_data) {
				CAM_ERR(CAM_EEPROM,
					"failed to copy, Invalid size");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}

			CAM_DBG(CAM_EEPROM, "copy the data, len:%d",
				e_ctrl->cal_data.num_data);
			memcpy(read_buffer, e_ctrl->cal_data.mapdata,
					e_ctrl->cal_data.num_data);
			if (cam_mem_put_cpu_buf(io_cfg->mem_handle[0]))
				CAM_WARN(CAM_EEPROM, "Fail in put buffer: 0x%x",
					io_cfg->mem_handle[0]);
		} else {
			CAM_ERR(CAM_EEPROM, "Invalid direction");
			rc = -EINVAL;
		}
	}

	return rc;

rel_cmd_buf:
	if (cam_mem_put_cpu_buf(io_cfg->mem_handle[0]))
		CAM_WARN(CAM_EEPROM, "Fail in put buffer : 0x%x",
			io_cfg->mem_handle[0]);

	return rc;
}

/**
 * cam_eeprom_pkt_parse - Parse csl packet
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_pkt_parse(struct cam_eeprom_ctrl_t *e_ctrl, void *arg)
{
	int32_t                         rc = 0;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	uintptr_t                        generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remain_len = 0;
	struct cam_packet              *csl_packet = NULL;
	struct cam_eeprom_soc_private  *soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	ioctl_ctrl = (struct cam_control *)arg;

	if (copy_from_user(&dev_config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_EEPROM,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remain_len = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_EEPROM,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), pkt_len);
		rc = -EINVAL;
		goto release_buf;
	}

	remain_len -= (size_t)dev_config.offset;
	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_EEPROM, "Invalid packet params");
		rc = -EINVAL;
		goto release_buf;
	}

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_EEPROM_PACKET_OPCODE_INIT:
		if (e_ctrl->userspace_probe == false) {
			rc = cam_eeprom_parse_read_memory_map(
					e_ctrl->soc_info.dev->of_node, e_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_EEPROM, "Failed: rc : %d", rc);
				goto release_buf;
			}
			rc = cam_eeprom_get_cal_data(e_ctrl, csl_packet);
			vfree(e_ctrl->cal_data.mapdata);
			vfree(e_ctrl->cal_data.map);
			e_ctrl->cal_data.num_data = 0;
			e_ctrl->cal_data.num_map = 0;
			CAM_DBG(CAM_EEPROM,
				"Returning the data using kernel probe");
			break;
		}
		rc = cam_eeprom_init_pkt_parser(e_ctrl, csl_packet);
		if (rc) {
			CAM_ERR(CAM_EEPROM,
				"Failed in parsing the pkt");
			goto release_buf;
		}

		e_ctrl->cal_data.mapdata =
			vzalloc(e_ctrl->cal_data.num_data);
		if (!e_ctrl->cal_data.mapdata) {
			rc = -ENOMEM;
			CAM_ERR(CAM_EEPROM, "failed");
			goto error;
		}

		rc = cam_eeprom_power_up(e_ctrl,
			&soc_private->power_info);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "failed rc %d", rc);
			goto memdata_free;
		}

		e_ctrl->cam_eeprom_state = CAM_EEPROM_CONFIG;
		rc = cam_eeprom_read_memory(e_ctrl, &e_ctrl->cal_data);
		if (rc) {
			CAM_ERR(CAM_EEPROM,
				"read_eeprom_memory failed");
			goto power_down;
		}

		rc = cam_eeprom_get_cal_data(e_ctrl, csl_packet);
		rc = cam_eeprom_power_down(e_ctrl);
		e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
		vfree(e_ctrl->cal_data.mapdata);
		vfree(e_ctrl->cal_data.map);
		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_setting_size = 0;
		power_info->power_down_setting_size = 0;
		e_ctrl->cal_data.num_data = 0;
		e_ctrl->cal_data.num_map = 0;
		break;
	default:
		break;
	}

	if (cam_mem_put_cpu_buf(dev_config.packet_handle))
		CAM_WARN(CAM_EEPROM, "Put cpu buffer failed : 0x%x",
			dev_config.packet_handle);

	return rc;

power_down:
	cam_eeprom_power_down(e_ctrl);
memdata_free:
	vfree(e_ctrl->cal_data.mapdata);
error:
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	vfree(e_ctrl->cal_data.map);
	e_ctrl->cal_data.num_data = 0;
	e_ctrl->cal_data.num_map = 0;
	e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
release_buf:
	if (cam_mem_put_cpu_buf(dev_config.packet_handle))
		CAM_WARN(CAM_EEPROM, "Put cpu buffer failed : 0x%x",
			dev_config.packet_handle);

	return rc;
}

void cam_eeprom_shutdown(struct cam_eeprom_ctrl_t *e_ctrl)
{
	int rc;
	struct cam_eeprom_soc_private  *soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (e_ctrl->cam_eeprom_state == CAM_EEPROM_INIT)
		return;

	if (e_ctrl->cam_eeprom_state == CAM_EEPROM_CONFIG) {
		rc = cam_eeprom_power_down(e_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_EEPROM, "EEPROM Power down failed");
		e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
	}

	if (e_ctrl->cam_eeprom_state == CAM_EEPROM_ACQUIRE) {
		rc = cam_destroy_device_hdl(e_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_EEPROM, "destroying the device hdl");

		e_ctrl->bridge_intf.device_hdl = -1;
		e_ctrl->bridge_intf.link_hdl = -1;
		e_ctrl->bridge_intf.session_hdl = -1;

		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_setting_size = 0;
		power_info->power_down_setting_size = 0;
	}

	e_ctrl->cam_eeprom_state = CAM_EEPROM_INIT;
}

/**
 * cam_eeprom_driver_cmd - Handle eeprom cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int32_t cam_eeprom_driver_cmd(struct cam_eeprom_ctrl_t *e_ctrl, void *arg)
{
	int                            rc = 0;
	struct cam_eeprom_query_cap_t  eeprom_cap = {0};
	struct cam_control            *cmd = (struct cam_control *)arg;

	if (!e_ctrl || !cmd) {
		CAM_ERR(CAM_EEPROM, "Invalid Arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_EEPROM, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	mutex_lock(&(e_ctrl->eeprom_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		eeprom_cap.slot_info = e_ctrl->soc_info.index;
		if (e_ctrl->userspace_probe == false)
			eeprom_cap.eeprom_kernel_probe = true;
		else
			eeprom_cap.eeprom_kernel_probe = false;

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&eeprom_cap,
			sizeof(struct cam_eeprom_query_cap_t))) {
			CAM_ERR(CAM_EEPROM, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_EEPROM, "eeprom_cap: ID: %d", eeprom_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_eeprom_get_dev_handle(e_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "Failed to acquire dev");
			goto release_mutex;
		}
		e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
		break;
	case CAM_RELEASE_DEV:
		if (e_ctrl->cam_eeprom_state != CAM_EEPROM_ACQUIRE) {
			rc = -EINVAL;
			CAM_WARN(CAM_EEPROM,
			"Not in right state to release : %d",
			e_ctrl->cam_eeprom_state);
			goto release_mutex;
		}

		if (e_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_EEPROM,
				"Invalid Handles: link hdl: %d device hdl: %d",
				e_ctrl->bridge_intf.device_hdl,
				e_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(e_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_EEPROM,
				"failed in destroying the device hdl");
		e_ctrl->bridge_intf.device_hdl = -1;
		e_ctrl->bridge_intf.link_hdl = -1;
		e_ctrl->bridge_intf.session_hdl = -1;
		e_ctrl->cam_eeprom_state = CAM_EEPROM_INIT;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_eeprom_pkt_parse(e_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "Failed in eeprom pkt Parsing");
			goto release_mutex;
		}
		break;
	default:
		CAM_DBG(CAM_EEPROM, "invalid opcode");
		break;
	}

release_mutex:
	mutex_unlock(&(e_ctrl->eeprom_mutex));

	return rc;
}

ssize_t calibration_flag_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  int rc = -1;
  struct cam_sensor_cci_client *cci_client = NULL;
  struct cam_eeprom_ctrl_t *e_ctrl = NULL;
  struct cam_eeprom_soc_private *eb_info = NULL;
  struct cam_sensor_power_ctrl_t *power_info = NULL;

  struct platform_device *pdev = container_of(dev, struct platform_device, dev);

  e_ctrl = platform_get_drvdata(pdev);

  if (!e_ctrl) {
    CAM_ERR(CAM_EEPROM, "eeprom device is NULL");
    return 0;
  }
  cali_flag = 0x0;
  ae_cali_flag = 0x0;

  eb_info = (struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
  power_info = &eb_info->power_info;
  eb_info->i2c_info.slave_addr = SLAVE_ADDRESS;
  power_info->dev = &pdev->dev;

  if (e_ctrl->io_master_info.cci_client) {
  	e_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
  }

  cci_client = e_ctrl->io_master_info.cci_client;
  cci_client->sid = eb_info->i2c_info.slave_addr >> 1;
  cci_client->cci_i2c_master = CCI_WRITE_MASTER;

  if(!power_setting_array_dc){
    CAM_ERR(CAM_EEPROM, "power_setting_array_dc is NULL");
    return 0;
  }
  power_info->power_setting =
    power_setting_array_dc->power_setting_a;
  power_info->power_down_setting =
    power_setting_array_dc->power_down_setting_a;
  power_info->power_setting_size =
    power_setting_array_dc->size;
  power_info->power_down_setting_size =
    power_setting_array_dc->size_down;

  rc = cam_eeprom_power_up(e_ctrl, power_info);
  if (rc < 0) {
    CAM_ERR(CAM_EEPROM, "Power Up failed for eeprom");
  }

  cali_flag = read_calibration_flag(e_ctrl);
  CAM_ERR(CAM_EEPROM, "Done: calibration_flag = 0x%x\n", cali_flag);

  ae_cali_flag = read_ae_calibration_flag(e_ctrl);
  CAM_ERR(CAM_EEPROM, "Done: ae_cali_flag = 0x%x\n", ae_cali_flag);

  rc = cam_eeprom_power_down(e_ctrl);
  if (rc) {
    CAM_ERR(CAM_EEPROM, "failed power down rc %d", rc);
  }
  return sprintf(buf, "cali_flag = 0x%x,ae_cali_flag = 0x%x\n", cali_flag,ae_cali_flag);
}

ssize_t calibration_flag_store(struct device *dev,  struct device_attribute *attr, const char *buf, size_t count)
{
  int value = 0;

  sscanf(buf, "%d", &value);
  CAM_ERR(CAM_EEPROM, "value %d", value);

  return count;
}


ssize_t calibration_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  return sprintf(buf, "%d\n", cali_value);
}

ssize_t calibration_data_store(struct device *dev,  struct device_attribute *attr, const char *buf, size_t count)
{
  int write_enable = 0;
  int size = -1, rc = -1;
  struct cam_sensor_cci_client *cci_client = NULL;
  struct cam_eeprom_ctrl_t *e_ctrl = NULL;
  struct cam_eeprom_soc_private *eb_info = NULL;
  struct cam_sensor_power_ctrl_t *power_info = NULL;

  struct platform_device *pdev = container_of(dev, struct platform_device, dev);

  e_ctrl = platform_get_drvdata(pdev);

  if (!e_ctrl) {
    CAM_ERR(CAM_EEPROM, "eeprom device is NULL");
    return count;
  }

  sscanf(buf, "%d", &write_enable);
  cali_value = write_enable;
  if (write_enable !=0){
    CAM_ERR(CAM_EEPROM, "start to read cali data,write_enable=%d",write_enable);
    size = read_dualcam_cali_data(write_enable);
    if (size <= 0) {
      CAM_ERR(CAM_EEPROM, "Fail to get new calibration data");
      return count;
    }

    eb_info = (struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
    if(!eb_info){
      CAM_ERR(CAM_EEPROM, "eb_info is NULL");
      goto end;
    }
    power_info = &eb_info->power_info;
    eb_info->i2c_info.slave_addr = SLAVE_ADDRESS;
    power_info->dev = &pdev->dev;

    if (e_ctrl->io_master_info.cci_client) {
      e_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
      cci_client = e_ctrl->io_master_info.cci_client;
      cci_client->sid = eb_info->i2c_info.slave_addr >> 1;
      cci_client->cci_i2c_master = CCI_WRITE_MASTER;
    }



    if(!power_setting_array_dc){
      CAM_ERR(CAM_EEPROM, "power_setting_array_dc is NULL");
      goto end;
    }
    power_info->power_setting =
      power_setting_array_dc->power_setting_a;
    power_info->power_down_setting =
      power_setting_array_dc->power_down_setting_a;
    power_info->power_setting_size =
      power_setting_array_dc->size;
    power_info->power_down_setting_size =
      power_setting_array_dc->size_down;

    rc = cam_eeprom_power_up(e_ctrl, power_info);
    if (rc < 0) {
      CAM_ERR(CAM_EEPROM, "Power Up failed for eeprom");
      goto end;
    }

    if(write_enable == 1){
        rc = write_eeprom_memory(e_ctrl, &e_ctrl->cal_data,CALI_DATA_SIZE);
        if (rc < 0) {
          CAM_ERR(CAM_EEPROM, "failed to write_eeprom_memory !");
        }
        //write_dualcam_cali_flag(PATH_CALI_FLAG,cali_flag);
      } else if(write_enable == 2){
        rc = write_eeprom_dc_ae(e_ctrl, &e_ctrl->cal_data,AE_BLOCK_SIZE);
        if (rc < 0) {
          CAM_ERR(CAM_EEPROM, "failed to write_eeprom_dc_ae !");
        }
        //write_dualcam_cali_flag(PATH_AE_FLAG,ae_cali_flag);
      }else if(write_enable == 3){
        rc = calibration_check(e_ctrl);
        if (rc < 0) {
          CAM_ERR(CAM_EEPROM, "failed to calibration_check");
      }
    }

    rc = cam_eeprom_power_down(e_ctrl);
    if (rc) {
      CAM_ERR(CAM_EEPROM, "failed power down rc %d", rc);
    }

  }

end:
  kfree(bin_buffer);
  return count;
}

