/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __HB_CAMERA_COMMON_H__
#define __HB_CAMERA_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hb_cam_interface.h"
#include "logging.h"

#define CAM_MAX_NUM	8
#define GPIO_NUMBER 6
#define SERDES_NUMBER 2
#define MAX_NUM_LENGTH 128
#define ENTRY_NUM 4
#define PWM_PIN_NUM  4

#define RET_OK 0
#define	RET_ERROR 1

#define SENSOR_IOC_MAGIC   'x'
#define SENSOR_OPEN_CNT        _IOR(SENSOR_IOC_MAGIC, 1, int)
#define SENSOR_SET_START_CNT   _IOW(SENSOR_IOC_MAGIC, 2, int)
#define SENSOR_GET_START_CNT   _IOR(SENSOR_IOC_MAGIC, 3, int)
#define SENSOR_USER_LOCK       _IOW(SENSOR_IOC_MAGIC, 4, int)
#define SENSOR_USER_UNLOCK     _IOW(SENSOR_IOC_MAGIC, 5, int)
#define SENSOR_AE_SHARE	       _IOW(SENSOR_IOC_MAGIC, 6, int)
#define SENSOR_SET_INIT_CNT    _IOW(SENSOR_IOC_MAGIC, 8, int)
#define SENSOR_GET_INIT_CNT    _IOR(SENSOR_IOC_MAGIC, 9, int)


enum HB_CAM_ERROR_CODE {
    HB_CAM_PARSE_BOARD_CFG_FAIL = 2,
    HB_CAM_PARSE_MIPI_CFG_FAIL,
    HB_CAM_DLOPEN_LIBRARY_FAIL,
    HB_CAM_INIT_FAIL,
    HB_CAM_DEINIT_FAIL,
    HB_CAM_START_FAIL,
    HB_CAM_STOP_FAIL,
    HB_CAM_I2C_WRITE_FAIL,
    HB_CAM_I2C_WRITE_BYTE_FAIL,
    HB_CAM_I2C_WRITE_BLOCK_FAIL,
    HB_CAM_I2C_READ_BLOCK_FAIL,
    HB_CAM_DYNAMIC_SWITCH_FAIL,
    HB_CAM_DYNAMIC_SWITCH_FPS_FAIL,
    HB_CAM_SERDES_POWERON_FAIL,
    HB_CAM_SERDES_CONFIG_FAIL,
    HB_CAM_SERDES_STREAM_ON_FAIL,
    HB_CAM_SERDES_STREAM_OFF_FAIL,
    HB_CAM_SENSOR_POWERON_FAIL,
    HB_CAM_SENSOR_POWEROFF_FAIL,
    HB_CAM_START_PHYSICAL_FAIL,
    HB_CAM_SPI_WRITE_BLOCK_FAIL,
    HB_CAM_SPI_READ_BLOCK_FAIL,
    HB_CAM_INVALID_PARAM,
    HB_CAM_SET_EX_GAIN_FAIL,
    HB_CAM_SET_AWB_FAIL,
    HB_CAM_I2C_READ_FAIL,
    HB_CAM_I2C_READ_BYTE_FAIL,
    HB_CAM_RESET_FAIL,
    HB_CAM_OPS_NOT_SUPPORT,
    HB_CAM_ISP_POWERON_FAIL,
    HB_CAM_ISP_POWEROFF_FAIL,
    HB_CAM_ISP_RESET_FAIL,
    HB_CAM_ENABLE_CLK_FAIL,
    HB_CAM_DISABLE_CLK_FAIL,
    HB_CAM_SET_CLK_FAIL,
    HB_CAM_DYNAMIC_SWITCH_MODE_FAIL,
};

/* for interface type */
#define INTERFACE_MIPI	"mipi"
#define INTERFACE_BT	"bt"
#define INTERFACE_DVP	"dvp"
#define INTERFACE_SDIO	"sdio"
#define INTERFACE_NET	"net"

/* not use now*/
enum GPIO_DEF_VALUE{
	ISP_RESET,
	SENSOR_POWER_ON,
	SENSOR_RESET,
	S954_POWER_ON,
	S954_RESET
};

typedef struct spi_data {
	int spi_mode;
	int spi_cs;
	uint32_t spi_speed;
}spi_data_t;

typedef struct mutex_package {
	pthread_mutex_t lock;
	pthread_mutexattr_t lock_attr;
	int shmid;
} mutex_package_t;

typedef struct sensor_info_s {
	int port;
	int bus_type;
	int bus_num;
	int isp_addr;
	int sensor_addr;
	int sensor1_addr;
	int serial_addr;
	int serial_addr1;
	int imu_addr;
	int sensor_clk;
	int eeprom_addr;
	int power_mode;
	int sensor_mode;
	int entry_num;
	int reg_width;
	int gpio_num;
	int gpio_pin[GPIO_NUMBER];
	int gpio_level[GPIO_NUMBER];
	int fps;
	int width;
	int height;
	int format;
	int resolution;
	int extra_mode;
	int power_delay;
	int deserial_index;
	int deserial_port;
	char *sensor_name;
	char *config_path;
	void *sensor_ops;
	void *sensor_fd;
	void *deserial_info;
	int stream_control;
	int config_index;
	spi_data_t spi_info;
	int init_state;
    int start_state;
	int sen_devfd;
	int dev_port;
	mutex_package_t *mp;
}sensor_info_t;

typedef struct deserial_info_s {
	int bus_type;
	int bus_num;
	int deserial_addr;
	int power_mode;
	int physical_entry;
	int gpio_num;
	int gpio_pin[GPIO_NUMBER];
	int gpio_level[GPIO_NUMBER];
	char *deserial_name;
	char *deserial_config_path;
	void *deserial_ops;
	void *deserial_fd;
	void  *sensor_info[CAM_MAX_NUM];
	int init_state;
}deserial_info_t;

typedef struct lpwm_info_s {
	int lpwm_enable;
	int offset_us[PWM_PIN_NUM];
	int period_us[PWM_PIN_NUM];
	int duty_us[PWM_PIN_NUM];
	int lpwm_start;
	int lpwm_stop;
}lpwm_info_t;

typedef struct board_info_s {
	int  config_number;
	char *board_name;
	char *interface_type;
	int  deserial_num;
	int  port_number;
	lpwm_info_t lpwm_info;
	deserial_info_t deserial_info[SERDES_NUMBER];
	sensor_info_t sensor_info[CAM_MAX_NUM];
}board_info_t;

#define HAL_LINE_CONTROL       0x00000001
#define HAL_GAIN_CONTROL       0x00000002
#define HAL_AWB_CONTROL        0x00000004
#define HAL_AF_CONTROL     0x00000008
#define HAL_ZOOM_CONTROL   0x00000010

typedef struct hal_control_info_s {
	uint32_t port;
	uint32_t bus_type;
	uint32_t bus_num;
	uint32_t sensor_addr;
	uint32_t sensor1_addr;
	uint32_t serial_addr;
	uint32_t serial_addr1;
	uint32_t sensor_mode;
	uint32_t eeprom_addr;
	spi_data_t sensor_spi_info;
	uint32_t af_bus_num;
	uint32_t af_addr;
	uint32_t af_info[4];
	uint32_t zoom_bus_num;
	uint32_t zoom_addr;
	uint32_t zoom_info[4];
} hal_control_info_t;

typedef struct {
	const char *module;
	int (*init)(sensor_info_t *sensor_info);
	int (*deinit)(sensor_info_t *sensor_info);
	int (*start)(sensor_info_t *sensor_info);
	int (*stop)(sensor_info_t *sensor_info);
	int (*power_on)(sensor_info_t *sensor_info);
	int (*power_off)(sensor_info_t *sensor_info);
	int (*power_reset)(sensor_info_t *sensor_info);
	int (*extern_isp_poweron)(sensor_info_t *sensor_info);
	int (*extern_isp_poweroff)(sensor_info_t *sensor_info);
	int (*extern_isp_reset)(sensor_info_t *sensor_info);
	int (*spi_read)(sensor_info_t *sensor_info,  uint32_t reg_addr, char *buffer, uint32_t sizee);
	int (*spi_write)(sensor_info_t *sensor_info, uint32_t reg_addr, char *buffer, uint32_t sizee);
	int (*set_awb)(int i2c_bus, int sensor_addr, float rg_gain, float b_gain);
	int (*set_ex_gain)( int i2c_bus, int sensor_addr, uint32_t exposure_setting,
			uint32_t gain_setting_0, uint16_t gain_setting_1);
	int (*dynamic_switch_fps)(sensor_info_t *sensor_info, uint32_t fps);
	int (*ae_share_init)(uint32_t flag);
	int (*get_vts)(sensor_info_t *sensor_info, uint32_t *vts);
	int (*get_hts)(sensor_info_t *sensor_info, uint32_t *hts);
	int (*set_vts)(sensor_info_t *sensor_info, uint32_t *vts);
	int (*set_hts)(sensor_info_t *sensor_info, uint32_t *hts);
	int (*aexp_gain_control)(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num);
	int (*aexp_line_control)(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num);
	int (*awb_control)(hal_control_info_t *info, uint32_t mode, uint32_t rgain, uint32_t bgain, uint32_t grgain, uint32_t gbgain);
	int (*af_control)(hal_control_info_t *info, uint32_t mode, uint32_t pos);
	int (*zoom_control)(hal_control_info_t *info, uint32_t mode, uint32_t pos);
	int (*userspace_control)(uint32_t port, uint32_t *enable);
}sensor_module_t;

typedef struct {
	const char *module;
	int (*init)(deserial_info_t *deserial_info);
	int (*stream_on)(deserial_info_t *deserial_info, int port);
	int (*stream_off)(deserial_info_t *deserial_info, int port);
	int (*deinit)(deserial_info_t *deserial_info);
	int (*start_physical)(deserial_info_t *deserial_info);
	int (*reset)(deserial_info_t *deserial_info);
}deserial_module_t;

extern int hb_cam_htoi(char s[]);

#ifdef __cplusplus
}
#endif

#endif


