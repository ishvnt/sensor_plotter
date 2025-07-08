/*
 * mpu6050.h
 *
 *  Created on: Jul 1, 2025
 *      Author: ishant
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx.h"

#define MPU6050_ADDR				0x68
#define MPU6050_INTR_DATA_READY		(1<<0)
#define MPU6050_INTR_FIFO_OVERFLOW	(1<<4)
#define MPU6050_INTR_MOTION_DETECT	(1<<6)

#define MPU6050_SELF_TEST_X  		(0x0D)
#define MPU6050_SELF_TEST_Y  		(0x0E)
#define MPU6050_SELF_TEST_Z  		(0x0F)
#define MPU6050_SELF_TEST_A  		(0x10)
#define MPU6050_SMPLRT_DIV 			(0x19)
#define MPU6050_CONFIG				(0x1A)
#define MPU6050_GYRO_CONFIG			(0x1B)
#define MPU6050_ACCEL_CONFIG		(0x1C)
#define MPU6050_MOT_THR				(0x1F)
#define MPU6050_FIFO_EN				(0x23)
#define MPU6050_INT_PIN_CFG			(0x37)
#define MPU6050_INT_ENABLE			(0x38)
#define MPU6050_INT_STATUS			(0x3A)
#define MPU6050_ACCEL				(0x3B)		/*6 bytes, acceleration x, y, z*/
#define MPU6050_ACCEL_X				(0x3B)		/*2 byte register high byte at lower address*/
#define MPU6050_ACCEL_Y				(0x3D)		/*2 byte register high byte at lower address*/
#define MPU6050_ACCEL_Z				(0x3F)		/*2 byte register high byte at lower address*/
#define MPU6050_TEMP				(0x41)		/*2 byte register high byte at lower address*/
#define MPU6050_GYRO				(0x43)		/*2 byte register high byte at lower address*/
#define MPU6050_GYRO_X				(0x43)		/*2 byte register high byte at lower address*/
#define MPU6050_GYRO_Y				(0x45)		/*2 byte register high byte at lower address*/
#define MPU6050_GYRO_Z				(0x47)		/*2 byte register high byte at lower address*/
#define MPU6050_SIGNAL_PATH_RESET 	(0x68)
#define MPU6050_MOT_DETECT_CTRL		(0x69)
#define MPU6050_USER_CTRL			(0x6A)
#define MPU6050_PWR_MGT_1			(0x6B)
#define MPU6050_PWR_MGT_2			(0x6C)
#define MPU6050_FIFO_COUNT			(0x72)		/*2 byte register high byte at lower address*/
#define MPU6050_FIFO_R_W			(0x74)
#define MPU6050_WHO_AM_I			(0x75)

#define FIFO_EN_BIT					(1<<6)
#define FIFO_RESET_BIT				(1<<2)
#define ACCL_FIFO_EN_BIT			(1<<3)
#define GYRO_FIFO_EN_BIT			(1<<4 | 1<<5 | 1<<6)
#define INT_LEVEL_BIT				(1<<7)
#define INT_OPEN_BIT				(1<<6)
#define LATCH_INT_EN				(1<<5)
#define INT_RD_CLEAR				(1<<4)
#define DEVICE_RESET				(1<<7)
#define SLEEP						(1<<6)
#define CYCLE						(1<<5)
#define TEMP_DIS					(1<<3)
#define STANDBY_GYRO_X				(1<<2)
#define STANDBY_GYRO_Y				(1<<1)
#define STANDBY_GYRO_Z				(1<<0)
#define STANDBY_ACCL_X				(1<<5)
#define STANDBY_ACCL_Y				(1<<4)
#define STANDBY_ACCL_Z				(1<<3)

#define FIFO_ACCL_PACKET_SIZE 6
#define FIFO_GYRO_PACKET_SIZE 6
#define FIFO_BOTH_PACKET_SIZE (FIFO_ACCL_PACKET_SIZE+FIFO_GYRO_PACKET_SIZE)

typedef struct
{
	float acc_x;
	float acc_y;
	float acc_z;
} accl_t;

typedef struct
{
	float gyro_x;
	float gyro_y;
	float gyro_z;
} gyro_t;

typedef enum							/* Accelerometer: */ 					/*Gyroscope*/
{									/*Bandwidth(Hz)*//*Delay(ms)*/		/*Bandwidth(Hz)*//*Delay(ms)*/
	DLPF_CFG_0 = 0,						/*260*/			/*0*/				/*256*/			/*0.98*/
	DLPF_CFG_1,							/*184*/			/*2.0*/				/*188*/			/*1.90*/
	DLPF_CFG_2,							/*94*/			/*3.0*/				/*98*/			/*2.80*/
	DLPF_CFG_3,							/*44*/			/*4.9*/				/*42*/			/*4.80*/
	DLPF_CFG_4,							/*21*/			/*8.5*/				/*20*/			/*8.3*/
	DLPF_CFG_5,							/*10*/			/*13.8*/			/*10*/			/*13.4*/
	DLPF_CFG_6							/*5*/			/*19.0*/			/*5*/			/*18.6*/
} lpf_t;

typedef enum
{											/* Full Scale Range */
	GYRO_SCALE_250deg = 0,					/* +-250 deg/s */
	GYRO_SCALE_500deg,						/* +-500 deg/s */
	GYRO_SCALE_1000deg,						/* +-1000 deg/s */
	GYRO_SCALE_2000deg						/* +-2000 deg/s */
} gyro_scale_t;

typedef enum
{										/* Full Scale Range */
	ACCL_SCALE_2g = 0,					/* +-2g */
	ACCL_SCALE_4g,						/* +-4g */
	ACCL_SCALE_8g,						/* +-8g */
	ACCL_SCALE_16g						/* +-16g */
} accl_scale_t;

typedef enum
{
	INTERNAL_8MHZ = 0,
	PLL_GYRO_X,
	PLL_GYRO_Y,
	PLL_GYRO_Z,
	PLL_EXTERNAL_32KHZ,
	PLL_EXTERNAL_19MHZ
} clk_select_t;

typedef enum
{
	WAKE_FREQ_1_25HZ = 0,
	WAKE_FREQ_5HZ,
	WAKE_FREQ_20HZ,
	WAKE_FREQ_40HZ,
} wake_up_freq_t;


void mpu6050_process_raw_accl(uint8_t* accl_raw_data, accl_t* accl);
void mpu6050_process_raw_gyro(uint8_t* gyro_raw_data, gyro_t* gyro);
void mpu6050_init(I2C_HandleTypeDef* i2c_handle); //check int_pin_cfg register when using interrupt
void mpu6050_set_low_pass_filter(lpf_t lpf_value);
void mpu6050_set_sample_rate_divider(uint8_t sample_rate_divider_value);
void mpu6050_set_gyro_scale(gyro_scale_t gyro_scale_value);
void mpu6050_set_accl_scale(accl_scale_t accl_scale_value);
void mpu6050_set_motion_detect_threshold(uint8_t threshold);
void mpu6050_fifo_enable(uint8_t accl, uint8_t gyro); //fist usr_ctrl register, then next
void mpu6050_fifo_disable(void); //fist usr_ctrl register, then next
uint16_t mpu6050_get_fifo_count(void);
void mpu6050_read_fifo_data(uint8_t* buffer, uint16_t fifo_count); // to be used in data ready interrupt handler
void mpu6050_get_fifo_data(accl_t* accl, gyro_t* gyro); // not intended to be used in interrupt handler
void mpu6050_fifo_reset(void);
void mpu6050_intr_config(uint8_t intr_flags); // take mask of values of interrupts
uint8_t mpu6050_intr_read_status_register(void);
void mpu6050_intr_handler(void);
__weak void mpu6050_motion_detect_intr_handler(void);
__weak void mpu6050_fifo_overflow_intr_handler(void);
__weak void mpu6050_data_ready_intr_handler(void);
void mpu6050_reset(void);
void mpu6050_sleep(void);
void mpu6050_set_clock_source(clk_select_t clk_option);
void mpu6050_cycling_enable(wake_up_freq_t wake_up_freq);
void mpu6050_cycling_disable(void);
void mpu6050_standby_mode_control(uint8_t gyro_x, uint8_t gyro_y, uint8_t gyro_z, uint8_t accl_x, uint8_t accl_y, uint8_t accl_z);
void mpu6050_low_power_mode_enable(wake_up_freq_t wake_up_freq);
void mpu6050_low_power_mode_disable(void);
void mpu6050_get_accl(accl_t* accl);
void mpu6050_get_gyro(gyro_t* gyro);
float mpu6050_get_temp(void);

#endif /* INC_MPU6050_H_ */
