/*
 * mpu6050.c
 *
 *  Created on: Jul 1, 2025
 *      Author: ishant
 */

#include "mpu6050.h"

static I2C_HandleTypeDef *mpu6050_i2c = NULL;
static float accl_divisor = 16384.0;
static float gyro_divisor = 131.0;
static const float temp_divisor = 340.0;
static const float temp_const = 36.53;

static void mpu6050_read_reg(uint8_t reg_addr, uint8_t* buffer, uint8_t len)
{
	HAL_I2C_Mem_Read(mpu6050_i2c, MPU6050_ADDR<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, buffer, len, HAL_MAX_DELAY);
}
static void mpu6050_write_reg(uint8_t reg_addr, uint8_t* data, uint8_t len)
{
	HAL_I2C_Mem_Write(mpu6050_i2c, MPU6050_ADDR<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}
void mpu6050_process_raw_accl(uint8_t* accl_raw_data, accl_t* accl)
{
	int16_t temp_acc_x = ((int16_t)accl_raw_data[0]<<8) | accl_raw_data[1];
	int16_t temp_acc_y = ((int16_t)accl_raw_data[2]<<8) | accl_raw_data[3];
	int16_t temp_acc_z = ((int16_t)accl_raw_data[4]<<8) | accl_raw_data[5];
	accl->acc_x = (float)temp_acc_x / accl_divisor;
	accl->acc_y = (float)temp_acc_y / accl_divisor;
	accl->acc_z = (float)temp_acc_z / accl_divisor;
}
void mpu6050_process_raw_gyro(uint8_t* gyro_raw_data, gyro_t* gyro)
{
	int16_t temp_gyro_x = ((int16_t)gyro_raw_data[0]<<8) | gyro_raw_data[1];
	int16_t temp_gyro_y = ((int16_t)gyro_raw_data[2]<<8) | gyro_raw_data[3];
	int16_t temp_gyro_z = ((int16_t)gyro_raw_data[4]<<8) | gyro_raw_data[5];
	gyro->gyro_x = (float)temp_gyro_x / gyro_divisor;
	gyro->gyro_y = (float)temp_gyro_y / gyro_divisor;
	gyro->gyro_z = (float)temp_gyro_z / gyro_divisor;
}
void mpu6050_init(I2C_HandleTypeDef* i2c_handle) //check int_pin_cfg register when using interrupt
{
	mpu6050_i2c = i2c_handle;
	uint8_t value = 0;
	mpu6050_write_reg(MPU6050_PWR_MGT_1, &value, 1);
	mpu6050_set_clock_source(PLL_GYRO_X);
	mpu6050_set_sample_rate_divider(7);
	mpu6050_set_low_pass_filter(DLPF_CFG_1);
	mpu6050_set_gyro_scale(GYRO_SCALE_250deg);
	mpu6050_set_accl_scale(ACCL_SCALE_2g);
}
void mpu6050_set_low_pass_filter(lpf_t lpf_value)
{
	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_CONFIG, &value, 1);
	value |= (uint8_t)lpf_value;
	mpu6050_write_reg(MPU6050_CONFIG, &value, 1);
}
void mpu6050_set_sample_rate_divider(uint8_t sample_rate_divider_value)
{
	mpu6050_write_reg(MPU6050_SMPLRT_DIV, &sample_rate_divider_value, 1);
}
void mpu6050_set_gyro_scale(gyro_scale_t gyro_scale_value)
{
	if(gyro_scale_value == GYRO_SCALE_250deg)
	{
		gyro_divisor = 131.0;
	}
	else if(gyro_scale_value == GYRO_SCALE_500deg)
	{
		gyro_divisor = 65.5;
	}
	else if(gyro_scale_value == GYRO_SCALE_1000deg)
	{
		gyro_divisor = 32.8;
	}
	else if(gyro_scale_value == GYRO_SCALE_2000deg)
	{
		gyro_divisor = 16.4;
	}
	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_GYRO_CONFIG, &value, 1);
	value &= ~(0x18); // clear FS_SEL[4:3] bits
	value |= (gyro_scale_value << 3); // set FS_SEL[4:3] bits
	mpu6050_write_reg(MPU6050_GYRO_CONFIG, &value, 1);
}
void mpu6050_set_accl_scale(accl_scale_t accl_scale_value)
{
	if(accl_scale_value == ACCL_SCALE_2g)
	{
		accl_divisor = 16384.0;
	}
	else if(accl_scale_value == ACCL_SCALE_4g)
	{
		accl_divisor = 8192.0;
	}
	else if(accl_scale_value == ACCL_SCALE_8g)
	{
		accl_divisor = 4096.0;
	}
	else if(accl_scale_value == ACCL_SCALE_16g)
	{
		accl_divisor = 2048.0;
	}
	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_ACCEL_CONFIG, &value, 1);
	value &= ~(0x18); // clear AFS_SEL[4:3] bits
	value |= (accl_scale_value << 3); // set FS_SEL[4:3] bits
	mpu6050_write_reg(MPU6050_ACCEL_CONFIG, &value, 1);
}
void mpu6050_set_motion_detect_threshold(uint8_t threshold)
{
	mpu6050_write_reg(MPU6050_MOT_THR, &threshold, 1);
}
void mpu6050_fifo_enable(uint8_t accl, uint8_t gyro) //fist usr_ctrl register, then next
{
	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_USER_CTRL, &value, 1);
	value |= (FIFO_EN_BIT | FIFO_RESET_BIT);
	mpu6050_write_reg(MPU6050_USER_CTRL, &value, 1);
	uint8_t mask = 0;
	mpu6050_read_reg(MPU6050_FIFO_EN, &mask, 1);
	if(accl != 0)
	{
		mask |= ACCL_FIFO_EN_BIT;
	}
	if(gyro != 0)
	{
		mask |= GYRO_FIFO_EN_BIT;
	}
	mpu6050_write_reg(MPU6050_FIFO_EN, &mask, 1);
}
void mpu6050_fifo_disable(void)
{
	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_USER_CTRL, &value, 1);
	value &= ~(FIFO_EN_BIT);
	mpu6050_write_reg(MPU6050_USER_CTRL, &value, 1);
	value = 0;
	mpu6050_write_reg(MPU6050_FIFO_EN, &value, 1);
}
uint16_t mpu6050_get_fifo_count(void)
{
	uint8_t buffer[2] = {0};
	mpu6050_read_reg(MPU6050_FIFO_COUNT, buffer, 2);
	uint16_t fifo_count = ((uint16_t)buffer[0] << 8) | buffer[1];
	return fifo_count;
}
void mpu6050_read_fifo_data(uint8_t* buffer, uint16_t fifo_count)
{
	mpu6050_read_reg(MPU6050_FIFO_R_W, buffer, fifo_count);
}
void mpu6050_get_fifo_data(accl_t* accl, gyro_t* gyro) // first check if fifo for accl and gyro is enabled or not and fifo count //1024 byte fifo register, write code accordingly
{
	uint8_t user_ctrl_register = 0;
	mpu6050_read_reg(MPU6050_USER_CTRL, &user_ctrl_register, 1);
	if(user_ctrl_register && FIFO_EN_BIT == 0)
	{
		return;
	}
	uint8_t fifo_en_register = 0;
	uint8_t accl_fifo_enabled = 0;
	uint8_t gyro_fifo_enabled = 0;
	mpu6050_read_reg(MPU6050_FIFO_EN, &fifo_en_register, 1);
	uint16_t fifo_count = mpu6050_get_fifo_count();
	if((fifo_en_register & ACCL_FIFO_EN_BIT) == ACCL_FIFO_EN_BIT)
	{
		accl_fifo_enabled = 1;
	}
	if((fifo_en_register & GYRO_FIFO_EN_BIT) == GYRO_FIFO_EN_BIT)
	{
		gyro_fifo_enabled = 1;
	}
	if(accl_fifo_enabled && gyro_fifo_enabled)
	{
		if(fifo_count < FIFO_BOTH_PACKET_SIZE)
		{
			return;
		}
		uint8_t buffer[FIFO_ACCL_PACKET_SIZE] = {0};
		mpu6050_read_reg(MPU6050_FIFO_R_W, buffer, 6);
		mpu6050_process_raw_accl(buffer, accl);
		mpu6050_read_reg(MPU6050_FIFO_R_W, buffer, 6);
		mpu6050_process_raw_gyro(buffer, gyro);
	}
	else if(accl_fifo_enabled)
	{
		if(fifo_count < FIFO_ACCL_PACKET_SIZE)
		{
			return;
		}
		uint8_t buffer[FIFO_ACCL_PACKET_SIZE] = {0};
		mpu6050_read_reg(MPU6050_FIFO_R_W, buffer, 6);
		mpu6050_process_raw_accl(buffer, accl);
	}
	else if(gyro_fifo_enabled)
	{
		if(fifo_count < FIFO_GYRO_PACKET_SIZE)
		{
			return;
		}
		uint8_t buffer[FIFO_GYRO_PACKET_SIZE] = {0};
		mpu6050_read_reg(MPU6050_FIFO_R_W, buffer, 6);
		mpu6050_process_raw_gyro(buffer, gyro);
	}
}
void mpu6050_fifo_reset(void)
{
	mpu6050_fifo_disable();
	uint8_t value = FIFO_RESET_BIT;
	mpu6050_write_reg(MPU6050_USER_CTRL, &value, 1);
}
void mpu6050_intr_config(uint8_t intr_flags) // take mask of values of interrupts
{

	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_INT_PIN_CFG, &value, 1);
	value |= (INT_LEVEL_BIT | LATCH_INT_EN); 		// active low | held high until intr cleared
	value &= ~(INT_OPEN_BIT | INT_RD_CLEAR);	// push pull | intr cleared on status reg read
	mpu6050_write_reg(MPU6050_INT_PIN_CFG, &value, 1);
	//enable interrupts
	mpu6050_read_reg(MPU6050_INT_ENABLE, &value, 1);
	value |= intr_flags;
	mpu6050_write_reg(MPU6050_INT_ENABLE, &value, 1);
}
void mpu6050_intr_handler(void)
{
	uint8_t reg = 0;
	mpu6050_read_reg(MPU6050_INT_STATUS, &reg, 1);
	if(reg & MPU6050_INTR_DATA_READY)
	{
		mpu6050_data_ready_intr_handler();
	}
	if(reg & MPU6050_INTR_FIFO_OVERFLOW)
	{
		mpu6050_fifo_overflow_intr_handler();
	}
	if(reg & MPU6050_INTR_MOTION_DETECT)
	{
		mpu6050_motion_detect_intr_handler();
	}
}
uint8_t mpu6050_intr_read_status_register(void)
{
	uint8_t temp = 0;
	mpu6050_read_reg(MPU6050_INT_STATUS, &temp, 1);
	return temp;
}
void mpu6050_reset(void)
{
	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_PWR_MGT_1, &value, 1);
	value |= DEVICE_RESET;
	mpu6050_write_reg(MPU6050_PWR_MGT_1, &value, 1);
}
void mpu6050_sleep(void)
{
	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_PWR_MGT_1, &value, 1);
	value |= SLEEP;
	mpu6050_write_reg(MPU6050_PWR_MGT_1, &value, 1);
}
void mpu6050_set_clock_source(clk_select_t clk_option)
{
	uint8_t value = 0;
	mpu6050_write_reg(MPU6050_PWR_MGT_1, &value, 1);
	value |= (uint8_t) clk_option;
	mpu6050_write_reg(MPU6050_PWR_MGT_1, &value, 1);
}
// FIFO doesn't work if cycling is enabled
void mpu6050_cycling_enable(wake_up_freq_t wake_up_freq)
{
	uint8_t value = ((uint8_t)wake_up_freq)<<6;
	mpu6050_write_reg(MPU6050_PWR_MGT_2, &value, 1);
	value = CYCLE;
	mpu6050_write_reg(MPU6050_PWR_MGT_1, &value, 1);
}
void mpu6050_cycling_disable(void)
{
	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_PWR_MGT_1, &value, 1);
	value &= ~CYCLE;
	mpu6050_write_reg(MPU6050_PWR_MGT_1, &value, 1);
}
// 1 == put on standby, 0 == don't put on standby
void mpu6050_standby_mode_control(uint8_t gyro_x, uint8_t gyro_y, uint8_t gyro_z, uint8_t accl_x, uint8_t accl_y, uint8_t accl_z)
{
	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_PWR_MGT_2, &value, 1);
	if(gyro_x != 0)
	{
		value |= STANDBY_GYRO_X;
	}
	if(gyro_y != 0)
	{
		value |= STANDBY_GYRO_Y;
	}
	if(gyro_z != 0)
	{
		value |= STANDBY_GYRO_Z;
	}
	if(accl_x != 0)
	{
		value |= STANDBY_ACCL_X;
	}
	if(accl_y != 0)
	{
		value |= STANDBY_ACCL_Y;
	}
	if(accl_z != 0)
	{
		value |= STANDBY_ACCL_Z;
	}
	mpu6050_write_reg(MPU6050_PWR_MGT_2, &value, 1);
}
void mpu6050_low_power_mode_enable(wake_up_freq_t wake_up_freq)
{
	mpu6050_cycling_enable(wake_up_freq);
	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_PWR_MGT_1, &value, 1);
	value |= TEMP_DIS;
	value &= ~SLEEP;
	mpu6050_write_reg(MPU6050_PWR_MGT_1, &value, 1);
	mpu6050_standby_mode_control(1, 1, 1, 0, 0, 0);
}
void mpu6050_low_power_mode_disable(void)
{
	mpu6050_cycling_disable();
	uint8_t value = 0;
	mpu6050_read_reg(MPU6050_PWR_MGT_1, &value, 1);
	value &= ~TEMP_DIS;
	value |= SLEEP;
	mpu6050_write_reg(MPU6050_PWR_MGT_1, &value, 1);
	mpu6050_standby_mode_control(0, 0, 0, 0, 0, 0);
}
void mpu6050_get_accl(accl_t* accl)
{
	uint8_t buffer[6] = {0};
	mpu6050_read_reg(MPU6050_ACCEL, buffer, 6);
	mpu6050_process_raw_accl(buffer, accl);
}
void mpu6050_get_gyro(gyro_t* gyro)
{
	uint8_t buffer[6] = {0};
	mpu6050_read_reg(MPU6050_GYRO, buffer, 6);
	mpu6050_process_raw_gyro(buffer, gyro);
}
float mpu6050_get_temp(void)
{
	uint8_t buffer[2] = {0};
	mpu6050_read_reg(MPU6050_TEMP, buffer, 2);
	int16_t temp = ((int16_t) buffer[0]<<8) | buffer[1];
	float temprature = ((float)temp/temp_divisor) + temp_const;
	return temprature;
}
