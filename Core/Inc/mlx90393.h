/*
 * mlx90393.h
 *
 *  Created on: 31 maj 2021
 *      Author: mzuru
 */

#ifndef INC_MLX90393_H_
#define INC_MLX90393_H_

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdbool.h"

uint8_t buffer[3];
uint8_t mem_map_buffer;
float multiplier_x;
float multiplier_y;
float multiplier_z;

struct MLX_data{
	int16_t X_axis;			// value from X axis
	int16_t Y_axis;			// value from Y axis
	int16_t Z_axis;			// value from Z axis
	int16_t temperature;	// temperature
}mlx_data;

struct calculated_data{
	float x;
	float y;
	float z;
}calc_data;

typedef struct calibration_settings{
	uint8_t gain;
	uint8_t osr;
	uint8_t dig_filter;
	uint8_t res_x;
	uint8_t res_y;
	uint8_t res_z;
	uint16_t offset_x;
	uint16_t offset_y;
	uint16_t offset_z;
	uint16_t field_intensity;
}calib_set;

calib_set gui_settings;
calib_set slave_settings;

#define OK 		true
#define NOT_OK	false
typedef bool RetStatus;

enum configuration_states
{
	WAIT_FOR_GUI,
	READ_VM_MEMORY,
	WRITE_NVM_MEMORY,
	CALIBRATION,
//	MEASURE,
}conf_state;



#define uT2Oe						0.01		// calculate uT to Oersted
#define MLX90393_ADDR 				(0x0C << 1)	// sensor address
#define TEMP_MASK					0x01		// mask to OR with RM (the mask | RM) command to read temperature
#define X_AXIS_MASK					0x02		// mask to OR with RM (the mask | RM) command to read data from X axis
#define Y_AXIS_MASK					0x04		// mask to OR with RM (the mask | RM) command to read data from Y axis
#define Z_AXIS_MASK					0x08		// mask to OR with RM (the mask | RM) command to read data from Z axis
#define XYZ_AXES_MASK				0x0E		// mask to OR with RM (the mask | RM) command to read data from X, Y, Z axes
#define NUMBER_OF_SAMPLES			50			// number of samples measured in order to calibrate offsets

/* MLX90393 COMMANDS */
#define CMD_NOP 					0x00		// do nothing
#define CMD_START_BURST 			0x10		// Start Burst Mode
#define CMD_START_WAKE_ON_CHANGE 	0x20		// Start Wake-up on Change Mode
#define CMD_START_SINGLE 			0x30		// Start Single Measurement Mode
#define CMD_READ_MEASUREMENT 		0x40		// Read Measurement
#define CMD_READ_REGISTER 			0x50		// Read Register
#define CMD_WRITE_REGISTER 			0x60		// Write Register
#define CMD_EXIT 					0x80		// Exit Mode
#define CMD_MEM_RECALL 				0xD0		// Memory Recall
#define CMD_MEM_STORE 				0xE0		// Memory Store
#define CMD_RST 					0xF0		// Reset

/* MLX90393 MEMORY MAP */

/*	REGISTER || BIT15 | BIT14 | BIT13 | BIT12 | BIT11 | BIT10 | BIT9 | BIT8 | BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0 |
 * 	_________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 * 	__0x00h__||________________ANA_RESERVED_LOW______________________|_BIST_|Z-SER_|______GAIN_SEL______|_________HALL_CONF_________|
 * 	_________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 * 	__0x01h__||TRG_INT|__COMMON_MODE__|WOC_DIF|EXT_TRG|TCMP_EN|_____BURST_SEL(xyzt)_______|_________BURST_DATA_RATE(BDR)____________|
 * 	_________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x02h__||_______|_______|_______|______OSR2_____|_____RES_Z____|_____RES_Y___|_____RES_X___|_____DIG_FILT_______|_____OSR_____|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x03h__||_________________________SENS_TC_HT__________________________|______________________SENS_TC_LT_______________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x04h__||_____________________________________________________OFFSET_X________________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x05h__||_____________________________________________________OFFSET_Y________________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x06h__||_____________________________________________________OFFSET_Z________________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x07h__||___________________________________________________WOXY_THRESHOLD____________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x08h__||____________________________________________________WOZ_THRESHOLD____________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x09h__||____________________________________________________WOT_THRESHOLD____________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x0Ah__||                                                                                                                     |
 *     ...   ||                                                         FREE                                                        |
 *  __0x1Fh__||_____________________________________________________________________________________________________________________|
 */


#define HALLCONF_MASK				0x000F
#define HALLCONF_START_BIT			0
#define GAIN_SEL_MASK				0x0070
#define GAIN_SEL_START_BIT			4
#define TCMP_EN_MASK				0x0400
#define TCMP_EN_START_BIT			10
#define COMM_MODE_MASK				0x6000
#define COMM_MODE_START_BIT			13
#define OSR_MASK					0x0003
#define OSR_START_BIT				0
#define DIG_FILT_MASK				0x001C
#define DIG_FILT_START_BIT			2
#define RES_X_MASK					0x0060
#define RES_X_START_BIT				5
#define RES_Y_MASK					0x0180
#define RES_Y_START_BIT				7
#define RES_Z_MASK					0x0600
#define RES_Z_START_BIT				9
#define OSR2_MASK					0x1800
#define OSR2_START_BIT				11
#define SENS_TC_LT_MASK				0xFF
#define SENS_TC_LT_START_BIT		0
#define SENS_TC_HT_MASK				0xFF
#define SENS_TC_HT_START_BIT		8
#define OFFSET_X					0x04			// 16 bit long
#define OFFSET_Y					0x05			// 16 bit long
#define OFFSET_Z					0x06			// 16 bit long
#define WOXY_THRESHOLD				0x07			// 16 bit long
#define WOZ_THRESHOLD				0x08			// 16 bit long
#define WOT_THRESHOLD				0x09			// 16 bit long
#define REG_00						0x00
#define REG_01						0x01
#define REG_02						0x02
#define REG_03						0x03
#define REG_04						0x04
#define REG_05						0x05
#define REG_06						0x06
#define REG_07						0x07
#define REG_08						0x08
#define REG_09						0x09

#define ERROR_BIT_MASK				0x10
// addresses from 0x0A to 0x1F are free for customer content

uint16_t read_register(uint8_t reg);					// read whole register
void read_measure_X(struct MLX_data * data_struct);		// read X axis measure
void read_measure_Y(struct MLX_data * data_struct);		// read Y axis measure
void read_measure_Z(struct MLX_data * data_struct);		// read Z axis measure
//void read_measure_XYZ(struct MLX_data * data_struct);	// read XYZ axis measure
void read_measure_T(struct MLX_data * data_struct);		// read temperature
void set_single_measurement(void);						// set single measurement mode
uint8_t get_resolution_X(void);							// get resolution of X axis
uint8_t get_resolution_Y(void);							// get resolution of Y axis
uint8_t get_resolution_Z(void);							// get resolution of Z axis
void set_resolution_X(uint8_t res);						// set resolution of X axis
void set_resolution_Y(uint8_t res);						// set resolution of Y axis
void set_resolution_Z(uint8_t res);						// set resolution of Z axis
uint8_t get_gain(void);									// get gain
void set_gain(uint8_t gain);							// set gain	
uint8_t get_filter(void);								// get digital filter setting
void set_filter(uint8_t conf);
uint8_t get_osr(void);
void set_osr(uint8_t osr); 								// Magnetic sensor ADC oversampling ratio
uint8_t get_osr2(void);
void set_osr2(uint8_t osr2); 							// Temperature sensor ADC oversampling ratio
uint16_t get_offset_X(void);
void set_offset_X(uint16_t offset_X); 					// Constant offset correction, independent for X
uint16_t get_offset_Y(void);
void set_offset_Y(uint16_t offset_Y); 					// Constant offset correction, independent for Y
uint16_t get_offset_Z(void);
void set_offset_Z(uint16_t offset_Z); 					// Constant offset correction, independent for Z
void set_offsets(uint16_t field_intensity);
RetStatus write_non_volatile_mem(void); 						// store volatile memory in non-volatile memory with HS command
void read_non_volatile_mem(void);						// read non-volatile memory with HR command
void set_multipliers(float* x, float* y, float* z);
void store_data_from_gui_to_struct(uint8_t* data, calib_set* data_struct);

#endif /* INC_MLX90393_H_ */
