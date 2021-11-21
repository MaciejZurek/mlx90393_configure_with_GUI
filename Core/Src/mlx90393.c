/*
 * mlx90393.c
 *
 *  Created on: 1 cze 2021
 *      Author: mzuru
 */
#include "mlx90393.h"

/** Lookup table to convert raw values to uT based on [HALLCONF][GAIN_SEL][RES].
 */
static const float mlx90393_lsb_lookup[2][8][4][2] = {

    /* HALLCONF = 0xC (default) */
    {
    // |    RES = 0   |  |   RES = 1  |  |   RES = 2  |  |   RES = 3  |
    // |SENSxy |SENSz |  |SENSxy|SENSz|  |SENSxy|SENSz|  |SENSxy|SENSz|
        /* GAIN_SEL = 0, 5x gain */
        {{0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}, {6.009, 9.680}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}, {4.840, 7.744}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.451, 0.726}, {0.901, 1.452}, {1.803, 2.904}, {3.605, 5.808}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.376, 0.605}, {0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.250, 0.403}, {0.501, 0.807}, {1.001, 1.613}, {2.003, 3.227}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.200, 0.323}, {0.401, 0.645}, {0.801, 1.291}, {1.602, 2.581}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.150, 0.242}, {0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}},
    },

    /* HALLCONF = 0x0 */
    {
    // |    RES = 0   |  |   RES = 1  |  |   RES = 2  |  |   RES = 3  |
    // |SENSxy |SENSz |  |SENSxy|SENSz|  |SENSxy|SENSz|  |SENSxy|SENSz|
        /* GAIN_SEL = 0, 5x gain */
        {{0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}, {6.292, 10.137}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}, {5.034, 8.109}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.472, 0.760}, {0.944, 1.521}, {1.888, 3.041}, {3.775, 6.082}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.393, 0.634}, {0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.262, 0.422}, {0.524, 0.845}, {1.049, 1.689}, {2.097, 3.379}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.210, 0.338}, {0.419, 0.676}, {0.839, 1.352}, {1.678, 2.703}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.157, 0.253}, {0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}},
    }};

/** Lookup table for conversion time based on [DIF_FILT][OSR].
 */
static const float mlx90393_tconv[8][4] = {
  //|        OSR          |
  //| 0  || 1 | | 2 | | 3 |
    /* DIG_FILT = 0 */
    {1.27, 1.84, 3.00, 5.30},
    /* DIG_FILT = 1 */
    {1.46, 2.23, 3.76, 6.84},
    /* DIG_FILT = 2 */
    {1.84, 3.00, 5.30, 9.91},
    /* DIG_FILT = 3 */
    {2.61, 4.53, 8.37, 16.05},
    /* DIG_FILT = 4 */
    {4.15, 7.60, 14.52, 28.34},
    /* DIG_FILT = 5 */
    {7.22, 13.75, 26.80, 52.92},
    /* DIG_FILT = 6 */
    {13.36, 26.04, 51.38, 102.07},
    /* DIF_FILT = 7 */
    {25.65, 50.61, 100.53, 200.37},
};


void read_measure_X(struct MLX_data * data_struct)		// read X axis measure
{
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, CMD_START_SINGLE | X_AXIS_MASK, 1, buffer, 1, HAL_MAX_DELAY);	// single measurement mode
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, CMD_READ_MEASUREMENT | X_AXIS_MASK, 1, buffer, 3, HAL_MAX_DELAY);
	data_struct -> X_axis = buffer[2] | (buffer[1] << 8);	// store second and third byte from slave (first is only status byte)
	//HAL_I2C_Init(&hi2c1);
}

void read_measure_Y(struct MLX_data * data_struct)		// read Y axis measure
{
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, CMD_START_SINGLE | Y_AXIS_MASK, 1, buffer, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, CMD_READ_MEASUREMENT | Y_AXIS_MASK, 1, buffer, 3, HAL_MAX_DELAY);
	data_struct -> Y_axis = buffer[2] | (buffer[1] << 8);	// store second and third byte from slave (first is only status byte)
}

void read_measure_Z(struct MLX_data * data_struct)		// read Z axis measure
{
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, CMD_START_SINGLE | Z_AXIS_MASK, 1, buffer, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, CMD_READ_MEASUREMENT | Z_AXIS_MASK, 1, buffer, 3, HAL_MAX_DELAY);
	data_struct -> Z_axis = buffer[2] | (buffer[1] << 8);	// store second and third byte from slave (first is only status byte)
}


//void read_measure_T(struct MLX_data * data_struct)		// read temperature
//{
//	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, CMD_READ_MEASUREMENT | TEMP_MASK, 1, buffer, sizeof(buffer), HAL_MAX_DELAY);
//	data_struct -> temperature = buffer[2] | (buffer[1] << 8);	// store second and third byte from slave (first is only status byte)
//}

void set_single_measurement(void)
{
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, CMD_START_SINGLE | X_AXIS_MASK, 1, buffer, 1, HAL_MAX_DELAY);
}

uint8_t get_resolution_X(void)					// get resolution of X axis
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY); // read whole register
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);

	value = buffer[2] | (buffer[1] << 8);	// write read value to buffer; buffer[0] - status byte

	return (value & RES_X_MASK) >> RES_X_START_BIT; // extract resolution X from the register
}


void set_resolution_X(uint8_t res)					// set resolution of X axis
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_02 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	register_value &= ~RES_X_MASK;
	register_value |= res << RES_X_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(res != get_resolution_X())	// keep trying to set filter to conf value if filter value is not equal to conf
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint8_t get_resolution_Y(void)					// get resolution of Y axis
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);

	return (value & RES_Y_MASK) >> RES_Y_START_BIT;
}

void set_resolution_Y(uint8_t res)					// set resolution of Y axis
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_02 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	register_value &= ~RES_Y_MASK;
	register_value |= res << RES_Y_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(res != get_resolution_Y())	// keep trying to set filter to conf value if filter value is not equal to conf
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint8_t get_resolution_Z(void)					// get resolution of Z axis
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);

	return (value & RES_Z_MASK) >> RES_Z_START_BIT;
}

void set_resolution_Z(uint8_t res)					// set resolution of Z axis
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_02 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	register_value &= ~RES_Z_MASK;
	register_value |= res << RES_Z_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(res != get_resolution_Z())	// keep trying to set filter to conf value if filter value is not equal to conf
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint8_t get_gain(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_00 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_00 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_00 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_00 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);

	value = buffer[2] | (buffer[1] << 8);

	return (value & GAIN_SEL_MASK) >> GAIN_SEL_START_BIT;
}

void set_gain(uint8_t gain)
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_00 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_00 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	register_value &= ~GAIN_SEL_MASK;
	register_value |= gain << GAIN_SEL_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(gain != get_gain())
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint16_t read_register(uint8_t reg)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (reg << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);
	return value;
}


uint8_t get_filter(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);

	return (value & DIG_FILT_MASK) >> DIG_FILT_START_BIT;
}

void set_filter(uint8_t conf) // TODO: test
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_02 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	// firstly, read the whole register and store its value in the register_value variable
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	// clear interesting bits and set them with conf value
	register_value &= ~DIG_FILT_MASK;
	register_value |= conf << DIG_FILT_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(conf != get_filter())	// keep trying to set filter to conf value if filter value is not equal to conf
	{
		// then, write data from master to the whole register
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}

}

uint8_t get_hallconf(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_00 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);

	return (value & HALLCONF_MASK) >> HALLCONF_START_BIT;
}

void set_hallconf(uint8_t hall_conf) // TODO: test
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_00 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_00 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	register_value &= ~HALLCONF_MASK;
	register_value |= hall_conf << HALLCONF_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(hall_conf != get_hallconf())
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}

}

uint8_t get_tcmp_en(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_01 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);

	return (value & TCMP_EN_MASK) >> TCMP_EN_START_BIT;
}

void set_tcmp_en(uint8_t tcmp_en) // TODO: test
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_01 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_01 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	register_value &= ~TCMP_EN_MASK;
	register_value |= tcmp_en << TCMP_EN_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(tcmp_en != get_tcmp_en())
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint8_t get_comm_mode(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_01 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);

	return (value & COMM_MODE_MASK) >> COMM_MODE_START_BIT;
}

void set_comm_mode(uint8_t comm_mode)	// TODO: test
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_01 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_01 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	register_value &= ~COMM_MODE_MASK;
	register_value |= comm_mode << COMM_MODE_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(comm_mode != get_comm_mode())
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint8_t get_osr(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);

	value = buffer[2] | (buffer[1] << 8);

	return (value & OSR_MASK) >> OSR_START_BIT;
}

void set_osr(uint8_t osr)	// TODO: test
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_02 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	register_value &= ~OSR_MASK;
	register_value |= osr << OSR_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(osr != get_osr())
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint8_t get_osr2(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);

	return (value & OSR2_MASK) >> OSR2_START_BIT;
}

void set_osr2(uint8_t osr2)	// TODO: test
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_02 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_02 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	register_value &= ~OSR2_MASK;
	register_value |= osr2 << OSR2_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(osr2 != get_osr2())
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint8_t get_sens_tc_lt(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_03 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);

	return (value & SENS_TC_LT_MASK) >> SENS_TC_LT_START_BIT;
}

void set_sens_tc_lt(uint8_t sens_tc_lt)	// TODO: test
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_03 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_03 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	register_value &= ~SENS_TC_LT_MASK;
	register_value |= sens_tc_lt << SENS_TC_LT_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(sens_tc_lt != get_sens_tc_lt())
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint8_t get_sens_tc_ht(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_03 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);

	return (value & SENS_TC_HT_MASK) >> SENS_TC_HT_START_BIT;
}

void set_sens_tc_ht(uint8_t sens_tc_ht)	// TODO
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_03 << 2;
	uint8_t buffer[3];
	uint16_t register_value;

	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_03 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	register_value = buffer[2] | (buffer[1] << 8);

	register_value &= ~SENS_TC_HT_MASK;
	register_value |= sens_tc_ht << SENS_TC_HT_START_BIT;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(sens_tc_ht != get_sens_tc_ht())
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint16_t get_offset_X(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_04 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_04 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_04 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);

	value = buffer[2] | (buffer[1] << 8);

	return value;
}

void set_offset_X(uint16_t offset_X)	// TODO: test
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_04 << 2;
	uint16_t register_value;

	register_value = offset_X;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(offset_X != get_offset_X())
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint16_t get_offset_Y(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_05 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_05 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);

	return value;
}

void set_offset_Y(uint16_t offset_Y)	// TODO: test
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_05 << 2;
	uint16_t register_value;

	register_value = offset_Y;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(offset_Y != get_offset_Y())
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

uint16_t get_offset_Z(void)
{
	uint8_t buffer[3];
	uint16_t value;
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_06 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, (CMD_READ_REGISTER << 8) | (REG_06 << 2), 2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	value = buffer[2] | (buffer[1] << 8);

	return value;
}

void set_offset_Z(uint16_t offset_Z)	// TODO
{
	uint8_t data_from_master[4];
	data_from_master[0] = CMD_WRITE_REGISTER;
	data_from_master[3] = REG_06 << 2;
	uint16_t register_value;

	register_value = offset_Z;
	data_from_master[1] = (uint8_t)(register_value >> 8);
	data_from_master[2] = (uint8_t)register_value;

	while(offset_Z != get_offset_Z())
	{
		HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, data_from_master, 4, HAL_MAX_DELAY);
	}
}

void set_offsets(uint16_t field_intensity)
{
	// X and Z axis should be 0, vector of the field parallel to Y axis

	uint8_t i;
	uint32_t sum_x;
	uint32_t sum_y;
	uint32_t sum_z;
	uint32_t avg_x;
	uint32_t avg_y;
	uint32_t avg_z;
	int16_t off_x_to_set;
	int16_t off_y_to_set;
	int16_t off_z_to_set;

	for(i = 0; i < NUMBER_OF_SAMPLES; i++)
	{
		read_measure_X(&mlx_data);
		read_measure_Y(&mlx_data);
		read_measure_Z(&mlx_data);
		sum_x += mlx_data.X_axis;
		sum_y += mlx_data.Y_axis;
		sum_z += mlx_data.Z_axis;
	}

	avg_x = sum_x / NUMBER_OF_SAMPLES;
	avg_y = sum_y / NUMBER_OF_SAMPLES;
	avg_z = sum_z / NUMBER_OF_SAMPLES;

	off_x_to_set = 0 - (uint16_t)avg_x;
	off_y_to_set = field_intensity - (uint16_t)avg_y;
	off_z_to_set = 0 - (uint16_t)avg_z;

	set_offset_X(off_x_to_set);
	set_offset_Y(off_y_to_set);
	set_offset_Z(off_z_to_set);

}


void set_multipliers(float* mult_x, float* mult_y, float* mult_z)
{
	uint8_t gain;
	uint8_t res_x;
	uint8_t res_y;
	uint8_t res_z;

	gain = get_gain();
	res_x = get_resolution_X();
	res_y = get_resolution_Y();
	res_z = get_resolution_Z();

	*mult_x = mlx90393_lsb_lookup[0][gain][res_x][0];
	*mult_y = mlx90393_lsb_lookup[0][gain][res_y][0];
	*mult_z = mlx90393_lsb_lookup[0][gain][res_z][1];
}

void store_data_from_gui_to_struct(uint8_t* data, calib_set* data_struct)
{
	data_struct -> gain = data[1];
	data_struct -> res_x = data[2];
	data_struct -> res_y = data[3];
	data_struct -> res_z = data[4];
	data_struct -> dig_filter = data[5];
	data_struct -> osr = data[6];
}

RetStatus write_non_volatile_mem(void)
{
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, CMD_MEM_STORE, 1, buffer, 1, HAL_MAX_DELAY);
	HAL_Delay(20);	// After the HS command, wait at least 15ms before sending the next command to allow the IC to update the NVRAM correctly.

	if(buffer[0] & ERROR_BIT_MASK)	// if there was an error while proceeding NVM writing, return NOT_OK status
	{
		return NOT_OK;
	}
	else	// or OK if no error occured
	{
		return OK;
	}
}

void read_non_volatile_mem(void)
{
	HAL_I2C_Mem_Read(&hi2c1, MLX90393_ADDR, CMD_MEM_RECALL, 1, buffer, 1, HAL_MAX_DELAY);
}
