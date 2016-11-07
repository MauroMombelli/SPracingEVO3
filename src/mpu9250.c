#include <stdbool.h>
#include <stddef.h>

#include "stm32f30x.h"

#include "spi.h"
#include "Timer.h"

/*public interface*/
#include "gyro.h"
#include "acce.h"
#include "magn.h"
#include "temperature.h"

#define MPU_RA_WHO_AM_I 0x75

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define MPU6500_CS_GPIO                  GPIOB
#define MPU6500_CS_PIN                   GPIO_Pin_9
#define MPU6500_SPI_INSTANCE             SPI1
#define MPU6500_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOB

#define DISABLE_MPU6500       GPIO_SetBits(MPU6500_CS_GPIO,   MPU6500_CS_PIN)
#define ENABLE_MPU6500        GPIO_ResetBits(MPU6500_CS_GPIO, MPU6500_CS_PIN)

#define SPI_9MHZ_CLOCK_DIVIDER      4

bool mpu6500WriteRegister(uint8_t reg, uint8_t data);
bool mpu6500ReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
void mpu6500InitHw(void);
void mpu6500InitLogic(void);
//void mpuIntExtiInit(void);

bool mpu6500WriteRegister(uint8_t reg, uint8_t data) {
	ENABLE_MPU6500;
	spiTransferByte(MPU6500_SPI_INSTANCE, reg);
	spiTransferByte(MPU6500_SPI_INSTANCE, data);
	DISABLE_MPU6500;

	return true;
}

bool mpu6500ReadRegister(uint8_t reg, uint8_t length, uint8_t *data) {
	uint8_t risLen;
	ENABLE_MPU6500;
	spiTransferByte(MPU6500_SPI_INSTANCE, reg | 0x80); // read transaction
	spiTransfer(MPU6500_SPI_INSTANCE, data, &risLen, length);
	DISABLE_MPU6500;
	if (risLen < length)
		return false;
	return true;
}

void mpu6500InitHw(void) {

	RCC_AHBPeriphClockCmd(MPU6500_CS_GPIO_CLK_PERIPHERAL, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = MPU6500_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(MPU6500_CS_GPIO, &GPIO_InitStructure);

	GPIO_SetBits(MPU6500_CS_GPIO, MPU6500_CS_PIN);

	spiSetDivisor(MPU6500_SPI_INSTANCE, SPI_9MHZ_CLOCK_DIVIDER);

}

enum clock_sel_e {
	INV_CLK_INTERNAL = 0, INV_CLK_PLL, NUM_CLK
};

enum gyro_fsr_e {
	INV_FSR_250DPS = 0,
	INV_FSR_500DPS,
	INV_FSR_1000DPS,
	INV_FSR_2000DPS,
	NUM_GYRO_FSR
};

enum accel_fsr_e {
	INV_FSR_2G = 0, INV_FSR_4G, INV_FSR_8G, INV_FSR_16G, NUM_ACCEL_FSR
};

#define MPU6500_BIT_RESET                   (0x80)

void mpu6500InitLogic(void) {
	//mpuIntExtiInit(); //hw for interrupt

	ENABLE_MPU6500;

	mpu6500WriteRegister(MPU_RA_PWR_MGMT_1, MPU6500_BIT_RESET);
	timer_sleep(100);
	mpu6500WriteRegister(MPU_RA_SIGNAL_PATH_RESET, 0x07);
	timer_sleep(100);
	mpu6500WriteRegister(MPU_RA_PWR_MGMT_1, 0);
	timer_sleep(100);
	mpu6500WriteRegister(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3 | 1); //3 == use FsChose to set Bandwidth
	mpu6500WriteRegister(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
	mpu6500WriteRegister(MPU_RA_FF_THR, 0); //4000Hz ODR acce
	mpu6500WriteRegister(MPU_RA_CONFIG, 7); //7 == 3600Hz gyro
	mpu6500WriteRegister(MPU_RA_SMPLRT_DIV, 0); // set Divider to 0
	timer_sleep(15);

	// Data ready interrupt configuration
	mpu6500WriteRegister(MPU_RA_INT_PIN_CFG,
			0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1
					| 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN

	mpu6500WriteRegister(MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable

	DISABLE_MPU6500;
}

#define MPU9250_WHO_AM_I_CONST (0x71)

struct vector3f gyro, acce;
float temp = 0;

uint8_t init(void) {
	static bool hardwareInitialised = false;

	if (!hardwareInitialised) {
		SPI_Config();

		mpu6500InitHw();

		mpu6500InitLogic();

		hardwareInitialised = true;
	}

	uint8_t response_who;
	bool ack = mpu6500ReadRegister(MPU_RA_WHO_AM_I, 1, &response_who);
	if (!ack) {
		return 2;
	}

	return !(response_who == MPU9250_WHO_AM_I_CONST);
}

uint8_t temp_init(void) {
	return init();
}

uint8_t acce_init(void) {
	return init();
}

uint8_t gyro_init(void) {
	return init();
}

uint8_t update(const uint32_t time_us) {
	static uint32_t lastUpdate = 0;

	if (time_us - lastUpdate < 1000000) {
		return 1;
	}
	lastUpdate = time_us;



	uint8_t data[6];
	bool ack;
/*
	const uint8_t firstRegisterAcce = MPU_RA_ACCEL_XOUT_H; //0x3B to 0x40 is accel, 41 and 42 temp, 43 to 48 gyro
	ack = mpu6500ReadRegister(firstRegisterAcce, 6, data);

	if (!ack) {
		return 2;
	}

	acce.x = (int16_t) ((data[0] << 8) | data[1]);
	acce.y = (int16_t) ((data[2] << 8) | data[3]);
	acce.z = (int16_t) ((data[4] << 8) | data[5]);


	offset = 6;
	int16_t temp_tmp = (int16_t) ((data[0 + offset] << 8) | data[1 + offset]);
	temp = temp_tmp;
*/
	const uint8_t firstRegisterGyro = MPU_RA_GYRO_XOUT_H; //0x3B to 0x40 is accel, 41 and 42 temp, 43 to 48 gyro
	ack = mpu6500ReadRegister(firstRegisterGyro, 6, data);

	if (!ack) {
		return 3;
	}
	gyro.x = (int16_t) ((data[0] << 8) | data[1]);
	gyro.y = (int16_t) ((data[2] << 8) | data[3]);
	gyro.z = (int16_t) ((data[4] << 8) | data[5]);

	return 0;
}

uint8_t temp_update(const uint16_t time) {
	return update(time);
}

uint8_t acce_update(const uint16_t time) {
	return update(time);
}

uint8_t gyro_update(const uint32_t time_us) {
	return update(time_us);
}

uint8_t temp_get_data(float *ris) {
	*ris = temp;

	return true;
}

uint8_t acce_get_data(struct vector3f * const ris) {

	ris->x = acce.x;
	ris->y = acce.y;
	ris->z = acce.z;

	return 0;
}

uint8_t gyro_get_data(struct vector3f * const ris) {

	ris->x = gyro.x;
	ris->y = gyro.y;
	ris->z = gyro.z;

	return 0;
}
