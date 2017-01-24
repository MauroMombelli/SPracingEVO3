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

/* debug with blink xD */
#include "BlinkLed.h"
#include "serial/usart.h"

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
uint8_t mpu6500ReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
void mpu6500InitHw(void);
void mpu6500InitLogic(void);
void mpuIntExtiInit(void);

volatile int data_is_ready = 0;
volatile int data_read = 0;

/* Handle PC13 interrupt */
void EXTI15_10_IRQHandler(void) {
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
		data_is_ready = 1;

		data_read++;

		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line13);
	}

}

void mpuIntExtiInit(void) {

	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	USART1_Write("T1\n", 3);

	GPIO_InitTypeDef GPIO_InitStructure;
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART1_Write("T2\n", 3);

	const uint32_t line = EXTI_Line13;
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

	USART1_Write("T3\n", 3);

	EXTI_InitTypeDef EXTIInit;
	EXTIInit.EXTI_Line = line;
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTIInit);

	USART1_Write("T4\n", 3);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART1_Write("T5\n", 3);
}

bool mpu6500WriteRegister(uint8_t reg, uint8_t data) {
	ENABLE_MPU6500;
	spiTransferByte(MPU6500_SPI_INSTANCE, &reg);
	spiTransferByte(MPU6500_SPI_INSTANCE, &data);
	DISABLE_MPU6500;

	return true;
}

uint8_t mpu6500ReadRegister(uint8_t reg, uint8_t length, uint8_t *data) {
	uint8_t readReg = reg | 0x80;
	ENABLE_MPU6500;
	uint8_t status = spiTransferByte(MPU6500_SPI_INSTANCE, &readReg); // read transaction
	if (status) {
		return status;
	}
	spiTransfer(MPU6500_SPI_INSTANCE, data, 0, length);
	if (status) {
		return status;
	}
	DISABLE_MPU6500;
	return 0;
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

	spiSetDivisor(MPU6500_SPI_INSTANCE, 2);

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


	mpu6500WriteRegister(MPU_RA_PWR_MGMT_1, MPU6500_BIT_RESET);

	/*
	while (i2cRead(MPU6500_ADDRESS, MPU6500_PWR_MGMT_1) & (1 << 7)) {
	        // Wait for the bit to clear
	};
	*/
	timer_sleep(100);

	mpu6500WriteRegister(MPU_RA_SIGNAL_PATH_RESET, 0x07);
	mpu6500WriteRegister(MPU_RA_USER_CTRL, 0x01);

	timer_sleep(100);
	mpu6500WriteRegister(MPU_RA_PWR_MGMT_1, 0x0); //0x10 == gyro disabled
	timer_sleep(100);
	/*
	mpu6500WriteRegister(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
	*/
	mpu6500WriteRegister(MPU_RA_CONFIG, 7 ); //7 == 8000Hz gyro, * == 1kHz. SPECIAL CASE: MPU_RA_GYRO_CONFIG == 1, 2, or 3 == 32kHz
	mpu6500WriteRegister(MPU_RA_GYRO_CONFIG, (INV_FSR_500DPS << 3)); //3 == use FsChose to set Bandwidth
	mpu6500WriteRegister(MPU_RA_FF_THR, 8); //8 == 4kHz

	//mpu6500WriteRegister(MPU_RA_SMPLRT_DIV, 0); // set Divider to 0

	//mpu6500WriteRegister(MPU_RA_FIFO_EN, 0x78); //enable FIFO for gyro and acce XYZ

	timer_sleep(15);

// Data ready interrupt configuration

	mpu6500WriteRegister(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0); // INT_ANYRD_2CLEAR, BYPASS_EN

	mpu6500WriteRegister(MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
}

#define MPU9250_WHO_AM_I_CONST (0x71)

struct vector3f gyro, acce;
float temp = 0;

uint8_t init(void) {
	static bool hardwareInitialised = false;

	mpuIntExtiInit();

	if (!hardwareInitialised) {
		SPI_Config();

		mpu6500InitHw();

		mpu6500InitLogic();

		hardwareInitialised = true;
	}

	uint8_t response_who;
	uint8_t ack = mpu6500ReadRegister(MPU_RA_WHO_AM_I, 1, &response_who);
	if (ack == 0) {
		return !(response_who == MPU9250_WHO_AM_I_CONST);
	}

	return (uint8_t) (ack + 1);
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
	(void) time_us;
	/*
	 static uint32_t lastUpdate = 0;

	 if (time_us - lastUpdate < (1000000/4000) ) {
	 return 1;
	 }
	 lastUpdate = time_us;
	 */
	if (!data_is_ready) {
		return 1;
	}

	uint8_t data[6];
	uint8_t ack;

	const uint8_t firstRegisterAcce = MPU_RA_ACCEL_XOUT_H; //0x3B to 0x40 is accel, 41 and 42 temp, 43 to 48 gyro
	ack = mpu6500ReadRegister(firstRegisterAcce, 6, data);

	if (ack) {
		USART1_Write("ACCNOT\n", 7);
		//return 2;
	}else{

		acce.x = (int16_t) ((data[0] << 8) | data[1]);
		acce.y = (int16_t) ((data[2] << 8) | data[3]);
		acce.z = (int16_t) ((data[4] << 8) | data[5]);
	}

	/*
	 offset = 6;
	 int16_t temp_tmp = (int16_t) ((data[0 + offset] << 8) | data[1 + offset]);
	 temp = temp_tmp;
	 */
	const uint8_t firstRegisterGyro = MPU_RA_GYRO_XOUT_H; //0x3B to 0x40 is accel, 41 and 42 temp, 43 to 48 gyro

	ack = mpu6500ReadRegister(firstRegisterGyro, 6, data);

	if (ack) {
		USART1_Write("GIRNOT\n", 7);
		//return ack;
	}else{
		gyro.x = (int16_t) ((data[0] << 8) | data[1]);
		gyro.y = (int16_t) ((data[2] << 8) | data[3]);
		gyro.z = (int16_t) ((data[4] << 8) | data[5]);
	}

	//read status to reenable the interrupt
	ack = mpu6500ReadRegister(MPU_RA_INT_STATUS, 1, data);

	data_is_ready = 0;

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
